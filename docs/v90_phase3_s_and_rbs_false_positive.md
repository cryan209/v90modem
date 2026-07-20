# V.90 Phase 3: why the analogue S never arrives

Investigation notes, 2026-07-20. Covers the long-standing `NO S RECEIVED` failure
against the softmodem interop rig, the spec reading that reframes it, and the
resampler defect that turned out to be the actual blocker.

Status: root cause identified and a fix implemented, **not yet confirmed by a
clean live run**. Read the "What is not yet proven" section before relying on it.

## 1. The S signals are Phase 3, not Phase 4

Worth stating first because it misdirected earlier work. Per Figure 7 and §9.4.2,
the analogue side of Phase 4 is:

```
CPt  CPt  SCR  CP  CP  CP'  CP'  E  B1  DATA
```

There is **no S anywhere in Phase 4**. Both S signals live in Phase 3, and each
gates a transition on our side:

| Spec | Analogue modem sends | Digital modem must |
|---|---|---|
| §9.3.2.7 / §9.3.1.5 | S, after receiving Jd | stop repeating Jd, send J'd |
| §9.3.2.8 | S̄ for 16T | — this is the *first* S→S̄ transition |
| §9.3.2.10 / §9.3.1.6 | S for 128T then S̄ for 16T | finish the **current DIL segment**, enter Phase 4 |

`docs/v90_phase_mapping.md` states that the caller sends `S`/`!S`/`TRN`/`MP` in
Phase 4. That is wrong and should be corrected.

Note §9.3.1.6 says complete the current **segment**, not the current cycle.

## 2. Timing: our Jd window was shorter than the spec gives the peer

- §9.3.2.7 lets the analogue modem wait **up to 5000 ms** from the start of its
  post-Ja silence before it begins transmitting S.
- §9.3.1.5 gives the digital modem **5100 ms + rtd, measured from the start of
  TRN1d**, before it must retrain.

`v90_jd_resync_symbols()` defaulted to 12000 symbols (1.5 s) and
`v90_jd_autoterminate_symbols()` to 19296 (2.4 s). Both expired roughly 3 s before
a *fully compliant* peer's own deadline, so we tore Phase 3 down or pushed on to
J'd/DIL while the peer was still legitimately silent and counting.

Fixed by `v90_jd_s_wait_symbols()` = `5100*8 - V90_TRN1D_LEN + rtd`, with the rtd
allowance under `ME_V90_JD_RTD_SYMBOLS` (default 4000 symbols = 500 ms; we do not
measure rtd anywhere yet). Used as the default for both knobs, and the
auto-terminate check now runs **before** the resync check so the DIL interop
fallback wins when both expire on the same symbol.

This was necessary but not sufficient — see §4.

## 3. Silence during the Jd wait carries no information

§9.3.2.4 requires the analogue modem to terminate Ja and **transmit silence**, and
§9.3.2.9 lets it stay silent through all of DIL. So silence in that window is the
normal case, not a diagnostic.

Two consequences:

- The existing `PEER_RETRAIN` silence detector in `v34rx.c` is gated
  `stage >= V34_RX_STAGE_PHASE3_TRAINING`, and the stage enum places
  `PHASE3_WAIT_S` *below* `PHASE3_TRAINING`. The Jd/DIL wait is therefore already
  excluded. This was checked and is **not** a bug — do not re-chase it.
- The real signal is **energy**. Until §9.3.2.7 starts S, the peer must not be
  transmitting at all, so sustained energy that never resolves into S means it has
  abandoned V.90.

Implemented as `phase3_expect_silence` plus
`v34_v90_set_phase3_expect_silence()`, set from `modem_engine.c` only while
`tx_phase == V90_TX_JD` (DIL is excluded because §9.3.2.9 permits SCR). 200 ms of
energy with no S raises `V34_EVENT_PEER_RETRAIN`.

Verified live, firing twice in one call:

```
Rx - Phase 3: 200 ms of energy during the Jd wait with no S; far end has left V.90
[ME] V.90: peer retrain detected; following it back to Phase 2 (accepted=1)
```

`accepted=1`, where the old silence-based path was giving `accepted=0`. This also
repairs the pollution risk that the longer Jd window would otherwise introduce.

## 4. The actual blocker: the peer abandons V.90 before it would ever send S

With verbose logging on the softmodem (`-d9`), the sequence is unambiguous:

```
Ja Flag set to 1
V90Demodulator: Error Energy = +3516 → +2432 → +1051 → +446     (converging on our Sd)
V90AutoDigitalImpDetector:  initail var (Trn1):  1020376 1365889 502571 475060 383165 1072410
V90AutoDigitalImpDetector:  initail AltRbsVarThresh (Trn1) = 680398
V90AutoDigitalImpDetector: alternate rbs false detection on phase 1 !!!
V90AutoDigitalImpDetector: alternate rbs false detection on phase 5 !!!
V90AutoDigitalImpDetector first update : trn1Sigma = -322122547
V90AutoDigitalImpDetector second update : trn1Sigma = 429496730
VPcmFloModem (V90): drop to V34 requested !!
VPcmV34Main: Initiating retrain, requested DP is 34
V34HSHAKE: txstate JaTXMIT=>SILENCERETRAIN
```

The peer never leaves `JaTXMIT`. Its equaliser converges on our downstream signal
perfectly well, then its robbed-bit-signalling detector decides the line is
impaired and drops to V.34. **No amount of patience on our side can fix this.**

`trn1Sigma = 429496730` is exactly 2^32/10, i.e. fixed-point garbage.

## 5. It is not our signal

Measured, not assumed:

| Property | Measured | Note |
|---|---|---|
| Our TRN1d/Jd transmit | 44640 samples of exactly ±943 | U_INFO=78, the peer's own INFO1a choice |
| Per-phase (mod 6) variance spread, our TX | **0.013%** | flat in every 200 ms window (worst ratio 1.02) |
| Rig 8k→9.6k rate | 44640@8000 → 53568@9600 = exactly 6/5 | **0 ppm** error |
| Per-phase variance at the DSP input | ratio ≤1.16 | |
| Clock-recovery slips | RX path only | all `me_cr_get_adjustment` sites feed `me_rx_*`; downstream TX untouched |

So the peer's 3.57× spread and its reported `Timing Offset = +10972 ppm` are
generated inside the far-end DSP, not on the wire.

## 6. Mechanism: the resampler maps one phase too well

The rig converts 8 kHz to the softmodem's native 9.6 kHz with a windowed-sinc
polyphase interpolator whose cutoff sits at exactly the input Nyquist. That has a
consequence which is easy to miss: **phase 0's kernel collapses to a single tap.**

```
phase 0: nonzero taps=  1     <-- the codeword passes through bit-exactly
phase 1: nonzero taps=247
phase 2: nonzero taps=249
phase 3: nonzero taps=249
phase 4: nonzero taps=249
phase 5: nonzero taps=247
```

Distinguishable levels per output phase, measured against a real downstream
capture:

| phase | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| levels | **∞** | 5.6 | 3.2 | 2.0 | 1.6 | 1.5 |

One phase resolves every codeword perfectly; the rest resolve almost nothing.
Per-phase differences in surviving level count is *precisely* the definition of
robbed-bit signalling, so the peer's detector is behaving correctly — it is being
shown something that genuinely looks like an impaired line.

Two things that are easy to get wrong here:

- **This is not spectral.** A white-noise per-phase model predicts a variance
  ratio of 1.01, and the measured 9600-domain per-phase variance is ≤1.16. The
  detector is reacting to *level resolution*, not to power or frequency response.
- **Lowering the interpolator cutoff barely helps** — the level ratio only falls
  from ∞ to 9.0 at 3900 Hz and 3.8 at 3400 Hz, because the asymmetry is structural
  rather than a tuning error.

## 7. Fix: model the physical path instead of ideal reconstruction

The interpolator implements mathematically ideal reconstruction, which is
physically wrong for this link. The real path is a CO D/A emitting a 125 µs
staircase into a local loop into the far end's sound card. Modelling that —
**zero-order hold to 9.6 kHz, then one lowpass** — makes every output sample derive
from exactly one codeword through the same filter, so per-phase uniformity is
structural rather than tuned.

Same capture, ZOH + 97-tap 4400 Hz lowpass:

| phase | 0 | 1 | 2 | 3 | 4 | 5 | ratio |
|---|---|---|---|---|---|---|---|
| levels | 13.6 | 13.3 | 13.4 | 13.4 | 13.6 | 13.2 | **1.02** |

Implemented in `rig/d-modem/d-modem.c` behind three environment knobs:

| Knob | Default | Meaning |
|---|---|---|
| `DM_RESAMPLER` | `zoh` | `sinc` restores the old interpolator for A/B |
| `DM_LOOP_FC` | 4400 | lowpass cutoff (Hz) |
| `DM_LOOP_TAPS` | 97 | kernel length, odd |

Headroom is still derived from the L1 bound so it provably cannot clip, and it is
much less punishing than for the ringy interpolator:

| Kernel | L1 | Headroom | Codeword level vs old 0.25 | Uniformity ratio |
|---|---|---|---|---|
| sinc 257-tap (old) | 3.81 | 0.25 | 1.00× | ∞ |
| ZOH + 97-tap | 2.15 | 0.465 | **1.86×** | 1.02 |
| ZOH + 33-tap | 1.69 | 0.592 | **2.37×** | 1.16 |
| ZOH + 9-tap | 1.11 | 0.90 | **3.6×** | 1.58 |

Shorter kernels ring less, so the L1 bound allows far more headroom — and level
resolution is what the peer's detectors actually consume. Pure ZOH with no filter
is the limit and is known to fail: its `floor()` edge quantisation puts up to
104 µs of jitter on a 125 µs symbol, which is what broke the Sd detector when ZOH
was tried alone. Some smoothing is required; how little is an empirical question.

## 8. Live results so far

One call at ZOH + 97-tap + 4400 Hz:

- **Sd survives.** 4 kHz line at the far-end DSP measured **−25.58 dB** against an
  ideal of −24.7 dB. Earlier aggressive band-limiting killed it to −56 dB, so this
  approach does not reproduce that failure.
- **The RBS false-positive did not occur.** Zero `initail var`, zero
  `alternate rbs`, zero `drop to V34` in the whole call. The peer stayed in V.90
  (`requested DP is 90`, not 34).

## 9. What is not yet proven

Be careful here — the above is suggestive, not conclusive.

- The call that showed no RBS false-positive **failed earlier for an unrelated
  reason** (`Error Energy = -0.000` in WaitForSd, never locked Sd), so the
  impairment detector may simply not have been reached rather than having been
  satisfied.
- A follow-up run at 33 taps failed at the known garbage bulk-delay confound
  (`rtd = 13592`), which mis-places the peer's Sd search window. Also unrelated to
  the resampler.
- Three consecutive runs on the new path have now shown `Error Energy = -0.000`.
  A control run with `DM_RESAMPLER=sinc` is needed to separate "ZOH regressed Sd
  detection" from "the pre-existing rtd confound", and that has not been done.
- The "distinguishable levels" figures come from a deliberately crude metric — how
  well an output sample predicts its source codeword. The absolute numbers should
  not be read as a fidelity spec; only the *ratio* between phases is meaningful,
  and that ratio is what the detector keys on.
- The peer's specific 3.57× has not been reproduced from first principles. The
  mechanism is matched; the number is not.

## 10. Test-methodology gotcha that invalidated earlier negatives

The AT command bridge into the rig container is a `socat` listener in `fork` mode.
Fork children from previous sessions accumulate holding the modem tty, so new
dials **queue for 80–90 s** rather than failing outright. The tell is an empty
read-back log while commands still eventually get through.

Always verify before trusting any result:

```
(printf 'AT\r'; sleep 3) | nc -w 6 <rig-host> <at-port>     # expect OK
```

Recovering requires killing both the modem daemon and `socat` and restarting both,
which yields a fresh pty. Auto-answer needs two rings, so the server must stay up
well past the dial. Several earlier negative results were taken with this broken,
and should not be trusted.

Separately, the rig degrades after roughly 15 calls; restart it before believing a
negative.

## 11. Code changes

All in the working tree, uncommitted at time of writing.

| File | Change |
|---|---|
| `v90.c` | `v90_jd_s_wait_symbols()`; spec-derived Jd S-wait deadline for both expiry knobs; auto-terminate checked before resync |
| `spandsp-master/src/v34rx.c` | dedicated `phase3_s_detect_armed`; energy-based peer-retrain detector for the Jd wait; `v34_v90_set_phase3_expect_silence()` |
| `spandsp-master/src/spandsp/private/v34.h` | `phase3_s_detect_armed`, `phase3_expect_silence`, `phase3_energy_samples`, `phase3_energy_retrain_reported` |
| `spandsp-master/src/spandsp/v34.h` | declares `v34_v90_set_phase3_expect_silence()` |
| `modem_engine.c` | sets the expect-silence flag while `tx_phase == V90_TX_JD` |
| `rig/d-modem/d-modem.c` | ZOH + loop-lowpass path, `DM_RESAMPLER` / `DM_LOOP_FC` / `DM_LOOP_TAPS` |

### The S-detector arming hole

The Phase 3 S detector used to be gated on `phase3_j_trn16 >= 0`. That field is a
constellation hint which doubles as a "Ja already consumed" latch, and it is
cleared to `-1` on entry to `PHASE3_WAIT_S`. A call that reached Ja through the
energy-gap heuristic or through `v90_note_ja_confirmed_by_descriptor()` — neither
of which runs the canonical J matcher — with no TRN lock left the detector **off
for the entire call**.

Now a dedicated `phase3_s_detect_armed`, set unconditionally in
`v34_v90_arm_phase3_s_detector()`. This is the fourth instance of the same defect
in this code: sequencing progress kept in a shared slot that another state machine
clears. See also the TRN lock and the Tone A reversal counter.

The `v34tx.c` clears of `phase3_j_trn16` were checked and are already guarded for
V.90 mode — not a live path.

## 12. Next steps

1. Control run with `DM_RESAMPLER=sinc` on a freshly restarted rig, to establish
   whether Sd lock is affected by the ZOH change at all.
2. If Sd lock is unaffected, sweep toward the high-level end (33 and 9 taps at
   4400 Hz) — level resolution is what matters, and those give 2.37× and 3.6× the
   codeword level.
3. Get one clean run that actually reaches `V90AutoDigitalImpDetector` with the new
   path, and confirm the per-phase variances it reports have tightened.
4. Only then revisit the DIL-termination issue: `v90.c` `V90_RX_EVENT_S` currently
   refuses the peer's S during DIL until a full cycle completes. At N=144 that
   cycle is longer than the 5000 ms §9.3.2.10 allows the peer, so the genuine
   Phase-4 trigger is logged as "ignored early far-end S". §9.3.1.6 wants the
   current *segment* completed, not the cycle.
