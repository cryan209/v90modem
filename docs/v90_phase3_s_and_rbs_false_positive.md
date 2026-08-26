# V.90 Phase 3: why the analogue S never arrives

Investigation notes, 2026-07-20. Covers the long-standing `NO S RECEIVED` failure
against the softmodem interop rig, the spec reading that reframes it, and the
resampler defect that turned out to be the actual blocker.

Status: **the RBS false-positive is solved** -- `ME_V90_SD_DELAY_MS=1` removes it
reproducibly (section 6c). It was never a signal-quality problem: the transport is
clean to 1% (section 6b) and the mechanism first proposed in section 6 is disproved
(section 6a). The zero-order-hold change built on that old mechanism is a measured
regression (section 8); `DM_RESAMPLER=sinc` remains the working default.

**Still blocked**: with the abort gone, the peer stays in V.90 but does not lock Sd
(`Error Energy` ~0) and retrains, so the analogue S still never arrives.

> **Correction 2026-07-22 — read [section 16](#16-correction-the-live-blocker-was-a-v92-qts-timeout).**
> The apparent `trn1Sigma` saturation in sections 14–15 was a SmartLink debug
> format bug: a promoted floating-point value was passed to `%d`. Correcting the
> format string shows finite values. Kernel uprobes identify the actual event-31
> source as the V.92 QTs timeout. Disabling the unsupported V.92 advertisement
> carries the same modem into V.90 Phase 4; the current blocker is strict CP
> acquisition, not Phase 3 sigma.

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

## 6a. CORRECTION: the section 6 mechanism is disproved

Section 6 is kept because the observation is real and the measurements are sound,
but **the causal story is wrong** and should not be built on again.

The claim was: phase 0 of the interpolator is a single tap, so one output phase
recovers codewords perfectly while the others do not, and the peer's detector reads
that as per-phase robbed-bit impairment.

The flaw is a domain confusion. "Phase" in section 6 means the **9600-domain output
phase**, `(5k) mod 6`. The detector's phases are **DS0 phases**, `n mod 6` at
8 kHz. Those are not the same thing, and the mapping between them is benign:

- An output sample is an exact copy of its input codeword when `(5k) mod 6 == 0`,
  i.e. `k = 6j`, which corresponds to DS0 index `n = 5j`.
- So the perfectly-represented DS0 samples are exactly those with `n = 0 mod 5`.
- `gcd(5, 6) = 1`, so `n mod 5` and `n mod 6` are independent. Over any period of
  30 samples **every DS0 phase receives exactly the same share** (one in five) of
  perfectly-represented samples.

Verified by direct enumeration: across n = 0..299, each RBS phase gets 10 exact
samples out of 50. The interpolator's per-output-phase asymmetry therefore averages
out completely in the domain the detector actually measures, and cannot produce the
1.84x-3.57x per-DS0-phase spread observed in section 8.

Two consequences:

1. The "distinguishable levels per phase" table in section 6, and every level/
   uniformity number derived from it (including the tables in section 7), is
   measured in the **wrong domain**. Those figures do not describe what the
   detector sees.
2. The motivation for the zero-order-hold change evaporates. It was independently
   measured to be a regression, so the conclusion does not change, but it was
   pursued for a reason that does not hold.

**What we still know for certain:** our own transmit path is flat across DS0 phases
to 0.013% (section 5), and the peer reliably computes a 1.84x-3.57x spread and
rejects the line (section 8). Something between those two points introduces it.
The interpolator is no longer a supported explanation for what.

Anything measuring "per-phase" behaviour from here must group by **DS0 index mod
6**, after reconstructing the 8 kHz stream, not by 9600-domain output phase.

## 6b. The transport is clean: measured in the correct domain

Section 6a said any future per-phase analysis must group by DS0 index mod 6 after
reconstructing the 8 kHz stream. Done, on the one call for which both halves of the
evidence exist -- the 9600 Hz DSP-side tap and the peer's own `initail var (Trn1)`
array from that same call.

Method: band-limited interpolation of the captured 9600 Hz stream at DS0 positions
`n * 6/5`, over the constant-magnitude TRN1d/Jd region, grouped by `n mod 6`.

| source | per-DS0-phase variance | ratio |
|---|---|---|
| our transmitted signal | flat at 889249 | 1.000 |
| recovered from the DSP-side tap | 884822 888971 885499 888807 880329 888143 | **1.010** |
| what the peer computed | 1020376 1365889 502571 475060 383165 1072410 | **3.56** |

Two things to note. The recovered variances match our transmitted 889249 to within
0.5%, so the transport is not attenuating or colouring anything per phase. And the
result holds over both TRN1d proper (2040 symbols, ratio 1.010) and the longer
~830 ms window the peer appears to analyse (6640 symbols, ratio 1.004).

**Conclusion: the signal arriving at the far-end DSP carries no per-DS0-phase
structure. The 3.56x spread is manufactured inside the peer's own processing.**

That closes off the entire transport chain -- our encoder (section 5), the rig
resampler (section 6a and here), and the rate (0 ppm, section 5). None of them can
be the cause.

### Where that points

The peer's own numbers are the remaining suspect, and two are visibly wrong:

- `Timing Offset [ppm]` reported as +7505, then +10417, then +10972, on a transport
  measured at **0 ppm**.
- `rtd = 13592`, which recurs across runs and is a known-bad bulk-delay estimate.

A misaligned or drifting DS0 sample grid on the peer side would sample the
staircase at the wrong instants and produce exactly this: per-phase variance
structure conjured from a clean signal.

If the rtd estimate is the root, this may still be our problem, but by a completely
different route than the resampler -- prior notes attribute the bad bulk-delay to
our own stale Phase 3/4 audio landing over the peer's retrain and Phase 1. The
energy-based retrain detector in section 3 reduces that pollution, so it is worth
re-measuring the per-phase spread on calls where `rtd` comes out sane.

## 6c. SOLVED: a 1 ms pre-Sd delay removes the false-positive

Section 6b pointed at the peer's own timing rather than the signal. Tested directly
with the existing `ME_V90_SD_DELAY_MS` knob (V.90 9.3.1.3 permits the digital modem
to wait before starting Sd). One millisecond is 8 samples at 8 kHz.

| `ME_V90_SD_DELAY_MS` | peer's `initail var (Trn1)` | ratio | `AltRbsVarThresh` | outcome |
|---|---|---|---|---|
| 0 (default) | 819254 981294 1510889 1025434 1141075 1241807 | **3.56** | 1412991 | `alternate rbs false detection`, `drop to V34 requested`, `DP is 34` |
| **1** | 159232 161628 154588 160067 163429 149694 | **1.09** | 228212 | no false detection, **no drop**, `DP is 90` |

Both rows reproduce **bit-identically** across repeated calls, so this is
deterministic, not run-to-run variance -- which has been the main hazard in every
other measurement here.

With the delay applied the spread collapses to 1.09 and every value falls well
under the threshold, so the detector no longer fires and **the peer stops
abandoning V.90**. That removes the blocker described in section 4.

Note the absolute variances also drop about 6x. Only the ratio matters for the RBS
test, but the magnitude change is a reminder that this knob is shifting how the peer
acquires timing, not how much signal it receives.

The specific prediction that motivated the test -- that a shift of 8 DS0 intervals
would *rotate* the per-phase array by `8 mod 6 = 2` positions -- was **wrong**. The
array flattens instead. Eight samples is an integer number of DS0 intervals and
changes no sub-sample alignment; what it changes is when Sd arrives relative to the
peer's rtd-derived search window. So the sensitivity is in the peer's timing
*acquisition*, not in DS0 phase mapping.

### What this does not fix

The peer stays in V.90 but still reports `Error Energy` of ~0 through its Sd wait
and then `retrain requested` with `DP is 90`. Our side runs a full clean Phase 3 --
Ja descriptor parsed, Sd, S-bar, TRN1d, Jd -- and still times out with no S.

The prime suspect remains `rtd = 13592`, which recurs on every call. That is about
1.7 s of claimed round-trip delay, and V.90 uses rtd to place the Sd search window,
so a value that wrong plausibly prevents Sd lock regardless of any millisecond-scale
transmit delay.

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

## 8. Live A/B result: ZOH as implemented is a regression

Two calls run back to back under an identical protocol, each with the active path
proved by the `ACTIVE downstream path:` log line, both entering Phase 3 with the
same `rtd = 13592` and both given ~3 s there:

| | peer's Error Energy | impairment detector | outcome |
|---|---|---|---|
| `sinc` (control) | converges 3516 -> 446, **sees our Sd** | runs, false-positives | `drop to V34 requested`, `DP is 34` |
| `zoh`, 97 taps, 4400 Hz | **-0.000 throughout, sees nothing** | never runs | stays V.90 (`DP is 90`), but blind |

The control reproduces the original failure, twice:

```
initail var (Trn1):  819254 981294 1510889 1025434 1141075 1241807   (ratio 1.84)
alternate rbs false detection on phase 2 !!!
trn1Sigma = -1968526677
VPcmFloModem (V90): drop to V34 requested !!
```

**The RBS false-positive "disappearing" under ZOH is not a fix.** It disappears
because the peer never locks Sd, so the detector has nothing to analyse. Absence
of the complaint was mistaken for absence of the problem.

This is a *timing* failure, not an amplitude one: the 4 kHz Sd line measured
-25.58 dB at the far-end DSP against an ideal of -24.7, so the spectral content is
intact and the peer still cannot lock. The zero-order hold quantises each DS0 edge
to the 9.6 kHz grid -- up to 104 us of error on a 125 us symbol -- and a lowpass
cannot recover edge positions the hold has already destroyed. The assumption that
adding the loop filter would fix the timing problem recorded against earlier ZOH
attempts was simply wrong.

So the two options as implemented are a straight trade:

- **sinc** -- correct symbol timing, non-uniform per-phase level resolution; peer
  locks Sd, then rejects the line as impaired.
- **ZOH + LPF** -- uniform per-phase level resolution, wrong edge timing; peer
  never locks Sd at all.

### Candidate that was tried and abandoned: hold on the LCM grid

8000 and 9600 have LCM 48000 (= 6 x 8000 = 5 x 9600), so holding to 48 kHz places
every DS0 edge exactly on a sample boundary -- a lossless hold rather than a 104 us
quantisation -- and decimating by 5 through one shared filter keeps every output
phase symmetric. Modelled offline before implementing.

It was **not** built, for two reasons:

1. On the same (9600-domain) metric used to justify the ZOH change it scored
   *worse*, not better: per-phase ratio 4.70 against the direct-9600 hold's 1.02.
   Proper anti-alias filtering before a 5:1 decimation has to be narrow at 48 kHz,
   which smooths across most of a DS0 interval, so output samples near an interval
   edge blend two codewords.
2. Chasing that number is what exposed section 6a. The metric is in the wrong
   domain, so neither the 4.70 nor the 1.02 describes what the detector sees, and
   there is no longer a reason to believe per-9600-phase uniformity is the property
   worth optimising.

## 9. What is still not proven

- The "distinguishable levels" figures come from a deliberately crude metric -- how
  well an output sample predicts its source codeword. The absolute numbers should
  not be read as a fidelity spec; only the *ratio* between phases is meaningful,
  and that ratio is what the detector keys on.
- The peer's specific per-phase variance ratio has not been reproduced from first
  principles. The mechanism is matched; the number is not. The two control runs
  reported different ratios (3.57 and 1.84) and flagged different phases (1 and 5,
  then 2), so the detector's exact trigger is not fully characterised.
- `rtd = 13592` recurs across runs and is a known bad bulk-delay estimate. It did
  **not** prevent the impairment detector from running in the control, so it is not
  the blocker here, but it remains unexplained.
- Whether the LCM-grid variant above actually works is untested.

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

1. **Decide whether `ME_V90_SD_DELAY_MS=1` should become the default.** It removes
   the V.90 abort reproducibly. Confirm against a second peer implementation before
   changing the default, since the value may be tuned to this one.
2. **Attack `rtd = 13592`.** It recurs on every call and is the prime suspect for
   the remaining Sd-lock failure. Prior notes attribute the bad bulk-delay estimate
   to our own stale Phase 3/4 audio landing over the peer's retrain and Phase 1;
   the section 3 energy detector should reduce that, so check whether rtd improves
   on calls where it fires.
3. Sweep `ME_V90_SD_DELAY_MS` further (2-10 ms) to see whether any value also
   restores Sd lock, not just the variance ratio.
4. Keep `DM_RESAMPLER=sinc`. ZOH is measurably worse and its rationale is gone.
5. Only then revisit the DIL-termination issue: `v90.c` `V90_RX_EVENT_S` currently
   refuses the peer's S during DIL until a full cycle completes. At N=144 that
   cycle is longer than the 5000 ms 9.3.2.10 allows the peer, so the genuine
   Phase-4 trigger is logged as "ignored early far-end S". 9.3.1.6 wants the
   current *segment* completed, not the cycle.

## 13. Unrelated defect found in passing

`sip_v90_modem` segfaults on exit: `main` -> `pjmedia_aud_subsys_shutdown` ->
`pjmedia_aud_driver_deinit`, `EXC_BAD_ACCESS` at address 0x8. It happens after the
call has finished and does not affect results, and crash reports predate the changes
in this document, so it is pre-existing. Worth fixing separately.

## 14. Update 2026-07-22: the blocker is now `trn1Sigma` saturation

> **Historical hypothesis, disproved by section 16.** The integer values below
> were produced by undefined varargs formatting and are not sigma measurements.

Three live V.90 calls (HEAD `a624dfa`, `DM_RESAMPLER=sinc`, `AT+MS=90,0,300,56000`)
show the failure has moved one stage deeper into the peer's impairment detector.
Two things genuinely improved versus the state above:

1. **Best-ever downstream.** The peer's `Error Energy` converges to **~20–30**
   (section 8's best was 446; ZOH gave a blind `-0.000`). Its equaliser fully
   locks our Sd/TRN1d now. Our Phase 3 otherwise runs clean every call: `selected=90`,
   Ja DIL descriptor parsed (`N=144 LSP=120 LTP=120` via hypothesis 8), Sd/S̄d/TRN1d
   all sent, retrains followed.
2. **The RBS variance-*ratio* false-positive is gone.** `initail var (Trn1)` is now
   e.g. `338 656 429 448 393 485`, ratio ~1.9, absolute values ~350–800 (were ~1 M),
   all far under `AltRbsVarThresh = 4000`. **No `alternate rbs false detection`
   line appears at all.** This is what sections 6–6c were chasing.

But the peer still ends every call `drop to V34 requested / DP is 34 / Link Error`,
now triggered by a later stage of `V90AutoDigitalImpDetector`:

```
initail var (Trn1):        338  656  429  448  393  485    <- passes the ratio test
trn1 second update:       3772 3772 3772 3772 3772 3772    <- perfectly uniform
trn1 Alt second update:   3772 3772 3772 3772 3772 3772
first update  : trn1Sigma = 1073741824        (= 2^30)
second update : trn1Sigma = -1073741824 / -1610612736 / -2147483648  (±2^30..−2^31)
VPcmFloModem (V90): drop to V34 requested !!
```

The proximate trigger is the saturated `trn1Sigma`, not the variance ratio. The
`trn1 second update` array is **byte-identical across all six DS0 phases**, so
whatever `trn1Sigma` normalises by the inter-phase spread divides by ~0 and pegs to
a fixed-point rail. Reproduced on all three calls.

**Root cause is the standing "our downstream is too perfect" theme, now biting a
different detector.** Once the peer's equaliser fully converges (Error Energy ~20),
the residual per-phase quantity it measures is identical across all six phases,
because our TRN1d is a spectrally/temporally perfect constant-`U_INFO` stream.
Improving the downstream is what *exposed* this: tighter lock ⇒ more uniform
residual ⇒ sigma overflow.

**`ME_V90_SD_DELAY_MS=1` no longer prevents the drop** (contrast section 6c, where a
clean ratio meant no drop). A call with the delay still saturated `trn1Sigma`
(`-2147483648`) and dropped at the first Ja cycle. Its only visible effect was a
**false** `[V90] far-end S detected → J'd → sending DIL` on our side: our Phase-3 S
detector triggered on the peer's *post-drop V.34 retrain* SSEG, not a genuine V.90 S
(the peer log shows `JaTXMIT ⇒ SILENCERETRAIN → Link Error`, never a `sending S`).
So the delay marches us uselessly into DIL/Phase 4 over a peer that has already
abandoned V.90 — a secondary S-detector false-positive worth gating (do not accept S
while the peer is mid-V.34-retrain).

Captures: `artifacts/v90-hardware/20260722T015126Z-dmodem_v90/` (baseline, no delay),
`.../20260722T020129Z-dmodem_v90_sddelay1/` (the false-S delay call). Grep the
per-call `/tmp/slm-current.log` (not the stale `/tmp/slm.log`) for
`trn1 second update|trn1Sigma|alternate rbs|drop to V34`; identify a call by its
monotonic `<NNN.xxx>` timestamp.

## 15. Per-phase TRN1d shaping: implemented and ruled out

The obvious hypothesis for section 14 is that a real CO line never presents a
perfectly uniform DS0 stream — robbed-bit signalling perturbs specific frames of the
six-frame superframe, which is exactly what this detector exists to measure — so
giving our TRN1d a small per-DS0-phase variation should give `trn1Sigma` a finite,
non-uniform array to work on. That was implemented as an env-gated, default-off lever
(`ME_V90_TRN1D_SHAPE`, helpers `v90_trn1d_shape_amplitude` / `v90_trn1d_phase_dither`
in `v90.c`) and tested live in two forms. **Both were ruled out.**

### 15.1 Per-phase level offset — ineffective (mechanistic)

A static, zero-sum six-phase Ucode offset (e.g. `{+3,−2,+1,+2,−3,−1}` around
`U_INFO=78`). Live at amp=3 the peer's `trn1 second update` stayed uniform
(`3775 ×6`), `trn1Sigma` still saturated, still dropped. Expected in hindsight: a
per-phase *mean* offset is **variance-invariant**, and as a smooth per-phase gain it
is exactly what the peer's equaliser normalises away before the detector measures.

### 15.2 Per-phase variance dither — reaches the wire, still ineffective (decisive)

Symmetric random magnitude dither with a **distinct per-phase radius** (pattern
`{3,2,1,2,3,1}` scaled by the amplitude), so each phase carries a different
within-phase variance — the one quantity a linear equaliser cannot predict away.

At amp=9 this was **verified present on the wire**: our `live-tx.g711` TRN1d window
carried a real 8.3 % per-phase magnitude spread,

```
per-phase mean |v|:  3921 3921 3628 3921 3933 3617   (μ-law turns the symmetric
                                                       Ucode dither into a per-phase
                                                       level difference too)
```

and yet the peer reported the **identical** uniform `trn1 second update` `3775 ×6`,
`trn1Sigma` still saturated (`-536870912 / 1073741824`), still `drop to V34`. Its
*pre*-equaliser `initail var` (`315 610 322 385 390 460`) was also unchanged from the
unshaped baseline (~1.9× spread, i.e. channel noise) — so even the first-stage
measurement does not reflect our deliberate per-phase content.

Capture with the TX-tap proof: `artifacts/v90-hardware/20260722T022359Z-dmodem_v90_dither9/`
(offset run: `.../20260722T021305Z-dmodem_v90_trn1dshape3/`).

### 15.3 Conclusion

Neither a per-phase mean nor a per-phase variance in our TRN1d moves the peer's
measurement; the `trn1Sigma` saturation reproduces identically across unshaped,
offset, and large-dither TX. The SmartLink/d-modem `V90AutoDigitalImpDetector` either
normalises per-phase level in its equaliser before measuring, or its
`second update`/`trn1Sigma` path is input-independent. **Either way, downstream
TRN1d amplitude shaping cannot reach it — this is a peer-side (blob) defect, not a
signal-content problem we can fix from the digital side.**

The shaping code is kept default-off as a tunable diagnostic (with an honest comment
at `v90_trn1d_shape_amplitude`); it is **not** a fix. `make` and the loopback test
pass both default-off and with shaping forced on (our own decoder tolerates a shaped
TRN1d).

### 15.4 The real next step: disassemble `V90AutoDigitalImpDetector`

Learn what `trn1Sigma` actually computes and whether it is input-independent, using
the objdump-the-linked-`slmodemd` method already used for the V.92 PCM-upstream work
(`docs/v92_pcm_upstream_findings.md`; copy `/src/slmodemd/slmodemd`, it is linked and
not stripped, strings resolve at file offset = vaddr). Two outcomes to distinguish:

1. **A peer-side data/logic fix** — as with `SLM_V92_PCM_UPSTREAM` and the resampler
   headroom, the fix may be a small `slmodemd` patch (e.g. a guard on the
   divide-by-spread, or a threshold), which we already carry patches for on the rig.
2. **A structural divide-by-zero on a clean line** — if `trn1Sigma` is
   `something / (per-phase spread)` and a no-impairment line legitimately yields zero
   spread, this build simply cannot complete a clean digital V.90 connection without
   that guard, and no conformant digital modem could satisfy it either.

Until that is known, do **not** re-attempt TRN1d amplitude shaping — sections 15.1
and 15.2 already cover per-phase mean and per-phase variance, on the wire, against
the actual peer.

## 16. Correction: the live blocker was a V.92 QTs timeout

Disassembly of the linked SmartLink binary (SHA-256
`ba330b87359a0f50c10b5e52ca777865c59a36c34583d15ede19c5040f83c79e`)
disproved the saturation interpretation. `studyUrefHandler()` stores sigma as a
`float`, promotes it to a `double` for the variadic debug call, but the two format
strings use `%d`. The rail-looking integers in sections 14–15 are therefore words
from the IEEE-754 argument, interpreted with the wrong type.

A behaviour-preserving diagnostic changed only those two format characters from
`d` to `f`. On a clean call it reported:

```
first update  : trn1Sigma = 795.481628
second update : trn1Sigma = 268.031006
```

These values are finite. The later V.34 drop was not evidence of overflow.

Kernel uprobes were then placed on every instruction known to assign demodulator
event 31. The only live hit was file offset `0x43f5a`, in
`V90Phase3Demodulator::getV92Decision()`. That instruction is the terminal branch
after the proprietary `faking ANSpcm energy drop detected (QTs timeout)` path.
The ordinary V.90 timeout at `0x4606f` and the six event-31 assignments in
`V90Demodulator::progress()` did not fire.

The server was advertising V.92 twice even though this call was later demoted to
the V.90 long-startup path:

- the V.92 octet in V.8 was enabled by default;
- INFO0d bit 27 advertised V.92 capability.

The SmartLink peer consequently stayed in `getV92Decision()` and waited for QTs,
while the server sent its V.90 Phase 3 waveform. Setting `ME_V92_ENABLE=0` removed
both advertisements. The resulting hardware capture,
`artifacts/v90-hardware/20260722T045937Z-smartlink_v90_no_v92_advertise/`, showed:

- INFO0d V.92 capability bit 27 clear;
- peer INFO0a V.92 capability bit 26 clear;
- CRC-valid INFO1a and J accepted;
- Phase 3 completed without the QTs event-31 drop;
- V.90 Phase 4 entered and Ri transmitted;
- 1,430 upstream control bits received, with 11 CP-like candidates rejected and
  no CRC-valid CP before disconnect.

This matches the V.90 Phase 4 ordering in Figure 7/section 9.4.2: after Ri the
analogue modem must send CP, then the exchange proceeds through CP-prime, MP-prime,
E, B1d and DATA. We are now at that CP boundary. V.92 is therefore opt-in
(`ME_V92_ENABLE=1`) until its QTs/ANSpcm startup path is interoperable end to end;
plain V.90 is the default.

## 17. Tone A gated (2026-08-12), and the DIL blocker is not the S detector

Two live Courier calls this session against the Cisco VG224 path (ext 8405,
`artifacts/v90-hardware/20260812T073702Z-*` and `20260812T075230Z-*`).

**The third false-S source named earlier is now gated.** §9.5.2.1 Tone A is a
real, loud, non-echo far-end signal, so the silence floor, the RX/TX ratio test
and the echo correlator all correctly pass it, and p3_demod's structural check
passes it too — a pure tone differentially demodulates to a constant dibit, and
a constant dibit stream *is* 6-symbol periodic. Only a narrowband test
separates it. `v90_s_tone_a_fraction_locked()` in modem_engine.c scores the
fraction of the same 100 ms window's energy in 2300–2500 Hz (Hann-windowed
Goertzel at exact DFT bins, so Parseval makes the ratio comparable to total
energy), and rejects an S at or above `ME_V90_S_TONE_A_MAX_FRACTION`
(default 0.35).

Measured on the 073702Z capture, µ-law, 100 ms windows:

| window | band 2300–2500 Hz |
|---|---|
| Courier Ja (9.5 / 10.5 / 11.5 s) | 0.100 / 0.036 / 0.103 |
| Tone A retrain (12.4 / 15.0 / 30.0 s) | 0.794 / 0.794 / 0.796 |

An order of magnitude, and the separation is structural rather than calibrated:
the real far-end S is V.34-modulated around the 1829/1920 Hz carriers and cannot
concentrate its energy in a 200 Hz band at the top of the passband. The same
test now also gates the WAIT_JA energy-return heuristic, which fires on exactly
the shape a retrain presents (peer stops, gap, loud steady tone).

**Both gates are offline-validated and live-unexercised.** In the 075230Z call
they logged nothing, because a pre-existing detector — "Tone A detected in stage
PHASE3_WAIT_S (80 ms)" — caught the retrain first and reported it per §9.5.1.2.
That call was much cleaner than 073702Z (`s_events=0 cp_bits=0 cp_rejected=0`
against `10 / 389 / 3`, and it followed the retrain into a legitimate second
Phase 2 instead of spending 32 s decoding a tone) but with the new gates silent
that difference is not attributable to them. Judge over several calls.

**The DIL blocker is upstream of all of this, and it is not a detector problem.**
DIL is skipped at `v90.c:3248` whenever `dil_requested` is false, and that flag
is set only by a CRC-valid Ja descriptor. Neither call parsed one — zero
"parsed Ja DIL descriptor" lines — so `dil=0` in both. In 073702Z the Courier's
real Ja ran 9.0–12.1 s while our TX sat at 0; p3_demod's J scan covered exactly
that window (scan t=0 is tap 8.84 s) and reported "no J found" eleven times,
after which the energy heuristic fired on the retrain tone at 12.20 s. So Sd
started against a modem that had already left Phase 3, and the Ja carrying the
descriptor was never decoded.

Offline, `./vpcm_decode --g711 --law ulaw --p3` on that 3.1 s Ja window picks
the right configuration on its own — 3200 baud low (1829 Hz), matching the
INFO1a we decoded live (U_INFO=74, upstream symbol rate code 4, downstream rate
code 6) — and still fails the same way: it classifies the whole window as TRN,
reports **dber 3684/19704 (19%)** and "Ja DIL search: no J segment". A 19%
differential bit-error rate on a transport with zero packet loss means the
demodulator is not locked, not that the descriptor is absent; the wide-transform
search does find 17-ones syncs at bits 48/4341/5040 and every parse off them is
garbage.

Next: fix Ja demodulation at 3200/1829 against this capture (it is a fixture now
— 3.1 s of foreign Ja, and the offline path reproduces the live failure exactly),
not the S detector. Until a descriptor parses, DIL cannot be transmitted at all.

## 18. Ja decode (2026-08-12): four hypotheses tested and killed

Six live Courier calls chasing "the descriptor does not decode". Nothing here
fixed it; all four candidate causes are now excluded, and two diagnostic traps
cost most of the session. Recorded so the next pass does not repeat them.

**Killed: "the demodulator is not locking (dber 19%)".** `vpcm_decode --p3` on
the July capture that *did* decode the descriptor twice reports **dber 21-44%**
and "Ja DIL search: no J segment" — worse than the 19% of the failing August
capture, which p3 renders as one clean 9852-symbol TRN segment. `--p3` is not
the live Ja path and its dber says nothing about whether the descriptor decodes.
Do not use it as the metric.

**Killed: carrier mismatch.** The RX carrier is `s->v90_info1d_high_carrier[]`,
i.e. whatever *we* announced in INFO1d (v34rx.c `process_rx_info1a()`), chosen
per call by the L2 line probe (v34tx.c `v34_l2_probe_result()`). It correlated
perfectly across four calls — both July calls probed high (1920 Hz) and parsed
the descriptor, both August calls probed low (1829 Hz) and did not — and a
forced-high live call (`20260812T080908Z-*carrier-high`) **still did not
parse**. The correlation was spurious. A head-to-head p3 scan (all 12
hypotheses, which needs a window under 12000 samples or the ranking silently
drops four candidates) also confirms each call's upstream really is at the
carrier we announced: July 1920 Hz wins 633 to 151, August 1829 Hz wins 2744 to
667. The Courier honours INFO1d's carrier bit.

**Killed: signal quality.** Measured on the August fixture: carrier offset
**-0.48 Hz**, symbol rate exactly **3200**, RTP 0 loss / 0 discard / 0 reorder.

**Killed: "the parser is fed nothing".** It looked conclusive — the parser's
only input is `v34_v90_copy_phase3_ja_bits()`, which reads
`phase3_ja_capture_hyp[]`, while the "Ja capture: emitted 17664 bits" span_log
comes from `phase3_ja_capture[]`, a different buffer filled from a different
stage. Instrumenting it showed `longest_hyp_len=0`. That was a sampling
artifact: the arrays are legitimately empty in PHASE3_TRAINING (stage 11) and
fill normally once the receiver reaches PHASE3_WAIT_S (stage 10), reaching
**15832 bits** by the end of a call. The parser gets its bits and still finds no
CRC-valid descriptor. A v34rx.c change to mirror the TRAINING-stage capture into
the hypothesis arrays was written, tested live (no effect — that stage's
contribution is wiped by the one-shot reset at the TRAINING→WAIT_S transition)
and reverted.

**Trap worth knowing: `ME_LOG` is gated on `me_verbose_enabled()`
(`VPCM_ME_VERBOSE`), and most visible `[ME]` lines are raw `fprintf`.** Adding
an ME_LOG diagnostic and seeing no output means nothing. Run hardware captures
with `VPCM_ME_VERBOSE=1` — the July runs had it on, which is also why their logs
carry `parsed Ja DIL descriptor` lines and August's do not. (The descriptor
genuinely did not parse in August: `v90_dil_capture_try_parse_at()` sets
`dil_requested` before it logs, and `dil=0` with no DIL on the wire is
independent of logging.)

The permanent diagnostic left behind is `[ME] V.90 Ja search input:` — the
parser's own input length, hypothesis and RX stage, verbose-gated.

**Where it actually stands.** The parser receives ~16k Ja-stage bits per call
and no window of them passes the 17-ones preamble plus descriptor CRC. The open
question is no longer "why is the demodulator broken" but **whether those bits
are Ja at all**: p3 classifies the whole 3.1 s as TRN on both the working and
failing captures, and the Courier retrains ~3.3 s into its Phase 3, at the point
where it expects Sd. Next pass should compare the July (parsed) and August (not
parsed) *hypothesis-array bit streams* directly — dump both with
`ME_V90_JA_DUMP_PREFIX` (note it triggers off hypothesis 0's length against
`ME_V90_JA_DUMP_MIN_BITS`, default 32000, which no August call reached) — rather
than re-deriving the modulation offline.

## 19. The Ja bit-stream diff (2026-08-12): the demod is fine, the margin is not

Replayed both captures through the *same* receiver offline, using a
parametrised copy of `tmp/v34_phase3_replay.c` (kept as
`artifacts/courier-ja-3200low/streams/ja_replay.c`): it forces Phase 3 RX, feeds
linear PCM, then dumps all 24 hypothesis streams and tries the descriptor parse
at every 17-ones+0 preamble. **Note the two captures use different RTP codecs —
July negotiated PCMA, August PCMU — while both log `law=ULAW`, which is the
V.90 PCM law, a different field. Decode each tap with its own law.**

The replay is faithful: July reproduces its live result exactly
(`DESCRIPTOR hyp=8 start=14312 N=197 LSP=66 LTP=66`, matching the live
`parsed Ja DIL descriptor` line).

| | July (dilgate-3) | August (jastage) |
|---|---|---|
| hyp 8 stream length | 16922 bits | 17198 bits |
| 17-ones+0 preambles | 2, at 14312 / 16372 | 2, at 14282 / 16342 |
| CRC-valid descriptors | 1, at 14312 | 1, at 14282 |
| descriptor | N=197 LSP=66 LTP=66 | N=197 LSP=66 LTP=66 |

**The two 1702-bit descriptor payloads are bit-for-bit identical — 0 differing
bits.** The Ja frame period is 2060 bits; the second preamble in each capture is
truncated by the end of the peer's transmission, so there is exactly **one**
complete decodable copy per call.

**The first preamble lands 14282 bits ≈ 2.23 s into the peer's ~2.9 s Phase 3**,
i.e. the descriptor occupies only the last ~0.6 s: the Courier sends ~2.2 s of
training first, which is why p3 classifies the whole window as TRN and finds
"no J segment" on *both* captures. Nothing before bit 14282 is decodable.

**So the demodulator is not broken, and section 18's framing was wrong.** The
descriptor decodes whenever the capture runs long enough, and the margin is one
frame:

- July captured 16922 bits — 908 past the 15984 the descriptor needs. Parsed.
- August `jastage` reached 15832 bits live... and **also parsed**
  (`parsed Ja DIL descriptor (N=197 LSP=66 LTP=66)`, hypothesis 8, +11688 ms),
  then ran Sd → S̄d → TRN1d (2496 symbols) → Jd, where **the Courier retrained
  during Jd** (`Peer retrained during tx_phase=6`) — i.e. that run reached the
  original §9.3.2.7 Jd blocker, not a Ja one.
- The earlier August runs fell short: 073702 never acquired PP at all
  (`PP start detected` = 0, and the replay reproduces that at every start offset
  and both carriers), and others cut the peer's Ja off early.

**Why a run fails is therefore a race, not a decode defect: the peer stops Ja as
soon as it sees our Sd (§9.3.2.4), and the descriptor is in the last 20% of Ja.**
Any Ja detection that fires before the descriptor parses truncates the very
frame we need. `v90_note_ja_confirmed_by_descriptor()` already encodes the right
rule ("a CRC-valid DIL descriptor is proof the peer is transmitting Ja right
now -- strictly better evidence than the energy-gap heuristic"), but the three
heuristic paths — p3_demod's J scan, the WAIT_JA energy-return gate, and
v34rx's J event — can each fire first and start Sd early.

Next change to try: gate Sd on the parsed descriptor, letting the heuristics only
arm a bounded fallback, so we never cut off the frame that carries DIL. Fixtures:
`artifacts/courier-ja-3200low/streams/{july-dilgate3,aug-jastage}-hyp8.bits`.

## 20. Sd is now gated on the descriptor (2026-08-12)

Section 19 left the mechanism as a hypothesis. It is now measured. Counting runs
of consecutive ones in the August hyp-8 stream, the whole 17198-bit capture
contains **exactly two** runs of >=10 ones — both exactly 17 long, at 14282 and
16342. If the Courier were transmitting the descriptor throughout Ja and our
receiver were merely converging, degraded repeats would still show 10-16 length
near-preambles at 2060-bit spacing; there are none. **The Courier transmits
~2.2 s of training and only then starts Ja frames** — which is also why p3
classifies the whole window as TRN on the working capture. The descriptor
arrives late because it *is* late, not because we decode it late.

That makes an early Sd actively harmful: §9.3.2.4 has the analogue modem stop on
Sd→S̄d, so a detector that fires during the peer's training can stop it before it
ever transmits a descriptor frame.

`v90_ja_heuristic_allowed()` in modem_engine.c now gates all three heuristic Ja
detectors — p3_demod's J scan, the WAIT_JA energy-return gate, and v34rx's J
event. They still run, and Ja decoding still runs on every one of them; what
they no longer do is fire `V90_RX_EVENT_J` and start Sd. Only a CRC-valid
descriptor does that, via the `v90_note_ja_confirmed_by_descriptor()` path that
already existed. Exceptions: a peer that declined V.90 (Table 10 downstream code
0-5) is doing plain V.34, whose J carries no descriptor, so the heuristics stay
live there. `ME_V90_JA_HEURISTICS=1` restores the old behaviour;
`ME_V90_JA_HEURISTIC_FALLBACK_MS>0` re-enables them that long after the first
suppressed attempt. Default 0 — never.

Measured over three live calls (`*desconly-1..3`), against four earlier ones:

| | heuristics on | descriptor-gated |
|---|---|---|
| descriptor parsed | 1 of 4 | 1 of 3 |
| reached Jd | 1 of 4 | 1 of 3 |
| phantom Phase 3/4 decoded from Tone A | 1 of 4 (32 s, cp_rejected=3) | 0 of 3 |

The hit rate is unchanged on this sample — too small to claim an improvement —
but the failure mode is cleaner: no run starts Sd during the peer's training,
and no run spends the call decoding a retrain tone. Runs that fail now simply
never transmit, and the peer's retrain gives another attempt inside the same
call.

**The blocker after this is the original one, unchanged:** `desconly-3` parsed
the descriptor at +12030 ms, ran Sd (64 reps) → S̄d → TRN1d (2496 symbols) → Jd,
and the Courier retrained during Jd (`Peer retrained during tx_phase=6`), same as
`jastage`. We now reach that point deliberately rather than by luck, which makes
it testable: TRN1d is still 2496 symbols / 312 ms here, against the 30000T /
3750 ms a working digital modem sends (`docs/eicon_downstream_comparison.md`),
and that remains the leading candidate for why the Courier never decodes our Jd.
Testing it needs the `ME_V90_TRN1D_SYMBOLS` clamp at `v90.c:121` raised past
16000.

## 21. TRN1d = 30000T tested live (2026-08-12): disproven, and harmful

The `ME_V90_TRN1D_SYMBOLS` ceiling was already 32000 (a previous session raised
it from 16000; CLAUDE.md's "clamp <=16000 blocks testing" line was stale). So
this was purely a live test: three descriptor-gated calls at
`ME_V90_TRN1D_SYMBOLS=30000`, one of which parsed the descriptor and reached
`TRN1d complete (30000 symbols)`.

**It does not fix the Courier's Jd, and it makes the window worse.** Both a
2496T and a 30000T run were measured from the same reference — Sd start, and the
peer's Tone A onset minus our detector's 80 ms:

| | TRN1d 2496T | TRN1d 30000T |
|---|---|---|
| Sd starts | 12030 ms | 11927 ms |
| Jd starts | 12396 ms | 15731 ms |
| peer Tone A, after end of Ja | 5076 ms | 4241 ms |
| **Jd airtime before it gave up** | **4764 ms** | **491 ms** |

The peer's retrain is anchored to §9.3.2.7's "within 4500 ms from the end of Ja",
not to anything about Jd: it fires at essentially the same point relative to end
of Ja in both runs. TRN1d length only decides how much of that fixed window is
left for Jd — 30000T spends 3750 ms of the 4500 ms budget and leaves Jd 491 ms.

So the hypothesis in `docs/eicon_downstream_comparison.md` — that our 312 ms
TRN1d is why the Courier never decodes our Jd — is **disproven for this peer**.
At the default length the Courier receives **4.7 seconds** of Jd and retrains
anyway. Whatever it objects to is in the Jd content or its framing, not in how
long it was trained beforehand. Do not adopt 30000T; the Eicon card's use of it
says something about the Eicon's peer, not about ours.

Next candidates, in order: Jd frame content and CRC against Table 13 (the
`goal11-jdp84` / `jd26k` / `jd22680` / `jdfix` runs churned here before without
resolving it), and §8.4.2's requirement that Jd's differential encoder be
initialised with the final symbol of the transmitted TRN1d — which is exactly
the kind of seam a TRN1d length change would have perturbed and did not.

## 22. §8.4.2 Jd seeding verified correct (2026-08-12)

Spec text, from `T-REC-V.90-199809`:

> §8.4.2: "The bits are scrambled and differentially encoded and then
> transmitted as the sign of the PCM codeword whose Ucode is UINFO... **The
> differential encoder shall be initialized with the final symbol of the
> transmitted TRN1d.**"
> §8.4.5: "Signal TRN1d is a sequence of the PCM codeword whose Ucode is UINFO
> with signs generated by applying binary ones to the input of the scrambler
> described in 5.3... **The scrambler is initialized to zero prior to the
> transmission of TRN1d.** TRN1d shall be an integer multiple of 6 symbols long."
> §8.4: "The digital modem shall use the polynomial, GPC... when generating
> signals Jd, Jd′ and TRN1d."

All four hold in `v90.c`: the scrambler is zeroed at the S̄d→TRN1d transition
(`v90_scrambler_init`, v90.c:3114); TRN1d puts the scrambler output straight on
the sign with no differential encoding; `s->diff_enc = sign` on the symbol where
`sample_count` reaches `v90_trn1d_len()`, i.e. the final TRN1d symbol
(v90.c:3151); and the length is forced to a multiple of 6. The scrambler
deliberately continues into Jd, which matches §8.4 naming one polynomial for all
three signals and §8.4.5 stating the only initialisation point.

**Verified empirically, not just by reading.** `tools/v90_jd_decode.py` is an
independent implementation of the same rule (`prev = s[jd - 1]` as the §8.4.2
seed, GPC descramble). Run against our own transmitted downstream, cut from
`*desconly-3/live-tx.g711`:

```
TRN1d: 2496T (312.0 ms), U_INFO=74      <- descrambles to all ones
Jd:    39152T (4894.0 ms), 543.8 frames
72-bit periodicity: 100.0%
CRC 0x76ee VALID    sync OK   start bits OK   fill 0000 OK
```

`vpcm_decode --v90` agrees independently: `Sd W_UCODE=90 (U_INFO=74) 64 reps`,
`S̄d 8 reps`, `TRN1d 2496 symbols descrambled to ones`, `Jd+J'd ~543 frames`.

Every Table 13 field checks out: sync 17 ones, start bits at 17/34/51 all 0,
reserved 41:46 all 0, shaping lookahead 49:50 = 1 (legal range 1–3), fill 0000,
CRC valid. The only field worth a second look is the rate mask, which advertises
**every** rate from 28000 to 56000 — a capability claim we may not be able to
honour, though the peer selects via CP regardless, so this is a weak lead.

**So the second and third candidates from §21 are both closed**: the Jd
differential seeding is right, and the Jd framing and CRC are right. The Courier
is being sent a spec-conformant, CRC-valid Jd for 4.7 seconds and retrains at
§9.3.2.7's deadline anyway. Nothing left in Jd's *content* explains it; the next
places to look are physical or alignment-level — downstream power (§8.4 says
Phase 3 digital-modem signals are not spectrally shaped, and Table 15's cap
applies in Phase 4, not here), and §8.4.4's "The first symbol of Sd is defined to
be transmitted in data frame interval 0. The digital modem shall keep data frame
alignment from this point on."

## 23. §8.4.4 Sd data frame alignment verified correct (2026-08-12)

> §8.4.4: "Sd consists of 64 repetitions of the sequence {+W, +0, +W, –W, –0, –W}
> where W is defined to be the PCM codeword whose Ucode is 16 + UINFO and 0 is
> the PCM codeword with Ucode 0... **The first symbol of Sd is defined to be
> transmitted in data frame interval 0. The digital modem shall keep data frame
> alignment from this point on.**"

Both halves check out on our own transmitted downstream
(`*desconly-3/live-tx.g711`, U_INFO = 74, so W = 90).

**The pattern that defines interval 0** — first four repetitions, decoded to
(sign, Ucode):

```
+90 +0 +90 -90 -0 -90
+90 +0 +90 -90 -0 -90
+90 +0 +90 -90 -0 -90
+90 +0 +90 -90 -0 -90
```

**Alignment kept from that point on** — every phase boundary is an exact
multiple of 6 symbols from Sd's first symbol:

| boundary | symbol | offset from Sd | data frames |
|---|---:|---:|---:|
| Sd | 480 | 0 | 0 |
| S̄d | 864 | 384 | 64 |
| TRN1d | 912 | 432 | 72 |
| Jd | 3408 | 2928 | 488 |

Segment lengths are exactly the spec's: Sd 384 (64×6), S̄d 48 (8×6), TRN1d 2496
(416×6). The Jd frame is 72 bits = 72 symbols = 12 data frames, so the framing
divides the alignment evenly.

The runtime hazard here is `me_cr_get_adjustment()` inserting or dropping a
single G.711 codeword to correct clock drift, which would shift the data frame
phase by ±1 and silently violate "shall keep data frame alignment from this
point on". It did not fire: zero slip events logged in either run, and
`v90_jd_decode.py` measures **72-bit periodicity of 100.0% across 543 Jd frames**,
which no inserted or deleted sample could survive.

Note in passing: we transmit −0 (µ-law 0x7F) in the negative zero slots where the
Eicon card transmits +0 (0xFF) in every zero slot (see CLAUDE.md). In µ-law both
decode to linear 0, so the analogue modem — which sees only the D/A voltage —
cannot distinguish them, and this is not a candidate here. **In A-law it would
be**: Ucode 0 decodes to ±8 there, a real 16-LSB difference, and the July runs
were PCMA.

**With §§8.4.2, 8.4.4, 8.4.5 and Table 13 all verified, nothing in the Phase 3
downstream we generate explains the Courier's behaviour.** The next place to look
is the transport, not the modem: this Courier is on a **Cisco VG224 port 2/5
(ext 8405)**, a different gateway from the 2911 FXS that the July `no vad` /
`no echo-cancel` / `fax protocol none` work was applied to
([[cisco-fxs-path-broken]]). CLAUDE.md's first constraint is that the RTP payload
*is* the DS0 stream the far end's D/A sees; any gain, echo cancellation or codec
hop in the VG224 makes a byte-exact downstream impossible and would present
exactly as "spec-perfect Jd, ignored". Verify the VG224's port configuration
before writing any more modem code.

## 24. VG224 gateway was not modem-safe; fixed (2026-08-12)

**The gateway changed between the July and August captures, which invalidates
every July-vs-August comparison above as a code comparison.** Confirmed from the
SIP To: headers: July's working runs (dilgate-2/-3, Ja decoded, DIL ran 2 full
cycles) were `To: "7802"` = **Cisco 2911 FXS 0/3/2**, the port configured in July
with `no vad` / `no echo-cancel` / `fax protocol none`. Every August run is
`To: "VG224 Port 2/5" <8405>` — a different box. Current wiring, probed by
dialling from each modem and reading the CDR caller ID: Courier #1
(`/dev/cu.usbserial-21210`) → VG224 2/5 (8405); Courier #2 (`FT4TQOFT`) → VG224
2/13 (8413); USR 56K (`21240`) → AudioCodes (6311). Nothing is on the 2911 FXS
any more.

**VG224 console is `/dev/cu.usbserial-630` @ 9600 8N1, already in enable mode.**
It needs several `\r` and a patient read — a short probe looks completely silent.
Telnet is open but refuses ("Password required, but none set").

Config as found. The VoIP side was fine — `modem passthrough nse codec g711ulaw`,
`no vad`, `codec g711ulaw`, `fax protocol pass-through g711ulaw` on
`dial-peer voice 8999 voip`. The per-port side was not:

| port | as found |
|---|---|
| 2/3 | `no echo-cancel enable`, `no non-linear` — deliberately modem-safe |
| **2/5** (Courier #1) | **EC enabled, non-linear enabled** (defaults, so absent from running-config; `show voice port summary` showed `EC y`) |
| **2/13** (Courier #2) | same |

plus `playout-delay minimum low` — an **adaptive** jitter buffer — on the VoIP
dial-peer. `modem passthrough nse` only engages when the far end signals NSE,
which we never do, so EC, NLP and adaptive playout were all live for the whole
call. NLP substitutes or mutes audio it judges to be echo, and adaptive playout
inserts and deletes samples to track drift; either destroys the sample continuity
a sign-only signal like Jd depends on (one substituted sample desynchronises the
§5.3 GPC descrambler for 23 symbols).

Applied with the user's approval, verified, **not written to startup-config** (a
VG224 reload reverts it):

```
voice-port 2/5     ! and 2/13
 no echo-cancel enable
 no non-linear
dial-peer voice 8999 voip
 playout-delay mode fixed
```

**Measured effect — the peer's behaviour changed substantially.** Its Phase 3
now runs **6.0 s** instead of ~2.9 s, i.e. it stays in Phase 3 more than twice as
long before giving up, and the offline replay of `noec-1` captures **26736 Ja
bits** (against the ~16000 the descriptor needs) with the descriptor present and
decodable: `DESCRIPTOR hyp=8 start=14420 N=197 LSP=66 LTP=66`.

**But the live path parsed 0 of 3**, against 1 of 3 before — a sample far too
small to call a regression, and the offline result says the signal got *better*,
not worse. So the failure has moved: we are now being handed 26k bits of Ja
containing a demonstrably decodable descriptor and are not extracting it. That is
a defect on our side, not the peer's or the line's, and it is the next thing to
chase. Note `PP start detected` fires twice in both a parsing and a non-parsing
run, so the one-shot reset of `phase3_ja_capture_hyp[]` at each
TRAINING→WAIT_S transition is a suspect but not established.

## 25. After the gateway fix: Ja is reliable, the blocker is unchanged

VG224 config saved to startup-config (`write memory`) so it survives a reload.

**`output attenuation -6` explained, partly.** The device reports the range as
`<-6 - 14>` dB of *attenuation*, so -6 is the least-attenuation (hottest) end,
not a neutral setting — neutral would be `output attenuation 0`. It is set on
every port. `show voice port 2/5` reports two different fields that do not agree
on their face — `Out Attenuation is Set to -6 dB` alongside `Analog interface
D-A gain offset = -3.0 dB` and `Output attenuation is set to 14 dB` — so the
configured value maps onto an internal scale rather than being a direct pad.
Left alone; worth an A/B against `output attenuation 0` if downstream level ever
becomes a suspect, since -6 is the hottest setting and clipping W = Ucode 90
would be nonlinear rather than a uniform gain.

**The "live miss" did not reproduce.** Two further runs after the gateway change
both parsed the descriptor (2/2, `N=197 LSP=66 LTP=66`), against 0/3 in the
batch taken immediately after the change. The `DISCARDING ... Ja bits on
re-entry to WAIT_S` diagnostic added to v34rx.c never fired — the second
`PP start detected` lands before any Ja is captured — so **the hypothesis that
the TRAINING→WAIT_S reset wipes the descriptor is disproven**, and the earlier
0/3 is best read as run-to-run variance.

**The blocker is exactly where it was, now with a clean transport and reliable
Ja.** Both runs: Ja parsed → Sd (64 reps) → S̄d → TRN1d (2496T) → Jd → the
Courier retrains. Measured from the tap during our Jd (11.6–16.4 s):

| window | far-end RMS | spectral flatness |
|---|---:|---:|
| peer transmitting Ja (9.0–11.4 s) | ~1530 | 0.19 |
| **during our Jd (11.6–16.4 s)** | **~54** | **0.26 (broadband noise)** |
| peer Tone A retrain (16.6 s+) | ~2920 | 0.000 (pure tone) |

So the Courier is genuinely silent — 29 dB below its own transmit level, with a
noise-shaped spectrum — for the whole of our Jd, then retrains at its §9.3.2.7
deadline. The S-event gates are behaving correctly here: every S in these runs
was rejected on the RX/TX ratio test at 0.03–0.10 against a 0.15 threshold, and
the tap confirms there was nothing to detect.

Everything we send is now verified conformant (§§8.4.2, 8.4.4, 8.4.5, Table 13,
§9.3.1.4), the transport is modem-safe, and Ja decodes. The Courier receives a
CRC-valid Jd for 4.7 s and does not answer it.

## 26. Why does the Courier ignore a valid Jd? Not the frame — proven

Compared our Jd frame against the only foreign V.90 digital modem we have: the
Eicon Diva Server downstream in `artifacts/eicon-digital-downstream/`, which a
USR Courier V.Everything answered with `CONNECT 32000/ARQ/V90/LAPM`. Same modem
model as the one refusing us.

Decoding both with `tools/v90_jd_decode.py`, the two Jd frames differed in
**exactly one field**:

| bits 49:50 (Table 13 shaping lookahead, 1–3) | |
|---|---|
| Eicon `11` → **3** | Courier answered CONNECT |
| ours `10` → **1** | Courier retrains |

Everything else was already bit-identical: 17-ones sync, both start bits, rate
masks `1111111111111111` / `111111000000`, both constellation bits 0 (4-point),
fill 0000, valid CRC. Ours was hardcoded at v90.c as "1 (minimum mandatory)".
There was a good reason to suspect it: §5.4.5's Sr on this project's own
analogue side is a precedent where a legal-looking shaping parameter (Sr = 1)
made every Phase 4 constellation unbuildable and the peer silently never
advanced.

Made it settable (`ME_V90_JD_SHAPING_LOOKAHEAD`, 1–3) and tested live at 3. The
transmitted frame is now

```
111111111111111110111111111111111101111110000000011001100110110011110000
CRC 0x66cf VALID
```

**byte-for-byte identical to the Eicon's frame, same CRC** — and the Courier
still ignores it and retrains (`ld3-3`: Ja parsed, Sd → S̄d → TRN1d → Jd, peer
retrain, no S accepted, DIL never sent).

**So the Jd frame content is definitively not the cause.** We now transmit the
exact 72 bits that this modem model accepted from a different digital modem, and
it is still refused. Combined with §§8.4.2/8.4.4/8.4.5 and §9.3.1.4 already
verified, and TRN1d length tested at both 2496T and 30000T, nothing about *what*
we send explains it.

What still differs from the Eicon's successful call, in order of interest:

1. **Level.** The Courier asked the Eicon for `U_INFO = 48` (W = Ucode 64,
   linear 1980) and asks us for `U_INFO = 74` (W = Ucode 90, linear 6652) —
   our downstream is ~10.5 dB hotter in linear terms. The VG224 also runs
   `output attenuation -6`, the *hottest* end of its −6..14 dB range. If W is
   clipping in the gateway D/A or the modem front end, that is nonlinear: it
   would corrupt TRN1d equaliser conditioning and Jd slicing while leaving the
   coarse Sd pattern detectable — which is exactly the observed split. Against
   this: the peer chose 74 itself, and a peer measuring a hot line should ask
   for less, not more. Testable directly by setting `output attenuation 0` on
   voice-port 2/5 and repeating.
2. **TRN1d duration**, 2496T vs the Eicon's 30005T. Tested at 30000T with
   lookahead 1 and it did not help (section 21); not yet retested with the two
   changes combined, and note the Eicon's peer answered after only 943T / 118 ms
   of Jd, so Jd airtime was never the constraint that section implied.
3. The transport itself — the Eicon capture did not traverse this VG224.

Default left at 1 pending evidence, but 3 is arguably the better default purely
because it makes our frame identical to the one known to interoperate with this
hardware.

## 27. Downstream level is not the cause either (2026-08-12)

Set `output attenuation 0` on VG224 voice-port 2/5 — note 0 is the IOS default,
so the setting disappears from running-config; the lab had explicitly configured
every port to `-6`, the hottest end of the −6..14 dB range. Left at 0 in the
running config; **startup-config still has −6** (it was written before this
test), so a VG224 reload restores the old value.

The change did reach the line. Measured from our own echo during Phase 3
transmission (EC is off, so our downstream leaks back through the hybrid), taken
over windows where we transmit and the peer is silent:

| run | echo RMS | our TX RMS | echo/TX |
|---|---:|---:|---:|
| `discard-2` (attenuation −6) | 69.7 | 3603 | −34.3 dB |
| `att0-2` (attenuation 0) | 43.7 | 3552 | −38.2 dB |
| `att0-4` (attenuation 0) | 44.2 | 3552 | −38.1 dB |

3.9 dB quieter, short of the full 6 dB because part of that residual is line
noise that does not scale with our output.

**Result: no change in behaviour.** Four calls, two of which reached Jd (Ja parse
is roughly one call in two): both ran Ja → Sd → S̄d → TRN1d → Jd → peer retrain,
no S accepted, DIL never sent — with shaping lookahead 3, i.e. a Jd frame
byte-identical to the Eicon's.

One further data point against the level theory: **the Courier still asks for
`U_INFO = 74`** after the 4 dB drop, the same value it asked for at −6. If its
choice of U_INFO were driven by the received level in the way the clipping
hypothesis assumed, it should have moved.

So downstream level joins the list of eliminated causes. As of now, against this
Courier: Ja decodes, Phase 3 is verified conformant in content
(§§8.4.2/8.4.4/8.4.5, Table 13), timing (§9.3.1.4, both 2496T and 30000T),
alignment and now level, the transport is modem-safe (EC/NLP off, fixed playout),
and the Jd frame is bit-identical to one this modem model answered with CONNECT.
It still receives 4.7 s of that Jd and does not answer.

**Proposed decisive next test: replay the Eicon downstream at the peer.** We hold
27 s of a downstream that a Courier V.Everything answered with
`CONNECT 32000/ARQ/V90/LAPM` (`artifacts/eicon-digital-downstream/`). Feeding it
to *this* Courier in place of our own Phase 3 splits the remaining space in one
call: if it answers the recording, the difference is still something in our
signal that all the above checks have missed; if it ignores the recording too,
the difference is this line, this gateway or this modem, and no amount of
transmitter work will fix it. It needs the engine to hand the transmitter over
to file playback at the Phase 2→3 seam, with the recording's own Sd as the
alignment reference.

## 28. The VG224 never enters modem passthrough (2026-08-12)

Queried the gateway *during* Phase 3 (VG224 console, `/dev/cu.usbserial-630`;
note the enable session times out, so a helper must re-`enable` each time).
Our call leg:

```
Tele 2/5 (301822) [2/5] tx:4550/4550/0ms g711ulaw noise:-32 acom:6 i/0:-13/-13 dBm
```

`tx:<tot>/<v>/<fax>ms` — **4550 ms total, 4550 ms voice, 0 ms fax/modem** — and
no `MODEMPASS` block on the leg. `modem passthrough nse codec g711ulaw` is
configured on `dial-peer voice 8999 voip`, but NSE has to be negotiated with the
far end and neither Asterisk nor this project signals it, so **the DSP stays in
voice mode for the entire call**.

That is the difference in kind we had not accounted for. Everything fixed so far
— EC off, NLP off, fixed playout, no VAD — removes individual voice features,
but the path is still the voice pipeline rather than a clear channel. CLAUDE.md's
first constraint is that the RTP payload *is* the DS0 stream the far end's D/A
sees; a voice-mode DSP does not promise that, whatever features are disabled.

Against this: the same voice path carries the Courier's upstream V.34 Ja well
enough to yield CRC-valid DIL descriptors, and carries our Sd/S̄d well enough
that the peer acts on the §9.3.2.4 transition. So it is not grossly destructive
in either direction — which is why this is a candidate, not a conclusion.

Two ways to settle it, in increasing order of effort:

1. **Force the DSP out of voice mode.** Either make our side answer the NSE, or
   find a VG224 configuration that engages passthrough on local tone detection
   without peer negotiation.
2. **Replay the Eicon downstream at this Courier** (section 27). Still the
   single most decisive call available: it removes our transmitter from the
   experiment entirely.

Also found: a **stuck call on port 2/21**, 57 minutes and counting, to a bogus
number, with an off-hook FXS and 171763 packets sent. It is not one of the three
modems under test (2/5, 2/13, AudioCodes) and sits on a different channel of the
same DSP (0/1:3 vs our 0/1:1). Two active calls is trivial load for that DSP so
it is unlikely to be causal, but it is real and should be cleared.

## 29. Modem passthrough cannot be engaged from the gateway alone

Tried both methods the VG224 offers on `dial-peer voice 8999 voip`
(`modem passthrough ?` → `nse | protocol | system`):

- `modem passthrough nse codec g711ulaw` (as found): mid-call
  `Tele 2/5 tx:4550/4550/0ms` — **0 ms modem**.
- `modem passthrough protocol codec g711ulaw` (SDP renegotiation, the SIP
  method): mid-call `Tele 2/5 tx:12500/12500/0ms` — **0 ms modem**.

Setting `protocol` needs the fax protocol removed first (`no fax protocol`;
`fax protocol none` is not enough — IOS still reports "need to unconfigure fax
protocol first"), and warns "modem passthru protocol supported only on sip
signaling", which is satisfied here.

**Neither engages.** Both methods trigger on detecting a modem answer tone and
then require the *far end* to complete the switch — NSE needs the peer to honour
Cisco's named signalling events, `protocol` needs the peer to accept a SIP
re-INVITE. Our ANSam originates on the IP side (V.90 puts the analogue modem on
the calling side, so the digital modem answers), and neither Asterisk nor this
project participates in either mechanism. Nothing configurable on the gateway
alone will switch that DSP out of voice mode.

Call outcome was unchanged in protocol mode: Ja parsed, Sd → S̄d → TRN1d → Jd,
peer retrained, no S, no DIL.

**So the voice-mode theory from section 28 is untested, not disproven.** Testing
it properly needs our side to participate in passthrough negotiation, which is a
real piece of work (recognising and answering NSE, or handling the gateway's
re-INVITE) and is only worth doing if there is other evidence the voice path is
lossy — the evidence so far says it is not grossly so, since it carries CRC-valid
Ja upstream and an Sd the peer acts on.

Gateway restored to `modem passthrough nse codec g711ulaw` +
`fax protocol pass-through g711ulaw` afterwards, so other users of this VG224 are
unaffected. Retained from earlier sections: per-port `no echo-cancel enable` /
`no non-linear` on 2/5 and 2/13, `playout-delay mode fixed`, and
`output attenuation 0` on 2/5 (startup-config still holds the old −6).

Note also: port 2/21's long-running call is a **live call to an Asterisk
extension**, not the stuck call section 28 assumed. Do not clear it.

## 30. SOLVED against SmartLink: the S confidence gate WAS the intermittency (2026-08-26)

The long-running intermittent `no S after 42304 Jd symbols` retrain loop is fixed,
and the cause was ours: the p3_demod structural check in `me_rx_audio()` was vetoing
the peer's real §9.3.2.7 S.

That check exists for section 6's problem — DIL's §9.3.2.10 early exit, where a single
false S discards the rest of a long impairment study. **It should never have been a
second veto during Jd.** Three reasons, all measurable:

- `v34rx.c` has already required at least 64 equalized symbols of §10.1.3.7's
  alternating or dominant-rotation structure before it publishes the event
  (`PHASE3_S_DOMINANT_MIN`/`STABLE`), so the event is not a bare threshold crossing.
- §9.3.2.7's S is only **128T** — 40 ms at 3200 baud. Re-running p3_demod over a
  200 ms window scores mostly the *preceding* silence and Jd interval, so its
  6-symbol-pattern test is being asked about a region that is largely not S.
- It is level-sensitive in exactly the way that produces intermittency. On the live
  rig it **rejected the real S at `rx_rms=653` and accepted the identical signal at
  `rx_rms=845`**. The peer had reached `waitForJd` and then sat waiting for a J'd we
  never sent, until our own §9.3.1.5 deadline fired and both sides retrained.

Now applied only when `v90_get_tx_phase() == V90_TX_DIL`. The echo-correlation,
silence, RX/TX ratio and Tone-A gates are unchanged and still apply during Jd.

Two smaller defects fell out beside it:

- The Jd interop-resync path set `jd_resync_wait = true` and then cleared it four
  lines later (a stray paste, visible in the indentation), so `V90_TX_WAIT_JA` always
  used `V90_WAIT_JA_FALLBACK_SAMPLES` rather than the resync window it had just
  chosen.
- `ME_V90_SD_DELAY_MS` and `ME_V90_SD_DELAY_RETRAIN_MS` clamped at 5000 ms. §9.3.1.3
  reads "After receiving Ja, the digital modem **may wait for up to 500 ms** and shall
  then transmit signal Sd for 384T"; both clamps are now 500.

Measured over the live captures in `artifacts/sd-*`, before and after:

| run | S events accepted | p3 rejects | `no S after 42304` | Phase 2 retrains |
|---|---|---|---|---|
| `sd-phase-sweep-011320Z` | 10 of 47 | 37 | 26 | 146 |
| `sd-delay250-fixed-005458Z` | 2 of 10 | 8 | 22 | 53 |
| `sd-jdwait5s-010714Z` | 1 of 4 | 3 | 0 | — |
| **`sd-jd-gate-fix-013254Z`** | **2 of 2** | **0** | **0** | **5** |
| **`sd-jd-gate-fix-r2-013643Z`** | **2 of 2** | **0** | **0** | **5** |

Both post-fix runs complete Phase 3 (S at 21392 Jd symbols, 2.67 s — well inside
§9.3.2.7's 5000 ms), reach Phase 4 CPt/TRN2d/MP and acquire B1 at 100% fit.

**Still open, and a different problem**: the first Phase 4 attempt in each post-fix
run ends in `Peer retrained during tx_phase=15` after the post-CPt classifier walks
`repeating-CPt` -> `CP-sync-candidate-incomplete`; the *second* attempt is the one
that reaches B1. That is CP acquisition, not S detection.

## 31. The first Phase 4 attempt transmitted 7.4 dB hot (2026-08-26)

Section 30 left "the first Phase 4 attempt always ends in `Peer retrained during
tx_phase=15`" as open, and separate from S detection. It is, and it is ours.

**It reproduces offline, deterministically**, which nothing in the upstream work
does:

```bash
./v90_engine_replay artifacts/sd-jd-gate-fix-r2-013643Z/live-rx.g711 ulaw --fast
```

The peer's retrain is a real 2400 Hz Tone A — there are exactly six pure-tone
stretches in that whole recording (three in Phase 2, three around the retrain),
all at mean square 8868394, and the detector's own Goertzel reads ratio 1.000.
The `V34_RETRAIN_TONE_DEBUG` print added here is what established that; the
detector had no visibility at all, and "which of the three PEER_RETRAIN sites
fired" cost a round of guessing before it existed.

**What made the peer retrain is the level we transmitted.** Its first CPt selects
Ucodes `46 68 81 88 95 99 103 107`; the one after the retrain selects
`33 53 65 72 79 83 87 91` — the same ladder shape one G.711 chord higher, which
is exactly the self-similarity trap DIL alignment falls into (`u` and `u+16`
differ by exactly 2×; see `docs/v90_constellation_selection.md`).

Both frames set `codec_constellations_differ`, and **their codec-output sets are
the same 4128**. So §8.5.2 is satisfied where the Recommendation measures it —
"the measurement point specified by bit 38 of INFO0d", which we set to the codec
output — and the peer is conformant. It has mis-measured the *pad*: its first CPt
asserts 6 dB of digital loss between our output and the far codec.

Nothing on this side looked at any of it. We transmitted the frame's
transmit-side Ucodes literally:

| | wire RMS | dBm0 | codec RMS | outcome |
|---|---|---|---|---|
| attempt 1 CPt | **8364** | **-5.6** | 4128 | peer retrains 2.4 s later |
| attempt 2 CPt | 4129 | -11.8 | 4129 | CP, CP', B1, data mode |
| data-mode CP | 4014 | -12.0 | 4014 | — |

Our own INFO0d announces a maximum transmit power of **-13 dBm0** (code 25), so
attempt 1 put TRN2d and MP on the DS0 7.4 dB above what we had told the peer we
would ever send.

**The fix follows from the bearer, not from a threshold.** There is no pad to
claim here — the RTP payload *is* the DS0 the far-end D/A sees — so when a CPt or
CP asks us to transmit above our declared Table 15 limit while naming a
codec-output set that is within it, we transmit the codec-output set. It is a
no-op whenever the two agree, which is every frame in every working capture.

A/B on one recording, one variable (`ME_V90_CP_PAD_REPAIR`): first-attempt
TRN2d/MP moves **-5.6 dBm0 → -11.8 dBm0**, the level the working attempt uses,
with the second attempt and the data-mode CP byte-identical.

No false positives across six calls: it fires on the first attempt of both
captures carrying this failure (`sd-jd-gate-fix-013254Z` and `-r2-013643Z`, the
same figures to the digit) and on none of four that reach data mode
(`goal-matrix-115515Z/rate19200-r1`, `rate24000-r1`, `rate28800-r1`,
`slip-ab-28800-001150Z/fixed-r1`), where the wire and codec sets are equal.

**Deliberately not a rejection.** Our own analogue role builds CPt from the
*uncapped* plan on purpose — power-thinning it fell the Eicon card back to Phase 3
in run 86 (`docs/v90_analogue_role.md`) — so refusing every over-Table-15 frame
would reject frames this project itself emits, on no evidence that refusing
helps. An unrepairable frame is logged and transmitted;
`ME_V90_CP_POWER_ENFORCE=1` refuses it instead. That tension is real and is now
visible in the log rather than hidden.

Knobs: `ME_V90_CP_PAD_REPAIR=0` (leave the frame alone),
`ME_V90_CP_POWER_MARGIN_DB` (default 3 dB — the middle of the 6 dB chord that
separates the two cases, rather than a value fitted to either edge; 24 disables
the check and restores the previous behaviour), `ME_V90_MAX_TX_DBM0_CODE`,
`V90_CP_POWER_DEBUG`, `V34_RETRAIN_TONE_DEBUG`.

**Open:** not verified live — the recording's peer behaves as recorded whatever we
transmit, so only a rig call can show whether the quieter first attempt survives.
Also open, and noticed on the way: INFO0d's bits 29:32 (nominal Phase 2 power)
and 33:37 (maximum transmit power) are both filled in from the same -13 dBm0
measurement in `prepare_info0d()`, which conflates two different fields.

## 32. Live: the pad repair is refuted, and the peer grades Phase 4 with a stopwatch (2026-08-26)

Section 31's fix was offline-only, and it does not survive contact with the rig.
**The overdrive it found is real; it is not why the first Phase 4 attempt
retrains.**

The instrument that settles it is the peer's own log. SmartLink grades Phase 4 in
a *linear mapping study in TRN2* and prints both ends of it:

```
V90Phase4Demodulator reset & enable linear mapping study in TRN2.
V90Phase4Demodulator: disable linear mapping study          <- completed
VPcmFloModem (V90): retrain requested !!                    <- gave up
```

Across six live calls that duration is deterministic to the millisecond:

| arm | attempt-1 study | outcome |
|---|---|---|
| `ME_V90_CP_PAD_REPAIR=1` | **0.060, 0.061, 0.060 s** | retrain |
| `ME_V90_CP_PAD_REPAIR=0` | **1.821, 1.820 s** | retrain |
| either arm, attempt 2 | **2.760 s** | completes, data mode |

So the peer wants the transmit-side levels it asked for, and substituting the
codec-output set fails its study **30× sooner**. The repair is default off, and
the numbers are in the code beside it.

Two further live results rule the level story out altogether:

- With `ME_V90_TRN1D_SYMBOLS=8004`, attempt 1's CPt claims **no pad at all**
  (wire 4129 == codec 4129, the same figures as the attempt that always works)
  — and it **still retrains**.
- With `ME_V90_TRN1D_SYMBOLS=16008` (2001 ms), attempt 1's study **completes at
  2.760 s and the first attempt reaches data mode** — while its CPt is *quieter*
  than any seen (wire 2391 / codec 1146).

**Whatever the first attempt fails on, it is not the constellation power.**

### The one lead that did move it

TRN1d length. §9.3.1.4 gives the digital modem 4000 ms from the start of TRN1d to
the start of Jd, and we send 2496T = 312 ms. At 16008T = 2001 ms the first attempt
went to data mode on the one call tried. **It is not adoptable as it stands**: the
downstream came out at **40000 bps against the usual 52000**, because the peer
then designs a much quieter constellation, and the call later retrained out of
data mode twice and fell back to V.34. 8004T (1000 ms) did not help at all.

That is one call each and the rate cost is real, so it is a lead, not a result.
Note also that §21 of this document records 30000T making the *Courier* retrain
sooner — for a different reason (it eats Jd's airtime against §9.3.2.7's deadline,
which TRN1d does not move), so the two findings do not conflict, but neither
generalises to the other peer.

### What the peer's log says the sequence actually is

Worth writing down, because it is the first clear view of the far side of §9.4:

```
Dil study Terminated. Enter error relaxation period.
V90TRN2Designer : ADI design report :
VPcmFloModem (V90): Building CPt, CPt length = 428
V90Demodulator: enter Phase 4
End of CP #1..#7 tx.... (terminateCpNot=1)      <- its CPt sequence, ~500 ms
GetV90CpBits: Indicating CP termination !!!!
V90Phase4Demodulator reset & enable linear mapping study in TRN2.   <- +200 ms
   ... 2.760 s if it converges, or "retrain requested" if it does not
```

Its `V90ConnectionEvaluator (phase4)` trips at `pdsnrCurrentV34DropThreshPhase4
= +250.000`, and its `V90Demodulator: Error Energy` climbs 0.56 → 113 → 222 → 247
→ 248.8 into that threshold. The evaluator's `reset called !` appears only once
per call, before the first Phase 3 — so the second attempt may simply be
*ungraded* rather than better, which is worth testing directly before assuming
attempt 2 proves anything about signal quality.

## 33. Live: the TRN1d lead replicates, and so does its price (2026-08-26)

§32 left `ME_V90_TRN1D_SYMBOLS=16008` (2001 ms, against the 312 ms default) as a
lead on one call: the first Phase 4 attempt reached data mode, where it otherwise
always retrains, but at 40000 bps instead of 52000 and the call then destabilised.
Both halves of that now have a sample.

`tools/soak/v90_trn1d_ab.sh` runs the two arms **alternated rather than blocked**,
one call per run, with `ME_V90_TRN1D_SYMBOLS` the only variable, and
`tools/trn1d_ab_summary.py` scores them. Four calls a side, kept as
`artifacts/trn1d-ab-063857Z/`:

| call | TRN1d | attempt 1 | attempt-1 study | downstream | retrains | U-lines |
|---|---|---|---|---|---|---|
| control-r1 | 2496 | no | 1.82 / retrain | 52000 | 3 | 0 |
| control-r2 | 2496 | no | 1.82 / retrain | 52000 | 1 | 0 |
| control-r3 | 2496 | no | 1.82 / retrain | 52000 | 1 | 0 |
| control-r4 | 2496 | no | 1.82 / retrain | 52000 | 2 | 189 |
| trn16008-r1 | 16008 | **YES** | 2.76 / done | 40000 | 4 | 8607 |
| trn16008-r2 | 16008 | **YES** | 2.76 / done | 40000 | 4 | 0 (fell to V.34) |
| trn16008-r3 | 16008 | no | 1.82 / retrain | 46666 | 4 | 1848 (fell to V.34) |
| trn16008-r4 | 16008 | **YES** | 2.76 / done | 40000 | 4 | 7510 |

**The claim replicates: 3/4 against 0/4.** Four a side cannot carry that on its own
(Fisher exact, two-sided, p = 0.14), but the control arm is not the only evidence
about the default — scoring **every capture in `artifacts/` that ran the default
TRN1d and reached data mode**, the first attempt wins **7 of 106**, so the base
rate is about 6.6% and 3/4 against it is p = 0.002. Those 106 span many code
versions, so they are context rather than a controlled arm; the controlled arm is
today's 0/4.

**The price replicates too, and it is uniform rather than occasional.** Every
control call landed at 52000; every 16008 call landed at 40000 or 46666, because
the peer designs a quieter constellation off the longer TRN1d. All four 16008
calls took **4** retrains against the control's 1–3, and **two of the four fell
back to V.34**, which no control call did. So the trade is not "the first attempt
for free": it is roughly 12 kbit/s of downstream and a call that holds less well,
in exchange for skipping one retrain cycle of about 15 s. **That is a bad trade on
this rig, and the knob stays an experiment.**

**The peer's study duration is a fingerprint, and it is worth trusting.** Across
all 113 scored captures, an attempt-1 study of **2.76 s / disable** means the
attempt reaches data mode and **1.82 s / retrain** means it does not — including
the one 16008 call that lost, which carries the control's signature exactly. It
identifies the outcome about 15 s before our own log does.

**One observation deliberately left unattributed**: the 16008 arm delivered
**17965 intact `U%07d` lines against the control's 189**, despite the lower rate
and the worse stability. That is the upstream frame-phase lock, whose measured
determinant is whether the peer is transmitting when data mode begins
(`docs/v90_upstream_data_path.md`: idle-start 18/19 calls deliver, busy-start
0/2), and nothing here controls for it. Eight calls cannot separate that from an
effect of TRN1d length, and it should not be read as one.

## 34. "V.90 declined by peer INFO1a" is a symptom three retrains downstream (2026-08-26)

A batch of calls that would not reach data mode logged
`V.90 declined by peer INFO1a; continuing as plain V.34` four times, which reads
like a negotiation failure. **It is not one. The peer does not decline V.90; it
gives up on it, and our log reports the giving-up rather than the cause.**

The chain, read off the peer's log and ours on the same call
(`artifacts/trn1d-knee-075140Z/control-r1`):

1. The peer enters Phase 3 and sets `V90Phase3Demodulator: initial state set to
   WaitForSd` — it is waiting for **our** Sd.
2. We receive its Ja and **suppress** the start of Sd: §9.3.1.3 starts Sd only on
   a CRC-valid DIL descriptor, and on this attempt the descriptor never parsed.
   `tx_phase=1` is `V90_TX_WAIT_JA` — we sat silent for the whole attempt.
3. The peer times out in `WaitForSd` after **1.88 s, 2.24 s, 1.88 s** on the three
   attempts and retrains each time (`retrain requested !!` out of `JaTXMIT`).
4. On the third it escalates: `VPcmFloModem (V90): drop to V34 requested !!`,
   `Initiating retrain, requested DP is 34`.
5. Its next INFO1a is therefore a plain V.34 offer — `U_INFO=9`,
   `downstream_code=4`, where V.90 needs 6 — which our strict validator correctly
   rejects, and *that* is the line we log.

So the log line is accurate and its timing is misleading: by the time it appears,
the call was lost three retrains earlier. **The cause is the Ja DIL descriptor
parse.**

That the parse is the discriminator is measurable over the whole corpus. Scoring
every capture in `artifacts/` that got as far as transmitting Phase 3:

| | captures | parsed a Ja DIL descriptor |
|---|---|---|
| reached data mode | 108 | **108/108** |
| never reached data mode | 35 | 28/35 |

Parsing Ja is **necessary and not sufficient** — no call has ever reached data
mode without it. In the calls that work, the descriptor comes back as
`parsed Ja DIL descriptor: N=144 LSP=120 LTP=120`, recovered with
`V.34 hypothesis 8`; in the failing attempts nothing validates and the
`Ja heuristic suppressed (v34rx), awaiting descriptor` line is the last thing
before the peer's retrain.

**Do not diagnose this from the two modems' clocks.** They differ by a per-call
offset (`docs/v34_plain_phase2_call_role.md` records a round lost to exactly
that); what is safe is the peer's own `enterPhase3` → `retrain requested`
interval, which is internal to its log, and the *presence or absence* of the
descriptor line in ours.

**Untested lead**: `ME_V90_DIL_PROFILE=smartlink-adi-qc` pre-loads this peer's
descriptor, so Sd can start without waiting on the parse. It is a bypass of the
Ja-parse lottery rather than a fix for it, and whether it converts these calls
has not been measured.

## 35. Which one: neither. The Ja bits differ from identical samples (2026-08-26)

§34 left the Ja descriptor parse as the cause and two candidates for *why*: our
capture starting too late, or the Ja demodulating badly. **Both are wrong, and
the measurement that settles it also rules out the line.**

The instrument is a timestamp on the existing `Ja search input` line
(`t=%.3fs` off `g_rx_audio_samples`), which is what makes the capture length
comparable between runs at all.

Replaying a call that recovers the descriptor (`trn1d-ab-063857Z/control-r1`)
against one that never does (`trn1d-knee-075140Z/control-r1`):

- **The capture does not start late.** Both begin at exactly `t=9.740s` with 94
  bits, and accumulate at the same 6400 bits/s (2 bits per symbol at 3200 baud).
  The first **48** `Ja search input` lines of the two runs are *identical*; they
  diverge only where the good one reports `parsed=1`.
- **The signal is not worse.** The two recordings are **byte-identical for their
  first 13.94 s** — which contains the whole Ja window. The peer's handshake is
  deterministic and the container is restarted per attempt, so two different
  calls put the same samples on the wire. Any explanation involving line
  quality, level or noise is therefore excluded by construction.
- **The receiver is deterministic**: each recording gives the same answer on
  every replay (0 recoveries and 6 recoveries, repeatedly).

So the same samples, fed to the same code, produce a bit stream that parses in
one run and not the other. Dumping hypothesis 8 from both at ~13000 bits shows
why that is possible: the two captures are **13086 and 13080 bits** — a
**6-bit (3-symbol) difference in where the capture begins** — and the contents
are not a shifted copy of one another (best agreement over any shift in
±64 bits is **0.64**, i.e. unrelated). A three-symbol difference in the instant
the receiver enters the Ja capture stage changes *every bit that follows*, not
merely their alignment.

**That is the Ja-parse lottery, and it is ours.** The 24-hypothesis search
covers the constellation rotation and the sliding window covers the frame start,
but neither covers the state the capture is anchored to. Note what this rules
out for future sessions: it is not the hypothesis (hypothesis 8 carries the
preambles in the failing call too — 3 of them, at an exact 1574-bit spacing), not
preamble brittleness (tolerating up to 3 bit errors in the 17 ones finds no
further frames), not the throttle (pinning hypothesis 8 ahead of the search
changes the parse point not at all — measured, and reverted for that reason),
and not the wire.

**Open**: what the capture anchors to, and why entering three symbols earlier or
later destroys the decode rather than shifting it. `ME_V90_JA_DUMP_EARLY=1`
dumps all 24 hypotheses *before* the search, so a run that parses can be
compared against one that does not; that is the tool this needs next.

### 35a. Correction: §35's comparison was not controlled (2026-08-26)

**§35 claimed the two calls' Ja windows contain byte-identical audio. They do
not, and the bit-level comparison built on that is void.** The arithmetic, which
§35 should have done:

- The replay prints `call starts at 2.88 s`, so the engine's sample counter is
  offset from file position by 23040 samples. The two recordings are identical
  to file byte 111520, i.e. **engine sample 88480**.
- The Ja capture begins around engine sample **77920** and the good run's parse
  lands at about **94357**.

So only the first ~10500 samples of the capture come from shared audio, and the
parse itself happens **after** the recordings diverge. The right conclusion is
the dull one: two different calls received different Ja, and comparing their bit
streams says nothing about our decoder.

Two things from §35 do survive, because they are single-recording facts:
**the receiver is deterministic** (same file, same dump, byte-identical across
runs — checked), and **hypothesis 8 carries the failing call's preambles**
(3 of them at an exact 1574-bit spacing), so the search is not the problem.
The claim that the *signal* is exonerated is withdrawn.

Also withdrawn: "the capture does not start late". The `Ja search input` line is
sampled once per 25 RX frames, i.e. every 500 ms, so two runs both first
reporting `rx_stage=10` at `t=9.740s` are equal only to within half a second —
which is more than enough to account for the 6-bit difference in capture length.
**That log cannot resolve the capture start, and no claim should rest on it.**

### 35b. What the capture is anchored to, from the code

Reading the fill site (`spandsp-master/src/v34rx.c`, the `PHASE3_WAIT_S` J/J'
detector) rather than inferring it from behaviour, each captured pair of bits is
produced from:

1. **The symbol decisions** — `map_phase4_raw_bits(data_bits, h)`, so everything
   the equalizer, carrier and timing loops do feeds straight in.
2. **The previous symbol**, via `phase3_j_prev_valid[h]` / `phase3_j_prev_z[h]`:
   Ja is differentially encoded, so a bit pair is emitted only once a previous
   symbol exists, and its value is a phase *difference*.
3. **The descrambler register** `phase3_j_scramble[h]` — but this is **not** an
   anchor: `descramble_reg()` shifts in the *input* bit
   (`*reg = (*reg << 1) | in_bit`), which is self-synchronising, so any initial
   state washes out within 23 bits.
4. **When the capture starts**, since (2) chains from the first captured symbol.
   The arrays are zeroed at two sites (v34rx.c:8607 and :16195), so a reset
   mid-Ja restarts that chain.

So the parts that can make the same signal decode differently are the symbol
decisions and the differential chain's starting point — **not** the descrambler
and **not** the hypothesis. That is where an experiment should be aimed, and it
needs **one recording** with the capture start deliberately perturbed, not two
different calls: §35's mistake was to vary the recording and the decoder state
together and attribute the result to the decoder.
