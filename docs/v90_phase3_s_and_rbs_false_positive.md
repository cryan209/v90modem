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
