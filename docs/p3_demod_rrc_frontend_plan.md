# p3_demod RX Front-End Rewrite Plan (RRC Filter + Resampler + Timing Recovery)

## Update 2026-07-12: spec-timed RRC/equalizer path is now the default

The missing 127-tap T/2 fractionally-spaced equalizer and normalized CMA stage
have been added after RRC+Godard. CMA freezes when the receiver acquires the
long S alternation so that S-bar and 16-point TRN retain their amplitude
structure. TRN analysis was also corrected to distinguish J's differential
encoding from TRN's direct constellation mapping and now evaluates the same
absolute/differential, 24-dibit-map, and serialization hypotheses used by
SpanDSP.

The final classifier treats Phase 4 as a stereo event. This is essential:
S(128T)+S-bar(16T) also opens Phase 3, so even a perfect single-channel S
match cannot identify which phase it belongs to. The decoder now pairs an
answer-channel S/S-bar/TRN boundary with the call channel's J'/TRN boundary
inside the specified response window. It uses the directional baud rates
from INFO1a, tests both carriers and both channel orientations, and only uses
the four-phase timing sweep as a fallback when the stable two-phase sweep
finds no joint event. Single-channel answer-side promotion is disabled by
default (`P3_ALLOW_UNPAIRED_ANSWER_PHASE4=1` exists for diagnostics only).

This recovers 22 of the 26 V.34 calls in the stereo corpus (85%); the V.32bis
capture is correctly excluded. The four remaining captures are
`aztech-ti-28800-33600`, `banksia-wavesp336-33600-33600`,
`maestro-woomera-28800-33600`, and `motorola-sm56-problematic1`. All accepted
events have an explicit call-J'/answer-S timing pair rather than a later
same-channel payload match. `motorola-sm56-problematic2` remains the reference
case: answer S is recovered at exactly 21466.0 ms, with the TRN boundary
exactly 360 samples later at 3200 baud.

The RRC/equalizer front end and normative stereo Phase 4 gate are therefore
now the defaults. `P3_LEGACY_FRONTEND=1` and `P3_LEGACY_PHASE4_GATE=1` retain
the old research baseline when comparison is needed. The old 3/54 result is
not a target: all three were shown to use unrelated later payload TRN and fail
at their claimed immediate spec boundary.

## Update 2026-07-12: normative, role-specific Phase 4 timing gate implemented

The timing contract is now taken directly from ITU-T V.34 (10/96), not from
the legacy segmenter's inferred gaps:

- clause 10.1.3.7 defines S as a two-point alternating sequence;
- clauses 11.4.1.2.1-2 require an answer modem transmitter to send exactly
  S(128T), S-bar(16T), then TRN immediately for at least 512T;
- clauses 10.1.3.4 and 11.4.1.1.1-2 require a call modem transmitter to end J
  with exactly one 16-bit (8T) J' block, then send TRN immediately for at
  least 512T;
- clause 11.3.1.2.6 permits the answer modem to wait no more than 500 ms after
  receiving J before it starts its Phase 4 S.

`p3_find_phase4_timing()` measures those exact symbol spans directly from the
demodulated stream. It tests both I/Q polarities for S, preserves the J
transform when testing J', evaluates the first 512T at the mandated boundary,
and excludes the descrambler's 23-bit synchronization prefix from TRN DBER.
`promote_v34_phases_from_p3()` now passes the receiver role explicitly so the
scanner tests the opposite (transmitting) modem's correct sequence. It also
examines every J candidate rather than assuming the first loose J segment is
the real transition.

At this stage two opt-in research controls were initially used to keep the
work reproducible while the front end was incomplete:

- the RRC front end selected the integrated SpanDSP-table RRC T/2 resampler
  and Godard timing loop;
- the normative gate required either J' plus strong TRN on a call-modem
  transmitter, or the much longer exact 128T S plus 16T S-bar transition on
  an answer-modem transmitter, with TRN beginning at the calculated boundary.

The three legacy 3/54 handoffs all fail the normative gate. At their claimed
immediate TRN boundaries the old front end produces only 46-64% recurrence and
36-54% DBER; the answer-side S/S-bar candidate is only 24%/31%. The RRC+Godard
path also currently yields no accepted transition, so the remaining problem
is no longer ambiguous: the front end/equalization still does not recover the
actual boundary well enough. A later, unrelated clean payload run must not be
used to certify Phase 4.

The initial combined path was 0/54 before equalization and the TRN-domain fix;
the newest update above supersedes that intermediate result.

## Update 2026-07-12: match-quality characterization found no separating scalar threshold

The proposed next experiment has now been run against the three verified
Phase 4 detections and against accepted false matches in ordinary payload data.
`p3_segment_t` now records the learned 16-bit J periodic-match percentage, the
canonical Table 18 match percentage, TRN scrambler-recurrence match percentage,
and TRN descrambled bit error rate. Setting `P3_MATCH_STATS=1` on
`vpcm_decode --v34` prints these values for the exact full-span hypothesis used
by `p3_find_phase4_handoff_sample()`, including its J anchor and chosen TRN.
The diagnostics do not change classification.

Verified handoff measurements:

| Capture/channel | J learned periodic | J Table 18 | Chosen TRN recurrence | Chosen TRN DBER |
|---|---:|---:|---:|---:|
| `motorola-sm56-problematic1` L | 83% | 73% | 1793/1801 (100% rounded) | 21/1824 (1%) |
| `motorola-sm56-problematic2` L | 85% | 69% | 8899/8905 (100% rounded) | 21/8928 (0% rounded) |
| `motorola-sm56-problematic2` R | 84% | 71% | 1262/1297 (97%) | 49/1320 (4%) |

The false/data-mode population overlaps and often looks *better*:

- accepted false J candidates reach 90-95% learned periodicity and 80-81%
  canonical Table 18 match;
- long false TRN candidates reach 100% recurrence with 0-1% descrambled BER;
- `banksia-wavesp336` alone contains a 996-symbol false TRN at 100%/1%, a
  684-symbol false TRN at 100%/1%, and false J candidates up to 95% learned
  periodicity and 81% Table 18 match.

This is enough to rule out a separating threshold on any one of these metrics:
raising J periodicity above 83% or Table 18 above 69% loses a verified handoff,
while lowering TRN BER below 4% or requiring recurrence above 97% retains
ordinary payload candidates that are cleaner than the genuine signal. A
weighted combination of the same monotonic quality metrics has the same
ordering problem.

**Result:** do not tighten `is_trn_pattern()` or `detect_j_pattern()` using
percentage/BER alone. The next segmenter experiment must add independent
structure: the ordered J/J' -> S (128T) -> S-bar (16T) -> TRN (>=512T)
topology, realistic inter-segment gaps/durations, and consistency with the
INFO1-negotiated role/rate. Reintroduce the RRC+Godard front end only together
with that context-aware sequence classifier.

## Update 2026-07-12: Stages 1-2 implemented and validated; Stage 3 revealed a new, more fundamental blocker

Implemented and independently validated Stages 1 (Bresenham T/2 resampler)
and 2 (Godard timing recovery) exactly as scoped below, using spandsp's
literal formulas (`create_godard_coeffs()`, `steps_per_baud[]` including
the 2800-baud approximation, `pri_symbol_sync()`'s cross-correlation +
DC-removal + integrate-and-trigger logic). Standalone validation against
real captured audio (not synthetic tones) confirmed exactly what Stage 1-2
of this plan asked for: the T/2-rate stream is smooth (no more of the
filter-only port's alternating-null artifact), Godard's timing corrections
engage sensibly and stay bounded, and `baud_phase`/`eq_put_step` remain
stable — checked over the *entire* ~20s file (68,571 bauds), not just a
short window. This part of the plan is technically sound and the
implementation is correct.

Wiring this into the actual `p3_demod.c` pipeline (Stage 3) **regressed
the tone matrix from 3/54 to 0/54 phase4** — every previously-verified
detection was lost, and the segmenter now finds dramatically *more* false
S/TRN/J activity than before (e.g. one file went from isolated small false
matches to a single spurious 2792-symbol "J" segment). Root cause: a
materially better demodulator doesn't just recover genuine training signal
more faithfully — it recovers *ordinary payload data* more faithfully too,
at a lower descrambled-bit-error-rate than the old crude filter ever
achieved. Since the segmenter's pattern thresholds (`is_s_pattern`,
`is_trn_pattern`, `detect_j_pattern`) were already shown (2026-07-12,
`v34-offline-decode` memory) to be triggered by real V.34 trellis-coded
data given a good enough recovery, *improving* the front end makes that
existing problem worse, not better. The `p3_find_phase4_handoff_sample()`
"first J" anchor heuristic (fixed the same day, commit a53ac73) assumed
the first J encountered would be the genuine one near INFO1; a
higher-fidelity demodulator increases the odds that a spurious, marginal
J-like match appears even earlier or more densely, breaking that
assumption too.

**This changes the plan's scope.** The front-end rewrite (Stages 1-2) is
necessary but empirically **not sufficient** on its own — this doc's
original Non-Goals section explicitly ruled out touching the segmenter,
but that's no longer defensible: any front-end quality improvement needs
to ship together with tighter segmenter discrimination (either stricter
match/error-rate thresholds in `is_trn_pattern()`/`detect_j_pattern()`,
or some other way to distinguish genuine spec-defined training patterns
from coincidentally-similar trellis-coded data — see the "not yet
attempted" note in the `v34-offline-decode` memory's pinned finding from
earlier the same day, which already flagged this exact tension without
yet having proof). Reverted (`git checkout -- p3_demod.c p3_demod.h`,
verified clean, matrix back to 3/54, known-good detections restored) —
nothing committed. The validated Stage 1-2 code was not preserved in a
branch; re-implementing it is straightforward given how precisely it's
specified below and in the reference line numbers, and re-validating it
standalone (before wiring in) is *not* the expensive part — re-scoping
Stage 3 to include segmenter changes is.

**Next attempt should:** implement Stages 1-2 as below (fast, since it's
now a known-good recipe), but treat Stage 3 as "front end + segmenter
tightening together," not "front end, existing segmenter unchanged."
Consider validating segmenter selectivity changes *independently* first —
e.g. characterize real match-quality statistics (percentage match,
descrambled-bit-error-rate) for confirmed-genuine J/TRN near real
handshakes vs. confirmed-spurious matches deep in data (both populations
now exist in this session's findings) to find a threshold that actually
separates them — before assuming a better front end will help rather than
hurt.

## Goal

Replace `p3_demod.c`'s current RX front end — a crude fixed 5-tap smoothing
filter over NCO-mixed baseband, with symbol timing found by an open-loop
`baud_phase` accumulator and naive linear interpolation between two adjacent
filtered samples — with a correct, closed-loop front end ported from
SpanDSP's own validated V.34 receiver (`spandsp-master/src/v34rx.c`,
`primary_channel_rx()`).

This targets recovering V.34 Phase 3/4 training signal (S/S-bar/TRN/J) on
real, successfully-connected calls where the current demodulator currently
loses lock. Confirmed on real captures (2026-07-12 investigation,
`gough-lui-v34-modem-sounds/`): the existing filter's poor SNR/ISI
performance is a genuine, demonstrated bottleneck, not a downstream
pattern-matching/threshold problem — see the three prior negative results
below.

## Why this, not something else

Three independent prior investigations (2026-07-12, recorded in the
`v34-offline-decode` memory) ruled out cheaper fixes:

1. **Gardner TED** (closed-loop timing correction bolted onto the existing
   5-tap filter): regressed the tone matrix at two different gain
   settings. The signal is oversampled far too tightly (~2.33
   samples/symbol at 3429 baud) for a half-symbol interpolated sample to
   be meaningful with a filter this crude.
2. **Periodic resync** (re-running the existing multi-trial phase sweep
   every ~1.5s instead of once per long window): exactly neutral —
   zero regressions, zero improvement anywhere on the 54-file corpus.
3. **TRN-fragment accumulation with a spec-motivated distance bound**:
   scanned all 54 files for total available TRN evidence within the
   correct ~4s-post-J window; the 3 currently-passing files have
   1704-4536 symbols of margin (3-9x the 512 needed), every failing file
   tops out at 384. No borderline case exists that a threshold or
   accumulation change would flip.

All three point the same way: the bottleneck is signal *quality* coming
out of the demodulator, not the segmenter's pattern-matching logic. Only a
better front end can move this.

## What's already verified (2026-07-12), safe to reuse as-is

- **RRC coefficient tables**: `spandsp-master/src/v34_rx_<baud>_<low|high>_carrier_rrc.h`
  (11 files: 2400/2743/2800/3000/3200 x low/high, plus one shared
  `v34_rx_3429_rrc.h` since low_d/e == high_d/e at that baud). Real,
  validated, 192-phase x 27-tap polyphase bandpass-quadrature (real+imag
  coefficient row pairs) matched filters — confirmed structurally
  consistent (`COEFF_SETS=192`, 27 taps) across all 11 files by direct
  grep, not spot-checked.
  - **Do not use** the same-named files sitting at the repo root
    (`./v34_rx_2400_low_carrier_rrc.h` etc.) — those are broken,
    unpopulated stub files (literal leftover command-line arguments as
    file content) from an unrelated, incomplete generation attempt.
    Include the real ones by explicit path,
    e.g. `#include "spandsp-master/src/v34_rx_2743_low_carrier_rrc.h"`,
    not a plain quoted include (which resolves to the same directory as
    the including file first and would silently pick up the stub).
- **Table-selection mapping**: `(baud_code, carrier_sel) -> table` is
  byte-for-byte identical to spandsp's own `v34_rx_shapers_re/im[6][2]`
  (`v34rx.c` ~line 305), baud order `{2400,2743,2800,3000,3200,3429}`
  confirmed against the authoritative `baud_rate_parameters[]` in
  `spandsp-master/src/v34_tables.h:311`.
- **Filter application primitive**: a circular-buffer dot product reading
  from the oldest sample forward (`pos` = write index just after the
  newest sample was pushed), matching spandsp's
  `vec_circular_dot_prodf(x, y, n, pos)` exactly. Verified against real
  captured audio: sweeping the phase index at a fixed real-data ring
  buffer position produces a smooth, single-peaked magnitude/phase
  response — the primitive itself has no bugs.
- **Per-baud Bresenham stepping constants** (`shaper_sets`, spandsp's
  `steps_per_baud[]` in `primary_channel_rx()`, ~`v34rx.c:7664`):
  `640 / 560 / 540 / 512 / 480 / 448` for `2400 / 2743 / 2800 / 3000 /
  3200 / 3429`. `shaper_sets/192` matches the true samples-per-symbol
  exactly for 5 of 6 bauds. **2800 baud does not** — `540/192=2.8125` vs.
  true `20/7=2.857143`, a ~1.6% error spandsp deliberately accepts (using
  `189` instead of `192` in that one formula to keep the arithmetic in
  exact integers) because the downstream closed-loop timing recovery
  corrects it dynamically during operation. This is the concrete proof
  that the three front-end stages are not independently portable — see
  next section.

## What's confirmed NOT to work — don't retry without redoing all three stages together

A prior attempt (2026-07-12, reverted, nothing committed) ported only the
matched-filter stage, deriving its polyphase phase index directly from
the existing per-symbol `frac` (fraction into the current sample where
the strobe boundary falls) as a simple linear scaling. Symptom on real
data: symbol magnitude alternated between exactly 0.000 and several units
every other symbol, well past filter warm-up — the phase estimate was
intermittently landing on a *null* of the Nyquist-shaped matched-filter
response instead of its peak, which a root-raised-cosine filter produces
by design at neighboring symbol instants (that's what makes it ISI-free).

Two follow-up attempts to independently sanity-check the RRC tables via
frequency-domain analysis of a single 27-tap coefficient row were both
numerically unreliable (wrong analysis axis; then correct axis but too
short a window for reliable frequency resolution) — don't retry ad-hoc
DFT/phase-rotation checks on isolated 27-tap slices. Trust the
table-selection mapping match against spandsp instead.

The correct conclusion: matched filtering, resampling, and timing
recovery are **not separable stages** in the reference design. A
standalone filter-only port cannot work correctly even with a perfect
Bresenham accumulator bolted on, because spandsp's own accumulator
constants (2800 baud) only produce a correct effective rate *given* the
downstream closed-loop correction. This has to be built and tested as
one subsystem.

## Non-Goals

- Do not touch the existing S/TRN/J segmenter (`is_s_pattern`,
  `is_trn_pattern`, `detect_j_pattern`, `p3_segment_symbols()`) or the
  Phase 3->4 handoff search (`p3_find_phase4_handoff_sample()`,
  `promote_v34_phases_from_p3()`) — those were independently fixed and
  verified 2026-07-12 (commits 711877d, 6140e01, a53ac73) and are out of
  scope here. This plan only replaces what feeds symbols *into* that
  layer.
- Do not attempt to replace SpanDSP's live-modem engine or the broader
  offline-decoder architecture — that is the separate, larger effort
  tracked in `docs/offline_v34_decoder_plan.md` (its own "Phase 4" is a
  different, bigger scope: replacing the SpanDSP-backed Phase 2 engine
  entirely). This plan is narrowly the Phase 3 `p3_demod` RX front end.
- Do not attempt a simplified, self-designed resampler or timing-recovery
  algorithm as a shortcut. Port spandsp's actual formulas; the 2800-baud
  finding above is a direct warning against approximating this stage.

## Staged Implementation Plan

### Stage 1: Bresenham T/2-rate resampler over the RRC filter

- Add a `float` (or fixed-point, matching spandsp's `SPANDSP_USE_FIXED_POINT`
  choice — this build uses the float path, confirmed via
  `nm spandsp-master/src/.libs/libspandsp.a | grep vec_circular_dot_prodf`)
  accumulator (`eq_put_step` equivalent) that decrements by
  `P3_RRC_PHASES` (192) every incoming sample, independent of
  `baud_phase`.
- Per baud, calibrate the increment using spandsp's exact
  `steps_per_baud[]` constants (listed above) — do not derive a "cleaner"
  equivalent from `samples_per_symbol * 192`; use spandsp's literal
  integers, including the 2800-baud approximation.
- Output: a continuous complex stream at T/2 rate (twice per baud
  period), matching spandsp's `process_primary_half_baud()` input cadence
  — not a once-per-symbol strobe.
- Verify against known-good real captures (`motorola-sm56-problematic1`
  L, `problematic2` L/R — currently the only 3 verified-genuine passes)
  by dumping the T/2 stream's magnitude/phase trajectory and confirming
  it's smooth (no more alternating-null artifacts) before proceeding.

### Stage 2: Closed-loop Godard symbol timing recovery

- spandsp uses a **Godard timing error detector** here, not Gardner —
  confirmed by reading the actual implementation
  (`create_godard_coeffs()`, `v34rx.c:3847`). Godard TED is a
  non-data-aided, band-edge-filter-based method (two single-pole IIR
  filters centered at `carrier +/- baud_rate/2`, i.e. the spectral edges
  of the signal's occupied bandwidth) that's specifically more robust
  than Gardner at tight oversampling ratios — likely *why* spandsp chose
  it, and a strong hint about why the earlier from-scratch Gardner TED
  attempt on this codebase's own ~2.33-samples/symbol signal regressed
  the tone matrix. Do not substitute Gardner for this stage.
- Port `create_godard_coeffs()` (computes `low_band_edge_coeff`,
  `high_band_edge_coeff`, `mixed_edges_coeff_3` from `carrier`,
  `baud_rate`, and a loop-gain `alpha`) and the per-sample filter
  update + timing-error computation that consumes them (`pri_ted`
  struct fields: `symbol_sync_low`, `symbol_sync_high`,
  `symbol_sync_dc_filter`, `baud_phase`; search `v34rx.c` for
  `process_primary_half_baud` and `total_baud_timing_correction`).
- This closes the loop that Stage 1 depends on for correctness (the
  2800-baud rate approximation, and in general any real capture's
  genuine clock-rate offset).

### Stage 3: Integrate with the existing symbol/segment pipeline

- Decimate the T/2-rate, timing-corrected stream down to one complex
  sample per baud period (spandsp's equalizer/primary-channel boundary),
  feeding into the existing `emit_symbol()` (differential decode,
  descrambler, AGC, symbol storage) unchanged.
- The existing `baud_phase`/`spb` strobe accumulator in
  `p3_demod_process()` goes away entirely for this path — Stage 1+2
  own symbol-boundary detection now, not the caller.
- Keep the existing carrier-frequency selection (`p3_get_baud_params()`,
  `P3_CARRIER_LOW/HIGH`) as the outer hypothesis-selection loop
  (`p3_scan_all_hypotheses()`, `p3_demod_run()`) — that logic is
  independently verified and unrelated to this rewrite.

### Stage 4: Validation

- `make v34-tone-matrix` before and after; the 3 currently-verified
  passes (`motorola-sm56-problematic1` L, `problematic2` L/R) must not
  regress, and their supporting J/S/TRN triples should be re-inspected
  (timing, error rate) to confirm they're still at least as strong as
  today's baseline, not just "still passing."
  - `problematic2` L's current supporting TRN: 4464 symbols, 0.07% error
    rate, landing 3.4s after J. Use this as the quality bar.
- Full `make test` regression suite must stay green.
- Re-test the known-hard cases that motivated this whole investigation
  (`conexant-rh56sp`, `netcomm-roadsteriiser`) as a stretch goal, not a
  hard requirement — their post-J TRN evidence was found to be
  genuinely sparse (48-96 symbols within the correct window, nowhere
  near 512 even summed), so a better front end may still not resolve
  them; that's an acceptable outcome as long as it's not silently masked.

## Effort / Risk Assessment

This is a materially larger, more coupled piece of DSP work than anything
else touched in the `p3_demod` area to date — the four prior attempts
(TRN accumulation, Gardner TED, periodic resync, filter-only port) were
each self-contained, reversible experiments completable in a focused
session; this one requires three new pieces of state working together
correctly *before* it can be validated at all (Stage 1 alone is
expected to still show artifacts without Stage 2, per the reverted
attempt's diagnosis). Budget accordingly:

- Treat as its own dedicated session/task, not something to fit inside a
  broader "improve V.34 detection" turn.
- Verify Stage 1 and Stage 2 independently against known-good real
  captures before wiring into the segmenter (Stage 3) — don't go straight
  to a full tone-matrix run the way the filter-only attempt did, since a
  bug anywhere in Stages 1-2 will look identical to a bug anywhere else
  without intermediate checkpoints.
- If Stage 1+2 together still don't produce clean, artifact-free symbol
  trajectories on the known-good captures, stop and reassess rather than
  proceeding to Stage 3/4 — that would mean the port itself has a bug
  worth finding before it can pollute the segmenter-facing results.

## References

- `spandsp-master/src/v34rx.c`: `primary_channel_rx()` (line 7648) is the
  full reference implementation for Stages 1-3; `steps_per_baud[]` is
  declared at line 7666 inside it; `create_godard_coeffs()` (Stage 2) is
  at line 3847.
- `spandsp-master/src/spandsp/private/v34.h:33-34`:
  `V34_RX_FILTER_STEPS` (27), `V34_RX_PULSESHAPER_COEFF_SETS` (192).
- `spandsp-master/src/v34_tables.h:311`: authoritative
  `baud_rate_parameters[]` (baud order, symbol-rate fractions).
- Godard, D.N. (1978), "Passband Timing Recovery in an All-Digital Modem
  Receiver" — the algorithm `create_godard_coeffs()` implements; useful
  background if the ported coefficients need debugging beyond
  transcription errors.
- `ITU Docs/`: V.34 Recommendation, section 9 (line coding / pulse
  shaping) and section 10 (Phase 3/4 signal structure), for cross-checking
  filter design intent against spec if deeper debugging is needed.
- Memory: `v34-offline-decode` (this session's full investigation
  history, including exact numeric findings referenced above).

## 2026-07-12 V.90 upstream payload investigation

The payload problem is downstream of the verified Phase-4 timing work and
must not be reported as solved merely because a plausible B1 timestamp is
found.

- The dormant SpanDSP receive mapper was hard-wired to the 16-state table and
  used an incomplete eight-branch Viterbi shortcut. The receive trellis now
  evaluates every physical 4D candidate pair and selects the negotiated
  16/32/64-state transition table. A deterministic TX-mapper to RX-mapper
  loopback recovers 15,288 bits with zero errors for each trellis selection.
- V.34 B1 is one complete 35/40-ms data frame (`P` mapping frames), not one
  eight-symbol mapping frame. It also starts with the superframe inversion
  state corresponding to the final data frame. Short 41-bit resemblance is
  therefore only a coarse candidate statistic; full-frame constellation
  geometry is the required discriminator.
- On `Agere-Mars-HV90-bipbipbipbip.wav`, INFO1a and the upstream carrier rise
  remain stable at 10,680 ms and 21,680 ms. However, the strongest zero-
  precoder (MP0) B1 hypothesis has only 7.5% normalized full-frame geometry
  and 41% descrambled ones. It is now explicitly rejected rather than printed
  as a recovered boundary.
- The missing negotiated state is the digital modem's V.90 MP/MP-prime. It
  selects the upstream rate, trellis, nonlinear encoder, shaping, and (for
  MP1) three complex precoder coefficients. INFO1a supplies the upstream
  symbol rate and a capability ceiling, not those selected data-mode values.
- Recovering MP requires first decoding the analogue modem's repeated CPt,
  because CPt defines the PCM constellations used by the digital modem for
  TRN2d/MP/Ed. CPt and MP may use the V.34 16-point Phase-4 modulation (four
  scrambled bits per symbol); a differential-dibit-only receiver discards the
  two quarter-constellation bits and cannot validate CPt.
- Payload success remains a valid B1 frame followed by FCS-valid HDLC/LAPM.
  The public modem-sound clip ends at 25.631 s and may not contain enough user
  payload to produce an HDLC frame, so connected live-call captures must be
  the final payload validation corpus.
