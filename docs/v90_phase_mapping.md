# V.90 Caller/Answerer Phase Mapping

This document maps the duplex V.90 startup flow by modem role so we stop
treating V.90 as "V.91 plus extra pieces".

The intent is to give `vpcm_v90_session` and `v90` a spec-shaped target:

- Phase 1 is still V.8/V.8bis handoff.
- Phase 2 is a V.34 control-channel exchange with V.90-specific INFO rules.
- Phase 3 is asymmetric between caller and answerer.
- Phase 4 is also asymmetric and is not the same thing as the V.91
  `SCR -> CP -> Es -> B1 -> DATA` shortcut.

## Why V.91 Is Not The Model

V.91 is a simplified PCM bearer startup model:

- `INFO/INFO'`
- `Eu/Em + DIL`
- `SCR`
- `CP/CP'`
- `Es`
- `B1`
- `DATA`

That is useful as a test seam, but it is not the native V.90 phase structure.

Real V.90 is closer to:

- V.8 handoff
- Phase 2 control-channel INFO/tone exchange
- asymmetric Phase 3 training
- asymmetric Phase 4 training and MP exchange
- only then data mode

So the V.91 startup objects in `vpcm_v90_session` should be treated as a
temporary compatibility layer, not the protocol model.

One important consequence is that V.90 DIL is not the same protocol object as
V.91 startup DIL. V.90 DIL is the Table 12 Phase 3 training descriptor
selected after `Jd/Jd'`, while V.91 DIL is part of the simplified PCM startup
contract. Reusing the same struct shape is acceptable as a temporary transport
adapter, but it should not be treated as a semantic match.

## Role Mapping

Throughout this document:

- caller = analogue modem
- answerer = digital PCM modem

This matches the local SpanDSP V.90 path and the comments in
`spandsp-master/src/v34tx.c`.

## High-Level Duplex Sequence

From the local V.34/V.90 comments and implementation:

### Caller / analogue side

1. V.8 sequence
2. `INFO0a`
3. `A`
4. `!A`
5. `A`
6. `L1`
7. `L2`
8. `A`
9. `!A`
10. `A`
11. `INFO1a`
12. `S`
13. `!S`
14. `MD`
15. `S`
16. `!S`
17. `PP`
18. `TRN`
19. `J`
20. wait for answerer Phase 4
21. `S`
22. `!S`
23. `TRN`
24. `MP`
25. `MP'`
26. `E`
27. `B1`
28. `DATA`

### Answerer / digital side

1. V.8 sequence
2. `INFO0d`
3. `B`
4. `!B`
5. `B`
6. `!B`
7. `L1`
8. `L2`
9. `INFO1d`
10. wait for `INFO1a`
11. `S`
12. `!S`
13. `MD`
14. `S`
15. `!S`
16. `PP`
17. `TRN`
18. `J`
19. `J'`
20. `TRN`
21. `MP`
22. `MP'`
23. `E`
24. `B1`
25. `DATA`

## Phase Ownership

## Phase 1

Shared:

- V.8 / V.8bis negotiation
- role selection
- V.90 capability advertisement

Implementation note:

- `vpcm_loopback_test` should keep creating the two sides and driving V.8.
- `vpcm_v90_session` should start at the post-V.8 handoff.

## Phase 2

### Caller / analogue side

- transmits `INFO0a`
- uses high CC carrier path (`A` / `!A`, 2400 Hz in local notes)
- performs `L1/L2`
- transmits `INFO1a`
- `INFO1a` carries:
  - `MD`
  - `U_INFO`
  - upstream symbol-rate code
  - downstream PCM rate code
  - frequency offset

### Answerer / digital side

- transmits `INFO0d`
- uses low CC carrier path (`B` / `!B`, 1200 Hz in local notes)
- performs Tone B / reversal / `L1/L2` handling
- transmits `INFO1d`
- waits specifically for analogue `INFO1a`

### Session meaning

At the end of Phase 2, the session should own:

- received `INFO0a`
- received `INFO1a`
- `U_INFO`
- analogue-side baud/rate preference
- completion state for handoff into Phase 3

This is the right place for:

- policy translation
- downstream/upstream startup profile selection
- any future DIL branch decision

This is not the place to jump into V.91 `SCR/CP/B1`.

## Phase 3

This is the first major place where V.90 diverges sharply from V.91.

### Caller / analogue side

- transmits `S/!S`
- optional `MD`
- second `S/!S`
- `PP`
- `TRN`
- `J`
- then waits for answerer-side Phase 4 transition

Implementation note:

- For loopback testing we need a caller-side Ja profile that is suitable for
  exercising the answerer parser and DIL branch. That profile should be treated
  as a reusable test vector, not as a statement that all real modems emit the
  same Ja sequence.

### Answerer / digital side

- after valid `INFO1a`, enters primary-channel training
- transmits `S/!S`
- optional `MD`
- second `S/!S`
- `PP`
- `TRN1d`
- `Jd`
- repeats `Jd` until caller `S` is detected
- terminates with `Jd'`
- hands into Phase 4

### Current repo status

Already present:

- Phase 3 receive-side detection scaffolding in `spandsp-master/src/v34rx.c`
- answerer-side waveform generation in `v90.c`
  - `Sd`
  - `S̄d`
  - `TRN1d`
  - `Jd`
  - `Jd'`

Still missing / incomplete:

- strict analogue-side waveform detection still needs hardware validation
- the synthetic session harness retains compatibility proxies in places
- full receiver-driven failure, retry, retrain, and timeout criteria

## Phase 4

Phase 4 is not "CP/CP' in disguise".

### Caller / analogue side

- detects answerer Phase 4 entry
- sends caller-side `S`
- sends `S̄`
- sends `TRN`
- exchanges `MP` / `MP'`
- sends `E`
- sends `B1`
- enters data

### Answerer / digital side

- waits after Phase 3 for caller completion window
- sends answerer-side Phase 4 `S`
- sends `S̄`
- sends `TRN2d`
- waits for far-end `J'/TRN` confirmation
- exchanges `MP` / `MP'`
- sends / detects `E`
- sends `B1d`
- enters data

### Current repo status

- the live V.34 bit callback feeds a strict Table 14 CPt/CP receiver
- `v90.c` enforces Ri/post-CPt/TRN2d timing and maps TRN2d with the negotiated
  `Sr=0` six-interval constellations
- the digital path repeats MP with acknowledge clear until data-mode CP, then
  repeats MP-prime until matching CP-prime or a valid received E event, and
  emits two mapped Ed frames
- B1d is 48 negotiated data frames with a zero-state mapper reset, followed by
  connected data using the same scrambler, differential, DFI, and modulus state
- CPt training and data mode support `Sr=1/2/3` spectral shaping with
  mandatory `ld=0/1`; optional `ld=2/3` remains unsupported
- deterministic PCMU and PCMA tests cover all 22 `Sr=0` downstream rates and
  shaped vectors for each supported Sr/lookahead combination
- repeatable real-modem evidence collection is documented in
  `docs/v90_hardware_interop.md`; no hardware result is implied by that tooling
- the clean-line session contract now couples `v90.c` directly to
  `v90_analogue_phase3.c`: authoritative G.711 downstream drives the analogue
  PCM receiver and its V.34 upstream drives the digital V.34 receiver.  Both
  roles must complete Phase 3, Phase 4 and enter data mode for that case to
  pass.  The clean PCMU path now also drives Phase 4 from CPt and CP frames
  recovered by the digital V.34 receiver rather than the old fixed 512-symbol
  bridge (which truncated a large CPt before one repetition completed).
  `VPCM_V90_NATIVE_UPSTREAM=1` carries the two live V.34 contexts into the
  payload runner; that strict diagnostic currently reaches DATA and produces
  the negotiated number of bits, but fails payload sync.  PCMA CP acquisition
  also remains open.  Compatibility remains the default payload truth until
  those two failures are fixed.  The strict path now detects E in the
  digital receiver's recovered bit clock (rather than at the analogue TX
  timestamp), which removes a measured one-2D-symbol mapping-grid error, and
  the analogue V.34 transmitter emits B1 as §10.1.3.1's complete P-mapping-
  frame data frame rather than a single mapping frame.  The upstream receiver no longer carries the 8 kHz/T/2 geometry into DATA.
  At the E→B1 seam, and only in the V.90 upstream receive branch, it now
  resamples the 8 kHz line stream to 9.6 kHz, applies the 3200-baud 12%-rolloff
  RRC at exactly T/3, searches the complete reset-state B1 frame for timing,
  and solves a supervised T/3 equalizer.  That resampler, RRC history, timing
  phase and equalizer continue unchanged through B1 into DATA; ordinary V.34
  retains its existing T/2 receiver.  The external G.711 accounting and the
  downstream byte-exact PCM path are untouched.  A PCMU-quantized native V.34
  transmitter→receiver regression now acquires B1 and carries **16,000
  pseudorandom payload bits at 21,600 bit/s with zero errors**, while verifying
  the independent 8 kHz/9.6 kHz counts (16,000 external samples, 19,180
  internal samples after causal-filter delay).  Clean synthetic sweeps pass
  through 26,400 bit/s; 28,800 and 31,200 bit/s are rejected by the complete-
  B1 fit gate rather than entering DATA with random output.  Hardware payload
  interoperability remains to be measured.

  A live d-modem attempt on 2026-08-11 did not reach that acceptance point.
  Two correctly configured calls (including one after a full container restart)
  retrained in Phase 3 before CPt/CP/E/B1: SmartLink reported repeated
  `WaitForSd`, either `Error Energy = -0.000` or an unconverged 1800-3600
  plateau, then `retrain requested`/`drop to V34`.  The new T/3 branch therefore
  never armed and the calls neither validate nor invalidate its hardware data
  decode.  Logs are under
  `artifacts/dmodem-t3-live/20260811T070448Z-correct-env/`.

  Follow-up A/B work found one concrete cause of the Phase 3 lottery.  The
  400 ms `p3_demod` Ja scanner accepted a J-labelled segment anywhere near the
  tail of its window even when its Table 18 match was only 69-71%; on two live
  retries those events preceded the canonical Ja event by 409-477 ms.  Sd was
  therefore transmitted while SmartLink was not yet ready and its error energy
  remained exactly zero.  Ja fast-path acceptance now requires both current
  (newest 80 ms) evidence and at least an 85% Table 18 match; weaker live
  signals fall through to the canonical detector.  Retrained attempts also no
  longer accidentally inherit the initial `ME_V90_SD_DELAY_MS`: absent an
  explicit `ME_V90_SD_DELAY_RETRAIN_MS`, they probe the §9.3.1.3 legal window
  at 0, 250 and 500 ms.  A delay-only A/B moved one attempt from zero energy to
  79.7, but not to CPt; the corrected Ja gate likewise has not yet produced a
  complete call.  Artifacts are under
  `artifacts/dmodem-t3-live/20260811T072246Z-adaptive-sd/`.

  The reason for this branch was isolated before implementation.  A direct
  reset-state mapper→demapper check at N=13/16-state trellis carried 77,688
  decoded bits with zero errors, while the old waveform path returned random
  B1.  Cross-checking that same waveform with
  `modem-dsp-emu/tools/v90_rx_reference_demod.py` gave **31.4 dB SNR / 2.7%
  EVM** (mu-law ceiling 37.0 dB) using the same 9.6 kHz/T/3 architecture.
  Set `VPCM_V90_NATIVE_UPSTREAM_DUMP=/tmp/native-up.ulaw` on the strict
  diagnostic to reproduce the captured input.
- peer-initiated retrains are answered per §9.3.1/§9.4.1/§9.5.1.2: a 2400 Hz
  Tone A detector (Goertzel, 80 ms confirmation) runs during the Phase 3/4 RX
  stages in `v34rx.c` and reports `PEER_RETRAIN`; the engine responds by
  restarting the answerer Phase 2 flow (`restart_v90_phase2_locked`) so
  Tone B/INFO0d answer the peer instead of parking in `WAIT_JA`.  This is the
  path SmartLink's failed-`V90TRN2Design` dummy-CPt cycle depends on: it
  studies our TRN2d, deliberately retrains (`SILENCERETRAIN` → Tone A), and
  declares a link error ~3.1 s later if Tone A goes unanswered (observed live
  2026-07-22; the prior silence-gap heuristic missed the 80 ms gap after
  transport filtering)

## What `vpcm_v90_session` Should Eventually Look Like

Target state machine:

1. `PHASE1`
   V.8 already complete, post-handoff silence/Ez only if needed by harness

2. `PHASE2_INFO`
   real V.90 `INFO0/tones/L1/L2/INFO1`

3. `PHASE3`
   role-specific native V.90 training

4. `PHASE4`
   role-specific native V.90 MP/data-mode negotiation

5. `DATA`
   downstream PCM encoder plus upstream V.34 path

Compatibility layers that should disappear over time:

- seeded V.91 INFO state
- V.91-style DIL startup sequencing as the real V.90 branch point
- V.91 `CP/Es/B1` standing in for native V.90 Phase 4

## Practical Refactor Order

1. Keep Phase 2 consumption in `vpcm_v90_session`.
2. Make post-Phase-2 policy decisions from consumed `INFO0a/INFO1a`.
3. Replace seeded V.91 DIL alignment with a native V.90 Phase 3 branch.
   Completed for the clean-line coupled session; impairment scenarios still
   use the compatibility branch when their deliberately sparse DIL yields no
   offerable native constellation.
4. Implement answerer-side Phase 3 control flow as:
   `Sd -> S̄d -> TRN1d -> Jd until S -> Jd'`.
   Completed in the clean-line coupled session.
5. Carry the negotiated Phase 4 mapper through B1d into connected data.
   Completed for `Sr=0/1/2/3` CPt training and data mode with mandatory
   `ld=0/1`.
6. Remove the remaining V.91 `CP/Es/B1` compatibility truth model from the
   synthetic session.

## Source Pointers

- `docs/v90_spec_review.md`
- `spandsp-master/src/v34tx.c`
- `spandsp-master/src/v34rx.c`
- `v90.c`
- `vpcm_v90_session.c`
