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

- `Jd until S` control flow
- DIL branch point in native Phase 3
- spec-faithful completion criteria

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

- local SpanDSP V.34 code has strong Phase 4 scaffolding
- native V.90 Phase 4 in our top-level `v90` path is still effectively missing
- the current V.90 session uses V.91 `CP/Es/B1` as a placeholder contract path

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
4. Implement answerer-side Phase 3 control flow as:
   `Sd -> S̄d -> TRN1d -> Jd until S -> Jd'`.
5. Add native Phase 4 entry and stop using V.91 `CP/Es/B1` as the session's
   truth model.

## Source Pointers

- `docs/v90_spec_review.md`
- `spandsp-master/src/v34tx.c`
- `spandsp-master/src/v34rx.c`
- `v90.c`
- `vpcm_v90_session.c`
