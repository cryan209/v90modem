# ITU Call Flows: V.34, V.90, V.92

This note captures call setup flows directly from local ITU-T recommendations so
we can implement role-correct state machines and decoders.

## Source Recommendations

- `ITU Docs/T-REC-V.34-199610-S!!PDF-E-1.pdf`
- `ITU Docs/T-REC-V.90-199809-I!!PDF-E-1.pdf`
- `ITU Docs/T-REC-V.92-200011-I!!PDF-E.pdf`

## Role Conventions Used Here

- `call modem` = originator
- `answer modem` = receiver
- For V.90/V.92 data calls in normal ISP topology:
  - `analogue modem` is usually the caller
  - `digital modem` is usually the answerer

## 1) V.34 Call Flow

Primary clauses:

- Phase 2: `11.2.1.1` (call modem), `11.2.1.2` (answer modem)
- Phase 3: `11.3.1.1` (call modem), `11.3.1.2` (answer modem)
- Phase 4: `11.4.1.1` (call modem), `11.4.1.2` (answer modem)
- Signal definitions: `10.1.3.3` (J), `10.1.3.4` (J'), `10.1.3.6` (PP), `10.1.3.8` (TRN)

### V.34 caller timeline

1. Phase 1 completes (V.8/V.8bis handoff).
2. Phase 2 probing/ranging (`11.2.1.1`):
   - exchange `INFO0c/INFO0a`
   - `Tone B` and phase reversals
   - receive/transmit `L1/L2`
   - send `INFO1c`, receive `INFO1a`
3. Phase 3 first receive window (`11.3.1.1.1`-`11.3.1.1.3`):
   - detect answer `S -> Sbar` (with optional `MD` wait from `INFO1a`)
   - train on `PP` then first `512T` of `TRN`
   - receive `J`
4. Phase 3 caller transmit window (`11.3.1.1.3`-`11.3.1.1.7`):
   - send `S` 128T + `Sbar` 16T
   - optional `MD` (from prior `INFO1c`)
   - send `PP`
   - send `TRN` (at least `512T`)
   - send `J`, then wait for `S` to enter Phase 4
5. Phase 4 final training (`11.4.1.1`):
   - stop `J`, send `J'` then `TRN`
   - exchange `MP/MP'`
   - send/receive `E`
   - send/receive `B1`
   - enter DATA

### V.34 answerer timeline

1. Phase 2 (`11.2.1.2`): mirror role of caller with `INFO0a/INFO1a`, `Tone A`,
   `L1/L2`.
2. Phase 3 (`11.3.1.2`):
   - send silence (70 +/- 5 ms), `S/Sbar`, optional `MD`, then `PP`, `TRN`, `J`
   - detect caller `S -> Sbar`, optional `MD` wait using `INFO1c`
   - train on caller `PP/TRN`, receive caller `J`
   - start sending `S` to enter Phase 4
3. Phase 4 (`11.4.1.2`):
   - send `S` 128T + `Sbar` 16T then `TRN`
   - detect caller `J'`
   - exchange `MP/MP'`, `E`, `B1`
   - enter DATA

## 2) V.90 Call Flow (Analogue Caller, Digital Answerer)

Primary clauses:

- Phase 2: `9.2.2` (analogue modem), `9.2.1` (digital modem)
- Phase 3: `9.3.2` (analogue modem), `9.3.1` (digital modem)
- Phase 4: `9.4.2` (analogue modem), `9.4.1` (digital modem)

### V.90 caller (analogue) timeline

1. Phase 1 completes (V.8/V.8bis handoff).
2. Phase 2 (`9.2.2.1`):
   - exchange `INFO0a/INFO0d`
   - `Tone A/B` reversals and `RTDEa`
   - `L1/L2` probing
   - receive `INFO1d`, send `INFO1a` with mode select
3. Phase 3 (`9.3.2`):
   - send silence 70 +/- 5 ms, `S/Sbar`, optional `MD`, then `PP`, `TRN`
   - send `Ja` while detecting `Sd -> Sdbar`
   - train on first `2040T` of `TRN1d`
   - receive `Jd`, then send `S`
   - detect `Jd'`, send `S` 16T
   - if DIL requested: receive DIL then send final `S/Sbar` to end DIL
4. Phase 4 (`9.4.2`):
   - send `CPt`, detect `Ri` transition
   - optional `SCR` up to 4000 ms
   - exchange `CP/CP'` with digital `MP/MP'`
   - send `E`, then `B1`
   - receive `Ed`, then `B1d`
   - enter DATA

### V.90 answerer (digital) timeline

1. Phase 2 (`9.2.1.1`): complementary side with `INFO0d/INFO1d`, `Tone B`,
   `L1/L2`.
2. Phase 3 (`9.3.1`):
   - detect caller `S/Sbar`, optional `MD` wait, train on `PP/TRN`
   - receive `Ja`
   - send `Sd` 384T + `Sdbar` 48T
   - send `TRN1d` (at least `2040T`)
   - send `Jd` and repeat until caller `S` is detected (`9.3.1.5`)
   - finish current `Jd`, send `Jd'`
   - if non-zero request, send DIL until subsequent caller `S/Sbar`
3. Phase 4 (`9.4.1`):
   - send `Ri` (>=192T), receive `CPt`
   - send `Ri` 24T + `TRN2d` (>=2040T)
   - exchange `MP/MP'` with caller `CP/CP'`
   - send `Ed`, then `B1d`
   - receive `E`, then `B1`
   - enter DATA

## 3) V.92 Call Flow (Analogue Caller, Digital Answerer)

Primary clauses:

- Full Phase 1: `9.1` (same as V.90)
- Short Phase 1: `9.2`
- Full Phase 2: `9.3` (same procedures as V.90 Phase 2, V.92 INFO bit usage)
- Short Phase 2: `9.4`
- Phase 3: `9.5.2` (analogue), `9.5.1` (digital)
- Phase 4: `9.6.2` (analogue), `9.6.1` (digital)

### V.92 caller flow selection

1. Phase 1:
   - use full Phase 1 (`9.1`) or short Phase 1 (`9.2`) when both support it.
2. Phase 2:
   - full Phase 2 (`9.3`) if short mode not selected.
   - short Phase 2 (`9.4`) only if both indicate V.92 and short-Phase-2 desire.
3. If V.92 capability is not mutual, fall back to V.90 behavior (`9.3` text
   explicitly references V.90 INFO usage fallback).

### V.92 caller (analogue) timeline

1. Phase 3 (`9.5.2.1`):
   - send silence, `Ru/Rubar`, optional `MD`, then `TRN1u` (>=2040T)
   - send `Ja` (DIL descriptor), detect `Sd -> Sdbar`, terminate `Ja`
   - train on `TRN1d`, receive `Jd`
   - send `Su` sequences, detect `Jp`, then detect `Jp'`
   - while receiving DIL (or SCR for zero-DIL), transmit `TRN1u`
   - transmit `CPt` until `Ri`, then send `E1u` and move to Phase 4
2. Phase 4 (`9.6.2.1`):
   - transmit `TRN2u`; when ready send `SUVu`
   - exchange `CPu/CPu'` with digital `CPd/CPd'` and `SUV` acknowledgements
   - send `E2u`
   - send `B1u` (or `FB1u` then `B1u` for FPE)
   - receive `Ed`, then `B1d`
   - enter DATA

### V.92 answerer (digital) timeline

1. Phase 3 (`9.5.1.1`):
   - detect `Ru/Rubar`, optional `MD` wait
   - train on `TRN1u`, receive `Ja` DIL descriptor
   - send `Sd/Sdbar`, then `TRN1d` (>=2040T), then repeated `Jd`
   - on `Su` detection, measure phase and transition to `Jp`, then `Jp'`
   - send requested DIL (or `SCR` if zero-DIL)
   - on `CPt`, send `Ri`, continue until `E1u`, then Phase 4
2. Phase 4 (`9.6.1.1`):
   - send `TRN2d`, then `SUVd`
   - exchange `CPd/CPd'` with analogue `CPu/CPu'` and `SUV` acknowledgements
   - send `Ed`
   - send `B1d`
   - receive `E2u`, then `B1u` (or `FB1u + B1u`)
   - enter DATA

## Decode/Implementation Checkpoints

Use these checkpoints to drive demod and state transitions:

1. INFO decode gates:
   - V.34: `INFO1c/INFO1a` drives MD duration and Phase 3 expectations.
   - V.90: `INFO1a` mode bits decide V.90 vs V.34 branch.
   - V.92: INFO bits choose full/short Phase 2 and V.92 vs V.90 behavior.
2. J-family transitions:
   - V.34: `J` completion followed by `J'` is the Phase 3->4 handoff anchor.
   - V.90: digital must repeat `Jd` until caller `S` (`9.3.1.5`).
   - V.92: `Jd -> Jp -> Jp'` plus `Su` phase alignment is mandatory.
3. Final training payload exchange:
   - V.34: `MP/MP' -> E -> B1`
   - V.90: `CP/CP' <-> MP/MP' -> E/Ed -> B1/B1d`
   - V.92: `CPu/CPd (+SUV acks) -> E2u/Ed -> B1u/B1d`
