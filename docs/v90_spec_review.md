# V.90 Spec Review And Implementation Plan

This document summarizes the current state of the repository's V.90 support
against ITU-T V.90 (09/98), using the local reference copy in
`ITU Docs/T-REC-V.90-199809-I!!PDF-E-1.pdf`.

## Summary

The current implementation is best described as:

- Phase 2: mostly implemented and reasonably close to the spec
- Phase 3 TX waveforms and receiver-gated control: implemented, pending real
  modem interoperability hardening
- Phase 4 digital TX: Ri/TRN2d/MP/MP-prime/Ed/B1d implements CPt-selected
  `Sr=0/1/2/3` and mandatory `ld=0/1`; received E can terminate MP-prime
- Data mode encoder: all 22 downstream rate indices are covered for `Sr=0`;
  `Sr=1/2/3` data shaping implements the mandatory `ld=0/1` algorithms

In particular, the existing code:

- correctly models much of the V.90 answerer-side Phase 2 handshake
- generates `Sd`, `S̄d`, `TRN1d`, `Jd`, and `Jd'` waveforms
- terminates repeated `Jd` only after a strict received-S event
- generates and receiver-gates the DIL branch
- decodes distinct strict CPt and CP/CP-prime parameter sets
- implements negotiated `Sr=0` Ri, TRN2d, Type-0 MP/MP-prime, Ed, and B1d
- carries the B1d mapper state into live data at the CP-selected rate
- surfaces the V.90 E detector from SpanDSP into the Phase 4 state machine
- implements section 5 spectral shaping for `Sr=1/2/3`, including the
  shaping trellis, differential signs, filter metric, and mandatory zero- and
  one-frame lookahead

## Clause-Level Findings

### Core encoder

- `5.1 Data signalling rates`: implemented for all 22 `Sr=0` rate indices
  - Data-mode CP selects D, and the live path reports and consumes that rate.
  - Deterministic PCMU and PCMA vectors cover every rate index.

- `5.3 Scrambler`: implemented
  - The V.34 GPC polynomial is used in the local V.90 code.

- `5.4 Mapping parameters / modulus encoder / mapper / spectral shaping`:
  implemented for data mode with mandatory lookahead support
  - CPt drives Phase 4 training while CP independently drives B1d/data DFI,
    per-interval modulus mapping, scrambler state, and differential signs.
  - `Sr = 1/2/3` supports mandatory `ld=0` and `ld=1` operation.
  - CPt training uses the same shaping rules for TRN2d, MP/MP-prime, and Ed.
  - Table 17's `K=6..24` boundary is enforced for every CPt Sr value.
  - Both laws cover every valid shaped CPt drn/Sr/ld combination and verify MP
    padding remains aligned to six-symbol mapping frames.
  - Optional `ld=2/3` is not implemented.

### Phase 2

- `8.2.3.1 INFO0d`, `8.2.3.2 INFO1d`, and `9.2.1`: mostly implemented
  - The SpanDSP V.34 path has been extended with V.90-specific INFO handling,
    carrier setup, Tone A/B behavior, and INFO1a wait/retry logic.

### Phase 3

- `8.4.2 Jd`: implemented
  - Jd repeats until a strict received-S event, then completes its current
    frame before Jd-prime.

- `8.4.3 Jd'`: implemented

- `8.4.4 Sd` and `S̄d`: implemented

- `8.4.5 TRN1d`: implemented

- `8.4.1 DIL`: implemented for the decoded Table 12 descriptor

- `9.3.1.5 Jd termination`: implemented with receiver event gating

- `9.3.1.6 DIL transmission`: implemented; hardware interoperability remains
  to be established

### Phase 4

- `8.6` and `9.4.1`: partially implemented
  - Strict CPt configures training; strict CP configures B1d/data.
  - Ri, post-CPt Ri, TRN2d, Type-0 MP/MP-prime, CP-prime gating, and Ed use
    Recommendation-shaped timing and mapping.
  - The receiver reports E explicitly, and E terminates MP-prime only after
    acknowledgement and data-mapper configuration are valid.
  - B1d is 48 mapped frames and continues into live data for `Sr=0`.
  - CPt-selected `Sr=1/2/3` shaping covers TRN2d, MP/MP-prime, and Ed with
    independent zero-initialized filter memory.

### Data mode

- Section 5 data mode supports `Sr=0/1/2/3` on the live raw-G.711 path, with
  the mandatory `ld=0/1` shapers and all-rate `Sr=0` fixtures. Legacy
  standalone compatibility APIs remain for older synthetic tests; rate
  rate renegotiation and optional `ld=2/3` remain incomplete.

## Reuse Strategy

The best reuse points already in the tree are:

- existing V.90-aware Phase 2 logic in `spandsp-master/src/v34tx.c`
- existing Phase 3 receive-side detectors in `spandsp-master/src/v34rx.c`
- existing bitstream/CRC/message framing patterns used by the V.34 MP code

What should be reused directly:

- Phase 2 V.90 INFO exchange and Tone A/B/L1/L2 logic
- Phase 3 receive-side `S` and J/TRN detector scaffolding
- framing/CRC patterns from MP serialization/parsing code

What should be used only as a template:

- V.34 Phase 4 MP/MPh exchange
- V.34 data-mode encoder

What still needs V.90-specific implementation:

- optional `ld=2/3` lookahead
- downstream and upstream hardware interoperability/retrain hardening

## First Patch Set

Status: completed.

The first patch set should stay small and reduce the biggest spec gap without
trying to implement all of Phase 4 at once.

Scope:

1. Export a Phase 3 `S` detection helper from the SpanDSP V.34 wrapper.
2. Make `Jd` termination depend on that event instead of a fixed timer.
3. Add the DIL branch point in the V.90 Phase 3 state machine.
4. Stop treating generic V.34 training success as complete V.90 success.

Expected result:

- the code becomes more honest about current V.90 support
- `Jd` behavior moves closer to V.90 `9.3.1.5`
- future DIL and Phase 4 work has a cleaner state-machine entry point

## Follow-On Implementation Order

1. Finish Phase 3 control flow — completed
   - `Jd until S`
   - DIL generation

2. Implement minimum viable Phase 4 — completed through mapped Ed
   - `Ri`
   - `TRN2d`
   - `MP/MP'`
   - `Ed`
   - `B1d`

3. Replace the simplified data encoder
   - negotiated `Mi`
   - modulus encoder
   - mapper
   - negotiated downstream rate

4. Add section 5 shaping support — mandatory data-mode support completed
   - `Sr = 1/2/3`
   - shaping trellis
   - lookahead and shaping filter
   - remaining: optional `ld=2/3`
