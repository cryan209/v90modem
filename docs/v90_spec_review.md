# V.90 Spec Review And Implementation Plan

This document summarizes the current state of the repository's V.90 support
against ITU-T V.90 (09/98), using the local reference copy in
`ITU Docs/T-REC-V.90-199809-I!!PDF-E-1.pdf`.

## Summary

The current implementation is best described as:

- Phase 2: mostly implemented and reasonably close to the spec
- Phase 3 TX waveforms and receiver-gated control: implemented, pending real
  modem interoperability hardening
- Phase 4 digital TX: `Sr=0` Ri/TRN2d/MP/MP-prime/Ed/B1d implemented
- Data mode encoder: negotiated `Sr=0` live path implemented; shaping pending

In particular, the existing code:

- correctly models much of the V.90 answerer-side Phase 2 handshake
- generates `Sd`, `S̄d`, `TRN1d`, `Jd`, and `Jd'` waveforms
- terminates repeated `Jd` only after a strict received-S event
- generates and receiver-gates the DIL branch
- decodes distinct strict CPt and CP/CP-prime parameter sets
- implements negotiated `Sr=0` Ri, TRN2d, Type-0 MP/MP-prime, Ed, and B1d
- carries the B1d mapper state into live data at the CP-selected rate

## Clause-Level Findings

### Core encoder

- `5.1 Data signalling rates`: implemented for live `Sr=0`
  - Data-mode CP selects D, and the live path reports and consumes that rate.

- `5.3 Scrambler`: implemented
  - The V.34 GPC polynomial is used in the local V.90 code.

- `5.4 Mapping parameters / modulus encoder / mapper / spectral shaping`:
  partially implemented
  - CPt drives Phase 4 training while CP independently drives B1d/data DFI,
    per-interval modulus mapping, scrambler state, and differential signs.
  - `Sr = 1/2/3` shaping remains missing.

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
  - B1d is 48 mapped frames and continues into live data for `Sr=0`.

### Data mode

- Section 5 data mode is implemented for `Sr=0` on the live raw-G.711 path.
  Legacy standalone compatibility APIs remain for older synthetic tests;
  shaping, rate renegotiation, and broad all-rate fixtures remain incomplete.

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

What needs new V.90-specific implementation:

- `Sr = 1/2/3` shaping and its trellis/lookahead behavior
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

4. Add full section 5 shaping support
   - `Sr = 1/2/3`
   - shaping trellis
   - lookahead and shaping filter
