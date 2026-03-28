# V.90 Spec Review And Implementation Plan

This document summarizes the current state of the repository's V.90 support
against ITU-T V.90 (09/98), using the local reference copy in
`ITU Docs/T-REC-V.90-199809-I!!PDF-E-1.pdf`.

## Summary

The current implementation is best described as:

- Phase 2: mostly implemented and reasonably close to the spec
- Phase 3 TX waveforms: partially implemented
- Phase 3 control flow: incomplete
- Phase 4: not implemented
- Data mode encoder: simplified placeholder, not a full V.90 section 5 encoder

In particular, the existing code:

- correctly models much of the V.90 answerer-side Phase 2 handshake
- generates `Sd`, `S̄d`, `TRN1d`, `Jd`, and `Jd'` waveforms
- does not terminate `Jd` according to the spec
- does not implement DIL
- does not implement V.90 Phase 4 (`Ri`, `TRN2d`, `MP/MP'`, `Ed`, `B1d`)
- does not use a negotiated downstream PCM encoder in data mode

## Clause-Level Findings

### Core encoder

- `5.1 Data signalling rates`: missing
  - The code assumes a fixed downstream rate instead of negotiating the
    selected V.90 downstream rate in Phase 4.

- `5.3 Scrambler`: implemented
  - The V.34 GPC polynomial is used in the local V.90 code.

- `5.4 Mapping parameters / modulus encoder / mapper / spectral shaping`:
  mostly missing
  - The data path uses a simplified fixed mapping (`Mi = 128`, `Sr = 0`)
    rather than the negotiated V.90 section 5 encoder.

### Phase 2

- `8.2.3.1 INFO0d`, `8.2.3.2 INFO1d`, and `9.2.1`: mostly implemented
  - The SpanDSP V.34 path has been extended with V.90-specific INFO handling,
    carrier setup, Tone A/B behavior, and INFO1a wait/retry logic.

### Phase 3

- `8.4.2 Jd`: partial
  - Bit layout is implemented, but the startup behavior is not.
  - The spec requires repeating `Jd` until the analogue modem's `S` is
    detected; the code currently terminates `Jd` after a fixed duration.

- `8.4.3 Jd'`: implemented

- `8.4.4 Sd` and `S̄d`: implemented

- `8.4.5 TRN1d`: implemented

- `8.4.1 DIL`: missing

- `9.3.1.5 Jd termination`: missing
  - This is the highest-impact spec mismatch in the current Phase 3 code.

- `9.3.1.6 DIL transmission`: missing

### Phase 4

- `8.6` and `9.4.1`: missing
  - The code currently enters a placeholder hold waveform instead of real
    V.90 Phase 4 signaling.

### Data mode

- Section 5 data mode is only partially represented
  - The repository contains a downstream PCM encoder, but it is a simplified
    placeholder and not the negotiated V.90 encoder required by the spec.

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

- DIL descriptor handling and DIL transmission
- `Ri`, `TRN2d`, `MP/MP'`, `Ed`, `B1d`
- V.90 CP/CPt handling
- full V.90 section 5 downstream PCM encoder

## First Patch Set

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

1. Finish Phase 3 control flow
   - `Jd until S`
   - DIL generation

2. Implement minimum viable Phase 4
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
