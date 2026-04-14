# V.32bis Compliance Plan

This project needs a standards-compliant ITU-T V.32bis implementation, not a
teaching modem or a "V.32bis-like" approximation. The local normative source is:

- [T-REC-V.32bis-199102-I!!PDF-E.pdf](/Users/scottcryan/v90modem/ITU%20Docs/T-REC-V.32bis-199102-I!!PDF-E.pdf)

## Scope

The implementation target is a duplex modem for GSTN and leased 2-wire circuits
with:

- Echo-cancelled full duplex operation
- 1800 Hz carrier
- 2400 symbols/s
- Supported synchronous rates: 4800, 7200, 9600, 12000, 14400 bit/s
- V.32 compatibility at 4800 and 9600 bit/s
- Startup rate-sequence exchange
- In-band rate change without retrain

## Compliance Strategy

We will build the modem in two layers:

1. A Python reference implementation driven directly from the recommendation.
2. An integration path into the main modem code only after the reference layer
   has stable clause-by-clause tests.

The Python layer is not a shortcut. It is the executable specification for the
standard.

## Current Local References

- `ITU Docs/T-REC-V.32bis-199102-I!!PDF-E.pdf`
- `spandsp-master/src/v32bis.c`
- `spandsp-master/src/v17tx.c`
- `spandsp-master/src/v17rx.c`
- `spandsp-master/src/v17_v32bis_tx_constellation_maps.h`
- `spandsp-master/tests/v32bis_tests.c`

Important note: the bundled SpanDSP tree is useful as an implementation
reference, but `spandsp-master/tests/v32bis_tests.c` explicitly marks its
V.32bis support as work in progress. It is not sufficient evidence of
compliance.

## Work Breakdown

### Phase 1: Spec-Locked Tables and Bit-Level Logic

Status: in progress

- Encode the supported bit rates and bits/symbol relationships.
- Implement the differential quadrant encoder from Table 1/V.32bis.
- Implement the trellis/convolutional encoder used by the coded rates.
- Add exact constellation tables for the V.32bis data modes.
- Add unit tests tied to the recommendation tables and figures.

Exit criteria:

- The Python reference encoder emits the correct coded symbol indices for all
  supported rates.
- Unit tests validate the differential encoder truth table and known
  constellation points.

### Phase 2: Scrambling, Framing, and Rate Sequences

Status: pending

- Implement transmit and receive scramblers with caller/answerer directionality.
- Implement startup rate-sequence exchange.
- Add explicit tests for rate-sequence generation and parsing.

Exit criteria:

- Both sides negotiate a common rate in an offline harness.
- Bit-level traces match the intended startup flow.

### Phase 3: Passband Modulation and Receiver Front End

Status: pending

- Pulse shaping
- Carrier generation and recovery
- Symbol timing recovery
- Adaptive equalization

Exit criteria:

- Offline loopback passes at all supported rates over a simulated clean channel.

### Phase 4: Full-Duplex Echo-Cancelled Operation

Status: pending

- Echo canceller
- Duplex startup sequencing
- Robustness under realistic line models

Exit criteria:

- Back-to-back duplex simulation completes training and exchanges data.

### Phase 5: Rate Renegotiation and V.32 Interop Boundaries

Status: pending

- In-band rate changes without retrain
- V.32-compatible operation at 4800 and 9600
- Compliance regression suite

Exit criteria:

- Automated regressions cover rate changes and V.32 compatibility paths.

## Immediate Deliverables

The first Python reference package should include:

- Exact supported-rate definitions
- Differential encoder
- Trellis encoder state machine
- Symbol-index generator for 4800/7200/9600/12000/14400
- Exact constellation coordinates for the lower-rate modes first
- Unit tests

## Known Gaps After This First Step

The initial reference package will not yet include:

- Startup training waveforms
- Passband audio generation
- Echo cancellation
- Full receive-side trellis decoding
- Rate-sequence exchange
- Rate renegotiation

Those are deliberate next milestones, not omissions in the final target.
