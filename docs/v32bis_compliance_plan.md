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

## Reference vs SpanDSP

The Python reference layer follows the ITU-T Recommendation by default.

- Default startup modelling uses the ITU-oriented handoff policy:
  differential state is derived from the final transmitted `TRN` symbol,
  scrambler continuity is carried from the end of `TRN`, and the
  convolutional state is explicitly zeroed at `B1` entry.
- The normal-startup scrambler carry-forward is still an interoperability
  assumption, because the Recommendation is less explicit here than it is for
  `TRN` initialization and renegotiation.
- The local SpanDSP comparison harness is still valuable, but it should be read
  as an implementation cross-check, not as the normative source.
- When the comparison harness reports a better match with a SpanDSP-specific
  startup seed, that does not override the Recommendation. It means SpanDSP is
  making additional startup-state choices beyond the explicitly modelled ITU
  handoff path.

## Startup Handoff Status

The startup path now has a sharper split between what the Recommendation says
explicitly and what the reference model still has to infer.

Explicitly anchored in the Recommendation:

- `TRN` starts with the scrambler register at zero.
- Startup differential state is derived from the final transmitted `TRN`
  symbol.
- The convolutional/trellis state is explicitly zero at `B1` entry.
- Renegotiation startup is a separate case with an explicit scrambler reset.

Current ITU-oriented reference policy:

- Normal startup carries scrambler continuity forward from the end of `TRN`
  into the `R`/`E`/`B1` path.
- Repeated startup `R` words are emitted as identical 16-bit words so the
  logical receiver can detect the required repeated-rate pattern.
- `E` is emitted as a standalone startup word with the same ITU-oriented seed
  model, and `B1` begins with the carried scrambler/differential state plus
  zero convolution state.

What remains inferred rather than fully proven from the Recommendation text:

- The normal-startup scrambler carry-forward across `TRN -> R -> E -> B1`.
- Whether every implementation should preserve exactly the same effective seed
  that SpanDSP uses at the first real post-training data symbol.

Current implementation evidence:

- The Python reference path is internally consistent with the ITU-oriented
  startup model and the local receiver/frontend tests.
- The SpanDSP comparison harness shows that the aligned datapump core matches
  SpanDSP once the startup seed is forced to SpanDSP's effective state.
- The current harness therefore distinguishes two questions:
  normative startup modelling and implementation-parity startup seeding.

Practical reading of the local comparison report:

- `seed_summary` tells us whether the ITU-oriented Python startup state and the
  SpanDSP post-training state agree on scrambler, differential, and
  convolutional state.
- `python_spec_exact_match_prefix_symbols` tells us how many initial startup
  symbols match under the ITU-oriented seed.
- `python_spandsp_seed_exact_match_prefix_symbols` tells us how many initial
  startup symbols match when the Python path is forced to SpanDSP's effective
  seed.
- If the SpanDSP-seeded path matches while the ITU-oriented path diverges at
  `scrambled_bits`, the remaining difference is startup-state policy, not the
  datapump core.

Current project stance:

- The Python default remains the ITU-oriented reference path.
- SpanDSP-seeded startup is a diagnostic mode, not the normative default.
- Any future integration into the main modem path should preserve this
  distinction explicitly rather than silently adopting SpanDSP startup seeding
  as the standard.

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
