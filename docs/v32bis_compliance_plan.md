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

## Native SpanDSP Baseline

The project build now configures the bundled SpanDSP with both
`--enable-v34` and `--enable-v32bis`. Enabling the dormant code exposed and
fixed its stale Godard-header dependency, obsolete echo-canceller API calls,
and incomplete public lifecycle API. `v32bis_spandsp_test` pins honest
initialisation and restart behaviour at 4800, 7200, 9600, 12000 and 14400
bit/s, supported-rate validation, signal-cutoff forwarding, and waveform
output from the inherited V.17 modulation core.

The clause-level startup logic is now native too. SpanDSP builds and validates
Table 5 R1/R2/R3 words, builds and validates the one-rate Table 6 E word,
selects the highest common rate, generates §5.2's 256T S / 16T S-bar / TRN
conditioning stream with the direction-specific scrambler, and encodes and
decodes startup words through Table 1 while carrying the TRN handoff state.
The native regression is pinned bit-for-bit to the Python reference for both
caller and answerer, including final scrambler and differential states.

This is still infrastructure, not a working modem claim. `v32bis_tx()` and
`v32bis_rx()` still delegate directly to V.17; the new startup symbols are not
yet feeding the V.17 pulse shaper or recovered from its equalizer, and the
allocated echo canceller is not yet in the sample path. The next native seam
is therefore symbol-domain TX/RX handoff, not more startup bit logic.

`make v32bis-test` runs the native smoke test plus all Python reference,
SpanDSP-comparison, waveform, and datapump tests.

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

Blind (oracle-free) startup word decoding:

- The reference transmitter encodes every startup word (each R word and E)
  from the same TRN-derived state: differential state from the final TRN
  symbol, scrambler register carried from the end of TRN. E does not continue
  from the end of R3.
- The blind datapump receiver therefore seeds each candidate word decode with
  the recovered constellation state at the end of the relevant conditioning
  segment (the TRN state labels map to the same points as `Q0..Q3`, so the
  recovered state index is the differential seed directly) and the nominal
  1280-symbol TRN-end scrambler register.
- Decoding isolated 8-symbol windows from a zero scrambler register is wrong
  under this policy: the 23-bit self-synchronizing descrambler never sees
  enough history inside a single 16-bit word to recover, which silently
  suppresses E detection while the looser R sync pattern can still match.
- An E detection is only accepted when its rate field decodes to exactly one
  rate, mirroring the logical receiver's guard.

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
