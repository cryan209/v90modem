# V.90 Live Raw-G.711 And Data-Pump Plan

## Goal

Make the live SIP modem use one standards-shaped V.90 implementation over an
octet-transparent G.711 bearer.

The project should have two deliberately different signal paths:

```text
Downstream:
PTY data -> V.90 mapper -> exact G.711 codewords -> RTP -> FXS -> analogue modem

Upstream:
analogue modem -> FXS -> RTP/G.711 -> linear PCM -> V.34 receiver -> PTY data
```

The downstream V.90 path must select G.711 codewords directly. It must not rely
on a linear-PCM round trip to preserve the selected codeword. The upstream path
must retain the raw G.711 capture while also decoding it once to linear PCM for
the V.34 receiver.

## Implementation Status

Completed on 2026-07-10:

- Milestone 0 green regression baseline
- `make test` running the full loopback suite
- raw `me_rx_g711()` and `me_tx_g711()` live engine APIs
- direct PJMEDIA passthrough routing through the raw APIs
- raw V.90 Phase 3/4 codeword generation in `v90.c`
- one live `v90_state_t` for Phase 3, Phase 4, and connected data
- six-symbol data-frame continuity across arbitrary RTP pull sizes
- live raw RX/TX taps and G.711 path counters
- μ-law and A-law raw/linear parity, reset, chunking, and round-trip tests
- typed strict receiver events for INFO1a, S, TRN/J, CP, E/B1, failure,
  retrain, and timeout conditions
- live Phase 3 entry gated by a CRC-valid, reserved-bit-clean INFO1a with valid
  V.90 symbol-rate fields
- distinct received-S event counting so Jd and DIL termination cannot collapse
  into one sticky receiver status
- negative transition tests proving J/J-prime cannot terminate Jd and malformed
  Phase 4 input cannot end TRN2d

Completed on 2026-07-11:

- strict Table 14 CPt framing on the live analogue-to-digital V.34 bitstream
- variable CPt length derived from the six DFI fields
- validation of frame sync, inserted starts, reserved bits, CRC, fill, DRN,
  4-point symbol alignment, and negotiated G.711 law
- live `CP_VALID` state transition plus CP input/valid/rejected diagnostics
- positive and malformed CPt vectors for both μ-law and A-law
- CPt shaping-redundancy and upstream-rate fields from Table 14
- negotiated `Sr=0` six-symbol Phase 4 modulus mapping from CPt DFI/masks
- standards-timed 192T Ri, post-CPt 24T Ri, and 2040T TRN2d
- Type-0 MP/MP-prime construction, CRC, mapping-frame padding, acknowledge
  exchange, CP-prime gating, and two-frame mapped Ed
- μ-law and A-law tests for timing, constellation membership, MP layout/CRC,
  MP-prime repetition, and strict malformed/unsupported CPt rejection
- distinct CPt training and CP/CP-prime data-mode parameter sets
- Table 14 bit-19 polarity plus lookahead, TRN1d gain, and shaping-filter
  fields preserved for strict repeated-frame comparison
- MP acknowledgement held clear until data-mode CP and Ed held until CP-prime
- B1d corrected to 48 mapped data frames with its required zero-state reset
- live payload mapping at the CP-selected rate with exact bit-reservoir
  consumption and continuity from B1d
- deterministic B1d/payload vectors and 100-frame packing tests in both laws

The parity test confirms why the raw bearer is required: 72 of the first 512
μ-law Phase 3 octets change when decoded to linear PCM and re-encoded, including
the `0x7F` negative-zero codeword. The raw path preserves those exact octets.

## Scope

This plan covers:

- a raw G.711 API between PJMEDIA and `modem_engine`
- one live `v90_state_t` owner for Phase 3, Phase 4, and data mode
- receiver-driven V.90 startup transitions
- a full negotiated V.90 section 5 downstream data pump
- V.34 upstream receive hardening
- RTP timing, loss, and codeword-transparency diagnostics
- synthetic, capture-based, and hardware interoperability tests

This plan does not initially cover:

- V.42 LAPM, MNP, V.42bis, or V.44
- modem relay with V.150.1
- broad PBX/provider interoperability before direct-LAN G.711 works
- replacing useful SpanDSP DSP blocks merely to make them local

V.42 is a follow-on milestone after the physical layer can exchange an
unframed test pattern reliably.

## Design Decisions

1. `vpcm_g711_stream.[ch]` remains the canonical raw bearer abstraction.
2. PJMEDIA is a transport adapter. It does not own modem phase progression.
3. `v90_state_t` becomes the only V.90 Phase 3/4/data state owner in the live
   application.
4. V.91 helpers may supply reusable mechanisms, but V.91 startup objects and
   semantics must not drive a live V.90 call.
5. Strict, CRC-valid decoders may change live modem state. Recovery heuristics
   are diagnostic-only.
6. Voice-path transcoding, VAD/CNG, PLC, AGC, noise reduction, and gateway echo
   cancellation remain disabled. A modem-specific echo canceller used by the
   V.34 receiver is a separate DSP function and remains allowed.
7. Downstream codeword generation and upstream waveform reception are tested
   independently before full-duplex interoperability is used as the gate.
8. Transport refactoring must preserve the existing tone-based V.8, V.34, and
   V.22bis paths while the raw V.90 path is introduced.

## Current Baseline

The repository already has:

- a buildable SIP application and offline decoder
- `vpcm_g711_stream`, call, link, and call-pair abstractions
- V.8 loopback over analogue-over-G.711
- V.90-aware V.34 Phase 2 work
- V.90 Phase 3 and Phase 4 transmit scaffolding in `v90.c`
- strict and recovery-oriented V.90/V.92 analysis tools
- raw G.711 capture support in the loopback harness
- real-modem capture artifacts

Remaining baseline issues:

- legacy standalone V.90 test APIs still expose the old byte-per-codeword
  compatibility mapper; the live modem path no longer uses it.
- `vpcm_v90_session.c` still uses V.91 compatibility and placeholder objects in
  parts of its startup contract.
- only `Sr=0` is accepted; spectral shaping (`Sr=1/2/3`) is not implemented.
- the synthetic coupled-training harness still infers some remote events from
  its proxy transmitter stages even though the live Jd/DIL path no longer does.
- `clock_recovery` is initialized and reset but is not connected to RTP timing.

## Target Interfaces

Exact names may change during implementation, but the ownership boundary
should look like this:

```c
/* Network -> modem. Preserve octets, then decode once where linear DSP needs it. */
void me_rx_g711(const uint8_t *codewords, int count);

/* Modem -> network. Produce exactly count PCMU/PCMA octets. */
int me_tx_g711(uint8_t *codewords, int count);

/* Optional transport metadata, kept separate from the sample payload. */
void me_note_rtp_timing(const me_rtp_timing_t *timing);
```

`me_tx_g711()` selects one of two internal paths:

- raw V.90 codeword generation during applicable Phase 3/4/data states
- existing linear waveform generation followed by exactly one companding step
  for V.8, V.34, and V.22bis

`me_rx_g711()` does both of the following without modifying the input octets:

- supplies raw codewords to capture and codeword-aware analysis
- converts once to linear PCM for V.8/V.34/V.22bis receive DSP

The PJMEDIA adapter copies extended-frame payload octets between the network
port and these APIs. It must not decode and re-encode a raw V.90 transmit frame.

## Milestone 0: Establish A Green Baseline

### Work

- Fix the V.91 full-duplex primitive test so a completed direction does not
  call the transfer helper with a zero-length block.
- Add a documented `make test` target that runs the intended local regression
  set.
- Separate fast unit/primitive tests from long capture and hardware tests.
- Record the current expected warnings so new warnings are visible.

### Acceptance criteria

- `make -j4` succeeds.
- `./vpcm_loopback_test --session-only` succeeds.
- `./vpcm_loopback_test --primitive-tests` succeeds.
- `./vpcm_loopback_test --all-tests` succeeds.
- The test runner exits non-zero on the first real failure and identifies the
  failing layer.

## Milestone 1: Add The Live Raw-G.711 Seam

### Work

- Add `me_rx_g711()` and `me_tx_g711()` to `modem_engine.[ch]`.
- Keep `me_rx_audio()` and `me_tx_audio()` as internal linear-DSP helpers or
  temporary compatibility entry points.
- Change the PJMEDIA passthrough wrapper to pass PCMU/PCMA octets directly.
- Use the linear fallback only when passthrough is unavailable, and label that
  mode unsuitable for proving V.90 codeword transparency.
- Preserve the negotiated law in one per-call value shared by the transport
  and modem engine.
- Add counters for G.711 octets received, transmitted, dropped, and generated
  through the linear fallback.
- Add optional raw RX/TX taps at the live engine boundary.

### Tests

- Exhaustively test all 256 PCMU and all 256 PCMA octets through the bearer
  copy path.
- Test representative V.90 Ucodes and both signs against the expected G.711
  tables.
- Test odd PJMEDIA subframe sizes and multiple subframes per media frame.
- Test law changes only at call setup, never in the middle of a call.
- Compare the engine TX tap byte-for-byte with the payload handed to PJMEDIA.

### Acceptance criteria

- A V.90 raw-codeword test frame is identical at the engine and PJMEDIA
  boundaries.
- The raw path performs no G.711-to-linear-to-G.711 round trip.
- V.8 and V.34 loopback behavior is unchanged.
- The live diagnostics state clearly whether the call is raw-transparent or
  using the linear compatibility path.

## Milestone 2: Make `v90_state_t` The Live Source Of Truth

### Work

- Add raw-codeword Phase 3/4 generation to `v90.c`; the existing linear API
  becomes a wrapper around raw generation when useful.
- Route live V.90 data generation through `v90_state_t`.
- Remove `g_v90_enc`, the duplicate scrambler, and the duplicate simplified
  mapper from `modem_engine.c` after parity tests pass.
- Move negotiated law, DIL, CP, rate, scrambler, differential-sign, and mapping
  state into `v90_state_t` or an explicitly owned child data-pump object.
- Make the transition to `ME_DATA` depend on V.90 completion and valid
  negotiated parameters, not generic V.34 carrier-up alone.
- Keep a temporary compatibility mode for the current simplified mapper so
  transport and state ownership can land before section 5 changes.

### Tests

- Feed the same input and initial state into the old and consolidated
  simplified mappers and compare every generated codeword.
- Verify that Phase 3, Phase 4, and data mode share continuous scrambler and
  differential-sign state where the Recommendation requires it.
- Verify idle generation without consuming application data.
- Verify partial application buffers and frame-boundary carry state.

### Acceptance criteria

- There is one V.90 transmit state owner in the live application.
- `modem_engine.c` contains no independent V.90 data mapper.
- Loopback and live transport use the same `v90.c` codeword generator.

## Milestone 3: Make Startup Receiver-Driven

### Work

- Define typed receive events for INFO, S/S-bar, TRN lock, J/Ja, MP/CP, E, B1,
  failure, retrain, and timeout conditions.
- Replace transmitter-stage inference with explicit receiver events.
- Make `Jd` and DIL termination obey received S events and valid boundaries.
- Use the strict V.90/V.92 Ja/DIL parser in the live path.
- Keep soft-lock, near-CRC, polarity search, and repair output in offline
  diagnostics only.
- Express every fallback as a named standards procedure with a bounded timer.
- Emit one machine-readable event trace containing sample/codeword index,
  state, event, decoded fields, CRC result, and reason for each transition.

### Acceptance criteria

- No normal V.90 phase transition is caused only by the local transmitter
  reaching a numeric stage.
- A malformed INFO, Ja/DIL, MP, or CP frame cannot advance the live state.
- Synthetic valid sequences reach data mode without forced-progress timers.
- Negative vectors produce a documented retry, fallback, retrain, or hangup.

## Milestone 4: Harden The V.34 Upstream Receiver

### Work

- Treat the V.34 receiver as a required V.90 component rather than only a
  fallback modem.
- Validate carrier recovery, symbol timing, equalizer convergence, trellis
  decoding, descrambling, and frame synchronization independently.
- Keep and test the modem-specific echo canceller needed to separate the
  far-end upstream signal from hybrid echo.
- Replace broad waveform heuristics with decoded events where the required DSP
  is available.
- Add stage-specific quality metrics: carrier error, timing error, equalizer
  error, constellation error, retrain reason, and decoded-frame CRC.
- Use `p3_demod` and the offline decoders as cross-checks against the live
  SpanDSP-derived receiver.

### Acceptance criteria

- Checked-in V.34/V.90 captures have explicit expected Phase 2/3 results.
- At least one real analogue modem consistently reaches the upstream data
  receiver on a direct-LAN G.711 call.
- Upstream bit-error measurements can run independently of downstream payload
  correctness.
- Receiver failures identify the failed synchronization or decode layer.

## Milestone 5: Implement The Negotiated V.90 Section 5 Data Pump

The live `Sr=0` milestone was completed on 2026-07-11. CPt configures
TRN2d/MP/Ed, data-mode CP independently configures B1d/data, and the live RTP
path consumes exactly the negotiated D bits per six-symbol frame. Remaining
work in this milestone is broader rate-vector coverage and `Sr=1/2/3` shaping.

### Work

- Represent the negotiated downstream data signalling rate explicitly.
- Derive the six mapping-interval parameters from the negotiated call data.
- Implement the modulus encoder with residue carried across mapping frames.
- Implement per-interval `Mi` and constellation selection from the negotiated
  DIL/CP information.
- Implement mapper framing and exact input-bit consumption.
- Implement differential sign coding with the correct frame-to-frame state.
- Implement `Sr=0` first as a conformance milestone.
- Add `Sr=1`, `Sr=2`, and `Sr=3` spectral shaping, including trellis,
  lookahead, and shaping filter behavior.
- Define underrun behavior that produces valid idle/fill signaling without
  corrupting mapper state.
- Define retrain behavior when the selected constellation becomes unusable.

### Test-vector strategy

For each supported rate and law, fixtures should record:

- input bits
- scrambler input and output
- modulus residue before and after the frame
- `Mi` for all six intervals
- mapper indices
- shaping state and selected signs
- Ucodes
- final PCMU/PCMA octets

Both encoder and diagnostic decoder must consume the same fixtures. Fixture
generation scripts are not the oracle; expected results must be independently
checked against the Recommendation or a trusted captured sequence.

### Acceptance criteria

- The byte-per-codeword compatibility mapper is not used by live V.90.
- Generated throughput matches the negotiated rate rather than an assumed
  64-kbit/s byte stream.
- All supported rates pass long-run residue, frame-boundary, and random-data
  tests.
- PCMU and PCMA synthetic tests pass, even if available hardware validation is
  initially limited to one law.

## Milestone 6: RTP Timing And Impairment Hardening

### Work

- Capture RTP timestamp, sequence, marker, payload size, and arrival-time
  continuity where PJMEDIA exposes them.
- Detect duplicate, missing, reordered, and discontinuous payloads.
- Do not use voice packet-loss concealment on modem media.
- Define phase-aware loss behavior: tolerate, retry, retrain, or terminate.
- Connect or replace `clock_recovery` only after the timing model is explicit.
- Do not insert or delete individual training samples blindly. If clock
  correction is unavoidable, constrain it to a tested, phase-aware mechanism.
- Test fixed packetizations such as 10 ms and 20 ms without changing the
  logical 8-kHz codeword stream.

### Acceptance criteria

- A clean direct-LAN call reports continuous timestamps and zero payload loss.
- Injected loss and reordering produce deterministic diagnostics and recovery.
- Packetization changes do not change the generated codeword sequence.
- The project can prove whether a failed call was a modem decode failure or a
  bearer discontinuity.

## Milestone 7: Hardware Interoperability Matrix

### Minimum topology

- SIP server and FXS gateway on the same LAN
- PCMU or PCMA only
- no transcoding, VAD/CNG, PLC, AGC, voice noise reduction, or gateway AEC
- raw RTP capture plus the four logical modem stream taps
- one analogue modem initially, then multiple independent modem chipsets

### Test progression

1. V.8 negotiation only.
2. V.34 fallback carrier and bidirectional pattern transfer.
3. V.90 Phase 2.
4. V.90 Phase 3.
5. V.90 Phase 4 and negotiated rate.
6. Downstream unframed test-pattern BER.
7. Upstream unframed test-pattern BER.
8. Simultaneous full-duplex transfer.
9. Retrain and rate-renegotiation cases.
10. Additional FXS gateways and PBX/provider paths.

### Acceptance criteria

- A real modem reaches `CONNECT` through the native live V.90 path.
- The negotiated rate and every startup decision are recoverable from logs and
  raw captures.
- Long-running upstream and downstream BER results are recorded separately.
- At least three modem chipset families are represented before broad
  interoperability is claimed.

## Milestone 8: Link Layer Follow-On

Once the physical layer exchanges reliable unframed data:

- add V.42 detection and LAPM
- add MNP fallback if required by the target modem set
- expose framed and unframed diagnostic modes
- add V.42bis and/or V.44 only after LAPM is stable
- test flow control between the negotiated modem rate and the PTY/TCP endpoint

## First Patch Set

Status: completed on 2026-07-10.

The first implementation patch set should stay narrow:

1. Fix the zero-length V.91 duplex test and add `make test`.
2. Add raw `me_rx_g711()` and `me_tx_g711()` entry points.
3. Route PJMEDIA passthrough payloads through those entry points.
4. Add live raw RX/TX taps and byte counters.
5. Add raw-codeword Phase 3/4 output in `v90.c` as a wrapper-compatible API.
6. Prove byte equality from the V.90 generator to the PJMEDIA payload.

This patch set must not change the V.90 mapping algorithm. Its purpose is to
make the bearer exact and establish the ownership boundary needed for later
work.

## Second Patch Set

1. Route live V.90 Phase 3/4/data through `v90_state_t`.
2. Remove the duplicate `modem_engine.c` encoder.
3. Preserve the existing simplified mapper temporarily behind the consolidated
   API.
4. Add state-continuity and partial-buffer tests.
5. Make V.90 data-mode entry depend on the consolidated state.

## Third Patch Set

1. Add typed receive events and an event trace.
2. Replace TX-stage-derived S and completion decisions.
3. Connect strict Ja/DIL parsing to the live state.
4. Add malformed-frame and timeout tests.
5. Create the initial real-call expectation manifest.

## Fourth Patch Set

Status: Phase 4 transmit core completed on 2026-07-11.

1. Decode and strictly validate the analogue modem's Table 14 CPt.
2. Configure the `Sr=0` six-symbol mapper from negotiated DFI and constellation
   masks.
3. Enforce Ri/post-CPt/TRN2d timing and frame alignment.
4. Generate Type-0 MP and repeated MP-prime with CRC and acknowledge handling.
5. Accept only a parameter-identical CP-prime and emit mapped Ed.
6. Replace the B1d/data compatibility mapper with the negotiated mapper.

## Fifth Patch Set

Status: live negotiated `Sr=0` B1d/data path completed on 2026-07-11.

1. Decode CPt and data-mode CP as distinct Table 14 parameter sets.
2. Hold MP acknowledge clear until CP, then repeat MP-prime until CP-prime.
3. Reset the data scrambler/differential state before 48-frame B1d.
4. Preserve mapper state from B1d into connected payload data.
5. Consume D negotiated bits per six G.711 codewords with a cross-frame byte
   reservoir and binary-one underrun fill.
6. Report the negotiated downstream rate rather than a fixed 56/64 kbit/s.
7. Next: add all-rate fixtures, then implement `Sr=1/2/3` shaping.

## Definition Of Done

The raw-G.711 and V.90 physical-layer project is complete when:

- the live downstream V.90 path is codeword-transparent from mapper to RTP
- one `v90_state_t` implementation owns live Phase 3, Phase 4, and data mode
- startup progression is receiver-driven and strict-parser-driven
- the V.34 upstream receiver exchanges a measurable bitstream with real modems
- the full negotiated V.90 section 5 mapper replaces byte-per-codeword mode
- clean, loss, timing, and malformed-frame tests are automated
- multiple real modem chipsets reach stable data mode with reproducible captures
- remaining failures can be assigned to bearer, startup, upstream DSP,
  downstream mapping, or link-layer causes from the recorded evidence

## Related Documents

- `docs/vpcm-refactor-plan.md`
- `docs/v90_spec_review.md`
- `docs/v90_phase_mapping.md`
- `docs/v90_v92_ja_review_plan.md`
- `docs/offline_v34_decoder_plan.md`
- `ITU Docs/T-REC-G.711-198811-I!!PDF-E.pdf`
- `ITU Docs/T-REC-V.8-200011-I!!PDF-E-1.pdf`
- `ITU Docs/T-REC-V.34-199610-S!!PDF-E-1.pdf`
- `ITU Docs/T-REC-V.90-199809-I!!PDF-E-1.pdf`
- `ITU Docs/T-REC-V.42-200203-I!!PDF-E.pdf`
