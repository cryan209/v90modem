# x2 implementation

The recovered payload mapper is implemented and checked against original DSP
execution. `x2_session.c` now connects capability exchange and downstream
training to the modem engine's raw-G.711 path. Select `--mode x2` (or
`ME_MODE=x2` in the engine replay tool). The supported initial profile is an
answering digital endpoint using PCMU, a 3200-baud high-carrier upstream and
the recovered low-level training alphabet.

The supported short-record path now receives MP, selects downstream data banks
and activates the payload source. It does not yet complete a bidirectional x2
connection: the upstream E/B1-to-data receiver handoff remains unimplemented.

## Implemented

- Six-position mixed-radix amplitude selection with supplied negotiated banks.
- MD=0..6 independent differential signs; rank by level with earlier-position
  tie preference; descending-mask disparity minimization for remaining signs.
- The recovered final output XOR (0 or 0x2a), treated as a format parameter.
- GPC and GPA source scrambling/descrambling with continuous 23-bit history.
- Stateful 8 kHz raw-G.711 transmission, with a source-bit callback, sample
  monitor history, arbitrary output block lengths and resumable source pauses.
- Exact-codeword inverse with continuous sign parity and validation of the
  unused mixed-radix space. Analogue equalization is outside this API.
- Courier 104-bit four-word MP serialization, reflected CRC and strict framing
  validation. The two unprotected tail bits are retained as raw values.
- Supervisor nominal rate-index display table, including special indices 1/2.
- 17-bit INFO0 and 7-bit directional marker codecs with CRC validation,
  1200 Hz differential reception and 2400 Hz digital INFO0 transmission.
- V.8 handoff preserving the last second of received audio, so overlapping
  peer INFO0/marker frames are retained.
- Streaming 16-point Courier MP demodulation with GPA descrambling, strict
  CRC validation and repeated protected-word agreement before committing.
- Fifteen short-record data allocations generated from original I-modem
  CFF4/D0AF execution, with sparse local-mask selection and MD=0..6.
- Six final training samples and 4080 mapped startup samples before switching
  to the engine data stack, preserving scrambler, parity and monitor history.
- Marker-selected V.34 upstream reception through PP/TRN/J, without an INFO1
  exchange. Unsupported marker profiles fail instead of substituting a rate.
- Raw downstream zero/A/B/J/J-ACK/C/D/E script, with J release driven by the
  observed upstream S-bar polarity reversal after sustained S. No guessed
  acknowledgement timer advances the script.

The API accepts explicit B and MD. It consumes B+MD source bits per six-sample
frame; it does not derive that count from a CONNECT string. In particular,
the live first I-modem allocation B=19, MD=5 consumes 24 bits, whereas its
nominal index-one string says 33333. This distinction is preserved.

## API usage

Validate/build `x2_pcm_config_t` from negotiated banks in emission order.
`x2_pcm_tx_init` copies it and resets source scrambler, sign parity and monitor
history. Supply tap 18 for the recovered role-zero source or 5 for role one.
`x2_pcm_tx_g711` emits raw octets at 8000 per second. Feed them directly to the
existing raw-G.711 bearer. A callback returning a negative value pauses input:
partial frames and buffered output remain intact, and the returned output
count may be short. The caller must schedule that case; the mapper does not
invent padding bits or silence. Initialization is required before use.

For manual frame operations, `x2_pcm_encode` takes already scrambled bits,
MD signs first and B amplitudes next, all LSB first, plus the signed delayed
monitor value. The frame API leaves maintaining that monitor to its caller.
`x2_pcm_decode` returns scrambled bits; use `x2_descramble_bit` afterward.

This branch covers local 039F bit 7 clear, FFD9 bit 12 clear, the recovered
monitor coefficients [0,1000,0,0] and six-slot alignment. Alternate companding
and monitor filters need separate implementation. The inverse mapper does not implement an analogue downstream receive front
end. Successful hardware interoperability has not yet been demonstrated.

## Evidence and reproduction

Reference: x2 Technical Specification Draft 0.33, clauses 10 (payload and
scrambling), 11 (signs, disparity and samples), 15/16 (allocation), 21
(Courier four-word MP) and 22 (rate handoff and source-bit accounting), in
`courier-emu/artifacts/x2-spec-20261004/x2-technical-specification.docx`.
Original program: QF060003 C862 builder with SPM=1, B2BA source scrambler,
C5DF amplitude extraction, C642 signs and C544 sample dispatcher. The
implementation is independent C based on recovered behavior, not vendor code.

`x2_firmware_vectors.h` contains outputs from 840 original-instruction
contexts spanning 30 seeded builder profiles and all MD values, plus 56
continuous original-dispatcher streams of 32 frames. These are constructed
profiles, not peer-negotiated captures. No firmware program words are included.
The regression checks the streams at output block sizes 1, 17 and 160 and
with source pauses every three bits. It also checks 7168 continuous frame
round trips through both source polynomials, all MD values and both formats,
single-bit corruption at every protected MP position and invalid parameters.

To regenerate the firmware vectors with the sibling emulator repository:

```
../courier-emu/.venv/bin/python tools/generate_x2_vectors.py ../courier-emu x2_firmware_vectors.h
make x2-test
```

## Startup verification and remaining work

`make x2-session-test` checks INFO codecs, all 256 directional marker/role
cases, unsupported-carrier rejection, training durations and received gates.
`x2_training_capture.h` preserves 1600 PCMU samples around the Courier's
second S/S-bar restart. The detector is checked with input blocks of 1, 17
and 160 samples; a reversed carrier alone and S without a reversal cannot
release J. An optional full peer recording checks INFO0 and marker reception:

```
./x2_session_test /path/to/imodem-rx.g711
ME_MODE=x2 ./v90_engine_replay /path/to/imodem-rx.g711 ulaw --fast --from 0
```

The preserved x2-rate-server-call recording yields peer INFO0 `21ff`, marker
`4d`, upstream J, downstream A/B, S-bar-gated J acknowledgement, then C/D/E
and MP `0344/03fe/0000/0500`, followed by `DATA_STARTUP` and `PAYLOAD`. A recording cannot react to newly generated transmission;
this verifies receive events and engine routing, not a closed-loop call.
INFO0/tone-reversal timing, initial training scrambler state and hardware
acceptance still need closed-loop confirmation. The initial tone waits are
explicit hypotheses in the implementation.

The short-record tests also cover 225 one-hot mask/ceiling selections, 105
frames from original Ie030002 CC95/CE85 execution, every protected single-bit
MP corruption with subsequent recovery, and captured MP demodulation with
1-, 17- and 160-sample input blocks. Payload callbacks remain untouched until
all 4080 mapped startup samples have been emitted; the first payload frame
then consumes exactly 24 bits for the captured B=19/MD=5 allocation.

The live trace's bit-7-clear profile uses A8F1/C940: six final CB60 training
samples, CFF4 bank selection, 0FF0 mapped samples, CC02 source activation.
It skips C936's alignment/record-transmit script. A valid incoming short MP
is sufficient for this branch; this implementation adds two matching CRC-valid
frames as an acquisition qualification. It does not require a transmitted
acknowledgement bit that was not recovered by the audio decoder.

Regenerate the supported banks and frame fixtures with the preserved I-modem
program/data snapshot (the program is executed locally and is not embedded):

```
../courier-emu/.venv/bin/python tools/generate_x2_short_banks.py ../courier-emu /path/to/x2-rate-server-call .
make x2-test x2-session-test
```

`x2_short_record_config` supports mode zero, zero position controls, low-level
PCMU and MD 0..6. Unsupported controls fail without replacing working banks.
The first data alphabet is `a5 a8 ab ae b3 b9 bf cb df`; E uses the distinct
`a5 a7 ad af b7 bd c5 cf e5` alphabet. Nominal index one is 33333, while B=19
and MD=5 carry 32,000 source bits/s. The engine clocks the data stack at this
actual bit rate.

Next work is the upstream E/B1-to-data receiver handoff and closed-loop
bidirectional payload verification. Until then the engine remains in TRAINING
while sending the downstream payload and does not report CONNECT. The separate
variable-length measurement record remains unsupported: the latest firmware
audit shows that it does not execute in this captured call. Its scheduling and
receiver must be recovered before adding that branch. Record waiting has a
bounded ten-second timeout; an activated payload stream is not a training timer.


## Upstream E continuation, 5 October 2026

The MP audio receiver now continues through downstream startup/payload until
it detects upstream E. It requires twenty consecutive decoded ones outside
an MP frame on a timing/phase hypothesis with two matching CRC-valid MPs.
Unqualified ones, the seventeen-one MP sync, other timing hypotheses and
interrupted prefixes cannot release it. The existing captured MP fixture
contains the event at relative sample 11434 (absolute bearer sample 100074),
identical at input block sizes 1, 17 and 160. Original Ie030002 A881/AABF
execution independently confirms the twenty-consecutive-one threshold and interrupted
prefix handling; Courier AE83/AF2E supplies five four-bit all-one symbols.
Evidence: `courier-emu/artifacts/x2-upstream-e-20261005` and
`courier-emu/tools/verify_x2_upstream_e.py`.

Full engine replay logs E and preserves downstream payload activation. The
initial opt-in `ME_X2_UPSTREAM_ACQUIRE=1` probe prepared the existing V.34 T/3
receiver at 3200/high, 24000 bit/s and 16-state trellis after MP, preserving
pre-E capture, then starts B1 acquisition on the detected E. It initially failed on the
preserved recording (template fit below 25 percent); see the B1 fix below.
This is a diagnostic handoff, not verified upstream payload support. The
engine continues to gate CONNECT and received user bits. The B1 continuation below resolves acquisition; user-data mode still needs
upstream payload verification.

Reproduce the recorded event and the optional acquisition probe:

```
make x2-test x2-session-test v90_engine_replay
./x2_session_test /path/to/x2-rate-server-call/imodem-rx.g711
ME_MODE=x2 ME_X2_UPSTREAM_ACQUIRE=1 ./v90_engine_replay /path/to/x2-rate-server-call/imodem-rx.g711 ulaw --fast --from 0
```


## Courier B1 acquisition fix, 5 October 2026

The Courier's original B1A1/A71B builder uses expanded shaping at upstream
N=10 (24000 bit/s, B=60, Q=3); B1C2 selects the 64-state trellis. The x2
receive path now marks its protocol explicitly and searches both shaping
choices alongside the existing two scrambler and three trellis candidates.
Standard V.90 retains its minimum-shaping template. The accepted x2 template
retains expanded shaping and installs the winning convolutional code in the
DATA decoder. The 95% fit requirement and separate post-B1 lattice/power
validation are unchanged.

Default engine replay acquires the preserved Courier recording at 99.9% fit
with GPA/expanded/64-state. The held-out 256 symbols have lattice distance
0.252 and power 147.2 against template power 147.7. `make x2-b1-test` checks
recorded acquisition at block sizes 17/160, the decoder's selected trellis,
and silence rejection. All 402 `v34_data_test` cases also pass. Evidence and
original-instruction mapper comparison: `courier-emu/artifacts/x2-upstream-b1-20261005`.

Acquisition runs by default; `ME_X2_UPSTREAM_ACQUIRE=0` disables it for
comparison. Upstream shell/frame decoding still has errors, and five
rotations remain in the controlled 480-symbol original-mapper comparison.
CONNECT and user bits remain gated until upstream V.42/LAPM payload is
verified. The completed B1 lock is not a bidirectional data-mode claim.


## Closed-loop original Courier testing, 5 October 2026

The engine now has a clocked raw-PCMU peer executable, `v90_engine_peer`.
`../courier-emu/tools/probe_v90modem_closed_loop.py` tests fresh feedback
against original Courier 403 analog and Ie030002 I-modem DSP paths. Both
complete V.8 but fail before accepted marker/B1/CONNECT; capture acquisition
success does not establish live interoperability.

These calls exposed the proprietary V.8 x2 carrier-phase signature. Native
Ie030002 DD62/DD74 negates the carrier between CM/JM messages. Without it,
DE14's classifier result is -131230 and x2 stays off; with it the result is
+210811, DE15 sets 039F bits 1400 and the native report becomes 0071:0007.
An unmodified native control confirms alternating amplitude writes and
positive classifier results. x2 sessions now enable phase inversion between
JM messages through `v8_x2_phase_reversal`; standard V.8 modes leave it off.
`ME_V8_X2_PHASE_REVERSAL=0` disables it for comparison. JM now also carries
PSTN digital-access octet 8D, which alone was insufficient.

The session stops Tone A in MARKER_WAIT and recognizes acknowledged 17-bit
INFO0 during recovery after its initial INFO0. The session/capture tests,
B1 at 99.9% (17/160-sample blocks, silence rejection), all 402 V.34 data cases,
and production builds pass. Default live calls still drop native x2 later:
analog clears its flags at 909B and emits fallback 7-bit marker 13; the
I-modem asymmetric diagnostic passes 95C4..95EF then clears the classification
at 92EF. The received-tone-driven fast Phase-2 transition remains unresolved.
I-modem symmetric x2 also requires a separate session implementation.

Full commands, raw streams, hashes, experiment matrix, and native traces:
`../courier-emu/artifacts/x2-closed-loop-20261005/README.md` and the closed-loop
section in `../courier-emu/docs/x2-v90-protocol-selection.md`.


## Live analog Phase 2 fix, 5 October 2026

The received-tone-driven transition is now implemented. The session detects
1200 Hz Tone B, reverses Tone A after at least 50 ms, detects B's reversal,
and schedules its second A reversal 40 ms after that received event. It
continues A for 10 ms, emits a 160 ms Table 17 multitone probe, and waits for
the x2 marker. V.34 (10/1996) §11.2.1.2.3–.5 and §10.1.2.4 define these
ordinary timing/probe elements. The original successful PCM-server capture
confirms a short probe with RMS about 2450; the previous silent-gap
interpretation was incorrect. Repeated unacknowledged INFO0 now triggers
acknowledged INFO0 recovery (§11.2.2.2.1). Tests exercise silence, recovery,
and timing at RX/TX block sizes 1/17/160.

Two fresh analog Courier 403 feedback calls accept marker 4D and CRC-valid MP
0344/03FE/0000/0500. Native PC 9083 executes and fallback 909B does not. The
software advances through DATA_STARTUP/PAYLOAD, but native upstream E/B1
handlers never execute and neither call establishes CONNECT/user data. The
next blocker is the native training/MP-to-E handoff, including acknowledgement
and startup conditions; changing upstream detection alone cannot fix it.

The I-modem S58=58 diagnostic now completes INFO0 recovery, both A reversals,
and the probe, but receives no marker. An original native S58=58 caller versus
native S58=48 answerer also fails, so this configuration does not qualify
software interop. Other asymmetric native settings remain to be recovered;
the known working symmetric S58=48 pair requires a separate software session.
The prior closed-loop failures above describe the earlier code checkpoint.

Production builds, recorded session/MP/E checks, B1 acquisition at 99.9%
(17/160 samples and silence rejection), and all 402 V.34 data tests pass.
Evidence: `../courier-emu/artifacts/x2-phase2-20261005/README.md`.
The live harness's `--require-marker` asserts acceptance of 4D, not CONNECT.

## Offline call evidence

`vpcm_decode --x2 --visualize-html x2.html recording.wav` receives analogue
INFO0/marker, repeated digital INFO0 and qualified Courier MP/E through
`legacy_pcm_decode.c`. It emits detection positions and raw CRC evidence,
without running a simulated transmitter dialogue. It does not decode the
user payload. See `docs/html_call_export.md` for scope and validation.
