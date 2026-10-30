# x2 implementation

The recovered payload mapper is implemented and checked against original DSP
execution. `x2_session.c` now connects capability exchange and downstream
training to the modem engine's raw-G.711 path. Select `--mode x2` (or
`ME_MODE=x2` in the engine replay tool). An answering call runs the digital
server; a call we place runs the analogue client (`x2_client.c`, see "The
analogue client, 19 October 2026"). Both use PCMU, a 3200-baud high-carrier
upstream and the recovered low-level training alphabet.

The supported short-record path receives MP, transmits its three-word response,
waits for upstream E and activates the downstream payload source. Fresh original
Courier 403 feedback reaches native CONNECT and upstream B1 acquisition.
The PCM response-mask correction below establishes complete messages in both
directions against that emulator. The engine enters DATA after accepted MP/E
and upstream B1 acquisition. LAPM is qualified against emulated peers in
both roles (Courier as client, Ie030002 as server); sustained hardware
operation remains unqualified.

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

## Native record/E handoff, 6 October 2026

Fresh feedback against Courier 403 now reaches native `CONNECT 53333/x2/NONE`
and acquires its 24000-bit/s upstream B1. This supersedes the MP-to-E blocker
above, but does **not** establish error-free user data.

The missing transmit stage was a three-word downstream record. Successful
native-to-native execution enters Ie030002 CBB8/B54C with the **training**
alphabet retained; it does not execute CBB5's data-bank rebuild. Emit the
48+6-sample C9F8 alignment sequence, then repeat seventeen sync ones, a zero,
three little-endian 16-bit words with zero separators, reflected-8408 CRC,
and zero padding to four six-sample/24-bit mapper frames. In this profile the
words are D284/03FE/0000 and CRC C2CC. This is distinct from the Courier's
four-word upstream MP (Draft 0.33 sections 6, 20 and 21).

The record sets ACK, expanded shaping and 64-state trellis, and selects the
upstream N independently of the downstream PCM index. Copying N1 into N2
made native B1B4 choose N=0 instead of N=10. Copying native F37C's nonlinear
flag was also wrong for our linear receive path: Courier B06E tests bit 13
(V.34 section 9.7). With it set, the fresh B1 fit was 93.4%; clearing it gives
100.0%, with the separate 256-symbol check at lattice distance 0.027 and
power 150.9 against template power 147.7. The 95% acquisition threshold and
held-out check are unchanged.

Received, qualified upstream E now releases the final six training samples
and data-bank handoff after the currently transmitted record finishes. MP
alone cannot release it. Waiting for MP continues mapped training rather
than inserting silence. Source history/parity continue across the final
training and 4080-sample data startup; user-source callbacks begin afterward.
Tests cover record decoding and repetition, padding/CRC, alignment, missing
MP/E, mid-record E, malformed MP and callback timing at blocks 1/17/160.

Payload remains open. A known-text native Courier call does not yield the
complete message through our waveform receiver. Direct decoding of the
Courier's exact mapper symbols recovers the first four source characters,
`x2-n`; the equalized waveform has localized large deviations from those
symbols despite a small distance to its own chosen lattice. Do not treat
that self-distance as proof of correct symbols. A four-character native
Courier-to-I-modem control delivers `AX2A` upstream but not `DX2D` downstream.
Longer native controls reach CONNECT but fail both complete-message checks.
Thus native downstream payload also needs qualification; neither native
CONNECT nor our B1 lock proves a bidirectional connection. Our engine's
CONNECT/user-bit gates remain in place. A proposed V0/B1 parking change was
not retained because it did not recover the known waveform payload.

`x2_test`, recorded session/MP/E, recorded B1 (99.9%), fresh B1 (100.0%)
at blocks 17/160 with silence rejection, all 402 exact-symbol V.34 data cases,
and production builds pass. `x2_b1_test <tap> <start> <E>` accepts explicit
bearer-clock anchors for fresh captures. Evidence, commands and source-clock
qualifications are in `reference-data/artifacts/x2-handoff-20261006/README.md`.

## Offline call evidence

`vpcm_decode --x2 --visualize-html x2.html recording.wav` receives analogue
INFO0/marker, repeated digital INFO0 and qualified Courier MP/E through
`legacy_pcm_decode.c`. It emits detection positions and raw CRC evidence,
without running a simulated transmitter dialogue. It does not decode the
user payload. See `docs/html_call_export.md` for scope and validation.

## Digital symmetric components, 6 October 2026

`x2_sym.c` implements the recovered raw-DS0 startup source and receiver for
both digital call roles, the capability-frame codec/merge, and continuous
seven/eight-bit data transforms. `x2_info_role_select` in `x2.c` separates
HOST, CLIENT and SYMMETRIC from the calling/answering role: both INFO0 bodies
setting ITU bit 23 select symmetric; otherwise different CME bits select the
asymmetric direction. Equal CME without mutual symmetric capability rejects
x2. Received INFO0 must already have passed its CRC before this selector is
called. References: Draft 0.33 §§12.2 and 13.1–13.3, and the decoded native
routines in `../courier-emu/docs/x2-v90-protocol-selection.md`.

The answerer emits 1747 `7e` octets, seven zeros and 128 `(i,255-i)` pairs.
The caller emits `ff` until it has acquired the answerer's complete ramp,
then starts its own source. The receiver tolerates low-bit errors and records
them per six-sample phase; a larger ramp error restarts acquisition. TX stops
at the exact source boundary and RX stops at the exact ramp boundary, leaving
subsequent capability words to the caller. The error-map phase is relative to
the component's receive counter; its absolute origin against native startup
still needs qualification.

The capability codec uses two start octets, five body octets and four
inverted-CRC nibbles. Merge unions error maps, intersects masks and enables
scrambling only if both flag words request it. An empty error map selects
64000 with the default mask `41`; low-bit impairment removes its full-width
flag and leaves 56000. This codec is tested independently of the session; its
wire output has not yet been compared against a fresh native capability
exchange. No guessed confirmation timer or CONNECT transition is supplied.

The data component masks source words, scrambles LSB first with GPC and
optionally reverses octets; RX reverses before masking and descrambling.
Separate TX/RX histories continue across words. It operates only on raw
G.711 octets and never converts them to linear audio. It does not use the
asymmetric six-symbol amplitude mapper.

`make x2-sym-test` checks both startup roles at block sizes 1/17/160, all six
single-phase low-bit impairments, larger-error rejection and reacquisition,
all 128 capability error maps, CRC corruption rejection without output
mutation, all INFO0 role combinations, and bidirectional data transforms at
both widths with scrambling/reversal enabled and disabled. It also checks
128 original Ie030002 E97E source/history transitions (64 per role), including
their derived wire octets. Regenerate the fixtures with:

```
python3 tools/generate_x2_sym_vectors.py ../courier-emu/artifacts/x2-xc-pair-20261004 x2_sym_vectors.h
make x2-sym-test x2-test
```

These are components, not three completed engine modes. The current engine
still runs only the asymmetric digital answerer. Next requirements are the
symmetric INFO0/tone handoff and streamed capability/confirmation dialogue,
then native closed-loop payload verification. The asymmetric client still
needs its downstream training/measurement/record receiver and upstream TX
integration; the ideal codeword inverse alone does not supply those pieces.
The host's existing upstream payload qualification remains open.

The asymmetric session regression was attempted at this checkpoint and fails
its expected N=10 assertion with the pre-existing local `0x0003` upstream mask
in `x2_session_receive_mp`. That concurrent change was not altered by this
work. The mapper and new symmetric tests and `make sip_v90_modem` pass.
An additional AddressSanitizer/UBSan run did not complete and was interrupted;
no sanitizer result is claimed.

## Symmetric engine and native interoperability, 6 October 2026

Select `--mode x2-symm`, `ME_MODE=x2-symm`, or `AT+MS=X2S`. The `x2-sym`
mode and `X2SYMM` carrier are aliases. Ordinary `x2` retains its asymmetric
answerer. Symmetric mode advertises INFO0 symmetric capability, runs the
calling/answering tone exchange, then carries the digital startup,
CRC-protected capability frame and four-word confirmation dialogue on the
raw G.711 path. TX and RX enter data at their separate confirmed boundaries.
The negotiated bit rate starts the existing data stack; LAPM establishes
before CONNECT, and PTY bytes are carried in both directions.

Native Ie030002 feedback exposed two additional requirements. The source
repeats until RX releases it: answerer switches to capability TX after the
peer ramp, caller after the peer's valid capability CRC. Premature capability
TX suppresses the native caller's source entirely. Original E365..E368,
E48D..E498 and E51B..E528 establish those gates. The answerer sends FF then
four 81 words first; the caller responds only after receiving those four
(E3F9..E41B, E548..E56D). Acquisition resets the error map and phase counter
to four (E580..E588), and zero words 2/3 set error bit six (E59C). This
supersedes the phase-origin qualification in the component notes above.

The engine-to-engine test establishes 64000 and exchanges complete PTY
payloads in A-law, including the calling startup path:

```
./engine_pair_test --alaw --seconds 15 --expect X2 --expect-connect 64000 --both-env ME_MODE=x2-symm --both-env ME_DATA_FRAMING=lapm
```

The native tests use our engine as answerer and the original I-modem as
caller. Both DTEs are configured for 8N1; both must report 64000, the native
CONNECT must identify x2/LAPM, and both complete test messages must arrive.
The driver verifies the native requested Q.931 bearer: `90 90 a2` for µ-law
and `90 90 a3` for A-law. No PCM recording supplies a response, no firmware
code or DSP state is patched, and no bearer codewords are transcoded.

**Ie030002's control alphabet is PCMU even on its A-law bearer.** Changing
S58 to 52 and offering A3 does not change its V.8/Phase-2 waveform coding.
`ME_X2_CONTROL_LAW=ulaw` explicitly selects that native control alphabet for
our symmetric mode while retaining the negotiated bearer law and the opaque
64k data path. It selects signal generation and receive lookup; it never
converts an incoming or outgoing bearer octet. Other A-law peers can use the
normal A-law control alphabet by leaving this override unset. This is a
peer compatibility setting, not general A-law hardware interoperability.

`tools/probe_x2_symmetric_imodem.py` creates a derivative sealed test profile
without altering the input NVRAM. A-law selects NET3, S58=52, A3 and the native
control override. The test network primes NET3's data link with a ring and
releases that call through Q.931 before native dialling. µ-law selects
National ISDN-1, S58=48 and A2. Reproduce fresh calls with:

```
../courier-emu/.venv/bin/python tools/probe_x2_symmetric_imodem.py imodem --law ulaw --fast --instructions 150000000 --output artifacts/x2-sym-u-fresh
../courier-emu/.venv/bin/python tools/probe_x2_symmetric_imodem.py imodem --law alaw --fast --instructions 150000000 --output artifacts/x2-sym-a-fresh
```

Evidence is in `reference-data/artifacts/x2-symmetric-20261006`. `call.json` contains checked
results, environment and executable/profile hashes; raw RX/TX and both DTE
transcripts are preserved. These native runs qualify our **answering** role.
The exploratory reverse-call native test fell to V.32bis during V.8 and did
not qualify our calling role against native firmware; the engine pair passes
that role. The asymmetric client and hardware/network interoperability remain
separate unfinished work. The existing asymmetric regression's N=10 failure
with the unrelated local `0x0003` upstream mask remains unchanged.

## Analog Courier host payload investigation, 6 October 2026

The asymmetric host now has a reusable fresh-feedback known-text harness,
`tools/probe_x2_host_courier.py`, with optional read-only native mapper capture.
The short `AX2A` control is recovered completely from the waveform receiver at
4800 bit/s. A longer `COURIER-X2-HOST-0123456789` message still fails, so engine
CONNECT remains gated and no bidirectional/LAPM qualification is claimed.

`ME_X2_UPSTREAM_MAX_RATE` exposes the existing host diagnostic limit (default
4800; multiples of 2400 through 33600). The session's supported mask is again
independent of this engine policy, intersecting peer W2 and N2 per Draft 0.33
section 20. Session tests now pass their native N=10 record and separately
check a host cap and empty intersection.

Native point-to-waveform comparison follows 44841 actual mapper points with
MSE about 0.00035 in most 3200-symbol windows, with eight isolated large errors
in the complete windows. Exact native-point decoding also loses the long
message after `COURIER-X2-HOS`, independently of those waveform errors. This
narrows the next investigation to common framing/source handoff and the native
AFC9/B006-to-AF6F caller change; it does not prove a native defect, because
both experiments still use our frame decoder. Evidence and commands:
`reference-data/artifacts/x2-host-analog-20261006/README.md`.


## Analog Courier downstream mask correction, 6 October 2026

The corruption was a direction mix-up in the host three-word response, not
an error in the PCM mapper. Draft 0.33 section 20's downstream W2 must carry
our PCM rate mask (`7FFF`); we had copied the peer's upstream V.34 mask
(`03FE`). Original Courier A675..A690 intersects that word with its PCM N1
ceiling. With N1=1, `03FE` excludes its only candidate and selects index zero.
C573 then reads the word before the amplitude-bit table (`C8CF`, 51407) instead
of index one's 19 bits. `7FFF` selects index one, B=19 and MD=5. Symmetric x2
uses a different capability exchange and does not exercise this selection.

The corrected response for the standalone N=10 fixture is
`D284/7FFF/0000`, CRC `FB0C`. Fresh native 403 calls with the diagnostic
`&U26&N39` clamp recover both `HOST-X2-0123456789-ABCDEFGHIJKLMNOPQRSTUVWXYZ`
and `COURIER-X2-0123456789-ABCDEFGHIJKLMNOPQRSTUVWXYZ` completely. The
transmitted profile carries 32000 bit/s downstream and 4800 upstream, despite
the native nominal `CONNECT 53333` label. The engine reports its actual
32000 bit/s downstream rate.

`tools/probe_x2_host_courier.py --pty-source` exercises the normal PTY source
and sink after an actual engine CONNECT. The engine's x2 receive branch now
flushes the upstream byte ring to that sink, just as the other data paths do.
The gate requires accepted MP/E, PAYLOAD stage and acquired upstream B1.
This does not qualify LAPM, higher upstream rates, or later native rate changes;
the long capture still becomes noisy after the initial complete message.

Evidence: `reference-data/artifacts/x2-downstream-20261006/`, with fresh calls in the sibling
`x2-downstream-20261006-mask-fixed`, `-long-fixed` and `-pty-fixed` directories.

The unclamped Courier control (`-pty-auto`) also receives the complete host
message, confirming the downstream correction beyond the fixed-rate fixture.
Its upstream long message has a localized corruption around `IJK`, with the
following suffix intact; both waveform decoding and the PTY show it. Upstream
reliability therefore remains open even at the selected 4800 bit/s. B1 fits
100% in that call, so acquisition alone is not sufficient evidence of an
error-free data connection.

## x2 V.34 B1 handoff and busy-line phase correction, 6 October 2026

The 4800 upstream corruption was partly our decoder state. The accepted
reset-state B1 now retains its natural Table-12 epoch into DATA instead of
parking it back at j-1, and h=0 trellis parity is enabled. The output gate
accounts for the 15-pair traceback delay. The subsequent frame-phase search
also treated an idle-to-busy DTE transition as a lost epoch and corrupted a
long transfer. x2 retains B1's authoritative epoch instead of sweeping from
user content. V.34 9.6.3, 10.1.3.1 and 11.4.1.1.4/5 govern these changes.

All 3500 bytes of a fresh native Courier source now reach the real engine
PTY; replays at 17/80/160 samples also recover the complete source. The
940-byte downstream stress is still incomplete at the native serial
interface. (since shown to be a Courier DTE overrun in the test, not a modem fault; see
`docs/x2_implementation.md`, 10 October 2026) The short downstream message remains the bidirectional control.
See `docs/x2_v34_upstream_review.md` for evidence, sampling decisions and
remaining echo/recovery qualification. The upstream cap stays at 4800 until
higher rates pass foreign payload tests.


## Downstream "stress" and lost upstream opening bytes, 10 October 2026

Two open x2 results turned out not to be x2 defects.

**The 940-byte downstream burst was a DTE overrun in the test, not a
modem fault.** Decoding our own transmit tap
(`reference-data/artifacts/x2-rx-improve-20261006-long-fixed/engine-tx.g711`, inverse mapper
from tx sample 111005, GPC descrambler, 8N1) recovers all 940 bytes exactly.
The call has no error control (`CONNECT 53333/x2/NONE`), so nothing can hold
off the far end. It sends 32000 bit/s into a Courier whose emulated DTE is
paced at the firmware-programmed serial rate. The Courier output shows a
clean 42-byte omission and then a missing tail, which is an overrun pattern,
not line errors. A/B with `tools/probe_x2_host_courier.py --host-pace-samples`:
an unpaced burst loses data again (454 bytes reach the Courier DTE), while
one byte per 16 samples delivers all 940
(`reference-data/artifacts/x2-ds-pace-20261010-{burst,paced}`).

**The upstream opening message was being discarded by V.42 detection.**
Since `ef7b5464` the factory `+ES` attempts V.42 detection on every call. The
Courier sends its first line inside the answerer's T400, and the detection
phase dropped those characters on fallback (V.42 Appendix I.3 option a).
Both arms above lost `COURIER-X2-HOST-0123456789`, with the receiver at 0.000
symbol error. `v90_engine_replay` of the paced tap reproduces this. With
`ME_DATA_FRAMING=v14`, or with the data-stack fix (Appendix I.3 option b,
forward the buffered characters after the fallback event), the message
reaches the PTY right after `CONNECT 32000`. This affected every
non-error-correcting peer that talks first, not just x2.

The line noise after about 18 s in these captures is the Courier's later
upstream collapse (the known MP/data discontinuity), and a replay cannot
judge it.

A fresh native call with the fix and the paced 940-byte source passes all
seven checks, with both directions complete through the real PTYs
(`reference-data/artifacts/x2-ds-pace-20261010-fixed`).

## Upstream above 4800, 10 October 2026

The 4800 default cap is gone. The engine offers every V.34 upstream rate and
the peer's W2 selects. The cap stood in for a peer misconfiguration: the
emulated Courier's NVRAM holds S39 (the V.34 transmit level index) at 0, an
index its own parser rejects. At its factory S39=8, every rate it offers
(7200..24000) carries a 3500-byte source exactly. The trace and the
qualification table are in `docs/x2_v34_upstream_review.md`.

## "Does the data work": the 13-second collapse, 10 October 2026

The harness used to pass a call whose PTY held the source *plus 71 KB of
noise*. It now also requires exact delivery after the CONNECT line in both
directions (`native_to_host_exact`, `host_to_native_exact`). Those checks
exposed three things:

- **The collapse is the emulated Courier's DSP stalling, not a protocol
  event.** From exactly 46.92 s after emulator start, in every run (every
  rate, the 6 October runs, every engine build), the Courier's transmit audio
  repeats one 40-sample block exactly: the 1800/2400 Hz lines are its 200 Hz
  harmonics. Its DSP sits in the `IDLE` at 80FA, waiting for a serial-port
  interrupt that never comes (the idle itself is the normal empty-ring wait).
  Its data stream to the supervisor stops at the same point. Nothing on the
  wire asks for a retrain or renegotiation, so there is nothing for our side
  to answer. The cause is inside courier-emu and still open. Calls are clean
  up to that point (13 s of data at 24000 here).
- **Squelch.** x2 runs without error control, so a receiver that has stopped
  decoding handed noise straight to the DTE. The x2 path now stops delivering
  when the T/3 receiver's fast decision error exceeds 4x the level it settled
  at after B1 (floor 0.08). It resumes only after one second of decisions
  below 2x, and the V.14 assembler then re-hunts for a start bit. New
  accessor: `v34_v90_upstream_rx_error()`.
- **Stray leading byte downstream.** Detection's T400 expires inside
  `v42_tx_bit()`, and the bit returned from that call came from the abandoned
  entity's freshly reset HDLC transmitter: a lone 0. The Courier read it as a
  start bit and printed `0xFF` before the first character. The answerer now
  sends mark for that bit (V.42 7.2.1.3), and `data_stack_test` checks that
  it sends marks only.

Fresh call against committed courier-emu, with both directions exact:
`reference-data/artifacts/x2-strict-20261010`.

## In-data fall forward, 10 October 2026

**Correction to the section above: the "13-second collapse" is not a DSP
stall.** The Courier trains at PCM index 1 (B=19, 32000 bit/s; "53333" is its
display label). Its in-data rate check (DSP report 0020, then supervisor
0054) then asks to fall forward, here to index 14. The original server
(Ie030002, an I-modem pair in courier-emu) completes the change in about
0.8 s. We did not answer, so the Courier fell back to V.90 9.5.2.1's retrain
(70 ms silence, then 2400 Hz Tone A with the 1800 Hz guard), which is the
1800/2400 Hz signal. Answering that Tone A with Tone B does make the Courier
reverse it, but the retrain is only the fallback.

The exchange, decoded from the I-modem pair and confirmed against our own
calls. It has V.90 9.6.2.1's shape, with x2 signals in place of Rd/CP:

| direction | signal |
|---|---|
| Courier -> server | V.34 S for 128T (40 ms; lines at 320/1920/3520 Hz), S-bar, then 4-point MP `0378 1ffe 0000 0500` (N1=14, N2=13, W2=1ffe) and `8378` (ACK), then E and B1 at the new upstream rate |
| server -> Courier | on the six-sample grid, our `RECORD_ALIGN` sign pattern (Courier f7d8..f854 matches the signs `0x71c7` four times, then `0x71f8`, within 1.5 s), then the three-word record **in the current data banks**: `733c 7fff 0000` x10, then `f33c` (ACK) once the Courier's MP is in. Then the 6 final samples and 4080 `DATA_STARTUP` samples in the new bank |

The 4-point MP is two bits per symbol, differential with the same clockwise
quadrant convention as the 16-point startup MP, LSB first, through GPA.
`x2_mp_rx` now has a `four_point` mode. It decodes the I-modem pair's
`0378`/`8378` and ours from raw audio, which the 16-point demodulator could
not.

Implemented: `x2_session.c` detects S then S-bar (or the end of S) in
PAYLOAD, sends `RECORD_ALIGN` and the record on a frame boundary (no source bit
is lost), ACKs after the Courier's MP, and on its E runs
FINAL_TRAINING/DATA_STARTUP into the bank the MP selects. The engine re-arms
the T/3 upstream (`v34_v90_upstream_rearm()`), so the next prepare and E
acquire a fresh B1 at the new rate, and it resets the squelch baseline.

Live (committed courier-emu, `reference-data/artifacts/x2-ff3-20261010`):
`X2 rate change complete: index=14 downstream=54666 upstream=31200`. B1 is
re-acquired at 31200 with 100% fit. The call runs on for more than 150 s, and
the Courier starts a second rate change at 182 s. **Upstream carries 498 of 500
lines across the change**: one line is lost in the switch (no error control)
and the last is in flight at hangup.

**Open: downstream at index 14 is corrupted at the Courier**: most lines
have short error bursts. Things ruled out:
- Our index-14 banks: they reproduce the I-modem's idle index-14 stream
  exactly (100% ones after descrambling).
- Our own transmit tap: it carries the message intact.
- DC: the I-modem's downstream has the same +7.6 mean.
- Upstream rate: capping it at 28800, as the I-modem had, changes nothing.
- Declining: the Courier picks the index itself, and the record's ceiling
  field does not limit it.

**Resolved as an emulator limitation, not ours.** An I-modem pair without error
control (courier-emu HEAD, `&M0` on both ends, a numbered line every second)
falls forward the same way, from index 8 to 14. The Courier then corrupts the
original server's index-14 downstream identically:
- lines sent at index 8 are intact;
- every line after the change has short error bursts, and then the DTE fills
  with garbage.

The errors are systematic: the same plaintext turns into the same garbage in
different lines. That points at a level- or bandwidth-dependent fault in
courier-emu's emulated receive path (8 to 9.6 kHz reconstruction into the
AC01 model, band-edge SNR, codec input gain or clipping), consistent with
courier-emu's own note that 56000 was never reached with the I-modem.
Evidence: courier-emu worktree `admiring-ramanujan-9d7101`,
`artifacts/x2-47s-fall-forward/imodem-pair-no-ec/`. Our downstream at
index 14 behaves exactly like the original server's, so qualifying x2 above
index 1 needs that emulator fix or real hardware.

## Fresh Courier emulator controls, 12 October 2026

Current engine `ac0b1fe5` was rebuilt and tested against committed courier-emu
`b73483f3aa2986e54f193496490a6543bc9b3998`, exported to an isolated temporary
source tree with its own native-library build. The sibling checkout has a
modified `native/c5x_core.cpp`; that modification was excluded. Firmware and
fixtures came from the committed export. No native runtime patches or capture
hooks were enabled. The harness used its factory `S39=8` setting, PCMU,
`--fast --pty-source --host-pace-samples 16`, and 180 million instructions.

Both calls pass all nine checks, including exact delivery after CONNECT in
both real PTYs (no extra data):

| call | Courier -> host | host -> Courier | evidence |
|---|---:|---:|---|
| short control | 28 bytes | 20 bytes | `reference-data/artifacts/x2-codex-20261012-control-socket/` |
| numbered lines | 3710 bytes / 70 lines | 1000 bytes / 20 lines | `reference-data/artifacts/x2-codex-20261012-long/` |

Each run clocks exactly 260000 received and transmitted bearer samples
(32.5 seconds). Initial selection is index 1, 32000 downstream / 24000
upstream. Both also complete the Draft 0.33 section 22 / V.90 9.6.2.1-shaped
fall-forward to index 14, 54666 downstream / 31200 upstream, and reacquire
upstream B1 with 100% fit. Payload injection finishes before the rate change;
these runs qualify initial exact transfer and the observed rate handoff,
not payload delivery after that handoff or long-term reliability.

`make x2-test x2-session-test x2-sym-test` passes. The initial sandboxed
attempt could not bind the local Unix socket and was interrupted before a
call; its directory `reference-data/artifacts/x2-codex-20261012-control/` is not evidence
of an interop failure. The completed tests used permission to open the local
socket and PTY.

## Payload after fall-forward, 12 October 2026

The same isolated committed emulator was tested with 20 numbered host lines
(1080 bytes), paced one byte per 16 bearer samples, injected at sample 220000
(27.5 seconds). The harness now offers `--host-after-rate-change`, requiring
the engine's completed-rate-change trace before injection, and records the
first source sample and that gate in `call.json`. Pacing starts when the gate
actually releases, avoiding a backlog burst if the change occurs late.

`reference-data/artifacts/x2-codex-20261012-after-change/` completes the index-14 handoff
(54666 downstream / 31200 upstream) and B1 reacquisition with 100% fit.
All 1080 source bytes are queued after the change. Initial Courier -> host
payload is exact, but host -> Courier complete/exact checks FAIL: the native
DTE contains localized corruption from the first line onward. This run does
not test upstream source injection after the change.

Our transmit tap was checked first, using `x2_pcm_decode` with the negotiated
index-14 banks, continuous GPC descrambling and 8N1 assembly. Of the six
candidate alignments starting at samples 219000..219005, sample 219005 decodes
6832 frames with zero invalid frames and recovers the entire 1080-byte source
exactly. Results are in `tx-tap-check.json` and `tx-tap-decoded.bin`. Thus the
payload is intact at our raw-G.711 transmit boundary, while the emulated
Courier fails to recover it. This reproduces the previously documented
index-14 receive limitation; it does not independently identify its cause.
No modem DSP constants or bearer handling were changed.

Validation: the fresh closed-loop test exercises the new injection gate and
pacing; Python compilation passes. The expected interop failure is retained
as a failed `call.json`, not counted as a passing payload qualification.

## Where the index-14 payload is lost, 12 October 2026

`tools/analyze_x2_courier_output.py` reads a PC-`832a` capture
(`X2_HOST_CAPTURE_PC=832a`, `--capture-receiver`, with B=36/MD=5 selecting
index 14 per Draft 0.33 sections 20 and 22). At that Courier 403 ROM
instruction ACC is the output-bank ready bit and data 0320 holds the next
descrambled word; ACC=0 returns without publishing it.

On `reference-data/artifacts/x2-codex-20261012-output-capture/` (the 1080-byte after-change
payload):

| stage | result |
|---|---|
| words reaching 832a at index 14 | 35306 |
| 8N1 decode of those words | all 1080 source bytes, exact |
| words dropped (bank not ready) | 2948 (8.3%) |
| 8N1 decode of the published words only | 1022 bytes, source NOT found |

So the Courier's own receive DSP demodulates our index-14 downstream
correctly; the corruption seen at its DTE is introduced after demodulation,
where the DSP hands a word to a supervisor that has not yet freed the output
bank. That is courier-emu service/scheduling timing (the earlier batch32,
service64, cpu1 and cpu3 artifacts probed the same area), not our transmit
path, which was already verified at our raw G.711 boundary. Qualifying x2
above index 1 therefore needs the emulator's DSP/supervisor pacing fixed or
real hardware. No modem code changed.

The capture tools now take `X2_HOST_EMU_ROOT` (emulator checkout, so a
committed export can be used instead of the sibling working tree) and
`X2_HOST_CAPTURE_PC`, and hook `step`, `step_cycles` and `service_step`
because the modern bearer advances by cycles.

## LAPM, rate caps and renegotiating from above index 1, 19 October 2026

Tested against a clean export of courier-emu `f1ccd92` (which includes the
C5x XC/RPTB fix, `186d653`), with the harness defaults (`S39=8`, PCMU,
`--fast --pty-source --host-pace-samples 16`) and the 1080-byte numbered
host payload from the 12 October after-change run.

**The index-14 corruption is not a supervisor-speed limit.** Lowering the
80186's cycles per instruction from 5.93 to 4.5 and 3.5 (a supervisor up to
70% faster, every real-time clock unchanged) leaves the same corruption.
courier-emu's `--frame-hz` cannot be used to test the INT0 rate: any value
other than 2401 stops the firmware booting.

**x2 with V.42 LAPM works.** Every earlier qualification forced the Courier to
`&M0`. With `--courier-at 'S39=8&M5&K0'` the call reports
`CONNECT 53333/ARQ/x2/LAPM`. V.42 detection, XID and LAPM come up, and the
short exchange passes all checks exactly in both directions.

**`AT+MS` now bounds x2 in both directions** (V.250 6.4.1). Until now
`x2_session` ignored the limits that already pruned the offer: the
downstream mask was fixed at `7FFF` and the upstream at `3FFF`.
`me_x2_start_locked()` now derives both masks from the call's limits.
- **Downstream (TX):** index N runs at (B+MD)·8000/6 bit/s
  (`x2_short_record_rate`): index 1 is 32000, index 14 is 54666.
- **Upstream (RX):** mask bit N-1 is N·2400 bit/s.
- If no rate survives, the call is refused.

The Courier selects under the record's W2 mask, both at startup and at every
fall forward. Capped at 44000, 48000 and 52000, it fell forward to exactly
index 6, 9 and 12. This corrects "the record's ceiling field does not limit
it" above, which tried the ceiling field and not W2.

**Renegotiating from above index 1 (fixed here).** When capped below the
index it measures, the Courier asks to fall forward again about 1.1 s after
the first change completes. We answered with the RECORD_ALIGN sign pattern
on a fixed `A5`, and with the record in the current data banks. Above index 1
the Courier did not accept this:
- it sent its MP without the ACK bit (`0378`, where the first change has
  `8378`);
- it then fell back to Tone A exactly 1.5 s after its S;
- we stayed in RECORD_TX for the rest of the call.

Our TX tap had both signals correct. The difference is the banks:
- **Alignment.** Every training bank and index-1 bank starts with `A5`, but
  index 6's position 0 starts with `A3`. With the alignment on each
  position's lowest-level code of the current banks, the Courier detects it
  and keeps repeating its MP.
- **Record.** It still never ACKed a record in the index-6 banks or in the
  training banks. It does ACK one in the index-1 (B=19) allocation at the
  current MD.

Experiment arms: `x2-20261019-rec1-cap44000` (record in index-1 banks, `A5`
alignment: no ACK), `-align0-44000` (lowest-code alignment, index-6 record:
the MP repeats for 2.7 s with no ACK), `-rb-training` (training-bank record:
the first change fails too) and `-rb-index1` (both changes: completes).
`-mptrace44000` logs every MP accepted before the fix.

With both changes, the second change completes at every cap (36000 to 52000,
two requests and two completions each), and the Courier stops asking. The
reference I-modem renegotiation (pair capture at 25.2 s) starts from index 1,
where both rules give the same signal as before. So that capture cannot tell
"index-1 banks" apart from "the call's startup banks". I found no
in-data renegotiation clause in Draft 0.33; this rests on the Courier's behaviour.
`x2_session_test` covers the index-6 case.

**Where data now holds**, after both changes
(`reference-data/artifacts/x2-20261019-*`):

| AT+MS max TX | final index | `&M0` (no EC) | `&M5` LAPM, 32.5 s run |
|---:|---:|---|---|
| 32000 | 1 | exact | exact |
| 36000 | 2 | 17/20 lines | exact |
| 40000 | 3 | 17/20 lines | exact |
| 44000 | 6 | 6/20 lines | 611/1080 bytes exact, still retransmitting |
| 48000 | 9 | 6/20 lines | 621/1080 bytes exact, still retransmitting |
| 52000 | 12 | 1/20 lines | stalls at 21 bytes |
| none | 14 | 0/20 lines | stalls at 81 bytes |

A longer LAPM soak at the 40000 cap delivered **15750 of 15750 bytes exactly**
(250 numbered lines) over 132 s of line time (1,060,000 bearer samples). It ran
at index 3 (40000 down, 31200 up) through both rate changes, and every check
passed.

Above index 1 our TX tap decodes byte-exact at every index tried: 6 and 12
here, and 14 on 12 October. The corruption grows with the rate and is on the
emulated Courier's receive side. courier-emu's original analog Courier
against its original I-modem also fails a sustained x2 traffic test
(`courier-emu/docs/imodem-analog-x2-traffic-20261018.md`, uncommitted
there). Until that is fixed, or real hardware is available,
`AT+MS=X2,0,0,40000` with LAPM is the highest rate here that delivers exactly.

One fault remained ours: if the Courier fell back to Tone A during a rate
change, we waited in RECORD_TX until the call ended. That is now bounded; see
"Bounding an unanswered rate change" below.

Reproduce (clean courier-emu export in `$EMU`):

```
X2_HOST_EMU_ROOT=$EMU ../courier-emu/.venv/bin/python tools/probe_x2_host_courier.py \
  --courier-at 'S39=8&M5&K0' --engine-at 'AT+MS=X2,0,0,40000' \
  --output artifacts/x2-arq-40000 --fast --pty-source --host-pace-samples 16 \
  --host-at 220000 --host-message-file reference-data/artifacts/x2-codex-20261012-after-change/host-source.bin
```

`--engine-at` sends DTE commands to our PTY before the call starts.

## Bounding an unanswered rate change, 19 October 2026

Only the Courier's E released `X2_RECORD_TX`. When the Courier gave up on a
rate change, nothing ended the wait. The generic 10 s stage watchdog never
fired: the stuck change started 24.5 s into a 32.5 s run.

**What the Courier does.** Reproduced by forcing `align_code()` back to a fixed
`A5` with `AT+MS=X2,0,0,44000` and `S39=8&M5&K0`, against a clean export of
courier-emu `f1ccd92`. The first change, from index 1, completes. The second,
from index 6, is not answered in a form the Courier accepts:
- the Courier sends its MP `0378` without the ACK bit (we accept it);
- it sends no E;
- **1.57 s after the end of its S**, it falls back to V.90 9.5.2.1's retrain:
  70 ms of silence (tap 248920..249480), then 2400 Hz Tone A with the 1800 Hz
  guard. Over a 40-sample window, 0.80 of the power is at 2400 Hz and 0.20 at
  1800 Hz, with an RMS of 1175.

It repeats nothing before the retrain: one S, one MP, then Tone A.

A trap when reading these runs: `engine-rx.g711` starts before x2 does.
x2's `rx_samples` is about 41440 samples behind the tap index, not the
~16000 a first guess gives. The second change's S is at tap 236960 (x2 rx
195840). The S at tap 211920 is the first change, which completed.

**What we do now** (`x2_session.c`):
- **Tone A.** In PAYLOAD, and during a rate change up to RECORD_TX, the 40-sample
  upstream window that finds S also measures 2400 Hz. 64 consecutive 5-sample
  steps (40 ms) with at least 0.6 of the power at 2400 Hz and RMS of at least
  250 end the session as `X2_FAILURE_PEER_RETRAIN`.
- **Deadline.** With no E 24000 samples (3 s) after the end of the Courier's S,
  the session ends as `X2_FAILURE_RENEG_TIMEOUT`. A completed change reaches E
  0.25 s after S.
- **Stage watchdog.** The old 10 s watchdog now records
  `X2_FAILURE_STAGE_TIMEOUT`.
- **Engine.** The engine logs
  `X2 failed: <cause> in <stage> (renegotiation N, M samples after its S, MP .., E ..)`.
  It sets the hang-up cause: Remote for Tone A, Modem for the deadline. It hangs
  up, so the DTE gets `NO CARRIER`.

**False-positive check.** The same detector, run offline over every retained
x2 `engine-rx.g711`, fires only where the Courier really sent Tone A: every
stuck run of 6, 10 and 19 Oct. On calls without Tone A, the longest qualifying
run is 3 steps, against the 64 needed. That includes the 132 s LAPM soak,
`ff3` (171 s) and all `fix-`/`arq-` runs.

**Evidence** (`reference-data/artifacts/`):
- `x2-20261019-tonea-repro-a5`: the forced-`A5` repro. It logs
  `X2 failed: peer retrain (Tone A) in RECORD_TX (renegotiation 2, 12651
  samples after its S, MP accepted, E none)`, about 50 ms after Tone A
  starts. The DTE gets `CONNECT 32000`, the Courier's line, then
  `NO CARRIER`.
- `x2-20261019-tonea-control-44000`: the same cap with the real `align_code()`.
  Both changes complete (index 6, 44000/31200), with no false trip, and all
  nine checks pass, including exact host-to-Courier delivery over this longer
  (220M-instruction) run.

**Why we hang up instead of answering the retrain.** (Superseded 30 October:
the server now answers it; see "The server's retrain" at the end.) Answering Tone A with
Tone B does make the Courier reverse it (10 October). After that comes a full
Phase 2 restart, with the Courier in the Tone A role, which is the reverse of
the startup roles our asymmetric session implements. We would then need the
probe, marker, J, training, MP and record again, plus a fresh upstream
receiver. Nothing shows how the Courier continues after the reversal, and with
the alignment fix it no longer abandons rate changes on its own. A clean,
logged hang-up is the bounded answer until a peer is seen to retrain for
another reason. `x2_session_test` covers the deadline, Tone A in RECORD_TX and
in PAYLOAD, S during RECORD_TX, and quiet or guard-only tones that must not
trip it.

## The analogue client, 19 October 2026

We can now be the other end: the analogue x2 client that dials a server, as
the Courier 403 does. `AT+MS=X2` (or `--mode x2`) on a call we place selects
it; answering still selects the server. PCMU only, like the server.

**What it sends**, from the Courier's own line output (Draft 0.33 3.3-3.4 and
`../courier-emu/docs/x2-v90-protocol-selection.md`):

- V.8 as caller, with the x2 carrier-phase signature, and a CM whose
  GSTN-access octet is present but empty (Courier `e0 c1 45 10 10 0d`; the
  server's JM sends `8d`). Ie030002's classifier needs that octet as well
  as the inversions: without it its `de14` ACC read -132648 (rejected),
  with it +205351.
- INFO0 `21ff` on 1200 Hz: high carrier, asymmetry and CME clear, so the
  server's gate (clause 12.2) takes the marker path. Tone B with the caller's
  reversal (V.34 11.2.1.1), shared with the symmetric caller in
  `x2_session.c`. A real server sends Tone A with the 1800 Hz guard tone
  6 dB down (V.34 10.1.2.1; our server does not), so the Tone A detector
  counts the guard toward the tone's purity.
- Silence through the server's 160 ms probe, then the marker `4d` (two lead-in
  ones and the 39-bit frame) at 600 bit/s on 1200 Hz.
- 60 ms later, V.34 at 3200 baud on the high carrier, through SpanDSP's
  modulator fed by `x2_client_get_symbol()`: S 128T, S-bar 16T, PP, 6400T of
  4-point TRN, 440T of 16-point J (`0x89b0`, V.34 Table 18), then silence.
- On the server's J word, S, S-bar, 16-point TRN and the four-word MP
  (clause 21): W1 = N1<<2 | N2<<6 with N1 our highest PCM index and N2 our
  highest V.34 N, W2 the V.34 mask, W4 = MD<<8 (MD 5, as the Courier).
  N1 is not forced to 1 the way the Courier forces it: our receiver is exact,
  so we ask for the top of the AT+MS RX range at once.
- Once the server's record has arrived, two MPs with the ACK bit, E, then
  B1 and V.34 data via `v34_v90_arm_tx_data()`. The rate is the highest N
  in our W2 under both the record's V.34 ceiling (bits 6..9) and our N2.
  The record's flags choose the encoder: bit 14 expanded shaping, bit 12
  the 64-state trellis, bit 13 the non-linear encoder.

**What it receives** (`x2_client_rx_codewords()`): the bearer delivers the
server's octets, so the downstream is decoded in the codeword domain with no
equalizer. Stretch B (`b1`/`31`) is descrambled with GPC until the 12-bit
script word `09b0` repeats. Through E training all six frame phases are
decoded; the phase whose descrambled stream holds two identical CRC-valid
three-word records (training banks, B=19, MD=5) carries the record. Our own
server's 54-octet RECORD_ALIGN word also sets the phase directly. Both
record fields are ceilings: the downstream index is the highest the record's
PCM mask allows under its index field and our N1. After our E the server
finishes the record, sends one frame of ones on a record boundary (96-bit
records start on frame boundaries) and switches to the data banks on the next
frame, keeping parity and scrambler history; from there every frame goes to
the data stack.

**Results.** `x2_client_test` runs our server's octets into the client at
indices 1, 6, 12 and 15 and recovers the server's payload bit-exact after its
DATA_STARTUP ones, and puts the client's MP and E through SpanDSP's modulator
into the server's MP receiver. `engine_pair_test --both-at AT+MS=X2` connects
whole engines in about 15 s: CONNECT 56000 both ends, 31200 upstream, DTE text
both ways, and a 60 s hold. AT+MS caps hold in both directions (client RX
40000 -> 40000; server TX 44000 -> 44000; client TX 9600 -> upstream 9600 with
56000 down). These are in the fast tier.

**Two server-side faults the client exposed**, both in `x2_mp_rx.c`, so both
also affect calls from a real Courier:

- *The carrier phase estimate flipped by 90 degrees.* The MP receiver
  derotates by atan2 of the fourth-power mean over 4. For the V.34 16-point
  grid that mean sits at 180 degrees when the carrier offset is zero, so noise
  flips the estimate between +45 and -45 degrees, and every flip is a
  differential symbol error on every hypothesis at once. It is now unwrapped
  per timing hypothesis; valid frames over the same signal rose about 15%.
- *E was declared early.* Every MP starts 17 ones, a zero, then W1's two low
  zeros, so one symbol error on bits 16..19 reads as E's 20 ones. With W1
  `034c` (N1=3) a timing hypothesis rejecting 44% of its frames declared E
  300 ms before the client sent it, and the call failed. E is now taken only
  from the hypothesis with the longest unbroken run of valid MPs. Requiring an
  acknowledged MP first is not an option: the captured Courier's E follows
  `0344`, its ACK set while the last frame was already on the line.

**AT+MS direction.** The offer pruning assumed our TX is the PCM direction
unless we are the V.90 analogue role, so an x2 client capped at 9600 TX
dropped x2 from its V.8 offer. Asymmetric x2 now uses the call direction:
the dialling end receives the PCM (`me_offer_pcm_received()`).

### The analogue client against Ie030002

`tools/probe_x2_client_imodem.py` runs our client (v90_engine_peer `--call`)
against the original Ie030002 I-modem firmware answering in courier-emu
(S58=48: x2, server and symmetric on, V.90 off), byte-exact PCMU, nothing
recorded on either side. With `ME_ANS_NOTCH_RATIO=3` and `AT\N0`:

- the I-modem prints **CONNECT 57333/x2/NONE** (its display label for index
  15) and we print CONNECT 56000 (index 15 at B=37, MD=5);
- its line `IMODEM-X2-SERVER-0123456789` arrives at our DTE exactly (the high
  bits are its DTE's parity on 7-bit characters), and ours,
  `OUR-X2-CLIENT-0123456789!`, arrives at its DTE exactly, six copies;
- upstream 31200 (N=13) by default, and 24000 when AT+MS caps our TX there.

Evidence: `reference-data/artifacts/x2-client-imodem-20261019` (both DTE
transcripts, our engine log, the I-modem result). Each of these had to change
before it worked, and each is something our own server had not exercised:

1. **ANSam.** Our caller never recognised the I-modem's ANSam: until its echo
   canceller settles (about 2 s) its output carries copies of our CI at
   -30 dB, and SpanDSP's purity test (notch energy under 1/6 of the channel)
   fails until 4.1 s, leaving two good cycles of the three it needs before
   the I-modem gives up at 5.3 s. `ME_ANS_NOTCH_RATIO=3` detects it at 3.8 s.
   The default is unchanged here because it is shared by every peer.
2. **The CM access octet**, above.
3. **The guard tone**, above.
4. **No alignment word.** The I-modem accepted our MP 425 times (CRC at its
   A9DF) and sent its record from 13.61 s, but in place, on the E-training
   mapper's frame phase with no 54-octet alignment word. Our server's
   RECORD_ALIGN is not what Ie030002 does at startup.
5. **Ceilings, not choices.** The record was `F37C/7FFF/0000` whatever we
   asked for, N1=1 included.
6. **The encoder flags.** Linear at 31200 printed garbage at the I-modem's
   DTE (its decoder ran on our idle marks); the non-linear encoder fixed it.

**LAPM (20 October 2026).** With V.42 left on, both ends now print CONNECT
with LAPM: the I-modem `CONNECT 57333/ARQ/x2/LAPM/V42BIS`, ours (`AT&A3`)
`CONNECT 56000/ARQ/X2/LAPM/V42BIS`. Both lines arrive exactly, ten copies of
ours, and V.42bis P0=3 P1=1024 P2=32 is negotiated. Evidence:
`reference-data/artifacts/x2-client-imodem-lapm-20261020`.

The cause was **the width of our XID's HDLC optional-functions parameter**,
not the emulator and not the I-modem's settings. The data link was fine before
the change. Decoding both bit streams (`DS_TX_BIT_DUMP`/`DS_RX_BIT_DUMP`) showed:

- our ODP was correct;
- the I-modem answered with ADP (`E`,`C` alternating, V.42 8.10.4) and then
  flags, and switched its DSP to HDLC (mailbox `000a:0003`);
- all 20 of our XID commands were CRC-valid and got **no response at all**;
- our SABME got UA, and our I-frames were acknowledged (RR, N(R) rising).
  But the I-modem never printed CONNECT and never passed a byte to its DTE.
  Its supervisor will not go to data without a completed XID exchange.

Our XID uses V.42 (03/2002) Table 11a's four-octet optional-functions field
(`03 04 8a 89 00 00`, GL 20). The Courier's XID, which this I-modem does
answer (`x2-20261019-arq-short`, upstream), uses three (`03 03 8e 89 00`,
GL 19). Ie030002 silently discards the four-octet form. A one-variable pair
on the same build:

| our XID | V.42bis offered | I-modem |
|---|---|---|
| 4 octets | yes | no XID response ×20, no CONNECT |
| 4 octets | no (`AT+DS=0`) | no XID response, no CONNECT |
| 3 octets (`ME_LAPM_XID_OPTION_OCTETS=3`) | yes | XID response to the first command, CONNECT ARQ/LAPM/V42BIS |

The RasFinder compatibility (`docs/rasfinder_v42bis_interop.md`) learns the
width from a three-octet *reply*. This peer sends no reply to learn from.
In automatic mode, `v42.c` now alternates 4 and 3 on each T401 retransmission
of an unanswered initial XID (V.42 8.10.2). A modern peer that only lost a
frame still gets four on every other attempt. With the defaults, the I-modem
answers the second command, after one T401. `v42_link_test` has a peer that
drops four-octet commands; without the change that test fails.
`ME_ANS_NOTCH_RATIO=3` is no longer needed (next paragraph).

**ANSam without a knob (28 October 2026).** Measured on our receive tap, the
I-modem's ANSam is 89-94% of the received power at 2100 Hz from 2.0 to
4.0 s. The rest is spread across about 1050, 1670-1860, 2350-2480 and
2900 Hz, and it is there in the gaps of our CI too. It clears (100%) only
from about 4.5 s, and the I-modem gives up at 5.3 s. The detector compares
mean absolute amplitudes, so 10% of the power off-tone puts the notch output
at about a third of the channel; the default acceptance is a sixth. A ratio
of 4 still missed it (no CONNECT) and 3 connects. courier-emu's hybrid model
is on the Courier's analogue DSP only, so this is the firmware's own output.

V.8's ANSam detector now has its own ratio of 3
(`modem_connect_tones_rx_set_notch_ratio()`). Every other 2100 Hz detector,
fax CED included, keeps 6. A calling DCE that takes ANSam sends CM and waits
for JM (V.8 8.1), so a false ANSam costs a CM, while a missed one costs the
call; on a clean line purity is ~1.0 and nothing changes.
`ME_ANS_NOTCH_RATIO` still overrides every detector. With no environment:
CONNECT 57333/x2/NONE against 56000 with `\N0`, and
57333/ARQ/x2/LAPM/V42BIS against 56000/ARQ/X2/LAPM/V42BIS by default, both
lines exact. The `\N0` probe is in `tests/slow.list` (141 s; needs
`../courier-emu`).

**Not done:**

- ~~**Fall forward and retrain.**~~ The client now opens in-data rate changes
  and answers a server's retrain, which is Tone B (next sections).
- **Bearer.** There is no analogue front end, so the client needs a
  byte-exact bearer.
- **Real hardware.** No real server has been tried.

## The client's in-data rate change, and Tone A (28 October 2026)

`x2_client_start_rate_change()` opens a rate change the way the Courier
falls forward ("In-data fall forward" above), with V.90 9.6.2.1's shape:
- **Upstream:** V.34 data hands the modulator back at the current symbol
  (`v34_v90_resume_external_symbols()`, which keeps carrier and baud
  phase). Then S 128T, S-bar 16T, and the 4-point MP with the new N1,
  repeated. Once the server's record carries bit 15, two acknowledged MPs
  follow, then E, and V.34 B1 and data at the record's upstream N.
- **Downstream:** while the change is open, the newest nine data frames
  are held back. The server's 54-octet alignment word starts on a frame
  boundary, so when it is recognised it is exactly those frames, and none
  of it reaches the DTE. The record is decoded on a fresh mapper in both
  candidate allocations: index 1 at the current MD (our server) and the
  current banks. Then come one frame of ones on a record boundary and the
  new banks. The server's 680 DATA_STARTUP frames are dropped, not
  delivered: it stopped taking source bits mid-character, and the ones
  would break a V.14 character.
- **Guards:** a change can open only 4080 codewords after the final frame
  (`x2_client_can_change_rate()`). The server listens for S only in
  PAYLOAD, and an S sent during its DATA_STARTUP was simply ignored. The
  change fails after 1.5 s without the alignment word, or 3 s without the
  final frame, as `X2_FAILURE_RENEG_TIMEOUT`.
- **Test hook:** `ME_X2_CLIENT_RENEG_AFTER_MS` with
  `ME_X2_CLIENT_RENEG_INDEX`. Nothing else opens one: the client asks for
  its ceiling at startup, and on a byte-exact bearer its downstream never
  degrades.

**Ie030002 does not wait for our E.** It ends its record about 45 ms after
acknowledging, on our acknowledged MP, so the final frame arrived before
our E was out and the first try timed out (`x2-client-imodem-rr8`). An
all-ones frame on a record boundary can only be the final frame, because a
record starts with 17 ones and a zero, so the client no longer requires its
own E for it. With that, against the original firmware:
- a change to index 8 before LAPM gave `CONNECT 48000/ARQ/x2/LAPM/V42BIS`
  at the I-modem and 46666/31200 here;
- a change 9 s into data, with LAPM up and our line flowing, completed in
  79 ms, and all ten copies reached its DTE.

The latter is in `tests/slow.list`.

**A server-side loss it exposed.** Our server passed upstream bits only in
PAYLOAD, but after a change the peer's data follows its E and B1 at once,
while our 0.5 s DATA_STARTUP is still running. That half second was thrown
away: 0.43 s of V.14 text from our client, and probably the Courier's "one
line lost in the switch" of 10 October. Upstream is now also accepted in a
change's DATA_STARTUP once B1 is acquired. Startup was left alone then;
the same loss at startup turned up live on 30 October and is fixed the same
way (see "Long and A-law live calls").

**V.14 across a change.** With V.42 a change mid-payload was invisible from
the start. Over V.14, measured with full DTE dumps (`ENGINE_PAIR_DTE_DUMP`)
and the server's received bits, it lost data in three places, all ours:
- **Downstream DATA_STARTUP ones** broke a character. Fixed by dropping
  them (above).
- **Our S decoded as data upstream** for the ~80 ms before our server
  recognised it: 65 junk octets. On V.14 the server now holds upstream bits
  for 120 ms, and when a change begins it drops those decoded from the S's
  onset on (`x2_session.c` records it, `s_onset`). Calls with V.42 are not
  delayed: CRC rejects that junk anyway.
- **The seam** cut 7 characters. The client now sends 40 ms of marks from
  V.34 data before its S, taking nothing from the DTE, so the seam and the
  server's decoder tail lose idle. The marks must start between characters
  (`ds_tx_between_characters()`): bit-level marks first landed after the
  first four bits of a character, and the server got that character's
  start, 620 ones, then the rest.

Now 15 of 15 changes mid-payload (2.5 to 3.5 s into data, to index 1, 6
and 12) carry the 12672-octet text exactly in both directions. One V.14
row is in `tests/fast.list`.

**The server's retrain is Tone B (traced 28 October).** The first version
of the client listened for a server Tone A (2400 Hz) and concluded that
Ie030002 never retrains. That was wrong. With our upstream silenced for
500 ms (`ME_X2_CLIENT_SILENCE_AFTER_MS`/`_MS`), the I-modem's own `ATI6`
afterwards reads `Retrains Granted 1` and `Disconnect Reason is Unable to
Retrain`.

Its downstream, our receive tap:
- data until our silence;
- 71 ms of silence (19.711 s);
- a pure 1200 Hz tone (200-Hz-periodic codewords, RMS 2435) for 60.1 s;
- then it clears down.

That is V.90 9.5.1.1, a digital modem initiating a retrain: 70 ± 5 ms of
silence, then Tone B, then wait for the analogue modem's Tone A. V.90
assigns the retrain tones by digital/analogue role, not caller/answerer.
The Courier's own fallback is Tone A, the analogue side's 9.5.2.1, which
is why x2_session's server detector is right to look for Tone A.

The supervisor/DSP mailbox (every command and reply recorded, a scratch
change to courier-emu's pair worker; the repository is untouched):
- during data the DSP reports `0020=000d` every 4M supervisor
  instructions (the rate check);
- at the loss it reports `000e=0000`, `0004=0000` and `0006=0000`, and
  starts the tone on its own;
- the supervisor then issues nothing for 300M instructions (the 60 s on
  the tap);
- it then sends `0057=003f`, `0001=003f` and DSP reset words, and prints
  NO CARRIER, with the ISDN stack logging `l4_DISCONN : modem primitive`.

The I-modem therefore does retrain, and waits a minute for an answer we
never sent.

The client now recognises that Tone B (1200 Hz, 40 ms; data, a 2400 Hz
tone and a quiet tone do not trip it) and ends the call as
`X2_FAILURE_PEER_RETRAIN`. The I-modem's own 0.5 s of silence and Tone B
is kept in `reference-data/x2-imodem-retrain/` and checked in
`x2_client_test`. Live, the client logs it 3.6 s into data.

### Answering the server's retrain (28 October, later)

The client now answers the retrain, and the call comes back to data. What
Ie030002 does was found one step at a time, recording its downstream and
its supervisor/DSP mailbox after each change:

1. **A steady Tone A gets nothing.** It keeps sending Tone B. V.90 9.5.2.2
   continues at 9.2.2.1.3: the analogue modem reverses Tone A once Tone B
   has been present 50 ms.
2. **With the reversal, it follows V.90 9.2.1.1.3.** It reverses Tone B
   about 40 ms later, sends 10 ms more, goes silent, then sends its own
   160 ms probe (period 160 samples, not ours echoed). No INFO0 is
   exchanged: 9.5.1.1 continues at 9.2.1.1.3.
3. **Then it gates, exactly as at startup.** It sends `006b:0021`, the
   marker path, decided from the startup INFO0 words, and waits for the
   x2 marker. V.90's next step (Tone A, reversal, INFO1) gets only
   `0006:0000` after a fixed 2.16M instructions and a 60 s stall.
4. **The marker on 1200 Hz is not heard either** (the same `0006` at the
   same instruction). In this retrain the I-modem sends on 1200 Hz, so it
   listens on 2400.
5. **`4d` on 2400 Hz is heard and refused.** The supervisor clears the call
   at once ("Normal call clearing"). `x2_marker_parse()` reads the second
   index for 6 on the non-answering side, and in the retrain the I-modem is
   that side.
6. **`69` (indices 4 and 6) on 2400 Hz is accepted.** It sends `0034:0004`,
   the retune, as after `4d` at startup, then digital zero, then x2 Phase 3
   if our upstream training starts.

So `x2_session_client_retrain()` does:
- 70 ms of silence, then Tone A;
- the answer side's reversals (x2_session's server branch of `TONE_A`,
  with INFO0 not waited for);
- silence through the server's probe;
- the marker `69` on 2400 Hz 60 ms after the probe;
- then `X2_CLIENT` again.

The engine frees the V.34 modulator, starts a fresh decoder with the same
masks, and keeps the data stack running. Up to two retrains are answered in
a call.

Against the original firmware, 500 ms of silence 6 s into data:
- the I-modem retrains, the client answers, and data resumes 3.7 s later;
- all ten of our lines reach its DTE;
- its line, sent three times well after the retrain, reaches ours each
  time;
- it never hangs up.

That run is in `tests/slow.list`. `x2_client_test` checks the timing
against a scripted digital side: reversals 50 ms into Tone A and 40 ms
after the server's, and the marker 87 ms after its probe.

One trap: the silence hook measured from data entry, which a retrain
resets, so every recovered call was silenced again and retrained again. It
now fires once per call.

## The server's retrain (30 October 2026)

Until now our server hung up when the Courier retrained, and never retrained
itself. The client's answer to Ie030002 (above) showed what an x2 server does
in a retrain, so `x2_session_server_retrain()` now does it with the roles the
other way round. V.90 9.5.1.1 (we start it) and 9.5.1.2 (we answer Tone A
heard for 50 ms) end in the same steps:
- 70 ms of silence, then Tone B on 1200 Hz. No INFO0 either way.
- 9.2.1.1.3: reverse Tone B 40 ms after the analogue modem's Tone A
  reversal, send 10 ms more, then go silent.
- Wait for its second Tone A reversal, then send our 160 ms probe 14 ms
  later. This is Ie030002's timing, measured from our own retrain answer to
  it (`artifacts/x2-rt-answer3-20261028`, receive tap): Tone B ends 10 ms
  after its reversal, and the probe starts 14 ms after our second reversal.
- Hear the marker on 2400 Hz. It is `69` (indices the other way round, so
  `x2_marker_parse()` now reads it for the role we hold), then Phase 3 runs
  from `X2_UPSTREAM_WAIT` as at startup.

The engine frees the V.34 receiver at the retrain and builds a fresh one at
the marker. The MP/E receiver restarts too: its old E would otherwise release
the new upstream at once. The data stack keeps running and V.42 carries what
the retrain cost. A call answers or starts at most two retrains
(`ME_X2_SERVER_MAX_RETRAINS`). After that, Tone A ends the call as before.

**Against the original Courier 403** (clean courier-emu `f1ccd92`,
`S39=8&M5&K0`, `AT+MS=X2,0,0,40000`, LAPM): `ME_X2_SERVER_SILENCE_AFTER_MS=6000`
silences our downstream for 1 s, 6 s into data. The Courier then:
- retrains with Tone A;
- reverses and sends the marker `69` on 2400 Hz, which was never seen before
  (the Courier picks the same marker Ie030002 expects from a client);
- trains Phase 3 again, and data resumes 8.3 s after the retrain began
  (66240 samples; most of it is Phase 3's own length, as at startup);
- 8.4 s later, falls forward from index 1 to index 3 (40000/31200), which
  also completes.

(Measure these in samples: in `--fast` runs the trace clock is wall time.)

Its DTE reads `CONNECT 53333/ARQ/x2/LAPM`, then our line (queued during the
retrain), and no NO CARRIER. All nine probe checks pass, including exact
delivery both ways (`artifacts/x2-srv-rt1-20261030`). Before this change the
same Tone A hung the call up. That row is in `tests/slow.list` (33 s).

**Starting a retrain on our own upstream loss** is opt-in:
`ME_X2_RETRAIN_ON_LOSS=<n>`, default 0. The reason is V.90's
(`ME_V90_RETRAIN_ON_LOSS`, `docs/retrain_and_resync.md`): a retrain
on our receiver's account costs a working downstream, and the analogue modem
retrains when its own receiver fails, which we now answer. With the knob set,
our client going silent for 1.5 s trips `v34_v90_upstream_carrier_lost()`
1 s in. Our client then answers the server's Tone B, and data resumes
8.3 s after the retrain starts. A 400-line V.42 payload arrives exact both ways across
it (`tests/fast.list`). `x2_client_test` runs the two sessions in lockstep.
It checks:
- Tone B at 70 ms;
- our reversal 40 ms after the client's;
- the probe 16 ms after its second reversal;
- marker `69` accepted;
- Tone A in data answered while a retrain is left, and the call ending once
  none is.

**Not done.** No real server or Courier hardware has been tried. (Our
client starting a retrain on its own downstream loss is now done; see the
next section but one.)

## The `\N0` client row and the first CM's inversion (30 October 2026)

The slow row for our client with `\N0` against Ie030002 had started failing
before the server-retrain work. The client sat in `TONE_A` until the stage
watchdog fired, because the I-modem never began x2 Phase 2. Its JM gave the
reason. It answered `c1 45 13 10 83`, its "not x2" JM (V.8 report
`0071:0006`), where a passing call gets `.. 8d` (`0071:0007`).

**What broke it.** `0a43d6a5` (V.25 4.3) holds CI back through 1.8-3.0 s so
it cannot land on an answerer's ANSam onset. Until then our third CI burst
was still going out when we recognised Ie030002's ANSam (onset 1.97 s,
recognised about 3.0 s). SpanDSP's fall-through then sent CM straight after
it, on the same carrier. Since the hold, no CI is draining at that moment,
so the 1 s Te is honoured and our CM starts **from silence**.

The x2 signature is a carrier inversion at each CM's preamble (`get_bit`'s
0x81). A CM that starts from silence has no carrier to measure its first
inversion against. The six-octet LAPM CM still scored enough to pass. The
five-octet `\N0` CM did not.

**Ruled out**, each run against Ie030002 with the hold in place:
- CM sent sooner: Te of 0.5 s, and a V.8 8.1.1 Te timed from the end of CI
  (CM at once, about 1 s after ANSam onset). Both still drew `83`.
- CM octets: ours are `e0 c1 45 13 10 0d`, the same as the passing run's;
  the Courier's are `.. 90 0d`.

**The fix.** When an x2 CM starts from silence, SpanDSP's V.8 now sends ten
marks of carrier ahead of it (`v8.c`, the `V8_HEARD_ANSAM` CM start). With
the CI hold kept:
- `\N0` gives JM `8d` and `CONNECT 57333/x2/NONE`, with lines both ways;
- the LAPM, rate-change and retrain rows still pass;
- all four x2 slow rows pass in parallel, and `make test-fast` passes
  655/655.

V.8 CM receivers synchronise on the ten-one preamble and the sync octet, so
leading marks are idle carrier. The lead-in applies only with the x2
signature on, so other modes are unchanged.

Open: Ie030002's DE14 arithmetic is still unread. The Courier's own first CM
also starts from silence, about 1.2 s after its CI, and it shows 11 marks
before its sync octet. Why that scores and ours did not is unexplained.
Read DE14's inputs before changing the lead-in.

## The client starts a retrain (30 October 2026)

That completes the four retrain cases. Each side can start one, and each
side answers the other's:

| starts | how | answered by |
|---|---|---|
| server (Ie030002, ours) | 70 ms silence, Tone B (9.5.1.1) | our client (9.5.2.2) |
| client (Courier, ours) | 70 ms silence, Tone A (9.5.2.1) | our server (9.5.1.2), Ie030002 |

**Trigger.** On a byte-exact bearer every data frame decodes. The client
now counts consecutive data frames that fall outside the banks
(`x2_pcm_decode` fails), which it already refused to deliver. At
`X2_CLIENT_LOSS_FRAMES` (333 frames, 250 ms) it sets `dn_lost`, and the
engine retrains through `x2_session_client_retrain()`. Those steps are
already 9.5.2.1's: silence, Tone A, its first reversal once Tone B has been
heard and Tone A has run 50 ms. 250 ms is longer than a server's own
retrain takes to show (70 ms of silence, then 40 ms of Tone B), so that
case is still recognised as the server's. The client's two retrains a call
are shared between the two causes. Any data frame that decodes clears the
count. `x2_client_test` checks the trigger at every allocation it tests:
48000 clean payload samples never set it, and 0xff silence sets it at
exactly 333 frames.

`ME_X2_CLIENT_DEAF_AFTER_MS`/`_MS` is the test hook. It feeds the client's
decoder silence in place of the downstream for a while, once per call.

**Against Ie030002** (deaf for 1 s, 6 s into data, LAPM):
- the client logs the loss at 333 frames and sends Tone A;
- the I-modem's Tone B arrives 70 ms after our Tone A begins;
- the reversals, its probe and our marker `69` follow as in its own retrain;
- data resumes, all ten of our lines reach its DTE, and there is no NO CARRIER.

This is the first time Ie030002 has been seen to answer an analogue
side's Tone A. It is in `tests/slow.list`.

**Against our own server** the same hook makes our server answer through
9.5.1.2, and a 400-line V.42 payload arrives exact both ways
(`tests/fast.list`).

## First live x2 calls, through Asterisk (30 October 2026)

Until now every x2 result came from byte-exact offline harnesses. The open
question was whether the live SIP path keeps the bytes as they are, since
x2's downstream depends on it. `tools/soak/x2_live_pair.sh` runs our x2
server (account 2902) and our client (2903) as two `sip_v90_modem`s, both
PCMU, on vmhost CT 117. They call through Asterisk, which relays the media
(both legs' RTP peer is 192.168.88.122, the PBX). Each side sends 40
numbered lines after CONNECT. The build was `aaa00968` in its own worktree;
tower was offline.

| run | what | CONNECT (both ends) | lines each way | bytes changed down / up |
|---|---|---|---|---|
| 1 | plain | 56000/ARQ/X2/LAPM/V42BIS | 40/40, 40/40 | 0 of 728960 / 0 of 728800 |
| 2 | repeat | 56000/ARQ/X2/LAPM/V42BIS | 40/40, 40/40 | 0 of 729120 / 0 of 729280 |
| 3 | client deaf 1 s, 5 s into data | 56000/ARQ/X2/LAPM/V42BIS | 40/40, 40/40 | 0 of 891840 / 0 of 891840 |
| 4 | `\N0`, V.14 | 56000 | 40/40, 40/40 | 0 of 729120 / 0 of 729280 |

All runs reached 56000 down and 31200 up, with no NO CARRIER.

**The bearer is byte-exact.** The bytes were checked by aligning each
sender's `live-tx.g711` tap with the receiver's `live-rx.g711` and comparing
every byte. Over about 26 minutes of bearer (four calls, both directions,
retrain included), not one codeword changed. The delay was a constant 180 or
200 ms, which includes the jitter buffers, and there were no slips. So
Asterisk with PCMU on both legs, and `PJMEDIA_HAS_PASSTHROUGH_CODECS` at our
ends, meets x2's assumption.

**Run 3 is the client's loss retrain, live.** The client logged the loss at
333 frames, and our server answered its Tone A (9.5.1.2). Both were in data
again 9.8 s after the loss. The lines, sent over 20 s, crossed it intact.

Still not tried live: a real x2 server or Courier hardware (A-law and long
soaks follow below). To rerun from the repository root on a rig host:

```
tools/soak/x2_live_pair.sh live/run1 90
CLI_ENV="ME_X2_CLIENT_DEAF_AFTER_MS=5000 ME_X2_CLIENT_DEAF_MS=1000" tools/soak/x2_live_pair.sh live/run3 110
```

## Long and A-law live calls, and the startup line loss (30 October 2026)

**A-law.** Asymmetric x2 is PCMU only by construction: the engine refuses
it on A-law ("asymmetric x2 requires PCMU"), because the server's mapper and
banks are µ-law. Symmetric x2 (`AT+MS=X2S`, 64000 each way) is the x2 mode
that runs on A-law. `x2_live_pair.sh` now takes `MODE=x2-symm LAW=alaw`, which
uses the A-law accounts 2906/2907. Live, both ends logged `PCMA (A-law
passthrough)`. A 90 s call gave `CONNECT 64000/ARQ/X2/LAPM/V42BIS`, 40/40 lines
each way, and 0 of 716320 bytes changed in each direction.

**Ten-minute calls** (vmhost CT 117, through Asterisk, 2000 lines each way at
four a second):

| call | mode | CONNECT | lines (server got / client got) | bytes changed down / up |
|---|---|---|---|---|
| L1 | x2, µ-law, LAPM | 56000/ARQ/X2/LAPM/V42BIS | 2000 / 2000 | 0 of 4793760 / 0 of 4793920 |
| L2 | x2-symm, A-law, LAPM | 64000/ARQ/X2/LAPM/V42BIS | 2000 / 2000 | 0 of 4794080 / 0 of 4793920 |
| L3 | x2, µ-law, `\N0` (V.14) | 56000 | **1999** / 2000 | 0 of 4792000 / 0 of 4792000 |

The delay stayed constant, 180 or 200 ms, for all ten minutes in every
direction: no slips and no retrains.

**L3's missing line was ours.** It was `cli 00000`, our client's first line,
sent as soon as its own CONNECT appeared. The client reports CONNECT once both
of its directions are in data. Our server takes upstream bits only in PAYLOAD,
which it reaches at the end of its 0.5 s DATA_STARTUP, 0.31 s later in L3.
Upstream that arrived in that window was discarded. The bearer delivered every
byte (0 changed), so the loss was in our server.

The 28 October fix accepted upstream in a rate change's DATA_STARTUP once B1
was acquired, but left startup as it was. The engine pair hid this because its
payloads start 0.5 s after both ends report CONNECT.
`engine_pair_test --payload-on-connect` starts each DTE on its own CONNECT, as
a real terminal does. Over V.14 our server then lost the client's entire
payload, 3 runs of 3.

The fix: upstream is accepted in startup's DATA_STARTUP too, once E has been
seen and B1 acquired. By then the peer is sending data. Results:
- `--payload-on-connect` over V.14: 3 of 3 pass (a fast-tier row);
- all five x2 slow rows still pass against the original Courier and Ie030002;
- live, three V.14 calls with the client's first line 0.1 s after its CONNECT
  delivered 300/300 lines each way.

## A-law asymmetric x2 in the I-modem firmware (30 October 2026)

Asymmetric x2 is not PCMU-only in the firmware: the I-modem's S58 bit 4 is
"Force x2 A-law mode", present since the first x2 release, `IE020104`. Where
the A-law mode lives was traced in Ie030002's DSP image
(`../courier-emu/tmp/imodem-x2data/dsp-program.bin`) and its supervisor.

**`ffd9` bit 2 is the -6 dBm constellation, not A-law.** The bank builder
(`CFF4`) changes every bank when `ffd9` bit 2 is set, to a set with about
twice the minimum spacing, and E's table becomes `d241` (`95 97 9d 9f a7 ad b5
bf d5`, 6 dB up). The only writer is the Phase 2 gate, `95cb..95d4`:
`ffd9 |= 4` when `fef0 & f6a0 & 0200`. Those are our transmitted and the
peer's received INFO0 bodies, each stored one bit right (`fef0` = `ffdc >> 1`).
So the test is capability bit 10, the -6 dBm option (S58 16), in both words.
Confirmed against the firmware: S58=48 with a client INFO0 of `25ff` set
`ffd9` to `8004`, and E used the `d241` table (98.2% of its octets). Our client,
which has no -6 dBm banks, then failed. Note that `c5x_disasm.py` already
prints true bit numbers; inverting them again sends you the wrong way.

**A-law is DSP host command `6b`.** The resident's dispatch table (base
`88d5`, from `85da..85e0`) sends tag `6b` to `82d7`: `039f |= 0800` and
`ffd9 |= 0001`.
- `039f` bit 11 selects A-law in the resident's G.711 codec (`81b6`), which the
  control signals (V.8, Phase 2) use.
- `ffd9` bit 0 makes `d1c4` set `@23 = 2a`, `@6b = 42`, `@6c = 0` (no bias),
  `@6a = 7f`. The per-sample output (`cc63..cc7b`) ends `@27 ^= @23`. Since
  µ-law stores `~field` and A-law `field ^ 55`, A = µ ^ `7f` ^ `55` = µ ^ `2a`:
  the same level coded in A-law. The banks themselves do not change (`CFF4`
  with `ffd9` bit 0 gives identical banks). `x2.c`'s "final output XOR (0 or
  0x2a)" is exactly this.

**Neither supervisor sends `6b`.** In Ie030002 and in IM020104:
- the 62 (respectively 56) calls to the command sender carry 36 immediate tags,
  none of them `6b`;
- the deferred-command routine `7992e` is called 34 times, with tags `5e 63 01
  5f 60 0f 61 5b 5d 0a`.

S58 bit 4 is read in one place (`b2801`). It sets capability bit 9 (`or bx,
0200`), which advertises A-law in INFO0 and feeds the availability check at
`b27cc` (S58 & 6 == 6 turns x2 off). Unexplored: the host writing DSP data
memory directly, and other server products (Total Control).

**With `6b` delivered, the I-modem runs A-law x2.** This was a DIAGNOSTIC: the
client probe's `--inject-alaw` hands the DSP host command `6b` once, right after
the supervisor's tag `70`; the firmware is not modified. The bearer was A-law
(NET3 profile, `90 90 a3`). Then:
- `039f` gained bit 11 and `ffd9` became `8001`;
- V.8 and Phase 2 decoded as true A-law at our end (no `ME_X2_CONTROL_LAW`);
- without an XOR our client never found B. With every received octet XORed by
  `2a` it found B, J, the record and data: `CONNECT 57333/x2/NONE` at the
  I-modem, `CONNECT 56000` here, and the I-modem's text arrived.

So A-law x2 transforms everything downstream, scripted training included, by
`0x2a`. Open: our upstream text did not reach the I-modem's DTE in that run,
although its DSP replies matched a µ-law run's and the call stayed in data to
the end. Before this, an "A-law" call between Ie030002 and our client was
µ-law-coded throughout: the codec was never switched, and the emulator passes
octets unchanged.
