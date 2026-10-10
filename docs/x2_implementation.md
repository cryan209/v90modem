# x2 implementation

The recovered payload mapper is implemented and checked against original DSP
execution. `x2_session.c` now connects capability exchange and downstream
training to the modem engine's raw-G.711 path. Select `--mode x2` (or
`ME_MODE=x2` in the engine replay tool). The supported initial profile is an
answering digital endpoint using PCMU, a 3200-baud high-carrier upstream and
the recovered low-level training alphabet.

The supported short-record path receives MP, transmits its three-word response,
waits for upstream E and activates the downstream payload source. Fresh original
Courier 403 feedback reaches native CONNECT and upstream B1 acquisition.
The PCM response-mask correction below establishes complete messages in both
directions against that emulator. The engine enters DATA after accepted MP/E
and upstream B1 acquisition; LAPM and sustained hardware operation remain
unqualified.

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
qualifications are in `artifacts/x2-handoff-20261006/README.md`.

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

Evidence is in `artifacts/x2-symmetric-20261006`. `call.json` contains checked
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
`artifacts/x2-host-analog-20261006/README.md`.


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

Evidence: `artifacts/x2-downstream-20261006/`, with fresh calls in the sibling
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
(`artifacts/x2-rx-improve-20261006-long-fixed/engine-tx.g711`, inverse mapper
from tx sample 111005, GPC descrambler, 8N1) recovers all 940 bytes exactly.
The call has no error control (`CONNECT 53333/x2/NONE`), so nothing can hold
off the far end. It sends 32000 bit/s into a Courier whose emulated DTE is
paced at the firmware-programmed serial rate. The Courier output shows a
clean 42-byte omission and then a missing tail, which is an overrun pattern,
not line errors. A/B with `tools/probe_x2_host_courier.py --host-pace-samples`:
an unpaced burst loses data again (454 bytes reach the Courier DTE), while
one byte per 16 samples delivers all 940
(`artifacts/x2-ds-pace-20261010-{burst,paced}`).

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
(`artifacts/x2-ds-pace-20261010-fixed`).
