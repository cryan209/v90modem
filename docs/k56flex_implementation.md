# K56flex implementation

`k56flex.c` (payload core), `k56flex_v8bis.c` (V.8bis exchange),
`k56flex_probe.c` (training sample generator), `k56flex_train.c` (server transmit
sequencer), `k56flex_client.c` / `k56flex_rxfe.c` (a client for our own server, over G.711
octets or over linear audio) and `k56flex_channel.c` (a simulated analogue loop for it),
exercised by `make k56flex-test`, `k56flex_v8bis_test`, `k56flex_probe_test`,
`k56flex_train_test` and `k56flex_client_test`. The client is a receive model for tests and capture analysis, without a live
analogue DATA role. It is the digital-side (server) downstream payload core
and its exact inverse, plus the V.8bis capability frame, the parameter record
and the accepted-report logic. `AT+MS=K56` (automode enabled) selects the
V.8bis preflight for the next call; `ME_K56FLEX=probe` selects the transmit
probe. The server now completes raw duplex connections against native HSF
over PCMU at the unchanged 129-tap analogue reconstruction default: negotiated
V.8bis/V.8, audio-derived S/S-bar, report, MP/MP', upstream B1 and the PCM
mapper. A two-minute run delivers 64 KiB exactly each way with no carrier
interruption. The client model remains incomplete. See the 21 October L2
window qualification below; earlier 65-tap results are diagnostic history.
Hardware interoperability and in-data renegotiation are not qualified
(Draft 0.23, Table 1a and clause 11).

**Read first:** MICA overlay 8E is the V.32/V.32bis datapump, not K56flex,
and the resident QAM engine once lifted here under the K56flex name is now in
`mica_qam.c` (`docs/mica_qam_firmware.md`). See "What in the MICA firmware
is K56flex" at the end. The K56flex PCM module is overlay 8C/8D.

## Implemented

- Shipped level payloads for all 82 (law, pad-group, rate) tables
  (`k56flex_tables.h`), with the allocation record and sign pattern D604 derives.
- Eight-sample (1 ms) mapper: `1 + D^5 + D^23` scrambler, bounded-composition
  shell unranking (alphabets up to 16), extra index bits, explicit/derived signs
  against the running DC sum, six-position RBS mask rotation, the alternating
  amplitude budget and the 1/2/4 impairment reduction cycle with rank shifts.
- G.711 output: level words are exact codeword magnitudes of the bearer law.
- `k56flex_pcm_rx_frame`: exact inverse (G.711 octets -> descrambled source
  bits), rejecting non-codewords, wrong derived signs and out-of-subset ranks.
- Report: `8FEF` construction (1521), header predicate, 24-bit repeated record
  scrambler and a blind-alignment acquirer.
- V.8bis MS/CL/ACK1/NAK1 payloads with the two template edits, FCS, flag
  framing and the MICA bit-stuffing rule.
- Parameter record: both record lengths, reflected-8408 checksum, the marker /
  separator framing as queued by DA56, and a validating parser.

## Diagnostic controls and precedence

These controls describe the experimental path, not a supported K56flex DATA
configuration. Prefer `AT+MS` for normal modulation policy (V.250 6.4.1).

| Control | Current behavior |
|---|---|
| `AT+MS=K56` | Enables the V.8bis exchange when `ME_K56FLEX` is unset/empty; does not by itself enable PCM probes. |
| `ME_K56FLEX=0` | Disables the K56flex path even if AT+MS offers it. |
| `ME_K56FLEX=1` | Exchange only, then ordinary V.8. Any nonempty value other than `probe` or a value beginning with `0` also selects exchange-only. |
| `ME_K56FLEX=probe` | Enables negotiated startup plus transmit probes when agreed; otherwise the legacy transmit-only probe followed by ordinary V.8. Exact lowercase spelling matters. |
| `ME_K56FLEX_RATE` | Probe downstream mapper rate, default 56000 bit/s; invalid/unsupported configurations fail initialization. Not a negotiated upstream rate. |
| `ME_K56FLEX_GATE_MS` | Gate wait, default 8000 ms; converted to 12-sample pairs (`ms * 2 / 3`). Not the generic V.8 timeout. |
| `ME_K56FLEX_ROLE=client` | Sends the client-model V.8bis capability octet. Does not select an implemented analogue DATA role or override the SIP call direction. |
| `ME_K56FLEX_BLIND` | Nonempty value not beginning with `0` enables blind V.8bis exchange; leave unset for the normal detected exchange. |
| `VPCM_G711_TAP_DIR` | Retains the byte-exact bearer both ways; use a fresh directory per run. |

V.8bis initiator/responder is normally chosen from SIP call direction. After
an agreed exchange, V.8 calling/answering follows the MS direction (V.8bis
9.9), while common Phase 2 remains the digital, answer-shaped prelude. These
are three separate role decisions; a socket initiator is not necessarily the
V.8 caller. `ME_K56FLEX_ROLE=client` changes none of those role rules.

## V.8bis exchange (live, opt-in)

`ME_K56FLEX=1` makes the engine run the V.8bis exchange before V.8 (Draft 0.23
clause 4.1: V.8bis precedes V.8). As answerer it listens up to 3 s for CRe
(1375+2002 Hz, then 400 Hz), waits 500 ms, sends CRd (1529+2225 Hz, then
1900 Hz), receives the client's CL on V.21 channel 1, applies the peer predicate
(below), replies MS (or NAK1) on channel 2 and logs a
trailing ACK1/NAK1. A dialling call runs the initiator side, which is the lab case: v90modem
calls the MultiTech RAS, so v90modem starts V.8bis with CRe and sends the
server's CL (version octet 0x42), and the RAS would answer CRd and MS. Whatever happens,
the engine then starts ordinary V.8, so the call carries on as V.90/V.34; the
log line `K56flex V.8bis ...` records the peer's CL octets, which is the useful
output of a lab call. It costs up to 3 s of ANSam delay when no CRe arrives, so
it is off by default.

Tone frequencies and segment lengths are the recovered oscillator increments
and block counts. Levels (tones -9 dBm0-ish per component, V.21 -12 dBm0),
the detector thresholds and the waits are this implementation's: the firmware
ticks are uncalibrated (clause 4.7). `k56flex_v8bis_test` loops initiator
against responder through linear samples, also 10 dB down with noise, and
checks the timeouts. That proves self-consistency only.

## Training stream (opt-in, transmit only)

`k56flex_probe.c` lifts module 8C's D873 block generator (D811 configuration
load, 90FF source fetch with last-word replay, D890 sequence selector, D83C
level lookup, D84F sign/DC adjuster). It matches the original on 168 runs
(7 stages x 2 laws x 3 pad groups x scrambled/raw x two sequence seeds; 48
blocks each), including the carry flag entering D890, which the vectors pin down.

`k56flex_train.c` strings the stages in the order DB11/DB2E/DB37/DB68/DB7F/D9CB
call them: 47 pairs of silence; identification (E4E4 x 22 pairs, 1B1B x 2);
probes of 1364, 704 and 1366 pairs; two peer gates; identification again; 688
pairs, `FFFF FFFF 0000` and 8 pairs, 340 pairs of D99C (D9AD when report bit 8);
a third gate; the parameter record sent as DA56 does, rebuilt with argument 1;
DA75's tail; six priming frames; then the verified data mapper. The fixed
stretches reproduce the original primitives' stream for both laws and both
bit-8 settings (56 segment hashes in `k56flex_train_vectors.h`). A pair is 12
samples, so the fixed prefix through probe 3 is 3,507 pairs, 5.26 s.

With an accepted K56flex MS/ACK, `ME_K56FLEX=probe` now runs negotiated V.8
and the experimental common Phase-2 prelude **before** raw PCM probes (see
the native HSF handoff below). A failed negotiated startup/probe ends the
call without a lower-mode DATA fallback. If V.8bis did not agree K56flex,
the explicit transmit-only diagnostic is preserved: it sends the probes,
waits `ME_K56FLEX_GATE_MS` (default 8000), then starts ordinary V.8. Neither
path manufactures a peer gate. Pair it with `VPCM_G711_TAP_DIR` to retain
untouched codewords both ways.

What the gates are: the firmware polls DM 8F31 bits 11, 10, 12|13, 13|14 and
14, which its receive worker sets from the client's signalling. How that looks
on the wire is not recovered, so `k56flex_train_status()` takes those bits from
the caller and nothing in the engine sets them. Resident B31D returns 8990 or
89B0, which are V.32bis rate signal R words (Table 5, LSB-first). The trace
that showed them setting 8F31 bit 11 ran overlay 8E, which is V.32bis, so it
does not identify K56flex's gate source (`docs/mica_qam_firmware.md`). The
probe still supplies its legacy FFFF fixture value. The priming frames' handling of leftover queue
bits and the scrambler history across the hand-off follow the firmware flow
only approximately. A live call stops at gate A at best.

## Audio/G.711-only peer boundary, 21 October 2026

The client API no longer exposes `ctl.status` or `ctl.report`. Neither
immediate callbacks nor delayed event queues may transfer a report or release
a peer's training gate. Draft 0.23 clauses 4.10-4.12 and 11 require the
actual upstream signalling; its complete waveform generation/reception
remains unfinished. No replacement tones or invented signalling are used.
The remaining `ctl.data` callback delivers decoded payload to the local host;
it is not connected to a peer's protocol state.

`k56flex_client_test` now tests the downstream prefix through G.711 or a
simulated analogue channel, in both laws at 32000/56000 bit/s. All eight
cases acquire/check the prefix and verify the server stays at gate A until
its configured timeout, with no received status/report and no DATA.
`legacy_pcm_decode_test` records that unanswered prefix and requires P1
acquisition without a parameter-record claim. Neither test uses callbacks
to construct its input dialogue. These replace the former synthetic full
handshake/data tests below; their old payload/SNR results are component-model
history, not current duplex qualification. Mapper, parameter checksum and
sequencer primitive tests remain separate unit tests, not interoperability
proof. The raw receiver's internal timeline bookkeeping never reaches a peer.

The independent HSF pairing tool already exchanges only byte-exact PCMU
between endpoints. Its original-pump reads are diagnostics only. That native
path remains the interoperability check; a full Flex connection is still
unverified.

## Historical client loopback using a side channel

Before removal of the side channel, `k56flex_client.c` received the server's raw G.711 and ran the whole startup
against `k56flex_train`: it follows the training timeline, checks every fixed
stage against the original-primitive stream, inverts each training block back to
its source bits by exhaustive search (unambiguous for the identification, P1, P3
and parameter stages; the P2 and silence stages carry no information it needs),
descrambles, signals the peer gates and the report, decodes the parameter record
(rate, flags, checksum), reads the six priming frames (all ones) and then the
data frames. `k56flex_client_test` runs seven configurations (both laws, 32-56
kbit/s, RBS selectors 0, 2, 3 and 4, both table-select bits, both record pacings)
bit-exact over 8,000-20,000 data bits and checks that a single corrupted octet
is caught.

### Over linear audio

A real client never sees codewords: its codec delivers 8 kHz linear samples of the network D/A
output after the analogue loop. `k56flex_client_rx_linear()` runs the same client from that.
`k56flex_rxfe.c` is the front end:

- **Acquisition.** Energy onset, then normalised cross-correlation against the known P1 probe
  (the scrambled-ones stream, which is noise-like) gives the symbol timing and a gain estimate.
  The client no longer has to be listening from the first sample.
- **Equalizer.** A 96-tap T-spaced equalizer solved by exponentially weighted least squares
  (Cholesky with diagonal loading toward the acquisition-time centre-tap solution, every 128
  symbols). RLS diverged (windup in the weakly excited directions) and NLMS was too slow on a
  coloured signal. It trains on the known stream (P1 through the parameter-training stages), is
  held while the sparse parameter-record stream runs, and adapts decision-directed on the
  priming and data frames.
- **Resampler** (windowed sinc, 48 taps) with a coarse clock-offset estimate from the drift of
  the training correlation.
- The client slices the equalized levels itself, and now **observes** the server's phase changes
  from the signal (silence at the end of gate A, energy at the end of gate B, the first symbol
  group that is off the training levels after "parameters accepted" for the start of priming)
  instead of mirroring them. That is what makes it tolerate upstream latency; the test delays
  the client's events by 17-30 symbols.

`k56flex_client_test` results over the simulated loop (`k56flex_channel.c`: gain, delay,
two RC sections, a high-pass, noise, clock offset, exact windowed-sinc sampling):

| Channel | Equalized SNR | Result |
|---|---|---|
| perfect wire, 9 dB pad | 74 dB | 56 kbit/s, 12,000 data bits exact |
| RC roll-off + HP + noise, 30-symbol upstream latency, A-law | 34 dB | 32 kbit/s exact |
| heavy roll-off (band edge 20 dB down) | 25 dB | training, gates and record only |
| fractional delay 5.7 samples, +3 ppm | 29 dB | training, gates and record only |

What the SNR buys is set by the shipped tables' minimum level gap: 520 (32 kbit/s), 384, 256
(40-44k), 128 (48k) and **32** at 56 kbit/s, so 56k needs the equalized error below about 3
level units, i.e. 65 dB or better. Only a nearly ideal channel gets there.

Limits (this is a front end that works on a simulated loop, not a validated one):
- **Clock offset is the weak point.** The equalizer absorbs a locked clock to a few ppm; the
  drift estimator moves a 60 ppm offset about halfway to the truth and the SNR still collapses.
  PCM at 8 symbols/ms needs timing to about 0.005 sample, and a real client would pull its codec
  clock with a PLL. Designs tried and dropped: centroid/peak/shape-matching of the correlation
  (the scrambler gives the reference a second lobe near lag 5 so peaks hop), equalizer group
  delay (couples to the adaptation), slips plus tap shifts.
- **A fractional sampling phase caps a T-spaced equalizer near 29 dB** for a full-band signal
  (the sinc tails need hundreds of taps). A real loop's analogue low-pass hides this by removing
  the energy near Nyquist, but it removes information with it.
- Equalizer training leans on the probe stages being periodic or low-rank: the LS solution
  generalises to the data stream only because of the centre-tap prior and the decision-directed
  adaptation. This was found, not designed, and is the most likely thing to break on a real loop.
- The channel is mine. A bilinear Butterworth low-pass has exact zeros at Nyquist and destroyed the
  band edge outright; the RC sections used here are gentler than a real telephone loop.
- Gain control is a one-shot estimate at acquisition; there is no level tracking, no echo and no
  robbed-bit corruption of the codewords.
- The upstream path is still a side channel (the client's events reach the server through the
  test), with latency now tolerated but not unknown in sign or size.

The octet-level client (G.711 in, zero latency) remains for the decision logic alone.

Building it found three server defects that the one-way tests could not see, now
fixed and covered: the scrambler ran a word ahead of what was transmitted (so
discarding the queued bits at the hand-off would have desynchronised the
descrambler), the probe source FIFO was 64 bits when a parameter record queues
192, and the silence/training stages read junk levels when report bits 6/7 were
set. The last one is a finding rather than a fix: the pad-group variants of the
probe records mostly point at unrelated words in the shipped data, so training
always uses the base tables and the firmware's behaviour there is unrecovered.

## Evidence

`k56flex_firmware_vectors.h` holds outputs of the original MICA C53 module 8C
(D604, D689, D744, 490E, BE68) executed in MicaEmu's isolated repeat-fetch core:
82 runs / 1,140 frames / 9,120 samples across both laws, all 13 rates, both
pad-table groups and 58 impaired report fields, plus 48 parameter-record queues
from bank 88 entry 4269 and the original source-queue path. The C core matches
every frame. `k56flex_test` also checks that the spec's V.8bis FCS values and
stuffed bit counts (184/185/64/64) are reproduced, and round-trips every vector
through the G.711 octets and the inverse mapper, at several output block sizes
and with a source that pauses.

Regenerate (needs the MicaEmu repository with `.build/k56flex-core` built by
its `tools/build_k56flex_core.py`):

```
python3 tools/generate_k56flex_vectors.py ../MicaEmu .
make k56flex-test
```

## V.8bis peer predicate and the Rockwell client

The client side was recovered from the Rockwell HCF controller
(`silver98.exe` `WIN95AC.VXD`) and cross-checked against the Patton server
(overlay 11); see `cruft/patton-re/K56-client-comparison.md`. Two
differences from the MICA template matter:

- The client's NPar block after SPar(1) is three octets (`80 B0 CF`), not
  MICA's one (`C2`). The NS block, and with it the version octet, therefore
  start two octets later: the version octet is at payload offset 15, not 13.
  `k56flex_v8bis_ns_offset` walks the blocks instead, as Patton PM2BD7 does:
  SPar(1) bits 0 and 1 each announce a block ended by an octet with both
  delimiting bits set, and bit 2 announces one octet. It requires I-field
  octet 1 bit 6 (NS present) and T.35 country `B5`.
- The version octet is a bitmap: bits 0-4 are versions, bit 5 an option and
  bit 6 means server. A client offers `0x03` in CL and answers a `0x42`
  server with `0x02`. Patton answers `0x03` with `0x42`.

`k56flex_v8bis_peer_acceptable` accepts AC6D's `0x02`/`0x21` and any client
value (bit 6 clear) with version bit 1, so it accepts the Rockwell CL offer.
The old fixed-offset predicate read the client's licensee octet (`0x81`) and
NAKed every Rockwell HCF client. HSF 7.12 soft modems never set the
NS-present bit, so they are still rejected and the call continues as V.90.

`client_model` now sends the recovered Rockwell CL/MS
(`k56flex_v8bis_rockwell_client_payload`), so the loopback tests exercise
the walker. These are static recoveries; no Rockwell modem has been
connected.

## Draft 0.23 correction found while implementing

Draft 0.23 labels the tables selected by DM EC6E bit 0 = 0 as A-law and = 1 as
mu-law. The shipped data say the opposite: every level in the bit-0 = 0 tables
is a mu-law codeword magnitude (and none of the 1 tables are), and every level
in the bit-0 = 1 tables is an A-law magnitude. `k56flex_test` asserts this for
all 3,148 levels. The probe levels in clause 4.10.1 follow the same pattern
(3772 is mu-law, 3904 is A-law). `k56flex_law_t` is named by the data, so
`K56FLEX_LAW_MU` is EC6E bit 0 = 0. The octet-18 law bit in the V.8bis template
is `0x20` when EC6E bit 0 is set, i.e. for the A-law tables. The Rockwell
client confirms the polarity: its NS parser (HCF/HSF `WIN95AC.VXD`,
100A4B54) reads bit 5 of that octet as "Server is an ALaw server", and bit 6
as "server forces the law". `k56flex_v8bis_payload` therefore sets `0x20` for
A-law. It used to set it for mu-law, which told a client on a PCMU call that
the server forced A-law.

The spec's recovered serial compander is mu-law only, but that is the DSP's
T1 output stage. Here the codeword is chosen in the bearer's law, so PCMU uses
the bit-0 = 0 tables and PCMA the bit-0 = 1 tables.

## Reject incomplete V.8bis NS blocks

The NS walker now requires all octets declared by the NS length before
returning a block offset (Draft 0.23 clause 4.1). Previously, Rockwell CL
prefixes of 16 or 17 bytes (full length 18), and MICA-layout client prefixes
of 14 or 15 bytes (full length 16), could pass the peer predicate because
the version byte was present even though the DSP/law tail was missing.
An overstated NS length was also accepted.

`k56flex_v8bis_test` checks every shorter prefix of both recovered layouts,
an oversized NS declaration and a provider length that places the version
outside the declared NS block. Complete templates and the clean/noisy audio
exchanges still pass. All five K56flex binaries and `legacy_pcm_decode_test`
pass after rebuilding. This is framing validation only, not progress on the
missing live upstream gates or hardware qualification.

## Not covered (the integration boundary)

- The client's side of the exchange: decoding what the client sends during and
  after the probes (the gate signals above), its report, and the upstream
  receiver.
- The upstream receiver (a V.34-style path, as for V.90) and receive of the
  client's report waveform. Only the decoded-bit report logic exists.
- Network RBS phase, retrain and fallback, gain/pad selector meanings
  (clause 7.24 is Provisional) and the peer parser AC6D for received MS/CL.
- Hardware interop. No result here says a real K56flex client will connect.

This probe was attempted from vmhost CT 117 against the current RasFinder
extension 6311; see the live attempt below. It did not produce a K56flex
V.8bis response. The native legacy HSF peer below does complete V.8bis and
exposes an earlier post-ACK startup defect in our engine.

## Offline call evidence

`vpcm_decode --k56flex --visualize-html flex.html recording.wav` runs the
existing linear-audio client without upstream gate callbacks and includes
the generic V.8bis scan. P1 acquisition and checksum-valid parameter records
are emitted as receive evidence; model stages, report assumptions and failures
are diagnostic rows. No payload bits are exported. The bundled 48k recording
acquires P1 but yields no valid parameter record with the current model. See
`docs/html_call_export.md` for limitations and validation.


### Gough Lui parameter recovery experiment, 5 October 2026

The normal exporter above still stops before parameters. An independent offline
FIR fit to known parameter training, followed by nearest **whole-block** probe
decisions, recovers repeated CRC-valid initial and final parameter records from
the bundled call: `03f1 8fff 0000 0000 0000 0000 0000 0000 0000` and the same
with `83f1` first. Two equalizer lengths reproduce these records; an independent
bit-level parser verifies their separators and CRCs (12 and 31 valid records).
This uses the PARAM_B pacing hypothesis, not a decoded upstream report.

The current firmware-derived rate interpretation reads field 15 as 60000 bit/s,
which disagrees with the 48000 recording label and best payload geometry trial.
That interpretation remains open. A longer offline equalizer yields 231 valid
48 kbit/s PCM frames under assumed mapping/alignment, but zero CRC-valid HDLC
frames and zero verified user bytes. These are candidate bits, not payload
recovery. The experimental receiver is kept out of the production audio path;
no DSP constants or live behavior changed. Measurements, scripts, raw records
and candidate bits are in `reference-data/artifacts/gough-payload-recovery-20261005/report.md`.


The reusable offline record tool is now available:

```sh
.venv/bin/python tools/k56flex_record_recover.py \
  gough-lui-v90-v92-modem-sounds/k56flex-48000.wav \
  --channel L --output artifacts/flex-records
```

It requires NumPy and a C compiler, compiles only the small probe/mapper helper,
and writes `records.json`, equalized streams and one-byte-per-bit candidate
files. Raw records are promoted only when the same CRC-valid words repeat at
least twice in both independent FIR fits. Pacing remains an explicit hypothesis;
the tool assigns neither a downstream rate nor a decoded upstream report. It
rejects the silence and x2 negative controls, and a one-bit mutation of a
recovered record fails its independent CRC check. This is an offline parameter
recovery tool, not a payload exporter or a live receiver.

Further mapping searches found no FCS-valid payload over 806400 rate/report/start
trials. Joint P1/PT_A fitting changes which rate wins the longest-PCM-run metric,
so that metric cannot resolve the wire rate. The parameter field widths and
interpretation outside the recovered MICA constructor remain open.

## A-law noisy-loop regression fixed (2026-10-09)

The 32 kbit/s RC/high-pass/noise case failed during priming despite a valid
parameter record. Its first equalized PCM frame had a roughly -240-level
residual baseline; subsequent nearest-level decisions violated the mapper's
frame constraints. The linear receiver now estimates codec DC from its first
160 quiet samples and tracks the mean decision residual after each valid PCM
frame. Invalid frames cannot feed adaptation. This is a receiver design change
within Draft 0.23 clauses 7.15-7.23, with no change to codewords, scramblers,
mapper rules or sample accounting.

The original row and four additional noise seeds with +/-20 codec offsets
recover all six priming frames and 12,000 data bits each without error. With
initial DC removal alone, the original row and one added row still fail;
baseline tracking is required too. The quiet-start assumption is inherited
from acquisition's initial noise-floor measurement. Hardware interop, larger
clock offsets and the missing live upstream path remain open.

## What in the MICA firmware is K56flex (2026-10-10)

- **K56flex PCM module: overlay 8C/8D.** 0C7C tests DM 8FAA bit 0. When it is
  set, 0C7F's loader 2BF6 loads 8C (or its processor-role companion 8D). That
  module holds the probes, the parameter record, data activation (DC3B) and
  the payload mapper (D604/D689) implemented here.
- **Overlay 8E is V.32/V.32bis, not K56flex.** It is loaded by 2C05 from the
  general answer path (98F5, 1FB6, 9DEB; 80/81 when DM EEAC bit 10 is set). It
  shares the D600 window with 8C, so it cannot run during a K56flex PCM
  session. Its fixed 2400 baud / 1800 Hz transmitter, its 8990 word (V.32bis
  Table 5 rate signal R, LSB-first), its 8880/888F headers (R and E sync bits,
  Tables 5/6) and its 5/23, 18/23 descramblers (V.32 GPA/GPC) identify it.
  The resident QAM engine lifts that were made under the K56flex name
  (response collector, coordinate decoder, FIR, resampler, timing, DAB7
  predictor, 90FF..8270 transmit chain, the 8E gate trace) now live in
  `mica_qam.c`; see `docs/mica_qam_firmware.md`.
- **The resident report path is K56flex's.** The 24-bit report collector here
  (`k56flex_report_*`, 1D5A, header `(report & 888F) == 8880`) uses a V.32bis
  R-shaped word, and its result DM 8FEF (1521) is consumed by 8C. MicaEmu's
  live rig (`artifacts/k56flex-client-20261009/README.md`, 10 October 2026)
  shows original 13DE/1402/1D5A accepting a repeated 24-bit report and the
  899F terminator inside K56flex calls, over the upstream described next.
- **Upstream: V.34 signalling and data at 3200 baud.** In the MicaEmu live
  rig, MICA accepts an upstream of S, S-bar, PP, TRN at 3200 baud (SpanDSP's
  V.34 modulator), V.34 MP framing (10.1.3.9, Table 20) with 18/23 scrambling,
  then E, B1 and V.34 data; a 2400-baud control fails at the first gate (23CD).
  The rig's verified reference connection is 38000 down / 9600 up (7616 exact
  upstream bytes); a 28800 proposal resets at MICA's 476A trellis-loss check.
  The Rockwell K56flex image reports client TX rates up to 33600 from the same
  $2F28 table as the PCM ladder. MICA's upstream receiver is resident code
  (5903/5961, the 4734/476A trellis path), not a K56flex overlay.

## Why the 28800 upstream fails in MicaEmu's rig (2026-10-10)

`tools/k56flex_upstream_lattice.py` compares MICA's trellis input (the 47B9
capture) with the client's transmitted points, in V.34 lattice spacings
(points are odd multiples of 128, so one spacing is 256). On the post-XC-fix
28800 call `connect-xc-input` (MicaEmu `artifacts/k56flex-client-20261009`):

| Window | Error RMS | After ISI fit: radial / tangential | Phase |
|---|---|---|---|
| 18.74-19.06 s (1024 symbols) | 0.69 | 0.32 / 0.42 | jitter 2.8 deg |
| 18.74-21.55 s (8996 symbols) | 2.69 | 0.48 / 2.61 | ramp -22.5 -> +23.6 deg, 0.0487 Hz |

- **Radial gain is flat (1.00 from |point| 3 to 31).** There is no compression
  or clipping. Intersymbol interference is small (largest tap 0.02).
- **Constant 0.0487 Hz frequency offset, untracked.** The client's carrier is
  exact (SpanDSP `carrier_frequency()` with code 4: 3200 x 4/7 Hz, 32-bit
  DDS). Rounding a 16-bit NCO increment for 1828.571 Hz gives 0.0419 Hz, so the
  offset is MICA's. A decision-directed loop normally removes it, and does at
  9600. At 28800 its decisions are already wrong, so it cannot. **The ramp is a
  consequence, not the cause.**
- **The cause is the floor: about 0.3 spacings of radial noise and 2.8 deg of
  phase jitter** (lag-1 autocorrelation 0.5) on a noise-free emulated link.
  28800 needs about 0.15 spacings or better. 9600's +/-1, +/-3 points
  tolerate it. That is why the reference connection works at 9600 and nothing
  above does.
- **Not mu-law.** The 9600 run peaks at 7082 (-13 dBFS) with no clipping,
  about 37 dB of mu-law SNR, far above the ~25 dB measured here.

### Located: MICA's equalizer, out of training (2026-10-10)

Method (no MicaEmu files changed): rerun the saved `connect-xc-input` command
(`*-command.json`) with every output in a scratch directory and
`tools/k56flex_upstream_rig/tee_peer.py` between `mica_trace` and the client.
It records the exact G.711 MICA is fed, and with `TEE_UP_GAIN` scales the
whole upstream like a line gain. The rerun is deterministic: its 47B9 capture
is byte-identical to the original, and it runs in 10 s.

- **The waveform is clean.** `offline_rx.py` fits the best fixed linear
  receiver (exact V.34 carrier, 25 taps, no adaptation) to the recorded
  upstream: **0.13 spacings, 38 dB, over all 2.8 s, with no drift**. That is
  the mu-law limit at the client's -24 dBFS. MICA's trellis input on the
  same audio is 0.51-0.62. About 11 dB is lost inside MICA's receiver.
- **Not level or fixed-point.** +5 dB of line gain leaves MICA's error at
  0.52 spacings, so it scales with the signal. A floor fixed relative to
  full scale would have dropped. That also rules out echo or downstream
  leakage. (+8 dB stops startup before data: a separate level limit.)
- **Not timing or carrier jitter.** Regressing each symbol's squared error
  on 1, |r|^2 and |r[k+1]-r[k-1]|^2 puts ~90% on the constant term, ~10% on
  phase and ~0% on the timing derivative. Radial and tangential errors are
  equal and flat with amplitude.
- **Equalizer.** A widely-linear ISI fit shrinks the error 0.62 -> 0.51
  (+/-3), 0.46 (+/-8), 0.40 (+/-16), 0.335 (+/-24) and no further (+/-40):
  ISI spread across exactly a 48-tap T/2 span, the size of MICA's 6910 FIR,
  where the channel needs ~10 symbols. The ISI is the same in every
  1024-symbol window from the first data symbol on (change 0.015), so it
  comes out of training and does not grow in data. The remaining 0.32 is
  coloured but not static-ISI, consistent with tap jitter (a large LMS step).
- **Not training length.** TRN 512 / 2048 / 4096 gives 0.63 / 0.56 / 0.68 at
  the start of data, and all three reset at 476A.

Budget at 28800 (spacings squared): static ISI 0.28, unexplained 0.085,
mu-law 0.017. Next: dump the 6910 coefficient rows twice in data
(`mica_trace --dump`) to see tap jitter and the step size this mode uses. Also
bear in mind that every oracle compares C against the same emulated C53 core,
so a core arithmetic bug in the equalizer path would pass them all, as the XC
repeat-end bug did. Real MICA runs 33600 on real lines.

```sh
python3 tools/k56flex_upstream_rig/offline_rx.py UPSTREAM.ulaw TX.caller
python3 tools/k56flex_upstream_rig/isi_by_window.py CAPTURE.txt TX.caller 18.74 21.55
```

### Equalizer timeline and an 8 dB shortfall (2026-10-10)

Watches and dumps on the deterministic rerun (`--watch 0x8d24 0xdae7`,
`--writers`, `--dump`, `--callers`):

| Time (s) | Event |
|---|---|
| 16.395 | 8F31 = 0800 (preamble gate) |
| 16.494 | 6AB5/6AB7 zero the 192 coefficients (DM 0B80..0C3F); worker 8D24 -> 588D (known-reference TRN) |
| 16.494-16.653 | 4DD0/4DD6 full-vector block LMS (16-term gradient, `satl` by TREG1 = 4/5, 16-bit rounded store): ~200 updates per tap, from zero |
| 16.653 | worker -> 58F2 (decision slicer). **Fixed ~509 symbols: TRN 4096 ends at the same instant** |
| 16.916 | 8F31 = 0400; DAE7 sequential-tap sweep starts (699C/69A0), decision-directed |
| 18.697 | data starts; the sweep sticks at tap 33 (6A02 rewrites 0BA1) |
| 19.061 | 6B30 clears DAE7: equalizer frozen |

Coefficients after TRN: main tap 36 at ~7600, tails ~410 RMS (about -25 dB),
and the decision-directed sweep never improves them. DAB7 (the "predictor",
an echo canceller) has all-zero taps throughout: echo is ruled out.
5882 (the other known-reference worker) is never called.

`tools/k56flex_upstream_rig/ideal_lms.py`: an ideal complex LMS from zero on
the recorded upstream (25 taps at 8 kHz per symbol parity) reaches **0.22
spacings after the same 509 symbols** (best step) and 0.16 after 2048. Least
squares gives 0.13. MICA reaches 0.50-0.63 after TRN plus the 2 s sweep:
**about 8 dB worse than its own algorithm class should get.**

Line gain -10, -5, 0, +5 dB: MICA error 0.528, 0.533, 0.507, 0.523. Gain
control normalises the level, so the LMS step is not mis-scaled by level.

Emulator audit so far: SATL/SATH (right shifts), ZPR, EXAR, ADD16, ADDS,
APAC, LTA and MADS match SPRU056D. Latent bug in courier-emu: MADD and MADS
(and the TREG0 write at c5x_ops.ipp:1876) do not copy TREG0 into TREG1/TREG2
when PMST.TRM = 0, while LT/LTA/MAC/MACD do. Harmless here: PMST = 003B
(TRM = 1) during the TRN LMS.

Open: why MICA's 509-symbol LMS lands 8 dB short. Candidates: the 16-bit
rounded coefficient store (the 32-bit path at 4DBB is not taken: DM
90A4+0x19 is zero), the 4-point TRN as the only excitation, its front end
(4580 resampler/AGC) before the equalizer, or a remaining core fault.

### 6910 coefficients through TRN, sweep and data (2026-10-12)

`tools/k56flex_upstream_rig/dump_equalizer.py COMMAND.json OUTDIR T...` reruns
the saved deterministic command with `mica_trace --dump T` and compares DM
0B80..0C3F (192 words) between call times. Relative change of the whole
vector (norm of the difference over the norm of the final vector):

| call time (s) | rms | distance to final | step from previous |
|---|---:|---:|---:|
| 16.45 (TRN starting) | 347 | 1.06 | - |
| 16.6 (TRN LMS ends 16.653) | 794 | 0.35 | 0.84 |
| 16.8 | 934 | 0.12 | 0.27 |
| 17.2 (DD sweep running) | 1256 | 0.23 | 0.31 |
| 17.8 | 1258 | 0.23 | 0.02 |
| 18.4 | 1256 | 0.23 | 0.01 |
| 18.8 (data from 18.697) | 1033 | 0 | 0.23 |
| 19.0, 20.5 (frozen from 19.061) | 1033 | 0 | **0.000** |

- **No tap jitter in data.** The vector is bit-identical at 18.8, 19.0 and
  20.5 s, so the "large LMS step" explanation for the unexplained 0.32 floor
  is ruled out: during data MICA's equalizer does not move at all.
- **The training result is the data-mode equalizer, and the sweep makes it
  worse.** TRN leaves the vector at 0.12 of its final value by 16.8 s; the
  decision-directed sweep (16.9 s on) then moves it AWAY (0.12 -> 0.23) and
  grows the main taps (the 5624/4449 pair gains ~+1700/+1400 by 17.2 s).
  After that it creeps 1-2% per 0.6 s.
- **A discrete event at data start.** Between 18.4 and 18.8 s the main taps
  drop by ~1250/~1040 (about -22% overall, rms 1256 -> 1033) in one step,
  which lines up with the data start at 18.697 s rather than with gradual
  adaptation. Worth identifying which routine writes it (`--writers-range
  0xB80 0xC40 --from 18.5 --to 18.8`): a gain renormalisation is plausible
  and, if it is a fixed rescale, would explain a coloured, static floor.
- The structure is 12 rows of 16: three rows of two interleaved halves with
  a main-tap pair per row (rows 2/5/8/11), as expected for a T/2 complex FIR.

Next: run the `--writers-range` pass over 18.5-18.8 s, and test whether the
sweep's outward drift is the 8 dB shortfall (freeze the vector at its 16.8 s
value in a scratch run and compare the data-start error).

#### Who writes the coefficients at data start (2026-10-12, `--writers-range 0xb80 0xc3f --from 18.5 --to 18.8`)

This corrects the "discrete event at data start" reading above. Only two
instructions write DM 0B80..0C3F in that window, 699C and 69A0: the
sequential-tap decision-directed sweep. It is still running, visiting every
tap once per ~10.4 ms (18-19 writes per tap, 3564 writes), and it stops at
18.697 s. Nothing else rescales the vector.

What changes is the size of its updates. RMS step per visit:

| call time (s) | 18.50-18.55 | 18.56 | 18.58-18.60 | 18.62 | 18.64-18.66 | 18.68 | 18.70 |
|---|---:|---:|---:|---:|---:|---:|---:|
| rms step | 3.5 | 20 | **60** | 42 | 29 | 16 | 3 |

Main tap 0C04 is flat (7679 -> 7677) for seven visits, then falls 7677 ->
6312 (-18%) in the last ten (about -100 to -260 per visit); 0BA5 behaves
the same (6881 -> 5624). Total variation over the window is 3x the net change,
so the small taps also dither.

The onset is the call's phase 13 at **18.56 s** (`requested_status` 12288,
then 16384 at 18.66, phase 17 at 18.74): the signal our client sends
changes at that instant and the sweep, which was settled on the training
signal, suddenly sees large errors and pulls the equalizer away from the TRN
solution. So the final vector is the TRN result *plus* ~110 ms of
adaptation to our own startup/E signalling.

**Lead:** check what our client transmits from 18.56 s (E / startup-ack frames,
`ME_K56` `--startup-e-symbols 5`, `--startup-ack-frames 4`) against what
MICA's slicer expects at that point (constellation, level, scrambler state).
A symbol set the slicer mis-decodes there would corrupt the equalizer exactly
this way, and it is OUR transmit, to be inspected on our TX tap
(`txw.caller` from V34_DATA_TX_DUMP) before blaming the peer. Test: rerun with
the client holding the training pattern, or with the sweep's inputs frozen at
18.55 s, and compare data-start error.

#### What our client transmits from 18.56 s (2026-10-12)

`tools/k56flex_upstream_rig/startup_constellation.py UP.g711 TX.caller` aligns
the `V34_DATA_TX_DUMP` data points to the upstream recording (`tee_peer.py`; the
`mica_trace --capture` option is parsed but unused), fits a linear receiver on
the first 2000 data symbols (0.13 spacings error) and applies it backwards.
The data start aligns to 18.6925 s (peak/median 61).

| interval (s) | what we send |
|---|---|
| up to 18.55 | constant-modulus 4-point (radius 11.6 +/-0.2, 4 distinct points, cv 0.01): TRN |
| 18.55 - 18.69 | a 4x4 16-point grid at +/-3.67 and +/-11.0 (radii 5.0, 11, 12, 15.6; rms 11.6, the same power as TRN): the sixteen-point MP/MP'/E frames of `--parameter-mp` / `--upstream-data` |
| 18.6925 on | data, 28800 lattice (half-integer points, spacing 1, up to +/-15.5) |

So the change at 18.55 s is a change of constellation, not an error: V.34
10.1.3.2/10.1.3.8/10.1.3.9 allow 4 or 16 points per J, and the client
deliberately uses MICA's resident sixteen-point labels (PM 02A0). Power is
continuous across it. Two things stand out:

- TRN is 4-point but MP/E are 16-point. The spec ties TRN, MP, MP' and E to
  one constellation size signalled by J (Table 18), so this mix is only
  right if MICA treats them separately; the rig accepts it (MP0 CRC and the
  gates pass).
- The 16-point grid (3.67, 11.0) is NOT on the half-integer data lattice
  (3.5, 10.5/11.5): a data-lattice slicer sees up to 0.5-spacing errors on
  the outer rows. If the 699C/69A0 sweep slices with the data lattice during
  MP, that alone would drive the tap changes seen from 18.56 s.

Not yet decided: which constellation MICA's slicer uses during the sweep at
18.55-18.69. Our transmit looks as designed, so this is not a defect found in
our TX; the next step is in MICA: read what 699C/69A0 compare against
(decision table pointer) during phase 13, or run with the client holding 4-point TRN
through that window and see whether the 18% main-tap drop disappears.

#### The sweep's decision table is the 16-point grid from 16.9157 s (2026-10-12)

The sweep (699C/69A0, routine 6952..69af) only applies updates; decisions come
from the worker slicer at 58F2 -> 5903, which finds the nearest point by
subtracting entries of the DM table at **DAD3** from |I| and |Q|
(`lar ar5,#dad3`, squared distance via `sqra`, entry count-1 in DM 8D2E).
Dumps of DAD3.. and 8D2E, and a `--writers-range 0xdad3 0xdada` pass:

| call time (s) | DM 8D2E | DAD3.. | meaning |
|---|---|---|---|
| 16.45 - 16.8 (known-reference TRN) | 0 | 9159, 9159 | **4-point**: one quadrant point, (8.2, 8.2) in our units |
| 16.91575 | 0 -> 3 | 4096,4096 / 4096,12288 / 12288,4096 / 12288,12288 | **16-point** (1,3) grid, spacing 4096 = 3.67 |
| 17.2 - 20.5 | 3 | unchanged | |

The switch is one event: routine 1507 copies 8 words from PM **0298** to DAD3
(`tblr`, writer 151C; 8D2E set at PC 6B38). The 4-point alternative sits at PM
0292 (9159, 9159). It coincides with the start of the DAE7 sweep (8F31 = 0400,
16.916 s) and the call's phase 8/10 at 16.96/17.0.

**This explains the sweep's drift.** From 16.9157 s MICA slices against the
16-point grid, but our client keeps sending 4-point TRN, points (+/-8.2,
+/-8.2), until 18.55 s (previous section). 8.2 is 2.2 spacings from 3.67 and 2.8
from 11.0, so every decision is wrong by a constant 2-3 spacings in the same
direction. The sweep therefore pushes the main taps UP (+1700 on the main pair
by 17.2 s, the 0.12 -> 0.23 move away from the TRN solution) for 1.64 s. When
the client's 16-point MP arrives at 18.55 s the decisions become right, the
update size jumps (rms 3.5 -> 60) and the main taps fall back ~18%
(correcting the earlier "rescale at data start" reading). The final vector is
then a compromise between wrong and right decisions, which fits the 8 dB
shortfall against an ideal LMS.

Per V.34 Table 18 and 10.1.3.8, J gives ONE constellation size for TRN, MP,
MP' and E. MICA selects 16-point at 16.9157 s, so a compliant client sends
16-point TRN from that instant; ours is 4-point throughout (SpanDSP's V.34
modulator) and only MP/E are 16-point.

Next: make the client's TRN 16-point from the switch (scrambled ones, four
bits per symbol: Q2n Q1n select the quadrant-point, In rotates, per 10.1.3.8)
and re-run `dump_equalizer.py` and the 28800 ISI measurement. If the main
taps stay put after 16.9 s and the data-start error falls from 0.51-0.62
toward the 0.22 an ideal LMS reaches, this is the 28800 blocker.

#### Putting the report on the 16-point grid (2026-10-12)

Correction to the previous section's "TRN": the 512-symbol known-reference TRN
ends at 16.65 s, before MICA switches its slicer. What our client sends from
16.9157 to 18.55 s is the repeated 24-bit **report** (4-point differential,
`k56flex_training_tx.inc`), and that is what the 16-point slicer mis-judges.
Making it a real 16-point signal would change the report path (2 bits per
symbol), so the experiment keeps the bits and moves only the amplitude:
`K56_REPORT_GRID_SCALE` (MicaEmu `native/k56flex_training_tx.inc`, rebuild with
`tools/build_k56flex_training_tx.py`) scales the report onto a grid corner.
Corner (3,3) is 12288/9159 = 1.3417 times the 4-point radius, corner (1,1)
0.4472. Full deterministic rerun of the 28800 connect command, MICA's trellis
input (47B9 capture) against our transmitted points, `isi_by_window.py
18.74 21.55`, first window:

| report scale | startup | raw error (spacings) | ISI energy | after own +/-24 fit |
|---|---|---:|---:|---:|
| 1.0 (baseline, off the grid) | reaches data | 0.630 | 0.051 | 0.325 |
| **1.3417 (corner 3,3)** | reaches data | **0.497** | **0.033** | 0.362 |
| 0.4472 (corner 1,1) | reaches data | 0.850 | 0.058 | 0.597 |
| 0.75, 1.15, 1.5 (off the grid) | **never reaches data** (capture empty) | - | - | - |

- **MICA's startup depends on the slicer's decisions being exact.** Only the
  two scales that land the report ON a grid point reach data; three nearby
  off-grid scales do not. That is direct confirmation that the DAD3
  16-point table is in the decision loop for the report.
- Corner (3,3) lowers MICA's error 0.63 -> 0.50 (-2.0 dB) and the static ISI
  energy by a third (0.051 -> 0.033), and the equalizer vector now stays put
  after training (17.2 -> 18.4 s step 0.024 and settled at 18.4 s, where it
  used to drift 0.23 away from the TRN solution).
- It does NOT reach the 0.22 an ideal LMS gets, and 28800 still decodes zero
  bits (needs ~0.15). The non-ISI part ("after own fit") rose slightly
  (0.325 -> 0.362) as the ISI part fell, so a second contributor remains.
- Corner (1,1) is worse: the report then sits at -7 dB, so the level into
  MICA's AGC/equalizer matters as well as the grid.

Remaining candidates for the other 0.36: the equalizer's 16-bit rounded
coefficient store and step size in the sweep, the 4-point known-reference TRN
itself (9159 reference), and the 18.55-18.69 s MP/E frames. Result: partial
fix, kept behind the environment variable (default 1.0 = unchanged).

#### The known-reference LMS step (2026-10-12)

All with the report on grid corner (3,3) (previous section; control 0.497).
`tools/k56flex_upstream_rig/mica_trace_poke.patch` adds `--poke T ADDR VAL` to a
private copy of `mica_trace` (a control run reproduces 0.497 exactly);
`rerun_poke.py` drives it.

- The LMS (4D72..4DE9) rounds its gradient with a right shift = TREG1,
  loaded from the record cell DM 90B7. That cell is a copy (4C08) of DM
  **90A0**, which routine 1B94 rewrites at each of three stage starts
  (16.494, 16.5766 and 16.662 s) with a PM-table value: **5, 4, 4**. A
  larger value is a smaller step.
- Poking 90A0 just after a stage's write (it is copied at the next 4C08):

| change | result |
|---|---|
| stage 1 (shift 5) -> 3, 4, 6 | never reaches data |
| stage 2 (shift 4) -> **5** | reaches data, raw error **0.462** (was 0.497), ISI 0.031 |
| stage 2 -> 3, 6, 7, 8 | never reaches data |
| stages 2+3 -> 5 | never reaches data |

The shipped schedule is close to the edge of what the startup tolerates in
both directions, and the best single change buys only -0.035 (0.6 dB): the step
is not what holds MICA at 0.46-0.50 versus the 0.22 of an ideal LMS. What
remains is the amount of known-reference training itself: three stages of
~83 ms (earlier measured as a fixed ~509-symbol block LMS), after which MICA is
decision-directed. Ideal LMS reaches 0.16 only after 2048 symbols. Next: find
what ends each stage (counter or timer in the 1B94 / 4C08 schedule) and whether
a longer known-reference stage is reachable; earlier tests with TRN 512 / 2048 /
4096 gave the SAME end instant, so the end is MICA's own.

#### What ends each known-reference stage (2026-10-12)

The stage machine is a continuation chain: DM 90F8 holds the next handler
(`splk @78,#...`, page 121), driven from the 1B17 / 1B97 / 1BBB / 1BF4
handlers. A `--writers-range 0x9080 0x90c9` pass over 16.48-16.68 s shows:

- **Stage descriptors are PM records** (5 words copied to DM 9080..9084 by
  1B2F): 9080 = 5, **9081 = 10** (passes per stage), 9082 = 4, and **9083 =
  next-descriptor pointer**: 0xBDB5 at 16.4941, 0xBDC1 at 16.5766, 0 at 16.6621.
  Three records; the last (pointer 0) ends the chain, and the call-through at
  1BFE (`lar ar2,*; banz 1c05`) loops to the next while the pointer is non-zero.
- **Only two stages run the block LMS**: 90BA (the busy flag, set by 4DF2,
  cleared by 4C71) is 1 for 16.533-16.576 and 16.616-16.661 s. Within a
  stage 90BB (copied from 9081 at 4C0A) counts passes 10 -> 0, each pass ~17
  symbols (the 47-word block at 90BC against a 16-tap window): 2 x 10 x 17 =
  340 plus setup, i.e. the "~509-symbol block LMS" is the TRN-length-matched
  window, not a free parameter.
- The step shift written to 90A0 by 1B94 (5, 4, 4) is a PM-table lookup per
  record (previous section).

Lengthening it: poking 9081 (just after 1B36 writes it, before the 4C0A
copy) to 20/40 with the stock TRN = the LMS then trains past the end of our
4-point TRN on the report, and with TRN 512 -> 768/1024 the startup is not
reached even at the stock pass count (`TRN=768 rerun_poke.py` with 9081 = 10
never gets to data; 1024 reaches data but not a decodable one). So the
512-symbol TRN is tied to MICA's own schedule (report/MP timing), and both the
TRN length and the stage table would have to move together. The block LMS is
therefore bounded by TRN = 512 symbols unless the descriptor records in PM
(near BDA9) are changed, which means modifying MICA's firmware tables rather
than our transmit.

Where that leaves the 28800 upstream: the transmit side now sits at 0.46-0.50
from 0.63 (report on the grid corner, stage-2 step 5); the rest of the gap to
the ideal 0.22 is MICA's short known-reference training plus the DD sweep.
Remaining options are on MICA's side (descriptor table, or a lower-order
equaliser span than the 48-tap T/2 FIR that the channel overfits), or a
smaller, cleaner TRN reference so the same 340 symbols train better. (The
measurement tool `isi_by_window.py` now takes `ISI_SEARCH=N` to widen its
600-symbol alignment search.)

#### Patching the PM descriptor chain (2026-10-12)

`mica_trace_poke.patch` now also has `--poke-pm T ADDR VAL` (`load_program`).
Program and data space are different memories here (data 0xBDAA reads 0xF445,
program 0xBDAA reads 10), so the earlier data-space `--poke` cannot reach the
descriptors. The three records are 12 words at **BDA9, BDB5, BDC1**: `[5,
passes=10, 4, next, 90F9, shift, FFFF, FFFF, 0, 0, 0, 0]`, so the pass counts
are PM BDAA / BDB6 / BDC2. Patched at 10 s (the table is already resident), all
three set to N, with the stock 512-symbol TRN, 28800 connect, report on grid
corner (3,3); `isi_by_window.py` first window:

| passes N | startup | raw error (spacings) |
|---|---|---:|
| 10 (stock) | data | 0.497 |
| 11 | data | 0.787 |
| 12, 13, 14, 16, 17 | stalls in phase 13 (MP), never data | - |
| **15** | data | **0.457** (ISI energy 0.030) |
| 18 | data, undecodable (error 203) | - |
| 20 | stalls in phase 13 | - |

With TRN lengthened to match (+34 symbols per extra pass: N=12/546, 15/682,
20/852) every run stalls in phase 13 as well.

Reading: a longer MICA-side training is NOT a clean win. The failures are not
in training; the runs reach phase 13 (MP exchange, requested_status 4096 /
8192) and then do not advance to data (phase 17), i.e. the MP/acknowledge
frames are no longer accepted when the equalizer comes out of a different
training length. So the startup MP decode is already marginal with the
equalizer MICA produces here, and its pass/fail pattern across N is chaotic
(11 worse, 15 slightly better, 12-14 dead). Best case 0.457, ~0.04 better than
stock, again far from 0.22. A stable improvement needs the MP/E frames decoded
robustly first: they are 16-point on a grid MICA's slicer now agrees with, so
the next place to look is why MP acceptance (4096 -> 8192 -> 16384 status
sequence) depends on the equaliser at all, not more training.

## Equalized-symbol upstream report receiver

`k56flex_upstream.c` implements the report-control part of Draft 0.23 clause
11 at the equalized four-point I/Q boundary. It uses resident PM 0294's
quadrant labels and PM 02B0's differential mapping (5903/5961), **not overlay
8E**. Dibits enter the existing blind 24-bit report acquirer LSB first,
with continuous 18/23 descrambler history (5/23 is a diagnostic option).
Quarter-turn carrier ambiguity cancels differentially after the first point.
One-shot events expose the accepted report/extension and the subsequent
899F terminator. Requiring three consecutive 899F words is a conservative
local policy, not a recovered firmware threshold. Terminators before report
acceptance do not arm the handoff.

`k56flex_upstream_test` is in `tests/fast.list`. Retained sign-pair vectors
come from MicaEmu's research transmitter, independently checked against
original 5903/5961/1D5A and the resident mapping tables. They cover both taps,
four carrier rotations, and joining during TRN or partway through the report.
Additional bit-source tests cover all 24 record offsets, incomplete/corrupted
terminators, one-shot events, and per-call reset. Invalid coordinates,
constant quadrants, random quadrants and a wrong-tap stream are negative
controls. All six K56flex fast-list tests pass, and the upstream test passes
Clang's undefined-behavior sanitizer. The original-firmware check passed 16
report cases and 60 continuous transmitter blocks; this is **not recorded
PCM or hardware interop evidence**.

Regenerate and check:

```
python3 ../MicaEmu/tools/verify_k56flex_training_tx.py --output artifacts/k56flex-upstream/training-oracle.json
python3 tools/generate_k56flex_upstream_vectors.py ../MicaEmu
make k56flex_upstream_test
python3 tools/run_tests.py --filter k56flex tests/fast.list
```

No engine gates are wired from this decoder. PCM acquisition is now exercised
by the offline front end below; streaming clock/carrier tracking, MP/E, and
upstream DATA remain open. A successful report alone must not stand in for
the distinct training/status gates.

## Offline PCM acquisition through report recovery

`k56flex_upstream_pcm.c` supplies a batch-only 8 kHz linear-PCM front end for
one research profile: **3200 baud, low carrier, 128T S, 16T S-bar, 288T PP,
512T four-point TRN**, then the repeated report. These lengths are the
MicaEmu transmitter's experimental profile, not established universal
K56flex requirements (Draft 0.23 clause 11; V.34 10.1.3.6-8 defines the
preamble waveforms). No sample or symbol boundary, report value, or ideal
I/Q reference is supplied by the caller.

The receiver reuses `p3_demod.c`'s vendored V.34 matched-filter bank and carrier
mixer at T/2, with its blind CMA/AGC/timing corrections explicitly disabled.
It searches for the known zero-initialized TRN sequence (resident 588D/581B),
fits a nine-complex-observation FIR on 256 TRN symbols, and requires low error
on 192 **held-out** TRN symbols. Before decoding report bits it also validates
the preceding PP and S/S-bar against their waveforms through the same FIR.
Independent real I/Q columns absorb constant carrier phase and conjugate
orientation; the T/2 span absorbs fixed fractional timing. Neither report
acceptance nor the expected report content selects the acquisition hypothesis.
Returned sample indices are observation indices including filter delay.

`k56flex_upstream_pcm_test` is in `tests/fast.list`. Retained inputs under
`reference-data/k56flex-upstream/` are **synthesized research-transmitter PCM,
not hardware recordings**: 3200 baud and a 2400-baud negative control,
with source/build and file hashes in `manifest.json`. The transmitter's
point mapping was independently checked against original resident firmware
as described above; that does not independently validate every PCM detail.
Tests encode each channel waveform once to the selected G.711 law and expand
it for the receiver, never transcode between laws. All 36 positive cases pass:

- Both laws; prefixes of 0, 1, 7, 37, 159 and 317 samples.
- Clean wire; half gain with a modest three-tap channel and additive noise;
  quarter gain with inverted polarity.
- Worst unit-RMS errors: held-out TRN **0.0230**, PP **0.0234**, S/S-bar
  **0.0239**. These are fixture results, not hardware acquisition rates.

Negative controls reject 2400 baud, the wrong scrambler tap, silence, noise,
a carrier tone, and TRN/report or report alone without the preceding preamble.
A truncated report does not acquire; a report before its terminator is exposed
without an END event. Repeated calls reset correctly. All seven K56flex
fast-list tests and a Clang UBSan build of the PCM test (including the shared
front end) pass.

```
python3 tools/generate_k56flex_pcm_fixtures.py ../MicaEmu
make k56flex_upstream_pcm_test
python3 tools/run_tests.py --filter k56flex tests/fast.list
```

**Not live-ready:** the front end buffers a bounded batch (at most 10 s),
uses a fixed nominal carrier/rate, and looks backwards from TRN to validate
the preamble. It has no streaming state machine, clock/frequency tracking,
echo handling, MP/E receiver, or engine gate callbacks. Next: qualify carrier
and clock offsets, then implement streaming acquisition before assigning any
recovered event to a server training gate.

## Live probe from vmhost CT 117, 19 October 2026

**No K56flex CONNECT.** `root@vmhost.net.cryan.nz` is reachable while tower
SSH times out. CT 117 (`v90modem-dev`, 192.168.88.230) was fast-forwarded to
`e094ff76`, rebuilt with vendored libraries, and passed the PCM and V.8bis
regressions. Calls used account 2903, SIP port 5078, RTP base 14800,
`SIP_FORCE_PCMU=1`, and `ME_K56FLEX=probe`; no existing server was stopped.
Evidence is retained locally under
`reference-data/artifacts/k56flex-vmhost-live/` (logs, PTY, untouched G.711,
RTP CSVs, source patch and hash manifest). These are call evidence, not new
fixtures required by `make test`.

The first attempt to the obsolete 8416 route stopped before media with
`PJSIP_EUNSUPTRANSPORT`. `sip_modem.c` created only UDP, although PJSIP may
switch a large INVITE to TCP (RFC 3261 18.1.1). Asterisk's TCP 5060 is
reachable. Creating both transport factories fixes the call setup: subsequent
calls to the current RasFinder route **6311** reach confirmed SIP media with
PCMU passthrough. This changes SIP signalling only, not RTP or codewords.

`k56flex-live-6311-1`: our own TX tap first checked independently. At 0.6 s,
1375 and 2002 Hz each account for about 0.502 of a 100 ms block's energy;
at 0.8 s, the 400 Hz tail accounts for 0.880. RX at 0.8 s contains a much
quieter copy of our CRe (echo), followed around 2.6 s by a strong 2100 Hz
answer tone (0.981 of the block energy). No CRd/MS is accepted. The probe
then reaches gate A without a peer status event. The 750 received RTP
packets have consecutive sequence numbers and 160-sample timestamp steps.
Do **not** infer a peer defect or lack of firmware support from this: its
K56flex capability/configuration has not been independently confirmed.

The call also exposed our premature timeout: at +15010 ms the generic V.8
timer hangs up while the K56flex probe is still waiting at gate A. SIP's
answered callback arms that timer even though ordinary V.8 has not started.
Draft 0.23 clause 4.1 puts K56flex V.8bis before V.8; its probe has its own
bounded gate timer. The generic V.8 timer now excludes active K56flex
preflight/probe contexts, and ordinary fallback still starts a fresh budget.

`tools/test_k56flex_probe_timeout.py` reproduces the engine bug with
non-idle noise and the real early-media/answered callback sequence via
`v90_engine_replay --sip-answer`. Before the fix it observes the erroneous
+15020 ms timeout; after it, the probe expires at +16660 ms and fallback
V.8 gets another 15020 ms. A media-connect-only replay missed the bug,
which is why the extra replay option exists. The new fast-list regression
and all seven existing K56flex rows pass (eight total).

`k56flex-live-6311-2` validates the timer fix on a fresh bounded call: no
CRd, probe expiry at **+16647 ms**, then V.8 expiry at **+31666 ms**. No
K56flex DATA, no forced gates, and no modem CONNECT. This call has variable
short PCMU packets and RX RTP stops after about 10.58 s; do not mistake the
subsequent receive-side idle samples for continued peer transmission or use
it as a demodulator A/B against the first call.

For physical-hardware qualification, confirm/enable K56flex on an analogue
peer and obtain a genuine V.8bis exchange. The native HSF experiment below
now provides a software peer for the earlier startup work; it is not evidence
that the RasFinder hardware supports the selected mode.

## Native legacy HSF peer: post-ACK startup failure

The user's suggested `../cruft/native/hsf` peer is available locally. This
is the static native translation of the **legacy K56 pump**, not modern HSF
7.12's V124 path that silently selects V.90/V.34. Its Flex-only client has
separately qualified binary duplex against MICA, so it is a useful independent
software peer (not a physical modem oracle). No cruft source or DSP decisions
were changed for this experiment.

First, the existing `native.hsf.test_v90modem_peer` was rerun against the
current checkout: CRe/CRd and CL/MS/ACK complete through the original tone
and FSK pumps in 229 blocks of 128 samples (3.664 s). This validates the
nonblind V.8bis boundary, not the rest of our engine.

`tools/k56flex_hsf_pair.py` now connects that normal Flex-only client to the
**whole `audio_sock_modem` engine** through byte-exact PCMU. The engine DTE
dials before socket connect so it takes the same digital/calling role as
the RasFinder attempt. Both sides exchange exactly 160 octets per iteration;
the client's existing 128-sample buffering supplies a causal delay. The
engine's media clock permits a bounded faster-than-real-time diagnostic.
No peer state, receive gates, role overrides, or ideal I/Q are injected.

```
make audio_sock_modem
python3 tools/k56flex_hsf_pair.py --cruft ../cruft --seconds 35 \
    --output artifacts/k56flex-hsf-engine-new
```

Observed against the current probe path:

- Our engine completes V.8bis at **+3640 ms**, accepts the original HSF MS
  `11 c9 80 80 81 c0 09 b5 02 00 94 81 81 02 47 85`, and sends ACK1.
- HSF accepts ACK at client block 223 and starts its original **V.8** pump.
- Our `me_k56_progress_locked()` instead starts the **PCM transmit probe
  immediately**, skipping V.8 and Phase 2. HSF sees no V.8 result, never
  starts its K56 pump, and has neither TX nor RX training completion.
- Our probe waits at gate A, expires at **+16900 ms**, and only then starts
  ordinary V.8. That also yields no common mode in this run, and the engine
  hangs up at **+31920 ms**. No DTE CONNECT or payload is reported.

This is **our earlier startup boundary**, not evidence that the HSF receiver
rejects the probes, cannot acquire TRN, or has a broken report decoder: its
high-speed pump was never started. The nominally successful V.8bis exchange
is not permission to jump straight to the PCM probes. Draft 0.23 clause 4.1
puts V.8 after V.8bis; the independent original controller follows that path.
The server's V.8 direction and Phase-2 digital role must also be handled
explicitly rather than assuming they follow which side initiated the socket.

Retained local evidence: `reference-data/artifacts/k56flex-hsf-engine/`
contains the whole-engine log, untouched codewords both ways, DTE bytes,
original client events, V.8bis-only verification, and input binary hashes.
Driver-derived captures remain local, not published as regression fixtures.
`verified_connection` is false. The handoff correction below supersedes
this direct-to-probe baseline; it does not qualify DATA.

## Negotiated V.8 / Phase-2 handoff to native HSF

`k56flex_startup.c` now owns the post-ACK startup for an actually agreed
K56flex `probe` call. V.8bis 9.9 assigns V.8 roles from the **MS direction**,
not SIP dial direction: our CL receives MS, so V.8 answers; a received CL
and locally sent MS selects V.8 calling instead. The digital common Phase-2
prelude remains answer-shaped (2400 Hz A, standard 49-bit INFO0), separate
from both V.8 direction and V.90's 62-bit INFO0d. Only common V.8 mode V34
is offered: the original Flex-latched client maps that startup mode to its
**K56** object, not a V.34 DATA pump (Draft 0.23 4.1).

The isolated context waits for actual V.8 completion before creating the
Phase-2 receiver/transmitter. It requires durable CRC-valid INFO1 flags,
a positive measured INFO1 selection, and completed local INFO1 transmission
before releasing PCM probes. No primary-channel DATA callback can connect
the DTE. Failure is bounded by received samples (15 s V.8, 30 s Phase 2,
local experimental limits) and fails closed. Disconnect frees the preflight,
startup and probe objects; probe-log state is per call, not function-static.
The exchange-only `ME_K56FLEX=1` policy is unchanged.

### Superseded short-probe interpretation

**Correction:** the short-probe workaround and startup-pass claim below were
superseded by inspection of the original HSF receiver. See "Corrected native
HSF qualification" below; `DEMY` alone was a false-positive handoff criterion.
The following records the intermediate experiment, not current behavior.


The first native run after fixing V.8 starts the real K56 pump and exchanges
INFO0, but misses INFO1c. The original HSF client sends L1 for about 160 ms,
then its **109-bit INFO1c**, inside SpanDSP's ordinary 420 ms L1/L2 receive
window. Waiting for that window to end loses the actual message. Simply
arming INFO1 early recovers CRC residue zero but leaves the ordinary L2-only
measurement analysing INFO rather than the probe, yielding an unusable zero
projected rate. That intermediate run is not a valid handoff result.

The explicitly enabled `v34_set_k56_phase2()` profile now receives INFO1c
concurrently with the probe, preserves its durable acceptance, and measures
the interior 40..160 ms of the observed L1-only probe. Its amplitudes are
normalized by the standard half-amplitude L1/L2 ratio before the existing
measured selection code (V.34 10.1.2, Tables 15/16). Plain V.34's L2 window,
framing, and selection are unchanged. This **research profile is scoped to
the observed legacy HSF client**, not a universal recovered K56flex clause.
The native run selects 2400 baud / projected 9600 bit/s in the common INFO1;
that projection is not a K56flex DATA rate or proof that all proprietary
Phase-2 semantics match MICA's different profile.

### Result and regression

```
make audio_sock_modem k56flex_startup_test
python3 tools/k56flex_hsf_pair.py --cruft ../cruft --seconds 35 \
    --require-startup --output artifacts/k56flex-hsf-startup-new
```

The final run passes `--require-startup`:

- HSF reports V.8 intersection **1**, selects its original **K56** pump,
  and retains `active_modulation=k56`.
- Our receiver accepts INFO0 and the 93 protected INFO1c bits with CRC
  residue **0**. HSF accepts our INFO1a and moves from `CALL_INFO1A` to
  `DEMY`, its Phase-3 receiver.
- At **+11960 ms** our engine starts PCM probes; at **+25220 ms** the
  still-unwired peer gate A expires and the negotiated call ends, without
  returning to V.8/V.34 DATA. Neither native training completion event,
  DATA, DTE CONNECT, nor a checked payload is claimed.

A 10-second negative control has already agreed V.8 and selected the K56
object, but is still in `CALL_L1PROBING`; `--require-startup` correctly
returns failure. Selecting K56 alone is therefore not the pass condition.

`k56flex_startup_test` covers both V.8 roles independently of Phase-2 role,
fragmented blocks (7 and 160 samples), missing-peer timeouts and fresh calls.
It never treats elapsed time as INFO1. All nine K56flex fast-list rows pass.
The ordinary V.34 phase-2 decoder, INFO1 validator, 2400/9600 duplex in both
laws, and data codec regressions pass (five rows). The startup module/test
also passes UBSan; the linked vendored archive was not sanitizer-instrumented.
The changed private header caused all six V.34 objects to rebuild.

Final and intermediate native traces remain local under
`reference-data/artifacts/k56flex-hsf-startup/`. No translated-driver assets
or driver-derived captures are published. Next: inspect the native client's
actual Phase-3 response and recover the gate/report handoff; do not satisfy
gate A just because Phase 2 completed.

## Corrected native HSF qualification (19 October 2026)

The original receiver's `DEMY` label is ambiguous: handler `1203a1df` can
reject low measured power and enter RX state **0x28**, handler `1203b470`,
without ever enabling the symbol receiver. The former `--require-startup`
criterion accepted that rejection. The baseline above therefore does **not**
prove successful Phase 2, regardless of our CRC-valid INFO1 or probe-start log.

`tools/k56flex_hsf_pair.py` now writes read-only original-pump observations to
`client-phase2.json`: RX state, handler, symbol-path flag (`RX+66c0`), classifier,
symbol-receiver selector, measured power and configured limits. A handoff now
requires V.8bis agreement, V.8 intersection 1, K56 selection, our probe-start
log, and an observed symbol-path flag of 1 outside rejection state 0x28.
`client_phase2_rejected` is reported separately. This remains an intermediate
startup diagnostic, not a successful connection or immunity to later fallback.

The concurrent INFO1/L1-only measurement workaround has been removed:
ordinary L2-only analysis and the full 420 ms receive window apply again.
`v34_set_k56_phase2()` now controls **TX CC-to-probe alignment only**. Per
V.34 11.2.1.2.5, the reversed Tone A must persist 10 ms on the wire. CC goes
through a nine-symbol shaper; L1 bypasses it. The K56 answer-shaped prelude
therefore drains approximately four bauds (53 samples at 8 kHz) of CC before
switching to L1, preserving continuous external sample accounting. This is
not a gain change, a fabricated INFO1, or an RX measurement exception.

Local completed diagnostic runs (each directory contains `report.json`,
`client-phase2.json`, engine log and untouched PCMU taps):

| Directory under `artifacts/` | Observation | Outcome |
|---|---|---|
| `k56flex-hsf-power-diag` | measured power -49.06 against -45 threshold; state 0x28, symbol path 0 | Phase-2 rejection, despite `DEMY` |
| `k56flex-hsf-normal-probe` | ordinary RX measurement; power -13.87, symbol path 1 | handoff seen, then V.34 fallback at 11.06 s |
| `k56flex-hsf-cc-tail` | CC-tail alignment; power -13.87, symbol path 1 at client block 672 | handoff seen, then retrain/fallback at 12.94 s |

An earlier `power-baseline` snapshot read the wrong power-object address and
reported 147.18; use the corrected `power-diag` measurement, not that value.
These scratch runs are not retained regression fixtures, and their binary
hashes differ: they are diagnostic history, not a controlled pass-rate A/B.
All report no TX/RX training completion, no DTE CONNECT and
`verified_connection=false`. The CC-tail run reaches the actual symbol path
but still does not establish a K56flex link. Next: inspect our transmitted
PCM prefix and the peer's classification/retrain at that boundary; do not
force gate A or treat the old gate-timeout result as current qualification.

## HSF INFO1a mode/law selection, 21 October 2026

The pending `v34_phase4_16pt_test` additions exercise V.90 Phase-2 wire
contracts (nominal power, final INFO0d fill, contiguous 109-bit INFO1d),
not a new Phase-4 defect. The existing Phase-4 J(16)/power regressions pass.
The native K56flex failure is an independent INFO1a interpretation error.

V.34 Table 16 assigns bits 40:49 to frequency offset. In the K56flex common
startup (Draft 0.23 clause 4.1), the original HSF classifier `1201efa0`
reads the same field from received INFO1a+0x30 as proprietary flags:
bit 0 enables K56, bit 1 enables high power, and bit 2 selects A-law (0 is
mu-law). It compares these against INFO1c+0x42, whose law is bit 3.
These bit meanings are recovered from that original classifier, not claimed
as normative V.34 semantics. A measured offset of -1 serializes as 0x3ff:
our mu-law server therefore advertised A-law. HSF rejected that law pairing
and initialized its V.34 symbol receiver before our PCM prefix arrived.

The fix sets this field to 1 only in the explicit mu-law K56 Phase-2
profile. Ordinary V.34 retains its measured frequency offset. No waveform
gain, G.711 coding, DSP table, scrambler, sample count or peer state changes.
`--require-startup` now also requires the original K56 symbol receiver
selector, so a V.34 symbol-path handoff cannot satisfy it.

Completed native runs, with the same peer-library and diagnostic-tool hashes:

| Local directory under `artifacts/` | Received server/client flags | K56 symbol path | Result |
|---|---|---|---|
| `k56flex-hsf-20261021-flags-control` | 1023 / 1 | no, receiver selector 0 | immediate V.34 fallback, 11.06 s; startup check fails |
| `k56flex-hsf-20261021-flags-fixed` | 1 / 1 | yes, receiver selector 1 | startup check passes; later retrain/fallback, 13.16 s |
| `k56flex-hsf-20261021-flags-confirm` | 1 / 1 | yes, receiver selector 1 | same later failure, 13.16 s |

The control disables only the two-line INFO1a override. Control and fixed
server TX taps are byte-identical through sample 87279 (10.91 s); their first
difference is within INFO1a. The native peer was not modified or forced past
a gate. Full reports, original state observations and byte-exact PCMU taps
remain in those disposable directories. A compact hash/state summary is
retained in `reference-data/artifacts/k56flex-hsf-info1a-20261021/summary.json`.

The serialized-field/CRC regression fails with the override removed and
passes with it restored. It covers unavailable, negative, zero and positive
frequency-offset measurements, plus disabling K56 on the same instance.
The focused runner passes 15 checks: seven K56 tests, Phase-4 generators,
MP, INFO1 validation, data mapping, and 2400/9600 and 3200/21600 duplex in
both G.711 laws. This is a Phase-2 interoperability correction only:
no training completion, CONNECT or verified payload is observed. The later
K56 training/retrain boundary remains open; no hardware interop is claimed.

## Native HSF Jd and audio-derived gates, 21 October 2026

The next failure was in our own transmitted handshake. The engine configured
`training_word=FFFF`, so the original HSF `120236a0` / `12035bf0` waited in
FindJD/CallWaitJ without starting upstream training. Send `8990` instead:
the recovered DBA8/B31D Jd exchange (Draft 0.23 clause 4.10), requesting
four-point upstream training as defined by V.34 10.1.3.3/Table 18. This goes
through the existing PCM scrambler and sample generator.

Before changing that word, an independent oracle checked our complete P1
and P2 against original HSF `12022500` / `120225d0`: 16368 and 8448 samples,
zero mismatches, with balanced sign histograms. The retained PCMU prefix and
stdlib checker are `reference-data/k56flex-hsf/downstream-probes-ulaw.*`
and `tools/k56flex_hsf_probe_check.py`. This checks our transmit tap, rather
than assuming the peer's probe detector is defective.

The server also lacked audio-derived progress at its first two gates.
Resident MICA `1258 -> 0FFE` posts `0800` after `13DE` S/S-bar acquisition;
`126C -> 0FFE` posts `0400` after `13BE` report acceptance. The incremental
receiver now observes V.34 10.1.3.7 S/S-bar to release DBA8, then validates
S/PP/held-out TRN and repeated report bits to release DB22. Waiting until
the full 512T TRN was a failed control: Jd continued into native echo training,
and HSF then failed to recognize our second identification. The early S/S-bar
event fixes that timing. Its 0.92 correlation floor is a local confidence
threshold, not a recovered ITU constant.

Original HSF `120342c0` sends one aligned `899F` J'a, followed by TRN.
The old local detector required three repeats. Accept one complete word at
the phase established by the accepted 24-bit report (V.34 10.1.3.4 also
defines J' as a single transmission; the K56 word is recovered behavior).
Unaligned, corrupted and partial markers remain rejected. Native report-only
and report-plus-J'a PCMU fixtures test incremental chunks of 1, 80 and 160
samples alongside the earlier 36-case impaired-waveform matrix.

The normal current engine completed this native run:

```
python3 tools/k56flex_hsf_pair.py --seconds 35 --dac-taps 65 \
  --output artifacts/k56flex-hsf-current-audio-gates-20261021 \
  --require-audio-gates
```

Only lockstep 8 kHz PCMU crosses between endpoints. Original-state reads are
diagnostics; they neither release gates nor configure a waveform. Engine
trace: S/S-bar at +16800 ms, report `00/89b0` at +18880 ms, and J'a at
+18960 ms. HSF recognizes the second identification, enters TRN2 and MP
receive (state 15), and emits repeated upstream MP. An independent offline
4-point MP decoder finds 161 CRC-valid, unacknowledged type-0 records in
client tap seconds 20.5..24.4 (3200 baud/high carrier, 1920 Hz).

**Qualification limit:** the server still waits at GATE_C and closes at
28.46 s. There is no training completion, CONNECT or checked payload.
The 65-tap option changes only the native client's analogue reconstruction;
byte-exact DS0 and the tool's 129-tap default are preserved. At default 129,
HSF still fails earlier at its Loop6 check (about 0.85 versus threshold
0.36); 65 taps passes that check (about 0.08). The original HSF library is
known to connect against the original MICA server at 129 taps, so this is
an unresolved interoperability limit, not grounds for changing that default.
The first two gates are fixed; live MP/parameter, E/B1 and payload remain
separate unfinished work. No hardware interoperability is claimed.

## Native HSF MP, B1 and checked duplex data, 21 October 2026

Continuation of the startup-only result above. The original receiver no
longer stops consuming audio at J'a. It differentially decodes the following
four-point MP using GPC, with strict V.34 10.1.3.9 Tables 20/21 sync, start,
CRC (10.1.2.3.2) and fill checks. Two identical MP records establish the
offer. One matching, CRC-valid MP' suffices: native HSF sends only one before
E/B1. Requiring two acknowledgements left the server stuck in PARAM_2 while
HSF had already completed transmit training.

Resident MICA `15BB -> 0FFE` posts `1000` after training; `15EE` posts `2000`
after MP acceptance; `15FF/160A` posts `4000` at the acknowledged exchange.
The server now uses the actual repeated audio MP to release GATE_C and the
first parameter wait, and the received MP' to release PARAM_2. These are
local receiver events; no native peer state or host callback releases a gate.
Deferring readiness until MP is conservative but works with this native peer.

HSF proposes upstream N=4 (9600), downstream code 11 (52000), mask `7FFF`,
16-state trellis, linear encoding and minimum shaping. Original `12023dc0`
uses downstream code `rate/2000 - 15`, unlike the ordinary V.34 rate field.
The server respects both its local PCM cap and that received limit, chooses
an enabled upstream rate, and fills the outgoing parameter record's upstream
rate/mask rather than the previous zero (cleardown) values. The native peer
accepts both parameter records and reports both training directions complete.

The local SpanDSP primary-channel receiver buffers audio from accepted MP,
then searches known B1 after MP' without fabricating an E event. V.34 clause
7 and 10.1.3.1 require GPC and independent B1 initialization here. A replay
fitted the GPC B1 template at 100%, yet the borrowed V.90 data path still
descrambled with GPA: its published bits were about 50% ones while the GPC
diagnostic was 100% marks. `v34_set_rx_data_scrambler(...,18)` now selects
GPC explicitly before capture/template construction; ordinary V.90 retains
its GPA default. Only successful B1 acquisition and PCM DATA permit host
CONNECT. The receive branch drains actual decoded bytes to the PTY.

The earlier DATA transition also had no source attached, so the mapper
stopped after six priming frames and the engine emitted DS0 silence. A local
word source now supplies continuous idle marks, then the normal data-stack
bits once online (Draft 0.23 4.12). This is not a callback between modems.
The native peer then holds carrier and decodes mapped idle exactly.

Completed 40-second native qualification `k56flex-hsf-duplex-payload-20261021`:
PTY CONNECT 52000, native carrier 3/data=1, actual 9600 up / 52000 down,
2048 deterministic binary serial bytes delivered exactly in **each**
direction, zero client framing errors. The tool now supports
`--payload-bytes N --require-payload`, compares the actual PTY/native host
outputs, drains packed idle marks, and rejects carrier interruptions; startup
or carrier observations alone never satisfy payload verification.

The final 40-second repeat `k56flex-hsf-duplex-8k-final-20261021` passes with
8192 exact serial bytes each way, zero framing errors and zero carrier
interruptions, using the loss-gated receiver. Reproduce with:

```
python3 tools/k56flex_hsf_pair.py --seconds 40 --dac-taps 65 \
  --payload-bytes 8192 --require-payload --require-audio-gates \
  --output artifacts/k56flex-hsf-duplex-new
```

Dependency control: rebuilding vendored SpanDSP from committed HEAD plus
only the receive-tap API fails before the native symbol handoff
(`k56flex-hsf-head-span-control-20261021`). Adding the K56 CC-tail alignment
restores 8192 exact bytes each way. A one-variable control restores HEAD's
short-probe RX behavior while retaining the tail fix and still passes
(`k56flex-hsf-tail-only-rx-control-20261021`). The commit therefore keeps
that receive behavior; its proposed removal and the V.90 nominal-power/fill/
INFO1d changes are not dependencies and remain separate. At the unchanged
129-tap default, the smaller build still fails Loop6 at 12.94 s
(`k56flex-hsf-tail-only-default129-20261021`). The
K56 tail also now handles its first-sample block boundary without returning
zero samples: the new waveform regression fails with that fix removed and
passes for requested chunks of 1, 7, 80 and 160 samples.

The 90-second, 32768-byte trial is a **failure**, not sustained qualification.
All 32768 downstream bytes are exact, but native HSF initiates renegotiation
around 56.5 seconds and loses carrier; the first upstream mismatch is byte
30602. The server does not yet implement K56 in-data renegotiation. Its
receiver now fails closed on upstream carrier loss instead of continuing to
deliver unframed noise. No native quality threshold or decision is overridden.
The receiver's slow carrier-loss flag alone did not stop this failure. A
local raw-data confidence check (four times settled decision error, floor
0.08, the same policy used for x2) now also ends the call. A completed retry
closes at 49.02 s after native renegotiation, with 14 corrupt trailing bytes
instead of continuing to emit noise for the rest of the 90-second trial.
`k56flex-hsf-quality-8k-20261021` still passes 8192 exact bytes each way with
that guard and zero carrier interruptions. Neither result solves renegotiation.

The new retained `upstream-mp-ulaw.*` fixture includes original Ja/J'a,
repeated MP and the single MP'. Tests cover 1/80/160-sample chunks, one-shot
events and fields, removal of the MP waveform, and negotiated rate/mask
limits. Shared V.34/V.90 regressions also guard the receiver handoff. All
media remains byte-exact G.711 with unchanged sample accounting, no added
gain, filtering or DSP-constant changes in the server path. Default 129-tap
native startup still fails at Loop6. No hardware or sustained-link claim.

## Phase-2 L2 window fixes default-129 startup and stability, 21 October 2026

The Loop6 failure was downstream of a corrupted **Phase 2 clock estimate**,
not incorrect P1/P2 PCM bytes. An unchanged original MICA server paired with
the same original HSF client (9600 upstream cap, default 129-tap analogue
reconstruction) connects. Its identification/P1/P2 bytes match ours; HSF's
clock estimate is about -0.5 ppm and its final P1 DFE metric is about 0.0009.
Our former estimate was +225.85 ppm and the metric about 0.85, above the
unchanged original Loop6 threshold 0.36.

Original HSF `1203c700` estimates frequency and timing slope from nine probe
tones, collecting into the probe-analysis object at RX+a94. It accumulates
timing slope at +2474 and window count at +2478; `1201f...` later seeds the
PCM resampler from these values. Read-only diagnostics now retain each
window in `client-probe-clock.json`. The first five windows are nearly clean.
In the sixth window the old server had already stopped L2 and was sending
Tone A: its fitted frequency jumps to -1.166 Hz and timing sum to 0.012492.
The corresponding MICA window stays on the probe and remains near zero.

Our own transmit tap confirms L1 from 9.35825 to 9.51825 s (160 ms), L2 to
9.91825 s (400 ms), then Tone A. HSF finishes its final measurement around
9.968 s. This is our truncated measurement stimulus, not grounds to modify
HSF's resampler, quality policy, Loop6 threshold or analogue reconstruction.

The fix changes only the explicit K56 common-startup answerer: emit 27
20-ms table periods of L2 (540 ms) instead of 20 (400 ms). K56 Draft 0.23
4.1 uses common V.34 Phase 2; V.34 10.1.2.4 and 11.2.1.2.6 permit L2 for up
to 550 ms plus round-trip delay. The new interval fits that bound without
estimating round-trip time and keeps the last HSF measurement inside L2.
Ordinary V.34 and V.90 durations, gain, tables, scrambler taps and bearer
sample accounting are unchanged.

At the default 129 taps, the sixth fitted frequency is now about -0.000079
Hz, timing sum -0.00000788, estimated drift -0.146 ppm, and P1 DFE metric
0.00093. Native HSF passes Loop6 and both training directions. A completed
40-second run delivers 8192 exact binary bytes each way. The isolated
mainline build (without pending V.90 TX edits or K56 RX-window removal)
then passes **120 seconds, 65536 exact serial bytes each way, 9600 up /
52000 down, zero framing errors and zero carrier interruptions**:

```
python3 tools/k56flex_hsf_pair.py --seconds 120 --payload-bytes 65536 \
  --require-payload --require-audio-gates --output artifacts/k56flex-new
```

No `--dac-taps` override, native peer modification, peer-control callback,
or gate injection is used. Host writes use the ordinary PTY and native
packed-8N1 queue, retrying partial writes. Only byte-exact PCMU crosses
between modems. The payload comparison covers all 256 octet values.

The generator regression checks L1/L2 waveform levels and emitted duration,
and unchanged ordinary V.34/V.90 choices. It fails with the scoped duration
change removed. All 22 focused K56 and shared V.34/V.90 regressions pass.
Retained compact results, probe-clock windows, hashes and before/after tap
comparison are in `reference-data/k56flex-hsf/l2-window-summary.json`.
Earlier Loop6/default129 and late-renegotiation failures above are superseded
for this clean local profile. Physical hardware, other line impairments and
handling an independently requested in-data renegotiation remain unqualified.

## Mainline integration, 13 October 2026

The `k56flex-rockwell-ns` work was consolidated onto main while retaining
the newer mainline GUI, fax and modulation documentation. The integration
adds Rockwell variable-offset V.8bis parameter parsing and corrects the
server law bit (Draft 0.23 clause 4.1; recovered WIN95AC client predicates
are cited in the implementation). All five fast-list K56flex regressions
pass after rebuilding with vendored SpanDSP. These are offline checks,
not qualification of a complete live K56flex connection.

## First probe of Eicon 7910 (2026-10-24)

Dialled 7910 (PCMU) from CT 117 with `AT+MS=K56,1` (`EICON_MS` overrides the
harness's `AT+MS` string), default and `ME_K56FLEX_ROLE=client`.  Our CRe goes
out at 0.5 s; no CRd (1529/2225 Hz) comes back in the 3.4 s window, V.8 then
completes as V.34 with LAPM, and no menu is reached.  The receive tap shows the
Eicon sending a 40 ms burst of 1375+2002 Hz (the CRe pair, at equal level) at
2.3 s, followed by 0.5 s of digital zeros, in a plain `AT+MS=V90` call too.
Whether that is a CRe truncated on the way (gateway tone handling) or the
Eicon's own start is not established; it is too short for our 120 ms detector.
Plain V.90 to 7910 also no longer works today: its JM offers V.34 only (PCM
unavailable), where the 8 October batch got 56000/31200.  The peer's
configuration has presumably changed.  We have no analogue K56flex DATA role
(`k56flex_client.c` is a receive model), so nothing past V.8bis can be
attempted yet.

### 7910 answers with PPP (2026-10-24)

`tools/eicon_ppp_direct.py` dials 7910 and runs `pppd` straight from CONNECT
(the peer assigns the addresses).  V.34 24000, LCP/IPCP up, peer 10.254.104.1,
**30/30 pings, 0 % loss**, RTT min/avg/max 422/557/1688 ms (the spread is
the interesting part: modem + 200 ms RTP jitter buffer + V.42).  CT 117 needed
`/dev/ppp`: host `/etc/pve/lxc/117.conf` gained `lxc.cgroup2.devices.allow:
c 108:0 rwm` and a bind mount for `/dev/ppp`, the host node was made 0666
(unprivileged root cannot open a 0600 root device), and `ppp` was installed
in the container (backup: `/root/117.conf.bak-20261024` on the host).
Evidence: `/root/ppp2` on CT 117.

PPP ping RTT vs the receive jitter buffer (`ME_JB_MS`, one 30-ping call each,
7910, V.34 24000, 0 % loss in all three):

| ME_JB_MS | min / avg / max (mdev) ms |
|---|---|
| 200 (default) | 422 / 557 / 1688 (319) |
| 40 | 268 / 291 / 334 (15) |
| 20 | 250 / 312 / 1330 (189) |

40 ms halves the latency and removes the tail; 20 ms brings the tail back, so
it is at the edge on this path.  Single calls, so a trend, not a setting: the
200 ms default stays because other rigs (Apple/ATA, AudioCodes) were tuned
with it.

### The jitter-buffer trade-off on 7900 (2026-10-24)

Latency is not free: lowering `ME_JB_MS` breaks V.34 startup.  7900 (A-law),
echo lines graded, from CT 117, calls alternated 200/60 in the last block:

| ME_JB_MS | V.90 | V.34 |
|---|---|---|
| 200 (default) | 15/15 (earlier batches) | 6/6 alternated + 10/10 + 3/3 control |
| 80 | - | 0/2 |
| 60 | - | 4/6 alternated (fails 2 and 5) |
| 40 | 5/5 | 1/5 |

The failure is always the same: Phase 2 completes our INFO1c, then the peer
never sends a decodable INFO1a (it holds a steady 2400+1800 Hz) while we sit
silent until the 60 s timeout.  Network arrival jitter never exceeded 23 ms,
our transmit stream had no gaps (rtp-tx.csv), and the receive tap carries no
concealed frames, so a starved buffer is not the cause; something in the
Phase 2 exchange depends on the loop delay.  Open lead: INFO1c timing against
the peer's reversal when the loop is 100-160 ms shorter.  Do not lower the
default until that is understood; PPP over 7910 shows what is at stake
(RTT 557 -> 291 ms).

**Fix: Tone A must be held, not glimpsed (V.34 11.2.1.1.7, 11.2.1.2.6).**  The
stall above was the call modem taking the answer modem's 50 ms Tone A +
reversal (11.2.1.2.6, followed by silence while it listens for our L1/L2) for
the Tone A that invites INFO1c: on a short loop that burst lands inside our
post-L2 wait, INFO1c went into the peer's silence, and both ends waited.
`post_l2_tone_a_seen()` now needs the 2400 Hz bin to hold >= 0.15 for four
consecutive 40 ms blocks (`rx.tone_a_run`); the real Tone A is held until
INFO1c arrives (11.2.1.2.8).  `V34_TONEA_RUN1=1` restores the single-block
test.  7900 V.34 at ME_JB_MS=60: 4/6 before, 6/6 after (alternated, same day);
at 40 ms still 0/6, so a second, unexplained dependence on loop delay remains
(there INFO1c now follows the real Tone A and the peer still never answers).
Regression at the 200 ms default: 7900 V.90 10/10 and V.34 10/10 with the fix.

**What the 40 ms failures look like on the wire (7900, with the Tone A hold in
place).**  No concealed RX frames (no isolated all-0xD5 20 ms blocks in any
tap), our Tone B reversals sit 36-46 ms after the peer's in passing and failing
calls alike, so neither starvation nor the 40 +/- 1 ms rule separates them.
The peer's own timeline does:

| | peer silence (listening to our L1/L2) | then |
|---|---|---|
| pass (JB 60, 200) | starts ~6.5 s, lasts 620-800 ms | Tone A, INFO1c, INFO1a |
| fail (JB 40, and 60 before the fix) | starts ~7.55 s, 440-460 ms, AFTER our L1/L2 ended | Tone A steady, no INFO1a |

In the failing calls the received probe energy continues 0.7 s longer (the peer
appears to send L1/L2 a second time, or our L1/L2 returns as an echo; the two
cannot be told apart in the tap because L1/L2 is the same signal on both ends),
and the peer's 50 ms Tone A + reversal comes only after our L2.  So the peer
runs its probe step twice and is not listening when our L1/L2 arrives.  Why it
repeats is not established: next try would be a pcap-level look at when the
peer's 11.2.1.2.5/11.2.1.2.6 steps complete against our Tone B, or sending
Tone B later (11.2.1.1.5 allows L2 up to 500 ms) at JB 40.

**The Tone B timing matters at a short loop (diagnostic `V34_L1L2_BLOCKS`).**
The call modem stops receiving the answer modem's probe after
`l1_l2_duration > 20` 20 ms blocks (8 of L1, so 260 ms of L2; the spec allows
L2 up to 500 ms, 11.2.1.1.5) and then raises Tone B.  7900 V.34 at
ME_JB_MS=40, with the Tone A hold in place, alternated calls:

| blocks | calls passing |
|---|---|
| 14 | 2/3 |
| 20 (default) | 0/6 |
| 30 | 3/3 |

So the failure is a timing coincidence between our Tone B and the peer's
probe/echo bookkeeping at this loop delay, not a shortage of anything on our
side: both a shorter and a longer analysis get through, the default does not.
The default is unchanged (200 ms buffer, 20 blocks); `V34_L1L2_BLOCKS=n`
overrides it for experiments.  Not enough peers here to pick a new value that
is safe for all of them.

**The probe-length sweep says "random", not "length" (same day, later).**  7900
V.34, ME_JB_MS=40, `V34_L1L2_BLOCKS` sweep, two calls each:

| blocks | round 1 | round 2 |
|---|---|---|
| 16 | pass | fail |
| 18 | fail | fail |
| 20 (default) | pass | pass |
| 22 | fail | fail |
| 24 | pass | pass |
| 26 | pass | pass |

The default passed here although it was 0/6 an hour earlier, and 16 flipped
between rounds.  Over everything at 40 ms the failure rate is roughly 40-60 %
and does not follow n; at 60 ms it is 0-30 % and at 200 ms none seen in ~40
calls.  So the stall is a race against something random whose window shrinks
with the loop delay, and the previous section's table overstates what the
block count does.  What we can see (the peer repeating its probe step) is
only the symptom; the peer's own view (`mlog`/`mantool` and the
`bri-test-answerer` journal on eicon420, which this session could not log in
to) is what is missing to say why.

### The card's own view of Phase 2 (eicon420, 2026-10-24)

eicon420 (login as the plain user; `sudo` is passwordless) runs the Diva card.
`sudo /usr/lib/divas/mlog -c 2 -o -b` traces controller 2 (7900), but it
needs a live stdin, else it spins on "unknown command" and writes tens of MB
per second: `(sleep 600 | sudo timeout 600 mlog -c 2 -o -b > /tmp/mlog2.txt)`.
The log has the DSP's `TRAINPROGRESS xxxx` steps and `Supervisor ...` decisions
with 1 ms stamps, and the Q.931 SETUP/DISC for each call.

For the answer modem's Phase 2 the interesting step is 0037 (Tone A sent, waiting
for INFO1c).  Every call, passing or failing, leaves 0037 exactly 648 ms
later: passing calls go to 003a (INFO1c accepted, then 003c/003e/0040...), failing
calls go to 0038 and 200 ms later `Supervisor retrain fallback 1 2c` (restart of
the probe exchange), which is the second probe step seen in the tap.  So at 40 ms
our INFO1c is not accepted inside that window.  The tap cannot say whether it
arrived late, early, or damaged; the DSP does not log the INFO1c bits.

Tone A hold in `post_l2_tone_a_seen()` at ME_JB_MS=40 (`V34_TONEA_RUN`, blocks of
40 ms): 4 blocks 3/4, 8 blocks 2/4 (so INFO1c 160 ms or 320 ms after Tone A makes
no difference), against 0/6 earlier with 4.  The knob stays a diagnostic.
Gateway/answerer facts found on the way: 7900/7910 are Diva controllers 2/1
(`/etc/bri-test/config.json`), the MultiTech BRI is controller 3, reached as
SIP 2910 by `diva-sip-gateway` (A-law only, new today); a call to 2910 is
cleared by the far end after 4 s (ISDN cause 0x90, no called number sent).

### First K56flex V.8bis exchange with real hardware (2026-10-24)

**7910 is a K56flex server that starts V.8bis itself.**  Every call to it
carries a 40 ms burst of 1375+2002 Hz at ~2.3 s (the CRe pair, then digital
silence): the gateway on that path clips the CRe, so our 120 ms detector never
fires.  Sending our own CRe (the default for a calling client) gets no reply.
Listening instead, with a 40 ms acceptance, completes the exchange:

    ME_K56FLEX_LISTEN=1 K56FLEX_DETECT_BLOCKS=2 ME_K56FLEX_ROLE=client
    AT+MS=K56,1   (tools/eicon_ppp_direct.py --law ulaw --mode v90 --ms AT+MS=K56,1)

3/3 calls: `K56flex V.8bis complete: ok at 6.1 s, ack=1`; default acceptance
(120 ms) 0/2 ("no CRe heard").  The card's MS, identical every time:

    12 c1 80 80 80 09 b5 02 00 94 01 83 43 55 e1

NS block: length 9, country b5 (USA), provider length 2, code 0094, licensee
0183, capability octet **0x43** (bit 6 = server, version bits 1 and 2 -- the
same shape as Patton's 0x42; `server_acceptable()` takes it).  After V.8bis the
call falls to ordinary V.8, whose JM offers V.34 and below, and we connect at
V.34 (21600 here, PPP 30/30): there is still no analogue K56flex data role to
take the call further (Phase 2/Phase 4 as a client, PCM receive).  That is the
next piece of work; the signalling it must answer is now observable on the card
with `mlog` (see above).  `ME_K56FLEX_LISTEN` and `K56FLEX_DETECT_BLOCKS` are
experiments, not defaults: our own server listens 3 s for CRe, so a calling
client that listens first would time it out.

**Bulk regression with today's V.34 changes (7900 V.90, default 200 ms buffer, 60 s
legs, `eicon_soak_test.py --test probe`).**  download 372,736 B 0 errors;
upload and duplex also `ok` (duplex 305,152 B out / 333,824 B in, 0 errors, longest
receive gap 0.32-0.38 s); evidence on CT 117 in `/root/soak1`.

**What the existing negotiated path does against the real card (7910, 2026-10-24).**
`ME_K56FLEX=probe ME_K56FLEX_LISTEN=1 K56FLEX_DETECT_BLOCKS=2 ME_K56FLEX_ROLE=client`
completes V.8bis (ack=1, card MS as above), then logs `negotiated K56flex: V.8
caller (MS sender), no fallback` and `post-ACK V8/Phase2 startup`, skips a real V.8
exchange (`[K56S] V.8 complete; enter digital Phase 2 (A/INFO0)`) and transmits
INFO0a/Tone A as the *digital* side; the card never answers and the call ends
without CONNECT after 47 s.  Two role decisions are wrong for this lab case
(v90modem is the K56flex client, the card the server):
- the V.8 roles: the card answers V.8 itself as soon as V.8bis ends (ANSam at
  +0.9 s after the V.8bis ACK, then JM with V.34 and below, no PCM bit), exactly
  as in the ordinary-V.8 run; there is no V.8 for us to skip;
- Phase 2: the client is the *calling* side (INFO0c, Tone B), not the digital
  A/INFO0 prelude the server path sends.
Without `ME_K56FLEX=probe` the same call continues through ordinary V.8 and
connects at V.34 with PPP.  The unfinished work is therefore the client path from
V.8 onwards (calling Phase 2 with the K56flex INFO bits, then PCM receive), which
the uncommitted `k56_phase2` work in `v34rx.c` appears to be heading for; it is
left alone here.

**INFO1c contents do not separate pass from fail either (`V34_INFO1C_DBG=1`, 7900
V.34, ME_JB_MS=40, eight calls).**  Passing calls 3, 5, 6; failing 1, 2, 4, 7, 8.
Every INFO1c carries power_reduction 0, additional 0, md 0 and the same per-baud
maxima (9/11/0/12/13/0, 3200 baud: low carrier, pre-emphasis 0, N 13); the only
fields that vary are freq_offset (1 or 2: passes 1,1,2; fails 1,2,2,1,2) and the
2400/2743 high-carrier flags, which are not used at 3200 baud.  Cheap hypotheses
now excluded for the 40 ms stall: concealed RX frames, RTP gaps, probe length
(sweep 16-26 blocks), Tone A hold length (4 vs 8), INFO1c contents.  What is left
is the unseen part of the card's INFO1c reception (its DSP does not log it) and
the exact arrival time of our INFO1c at the card.

**Smallest reliable buffer, 7900 V.34, Tone A hold in place (12 interleaved calls).**

| ME_JB_MS | pass |
|---|---|
| 80 | 2/4 |
| 100 | 4/4 |
| 120 | 4/4 |

Together with the earlier runs (60: 6/6 after the fix, 5/5 and 6/6 in two blocks;
40: roughly 40-60 %; 200: no failure in ~40 calls) there is no clean threshold:
80 ms is worse than 60 ms here, so the stall is a probabilistic race rather than a
step in the buffer size.  Four or five passes in a row is not enough to call 100 ms
safe (a 50 % failure rate would still do it one time in 16); the default stays at
200 ms.  Evidence: `/root/jbs`, `/root/jbfix`, `/root/jbab` on CT 117.

**100 ms, longer (7900, interleaved V.34 / V.90, 6 each, same build).**  V.90 6/6,
V.34 4/6; with the earlier 4/4 that is V.34 8/10 at 100 ms, so 100 ms is not a safe
default for V.34 either (200 ms: no failure in ~40 calls).  V.90 has passed at every
size tried (40 ms 5/5, 100 ms 6/6): the stall is in V.34 Phase 2, not in the V.90
path.  A buffer chosen per mode is not possible, because the mode is only known
after V.8, and the buffer is fixed when the call starts (changing it mid-call would
change the sample accounting).  Default stays at 200 ms.

**Long default-buffer batch (7900/7910, 32 calls, 8 rounds, build with every change of
2026-10-24, `/root/e79-long` on CT 117).**  7900: 16/16 (8 V.90 + 8 V.34, 10 exact echo
lines each).  7910: 0/16 at the echo menu (V.34 and V.90 alike), because it answers
with PPP and offers no PCM.

**Card-side step times are identical up to Tone A (40 ms buffer, mlog of three calls).**
From `Q.931 SETUP` the card reaches 0001/0003/0004/0009/0020/002a/0030 at
0/3/5/3719-3728/3757-3767/3886-3896/4061-4071 ms in the two passing calls and in
the failing one, and 0037 (Tone A, waiting for INFO1c) at 4302-4312 ms in all three
(within 10 ms).  Only after 0037 do they differ: 003a at +648 ms for the passes,
0038 at +648 ms and a retrain fallback 200 ms later for the failure.  So everything
before the INFO1c window looks the same to the card, in step timing, and everything
we can see of our own INFO1c (contents, start relative to Tone A) is the same too.
The remaining difference is in how the card receives our INFO1c itself.

### Fix: answer the answer modem's retrain while waiting for INFO1a (V.34 11.2.2.1.6, 11.5.1.2)

When the Diva card does not accept our INFO1c it leaves its 648 ms INFO1c wait, falls
back and restarts (11.5.2.1: silence, then Tone A).  We kept waiting for INFO1a until
the 60 s training timeout.  `modem_engine.c` now treats "still in the INFO1a wait
1.8 s after our INFO1c started, and the peer is holding Tone A (`rx.tone_a_run` >= 3
blocks, exported as `v34_get_rx_tone_a_run`)" as the peer's retrain and restarts
Phase 2 per 11.5.1.2 (`ME_V34_NO_INFO1C_RETRAIN=1` disables it).  V.34 caller only.

7900 V.34, ME_JB_MS=40, six pairs alternated, same build: **with 6/6** (the response
fired in 3 of the 6), **without 2/6**.  Default buffer (200 ms): V.34 3/3 and V.90 3/3
(10 exact lines each), never fired; fast tests 623/623.  This does not explain why the
card refuses our INFO1c about half the time at 40 ms; it makes the call recover from
it in one extra Phase 2 pass instead of timing out.  A short buffer (and the lower
PPP latency that goes with it, 291 ms against 557 ms) is therefore usable for V.34 in
these runs; the default is still 200 ms, because only 7900 has been tried.

**Confirmation with the retrain response (7900 V.34, 18 interleaved calls, `/root/mx`).**
40 ms 6/6, 60 ms 6/6, 100 ms 6/6: 18/18 with every buffer, the response firing in 5 of
them (so about one call in four needs it).  Before it: 40 ms about 40-60 %, 60 ms 4/6,
100 ms 8/10.  Default stays at 200 ms until another peer is tried.

**PPP over 7910 at 40 vs 200 ms (12 alternated calls, build with the retrain response,
`/root/pp`).**  Link up and pinging: 40 ms 4/6, 200 ms 4/6.  Round trip, min/avg/max:
40 ms 265-289 / 277-336 / 296-1292 ms (steady in three of four); 200 ms 411-432 /
435-701 / 453-2535 ms.  So the short buffer halves the latency without costing
reliability here.  Failures (3 of 12 were `pppd` LCP timeouts with the V.42 LAPM link up
on both sides and our ConfReq unanswered, at both buffer sizes: not the buffer; one was
a Phase 3 stall at 40 ms: INFO1a decoded, 55 s in `FIRST_S`, fell back to V.22bis
at 1200).  The LCP timeouts may be a race between the card's first LCP frames and the
harness starting `pppd` after reading CONNECT (the frames sent in between are read
and dropped by the harness); not yet separated from a modem-side loss.

**PPP over 7910 with the harness relaying the PTY (so the card's first LCP frames are not
dropped before `pppd` starts), `/root/pq`.**  Link up and pinging 7 of 8: 40 ms 4/4,
round trip 262-271 / 277-291 / 293-314 ms; 200 ms 3/4, 409-451 / 431-455 / 452-472 ms.
The LCP timeouts seen before were therefore the harness race: none in 8 calls now.
The one failure (200 ms) is a Phase 3 stall, three times in a row in one call:
INFO1a decoded, our receiver in `PHASE3_TRAINING`, our transmitter silent as 11.3.1.1.1
asks ("initially silent", waiting for S and the S-to-S-bar transition), while the card sent
one steady 4092-RMS signal for ~14 s (not S, S-bar, PP, TRN, J), then retrained; each
retry (the existing 11.5.1.2 response) ended the same way until the 60 s timeout, no
CONNECT.  Same shape as the 40 ms stall seen earlier (55 s in `FIRST_S`, V.22bis
fallback).  Not a buffer effect (it occurs at 200 ms too, 2 of ~30 PPP calls); not yet
explained, the next candidate for the 7910 V.34 path.

**The 7910 Phase 3 stall, captured and replayed (`/root/hunt/3`, PPP call at 40 ms, I/O
schedule recorded).**  `v90_engine_replay --sync --schedule io.bin` on its receive tap
reproduces it stage for stage (INFO1a decoded, `PHASE3_TRAINING` for 15 s, retrain,
three cycles, V.22bis fallback).  With `ME_V34_SPAN_FLOW_LOG=1` (replay only, never a
live call) the receiver's `Phase 3 PP acquire` lines show no lock at all: the phase
estimate wanders (4..46), scores 170-330, magnitude 0.5-1.9, hold counts jumping 1..43.
What it is not: lost or reordered RTP (none in the stalled call), concealed frames, or
a clock offset (the card's Tone A is exactly 2400.000 Hz in the received samples, as it
must be in a packet stream).  What it is: either the card's Phase 3 signal after its
INFO1a is not the S, S-bar, PP we are looking for (11.3.1.2.1 gives them after 70 ms of
silence), or our PP acquisition cannot find it.  The card's INFO1a differs from attempt
to attempt in the same call (pre-emphasis index 9, 0, 8, 0; power reduction 7+5, 2+0,
7+0, 2+0) although the line has not changed, so its measurement of our probe is noisy;
the one attempt that connected after a retrain in call 2 was a 0/2+0 one, but call 3's
0/2+0 attempts failed as well.  Next step if this matters: dump the first 600 symbols
after the card's INFO1a (`--p3-symbol-export`) on the replay tap and look at what the
card sends.

### Fix: retrain after 3.5 s without a Phase 3 lock, and allow retrains 120 s (V.34 11.3.2.1.1, 11.5.1.1)

About half of the 7910 card's Phase 3 attempts get no PP/TRN lock in our receiver, whatever
its INFO1a says (44 attempts: 21 fail for each of the INFO1a pre-emphasis/power-reduction
combinations alike).  A working attempt leaves `PHASE3_TRAINING` in 280-301 ms
(24 of 24).  We waited for the peer to give up (~15 s) and then answered its retrain, so three
or four attempts used the 60 s training limit and the call fell to V.22bis.

`modem_engine.c`: a plain-V.34 caller still in `PHASE3_TRAINING` after 3500 ms
(`ME_V34_PHASE3_TRAIN_TIMEOUT_MS`, 0 = off) retrains, and once a call has restarted Phase 2
the training limit is 120 s (`ME_V34_RETRAIN_TRAINING_MS`, 0 = 60 s).  Interleaved A/B on
7910 (PPP over V.34, 40 ms buffer, 40 calls alternated, `/root/hunt4`):

| | calls up | median / mean / max seconds to connect | attempts per success |
|---|---|---|---|
| with both changes | 20/20 | 42.7 / 52.8 / 113 (three calls over 90 s) | 2.10 |
| old behaviour | 18/20 (2 fell to V.22bis) | 51.6 / 48.6 / 70 (successes only) | 2.05 |

Per attempt the success rate is the same (the earlier worry that the early retrain lowered it
was a time-of-day effect in a non-interleaved run); what changes is that no call falls to
V.22bis and the typical connect is ~9 s shorter.  The price is a longer worst case (up to
two minutes of retraining instead of a V.22bis connection at 60 s).  The harness now waits
150 s for CONNECT and records the I/O schedule (`io.bin`) so a stall can be replayed.

### Root cause of the 7910 Phase 3 failures: the frequency offset in INFO1c (V.34 10.1.2.3.5)

In the stalled calls the card's signal after its INFO1a is wideband (about 30 % of the energy
above 3400 Hz, flat to Nyquist, constant RMS 4092 = random mu-law codewords); in healthy
calls it is a band-limited V.34 Phase 3 signal (8 % above 3400 Hz).  So the K56flex card
chooses between a V.34 and a PCM-style Phase 3, and our V.34 receiver cannot lock to the
latter (`Phase 3 PP acquire` never converges).  It chooses from our INFO1c.  The only field
of ours that varied between attempts was the 10-bit frequency offset: we sent the measured
1050 Hz probe offset in 0.02 Hz units, i.e. 1, -1, 2 (quantisation noise: the real offset on a
packet bearer is exactly zero), and after a retrain stale -282/-290 (-5.8 Hz).  Interleaved,
17 calls (`/root/hunt5-done`): every first attempt with a nonzero offset failed (5 of 5), every
one with 0 succeeded; forcing 0 (`V34_INFO1C_FO=0`) needed 1.12 attempts per call against 1.89
for the measured value.

Fix: `v34_probe_frequency_offset()` reports 0 for anything within +/-0.24 Hz (10.1.2.3.5 asks
for 0.25 Hz accuracy; `V34_INFO1C_FO_NO_DEADBAND=1` restores the raw value, `V34_INFO1C_FO=n`
forces a value).  Interleaved A/B on 7910, 8 calls per arm so far (`/root/hunt6`, 40 ms
buffer): dead band 8/8 up, 1.12 attempts per call, one call needed a second attempt; old
behaviour 7/8 up (one failed outright after six attempts), 2.50 attempts per call, five
calls needed more than one.  This is the likely cause of the "half of Phase 3 attempts fail"
seen all day, and of the earlier "INFO1c not accepted at short buffers" on 7900, whose
INFO1c carried the same 1/2 codes (that failure was fixed by answering the retrain and was
not re-tested with the dead band).

### 7910 INFO1c dead band: final hunt6 numbers (35 interleaved calls)

Dead band (`820c854f`): 18 calls, 0 FAIL, 1.11 Phase 3 attempts per call.
Old (raw offset): 17 calls, 2 FAIL (6 attempts each, 124 s), 2.76 attempts per
call. Rates reached 19200-24000 in both arms. The 7900 default-buffer
regression with the change is still to be run.

Regression with the INFO1c dead band (`820c854f`) against 7900 at the default
200 ms buffer, alaw: V.34 5/5 and V.90 5/5 calls complete with all 10 echo
lines exact, no INFO1c-retrain responses fired. (An earlier ulaw run of the
same script dialled 7910, which has no echo menu; it is not a 7900 result.)

Short-buffer check (ME_JB_MS=40, 7900, alaw, dead band in place): V.34 4/4 and
V.90 4/4 complete with all 10 echo lines exact. The INFO1c-retrain response
(11.5.1.2) fired once on every V.34 call and never on V.90, and each of those
calls still completed, so the dead band does not remove the case the response
exists for.

7910 PPP over V.34 with the INFO1c dead band, 16 calls alternating
ME_JB_MS=40 / 200 (hunt7): 16/16 connected and passed pings (21600-24000 bps),
8/8 at each buffer, 33-43 s to CONNECT. Server logs: 14 calls needed no
retrain; calls 8 and 16 (both 200 ms, the 42.8 s ones) each had one peer
retrain tone and recovered; no early-retrain or INFO1c-retrain responses. Before the
change the same call mix needed up to 6 attempts and failed 2 of 17.

Longer 7900 run with the dead band (alaw, default 200 ms buffer, 40 echo lines
per call): 7 V.34 and 7 V.90 calls, all 14 complete with 40/40 echoes exact; no
INFO1c-retrain responses fired. With the earlier 5+5 (10 lines) and 4+4 (40 ms)
batches that is 16 V.34 and 16 V.90 calls on 7900 with no failure.

What the 7910 retrain on hunt7 call 8 is: the first pass is a normal Phase 2
(INFO1c/INFO1a identical to the clean calls), then Phase 3 reaches
PHASE3_WAIT_S with our FIRST_S going out and no far-end S arriving; about 5 s
later the card sends its retrain tone, we restart Phase 2 (11.5), and the second
pass completes. So it is the known Phase 3 stall, not an INFO offer problem, and
the recovery already works (about 6 s lost, 2 of 16 calls). Not pursued further
without a capture of the stalled pass.

7910 hunt8 (24 PPP calls, ME_JB_MS=200, dead band): 23/24 connected (19200-24000
bps, 36-45 s; calls 15-17 took 43-45 s with one card retrain each). Call 19
failed with no CONNECT: the card's ANSam arrived at 4.0 s exactly as in the
neighbouring good call, our CM went out, and no JM ever came (V.8 timeout at
15 s, card kept sending a steady non-2100 Hz signal). Our TX RMS sequence is the
same as call 18 up to the point where 18 received JM, so this looks like a
card-side V.8 miss, 1 in 58 calls over hunt6-8 (not 1 in the dead-band arm
alone); not pursued without a second occurrence. Evidence: CT 117
/root/hunt8/19 (taps, io.bin).

7910 hunt9 (30 PPP calls, ME_JB_MS=200, dead band): 30/30 connected at
19200-24000 bps; 4 calls (9, 12, 16, 21) took ~43 s, the rest 36-37 s (the
longer ones each had exactly one peer retrain tone: the Phase 3 stall). The hunt8
V.8 no-JM did not recur. Dead-band calls since the change: 16 + 24 + 30 = 70,
69 connected.

Timing of the stalled pass (hunt9 call 9, stage trace): Phase 2 and INFO1 are
identical to the retried pass. Stalled pass: PHASE3_WAIT_S at +13.9 s with our
tx stuck in FIRST_S; no far-end S by the card's retrain tone at about +16.6 s
(2.7 s, inside our own 3.5 s early-retrain limit, so ours never fires; the card
always times out first). Retried pass: PHASE3_WAIT_S at +20.3 s, tx
FIRST_S -> FIRST_NOT_S at +22.0 s (1.7 s), far-end S detected at about +23.4 s.
So in the stalled pass the card never started its S (or started it outside the
band our detector watches); the difference between passes is on the card side
of Phase 3, and the cheapest recovery is the one already working.

7900 at ME_JB_MS=40 with 40 echo lines per call (alaw, dead band): 6 V.34 and
6 V.90 calls, 12/12 complete with 40/40 echoes exact. The INFO1c-retrain
response fired on 3 of the 6 V.34 calls (never on V.90) and each still
completed. 7900 total with the dead band: 22 V.34 + 22 V.90 calls, no failure.

Correction and replay of the stalled pass (hunt9 call 9, `v90_engine_replay
--dial --sync --schedule io.bin --echo-ref` on the call's own taps): the replay
reproduces the live stall to the millisecond (same stages, card retrain tone at
+16.63 s). It shows the earlier "the card never started its S" reading was
wrong. Spectra of the taps: the card transmits a continuous wideband Phase 3
signal (RMS ~3270) through the whole stalled window, just as in the retried
pass. OUR transmit is exactly silent for that whole window, by design: the V.34
caller (`get_v34_call_phase3_wait_baud`, 11.3.1.x) stays silent until the
answerer's lead ends in J, then sends S. In the retried pass our S starts 1.5 s
after INFO1 (card lead ended, J seen); in the stalled pass the card's lead ran
>2.7 s with no J on our side and the card gave up first. So the open question is
whether the card's J is missing or our receiver fails to detect it in that pass.
Next step: p3_demod/vpcm_decode --p3 on the stalled window of
/root/hunt9/9/live-rx.g711 (taps copied to /tmp/h8/c9 on the Mac, not retained).

`vpcm_decode --p3` on the stalled window (13.3-16.7 s of call 9's rx tap) and on
the retried window (19.8-23.2 s) is inconclusive: both give SNR near 0 dB and
hundreds of spurious J/TRN "segments" under every baud/carrier hypothesis, and
the stalled window's best hypothesis (3200 baud high, 1920 Hz) is not the
negotiated card-to-us carrier (3200 low, 1828.6 Hz). The lightweight scan cannot
settle whether a J is on the wire. A decision needs the real receiver at the
negotiated carrier on those windows (v34 receiver with the negotiated Phase 3
parameters), not this scan.

7910 carrier lead (hunt7-9, 69 calls): the card-to-us carrier our first pass
negotiated for 3200 baud was high in 31 calls (0 retrains) and low in 38 (9
retrains, 24%). It is our own INFO1c request (L1/L2 probe result per baud, so it
varies call to call), and the card's Phase 3 signal centres near 1908 Hz, closer
to the high carrier (1920 Hz) than the low (1828.6 Hz).

A/B `V34_INFO1C_FORCE_HIGH=1` (INFO1c asks for the high carrier wherever the
rate is offered and high is supported), hunt10 interleaved, first 34 of 40
calls: base 17 calls / 0 FAIL / 2 retrains (7 of the 17 negotiated low; 2 of
those retrained; the low calls ran at 21600), forced-high 17 calls / 0 FAIL /
1 retrain (call 30, so high is not immune), every call high and 24000. The
stall rate difference (2/17 vs 1/17) is not significant; the clear effect is
rate: forcing high removes the 21600 low-carrier calls. Env knob only, default
unchanged.

hunt10 final (40 calls, interleaved): base 20 calls / 0 FAIL / 2 retrains /
24000 on 11 (21600 on the 9 that negotiated low); `V34_INFO1C_FORCE_HIGH=1`
20 calls / 0 FAIL / 2 retrains / 24000 on all 20. So the carrier choice does not
cause the Phase 3 stall (2 vs 2); what it costs is rate: the low carrier
costs 2400 bps on this rig. 7900 with the knob set: 6 V.34 + 6 V.90 (10 echo
lines), 12/12. Left as an env knob, not the default: the probe's low choice is a
real measurement and nothing here shows it is wrong on a real line, only that
the 7910/7900 rig reaches 24000 at the high carrier.

hunt11, `V34_CALL_P3_S_AFTER_MS=1800` (caller starts S after 1.8 s of silence
instead of waiting for J; off by default; V.34 11.3.1.1 has the call modem
silent until J, so this is an experiment only). 40 calls interleaved: base 20 /
0 FAIL / 3 retrains; timer 20 / 0 FAIL / 1 retrain (call 24, so the timer does
not prevent it). Our TX tap shows the effect is small: the post-INFO1 silence is
2.36-2.40 s on base calls and 2.17 s on timer calls, i.e. the J normally ends
the wait only ~0.2 s after the timer. (V34_TX_LOG lines do not reach server.log
on these runs, so "the timer fired" had to be read from the tx tap.) 3 vs 1
retrains is not significant. The stalled pass's silence is >3 s, far past the
normal 2.4 s, so the lead is late or its J is missed, which the timer at 1.8 s
should have bypassed; call 24 retrained anyway, so in that call the card was not
waiting for our S. Conclusion: not a TX-timing fix; left as an env knob.

Card-side view of 7910 (attempt, 2026-10-25): `sudo /usr/lib/divas/mlog -c 1 -o -b`
on eicon420 during 14 PPP calls to 7910 captured no Q.931 or TRAINPROGRESS lines at
all (header only), so the MultiTech ISI card is not on Diva adapter 1 and the DSP
step trace used for 7900 (controller 2) is not available for 7910. The 7910 stall
therefore cannot be seen from the card side with the tools on eicon420; the J/PP
question remains answerable only from our own taps. (The calls from that attempt
were also lost: a second launch of the hunt script deleted the first run's output
directory.)

hunt12 (14 PPP calls to 7910, base build, ME_JB_MS=200): 14/14 connected; 6 high
and 8 low first-pass carrier (24000 on every high call, 21600 on every low call);
one retrain (call 2, low, 42.8 s). Retrain totals by first-pass carrier across
hunt7-12 are still skewed to low (low carrier ~10 of ~46 calls, high ~1 of ~37 in
the base arms), but forcing high (hunt10) did not remove them, so the carrier is
a marker, not the cause.

V.92 mode (AT+MS V.92 offer) against 7900, alaw, default buffer, dead band:
4/4 complete with 10/10 echoes exact (call 1's log: V.8 result mods=V90|V34, "V8 selected V90 analogue role", so
the card answers a V.92 offer as V.90 and the call ran as V.90, not V.92).

7910 PPP soak (3 calls x 300 pings over ~5 minutes, V.34 21600-24000, JB 200 ms,
dead band): 900/900 echoed, 0% loss on every call, RTT min/avg/max
403/432/455, 420/440/471 and 404/433/472 ms (mdev ~10 ms). No mid-call retrain
or hangup. This is the first 7910 data-quality figure; earlier runs only sent 3
pings. Default buffer; at 40 ms the earlier 3-ping runs showed ~290 ms.

7900 long soak (alaw, default buffer, 400 echo lines of 201 bytes per call, ~35
minutes each): V.34 400/400, V.90 #2 400/400, no retrain in either. V.90 #1
echoed 282 lines clean over 31.5 minutes, then the card sent Tone B (we logged
"peer Tone B; 9.5.2 retrain 1 (3200/31200)"); we answered per 9.5.2.2, ran
Phase 2-4 again in 12 s and re-entered DATA (upstream 31200, downstream 56000).
Line 283 was in flight and was never echoed, and the batch harness waits 40 s for
an echo and does not resend, so it recorded a failure. One card-initiated
retrain in ~100 minutes of V.90 data; our side recovered. Not a modem defect as
far as these logs show; the harness could resend after a retrain if the 7900
echo menu were to be used for longer runs.

## K56flex client path: where it stands (2026-10-25, started at the user's request)

Re-checked against the live card with `ME_K56FLEX_LISTEN=1 ME_K56FLEX_ROLE=client
AT+MS=K56,1` (shell exports, not inlined): the 40 ms CRe burst is on the wire at
2.3 s and V.8bis completes with `K56FLEX_DETECT_BLOCKS` 1 and 2, 4/4 (the card's
MS licensee octet is 0x81 or 0x83, otherwise identical:
`12 c1 80 80 80 09 b5 02 00 94 01 8x 43 55 e1`). The first attempt of the day
missed the burst (default 3 s window, 0/1) and a further attempt without the knob
would too, so any change here must keep the 2-block acceptance explicit.

What is missing, in order, for an analogue K56flex data role against 7910:
1. After V.8bis the card still answers ordinary V.8 (JM V.34 and below, no PCM
   bit), so the K56flex indication must travel in Phase 2 (INFO0/INFO1 bits) or in
   a K56-specific Phase 2/3 we do not decode; the card's INFO0a/INFO1a under
   `AT+MS=K56` have not been compared with a plain V.34 call (the standalone
   `vpcm_decode --phase12` does not decode the INFO bits from these taps).
2. A calling-side Phase 2 that carries the K56flex bits (k56_phase2 exists only
   for the answering/server role: `v34tx.c` l2_cycles/l1_l2_signal_init).
3. A PCM downstream receiver for the client (the receive model in
   `k56flex_client.c`/`k56flex_rxfe.c` runs only on our own server's output).
Step 1 is the cheapest: log both INFO0a/INFO1a frames from the card under K56 and
under plain V.34 and diff the reserved bits. Nothing in the tree was changed for
this section.

The card's INFO0a under `AT+MS=K56` (V.8bis acked) and under plain V.34 is
byte-identical (`V34_INFO0_DBG=1` prints the received frame, 2 calls each, all
`ff 21 e2 37 9b 00 00 00`). So INFO0a does not change with our K56flex offer and
carries no K56flex indication that we can see; whatever makes the card a K56flex
server is not in that frame. Remaining candidates: INFO1a, or a K56-specific
Phase 2 that the card only starts on a signal we have not sent.

INFO1a from the card (`V34_INFO0_DBG=1` also prints it as `[INFO1ARX]`), 3 K56 and
3 plain V.34 calls: the common frame is `2f e0 34 09 a0 b2 66` (K56 calls 1, 2, 3,
and plain 1, 3); the other frames (`02 20 34 09 a0 8e fa`, `2f 20 36 09 20 db 4f`)
appear once each, one K56 and one plain, and track our INFO1c request (they differ
in the carrier/pre-emphasis fields), not our K56 offer. So neither INFO0a nor INFO1a
differs with the V.8bis K56flex exchange: the card signals nothing K56flex-specific
in V.34 Phase 2 as a result of our offer. **Superseded, see the next section:** the
frame comparison missed that every card INFO1a carries 0x200 in bits 40:49 (K56
bit 0 clear), the answer to our INFO1c, which since the dead band always says 0.
What the card needs is the K56 flag in *our* INFO1c, not anything in V.8bis.

Tone timeline of a K56 call to 7910 (hunt-style `k56c/k1`, tap time): card sends the
40 ms CRe-pair burst at 2.3 s, ANSam (2100 Hz) from 4.1 s to about 6.9 s, then FSK
alternating 1650/1850 Hz (V.21 channel 2: its JM) 7.4-8.3 s, silence, then V.34
Phase 2 tones from 11.2 s. Our side: V.8bis FSK 4.0-5.4 s, V.8 CM-style FSK 6.4-8.0 s,
Phase 2 from 11.5 s. Nothing in the card's signalling after V.8bis differs from a
plain V.8 answer; with JM carrying no PCM bit and INFO0a/INFO1a carrying no K56
marker, the card never offers us a K56flex PCM startup after our V.8bis. Whether it
would for a client that sends something other than our MS/CL (the Rockwell client's
exact sequence is in `cruft/patton-re/K56-client-comparison.md`) is the open
question; this needs a known-good Rockwell/K56flex client capture against the same
card, which we do not have. The client path is therefore blocked on evidence, not
only on code. (Wrong: the card's K56flex training was already in the hunt5 taps;
see below.)

## 7910 starts K56flex on INFO1c bit 0, and its P1 is ours (2026-10-25)

The "wideband PCM-style Phase 3" that the INFO1c dead band suppresses
(`v34tx.c` `v34_info1c_frequency_offset`) is the card's K56flex downstream
training, started by our own INFO1c. Evidence: CT 117 `/root/hunt5` (plain
`--mode v34` PPP calls, no V.8bis at all), calls 3 and 17, two attempts each
with `fo=1`; retained locally in
`reference-data/artifacts/k56flex-7910-info1c-k56-20261025/`.

INFO1c bits 40:49 are the K56 flags legacy HSF `1201efa0` reads (bit 0 enable,
bit 3 A-law; see "HSF INFO1a mode/law selection" above). Per attempt in
hunt5: bit 0 set (`fo=1`, or `fo=-1` = 0x3ff) -> PCM Phase 3 every time;
bit 0 clear (`fo=0`, `fo=2`, stale `-290`/`-282`) -> V.34 Phase 3. The card's
INFO1a after a bit-0-clear INFO1c has 0x200 in the same field (decoded from
the 2026-10-25 `[INFO1ARX]` frames).

What the card sends after INFO1a (call 3, first attempt; tap time):

| time | codewords | our server (HSF staged-8k run) |
|---|---|---|
| 12.455 s | `2a 41 c1 aa` repeated (2 kHz marker) | `2a 42 c2 aa` at 11.05 s |
| 12.489-14.535 s | 4-level ±`2f`/±`30` (±4092/±3900) | P1, ±`30`/±`31` |
| 14.535 s on | 2-level ±`2f`, biphase, until we retrain at ~15.9 s | P2 (12 levels), then P3 ±`30` |

- **P1 is byte-exact ours** after two fixed transforms: every sign inverted
  and each magnitude one codeword higher (`2f->30`, `30->31`). It matches for
  16368 codewords (1364 pairs x 12, our `K56T_P1` length to the sample) on all
  four bursts (calls 3 and 17, attempts 1 and 3). So our scrambler, seed,
  mapping and P1 length are right for real hardware.
- **The level "shift" is A-law.** Our *A-law* P1 (`k56flex_probe`, base table
  3904/4032), coded with SpanDSP `linear_to_alaw` and converted with
  `alaw_to_ulaw`, equals the card's PCMU P1 sign-inverted, 16368/16368, with
  no remapping. The card sends A-law K56flex (the NZ ISDN line law) and the
  path transcodes it to our PCMU leg, so on 7910 every PCM mode crosses an
  A->mu converter. The identification marker does not survive the same
  check exactly (card `2a 41 c1 aa`, ours `2a 40 c0 aa`): its 1888 level
  lands one mu codeword lower than SpanDSP's conversion puts it.
- The card then **skips our P2**: the 2-level signal is biphase (one bit per
  `01`/`10` sample pair, no violations), and the bits obey
  `b ^ b(n-5) ^ b(n-23) = 0` exactly, scrambled constant input like our P3.
  It holds this until **we** give up: our "no PP/TRN lock 3500 ms into
  Phase 3" watchdog retrains, not the card.
- During the same stretch of our server's run the native HSF client is
  **silent** (11.0-16.25 s) and starts its upstream at 16.5 s, about 0.25 s
  after our server enters GATE_A. Against the card we were sending V.34
  Phase 3 (S/TRN) instead.
- `vpcm_decode --k56flex` acquired P1 on our server's tap (0 mismatched blocks)
  but not on the card's, even with the levels remapped: the front end took
  only a positive correlation. Fixed: `k56flex_rxfe` acquires on |correlation|
  and the signed gain carries the polarity (the LS equalizer starts at
  1/gain; the timing loop compares phase block to block, so a constant pi
  drops out). `legacy_pcm_decode_flex` now also tries the other server law
  when the line law does not acquire. `k56flex_7910_p1_test` runs the client
  over `reference-data/k56flex-7910/p1-ulaw.g711` (the card's marker, whole
  P1 and the start of its hold, cut from the silence before the marker):
  mu-law and A-law models, card and sign-flipped polarity, all four lock and
  check all 2728 P1 blocks with 0 mismatches at 54 dB; gain -1.041 (mu model)
  and -1.0065 (A-law model, the closer fit). Without the front-end change
  the card-polarity cases do not lock. Past P1 the model follows our P2 and
  mismatches, because the card does not send it: after P1 it sends about 20
  four-level samples and then the biphase hold, which neither our P2 nor a
  P3 entered straight from P1 reproduces (A-law, sign-inverted). That stage
  is not recovered.

So the client path is not blocked on evidence. What it needs, in order:
1. A client Phase 2 that sets INFO1c bits 40:49 = 1 (mu-law K56) when
   K56flex is wanted (`V34_INFO1C_FO=1` already forces this for experiments),
   and on the card's INFO1a goes silent instead of starting V.34 Phase 3.
2. A receiver that accepts the card's polarity and level table (P1 acquisition,
   then the biphase hold).
3. The client's upstream reply after the hold, as the HSF client sends from
   16.5 s in `artifacts/k56flex-hsf-staged-8k-20261021/client-tx.g711`; we have
   no client transmitter for it yet.

### Live: a silent K56flex client holds the 7910 through P1, P3 and gate A (2026-10-25)

`ME_K56FLEX_CLIENT_PROBE=<ms>` (4fd1277b, off by default): on a calling V.34
call, once our receiver enters Phase 3 the engine transmits silence, skips the
3.5 s Phase 3 retrain, runs the client model for both server laws on the raw
line and hangs up when the window ends or the V.34 receiver leaves Phase 3.
INFO1c's K56 flag comes from `V34_INFO1C_FO=1`. One call from CT 117
(`/root/k56probe`, recipe in the evidence's `RUN.txt`):

    V34_INFO1C_FO=1 ME_K56FLEX_CLIENT_PROBE=20000 ME_V34_PHASE3_TRAIN_TIMEOUT_MS=0 \
    V34_INFO1C_DBG=1 V34_INFO0_DBG=1 ME_JB_MS=40 \
    python3 tools/eicon_ppp_direct.py OUT --law ulaw --mode v34 --local-port 5079 --rtp-port 14610 --pings 3

- **INFO1a answers the flag.** With our INFO1c bits 40:49 = 1 the card's INFO1a
  carries `0x005` there (`2f 60 36 59 c0 95 c6`): bit 0 K56 enabled, bit 2
  A-law, as legacy HSF `1201efa0` reads them. Without the flag it is `0x200`.
  So the card itself declares the A-law server the transcode analysis found.
- **P1 acquired live**: both models lock 142 ms into Phase 3 at inverted
  polarity (gain -1.041 mu, -1.0065 A) and check all 2728 P1 blocks with 0
  mismatches at 53.9 dB. The tap matches our A-law P1 (A->mu, inverted)
  16368/16368 again. Our transmit tap is exactly zero from Phase 3 on.
- **The card's timeline, now complete to its timeout** (tap time): P1
  12.489-14.535 s; 19 transition samples; then the biphase two-level stage,
  whose descrambled source (complemented for the polarity) is FFFF for 8169
  bits = 2.048 s (our P3, 1366 pairs) and then the word **`8990`** repeated
  from 16.585 s: the same four-point Jd word our server appends at GATE_A
  (`K56T_GATE_A`, `training_word`). So the card runs our sequence
  P1 -> P3 -> GATE_A and **omits P2**. It holds gate A for 10.87 s waiting
  for the client and then sends its retrain tone (27.455 s); our receiver
  answers it (11.5), the probe logs its state and hangs up.
- Past P1 the client model follows our P2 and fails, as expected: it needs a
  timeline without P2 for this server, and the inverted complement in its
  descrambled bits.

What the client still needs to go further: skip P2 for this server (or
detect it), recognise gate A's Jd word, and then the client's upstream reply
within ~10.9 s of gate A (the native HSF client starts its upstream 0.25 s
after our server's GATE_A). No upstream is transmitted yet.
Evidence: `reference-data/artifacts/k56flex-7910-client-probe-20261025/`
(local, indexed).

Build note: HEAD has not built from a clean tree since c9b9a3af (V.90 T/3
early windows): it changed `v90_t3_acquire_pass` and
`v90_t3_acq_retry_or_abandon` but three call sites in `v34rx.c` keep the old
argument lists. Trees with the uncommitted `v34rx.c` work in them build; the
CT 117 tree above carries only those three call-site fixes.

### A known-good K56flex client on the 7910 path: native HSF over SIP (2026-10-25)

`ME_AUDIO_RELAY=<socket>` (57340d44) bypasses the engine DSP and exchanges a
call's G.711 byte-exact with a peer; `tools/k56flex_hsf_sip.py` puts cruft's
native legacy HSF Flex client on it (built on CT 117 with
`python3 -m native.hsf.lift.build --opt 1 --jobs 4`, 16.5 min; it runs at
2.7x real time there). Evidence: `reference-data/artifacts/k56flex-7910-hsf-sip-20261025/`
(local, indexed; `RUN.txt` says which call is which).

**The 7910 path squelches the card's CRe.** Asterisk bridges our leg to
`6501` at 192.168.88.62 (context `from-cisco`, a Cisco gateway; Asterisk's own
legs are RFC 4733 with fax detection off, and on PCMA as well as PCMU the
card's CRe arrives as 40 ms of 1375+2002 Hz followed by exact digital
silence for the rest of the 500 ms. That is a tone detector muting a tone it
has recognised, not the card: no CRe detector fires on it, HSF's included.
Without help HSF hears ANSam, never starts V.8bis, sends nothing and the card
clears at 12.5 s (call 1). `--repair-cre` writes the missing part of the CRe
(1375+2002 Hz to 400 ms, then 400 Hz for 100 ms) over the muted codewords of
the *client's input only*; the line recording is untouched and everything the
client sends is its own. (The 2910 path through our own `diva-sip-gateway`
avoids the Cisco but its MultiTech BRI has never answered: the gateway sends no
called number and the BRI clears with 0x3490.)

**Call 2 (repaired CRe, 60 ms relay cushion):** HSF detects the CRe, parses the
card's MS (`12c180808009b502009401834355e1`, accepted), replies with its own MS
(`11c180808009b50200948181024785`), the card ACKs (`14`), V.8 runs and HSF
starts its K56 pump; Phase 2 completes, INFO1a is received and HSF reaches
CallWaitJ, then **CallWaitAfterRxGotJ** about 5.3 s later (it has seen the
card's gate A Jd). Line timeline of the first cycle:

| line time | card | HSF |
|---|---|---|
| 13.85-15.9 s | P1 | silent |
| 16.0-17.0 s | **P2** (12 levels) | silent |
| 17.0 s | P3, then gate A (+-`2f`) | silent |
| 19.5 s | | upstream starts after Jd |
| 19.75-22.7 s | one constant codeword `29` | upstream continues |
| 22.75 s | Phase 2 again (retrain) | follows into its own retrain |

So against a real client the card *does* send P2 (with our INFO1c-only client
it skipped it: the V.8bis exchange or the INFO1c content selects that), leaves
gate A when the client's upstream starts, and then waits ~3 s in a stage that
sends a constant level before giving up. Three such cycles, then HSF reports
"K56flex training fell back to another modulation". That call had 2720 relay
underrun codewords (17 frames: HSF took up to 87 ms on a frame, more than the
cushion), so its upstream had silence inserted and may not be judgeable.
**Call 3 (300 ms cushion):** 0 underrun, but V.8bis never completes (CRe seen,
no MS): the added round trip breaks V.8bis timing. **Call 4 (120 ms):**
dropped at 4.6 s because `multitech-ppp.service` (the 7910 answerer on
eicon420) was stopped by another session working on the rig at the same time;
it was left failed. No conclusion from calls 3 and 4.

Next, when the rig is free: 120 ms cushion (with `nice`) to get an
underrun-free call 2; then compare HSF's upstream after Jd with what our client
must send, and what releases the card's constant-level stage.

### The 7910 holds gate B for a report that HSF never sends (2026-10-26)

**Call 5 (120 ms cushion, `nice -n -10`, `--repair-cre`):** no underrun in the
first training cycle (the only four are at 60.5 s), and it fails exactly like
call 2. Evidence: `artifacts/k56flex-7910-hsf-sip-20261026-call5/` (local).
Line timeline, by codeword statistics per 125 ms:

| line time | card |
|---|---|
| 14.0-16.1 s | P1 (4 levels, `2f/30/af/b0`) |
| 16.1-17.1 s | P2 (12 levels) |
| 17.1-20.1 s | P3, then gate A (`2f/af`) |
| 20.1-22.9 s | one constant codeword `29` |
| 22.9 s on | Phase 2 again (retrain) |

Our server's upstream receiver (`k56flex_upstream_pcm_rx`, 3200/high) on HSF's
`client-tx.g711` finds **S/S-bar at 19.8 s** and nothing after it: no PP/TRN
acquisition, no report, although HSF keeps transmitting to 23.75 s. The card
leaves gate A 0.3 s after that S/S-bar, which is our server's GATE_A rule (S/S-bar
releases it, `k56flex_train.c`), and the constant `29` stretch is its gate B,
which in our server ends only on an accepted client report (DM 8F31 bit 10).
The card gives up 2.8 s into it. Against our own server HSF's report arrives
2.1 s after its S/S-bar (`00/89b0`, staged-8k run: S/S-bar +16800 ms, report
+18880 ms, MP +22000 ms) and HSF moves on to `Tx_MD`; against the card it stays
in `CallWaitPh3_10ms`. So the 7910 stall is HSF waiting for something in the
downstream before it sends its report, and the card waiting for that report.
Underruns were not the cause. One visible difference: our server's gate B is
an 8-level signal, the card's a single constant level.

**A recorded client upstream** (uncommitted experiment knob
`ME_K56FLEX_CLIENT_TX_FILE` / `ME_K56FLEX_CLIENT_TX_AT_MS` in the client probe):
play HSF's own S/S-bar, PP, TRN, report, 899F, MP and MP'
(`reference-data/k56flex-hsf/upstream-mp-ulaw.g711`) into the hold 4.5 s after
Phase 3 entry, to ask whether a valid report releases the card's gate B. Not
yet answered: on 4 of 4 calls (`/root/k56tx/1-4` on CT 117) the card answered
our K56-flagged INFO1c (`fo=1`) with a plain V.34 INFO1a (`00 e0 34 09 a0 df 4f`,
`00 20 36 09 20 b6 66`) and ran V.34 Phase 3, where on 25 October it answered
with `0x005` K56 flags every time. Its INFO0a differs too: `ff 21 e2 37 9b`
today against `7f 21 2e bb dd` in the 25 October probe, which looks like a
different modem answering. Since 22:45 on 25 October 7910 has two MultiTech
answerers (`ttyM4`, `ttyM5`, same chat script `ATE0V1S0=1S109=1`), and the
`ttyM4` chat has been waiting since an abnormal call end at 23:51.

**That was not it (later the same day).** Both answerers were restarted
(15:39, fresh chat init `ATE0V1S0=1S109=1`) and 16 more probe calls got the same
plain-V.34 answer. Two short PPP calls with INFO0 logging both landed on `ttyM4`
(peer 10.254.104.1) and both got the non-K56 INFO0a, so it is the same modem
that answered the K56 flag on 25 October; a held call on `ttyM4` put a
concurrent probe on `ttyM5`, which answered the same way. Both modems are
**Lucent ISI56128 Central Site Data Modem 2.10V1** (`ATI0`), with identical
registers: S37=0 (line rate automatic), S38=200, S109=1, S40=0, S41=13, profile
not stored.

**The one INFO0a bit that moves is bit 19, 3429 allowed** (`vpcm_decode --v34`
on the taps; everything else is CRC): clear on every call where the card went
on to K56flex (CRC `0xDD97`, SpanDSP's raw `7f 21 2e bb dd`), set on every call
where it did not (`0x9BF1`, `ff 21 e2 37 9b`). Across every retained 7910 log
the first form always comes with an INFO1a carrying the K56 flags and the
second never does. The card sends INFO0a after the client's INFO0c (HSF's
INFO0c at 9.89 s, the card's INFO0a at 10.29 s), so it can react to it.

**The card still offers K56flex to HSF, deterministically.** A native HSF call
at 16:13 (`/root/k56h/6`, same recipe as call 5) got bit 19 clear and P1 at
14.4 s. Our calls, with signalling byte-identical to the 25 October probe that
got K56 (CM `C1 45 10 10 2A`, INFO0c with bits 21:23 = 5, INFO1c `fo=1`), get bit
19 set. Neither of the visible differences from HSF is the cause on its own:

| tried | INFO0a bit 19 | calls |
|---|---|---|
| `--mode v34`, JB 40 ms (the 25 October recipe) | set | 9 |
| plus V.8bis K56flex client exchange first (`AT+MS=K56,1`, completes, card ACKs) | set | 3 |
| V.8bis, default 200 ms jitter buffer | set | 3 |
| `AT+MS=V34,0` | set | 2 |
| native HSF (CM `C1 45 10 10`, INFO0c bits 21:23 = 0) | clear | 2 |

Not yet compared: our INFO0c with bits 21:23 = 0 as HSF sends them, CM without
the third modulation octet, and V.8/Phase 2 timing. The recorded-upstream
question (does a report release gate B?) is still open: no call today reached
gate A.

**Matching HSF's CM and INFO0c does not do it either (A/B, same day).** Correction to
the table above: our CM's fifth octet `2A` is the **protocols** octet (V.8 tag
`0x0A`, LAPM), not a modulation octet; `AT+ES=1` (originator error control off)
drops it and gives HSF's `C1 45 10 10`. `V34_INFO0_MAX_BAUD_DIFF=0` (new, off by
default, `v34tx.c`) sends INFO0 bits 21:23 = 0 as HSF does, and applies the same
bound to our own Table 16 choice. Both checked on our own taps (offline engine
pair: CONNECT both ways, INFO0c bits 21:23 = 0 with the knob, 5 without; CM
`C1 45 10 10` with `+ES=1`) and on the rig's taps for every arm. Calls to 7910 from
CT 117 (`/root/k56ab`), arms alternated:

| arm | card INFO0a bit 19 | K56 INFO1a | calls |
|---|---|---|---|
| baseline `AT+MS=V34,0` | set | no | 2 |
| INFO0c bits 21:23 = 0 | set | no | 2 |
| CM without the protocols octet (`AT+MS=V34,0;+ES=1`) | set | no | 2 |
| both | set | no | 2 |
| both, after a completed V.8bis client exchange (`ME_K56FLEX=1`, listen) | set | no | 2 |

In the last arm our CM and INFO0c are bit-identical to HSF's (INFO0c CRC `0x9BF1`
both), and the card still answers HSF's INFO0c with bit 19 clear and ours with it
set (two V.8bis attempts more failed with "no CRe heard" in the 3 s window and
are not counted). So neither field, nor V.8bis as such, is what the card keys
on. What still differs before its INFO0a: our V.8bis MS reply's contents (not
yet logged), the timing of our INFO0c against the card's (ours 10.07 s, HSF's
9.89 s on these taps; the card answers HSF 0.4 s later), and levels/spectrum.
The recorded-upstream question is still open: no call reached gate A.

### Our V.8bis MS reply is what turned the 7910 away; a played-back report does not release its gate B (2026-10-26)

Our client MS against native HSF's, both after the card's MS
(`12 c1 80 80 80 09 b5 02 00 94 01 83 43 55 e1`), now logged as `our message [...]`:

```
ours: 11 c1 80 80 81 80 b0 cf 09 b5 02 00 94 81 83 02 47 85
HSF : 11 c1 80 80 80          09 b5 02 00 94 81 81 02 47 85
```

Two differences: SPar(1) `81` with a three-octet NPar block (`80 b0 cf`) where
HSF sends `80` and none, and NS octet 7 `83` where HSF sends `81` (in our server's
own template that octet is `v90_capable ? 0x83 : 0x81`, so our client claims
V.90). `K56FLEX_CLIENT_MS_HEX` (new, off by default, `k56flex_v8bis.c`) replaces
the client MS payload. A/B on 7910 with everything else HSF-identical (V.8bis
client exchange, CM `C1 45 10 10` via `AT+ES=1`, `V34_INFO0_MAX_BAUD_DIFF=0`),
arms alternated, only calls whose V.8bis completed counted (`/root/k56ms`):

| MS | INFO0a bit 19 | K56 INFO1a | P1 acquired | calls |
|---|---|---|---|---|
| HSF's (`11c180808009b50200948181024785`) | clear | yes (`2f e0 34 59 40 4b b5`) | yes, both models | 2/2 |
| ours | set | no | no | 0/2 |

Which of the two differences matters is not yet split. The card also sends P2
after this MS (as it does for HSF, not after an INFO1c-only offer).

**The played-back upstream reaches the card but its report does not release
gate B.** With the K56 session up, the probe played HSF's recorded upstream
from +4.5 s. Our own receiver decodes it from our transmit tap: S/S-bar 18.9 s,
PP/TRN 19.2 s (correlation 0.79), report `00/89b0` 20.24 s, 899F 20.32 s, MP
(9600/52000) 23.3 s. The card's downstream (tap time):

| call | gate A | gate B (`29`) | then |
|---|---|---|---|
| hsf1 | 17.4-19.6 s | 19.6-22.4 s | retrain (Phase 2) |
| hsf3 | 17.4 s to hang-up at 29.3 s | never | - |

In hsf1 the card left gate A 0.7 s after our S/S-bar, received the report
0.6 s into gate B and still timed out 2.8 s later, as it does against HSF
itself. In hsf3 it never saw the S/S-bar. Both calls' INFO1a asked our transmitter
for 3200/high **with 7+5 dB power reduction and pre-emphasis 3**, which a
recording cannot honour (its level and filter are those HSF used against our own
server). That, and the report's contents (`89b0` is HSF's report of our server's
downstream), are the open candidates. Next: our own client upstream
transmitter, so level, pre-emphasis, timing and report value are ours to set.

### Our own client upstream: the 7910 accepts our report; it waits in gate C past our MP (2026-10-26)

`k56flex_client_tx.c` is a K56flex client upstream generator: one unit-RMS
symbol per baud for SpanDSP's V.34 modulator, through the existing
`v34_tx_start_external_symbols()`, so the symbol rate, carrier, power reduction
and pre-emphasis are what the server's INFO1a set for our transmitter (V.34
11.2.1.1.8; Phase 3 set-up has already applied them). What it sends, measured
off native HSF's upstream (`reference-data/k56flex-hsf/upstream-mp-ulaw.g711`)
with our own receiver's conventions:

| stage | content | native HSF |
|---|---|---|
| S, S-bar | V.34 10.1.3.7 | 128T S-bar at 16T |
| PP | V.34 10.1.3.6 | 288T |
| TRN | four-point, scrambled ones, GPC from zero | **3910** symbols (our receiver uses 512) |
| report | 24-bit record (ext, report), differential, histories carried from TRN | 21 records of `00/89b0` |
| J' | **one more record with `899F` in the report word's place** | a bare 899F after the last record misses the receiver's record phase |
| TRN2 | **a second TRN, fresh scrambler** | **9614** symbols straight after J' |
| MP, MP' | V.34 10.1.3.9, K56 rate fields; histories carried from TRN2 | type 0: 9600 up / 52000 down, mask 0x7fff, bit 50 set |

`k56flex_client_tx_test` (fast list) runs it through the V.34 modulator and
G.711 into our server's streaming upstream receiver: every event in order, the
report, MP fields and MP' decoded, at 3200 high and low carrier, pre-emphasis
0/3/6, both laws, with INFO1a power reduction measured on the line (7.00 dB for
7); the default MP is native HSF's bit for bit; an invalid report header is
rejected; the post-J' symbols are a fresh TRN.

In the engine it runs inside the client probe (`ME_K56FLEX_CLIENT_UPSTREAM=1`,
off by default), started 250 ms after the client model enters the server's gate A
(after its Jd; the recorded playback started during P3, which is one reason it
failed). Overrides: `ME_K56FLEX_CLIENT_REPORT`, `_TRN_SYMBOLS`, `_GAP_SYMBOLS`
(TRN2), `_MP_REPEATS`, `_MP_ACK_REPEATS`, `_MP_UP_CODE`, `_MP_DOWN_CODE`,
`_MP_MASK`.

**Live, 7910** (CT 117 `/root/k56up`, `/root/k56mp`, `/root/k56rate`, `/root/k56t2`,
`/root/k56t2b`; every call with HSF's V.8bis MS, CM and INFO0c, so the card
offers K56flex): the card's INFO1a asked for 3200/high, 7 dB power reduction,
pre-emphasis 3, and our upstream went out with them. On every call where V.8bis
completed (10):

| tap time | card | us |
|---|---|---|
| 17.5-19.9 s | P3, gate A | S at +280 ms after gate A, PP, TRN |
| 19.9-21.6 s | gate B (constant `29`) | report from 21.1 s, J' |
| **21.6 s** | **leaves gate B**: identification marker | TRN2 |
| 21.75-22.75 s | PT_A (two-level) | |
| 22.6-29.1 s | eight-level PT_C / gate C | MP, MP' at 24.2 s |
| 29.3 s | retrain (Phase 2) | |

**The card accepts our report**: it leaves gate B 0.4 s after it, which neither the
played-back recording nor native HSF itself ever achieved. It then holds gate C
for 6.5 s and retrains; the hold ends at the same 15.2 s into Phase 3 on every
call whatever we send. Not the cause, each A/B'd one variable at a time:
MP repeats (2+1 against 40+4, and 40+8 with TRN2), downstream rate code (11,
52000 against 1, 32000), TRN2 instead of differential idle. In MICA's server
logic gate C ends on DM 8F31 bit 12 (training complete) or 13 (MP accepted); the
card shows neither. Open candidates: what the Lucent server needs to call its
upstream training complete (TRN2 length or timing against its PT stages), MP
fields other than the rates (reserved bits, type 1), and the report value
(`89b0` is HSF's report of our own server's downstream, not of this card's).

**TRN2 length and start time do not move gate C (A/B, same day).** New, off by
default: `cfg.trn2_hold` / `ME_K56FLEX_CLIENT_TRN2_START=eight` holds the second
TRN after J' (differential idle) until the card's eight-level stage is seen on the
line (three 50 ms windows of exactly eight distinct codewords after our report;
logged on every call as "server eight-level stage seen"). Calls from CT 117
(`/root/k56len`, `/root/k56start`), arms alternated, only calls whose V.8bis
completed and whose upstream started counted (9 of 22; the rest missed the
clipped CRe):

| arm | TRN2 | MP at | card's eight-level stage | then | calls |
|---|---|---|---|---|---|
| length 4800, after J' | +6.95 s | +8.45 s | 6.50, 6.60 s | retrain | 2 |
| length 9614, after J' | +6.95 s | +9.97 s | 6.55, 6.50, 6.50 s | retrain | 3 |
| length 19000, after J' | +6.95 s | +12.89 s | 6.50, 6.50 s | retrain | 2 |
| length 9614, released on the eight-level stage | +8.61, +8.65 s | +11.6 s | 6.50, 6.50 s | retrain | 2 |

(Times from Phase 3 entry.) The card's eight-level stage starts at the same
point every call (~22.7 s tap, detected +8.6 s into Phase 3) and lasts 6.5 s
whatever we send, so it ends on a fixed timer, not on anything of ours we have
varied. Neither TRN2 length (4800-19000) nor its timing against the card's PT
stages is what this server waits for.

**The 7910 checks the report word; the extension octet changes nothing (A/B,
same day).** Arms each one change from `00/89b0`, round-robin, two calls each
whose upstream started (`/root/k56rep` on CT 117; 10 of 14 calls; offline, every
value decodes as sent in `k56flex_client_tx_test`):

| report | change | gate B | eight-level stage | probe ended |
|---|---|---|---|---|
| `00/89b0` | baseline | left 0.4 s after the report (21.55, 21.65 s) | 6.60, 6.50 s, then retrain | 15.2 s |
| `40/89b0` | pad-table group 0x40 | left (21.60, 21.55 s) | 6.50, 6.50 s, retrain | 15.2 s |
| `80/89b0` | pad-table group 0x80 | left (21.65, 21.55 s) | 6.50, 6.50 s, retrain | 15.2 s |
| `00/8880` | field bits clear | **held to its 2.7 s timeout** (22.60, 22.65 s) | none: retrain | 8.6 s |
| `00/8890` | pacing bit (4) only | **held to timeout** (22.75, 22.75 s) | none: retrain | 8.6 s |

So the card does not accept every header-valid report: with bits 4, 5 and 8 set
(`89b0`) it releases gate B every time, with bit 5 and 8 clear it never does.
Bit 8 is the one our server reads to choose D9AD for the parameter-training
stage (report bit 8, see the stage list above); what bit 5 means is not
recovered. The extension octet's pad-table group made no difference anywhere,
and no accepted variant changed the 6.5 s gate C. Next candidates: the other
field bits one at a time from `89b0` (5, 6, 8, 9, 10, 12-14), to map which
values the card accepts and whether any of them moves gate C.

**The report word is a V.34 J pattern: the 7910 accepts exactly 8990 and 89b0
(A/B, 27 October).** One field bit changed at a time from `89b0`, round-robin,
two calls each whose upstream started (`/root/k56bits` on CT 117, 18 of 25):

| report | change | gate B | stage after PT_A | probe ended |
|---|---|---|---|---|
| `89b0` | baseline | left (21.60, 21.65 s) | eight-level, 6.50 s, retrain | 15.2 s |
| `8990` | bit 5 clear | left (21.65, 21.55 s) | **four-level** (`c0 29 a9 40`), 6.50, 6.55 s, retrain | 15.2 s |
| `88b0` | bit 8 clear | held to timeout | - | 8.6 s |
| `89f0` | bit 6 set | held to timeout | - | 8.6 s |
| `8bb0` | bit 9 set | held to timeout | - | 8.6 s |
| `8db0` | bit 10 set | held to timeout | - | 8.6 s |
| `99b0` | bit 12 set | held to timeout | - | 8.6 s |
| `a9b0` | bit 13 set | held to timeout | - | 8.6 s |
| `c9b0` | bit 14 set | held to timeout | - | 8.6 s |

The two accepted words are V.34 Table 18's J patterns, `8990` asking for
four-point and `89b0` for sixteen-point training, and the card's parameter
training follows the choice: four levels after `8990`, eight after `89b0`. They
are also the two values resident B31D returns (above). So "the report", as far
as this server is concerned, is J with the extension octet, and nothing else is
accepted. Either choice still ends in the same 6.5 s gate C timeout.

Offline, `00/99b0` cannot be decoded by a header-only collector: rotated 21
bits its record reads `04/cd80`, which also passes `(report & 888F) == 8880`,
so our receiver takes the wrong record phase and misses J' (the test asserts
the alias). Irrelevant in practice now: the card rejects `99b0` anyway.

Every report-side variable we have is now exhausted against gate C (report
word, extension octet, TRN2 length and timing, MP repeats and rate). What is
left is the upstream itself during the card's PT/gate C stages, and the
downstream: the client presumably has to train on the server's PT signal and
answer something the server can see before gate C will end.

**The card's Jd asks for a sixteen-point TRN2; sending one does not move gate C
either (A/B, 26 October).** Native HSF's own client logic, read from the legacy
V124 VxD in cruft (`native/hsf/recovered/legacy-stack/LEGACYV124.decompiled.c`,
object 1): on `RX_GOT_J` (`2202fa20`) it records the server's Jd, `8990` or
`89b0`; `Tx_Jtag` (`2202e0f0`, the J' with `899F`) then calls `2202c2a0`, which
puts TRN2 (`2202d990`), MP (`2202e240`) and E (`2202e490`) on the sixteen-point
constellation, four bits per symbol, when that Jd was `89b0`. That is V.34
10.1.3.3 (J chooses the remote's TRN, MP, MP' and E constellation), 10.1.3.8
and 10.1.3.9. Our server sends `8990`, so every HSF upstream we copied was
four-point. The 7910 does not: decoding its gate-A word (two-level biphase, bits
descrambled with 1 + x^-5 + x^-23) gives `8990` on the 25 October INFO1c-only
probe but **`89b0`** on both native-HSF calls that reached gate A (25 October
call 2, 26 October call 5); the V.8bis MS exchange is what changes it.

New: `k56flex_jd_rx` (in `k56flex_client_tx.c`) reads the Jd off the downstream
blind to polarity, pair alignment and word phase (8990 on our server's taps,
nothing on any upstream tap or on calls that never reach gate A), and
`cfg.trn16` sends TRN2, MP, MP' and the E/idle tail sixteen-point; the engine
sets it from the Jd before TRN2 starts, `ME_K56FLEX_CLIENT_TRN16=0/1` forces it.
`k56flex_client_tx_test` checks TRN2 symbol by symbol against 10.1.3.8 and
decodes MP/MP' back with an independent 10.1.3.9 decoder.

Live (CT 117 `/root/k56j16-runs`, the recipe below, arms alternated, 10 calls):

| arm | V.8bis completed | card's Jd | TRN2 | eight-level stage | probe ended |
|---|---|---|---|---|---|
| follow the Jd (sixteen-point) | 4/5 | `89b0` at +5.36 s, all 4 | +6.95 s | +8.65 s | 15.2 s, retrain, all 4 |
| forced four-point (as before) | 5/5 | `89b0`, all 5 | +6.95 s | +8.60-8.65 s | 15.2 s, retrain, all 5 |

So the card's timeline is identical whichever constellation TRN2 and MP use:
the four-point TRN2 was a spec violation (kept as the follow-the-Jd default,
which leaves calls to our own server unchanged), but it is not what gate C
waits for. Two further things the VxD shows about HSF's side, neither tried yet:
it sends MP only when its receiver has an MP ready (`2202dbb0`; otherwise "MP
NOT READY - Going To ReTrn"), and it repeats MP until its receiver reports the
server's MP (event `0xb`, then MP') or MP' (event `0xc`, then E and B1); TRN2
also ends early on the receiver's "ClearDown" event `0xf`. Every exit from
HSF's TRN2/MP is driven by its downstream receiver, which our client still does
not have past P1.

### The real MICA firmware as a gate oracle; J' matters, PT does not (26 October)

Asked for next: a client receiver for the 7910's PT/parameter stage. Before
building one, two checks on whether the server waits for anything the client
derives from PT.

**The 7910's PT stage is our server's.** On today's taps the card sends, after
our report: its second identification, PT_A (scrambled ones, biphase, 1.03 s,
descrambled bit-exact all ones as in our `K56T_PT_A`), then the eight-level
PT_C running on into gate C. Stage lengths match `k56flex_train.c`; the
scrambler phase differs, as expected for a self-synchronising scrambler.

**MICA replay oracle.** MicaEmu's `mica_trace` replays a recorded upstream
into the original MICA firmware deterministically and watches its gate word
DM 8F31. Native HSF's MICA call (`cruft/artifacts/hsf-mica-k56-9600.client.ulaw`)
replays exactly: 0800 (S/S-bar) 26.935 s, 0400 (report accepted) 29.062 s,
**1000 (training complete, ends gate C) 30.469 s**, 4000 (MP') 32.212 s.
`tools/k56flex_client_splice` (new, not in the test lists) keeps that call's
V.8bis/Phase 2 and replaces everything from S on with ours:

```sh
make k56flex_client_splice
MD=2200 TRN=4600 ./k56flex_client_splice hsf-mica-k56-9600.client.ulaw ours.ulaw 26.88 0 5 0 1 14
mica_trace ours.ulaw --map mica-portware-map.bin --seconds 35 --greg 0x80 --watch 0x8f31 --host
```

| our upstream after S | MICA 8F31 |
|---|---|
| as sent live (one S/S-bar, TRN 3910, 21 records) | 0800, then 2.7 s gate-B timeout; its TRN worker (DM 8D24) never leaves `582d` |
| + MD 2200 symbols and a second S/S-bar | TRN worker runs (`588d`, `58f2`); report still missed |
| + TRN 4600 (or 100 report records) | **0400, then 1000 at +1.335 s**: gate C passed |
| ... then silence from 0.34 s after J' | 0400, 1000 at the same instant |
| ... without J' | 0400, then abandoned 180 ms later |
| ... any TRN2 length (160 ms to 3 s), 4- or 16-point, J' as one record, 100 records or bare `899F` | 1000 every time |
| MP / MP' in any variant tried (16- or 4-point, x1.5 and x2.24 16-point scale, mask `3fff`, down code 4, 80 MP', J' straight into MP) | never 2000 or 4000: an artefact of the 2200-symbol MD, see the next section |

The MD row is a property of the splice, not of our client: HSF's INFO1c,
which MICA heard in Phase 2, announces an MD of ~700 ms, and V.34 11.3.1.2
makes the answerer wait that long after S-bar. Our own INFO1c says `md=0`.
The TRN/report row is a MICA timer: it reads the report only from about 1.4 s
after TRN starts, and native HSF repeats its report until it hears the
server's second identification (VxD `2202dce0`). So in MICA, **gate C needs
nothing the client derives from PT**: a valid report, J', and a few hundred
ms of carrier. A PT receiver would not change it. What MICA cannot do is
read our MP: its received-bit register DM 8F13 shows HSF's MP' frames cleanly
(sync, up 4, down 11, mask `7fff`, ack bit set) but ours as random bits (88-bit
autocorrelation 0.50), although the sixteen-point labelling is identical to
the one MicaEmu's own client used to get 2000/4000 (all 16 points checked).
MICA was also sending Jd `89b0` in this call.

**The 7910 sees our J' too (live A/B, CT 117 `/root/k56jp-runs`, 12 calls, 8
with V.8bis):**

| J' | calls | card |
|---|---|---|
| one record (`00` + `899F`, as before) | 2 | PT_A, eight-level, gate C to 15.2 s |
| 100 records | 2 | the same |
| bare `899F` x150 | 2 | the same |
| **none** | 2 | second identification, PT_A cut after 0.25 s, call over at 7.8 s |

The card's reaction to a missing J' is MICA's. **Neither the first TRN nor the
report length moves gate C** either (`/root/k56trn-runs`, 10 calls with
V.8bis: TRN 3910/4600/6000, 21/100 records: 15.2 s on all; the eight-level
stage just shifts with our report). So on the 7910, everything up to J' is
accepted as on MICA. The difference is after it: MICA sets "training
complete" on a timer, the Lucent card does not, and the one thing both
servers fail to read is our MP. If the card's gate C waits for an accepted MP
(DM 8F31 bit 13 in MICA's terms), that would explain the 6.5 s hold.

Next: make MICA accept our MP. MicaEmu's accepted client (`MicaEmu/artifacts/
k56flex-client-20261009`, `connection-verification-call-command.json`) sends
TRN 512, 8 report records, bare `899F` until its receiver sees MICA's
training-ready, then MP. Rerunning it with its upstream captured gives a
reference MP waveform to compare symbol by symbol with ours. New client knobs
for these experiments (all off by default): `ME_K56FLEX_CLIENT_REPORT_RECORDS`,
`ME_K56FLEX_CLIENT_JPRIME=record|bare|none`, `ME_K56FLEX_CLIENT_JPRIME_RECORDS`;
`cfg.md_symbols` exists in the transmitter only.

### MICA accepts our MP; the earlier rejection was the splice's MD (26 October, later)

The previous section ended on MICA never posting 2000/4000 (MP, MP'
accepted) for our upstream. Done since, offline against the MICA firmware:

1. **Reference.** MicaEmu's accepted client call
   (`artifacts/k56flex-client-20261009/connection-verification-call-command.json`)
   reruns with `tools/k56flex_upstream_rig/tee_peer.py` capturing what MICA
   is fed (MicaEmu's live client must reuse its 18 October receiver library:
   rebuilding it fails since v90modem removed `ctl.status`/`ctl.report` on
   21 October). MICA posts 2000 at 15.842 s and 4000 at 16.009 s, and a
   replay of the capture reproduces that exactly.
2. **Symbol-level comparison.** `tools/k56flex_lsdemod.py` (new; numpy and
   `make k56flex_modulate`) recovers a recording's symbols on our own
   modulator's grid by least squares (7% interior residual on HSF's MICA
   call, 1.4% symbol error on our own signal) and re-modulates symbol vectors.
   Findings:
   - **Native HSF follows our conventions**: S 45/135 degrees, S-bar
     -135/-45, PP and TRN within 2 degrees, TRN2 the same 16-point sequence
     (|c| 0.989), the same power for 16-point as for 4-point. MicaEmu's
     reference rotates S by -45 and TRN by 180 degrees against PP; MICA
     tolerates both.
   - **HSF's MP' frames are 96 bits** (11 zero fill bits after the CRC
     instead of V.34 Table 20's 3) and start exactly 9600 symbols into TRN2;
     MICA reads them as up 4, down 11, mask `7fff`, bit 50, ack. MicaEmu's
     reference sends up 12, down 10, mask `07fe`. Both are V.34 10.1.3.9
     16-point MP and decode with our decoder.
   - In MicaEmu's call MICA wants the **report as a bare repeated 16-bit
     word** (it decodes `8990` every 16 bits); our 24-bit record (HSF's
     form) is decoded but never accepted there, whereas in HSF's call it is
     the 24-bit record that MICA accepts and the bare word it rejects. The
     expected form follows the call's V.8bis/Phase 2. On the 7910 we send
     HSF's MS, so HSF's form is the relevant one. `cfg.report_bare` /
     `RBARE` added.
   - MICA reads the report only once its decision-directed stage has run
     for a while: about 0.7 s after TRN starts at 3000 baud in MicaEmu's
     call, 1.4 s at 3200 baud in HSF's.
3. **Bisection by re-modulation.** HSF's recording with HSF's own MP'
   re-modulated (control): 4000 at 32.212 s. Our MP' on HSF's grid after
   HSF's TRN2: **4000 at 32.204 s**. Ours from progressively earlier points
   (TRN2, report, J', TRN1, PP, S-bar, the second S): accepted every time.
   Only HSF's first S/S-bar and its MD were left to differ, so the fault was
   the MD: HSF's INFO1c announces 20 x 35 ms, MICA waits exactly 700 ms
   (V.34 11.3.1.2), and our splice's 2200-symbol MD was 12.5 ms short.
4. **With MD = 2240 symbols (700 ms) our own generator is accepted end to
   end**: 0400 29.228 s, 1000 30.580 s, **2000 32.318 s, 4000 32.324 s**
   with the default MP (88-bit, 2 MP + 1 MP', TRN2 9614); with 8 + 8 frames
   MICA goes on to `c000` at 34.66 s. So our MP was never wrong, and the
   "MP never accepted" row above was a property of the splice.

```sh
make k56flex_client_splice
MD=2240 TRN=4600 ./k56flex_client_splice hsf-mica-k56-9600.client.ulaw ours.ulaw 26.88 0 5 0 1 14
mica_trace ours.ulaw --map mica-portware-map.bin --seconds 35 --greg 0x80 --watch 0x8f31 --host
```

**On the 7910, MD does not move gate C (live A/B, CT 117 `/root/k56md-runs`,
10 calls, 7 with V.8bis).** New experiment knob `V34_INFO1C_MD=<units>`
(V.34 Table 12 bits 18:24; our transmitter sends no MD itself, the client
probe sends one of the announced length): with 20, as HSF, the card waits it
out (our report, TRN2 and its eight-level stage all 0.76 s later), then holds
gate C to the same 15.2 s; with 0, 15.2 s on all five.

So the upstream that the card sees is, by the MICA firmware's standard,
correct through MP'. MICA ends gate C on a timer 1.3 s after the report;
whatever ends the Lucent card's gate C is not something MICA needs. Native
HSF, the only other K56flex client tried on this card, never gets past its
gate B, so there is no reference for it. New transmitter options (all off by
default): `cfg.report_bare`, `cfg.mp_extra_fill` (8 gives HSF's 96-bit
frame), MP' without a preceding MP (`mp_repeats` 0).

### Our K56flex client connects to the MICA firmware, data both ways (27 October)

Closed loop, our whole engine as the calling K56flex client against the
original MICA firmware in MicaEmu (`mica_trace ... -- v90_engine_peer`, built
from this tree by MicaEmu's `tools/mica_peer_probe.py`): **CONNECT 52000,
V.42 LAPM with V.42bis, 9600 upstream, exact text both ways** (MICA's host
receives 119 copies of our test line byte for byte; our DTE receives
`CONNECT 52000` and both lines MICA's host sent). With `ME_DATA_FRAMING=v14`
the same call carries async data both ways. Checked on a clean build of HEAD
plus only these changes. What it took, in call order:

1. **V.34 Phase 2 as the call modem** (`v34tx.c`). Against MICA our caller
   never left Tone B. Two stages waited on the single receiver event slot:
   `FIRST_B` for `INFO0_OK` (or `TONE_SEEN`), `FIRST_B_INFO_SEEN` for
   `REVERSAL_1`; MICA's events overwrote both. Both now test durable state
   (`info0_received`, `phase2_reversal_count`). That exposed a second fault:
   the receiver counts a "reversal" on MICA's INFO0a/Tone A seam, before
   MICA can legally reverse (V.34 11.2.1.2.3: only after it detects Tone B),
   so our Tone B reversal went out 40 ms into Tone B and MICA never advanced.
   Reversals counted before Tone B starts are now discarded. MICA then
   completes Phase 2 with our INFO0c only as HSF sends it
   (`V34_INFO0_MAX_BAUD_DIFF=0`).
2. **Upstream through MP'** (`k56flex_client_tx.c`, as before; Jd-following
   16-point TRN2/MP): MICA posts 0800, 0400, 1000, 2000, 4000.
3. **Downstream receiver** (`k56flex_client.c`, the linear path, following
   MicaEmu's 10 October client): the front end is not trained on the
   shadow's gate-A word (the server chooses 8990 or 89B0); gate B ends on
   received PCM activity, not equalizer ringing; the PT_C table follows the
   report we actually sent (the engine used a placeholder `8880`); the final
   parameter record is decoded; training DC carries into the mapper; and the
   mapper rate is identified from the six all-ones priming frames, because
   MICA's record says rate field 15 (60000) and maps at 52000 (exactly one
   candidate decodes them). Replaying held frames must keep each frame's
   symbol index for the front end's decision update. The model then decodes
   MICA's data frames exactly (checked against a raw-codeword decode).
4. **E, B1 and data** (`k56flex_client_tx.c` `send_e`, SpanDSP
   `v34_tx_set_data_scrambler_tap`): after MP', E (20 ones, V.34 10.1.3.2),
   then SpanDSP's existing B1/data handover (`v34_v90_arm_tx_data`) at the
   MP's upstream rate with the call modem's GPC scrambler. MICA's trellis
   input (captured at PC 47B9) equals our transmitted data symbols, index for
   index (correlation 1.000).
5. **Engine DATA** (`modem_engine.c`, as the x2 client does): the data stack
   starts on the model's first data frame, DATA and CONNECT once our
   transmitter is past B1. V.34's own B1 rate check must be marked done: its
   mark hold times from a native V.34 data entry this path never makes, and
   without that the upstream idled at mark for the whole call (which is what
   the earlier "MICA decodes noise" sweeps were misreading).

Knobs (experiment, off by default): `ME_K56FLEX_CLIENT_DATA=1` (the above),
`ME_K56FLEX_CLIENT_DATA_TAP` (17), `ME_K56FLEX_CLIENT_DATA_N` (MP's rate),
`ME_K56FLEX_CLIENT_DATA_ADVANCE` (0; MicaEmu's fixture needed 25, ours needs
none), `ME_K56FLEX_CLIENT_MP_FILL`. MICA's IOS sequence matches native HSF's
connected call (`1901`, then `1e01` about 5 s later).

**The same client on the 7910 (CT 117 `/root/k56conn-runs`, 6 calls, 5 with
V.8bis): unchanged.** The card ends its gate C at 15.2 s and retrains on all
five. Our receiver model also stops following the card in PT_A there (both
law models, failing at the retrain); against MICA it follows PT_A bit-exact,
so the card's A->mu transcoded downstream needs its own look. Nothing the
client transmits depends on the model, so that cannot explain gate C.

## K56flex client against the 7910: where it stands (27 October 2026)

Every piece below is an experiment knob inside the client probe; there is no
client call mode yet. Recipe that gets furthest (CT 117, `eicon_ppp_direct.py
--ms 'AT+MS=V34,0;+ES=1'`): `K56FLEX_CLIENT_MS_HEX=11c180808009b50200948181024785
ME_K56FLEX=1 ME_K56FLEX_LISTEN=1 ME_K56FLEX_ROLE=client K56FLEX_DETECT_BLOCKS=2
V34_INFO0_MAX_BAUD_DIFF=0 V34_INFO1C_FO=1 ME_K56FLEX_CLIENT_PROBE=40000
ME_V34_PHASE3_TRAIN_TIMEOUT_MS=0 ME_K56FLEX_CLIENT_UPSTREAM=1`.

| stage | state |
|---|---|
| V.8bis | works when the clipped CRe is caught (about half the calls; the Cisco path mutes it after 40 ms). Our default MS is **wrong** for this card: only HSF's (no NPar block, NS octet 7 `81`) gets K56flex. |
| Phase 2 | works with HSF's CM (no protocols octet) and INFO0c (bits 21:23 = 0) and INFO1c's K56 flag (`fo=1`); none is a default. |
| downstream P1-P3, gate A | received: P1 byte-exact through the A->mu transcode (sign inverted), P2, P3, gate A's Jd. Our receive model follows them only by timing past P1. |
| upstream S/PP/TRN/report/J'/TRN2/MP/MP' | ours (`k56flex_client_tx.c`), honouring INFO1a; **the card accepts the report** (only the J words `8990`/`89b0`); TRN2/MP on the constellation the card's Jd (`89b0`, sixteen-point) asks for. |
| gate C | **passed on about half the calls** with MP repeated 400 times before MP' (27 October, below). Before that: the card's PT stage (four- or eight-level) always ends on its own 6.5 s timer and it retrains. It sees our J' (without one it quits at once, as MICA does) and honours an announced MD. The MICA firmware accepts our upstream end to end (report, training complete, MP, MP'); see the MICA sections above. |
| parameter record, priming, data | record: **decoded** (short form, read by magnitude; 27 October, last section). Priming and data: **decoded** by content (blind priming, below), error-free on a PCMA leg. |
| E, B1, upstream data | implemented (`ME_K56FLEX_CLIENT_DATA=1`); carries data to the MICA firmware, and to the 7910 **at 4800 only** (`ME_K56FLEX_CLIENT_MP_UP_CODE=2`: LAPM and PPP, 3/3; last section). |

What gate C is waiting for, most likely first. In the MICA server our own mirrors,
gate C ends on "training complete" or "MP accepted"; the card gives neither.
Varied without effect: report word and extension, TRN2 length (4800-19000) and
start (after J' or on the card's PT stage), TRN2/MP constellation (four- or
sixteen-point), MP repeats (to 40+8), MP downstream rate.

1. **The card does not accept our MP as K56flex MP.** HSF's MP layout came from
   HSF talking to *our* server, which reads K56 rate fields exactly as HSF writes
   them; no K56flex client has ever reached gate C of this Lucent server, so its
   MP format is unverified. Candidates: MP type 1 (with precoder), the reserved
   bits (19, 28, 51-67), a V.34-style downstream field.
2. **The card waits for the client to have trained on its PT stage** and to say
   so in a form we do not send (a second report or J after PT, or an MP sent only
   after the client's receiver has locked to PT). This needs a client receiver
   for the card's PT signal before it can even be tried properly.
3. **The card's upstream receiver loses us after the report.** Least likely: the
   report is accepted, and TRN2 and MP use the same modulator.

The decisive evidence would be a capture of any K56flex client getting past gate
C against this card (a hardware K56flex modem on an analogue line to 7910, if one
of the rig modems has K56flex firmware); native HSF never gets past gate B here.

### The 7910 reaches data on our side; its upstream receiver never locks (27 October)

Recipe as above plus `ME_K56FLEX_CLIENT_DATA=1 ME_K56FLEX_CLIENT_MP_REPEATS=400
ME_K56FLEX_CLIENT_MP_ACK_REPEATS=8`, `eicon_ppp_direct.py --law alaw --ext 7910`.
Runs: `artifacts/k56flex-7910-{blind,lapm,lapm2,alaw}-20261027/` (local).

**Blind priming.** Our PT_C generator leaves the card's after a block or two
(matching the DC-flipped samples by magnitude, `k56flex_probe_invert_magnitude`,
rescues one more block, no more; the report word does not change it), so the
parameter record cannot be read. The data path does not need it:
`k56flex_client.c` (`blind_symbol`) slices the last 48 symbols to every rate's
table and takes the first rate and frame phase whose six frames descramble to
ones after the self-synchronising descrambler's 23 bits (Draft 0.23 4.12).
Derived signs carry no bits and follow the server's running DC, which we
cannot follow here (`k56flex_pcm_rx_t.ignore_derived_sign`; the best DC model,
a running sum of levels plus a constant, fits only 94% of them). On every
call that left gate C the card primes at **52000, A-law mapper**, and its data
is ones for 0.6 s, then HDLC flags (`k56flex_client_test` on
`reference-data/k56flex-7910/blind-data-ulaw.g711`).

**Codec identity.** On a SIP leg the client's samples are the server's levels;
the front end's acquired fractional phase was already integral, but it
subtracted a "codec DC" learned from the first 160 samples. Off now when
`codec_identity` is set, as the engine sets it.

**Use a PCMA leg.** Over PCMU the gateway transcodes the card's A-law to mu by
nearest level (not G.711's table: `alaw_to_ulaw` fails frame 27 of mp400-1),
and A-law levels 4032/4224 (and others) share a mu codeword: 11% of 52000
frames are ambiguous, which corrupted 11% of the flag words. On PCMA
(`--law alaw --ext 7910`) the same decoder reads the flags with **0 errors**.

**The V.8 CM protocols octet.** With `V8_PROTOCOL_LAPM_V42` in our CM (as
`+ES=3` sends) the card reached priming on 0 of 4 calls; without it, on 7 of
14. HSF's CM has no such octet, so the client role now omits it
(`modem_engine.c`) and `+ES` alone picks the data framing. Weak evidence:
priming is itself only about 50%.

**What still fails: the card never hears our upstream data.** Every call that
reaches data ends the same way: we enter DATA (CONNECT 52000 on our side with
`+ES=1`), the card sends ones then flags (no ADP: our "V.42 detection
succeeded" fired on its flags), never answers our LAPM frames (no frame with a
good FCS downstream), and starts a V.34 retrain 4.7-5.7 s into data. The card's
host never logs CONNECT (`journalctl -t pppd -t chat` on eicon420: nothing
since 26 October). Not the cause, each tried live: upstream scrambler GPC (17)
or GPA (4); E sent 120 ms before the card's priming or 250 ms after it
(MP' x60); V.14 or V.42 framing. Most likely the card's parameter record
asks our upstream for something we cannot read while PT_C is undecoded
(precoding coefficients, trellis or rate, as a V.34 MP would), so decoding
the 7910's PT_C is the next step; alternatively its MP acceptance (and so
gate C) is marginal for the same reason.

### The 7910's parameter record decodes; calls that fail stop before its final record (27 October)

Offline, on the three PCMA calls above (`artifacts/k56flex-7910-alaw-20261027/mp400-{2,3,4}`).
Our model of the 7910 is exact through P3 and inverts every PT_A/PT_B block.
"Our PT_C generator leaves the card's" was a sign problem. The card's PT_C
is our PARAM_B stage, A-law table (levels ±1008/2752/4480/6016, Draft 0.23
4.10), and in that stage the source bits set only the **magnitudes**. Each
block has exactly one magnitude inverse. The signs come from the sequence
state and the running DC, and the card runs those differently: odd samples
follow our DC rule only 82% of the time. `k56flex_probe_invert_abs` matches
on magnitude only, and the client falls back to it from PT_C on
(`abs_blocks`). Every block of the card's 4.3 s eight-level stage then
inverts, and the descrambled source reads:

- 0101... alternation for about 2.7 s (our PT_C source is FFFF; the card's
  differs and nothing depends on it);
- then the **short (mode 0) record**: FFFF prefix, marker, three words and
  a CRC in 96 bits. MICA sends the long one. `parse_record` now tries both
  forms and checks the mode field in the record's own first word.

| call | initial record (repeats) | final record | then |
|---|---|---|---|
| mp400-3 | `6374 9fff 0000` (x2) | `e374 9fff 0000` (x385) | priming at 52000, data |
| mp400-4 | `6354 9fff 0000` | `e354 ...` | priming at 52000, data |
| mp400-2 | `634c 9fff 0000` (x752, about 6 s) | **never** | retrain |

Fields (`k56flex_param_raw` layout): mode 0, rate field 13, u = v = 1,
control 1FFF, bit 1. **extra** (DM 8C77 in MICA) changes from call to call:
13, 5, 3. The rate field is not the mapper rate. The card maps at 52000 on
every call, and MICA's field 15 also mapped at 52000, so priming still
chooses the rate. The engine log now prints the record's fields instead of
a bps derived from the field.

Timing, in hold time: the initial record starts at +10.55 s on all three
calls, about 580 ms after our MP starts (+9.97 s). On the calls that reach
data, the final record follows 24 ms later, long before our MP' (+12.71 s),
and repeats until priming (+12.89 s, just after our E at +12.77 s). So the
half of the calls that fail are failing **between the card's two records**.
The card has finished its gate C and sends PARAM_1, but its condition for
PARAM_2 never arrives. In MICA the initial record follows gate C's
"training complete" (bit 12, `1000`), and PARAM_1 runs until bit 13, which
`15EE` posts when it accepts MP (`2000`; `k56flex_train.c`, MICA sections
above). Read the same way, the 7910 finished training on our upstream on
all three calls and accepted our MP only on the two that reached data. That
supports cause 1 in the list above (MP acceptance) over causes 2 and 3. It also means the
answer is on the wire: in a failing call, our MP is being sent for those
6 s, and the card does not accept it.

When the record path's rate check finds no candidate, the client now
replays the held symbols into the blind search instead of failing. The
card's priming does not start on the pair the record path chooses (28
samples early on mp400-3). `k56flex_client_test` checks the record on the
retained PCMU fixture: it survives the A->mu transcode.

### Our MP is identical on passing and failing calls; the record's "extra" is the card's upstream rate (27 October, later)

**Our upstream does not differ.** The `live-tx.g711` taps of mp400-2 (no
final record), mp400-3 and mp400-4 are **byte-identical** from S through
TRN, report, J', TRN2, MP x400, MP' and E (0 differing octets over +5.5 to
+12.8 s once aligned on the hold start). The card's own timeline, decoded by
the client, lines up with our transmitter to within ±10 ms on all three
calls: P1, P2, P3, gate A, gate B, ID2, PT_A, PT_B, PT_C, gate C at
+9.00..9.02 s, and the initial record at +10.54..10.56 s. Nothing we send,
and no timing between the two sides, separates the calls.

**All 21 captured 7910 calls** (`artifacts/k56flex-7910-*-2026102[67]`,
both laws; the record survives the A->mu transcode):

| our MP | calls | final record | extra in initial record |
|---|---|---|---|
| x400 then MP' x8 | 15 | 13 (12 then primed and carried data) | 13 (x10), 10, 7, 5 |
| x400 then MP' x8 | | 2, never | **3, 3** |
| default repeats | 5 | none | 11, 11, 12, 13, 13 |
| no J' | 1 | no record at all | - |

So the card accepts our MP on 13 of the 15 x400 calls, and on none of the
calls with the default repeat count, which is the "MP x400 passes gate C
about half the time" result read correctly: the gate is passed (the initial
record follows) and the MP is accepted.

**What extra means.** In MICA (MicaEmu `artifacts/k56flex-client-20261009/rate-selection.md`)
the record from the real firmware, with our client, carries 8C77 = 12 and
control 0FFF, as does the Gough Lui call. The 7910 sends control 1FFF and
extra 13 on most calls. Both fit control = the server's upstream rate mask
in V.34 N codes (1..12, 1..13) and extra = the upstream N it chose, which
is never above the mask's top bit. That is how our own server fills the
record (`k56flex_train_accept_mp`, which native HSF accepts). On the 7910,
extra changes from call to call while our MP stays the same, so the card
sets it from its own measurement of our upstream. The record that carries it
arrives about 580 ms into our MP.

**What it accounts for.** The two MP x400 calls without a final record are
the only two where the card chose **N=3 (7200)**, below our MP's upstream
code 4 (9600); every accepted MP had extra ≥ 5. n is small. Extra is also
the N that the card's INFO1a already projects for our direction (13 on the
calls below), so it is the card's capability from line probing, not the
rate it demodulates at. The next section refutes the idea that we send 4
while the card expects its extra.

### Live: the 7910 carries our upstream at 4800 only; 31200 fails like 9600 (27 October)

CT 117 against the 7910: PCMA (`--law alaw --ext 7910`), MP x400 / MP' x8,
`+ES=3`, recipe as above. `ME_K56FLEX_CLIENT_MP_UP_CODE` and
`ME_K56FLEX_CLIENT_DATA_N` set together (the MP proposes the rate we then
send at). Two batches: `/root/k56up-runs` (10:45, arms 2/1/6 alternated,
unrecorded until now) and `/root/k56up13-runs` (14:47, 13/4 alternated, then
one 2 as control). Local copies: `artifacts/k56flex-7910-up{,13}-20261027/`.
Calls where V.8bis did not complete (no K56flex attempt) are excluded.

| upstream | calls | final record | result |
|---|---|---|---|
| N=1 (2400) | 2 | 2/2 | card stays in data for 124 s; LAPM never establishes |
| **N=2 (4800)** | **3** | 3/3 | **CONNECT 52000, V.42 LAPM + V.42bis, PPP up (LCP, peer IP, pings)** |
| N=4 (9600) | 2 | 2/2 | card retrains 4.7 s into data |
| N=6 (14400) | 2 | 2/2 | card retrains 4.7 s into data |
| N=13 (31200) | 2 | 2/2 | card retrains 4.7 s into data |

Every record on these calls has extra 13 and control 1FFF, including the
4800 calls that connect. So the card does not demodulate at its own extra,
and it accepts an MP proposing any of these rates. **This is the first
complete K56flex call between our client and the 7910**, data both ways at
52000/4800, three times out of three.

What separates 4800 from 9600 and above lies in our upstream data path at
those rates. The MICA firmware decodes our 9600 upstream exactly
(closed loop, above), so the question is what the 7910 does differently
there. Neither shell mapping nor the mapping-frame size separates them: at
3200 baud, b = 6 at 4800 and 12 at 9600, both K = 0 (V.34 9.3). Candidates
in V.34 order: the constellation and the uncoded-bit count q above 4800, the
precoder and non-linear encoder (our MP is type 0, with no coefficients; the
card's MP to us has not been decoded), and the trellis. 2400 failing
differently (no retrain, no LAPM) is a separate issue and may be a rate the
card does not support. Next: decode our own N=2 and N=4 upstream taps
(`live-tx.g711`) with SpanDSP's V.34 receiver at those rates, to check that
both are valid V.34 data with what our MP announced. If both decode, the
difference is in what the 7910 assumes above 4800 (precoding,
constellation or trellis), and the parameter record is the only thing it
tells us. Meanwhile 4800 is a working setting for the client.
