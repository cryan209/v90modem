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
