# K56flex implementation

`k56flex.c` (payload core), `k56flex_v8bis.c` (V.8bis exchange),
`k56flex_probe.c` (training sample generator), `k56flex_train.c` (server transmit
sequencer), `k56flex_client.c` / `k56flex_rxfe.c` (a client for our own server, over G.711
octets or over linear audio) and `k56flex_channel.c` (a simulated analogue loop for it),
exercised by `make k56flex-test`, `k56flex_v8bis_test`, `k56flex_probe_test`,
`k56flex_train_test` and `k56flex_client_test`. The client is test-only and not
linked into the modem. It is the digital-side (server) downstream payload core
and its exact inverse, plus the V.8bis capability frame, the parameter record
and the accepted-report logic. **It is not wired into a call**: `--mode` is
unchanged and K56flex cannot be selected live. The specification itself says a
completed K56flex call is not established (Draft 0.23, Table 1a and clause 11).

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

## V.8bis exchange (live, opt-in)

`ME_K56FLEX=1` makes the engine run the V.8bis exchange before V.8 (Draft 0.23
clause 4.1: V.8bis precedes V.8). As answerer it listens up to 3 s for CRe
(1375+2002 Hz, then 400 Hz), waits 500 ms, sends CRd (1529+2225 Hz, then
1900 Hz), receives the client's CL on V.21 channel 1, applies the documented
offset-13 predicate (0x02 or 0x21), replies MS (or NAK1) on channel 2 and logs a
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

`ME_K56FLEX=probe` runs it after V.8bis as raw G.711, then gives up and starts
V.8 if no peer gate is satisfied within `ME_K56FLEX_GATE_MS` (default 8000)
of the probes. Pair it with `ME_G711_CAPTURE` to record the client's reply.

What the gates are: the firmware polls DM 8F31 bits 11, 10, 12|13, 13|14 and
14, which its receive worker sets from the client's signalling. How that looks
on the wire is not recovered, so `k56flex_train_status()` takes those bits from
the caller and nothing in the engine sets them. B31D's training word is
unknown and defaults to FFFF. The priming frames' handling of leftover queue
bits and the scrambler history across the hand-off follow the firmware flow
only approximately. A live call stops at gate A at best.

## Client (loopback against our own server)

`k56flex_client.c` receives the server's raw G.711 and runs the whole startup
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

## Draft 0.23 correction found while implementing

Draft 0.23 labels the tables selected by DM EC6E bit 0 = 0 as A-law and = 1 as
mu-law. The shipped data say the opposite: every level in the bit-0 = 0 tables
is a mu-law codeword magnitude (and none of the 1 tables are), and every level
in the bit-0 = 1 tables is an A-law magnitude. `k56flex_test` asserts this for
all 3,148 levels. The probe levels in clause 4.10.1 follow the same pattern
(3772 is mu-law, 3904 is A-law). `k56flex_law_t` is named by the data, so
`K56FLEX_LAW_MU` is EC6E bit 0 = 0. The octet-18 law bit in the V.8bis template
(`0x20` when EC6E bit 0 is set) is a separate documented field and is kept as
the spec gives it; whether it should be inverted too is unresolved.

The spec's recovered serial compander is mu-law only, but that is the DSP's
T1 output stage. Here the codeword is chosen in the bearer's law, so PCMU uses
the bit-0 = 0 tables and PCMA the bit-0 = 1 tables.

## Not covered (the integration boundary)

- The client's side of the exchange: decoding what the client sends during and
  after the probes (the gate signals above), its report, and the upstream
  receiver.
- The upstream receiver (a V.34-style path, as for V.90) and receive of the
  client's report waveform. Only the decoded-bit report logic exists.
- Network RBS phase, retrain and fallback, gain/pad selector meanings
  (clause 7.24 is Provisional) and the peer parser AC6D for received MS/CL.
- Hardware interop. No result here says a real K56flex client will connect.

Next step: from v90modem dial the MultiTech RAS (it answers; it cannot dial
out) with `ME_K56FLEX=probe` and `ME_G711_CAPTURE`. The log shows whether it
answers our CRe and what its MS holds. Whether it reacts to the probes at all
is visible in the capture, and would be the first on-wire evidence for the
gate signals.
