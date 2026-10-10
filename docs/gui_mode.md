# Live modem GUI

`--gui` opens a native AppKit window on macOS. `--gui-web` opens the same
monitor and controls in a local browser; on other platforms `--gui` uses that
browser view. Both use Python 3's standard library. The native window needs
Apple's command line tools for its first build (`make gui` builds it ahead of
time). The native binary and app bundle are small, fixed build artifacts;
compiler module caches are temporary and removed after compilation.

```sh
./sip_v90_modem --gui --sip-server asterisk.net.cryan.nz --username 6001 --password 6001
```

The GUI creates separate temporary control and data PTY links. Supply
`--control-link` and `--data-link` to choose them. `--pty-link` is rejected because
a combined command/data port cannot give two independent consoles. Dial with a
Hayes number (`ATD6004`), resolved against `--sip-server`; SIP punctuation in an
AT dial string is rejected by the existing interpreter. All AT commands still
run through that interpreter, including `AT+MS` and unsolicited result codes.
The serial console enables sending after the DCE reports the data connection,
not merely after the physical carrier appears. Closing the native window stops
the child modem; for the browser monitor use Ctrl-C in its launching terminal.

## What the views measure

- **Waveforms:** the last 512 received/transmitted 8 kHz samples, expanded from
  the actual G.711 codes (64 ms, fixed ±32768 scale). Expansion is diagnostic
  only; no samples are resampled or replaced in the media path.
- **QAM/TCM constellation:** the receiver's public QAM report callback in V.22bis,
  V.32bis and V.34, including the V.34 receiver used for V.90 upstream. Measured
  points and reported decisions are separate. V.34's orange points are its
  delayed Viterbi traceback lattice decisions. Receivers without that callback
  show no points; the GUI does not synthesize a constellation from arbitrary
  line audio. TCM is coding over a QAM signal, so it shares the QAM eye view.
- **Recovered eye:** actual receiver T/2 samples, with separate I and Q traces,
  from V.22bis, V.32bis's V.17 front end, and V.34's primary T/2 path.
  Select receiver input or equalizer output. The receiver's phase flag aligns
  decision samples at 0, T and 2T; intervening samples are measured at T/2.
  Lines join measurements for readability; they are not extra measured points.
  TCM uses the same QAM view. TX has no receive clock, and PCM/T/3 paths do
  not yet expose this recovered-eye tap; unavailable/stale taps are labelled.
- **Line estimate:** the previous exploratory two-period fold remains an
  explicit option, using the reported baud and carrier and automatic PCM/QAM
  direction selection. It is display-only sinc reconstruction/mixing, not
  recovered timing or the remote D/A's actual filter. V.90 digital downstream
  is TX PCM; the analogue role reverses the direction.
- **Datapump wire bytes:** observations of `ds_tx_get_bit` / `ds_rx_put_bit`,
  after transmit framing/compression and before receive deframing/decompression
  (V.14 §6; V.42 §7). Eight consecutive bits are packed LSB first from call
  start. Idle marks, detection patterns, HDLC framing, stuffed bits and
  compressed data are included. These are not serial characters, aligned LAPM
  frames, or the modulation scrambler's output. Negative/status values are
  ignored, and incomplete octets are reset per call. Clear-channel paths that
  bypass the data stack are visible in the DS0 view instead.
- **G.711 DS0 bytes:** exact codewords entering/leaving the engine, including
  negotiation, training and silence. These are PCM sample codes, not recovered
  payload bytes. They also make an untrained call observable.
- **Training:** readable current RX/TX engine stages plus the existing phase
  transition trace, even without `--verbose`. V.34 §10.1 and V.90 §§9.2–9.4
  define the A/B, S, TRN, J, MP, E and B1 families. The engine's trace reports
  the transitions it actually observes; not every internal DSP substage has
  a public stage name. The process log retains more detailed diagnostics.

## Bounded storage and media isolation

No GUI waveform, byte or process-log files are written. GUI mode also disables
the engine’s automatic V.34 training raw files unless `ME_DUMP_DIR` is explicitly
set. The existing optional capture environment variables still have their
existing behavior if explicitly set. The monitor retains a one-second audio ring each way, 256 wire octets and
256 G.711 codes each way, 256 measured and 256 decided IQ points, and 16 phase
events. Snapshots expose just the latest 512 audio samples.

Each control/data/log stream holds at most 256 chunks of at most 4 KiB in the
Python relay. Every native/browser console displays at most 24,000 characters;
training history retains at most 120 events. New output evicts old output.
There is no per-call file archive and no backlog for a slow client. A paused UI
freezes its plots while the relay continues overwriting its bounded histories.
The native client permits only one outstanding polling request.

Media callbacks only copy into bounded rings under a leaf mutex. Snapshot
formatting and nonblocking loopback UDP sends run on the control thread at
10 Hz. The GUI's drawing, signal reconstruction, HTTP and PTY handling run in
separate processes. Failed UDP sends discard diagnostic snapshots. On macOS the
UDP send buffer is explicitly large enough for snapshots beyond its default
9 KiB datagram limit; this was found when QAM populated during a real local
SIP call. `ME_GUI_PORT` is a launcher-owned diagnostic port, not a modem setting.
The relay listens only on loopback with an unpredictable session URL and
requires the expected Origin for PTY writes.

## Verification

`make test-fast`: 469 checks passed when the passive taps were introduced.
`line_monitor_test` covers exact codewords, LSB packing, bounded wrap, JSON
escaping/truncation, and per-call reset. `data_stack_test` verifies that observing
line bits preserves the payload and idle/status behavior.

`make gui-smoke-test` runs two real localhost SIP/RTP modems through V.22bis:
AT response, training/data state, 512-sample waveforms, a full QAM ring,
snapshots larger than 9 KiB, increasing wire counters, byte-exact binary serial
data both ways, and hangup. It uses temporary files/PTYs and cleans up children.
This is a local GUI/transport check, not evidence of hardware interoperability.
The native AppKit console was additionally opened and its AT controls checked.

## Compact native dashboard

The native window fits a single 1200 × 760 screen area with no page scrolling.
AT and serial controls remain visible together. Signal, audio analysis and
process output share one tab area. Consoles show the latest visible lines and
have no scrolling history; the relay still uses bounded in-memory retention.
The browser view also uses a compact fixed viewport and latest-line consoles.

The manual PCM/QAM selector is replaced by automatic direction/role selection.
TCM codes a QAM constellation; PCM is a sample-level signal. The explicit line estimate still uses display-only timing/mixing; the default
RX eye uses recovered receiver samples where available.
Fewer overlaid eye traces and just centre axes reduce clutter.

Wire telemetry retains 256 run records per direction in addition to its raw
256-byte suffix. Repetitions of four or more bytes display as `FF ×2109`;
shorter runs remain hex bytes. A continuing idle run updates its complete count
without displacing preceding records. The dashboard shows the latest lines;
it is not a capture/archive browser.

The native Audio tab shows an RX amplitude histogram and a Hann-windowed
spectrum (62.5 Hz bins, relative magnitude). A yellow marker and numeric label
show the live receiver carrier estimate from SpanDSP where available. Carrier
mixing removes the passband carrier to form baseband I/Q; a constellation
alone does not show its absolute frequency.

Audio off / Listen RX / Listen TX plays a diagnostic copy through the Mac's
output device at 25% volume. It is off initially. Snapshots carry at most 1600
samples per direction with a per-call sample counter; playback deduplicates
snapshots and queues at most three buffers. Missed samples are discarded,
never replayed from an unbounded backlog. Device-rate conversion exists only
in the monitor player and never changes modem/RTP samples. Slow UI polling
can cause listening gaps; this is a monitor, not a recording facility.

ASVD is V.61, not a pure constellation rotation. V.61 §5.5.1.1 sums a complex
voice signal element with each data signal element, constrained to its data
region (Figure 6). A suitable measured constellation can show displacement
around data points, but our existing QAM reports do not implement or identify
V.61. Future ASVD inspection would need its own receiver tap before voice
subtraction. V.61 §5.1 specifies its carrier and modulation rate.

## Spectrum waterfall

The native Waterfall tab shows RX and TX together in the existing diagnostic
area, without enlarging the window. Frequency runs from 0 to 4000 Hz;
newest rows appear at the top. Each row uses the latest 256 samples with a
Hann window and 129 bins (31.25 Hz spacing). Colour is fixed from −90 to
0 dBFS, rather than normalized per row, so level changes remain visible.
Yellow lines mark the recovered RX / nominal TX carrier where available.
Each direction holds exactly 128 rows in memory, overwriting old rows.
Duplicate sample counters add no rows; call reset clears the display, and
Freeze pauses it. Rows represent received GUI updates rather than a calibrated
time axis; skipped telemetry is not reconstructed or recorded.

## Recovered-eye observation

A separate optional callback reports the input and current FSE output at every
recovered T/2 insertion, with phase 0 at the decision instant and phase 1 at
its intervening sample. The V.34 tap is on the primary channel, not its
control-channel decoder. Its diagnostic dot product deliberately bypasses
`equalizer_get()`'s divergence reset: observation cannot change coefficients.
V.22bis and V.32bis similarly use read-only FSE evaluation. These callbacks
are installed only in GUI mode; normal decoding, carrier/timing loops and
sample accounting retain their existing arithmetic and order. QAM modulation
references: V.22bis §2.3, V.32bis §2.2, V.34 §§5.2, 10.1.

The monitor stores 128 five-value records (input I/Q, equalized I/Q, phase)
in a per-call ring. Nonfinite samples and invalid phases are ignored. No eye
captures go to disk. The native/browser clients reject stale samples and skip
trace segments across a discontinuous phase sequence; receiver timing slips
are not disguised with synthesized samples.

Validation includes eye ring wrap/reset/JSON checks, real localhost V.22bis
SIP samples with alternating T/2 phases and byte-exact serial transport, and
V.32bis/V.34 duplex tests with the observer enabled and error-free PRBS payload
in both directions. Offline tests do not establish hardware interoperability.

## Built-in local loopback

```sh
./sip_v90_modem --gui-loopback
# Browser:
./sip_v90_modem --gui-web --loopback
```

The launcher owns two loopback-only SIP endpoints and a serial echo peer.
Select a Local test profile and click Start loopback; a fresh pair of modem
processes is started for that test, and both receive
`AT+MS` through their separate control PTYs before the caller dials the peer.
Hang up before changing profiles. The 256-byte pattern button sends every
byte value from 00 through FF; returned bytes appear in the serial console.
The echoed-byte counter counts far-end PTY bytes successfully written back,
not a BER measurement or a guarantee that every sent byte arrived correctly.

Verified echo profiles cover V.21 FSK at 300, V.22 at 1200, V.22bis at 2400,
V.32 at 9600, V.32bis at 14400, and V.34 at 9600/21600. Defaults use V.14 framing on both ends;
an explicitly set `ME_DATA_FRAMING` is inherited by both. `--mode` can set
an initial offer; the Local test selector controls each test.
The V.90 profile starts an analogue caller and digital answerer
(V.90 §5), with V.8/PCM training displayed live. It uses the deterministic
engine-pair fixture's strict Ja setting on the digital peer. The pattern button
waits until both ends report data ready, then allows one second for the PCM
carrier to settle. Each V.90 process also starts with its V.90 power-on offer;
starting it as V.22 and only changing +MS did not complete PCM training.
The settled SIP pair has returned the complete all-byte pattern; immediate writes at CONNECT produced errors
in an earlier run. This is a GUI test readiness delay, not DSP sample buffering.
V.91 adds a verified symmetric digital PCM pair. The V.92 offer profile
configures both ends with V92 but the analogue role currently suppresses
its V.92 V.8 capability octet (`modem_engine.c`), so the actual connection
is V.90. The selector labels this fallback explicitly; it does not test
V.92 PCM upstream. This is a two-modem local
SIP/G.711 test, not an implementation of V.54 analogue/digital loopbacks.

Both modem logs and telemetry stay in bounded memory, including on the peer,
with no automatic V.34 raw captures. Echo holds at most one 4 KiB block under
backpressure. Closing the window or stopping the launcher terminates both
modems and removes temporary PTYs. SIP server/credentials/network port/profile
options are rejected in loopback mode, which owns its localhost addressing.
Optional explicit diagnostic capture settings retain their usual effect.

`make gui-loopback-test` verifies all ten profiles over real localhost SIP,
byte-exact echoes including all 256 byte values, repeated calls and PTY
cleanup. `python3 tools/gui_loopback_test.py v90` runs just the strict
V.90 payload check. The deterministic
V.90 `engine_pair_test` continues to exchange payloads successfully.
This does not establish hardware interoperability.

The initial reuse-of-processes experiment passed 1200 bit/s but the next
2400 bit/s call ended with NO CARRIER. The loopback runner therefore isolates
each test with fresh SIP/DSP instances; this does not fix or claim to diagnose
that production reconnect issue. Its regression covers switching all offered
profiles, with cleanup between test runs. The UI service and bounded histories
remain in the same window while modem processes restart.

The 33600 bit/s local preset did not train with the default offers and is
excluded from the selector. No DSP negotiation settings are changed by this UI.

V.21 uses the vendored FSK channels at 300 baud (V.21 §§2–4, 7(a)).
The FSK view labels its nominal centre frequency and leaves QAM/eye panels
explicitly unavailable. V.21's 980/1180 Hz and 1650/1850 Hz shifts are visible
in the audio spectrum and waterfall. Native and browser monitoring suppress
the automatic V.90 Phase 3 raw dump as well as ordinary V.34 dumps.

## Native listening playback

RX/TX listening primes 300 ms before playback and limits pending audio to
800 ms. Listening snapshots retain 400 ms, accommodating the 100 ms publisher
and 150 ms GUI poll cadence plus modest jitter. The first prebuffer attempt
used a 200 ms snapshot and whole-buffer completion counts; delayed polls
could repeatedly stop/re-prime playback, while already played buffer prefixes
were counted as pending. The corrected player measures pending samples from
AVAudioPlayerNode's actual sample clock and does not restart at buffer ends.

Switching direction, call epoch changes, genuinely missed ring windows or
excess queued latency re-prime listening. Buffering remains in bounded RAM
and never affects RTP or DSP sample accounting. Outages longer than the
snapshot window still lose listening audio.

`make gui-audio-test` on macOS runs an offline AVAudioEngine check for
partial-buffer occupancy, continuous PCM across scheduled buffers and the
sample-clock origin after a stop/restart. It plays no sound and saves no audio.
