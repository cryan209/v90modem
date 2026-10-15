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

## ATA test server

`make gui-ata` starts a local UDP SIP listener with a single-endpoint registrar.
The native ATA test server tab and browser panel show address/ports, answering,
registration status, incoming peer, negotiated codec and RTP RX/TX packets,
receive loss and mean jitter. These are PJSIP transport statistics, not modem
CONNECT or decoded-data success. The engine plots and AT/serial consoles remain
available for testing a modem attached to the ATA's analogue port.

```sh
# Explicit LAN address (recommended when several interfaces are available):
./sip_v90_modem --gui --ata-server --bind-addr 192.168.1.20 --local-port 5060 --rtp-port 4000
# Browser alternative:
python3 tools/modem_gui.py --web ./sip_v90_modem --ata-server
```

The launcher chooses the IPv4 address used by the default route when no address
is supplied. Verify it in the panel. Start / apply replaces an idle listener;
hang up first when a call is in progress. Stop terminates the SIP backend and
releases its SIP/RTP sockets while retaining the GUI. The RTP base must be even;
its adjacent port is used for RTCP. Answer rings 0 means manual `ATA`, 1–9 selects
S0 automatic answering. Port/address failures remain visible in the process log
and the GUI stays available for correcting settings.

Copy ATA setup supplies proxy/registrar, username, realm and the endpoint URI.
Configure the ATA to use this LAN address and UDP port as its proxy and registrar;
registration is optional for direct calling. Dial `sip:modem@ADDRESS:PORT` (or use
the ATA's dial plan to route a test number there). Select G.711 PCMU/PCMA and
turn off VAD, echo cancellation and gain processing. SIP and RTP must be reachable
between the ATA and host; the local web control API remains loopback-only.

A nonempty password enables SIP digest authentication for REGISTER and incoming
INVITE. The realm is `modem-test`; configure the same username/password on the ATA.
An empty password disables authentication. Passwords are held in the child
process environment, never returned by the state API or copied into setup text.
CLI-only servers accept `ATA_TEST_USER` / `ATA_TEST_PASSWORD`. GUI fields are
session settings and are not saved to disk.

This is a local single-endpoint test registrar (RFC 3261 §§10.2–10.3, 22.2),
not a PBX or routing proxy. The latest registration's source and expiry are shown;
it does not route calls between subscribers. Registration lifetimes are capped
at one hour. Digest challenges use a per-process nonce. Listener restart clears
registration and authentication state. Direct SIP mode without `--ata-server`
continues to behave as before.

`make gui-ata-test` verifies registration/unregister, authentication challenges,
wrong/stale credentials, an authenticated incoming INVITE, input validation and
stop/restart using localhost UDP. `make gui-smoke-test` separately exercises the
existing bidirectional media/data path. Neither establishes physical ATA interop.

## Apple USB Modem (macOS)

The native and browser GUIs can run the existing A1082 (`05ac:1401`) analogue
backend instead of SIP. This device is a host-driven Motorola SM56 codec/DAA;
it does not expose a serial AT port. Our coupler supplies the engine and AT
interpreter, USB line control and CoreAudio transport. See
`docs/apple_usb_modem_sm56.md` for the hardware findings and live limitations.

```sh
make gui-apple
# Browser alternative, after building the coupler:
python3 tools/modem_gui.py --web --apple ./apple_usb_modem_coupler --hold 3600
```

The GUI starts on-hook. The Apple tab provides USB and CoreAudio device
pickers; refresh lists both independently, so select the matching pair when
using multiple modems. Environment selections remain supported. Saved choices
live in `artifacts/gui-apple/hardware.json` (`APPLE_GUI_HOME` overrides the root).

Check line opens the selected USB device and displays the actual hook register
and sense register. Zero sense means no pair; nonzero values are uncalibrated.
On-hook monitor enables the DAA monitor bit and CoreAudio with silent output,
without feeding the engine or seizing the line. Device audio selects raw codec
RX/TX for waveforms and native FFT, waterfall and listening, including DTMF
and pre-engine audio. Turn it off to inspect the engine's 8 kHz diagnostic taps.
Raw views use the actual codec rate (normally 9600 Hz); recovered eye and
constellation remain engine measurements. Diagnostic playback is output-only.

Watch rings enables on-hook audio and the CID receiver before polling the CDC
interrupt endpoint. RX stays active through rings and the gaps between them; TX
is silent. Disabling the manual monitor cannot stop an enabled ring watch.
Stopping watch stops audio unless the manual monitor is also enabled.
Notifications separated by more
than 1.5 seconds count as separate bursts; this heuristic needs hardware
validation. Auto-answer uses the existing S0 interpreter, with 0 disabling it
and 1–9 selecting a burst count. A saved positive setting resumes ring watching
on launch and after calls. `ATA` waits for a ring unless one was recently seen;
`--ring-wait` defaults to 60 seconds. The CLI `--answer` also listens on-hook;
`--answer-rings 2` waits for a second burst to leave time for inter-ring CID
(the CLI default remains one burst). Use S0=2 or higher in the GUI for this
style of CID; S0=1 answers before the inter-ring packet can arrive. `ATDnumber` dials the physical line.
`ATH`, closing the GUI, or the `--hold` limit releases an active call.

Caller ID is decoded from a private audio copy at 8 kHz by the vendored
SpanDSP Bell 202 and V.23 receivers. SDMF and MDMF number, name, received
date/time and private/out-of-area reasons appear in both GUIs. `AT+VCID=1`
enables formatted `DATE`, `TIME`, `NMBR` and `NAME` reports (V.253 §9.2.3.1.3).
Pre-ring CID is retained until the first RING; inter-ring CID is reported when
received, without resetting S0's ring count. Invalid checksums are rejected;
there is no invented caller timestamp when no packet was received. Repeated
packets in one ring sequence are suppressed. The telemetry `decoder` field
names the receiver that accepted the packet, not a measured modulation type:
the two FSK passbands overlap. DTMF CID and call-waiting CID are not implemented.

The line measurements update once a second while the device is open. They show
raw signed sense and an **inferred, uncalibrated voltage estimate**, off-hook
current code and approximate mA (3.3 mA/code), link validity, the hardware
protection enable bit and receive overload. Hardware OPD, current code 31 or
the USB overload notification can trip the coupler guard.
Dropout is meaningful only off-hook. A missing read is unavailable, not zero.
The coupler checks for current overload during seizure and active calls, and
recognizes the driver's USB overload notification; it releases the hook and
ends the call on either indication. This is host-side detection, not a claim
of independently certified electrical protection or digital-line identification.
`apple_usb_modem_probe --line-status` returns the same read-only JSON snapshot.
The probe itself does not apply the coupler's automatic guard.

Save AT settings writes profile 0. The launcher saves it before dial/answer and
before device, recording or replay restarts; each fresh backend loads it from
`artifacts/gui-apple/profile.at`. This retains `AT+MS` across calls. Each call
still owns a fresh process, clearing all per-call DSP state. Errors remain in
the process log; startup/configuration errors stop the launcher.

Record RX/TX creates a timestamped directory under `artifacts/gui-apple/captures`
with `session.json`, `rx.s16` and `tx.s16`: signed little-endian raw codec samples
at the recorded rate. Recording starts when CoreAudio starts and continues from
on-hook monitoring into a call. Replay RX feeds a selected capture through the
existing receive path at its recorded pace without opening USB or seizing a
line. Stop replay returns to the live idle backend and its original rate.
Recordings consume disk space until deleted; ordinary GUI telemetry stays bounded.

The Apple diagnostics show raw DC, RMS after removing the mean, near-rail sample
count (absolute amplitude at least 32760), CoreAudio render errors, empty TX
samples and TX ring overruns. Both GUIs also show the recent window's minimum/maximum, AC RMS in dBFS
(peak full scale reference), and a literal sample classification: no samples,
digital silence, constant input, constant rail, near rail or varying signal.
The bounded window is at most 1600 samples per direction; zero and near-rail
window counts and total sample counts are available in `hardware_audio.metrics`.
`clipped` remains a cumulative near-rail count, not proof of analogue clipping.
A constant rail can accompany an unpowered path or off-hook operation without a
pair; these labels do not diagnose a bearer or estimate SNR. Zero AC RMS has
JSON `null` dBFS and displays as −∞. Engine TX gap runs/samples count only
underruns while the engine runs, separating them from expected idle empty TX.
These measurements do not change samples, gains,
resamplers or the SIP G.711 path (V.90 §5; V.34 §10.1).

`make gui-apple-test` runs `tests/apple.list` through the normal isolated test
runner, with logs in `test-logs/apple`. The GUI integration check uses
deliberately invalid USB address `0:0` and a synthetic
capture to verify settings persistence, device validation, failure recovery,
capture metadata, bounded raw replay, stopping and rate restoration. The
coupler's `--diagnostic-selftest` checks raw copies, metrics and ring debounce.
The CID waveform test covers both rates and carrier pairs, SDMF/MDMF, privacy,
bad checksums and malformed fields. These and the resampler selftest do not
establish live hardware interoperability. Live CID checks on both modems and
the second-ring answer transition are recorded in `docs/apple_usb_modem_sm56.md`.

To decode an on-hook recording without hardware:

```sh
./apple_usb_modem_coupler --cid-replay path/to/rx.s16 --rate 9600
```


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

Except for explicitly enabled Apple raw recording, no GUI waveform, byte or
process-log files are written. GUI mode also disables
the engine’s automatic V.34 training raw files unless `ME_DUMP_DIR` is explicitly
set. The existing optional capture environment variables still have their
existing behavior if explicitly set. The monitor retains a one-second audio ring each way, 256 wire octets and
256 G.711 codes each way, 256 measured and 256 decided IQ points, and 16 phase
events. Snapshots expose just the latest 512 audio samples.

Each control/data/log stream holds at most 256 chunks of at most 4 KiB in the
Python relay. Every native/browser console displays at most 24,000 characters;
training history retains at most 120 events. New output evicts old output.
Apple raw recording is the explicit file-archive exception; there is no
telemetry backlog for a slow client. A paused UI
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

The native Audio tab shows an RX amplitude histogram and overlaid RX (teal)
and TX (orange) Hann-windowed spectra. The fixed −90 to 0 dBFS scale shows
one-sided peak amplitude: a full-scale bin-centred sine reads 0 dBFS. Hover
for the nearest bin's frequency and both levels. Carrier markers show the
recovered RX and nominal TX frequencies where available.

The FFT selector applies to the spectrum and both waterfalls: 128, 256 or
512 samples, giving 62.5, 31.25 or 15.625 Hz bin spacing at 8 kHz. These use
the most recent contiguous samples (16, 32 or 64 ms); Hann-window resolution
is broader than bin spacing. Average applies exponential smoothing to power
(80% previous, 20% new) on fresh updates. Peak hold adds dashed traces of the
maximum unaveraged amplitude; Clear peaks restarts them from current levels.
Changing options or starting a new call resets spectrum histories.

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
newest rows appear at the top. Each row uses the selected FFT size with a
Hann window. Colour is fixed from −90 to 0 dBFS, so level changes remain
visible. Yellow lines mark the recovered RX / nominal TX carrier where
available. Each direction holds exactly 128 rows in memory, overwriting old
rows. Duplicate sample counters add no rows; call reset or option changes
clear the display, and Freeze pauses it.

Timestamp labels use the per-call 8 kHz sample counter at the end of each
window. Horizontal markers label observed call-state and RX/TX signal
transitions at the first GUI update showing them, rather than claiming an
exact protocol-event timestamp. Rows represent received GUI updates; gaps
are omitted, explicitly labelled, and never reconstructed. The displayed
time range comes from the oldest and newest retained sample counters.

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
V.91 adds a verified symmetric digital PCM pair. The V.92 profile sets
`AT+MS=V92,0` and `AT+PIG=0` on both ends (+PIG is factory 1 here), so the
pair negotiates V.92 PCM upstream through INFO0 bits 26/27 and a Table 18
INFO1a; the test asserts V.92 and PCM in both directions. (V.92 capability
is carried in INFO0, not the V.8 QC octet; the earlier fallback was a
Phase 2 deadlock and an undrained receive path, see
`docs/v92_full_startup.md`, 2026-10-11.) This is a two-modem local
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


### Apple line events and hardware self-test

The Apple panel offers **Hardware self-test** while idle and on-hook. It pauses
watching for a roughly three-second internal digital loopback, reports sample
and bit errors, verifies register restoration, and resumes monitoring. Stop raw
recording before using it. A pass tests the USB/digital sample path; it does not
test the telephone line or establish hardware modem interoperability.

The web panel's **Line event log** shows the latest 16 events; the native panel
shows the latest event with recent history in its tooltip. Persistent JSONL
includes UTC and monotonic timestamps, process ID, hook, ring/CID, line readings,
protection events and engine transitions. It includes received caller fields.
The launcher writes `line-events.jsonl` under its Apple GUI home (default `artifacts/gui-apple/`); raw recordings
keep it beside their RX/TX captures. The file rotates at 2 MiB with one `.1`
backup. For direct coupler use, choose `--line-events FILE` or the diagnostic
`APPLE_MODEM_LINE_EVENTS` environment variable. These observations do not alter
DSP samples or the SIP audio path.
