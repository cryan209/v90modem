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
- **Eye:** a two-period view folded from line samples using manual baud, carrier
  and phase. The native PCM view uses an eightfold windowed-sinc reconstruction
  for display; QAM also mixes down an I channel and applies a short smoother.
  The browser uses direct sample interpolation. These are exploratory
  line-derived eyes, **not recovered-clock/equalizer eyes** or a model of the
  remote D/A's actual filter. Eye settings never affect DSP. For a V.90 digital
  call the PCM downstream is TX, while the QAM upstream is RX; the analogue
  role reverses those directions. V.92 can use PCM in both directions.
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
