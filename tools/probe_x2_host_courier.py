#!/usr/bin/env python3
"""Fresh x2 host feedback against the original analog Courier emulator.

The sibling courier-emu supplies its existing calibrated analog bearer; no
recorded audio or firmware patches supply either endpoint's response. Explicit
X2_HOST_DIAGNOSTIC_* controls mark modified native-runtime experiments.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
EMU = Path(os.environ.get('X2_HOST_EMU_ROOT', ROOT.parent / 'courier-emu'))
sys.path.insert(0, str(EMU))


def async_octets(packed):
    """Decode 8N1 source bits (dump packing is MSB first)."""
    bits = [(v >> shift) & 1 for v in packed for shift in range(7, -1, -1)]
    out = bytearray()
    i = 0
    while i + 10 <= len(bits):
        if bits[i] == 0 and bits[i + 9] == 1:
            out.append(sum(bits[i + 1 + k] << k for k in range(8)))
            i += 10
        else:
            i += 1
    return bytes(out)


def native_payload_chunks(message, chunk_bytes):
    """Bound unflow-controlled DTE bursts; Courier schedules chunks 1.5 s apart."""
    if chunk_bytes < 0:
        raise ValueError('native chunk size must be nonnegative')
    if not message:
        return []
    if not chunk_bytes:
        return [message]
    return [message[i:i + chunk_bytes] for i in range(0, len(message), chunk_bytes)]


def read_serial_capture(native, path):
    """Only a complete capture is eligible for payload delivery checks."""
    if path.exists():
        serial = path.read_bytes()
        total = native.get('serial_bytes_total')
        if total is None or len(serial) != total:
            raise ValueError(f'serial capture length {len(serial)} != reported total {total}')
        return serial
    if native.get('serial_truncated'):
        raise ValueError('serial report truncated and full-stream capture is missing')
    raise ValueError('full-stream serial capture is missing')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--engine', type=Path, default=ROOT / 'v90_engine_peer',
                        help='engine executable (use a pinned copy for matched controls)')
    parser.add_argument('--modulation', choices=['x2', 'v90'], default='x2',
                        help='requested analog modulation; rejects fallback in validation')
    parser.add_argument('--instructions', type=int, default=180000000)
    parser.add_argument('--fast', action='store_true')
    parser.add_argument('--message', default='COURIER-X2-HOST-0123456789\r\n')
    parser.add_argument('--host-message', default='HOST-X2-0123456789\r\n')
    parser.add_argument('--host-at', type=int, default=120000,
                        help='inject host source bytes at this bearer sample')
    parser.add_argument('--host-after-rate-change', action='store_true',
                        help='with --pty-source, wait for a completed x2 rate change '
                             'before queuing host payload')
    parser.add_argument('--host-message-file', type=Path,
                        help='host source bytes from a file (overrides --host-message)')
    parser.add_argument('--host-pace-samples', type=int, default=0,
                        help='with --pty-source, queue at most one host byte per this many '
                             'bearer samples (0: one burst). Without error control nothing '
                             'stops a burst faster than the Courier DTE rate overrunning it')
    parser.add_argument('--dsp-watch', metavar='ADDR',
                        help='record every native DSP write to this data cell (hex), '
                             'replacing the default 039f watch')
    parser.add_argument('--native-arg', action='append', default=[],
                        help='extra argument for the Courier emulator run (repeatable)')
    parser.add_argument('--courier-at', default='S39=8',
                        help="extra Courier AT settings inserted before dialling. The default "
                             "restores the factory V.34 transmit level index: the emulated "
                             "unit's NVRAM holds S39=0, which its own parser rejects (1..29) "
                             "and which wraps its transmit word above 4800 bit/s. Pass '' "
                             "for the unmodified NVRAM profile")
    parser.add_argument('--pty-source', action='store_true',
                        help='send and receive through the real engine PTY after CONNECT')
    parser.add_argument('--engine-at', action='append', default=[],
                        help="with --pty-source, an AT command for our engine's PTY before "
                             "the call starts, as its DTE would send (e.g. AT+MS=X2,0,0,48000); "
                             'repeatable')
    parser.add_argument('--fixed-native-rate', action='store_true',
                        help='retain the old diagnostic &U26&N39 Courier rate clamp')
    parser.add_argument('--native-chunk-bytes', type=int, default=0,
                        help='split native DTE source into chunks scheduled 1.5 guest seconds '
                             'apart (0: legacy burst); bounds offered load with &H0')
    parser.add_argument('--capture-native', action='store_true',
                        help='read-only capture of Courier upstream mapper points')
    parser.add_argument('--capture-receiver', action='store_true',
                        help='read-only capture of Courier PCM receive frame state')
    parser.add_argument('--capture-rate', action='store_true',
                        help='read-only capture of Courier 403 rate metric and ceilings')
    parser.add_argument('--clock-ppm', type=float, default=None,
                        help='resample the bearer as an analogue line does: the '
                             'network (engine) clock runs this many ppm fast of '
                             'the Courier (0 = resampler only; a Courier on a '
                             'Cisco FXS port measured +80)')
    parser.add_argument('--clock-phase', type=float, default=0.0,
                        help='with --clock-ppm: starting sampling offset between '
                             'the two clocks, in samples (0.5 = half a sample)')
    parser.add_argument('--codec-cutoff', type=float, default=0.0,
                        help='with --clock-ppm: low-pass both directions at this '
                             'frequency first, as a G.712 codec does (3400)')
    parser.add_argument('--line-fir', type=Path,
                        help='with --clock-ppm: filter both directions with these '
                             'taps (one per line), e.g. a response measured on a '
                             'real line, instead of --codec-cutoff/--codec-highpass')
    parser.add_argument('--line-taps', type=int, default=16,
                        help='with --clock-ppm: half-length of the resampling '
                             'sinc (16 rolls off above about 3.6 kHz at half a sample)')
    parser.add_argument('--line-linear', action='store_true',
                        help='with --clock-ppm: hand the Courier the line in linear '
                             'samples instead of requantising it to PCMU (a real '
                             'analogue line adds no mu-law noise of its own)')
    parser.add_argument('--line-noise-db', type=float, default=None,
                        help='with --line-linear: add white noise this many dB '
                             'below the downstream RMS, per sample')
    parser.add_argument('--line-down-only', action='store_true',
                        help='with --clock-ppm 0: impair only our downstream; the '
                             "Courier's upstream reaches the engine untouched")
    parser.add_argument('--codec-highpass', type=float, default=0.0,
                        help='with --clock-ppm: also remove everything below this '
                             'frequency (a Cisco FXS port is 12 dB down at 200 Hz)')
    parser.add_argument('--path-delay-ms', type=int, default=0,
                        help='one-way bearer delay added in each direction (a live '
                             'SIP/FXS path measured about 150 ms each way)')
    args = parser.parse_args()
    if args.native_chunk_bytes < 0:
        parser.error('--native-chunk-bytes must be nonnegative')
    if args.capture_rate and (args.capture_native or args.capture_receiver):
        parser.error('--capture-rate cannot be combined with other capture modes')
    if args.host_after_rate_change and not args.pty_source:
        parser.error('--host-after-rate-change requires --pty-source')
    if args.engine_at and not args.pty_source:
        parser.error('--engine-at requires --pty-source')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    source = output / 'host-source.bin'
    if args.host_message_file:
        args.host_message = args.host_message_file.read_bytes().decode('ascii')
    source.write_bytes(args.host_message.encode('ascii'))
    os.environ['ME_V90_UPSTREAM_BIT_DUMP'] = str(output / 'upstream.bits')
    os.environ['ME_V90_UPSTREAM_SYM_DUMP'] = str(output / 'upstream-symbols.txt')
    if args.capture_native or args.capture_receiver or args.capture_rate:
        os.environ['X2_HOST_CAPTURE_RATE'] = '1' if args.capture_rate else '0'
        os.environ['X2_HOST_NATIVE_CAPTURE'] = str(output / 'native-symbols.json')
        os.environ['X2_HOST_CAPTURE_RECEIVER'] = '1' if args.capture_receiver else '0'
        os.environ['COURIER_DSP_DUMP'] = str(output / 'native-dump')
    spec = importlib.util.spec_from_file_location('courier_host_probe', EMU / 'tools/probe_v90modem_closed_loop.py')
    probe = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(probe)
    launch = probe.subprocess.Popen

    if args.pty_source:
        base_peer = probe.EnginePeer

        class PtyPeer(base_peer):
            def start(self):
                super().start()
                if not hasattr(self, 'dte'):
                    self.dte = bytearray()
                    self.pty_fd = None
                    self.source_offset = 0
                    self.source_first_sample = None
                    self.source_pace_origin = None
                    self.rate_change_seen = False
                    self.engine_at_replies = []
                    for command in args.engine_at:
                        self.engine_at_replies.append(self.send_at(command))

            def send_at(self, command, timeout=10.0):
                """One DTE command before the first bearer frame; returns the reply."""
                deadline = time.monotonic() + timeout
                while self.pty_fd is None and time.monotonic() < deadline:
                    self.drain()
                    if self.pty_fd is None:
                        time.sleep(0.05)
                if self.pty_fd is None:
                    raise RuntimeError('engine PTY never appeared')
                start = len(self.dte)
                os.write(self.pty_fd, command.encode('ascii') + b'\r')
                while time.monotonic() < deadline:
                    self.drain()
                    reply = bytes(self.dte[start:])
                    if b'OK\r\n' in reply or b'ERROR\r\n' in reply:
                        del self.dte[start:]
                        if b'OK\r\n' not in reply:
                            raise RuntimeError(f'{command}: {reply!r}')
                        return reply.decode('ascii', 'replace')
                    time.sleep(0.02)
                raise RuntimeError(f'{command}: no reply')

            def drain(self):
                if self.pty_fd is None:
                    try:
                        self.pty_fd = os.open('/private/tmp/' + self.output.name + '-pty',
                                              os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
                    except FileNotFoundError:
                        return
                while True:
                    try:
                        data = os.read(self.pty_fd, 4096)
                    except (BlockingIOError, OSError):
                        break
                    if not data:
                        break
                    self.dte.extend(data)

            def exchange(self, octets):
                reply = super().exchange(octets)
                self.drain()
                payload = args.host_message.encode('ascii')
                if args.host_after_rate_change and not self.rate_change_seen:
                    self.rate_change_seen = 'X2 rate change complete:' in (
                        self.output / 'engine.log').read_text(errors='replace')
                if (b'CONNECT ' in self.dte and self.samples >= args.host_at
                        and (not args.host_after_rate_change or self.rate_change_seen)
                        and self.source_offset < len(payload)):
                    if self.source_pace_origin is None:
                        self.source_pace_origin = self.samples
                    end = len(payload)
                    if args.host_pace_samples > 0:
                        end = min(end, 1 + (self.samples - self.source_pace_origin) // args.host_pace_samples)
                    try:
                        if end > self.source_offset:
                            written = os.write(self.pty_fd, payload[self.source_offset:end])
                            if written and self.source_first_sample is None:
                                self.source_first_sample = self.samples
                            self.source_offset += written
                    except BlockingIOError:
                        pass
                return reply

            def stop(self):
                if hasattr(self, 'dte'):
                    self.drain()
                    (self.output / 'engine-dte.bin').write_bytes(self.dte)
                super().stop()
                if getattr(self, 'pty_fd', None) is not None:
                    os.close(self.pty_fd)

            def status(self):
                status = super().status()
                status['dte_hex'] = bytes(getattr(self, 'dte', b'')).hex()
                status['source_queued'] = getattr(self, 'source_offset', 0)
                status['source_first_sample'] = getattr(self, 'source_first_sample', None)
                status['source_after_rate_change'] = getattr(self, 'rate_change_seen', False)
                status['engine_at'] = getattr(self, 'engine_at_replies', [])
                return status

        probe.EnginePeer = PtyPeer

    if args.path_delay_ms > 0:
        delayed_peer = probe.EnginePeer
        delay_octets = args.path_delay_ms * 8

        class DelayedPeer(delayed_peer):
            """The same call with each direction held in a FIFO of idle PCMU."""
            def exchange(self, octets):
                if not hasattr(self, 'to_engine'):
                    self.to_engine = bytearray(b'\xff' * delay_octets)
                    self.to_courier = bytearray(b'\xff' * delay_octets)
                if not octets:
                    return b''
                self.to_engine.extend(octets)
                head = bytes(self.to_engine[:len(octets)])
                del self.to_engine[:len(octets)]
                self.to_courier.extend(super().exchange(head))
                tail = bytes(self.to_courier[:len(octets)])
                del self.to_courier[:len(octets)]
                return tail

        probe.EnginePeer = DelayedPeer

    if args.clock_ppm is not None:
        import math
        import random
        random.seed(1)
        resampled_peer = probe.EnginePeer
        ratio = 1 + args.clock_ppm * 1e-6
        half = args.line_taps

        def interpolate(history, position):
            base = int(position)
            total = 0.0
            for k in range(-half + 1, half + 1):
                x = k - (position - base)
                weight = 1.0 if abs(x) < 1e-9 else math.sin(math.pi * x) / (math.pi * x)
                total += history[base + k] * weight * (0.54 + 0.46 * math.cos(math.pi * x / half))
            return total

        codec = []
        if args.codec_cutoff > 0 or args.codec_highpass > 0:
            def low_pass(cutoff):
                out = []
                for k in range(-63, 64):
                    x = 2 * cutoff / 8000 * k
                    weight = 1.0 if k == 0 else math.sin(math.pi * x) / (math.pi * x)
                    out.append(2 * cutoff / 8000 * weight
                               * (0.54 + 0.46 * math.cos(math.pi * k / 63)))
                return out
            codec = (low_pass(args.codec_cutoff) if args.codec_cutoff > 0
                     else [1.0 if k == 63 else 0.0 for k in range(127)])
            if args.codec_highpass > 0:
                codec = [a - b for a, b in zip(codec, low_pass(args.codec_highpass))]

        if args.line_fir:
            codec = [float(v) for v in args.line_fir.read_text().split()]

        def band_limit(state, samples):
            if not codec:
                return samples
            out = []
            for v in samples:
                state.append(v)
                del state[:-len(codec)]
                out.append(sum(a * b for a, b in zip(state, codec))
                           if len(state) == len(codec) else 0.0)
            return out

        class ResampledPeer(resampled_peer):
            """Both directions through a windowed-sinc resampler in the linear
            domain and back to PCMU: no codeword survives, as on a real line."""
            def exchange(self, octets):
                if not hasattr(self, 'courier_line'):
                    self.courier_line = [0.0] * (2 * half)
                    self.engine_line = [0.0] * (2 * half)
                    self.courier_at = self.engine_at = half + args.clock_phase
                    self.engine_due = bytearray()
                    self.courier_codec, self.engine_codec = [], []
                    self.line_power = 0.0
                if not octets:
                    return b''
                if args.line_down_only:
                    if ratio != 1 and len(self.engine_line) == 2 * half:
                        # room for the reader to gain on (or fall behind) the
                        # engine: 400 samples covers 100 ppm for 500 s
                        self.engine_line.extend([0.0] * 400)
                    reply = super().exchange(octets)
                    self.engine_line.extend(band_limit(
                        self.engine_codec, [probe.ulaw_to_linear(v) for v in reply]))
                    octets_in = b''
                else:
                    octets_in = octets
                self.courier_line.extend(band_limit(
                    self.courier_codec, [probe.ulaw_to_linear(v) for v in octets_in]))
                line = self.courier_line
                while self.courier_at + half < len(line):
                    self.engine_due.append(probe.linear_to_ulaw(
                        max(-32768, min(32767, round(interpolate(line, self.courier_at))))))
                    self.courier_at += 1 / ratio
                while len(self.engine_due) >= len(octets):
                    reply = super().exchange(bytes(self.engine_due[:len(octets)]))
                    del self.engine_due[:len(octets)]
                    self.engine_line.extend(band_limit(
                        self.engine_codec, [probe.ulaw_to_linear(v) for v in reply]))
                line = self.engine_line
                out = [] if args.line_linear else bytearray()
                while len(out) < len(octets):
                    if self.engine_at + half < len(line):
                        value = interpolate(line, self.engine_at)
                        if args.line_noise_db is not None:
                            self.line_power += (value * value - self.line_power) / 4000
                            value += random.gauss(0, math.sqrt(self.line_power)
                                                  * 10 ** (-args.line_noise_db / 20))
                        out.append(float(value) if args.line_linear else probe.linear_to_ulaw(
                            max(-32768, min(32767, round(value)))))
                        self.engine_at += ratio
                    else:
                        out.append(0.0 if args.line_linear else 0xff)
                drop = int(min(self.courier_at, len(self.courier_line))) - 4 * half
                if drop > 4096:
                    del self.courier_line[:drop]
                    self.courier_at -= drop
                drop = int(self.engine_at) - 4 * half
                if drop > 4096:
                    del self.engine_line[:drop]
                    self.engine_at -= drop
                return out if args.line_linear else bytes(out)

        if args.line_linear:
            # analog() turns each octet we return into a linear sample; let a
            # float through as the sample it already is.
            decode = probe.ulaw_to_linear
            probe.ulaw_to_linear = lambda v: v if isinstance(v, float) else decode(v)
        probe.EnginePeer = ResampledPeer

    def native_payload(command, *a, **kw):
        if str(command[0]) == str(args.engine) and not args.pty_source:
            command = [*command, '--tx-file', str(source), '--tx-at', str(args.host_at)]
        if '--at' in command:
            command = list(command)
            pos = command.index('--at') + 1
            # Sustained V.90 must let the modem stop the host during
            # firmware buffer pressure/retrain. The analog emulator observes
            # the real active-low CS/CTS latch; disabling it loses DTE bytes
            # above the UART without an OE indication. Retain x2's old profile.
            flow = '&H1' if args.modulation == 'v90' else '&H0'
            command[pos] = command[pos].replace('ATX1', f'ATE0&M0{flow}X1')
            if args.courier_at:
                command[pos] = command[pos].replace('DT', args.courier_at + 'DT', 1)
            if not args.fixed_native_rate:
                command[pos] = command[pos].replace('&U26&N39', '')
            if args.dsp_watch and '--dsp-write-watch' in command:
                command[command.index('--dsp-write-watch') + 1] = args.dsp_watch
            for chunk in native_payload_chunks(args.message, args.native_chunk_bytes):
                command += ['--send-after-connect', chunk]
            command += [*args.native_arg, '--serial-output', str(output / 'courier-dte.bin')]
            if args.capture_native or args.capture_receiver or args.capture_rate:
                assert command[1:3] == ['-m', 'courier_emu']
                command = [command[0], str(ROOT / 'tools/x2_courier_capture.py'),
                           'cli', *command[3:]]
        return launch(command, *a, **kw)

    probe.subprocess.Popen = native_payload
    options = argparse.Namespace(output=output, engine=args.engine,
                                 instructions=args.instructions, fast=args.fast,
                                 modulation=args.modulation)
    try:
        probe.analog(options)
    finally:
        probe.subprocess.Popen = launch
    bits = output / 'upstream.bits'
    decoded = async_octets(bits.read_bytes()) if bits.exists() else b''
    (output / 'upstream-decoded.bin').write_bytes(decoded)
    result = json.loads((output / 'call.json').read_text())
    result['native_runtime_controls'] = {
        'diagnostic_tx_gain': os.environ.get('X2_HOST_DIAGNOSTIC_TX_GAIN'),
        'diagnostic_library': os.environ.get('X2_HOST_CAPTURE_LIBRARY'),
    }
    try:
        native = json.loads((output / 'analog-result.json').read_text())
    except (OSError, json.JSONDecodeError) as exc:
        result['validation_status'] = 'invalid_capture'
        result['validation_error'] = f'native summary missing or incomplete: {exc}'
        result['checks'] = {'capture_valid': False}
        (output / 'call.json').write_text(json.dumps(result, indent=2) + '\n')
        print(f'INVALID TEST: {result["validation_error"]}', file=sys.stderr)
        return 2
    try:
        serial = read_serial_capture(native, output / 'courier-dte.bin')
    except ValueError as exc:
        result['validation_status'] = 'invalid_capture'
        result['validation_error'] = str(exc)
        result['checks'] = {'capture_valid': False}
        (output / 'call.json').write_text(json.dumps(result, indent=2) + '\n')
        print(f'INVALID TEST: {exc}', file=sys.stderr)
        return 2
    result['validation_status'] = 'complete_capture'
    result['serial_capture'] = {'path': str(output / 'courier-dte.bin'),
                                'bytes': len(serial),
                                'report_truncated': native.get('serial_truncated', False)}
    received = bytes.fromhex(result['engine'].get('dte_hex', '')) if args.pty_source else decoded
    serial_data = bytes(value & 0x7f for value in serial)
    checks = {'native_connect_requested_modulation':
                  (b'/' + args.modulation.encode() + b'/') in serial_data.lower()
                  and b'CONNECT ' in serial_data,
              'engine_exit_ok': result['engine']['engine_exit'] == 0,
              'native_to_host_complete': args.message.encode() in received,
              # Courier's DTE is 7E1; retain the untouched wire octets too.
              'host_to_native_complete': args.host_message.encode() in
                  bytes(value & 0x7f for value in serial)}
    result['requested_modulation'] = args.modulation
    if args.modulation == 'x2':
        checks['marker_4d'] = '4d' in result['engine']['accepted_markers']
        checks['native_connect_x2'] = checks['native_connect_requested_modulation']
    else:
        checks['engine_selected_v90'] = 'V8 selected V90' in (
            output / 'engine.log').read_text(errors='replace')
    # Exact delivery: after the CONNECT line, the DTE must see the source and
    # nothing else (a trailing result code such as NO CARRIER is allowed).
    # "Contains the message" let 71 KB of post-collapse noise pass.
    def exact_after_connect(stream, payload):
        start = stream.find(b'CONNECT ')
        if start < 0:
            return False
        body = stream[stream.find(b'\r\n', start) + 2:]
        for code in (b'\r\nNO CARRIER\r\n', b'NO CARRIER\r\n'):
            if body.endswith(code):
                body = body[:-len(code)]
                break
        return body == payload
    checks['host_to_native_exact'] = exact_after_connect(
        bytes(value & 0x7f for value in serial), args.host_message.encode())
    if args.pty_source:
        checks['native_to_host_exact'] = exact_after_connect(received, args.message.encode())
        checks['engine_connect'] = b'CONNECT ' in received
        checks['pty_source_queued'] = result['engine']['source_queued'] == len(args.host_message.encode())
        if args.host_after_rate_change:
            checks['host_queued_after_rate_change'] = (
                result['engine']['source_after_rate_change']
                and result['engine']['source_first_sample'] is not None)
    result['checks'] = checks
    result['courier_at'] = args.courier_at
    result['payload'] = {'expected': args.message,
                         'native_chunk_bytes': args.native_chunk_bytes,
                         'host_pace_samples': args.host_pace_samples,
                         'host_expected': args.host_message,
                         'host_source_at_sample': args.host_at,
                         'host_after_rate_change': args.host_after_rate_change,
                         'host_to_native_complete': checks['host_to_native_complete'],
                         'native_to_host_complete': args.message.encode() in received,
                         'decoded_octets': len(decoded),
                         'source': 'engine PTY' if args.pty_source else 'waveform receiver bits'}
    (output / 'call.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result['payload'], indent=2))
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
