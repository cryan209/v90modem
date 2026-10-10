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

ROOT = Path(__file__).resolve().parents[1]
EMU = ROOT.parent / 'courier-emu'
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--instructions', type=int, default=180000000)
    parser.add_argument('--fast', action='store_true')
    parser.add_argument('--message', default='COURIER-X2-HOST-0123456789\r\n')
    parser.add_argument('--host-message', default='HOST-X2-0123456789\r\n')
    parser.add_argument('--host-at', type=int, default=120000,
                        help='inject host source bytes at this bearer sample')
    parser.add_argument('--host-message-file', type=Path,
                        help='host source bytes from a file (overrides --host-message)')
    parser.add_argument('--host-pace-samples', type=int, default=0,
                        help='with --pty-source, queue at most one host byte per this many '
                             'bearer samples (0: one burst). Without error control nothing '
                             'stops a burst faster than the Courier DTE rate overrunning it')
    parser.add_argument('--pty-source', action='store_true',
                        help='send and receive through the real engine PTY after CONNECT')
    parser.add_argument('--fixed-native-rate', action='store_true',
                        help='retain the old diagnostic &U26&N39 Courier rate clamp')
    parser.add_argument('--capture-native', action='store_true',
                        help='read-only capture of Courier upstream mapper points')
    parser.add_argument('--capture-receiver', action='store_true',
                        help='read-only capture of Courier PCM receive frame state')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    source = output / 'host-source.bin'
    if args.host_message_file:
        args.host_message = args.host_message_file.read_bytes().decode('ascii')
    source.write_bytes(args.host_message.encode('ascii'))
    os.environ['ME_V90_UPSTREAM_BIT_DUMP'] = str(output / 'upstream.bits')
    os.environ['ME_V90_UPSTREAM_SYM_DUMP'] = str(output / 'upstream-symbols.txt')
    if args.capture_native or args.capture_receiver:
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
                if (b'CONNECT ' in self.dte and self.samples >= args.host_at
                        and self.source_offset < len(payload)):
                    end = len(payload)
                    if args.host_pace_samples > 0:
                        end = min(end, 1 + (self.samples - args.host_at) // args.host_pace_samples)
                    try:
                        if end > self.source_offset:
                            self.source_offset += os.write(self.pty_fd, payload[self.source_offset:end])
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
                return status

        probe.EnginePeer = PtyPeer

    def native_payload(command, *a, **kw):
        if str(command[0]) == str(ROOT / 'v90_engine_peer') and not args.pty_source:
            command = [*command, '--tx-file', str(source), '--tx-at', str(args.host_at)]
        if '--at' in command:
            command = list(command)
            pos = command.index('--at') + 1
            command[pos] = command[pos].replace('ATX1', 'ATE0&M0&H0X1')
            if not args.fixed_native_rate:
                command[pos] = command[pos].replace('&U26&N39', '')
            command += ['--send-after-connect', args.message]
            if args.capture_native or args.capture_receiver:
                assert command[1:3] == ['-m', 'courier_emu']
                command = [command[0], str(ROOT / 'tools/x2_courier_capture.py'),
                           'cli', *command[3:]]
        return launch(command, *a, **kw)

    probe.subprocess.Popen = native_payload
    options = argparse.Namespace(output=output, engine=ROOT / 'v90_engine_peer',
                                 instructions=args.instructions, fast=args.fast)
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
    native = json.loads((output / 'analog-result.json').read_text())
    serial = bytes.fromhex(native.get('serial_hex', ''))
    received = bytes.fromhex(result['engine'].get('dte_hex', '')) if args.pty_source else decoded
    checks = {'marker_4d': '4d' in result['engine']['accepted_markers'],
              'native_connect_x2': b'/x2/' in serial and b'CONNECT ' in serial,
              'engine_exit_ok': result['engine']['engine_exit'] == 0,
              'native_to_host_complete': args.message.encode() in received,
              # Courier's DTE is 7E1; retain the untouched wire octets too.
              'host_to_native_complete': args.host_message.encode() in
                  bytes(value & 0x7f for value in serial)}
    if args.pty_source:
        checks['engine_connect'] = b'CONNECT ' in received
        checks['pty_source_queued'] = result['engine']['source_queued'] == len(args.host_message.encode())
    result['checks'] = checks
    result['payload'] = {'expected': args.message,
                         'host_pace_samples': args.host_pace_samples,
                         'host_expected': args.host_message,
                         'host_source_at_sample': args.host_at,
                         'host_to_native_complete': checks['host_to_native_complete'],
                         'native_to_host_complete': args.message.encode() in received,
                         'decoded_octets': len(decoded),
                         'source': 'engine PTY' if args.pty_source else 'waveform receiver bits'}
    (output / 'call.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result['payload'], indent=2))
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
