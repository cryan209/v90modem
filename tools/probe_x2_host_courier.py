#!/usr/bin/env python3
"""Fresh x2 host feedback against the original analog Courier emulator.

The sibling courier-emu supplies its existing calibrated analog bearer; no
recorded audio or firmware patches supply either endpoint's response.
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
    parser.add_argument('--capture-native', action='store_true',
                        help='read-only capture of Courier upstream mapper points')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ['ME_V90_UPSTREAM_BIT_DUMP'] = str(output / 'upstream.bits')
    os.environ['ME_V90_UPSTREAM_SYM_DUMP'] = str(output / 'upstream-symbols.txt')
    if args.capture_native:
        os.environ['X2_HOST_NATIVE_CAPTURE'] = str(output / 'native-symbols.json')
    spec = importlib.util.spec_from_file_location('courier_host_probe', EMU / 'tools/probe_v90modem_closed_loop.py')
    probe = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(probe)
    launch = probe.subprocess.Popen

    def native_payload(command, *a, **kw):
        if '--at' in command:
            command = list(command)
            pos = command.index('--at') + 1
            command[pos] = command[pos].replace('ATX1', 'ATE0&M0&H0X1')
            command += ['--send-after-connect', args.message]
            if args.capture_native:
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
    native = json.loads((output / 'analog-result.json').read_text())
    serial = bytes.fromhex(native.get('serial_hex', ''))
    checks = {'marker_4d': '4d' in result['engine']['accepted_markers'],
              'native_connect_x2': b'/x2/' in serial and b'CONNECT ' in serial,
              'engine_exit_ok': result['engine']['engine_exit'] == 0,
              'native_to_host_complete': args.message.encode() in decoded}
    result['checks'] = checks
    result['payload'] = {'expected': args.message,
                         'native_to_host_complete': args.message.encode() in decoded,
                         'decoded_octets': len(decoded),
                         'source': 'waveform receiver bits, before engine qualification gate'}
    (output / 'call.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result['payload'], indent=2))
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
