#!/usr/bin/env python3
"""Grade known foreign payload through the complete x2 engine on a saved RX tap.

This is an offline receiver regression, not a fresh interop call. Different
block sizes must consume every bearer sample and recover the same source.
"""
import argparse
import json
import os
from pathlib import Path
import struct
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def octets(packed):
    bits = [value >> shift & 1 for value in packed for shift in range(7, -1, -1)]
    out = bytearray()
    position = 0
    while position + 10 <= len(bits):
        if bits[position] == 0 and bits[position + 9] == 1:
            out.append(sum(bits[position + 1 + k] << k for k in range(8)))
            position += 10
        else:
            position += 1
    return bytes(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--expected-file', type=Path)
    parser.add_argument('--expected', default='COURIER-X2-0123456789-ABCDEFGHIJKLMNOPQRSTUVWXYZ')
    parser.add_argument('--blocks', default='17,80,160')
    args = parser.parse_args()
    raw = args.capture.read_bytes()
    expected = args.expected_file.read_bytes() if args.expected_file else args.expected.encode('ascii')
    results = []
    for block in map(int, args.blocks.split(',')):
        if block < 1 or block > 4096:
            parser.error('block sizes must be between 1 and 4096')
        wire = b''.join(struct.pack('<H', len(piece)) + piece
                        for offset in range(0, len(raw), block)
                        if (piece := raw[offset:offset + block]))
        with tempfile.TemporaryDirectory(prefix='x2-payload-') as directory:
            root = Path(directory)
            env = os.environ.copy()
            env['ME_MODE'] = 'x2'
            env['ME_V90_UPSTREAM_BIT_DUMP'] = str(root / 'bits')
            env.pop('ME_V90_UPSTREAM_SYM_DUMP', None)
            run = subprocess.run([str(ROOT / 'v90_engine_peer'), str(root / 'pty')],
                                 input=wire, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.PIPE, env=env, timeout=120)
            log = run.stderr.decode(errors='replace')
            bits = root / 'bits'
            decoded = octets(bits.read_bytes()) if bits.exists() else b''
            checks = {'engine_exit': run.returncode == 0,
                      'data': 'state=DATA modulation=X2' in log,
                      'sample_accounting': (f'samples={len(raw)} ' in log and
                                            f'rx={len(raw)} tx={len(raw)}' in log),
                      'three_samples_per_symbol': 'at 9600 Hz T/3' in log,
                      'complete_foreign_source': expected in decoded,
                      'b1_handoff': 'x2 reset-state B1 complete' in log,
                      'no_content_phase_sweep': 'frame phase shift' not in log}
            results.append({'block': block, 'checks': checks})
    print(json.dumps(results, indent=2))
    return 0 if all(all(row['checks'].values()) for row in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
