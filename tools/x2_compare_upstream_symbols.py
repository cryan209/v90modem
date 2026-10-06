#!/usr/bin/env python3
"""Compare native Courier mapper points with our waveform receiver output.

Uses actual point values to find the start offset; file lengths and wall-clock
origins do not establish alignment. Capture with probe_x2_host_courier.py.
"""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--frame-bits', type=int, default=12)
    args = parser.parse_args()
    root = args.capture
    trace = json.loads((root / 'native-symbols.json').read_text())
    # NativeC5x capture header has five words; use the recorded address list.
    addresses = trace['addresses']
    re = 5 + addresses.index(0x3f8)
    im = 5 + addresses.index(0x3f9)
    b = 5 + addresses.index(0x3a4)
    def signed(value):
        return (value + 32768) % 65536 - 32768
    native = [complex(signed(row[re]), signed(row[im])) / 128
              for row in trace['captures'] if row[b] == args.frame_bits]
    received = [complex(float(parts[1]), float(parts[2]))
                for line in (root / 'upstream-symbols.txt').read_text().splitlines()
                if (parts := line.split())]
    if len(native) < 2256 or len(received) < 256:
        raise SystemExit('not enough symbols to align')
    error, offset = min((sum(abs(a-z)**2 for a, z in
                                  zip(native[k:k+256], received[:256])) / 256, k)
                        for k in range(2000))
    # An unmatched start must fail instead of manufacturing a comparison.
    if error > 0.05:
        raise SystemExit(f'no qualified point alignment: MSE={error}')
    errors = [abs(a-z)**2 for a, z in zip(native[offset:], received)]
    windows = []
    for start in range(0, len(errors), 3200):
        values = errors[start:start+3200]
        windows.append({'start': start, 'symbols': len(values),
                        'mse': sum(values)/len(values),
                        'large_errors': sum(value > 1 for value in values)})
    result = {'native_start_index': offset, 'alignment_mse': error,
              'compared_symbols': len(errors), 'windows': windows}
    (root / 'symbol-comparison.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
