#!/usr/bin/env python3
"""Locate Courier 403 DSP-to-supervisor loss in an 832a PC capture.

Capture with X2_HOST_CAPTURE_PC=832a and --capture-receiver. At this ROM
instruction ACC contains the output-bank ready bit, and data 0320 holds
the next descrambled word. ACC=0 returns without publishing that word.
This examines firmware output, not a replacement decoder for the modem.
"""
import argparse
import json
from pathlib import Path


def async_octets(bits):
    out = bytearray()
    i = 32  # Allow acquisition/framing to settle at the capture boundary.
    while i + 10 <= len(bits):
        if bits[i] == 0 and bits[i + 9] == 1:
            out.append(sum(bits[i + 1 + k] << k for k in range(8)))
            i += 10
        else:
            i += 1
    return bytes(out)


def analyze(capture, source):
    if capture['pc'] != 0x832a:
        raise ValueError('requires a Courier 403 output-bank capture at PC 832a')
    columns = {address: 5 + i for i, address in enumerate(capture['addresses'])}
    streams = {'before_bank_check': [], 'published': []}
    dropped = words = 0
    for row in capture['captures']:
        # x2 index 14: B=36, MD=5 (Draft 0.33 sections 20 and 22).
        if row[columns[0x351]] != 36 or row[columns[0x364]] != 5:
            continue
        count = row[columns[0x322]]
        if count != 8:
            raise ValueError(f'expected raw 8-bit output mode, got {count}')
        bits = [(row[columns[0x320]] >> k) & 1 for k in range(count)]
        words += 1
        streams['before_bank_check'].extend(bits)
        if row[2]:
            streams['published'].extend(bits)
        else:
            dropped += 1
    decoded = {name: async_octets(bits) for name, bits in streams.items()}
    return {
        'index': 14, 'source_bytes': len(source), 'captured_words': words,
        'dropped_words': dropped,
        'source_exact_before_bank_check': bool(source) and source in decoded['before_bank_check'],
        'source_exact_published': bool(source) and source in decoded['published'],
        'decoded_bytes': {name: len(value) for name, value in decoded.items()},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('source', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(json.loads(args.capture.read_text()), args.source.read_bytes()),
                     indent=2))


if __name__ == '__main__':
    main()
