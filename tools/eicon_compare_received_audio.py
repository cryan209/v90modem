#!/usr/bin/env python3
"""Compare Eicon Audio1 paired PCMU samples with a local tap at a verified offset.

Offset is card sample index minus local sample index; origins are independent.
Zero polarity differences are counted separately from decoded waveform errors.
"""
import argparse
import json
import re
from pathlib import Path


def linear(code):
    value = (~code) & 255
    magnitude = (((value & 15)*8 + 132) << ((value >> 4) & 7)) - 132
    return -magnitude if value & 128 else magnitude


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('card_trace', type=Path)
    parser.add_argument('local_tx', type=Path)
    parser.add_argument('--offset', required=True, type=int)
    parser.add_argument('--card-channel', choices=('rx', 'tx'), default='rx')
    parser.add_argument('--start', type=float, default=18)
    args = parser.parse_args()
    card = bytes(int(word[:2] if args.card_channel == 'rx' else word[2:], 16)
                 for line in args.card_trace.read_text().splitlines()
                 if '[*,1] SAMPLE[]' in line
                 for word in re.findall(r'\b[0-9A-F]{4}\b', line.split('SAMPLE[]')[1]))
    local = args.local_tx.read_bytes()
    start = max(round(args.start*8000), -args.offset, 0)
    end = min(len(local), len(card)-args.offset)
    if start >= end:
        parser.error('no overlapping comparison interval')
    byte_errors = wave_errors = prefix_byte_errors = 0
    first_wave_error = None
    for index in range(start, end):
        a, b = local[index], card[index+args.offset]
        if a != b:
            byte_errors += 1
            if linear(a) != linear(b):
                wave_errors += 1
                if first_wave_error is None:
                    first_wave_error = index
            elif first_wave_error is None:
                prefix_byte_errors += 1
    exact_end = first_wave_error if first_wave_error is not None else end
    print(json.dumps(dict(card_trace=str(args.card_trace), local_tx=str(args.local_tx),
        card_channel=args.card_channel, offset_samples=args.offset, compared_local_seconds=[start/8000, end/8000],
        byte_mismatches=byte_errors, waveform_mismatches=wave_errors,
        exact_waveform_prefix_local_seconds=[start/8000, exact_end/8000],
        exact_waveform_prefix_samples=exact_end-start,
        zero_polarity_changes_in_prefix=prefix_byte_errors), indent=2))


if __name__ == '__main__':
    main()
