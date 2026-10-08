#!/usr/bin/env python3
"""Check complete HDLC frames in a DS_TX_BIT_DUMP ASCII bit stream.

Ignores the unframed prefix and unfinished suffix; validates destuffing,
octet alignment and reflected CRC-16 residue 0xf0b8 independently.
"""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bits', type=Path)
    args = parser.parse_args()
    bits = args.bits.read_text().strip()
    if set(bits)-{'0', '1'}:
        parser.error('expected ASCII bits')
    valid = bad = aborts = 0
    for frame in bits.split('01111110')[1:-1]:
        if not frame:
            continue
        data = []
        ones = 0
        aborted = False
        for char in frame:
            bit = int(char)
            if bit:
                ones += 1
                data.append(1)
            else:
                if ones != 5:
                    data.append(0)
                ones = 0
            if ones >= 6:
                aborted = True
                break
        if aborted:
            aborts += 1
            continue
        if len(data) < 24 or len(data) % 8:
            bad += 1
            continue
        crc = 0xffff
        for index in range(0, len(data), 8):
            crc ^= sum(bit << j for j, bit in enumerate(data[index:index+8]))
            for _ in range(8):
                crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
        if crc == 0xf0b8:
            valid += 1
        else:
            bad += 1
    print(json.dumps(dict(valid_complete_frames=valid, bad_complete_frames=bad,
                         aborted_segments=aborts), indent=2))
    if bad:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
