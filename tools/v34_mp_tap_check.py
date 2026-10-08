#!/usr/bin/env python3
"""Independently decode caller 4-point V.34 MP from a G.711 tap.

Use a window containing MP; times refer exclusively to this file's origin.
Requires numpy. Does not grade 16-point MP or the far-end analogue waveform.
"""
import argparse
import json
import numpy as np
from v90_phase4_upstream_grade import ulaw_lin, demod


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tap')
    parser.add_argument('start', type=float)
    parser.add_argument('end', type=float)
    parser.add_argument('--law', choices=('ulaw', 'alaw'), default='ulaw')
    parser.add_argument('--baud', type=float, default=3200)
    parser.add_argument('--carrier', type=float, default=3200*4/7)
    args = parser.parse_args()
    raw = np.fromfile(args.tap, dtype=np.uint8)
    if args.law == 'ulaw':
        audio = ulaw_lin(raw)
    else:
        a = raw.astype(np.int32) ^ 0x55
        segment = (a >> 4) & 7
        magnitude = ((a & 15) << 4) + np.where(segment == 0, 8, 264)
        magnitude <<= np.maximum(segment-1, 0)
        audio = np.where(a & 128, magnitude, -magnitude).astype(float)
    symbols = demod(audio, args.start, args.end, args.baud, args.carrier, skip=500)
    angle = np.angle(symbols[1:]*np.conj(symbols[:-1]))
    dibits = (-np.rint(angle/(np.pi/2))).astype(int) % 4
    result = dict(tap=args.tap, window=[args.start, args.end], hypotheses=[])
    for rotation in range(4):
        for order in range(2):
            q = (dibits+rotation) % 4
            bits = np.empty(2*len(q), dtype=np.uint8)
            bits[order::2] = q & 1
            bits[1-order::2] = (q >> 1) & 1
            # V.34 7.1: calling-modem GPC, 1+x^-18+x^-23.
            data = bits[23:] ^ bits[5:-18] ^ bits[:-23]
            frames = []
            for offset in range(len(data)-188):
                frame = data[offset:offset+188]
                if not np.all(frame[:17] == 1) or frame[17]:
                    continue
                end = 170 if frame[18] else 68
                if any(frame[j] for j in range(17, end+1, 17)):
                    continue
                # V.34 10.1.2.3.2/Table 13: start/sync/fill excluded.
                crc = 0xffff
                for block in range(17, end, 17):
                    for bit in frame[block+1:block+17]:
                        crc = (crc >> 1) ^ (0x8408 if (crc ^ int(bit)) & 1 else 0)
                wire = sum(int(frame[end+1+j]) << j for j in range(16))
                if crc == wire:
                    frames.append(dict(bit_offset=offset, type=int(frame[18]),
                                       acknowledged=int(frame[33])))
            if frames:
                result['hypotheses'].append(dict(rotation=rotation, order=order,
                    crc_valid=len(frames), acknowledged=sum(f['acknowledged'] for f in frames),
                    frames=frames))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
