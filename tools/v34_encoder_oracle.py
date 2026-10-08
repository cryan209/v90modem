#!/usr/bin/env python3
"""Independent V.34 raw-symbol oracle for 3200/3429 baud, expanded, 16-state.

Implements V.34 7, 9.1, 9.3–9.6 and B1 10.1.3.1 using integer arithmetic.
No production constellation, shell or convolutional tables are imported.
Input is DS_TX_BIT_DUMP or V34_PRIMARY_TX_BIT_DUMP (ASCII bits);
compare against V34_DATA_TX_DUMP (Q9.7).
Requires a continuous bit capture from the first payload mapping frame.
This checks raw x(n); nonlinear projection/modulation are separate stages.
"""
import argparse
from bisect import bisect_right
import json
from pathlib import Path
import struct


def convolution(a, b):
    out = [0]*(len(a)+len(b)-1)
    for i, x in enumerate(a):
        for j, y in enumerate(b):
            out[i+j] += x*y
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bits', type=Path)
    parser.add_argument('symbols', type=Path)
    parser.add_argument('--coefficients', nargs=6, type=int, required=True)
    parser.add_argument('--bps', type=int, choices=(4800, 9600, 12000, 14400, 21600, 31200), default=12000)
    parser.add_argument('--baud', type=int, choices=(3200, 3429), default=3200)
    parser.add_argument('--scrambler', choices=('gpa', 'gpc'), default='gpa')
    parser.add_argument('--idle-frames', type=int, default=0,
                        help='unrecorded idle mapping frames before V.42 bit dump starts')
    parser.add_argument('--frames', type=int, default=1000)
    parser.add_argument('--includes-b1', action='store_true',
                        help='input is V34_PRIMARY_TX_BIT_DUMP, including B1')
    args = parser.parse_args()
    if args.idle_frames < 0 or args.frames < 16:
        parser.error('idle-frames must be nonnegative and frames at least 16')
    payload = args.bits.read_text().strip()
    if set(payload)-{'0', '1'}:
        parser.error('bit dump must contain only ASCII 0/1')
    if args.baud == 3429:
        if args.bps not in (4800, 9600, 14400):
            parser.error('3429 oracle covers 4800, 9600 and 14400 bit/s')
        # Tables 7, 8, 10: J=8 P=15, expanded shaping, no Q bits.
        bbits, base_kbits, rings, high_count = {
            4800: (12, 0, 1, 3), 9600: (23, 11, 3, 6),
            14400: (34, 22, 8, 9)}[args.bps]
        qbits, pframes, jframes = 0, 15, 8
    else:
        bbits = args.bps//400
        base_kbits, rings, qbits = {4800: (0, 1, 0), 9600: (12, 4, 0),
                                  14400: (24, 10, 0), 12000: (18, 6, 0), 21600: (26, 12, 2),
                                  31200: (26, 12, 5)}[args.bps]
        pframes, jframes, high_count = 16, 7, 16
    def frame_bits(frame):
        # 8.2: reset counter at each data frame, increment before each frame.
        i = frame % pframes
        high = ((i+1)*high_count)//pframes > (i*high_count)//pframes
        return bbits if high else bbits-1
    prefix_bits = sum(frame_bits(i) for i in range(pframes+args.idle_frames))
    source = payload if args.includes_b1 else '1'*prefix_bits + payload
    if args.includes_b1 and (args.idle_frames or source[:prefix_bits] != '1'*prefix_bits):
        parser.error('includes-b1 requires the complete all-ones B1 and no idle-frames')
    raw = args.symbols.read_bytes()
    observed = list(struct.iter_unpack('<hh', raw[:len(raw)//4*4]))
    if len(observed) < pframes*8:
        parser.error('symbol capture must contain at least the full B1')
    # 9.4 shell enumerator; independently constructed, not production tables.
    g2 = [rings-abs(p-rings+1) for p in range(2*rings-1)]
    g4 = convolution(g2, g2)
    g8 = convolution(g4, g4)
    z8 = [0]
    for count in g8:
        z8.append(z8[-1]+count)
    # 9.1/Figure 5: quarter points have both coordinates 1 modulo 4;
    # increasing energy, greatest imaginary coordinate breaks ties.
    quarter = sorted(((x, y) for x in range(-43, 46, 4)
                      for y in range(-43, 46, 4)),
                     key=lambda xy: (xy[0]**2+xy[1]**2, -xy[1]))[:416]
    labels = [[0, 7, 4, 3], [5, 2, 1, 6], [4, 3, 0, 7], [1, 6, 5, 2]]
    table13 = [[0,0,1,1,8,8,9,9], [3,2,2,3,11,10,10,11],
               [5,5,4,4,13,13,12,12], [6,7,7,6,14,15,15,14],
               [8,8,9,9,0,0,1,1], [11,10,10,11,3,2,2,3],
               [13,13,12,12,5,5,4,4], [14,15,15,14,6,7,7,6]]
    coeff = list(zip(args.coefficients[::2], args.coefficients[1::2]))
    history = [(0, 0)]*3
    reg = rotation = state = symbol_count = 0
    inversion = '0111011111111010' if jframes == 8 else '01110111111110'
    source_pos = 0
    inversion_index = 2*(jframes-1)

    def round_towards_zero_tie(value, denominator):
        magnitude, rem = divmod(abs(value), denominator)
        if 2*rem > denominator:
            magnitude += 1
        return magnitude if value >= 0 else -magnitude

    def prediction():
        re = sum(x*h-y*k for (x,y), (h,k) in zip(history, coeff))
        im = sum(x*k+y*h for (x,y), (h,k) in zip(history, coeff))
        p = (round_towards_zero_tie(re, 16384), round_towards_zero_tie(im, 16384))
        quantum = 4 if bbits >= 56 else 2
        c = tuple(quantum*round_towards_zero_tie(v, quantum*128) for v in p)
        return p, c

    def split_shell(total, left, right):
        for a in range(len(left)):
            b = total-a
            count = left[a]*right[b] if 0 <= b < len(right) else 0
            if rank[0] < count:
                return a, rank[0]
            rank[0] -= count
        raise ValueError('shell rank outside support')

    p, c = prediction()
    for frame in range(min(len(observed)//8, args.frames)):
        nbits = frame_bits(frame)
        kbits = max(0, base_kbits-(bbits-nbits))
        if source_pos+nbits > len(source):
            break
        scrambled = []
        for bit in source[source_pos:source_pos+nbits]:
            out = (int(bit) ^ (reg >> (4 if args.scrambler == 'gpa' else 17)) ^ (reg >> 22)) & 1
            reg = ((reg << 1) | out) & ((1 << 23)-1)
            scrambled.append(out)
        if bbits <= 12:
            # 9.3.2: groups with only two input bits have implicit I3=0.
            grouped, cursor = [], 0
            for pair in range(4):
                width = 3 if pair < nbits-8 else 2
                grouped.extend(scrambled[cursor:cursor+width])
                cursor += width
                if width == 2:
                    grouped.append(0)
            scrambled = grouped
        r0 = sum(bit << j for j, bit in enumerate(scrambled[:kbits]))
        a = bisect_right(z8, r0)-1
        rank = [r0-z8[a]]
        b, r1 = split_shell(a, g4, g4)
        rank = [r1 % g4[b]]
        cc, r4 = split_shell(b, g2, g2)
        rank = [r1 // g4[b]]
        d, r5 = split_shell(a-b, g2, g2)
        pairs = []
        for total, index in ((cc, r4 % g2[cc]), (b-cc, r4 // g2[cc]),
                             (d, r5 % g2[d]), (a-b-d, r5 // g2[d])):
            pairs.extend((index, total-index) if total < rings
                         else (total-(rings-1-index), rings-1-index))
        for pair in range(4):
            pos = kbits+pair*(3+2*qbits)
            i1, i2, i3 = scrambled[pos:pos+3]
            rotation = (rotation+i2+2*i3) % 4
            subset = []
            # 9.6.3: first 4D interval of each HALF data frame. With P=15
            # the second boundary lands in the third pair of frame 7.
            v0 = 0
            if (4*(frame % pframes)+pair) % (2*pframes) == 0:
                v0 = int(inversion[inversion_index % (2*jframes)])
                inversion_index += 1
            u0 = 0
            for k in range(2):
                q = sum(bit << j for j, bit in enumerate(
                    scrambled[pos+3+k*qbits:pos+3+(k+1)*qbits]))
                x, y = quarter[(pairs[2*pair+k] << qbits)+q]
                turns = rotation if k == 0 else (rotation+2*i1+u0) % 4
                for _ in range(turns):
                    x, y = y, -x
                yr, yi = x+c[0], y+c[1]
                expected = (yr*128-p[0], yi*128-p[1])
                got = observed[symbol_count]
                if expected != got:
                    print(json.dumps(dict(ok=False, frame=frame, symbol=2*pair+k,
                        symbol_index=symbol_count, expected=expected, observed=got), indent=2))
                    raise SystemExit(1)
                symbol_count += 1
                subset.append(labels[((yi+3) % 8)//2][((yr+3) % 8)//2])
                old_c = c
                history = [expected]+history[:2]
                p, c = prediction()
                if k == 0:
                    c0 = ((sum(old_c)//2) ^ (sum(c)//2)) & 1
                    u0 = (state & 1) ^ c0 ^ v0
                else:
                    inp = table13[subset[0]][subset[1]]
                    t1, t2, t3, t4 = [(state >> j) & 1 for j in range(4)]
                    state = ((t1 << 3) | ((t4 ^ t1 ^ ((inp >> 1)&1)) << 2)
                             | ((t3 ^ ((inp >> 1)&1)) << 1) | (t2 ^ (inp & 1)))
        source_pos += nbits
        if frame == pframes-1:
            inversion_index = 0
    print(json.dumps(dict(ok=True, baud=args.baud, bps=args.bps,
                         scrambler=args.scrambler, raw_symbols_checked=symbol_count,
                         payload_bits_available=len(payload)), indent=2))


if __name__ == '__main__':
    main()
