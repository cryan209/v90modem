#!/usr/bin/env python3
"""List every V.34 Phase 4 MP / MP' frame on a G.711 tap, with its time.

Answers "did the far end ever receive our MP' / E?" from the wire rather than
from either modem's log: run it on our TX tap (what we sent) and on the RX tap
(what the peer sent) over the same window and line the two up.

  v34_mp_timeline.py <tap.g711> <start_s> <end_s> <baud> <carrier_hz> <taps>
                     [ulaw|alaw]

taps is the transmitter's scrambler: "18,23" for the V.34 call modem (GPC),
"5,23" for the answer modem (GPA).  Demodulation is the independent T/2 CMA
front end in v90_phase4_upstream_grade (shares nothing with the receiver
under test); the 4-point differential decode (10.1.3.3, recovered dibit =
negation of the transmitted one), bit order and CRC (10.1.2.3.2, MSB-first
field) are taken from whichever interpretation validates the most frames.

Also reports runs of >= 20 consecutive descrambled ones that do not start an
MP frame -- candidate 20-bit E sequences (10.1.3.2) -- and where the data
after them stops being ones.
"""
import sys
import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from v90_phase4_upstream_grade import FS, ulaw_lin, demod   # noqa: E402


def alaw_lin(b):
    b = b.astype(np.int32) ^ 0x55
    s = b & 0x80
    e = (b >> 4) & 7
    m = b & 15
    v = np.where(e == 0, (m << 4) + 8, ((m << 4) + 0x108) << (e - 1))
    return np.where(s, v, -v).astype(float)


def crc16(bits):
    reg = 0xffff
    for b in bits:
        fb = ((reg >> 15) & 1) ^ (b & 1)
        reg = (reg << 1) & 0xffff
        if fb:
            reg ^= (1 << 12) | (1 << 5) | 1
    return reg


def frames(bits):
    out = []
    n = len(bits)
    i = 0
    while i < n - 88:
        if all(bits[i:i + 17]) and bits[i + 17] == 0:
            t = bits[i + 18]
            total = 188 if t else 88
            if i + total <= n:
                if t == 0:
                    body = bits[i + 18:i + 34] + bits[i + 35:i + 51] + bits[i + 52:i + 68]
                    crc = bits[i + 69:i + 85]
                else:
                    body = sum((bits[i + a:i + a + 16] for a in
                                (18, 35, 52, 69, 86, 103, 120, 137, 154)), [])
                    crc = bits[i + 171:i + 187]
                got = 0
                for k, b in enumerate(crc):
                    got |= (b & 1) << (15 - k)
                out.append((i, t, crc16(body) == got, bits[i + 33]))
                if crc16(body) == got:
                    i += total
                    continue
        i += 1
    return out


def main():
    tap, t0, t1, baud, fc, taps = sys.argv[1:7]
    law = sys.argv[7] if len(sys.argv) > 7 else 'ulaw'
    t0, t1, baud, fc = float(t0), float(t1), float(baud), float(fc)
    a, b = (int(x) for x in taps.split(','))
    raw = np.fromfile(tap, np.uint8)
    x = (alaw_lin(raw) if law == 'alaw' else ulaw_lin(raw)).astype(float)
    s = demod(x, t0, t1, baud, fc, skip=0)
    ang = np.angle(s[1:] * np.conj(s[:-1]))
    q = np.rint(ang / (np.pi / 2)).astype(int) % 4
    origin = t0 + 11 / baud                 # 41-tap T/2 FSE centre, as elsewhere
    best = None
    for neg in (True, False):
        d = (-q) % 4 if neg else q
        for order in (0, 1):
            bits = []
            for v in d:
                i1, i2 = int(v) & 1, (int(v) >> 1) & 1
                bits += [i1, i2] if order == 0 else [i2, i1]
            ds = bits[:]
            for n in range(len(bits)):
                ds[n] = bits[n] ^ (bits[n - a] if n >= a else 0) ^ (bits[n - b] if n >= b else 0)
            fr = frames(ds)
            good = sum(1 for f in fr if f[2])
            if best is None or good > best[0]:
                best = (good, neg, order, ds, fr)
    good, neg, order, ds, fr = best
    print(f'{tap}: {len(q)} symbols {t0:.3f}-{t1:.3f} s; best interpretation '
          f'{"negated" if neg else "direct"} dibit, order {"I1,I2" if order == 0 else "I2,I1"}: '
          f'{good} CRC-valid MP frames')
    covered = set()
    for i, t, ok, ack in fr:
        if not ok:
            continue
        ts = origin + i / 2 / baud
        print(f'  {ts:8.3f} s  MP{"1" if t else "0"}{chr(39) if ack else " "}  (ack={ack})')
        covered.update(range(i, i + (188 if t else 88)))
    # candidate E: >= 20 ones not inside a valid MP frame
    run = 0
    for n, v in enumerate(ds):
        if v and n not in covered:
            run += 1
        else:
            if run >= 20:
                st = n - run
                print(f'  {origin + st / 2 / baud:8.3f} s  run of {run} ones (E/B1 candidate) '
                      f'ending {origin + n / 2 / baud:.3f} s')
            run = 0


if __name__ == '__main__':
    main()
