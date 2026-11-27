#!/usr/bin/env python3
"""Frequency-shift a raw G.711 capture by a fixed offset, offline.

Builds the analytic signal with a windowed Hilbert FIR, multiplies by
e^{j 2 pi f t} and re-encodes the real part.  This is for making test inputs
(V.32bis 2.1's +-7 Hz received frequency offset, for example); it must never
be used in the live audio path (CLAUDE.md: never transcode G.711 live).

usage: g711_freq_shift.py in.g711 out.g711 HZ [ulaw|alaw]
"""
import math
import sys

RATE = 8000
TAPS = 255  # odd, type III Hilbert; 1800 +/- 1200 Hz band is far from 0/4000


def ulaw_dec(b):
    b = ~b & 0xFF
    t = ((b & 0x0F) << 3) + 0x84
    t <<= (b & 0x70) >> 4
    return (0x84 - t) if (b & 0x80) else (t - 0x84)


def ulaw_enc(x):
    x = max(-32635, min(32635, int(round(x))))
    sign = 0x80 if x < 0 else 0
    if x < 0:
        x = -x
    x += 0x84
    exp = 7
    mask = 0x4000
    while exp > 0 and not (x & mask):
        exp -= 1
        mask >>= 1
    mant = (x >> (exp + 3)) & 0x0F
    return ~(sign | (exp << 4) | mant) & 0xFF


def alaw_dec(b):
    b ^= 0x55
    t = (b & 0x0F) << 4
    seg = (b & 0x70) >> 4
    if seg == 0:
        t += 8
    elif seg == 1:
        t += 0x108
    else:
        t += 0x108
        t <<= seg - 1
    return t if (b & 0x80) else -t


def alaw_enc(x):
    x = max(-32767, min(32767, int(round(x))))
    if x >= 0:
        mask = 0xD5
    else:
        mask = 0x55
        x = -x - 1
    seg_end = [0xFF, 0x1FF, 0x3FF, 0x7FF, 0xFFF, 0x1FFF, 0x3FFF, 0x7FFF]
    seg = 0
    while seg < 8 and x > seg_end[seg]:
        seg += 1
    if seg >= 8:
        return 0x7F ^ mask
    aval = seg << 4
    aval |= (x >> 4) & 0x0F if seg < 2 else (x >> (seg + 3)) & 0x0F
    return aval ^ mask


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    src, dst, hz = sys.argv[1], sys.argv[2], float(sys.argv[3])
    law = sys.argv[4] if len(sys.argv) > 4 else "ulaw"
    dec, enc = (ulaw_dec, ulaw_enc) if law == "ulaw" else (alaw_dec, alaw_enc)
    raw = open(src, "rb").read()
    table = [dec(b) for b in range(256)]
    x = [table[b] for b in raw]
    n = len(x)
    half = TAPS // 2
    # Hilbert FIR: h[k] = 2/(pi k) for odd k, Blackman window; odd taps only.
    hk = []
    for k in range(1, half + 1, 2):
        w = 0.42 + 0.5 * math.cos(math.pi * k / (half + 1)) \
            + 0.08 * math.cos(2 * math.pi * k / (half + 1))
        hk.append((k, 2.0 / (math.pi * k) * w))
    pad = [0.0] * half
    xp = pad + x + pad
    out = bytearray(n)
    w = 2 * math.pi * hz / RATE
    for i in range(n):
        c = i + half
        q = 0.0
        for k, h in hk:
            q += h * (xp[c - k] - xp[c + k])
        re = x[i]
        ph = w * i
        out[i] = enc(re * math.cos(ph) - q * math.sin(ph))
    open(dst, "wb").write(bytes(out))


if __name__ == "__main__":
    main()
