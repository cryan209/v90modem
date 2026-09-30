#!/usr/bin/env python3
"""Decode DTMF from a raw s16 capture made by apple_usb_modem_audio.

    usage: tools/apple_modem_dtmf_decode.py <capture.s16> [rate]

This is the one read-back the PBX gives that needs no speech understanding,
so it is what identifies a line objectively -- extension 9333 here answers
with the calling line's own number in DTMF.  It also reports each digit's
duration, level and twist, which is what to compare our own dialling against
when an exchange is rejecting it.

Both tones must dominate their own group AND carry most of the frame's
energy; without that last test a 400 Hz dial tone with noise on it decodes as
a stream of digits.  Pure python, no numpy.
"""
import sys, struct, math, cmath

LOW = [697, 770, 852, 941]
HIGH = [1209, 1336, 1477, 1633]
KEYS = [['1', '2', '3', 'A'],
        ['4', '5', '6', 'B'],
        ['7', '8', '9', 'C'],
        ['*', '0', '#', 'D']]


def goertzel(s, f, rate):
    return 2 * abs(sum(v * cmath.exp(-2j * math.pi * f * n / rate)
                       for n, v in enumerate(s))) / len(s)


def decode(path, rate):
    d = open(path, 'rb').read()
    x = [v / 32768.0 for v in struct.unpack('<%dh' % (len(d) // 2), d[:len(d) // 2 * 2])]
    m = sum(x) / len(x) if x else 0.0
    x = [v - m for v in x]
    W, H = int(0.020 * rate), int(0.010 * rate)
    frames = []
    for k in range((len(x) - W) // H):
        s = x[k * H:k * H + W]
        lo = [goertzel(s, f, rate) for f in LOW]
        hi = [goertzel(s, f, rate) for f in HIGH]
        i = max(range(4), key=lambda n: lo[n])
        j = max(range(4), key=lambda n: hi[n])
        tot = math.sqrt(sum(v * v for v in s) / len(s))
        ok = (lo[i] > 0.002 and hi[j] > 0.002
              and lo[i] > 2.5 * sorted(lo)[-2] and hi[j] > 2.5 * sorted(hi)[-2]
              and (lo[i] ** 2 + hi[j] ** 2) / 2 > 0.35 * tot * tot)
        frames.append((KEYS[i][j], lo[i], hi[j]) if ok else None)

    out, run, n, acc = [], None, 0, []
    for v in frames + [None]:
        k = v[0] if v else None
        if k == run:
            n += 1
            if v:
                acc.append(v)
        else:
            if run and n >= 3 and acc:
                l = sum(a[1] for a in acc) / len(acc)
                h = sum(a[2] for a in acc) / len(acc)
                out.append((run, n * 10, 20 * math.log10(l), 20 * math.log10(h)))
            run, n, acc = k, 1, [v] if v else []
    print("%s: digits = %s" % (path.split('/')[-1],
                               ''.join(d for d, _, _, _ in out) or '(none)'))
    for k, ms, l, h in out:
        print("   '%s' %5d ms   low %+6.1f dBFS   high %+6.1f dBFS   twist %+5.1f dB"
              % (k, ms, l, h, h - l))


decode(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 9600.0)
