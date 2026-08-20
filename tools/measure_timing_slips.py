#!/usr/bin/env python3
"""Census the whole-sample timing slips in a recorded G.711 tap.

Receiver-independent.  A V.34/V.90 upstream signal with excess bandwidth
carries a spectral line at the symbol rate in the squared envelope of its
analytic signal, and the PHASE of that line is the transmitter's symbol
epoch measured against the 8 kHz bearer.  One inserted or dropped sample
moves it by exactly 360 x baud/8000 degrees -- 144 degrees at 3200 baud --
so a slip is a step in that phase and nothing else in the call looks like
one.

This is what says whether a collapse in the receiver is the wire or us.
Measured on artifacts/dmodem-soak-0821-rounds/round1 (tower d-modem,
PCMU, 3200 baud): the phase sits at -119 degrees, steps to -263 at
t=129.6 s, and steps back a second later.  Both are one sample.

  tools/measure_timing_slips.py <tap.g711> [--alaw] [--baud 3200]
                               [--from S] [--to S] [--window 0.2]
"""
import argparse
import sys

import numpy as np

FS = 8000.0


def ulaw2lin(b):
    b = ~b & 0xFF
    sign = b & 0x80
    exp = (b >> 4) & 7
    man = b & 0xF
    mag = ((((man.astype(np.int32) << 1) + 33) << exp) - 33)
    return np.where(sign != 0, -mag, mag).astype(float) * 4


def alaw2lin(b):
    b = (b ^ 0x55).astype(np.int32)
    sign = b & 0x80
    exp = (b >> 4) & 7
    man = b & 0xF
    mag = np.where(exp == 0, (man << 4) + 8, ((man << 4) + 264) << (exp - 1))
    return np.where(sign != 0, -mag, mag).astype(float) * 4


def analytic(x):
    n = len(x)
    X = np.fft.fft(x)
    h = np.zeros(n)
    h[0] = 1
    h[1:(n + 1) // 2] = 2
    if n % 2 == 0:
        h[n // 2] = 1
    return np.fft.ifft(X * h)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tap")
    ap.add_argument("--alaw", action="store_true")
    ap.add_argument("--baud", type=float, default=3200.0)
    ap.add_argument("--from", dest="t0", type=float, default=0.0)
    ap.add_argument("--to", dest="t1", type=float, default=None)
    ap.add_argument("--window", type=float, default=0.2,
                    help="seconds per phase measurement")
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    raw = np.frombuffer(open(args.tap, "rb").read(), dtype=np.uint8)
    x = alaw2lin(raw) if args.alaw else ulaw2lin(raw)
    t1 = args.t1 if args.t1 is not None else len(x) / FS
    seg = x[int(args.t0 * FS):int(t1 * FS)]
    if len(seg) < int(2 * args.window * FS):
        sys.exit("not enough audio in that range")

    p = np.abs(analytic(seg)) ** 2
    n = np.arange(len(p))
    tone = np.exp(-2j * np.pi * args.baud * n / FS)
    w = int(args.window * FS)
    step = int(args.step * FS)

    starts = np.arange(0, len(p) - w, step)
    z = np.array([np.sum(p[i:i + w] * tone[i:i + w]) for i in starts])
    mag = np.abs(z)
    ph = np.degrees(np.unwrap(np.angle(z)))
    good = mag > 0.5 * np.median(mag)

    # One sample of slip is this many degrees of the symbol-rate line.
    per_sample = 360.0 * args.baud / FS

    print("%s: %.1f s of %s at %g baud; one sample = %.1f degrees"
          % (args.tap, len(seg) / FS, "A-law" if args.alaw else "u-law",
             args.baud, per_sample))
    print("timing-line magnitude: median %.3g" % np.median(mag))

    # Walk the phase, ignoring windows that straddle a step (their magnitude
    # collapses), and report every crossing of half a sample.
    slips = []
    ref = None
    for k in range(len(starts)):
        if not good[k]:
            continue
        if ref is None:
            ref = ph[k]
            continue
        d = ph[k] - ref
        if abs(d) >= 0.5 * per_sample:
            slips.append((args.t0 + starts[k] / FS, d / per_sample))
            ref = ph[k]
        else:
            ref = 0.9 * ref + 0.1 * ph[k]
        if args.verbose:
            print("  t=%8.3f |line|=%9.3g phase=%+9.1f" % (
                args.t0 + starts[k] / FS, mag[k], ph[k]))

    if not slips:
        print("no whole-sample slips found")
        return
    print("%d slips:" % len(slips))
    prev = None
    for t, s in slips:
        gap = "" if prev is None else "  (+%.2f s)" % (t - prev)
        print("  t=%8.3f  %+.2f sample%s" % (t, s, gap))
        prev = t
    ts = [t for t, _ in slips]
    net = sum(s for _, s in slips)
    print("net %+.2f samples over %.1f s (%.1f ppm), mean gap %.2f s"
          % (net, ts[-1] - ts[0],
             1e6 * net / ((ts[-1] - ts[0]) * FS) if ts[-1] > ts[0] else 0.0,
             (ts[-1] - ts[0]) / max(1, len(ts) - 1)))


if __name__ == "__main__":
    main()
