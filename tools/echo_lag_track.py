#!/usr/bin/env python3
"""Track the bulk delay of our own echo through a call, from the DS0 taps.

    tools/echo_lag_track.py <dir with live-rx.g711 and live-tx.g711> [options]

For each window it cross-correlates the received tap against the transmit tap
over a range of lags and prints the lag of the largest correlation.  Both are
first whitened by an LPC predictor fitted to the window's received signal: the
far end's signal fills the band and the echo only stands out of it at the band
edges, so without whitening the peak is barely visible under a live far end.

Read it as: a lag that steps mid-call means the far end's gateway moved its
playout, and a line echo canceller fitted earlier is then cancelling at the
wrong delay (docs/v90_rasfinder_phase4.md, 2026-10-16: 1923 -> 2283 -> 2323 on
the RasFinder's 9600 call, the steps at the moment its upstream eye closed).
Lags are in TAP samples, which can differ from the canceller's own count by a
constant (80 samples on that rig).

Needs NumPy only.
"""
import argparse
import os
import sys

import numpy as np


def ulaw_decode(raw):
    u = ~np.frombuffer(raw, np.uint8).astype(np.int32) & 0xFF
    sign = u & 0x80
    exp = (u >> 4) & 7
    mant = u & 0x0F
    mag = (((mant << 3) + 0x84) << exp) - 0x84
    return np.where(sign, -mag, mag).astype(float)


def alaw_decode(raw):
    a = np.frombuffer(raw, np.uint8).astype(np.int32) ^ 0x55
    sign = a & 0x80
    exp = (a >> 4) & 7
    mant = a & 0x0F
    mag = np.where(exp == 0, (mant << 4) + 8, ((mant << 4) + 0x108) << np.maximum(exp - 1, 0))
    return np.where(sign, mag, -mag).astype(float)


def lpc(x, order):
    r = np.array([np.dot(x[:len(x) - k], x[k:]) for k in range(order + 1)])
    r[0] *= 1.0001
    if r[0] <= 0:
        return np.array([1.0])
    m = np.array([[r[abs(i - j)] for j in range(order)] for i in range(order)])
    a = np.linalg.solve(m, r[1:order + 1])
    return np.concatenate([[1.0], -a])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("dir")
    ap.add_argument("--law", choices=("ulaw", "alaw"), default="ulaw")
    ap.add_argument("--start", type=float, default=0.0, help="seconds")
    ap.add_argument("--end", type=float, default=None, help="seconds")
    ap.add_argument("--window", type=float, default=1.0, help="seconds per estimate")
    ap.add_argument("--min-lag", type=int, default=0)
    ap.add_argument("--max-lag", type=int, default=4000)
    ap.add_argument("--order", type=int, default=16, help="whitening predictor order")
    args = ap.parse_args()

    dec = ulaw_decode if args.law == "ulaw" else alaw_decode
    with open(os.path.join(args.dir, "live-rx.g711"), "rb") as f:
        rx = dec(f.read())
    with open(os.path.join(args.dir, "live-tx.g711"), "rb") as f:
        tx = dec(f.read())
    n = min(len(rx), len(tx))
    w = int(args.window * 8000)
    a0 = max(int(args.start * 8000), args.max_lag + args.order)
    a1 = n if args.end is None else min(n, int(args.end * 8000))
    lags = np.arange(args.min_lag, args.max_lag + 1)
    print("time_s  lag  peak/median  rx_rms")
    for a in range(a0, a1 - w + 1, w):
        y = rx[a:a + w]
        h = lpc(y, args.order)
        yw = np.convolve(y, h)[:w]
        seg = tx[a - args.max_lag - args.order:a + w - args.min_lag]
        sw = np.convolve(seg, h)[args.order:]
        # sw index j holds tx sample (a - max_lag + j); lag L uses j = max_lag - L + i.
        c = np.array([np.dot(yw, sw[args.max_lag - lag:args.max_lag - lag + w]) for lag in lags])
        k = int(np.argmax(np.abs(c)))
        med = np.median(np.abs(c))
        print("%7.2f %5d %8.1f %8.0f" % (a / 8000.0, lags[k], abs(c[k]) / med if med > 0 else 0.0,
                                         np.sqrt(np.mean(y * y))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
