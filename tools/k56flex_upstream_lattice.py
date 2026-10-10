#!/usr/bin/env python3
"""Measure MICA's K56flex upstream trellis input against the transmitted points.

Reads a MicaEmu `mica_trace --capture-pc 0x47b9` capture (AR7, INDX, 8DB1 and
DM 0D00..0D3F; tools/probe_k56flex_connection.py --inputs) and the client's
`*-tx.caller` points (int16 I/Q, odd multiples of 128, so the V.34 lattice
spacing is 256). Reports, in lattice spacings: error RMS by point amplitude,
error left after a widely-linear ISI fit, its radial/tangential split, and the
phase error over time with its linear slope (a carrier frequency offset).
Diagnostic only: it supplies nothing to the receiver.

    .venv/bin/python tools/k56flex_upstream_lattice.py CAPTURE.txt TX.caller \
        --from 18.74 --to 21.55
"""
import argparse
import struct
import numpy as np


def reverse16(v):
    return int(f'{v:016b}'[::-1], 2)


def load(path, t0, t1):
    rows = open(path).read().splitlines()
    addresses = [int(x, 16) for x in rows[0].split()[7:]]
    out = []
    for row in rows[1:]:
        f = row.split()
        if not t0 < float(f[0]) < t1:
            continue
        w = dict(zip(addresses, [int(x, 16) for x in f[7:]]))
        p, i = w[0x17], w[0x18]
        q = reverse16((reverse16(p) + reverse16(i)) & 0xffff)  # 47BB/47BE
        s = lambda x: x - 65536 if x >= 32768 else x
        out.append(complex(s(w[p]), s(w[q])))
    return np.array(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('capture')
    ap.add_argument('tx')
    ap.add_argument('--from', dest='t0', type=float, default=18.74)
    ap.add_argument('--to', dest='t1', type=float, default=19.06)
    ap.add_argument('--baud', type=float, default=3200)
    a = ap.parse_args()
    rx = load(a.capture, a.t0, a.t1)
    ref = np.array([complex(x, y) for x, y in struct.iter_unpack('<hh', open(a.tx, 'rb').read())])
    n = len(rx)
    best = None
    for s in range(min(600, len(ref) - n)):
        r = ref[s:s + n]
        c = abs(np.vdot(r, rx)) / np.sqrt(np.vdot(r, r).real * np.vdot(rx, rx).real)
        if best is None or c > best[0]:
            best = (c, s, np.vdot(r, rx) / np.vdot(r, r))
    c, s, g = best
    r = ref[s:s + n]
    e = (rx / g - r) / 256
    print(f'symbols {n}, shift {s}, correlation {c:.5f}, gain {abs(g):.3f} at {np.degrees(np.angle(g)):.2f} deg')
    print(f'error RMS {np.sqrt(np.mean(abs(e) ** 2)):.3f} lattice spacings')
    amp = abs(r) / 128
    for lo, hi in [(0, 3), (3, 8), (8, 16), (16, 24), (24, 64)]:
        m = (amp >= lo) & (amp < hi)
        if m.any():
            radial = np.mean(((rx[m] / g) * np.conj(r[m])).real / abs(r[m]) ** 2)
            print(f'  |point| {lo:2d}-{hi:2d}: n {m.sum():5d}, error {np.sqrt(np.mean(abs(e[m]) ** 2)):.3f}, radial gain {radial:.4f}')
    big_e = rx / g - r
    L = 8
    idx = np.arange(L, n - L)
    A = np.array([v for j in range(-L, L + 1) for v in (r[idx + j], np.conj(r[idx + j]))]).T
    coef = np.linalg.lstsq(A, big_e[idx], rcond=None)[0]
    res = big_e[idx] - A @ coef
    u = r[idx] / abs(r[idx])
    print(f'after ISI fit {np.sqrt(np.mean(abs(res) ** 2)) / 256:.3f}; radial {np.sqrt(np.mean(((res * np.conj(u)).real) ** 2)) / 256:.3f}, '
          f'tangential {np.sqrt(np.mean(((res * np.conj(u)).imag) ** 2)) / 256:.3f} spacings')
    k = np.arange(n)
    big = amp >= 9
    ph = np.unwrap(np.angle((rx / g) / r)[big])
    slope = np.polyfit(k[big], ph, 1)[0]
    jitter = np.degrees(ph - np.convolve(ph, np.ones(64) / 64, 'same'))[64:-64]
    print(f'phase slope {slope * a.baud / (2 * np.pi):.4f} Hz; jitter about the 64-symbol mean {jitter.std():.2f} deg')
    for w in range(0, n - 255, max(256, (n // 8) // 256 * 256)):
        m = big & (k >= w) & (k < w + 256)
        print(f'  symbols {w:5d}+256: mean phase error {np.degrees(np.angle((rx / g) / r)[m]).mean():+.2f} deg')


if __name__ == '__main__':
    main()
