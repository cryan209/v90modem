#!/usr/bin/env python3
"""Measure a V.34 PP-preceded burst's S/S-bar and following symbols against its PP.

V.34 10.1.3.6 defines PP on fixed axes, so a fixed T/2 equalizer fitted to
PP alone (held-out validated) puts every neighbouring symbol of the same
burst on the axes the Recommendation draws its constellations on.  That makes
the absolute orientation of S, S-bar (10.1.3.7), TRN (10.1.3.8) and B1
(10.1.3.1) measurable -- which an offline data decoder cannot see, because
differential decoding and the 90-degree-invariant trellis make it blind to a
rotation, while a receiver that takes its phase from S, or times PP from the
S-to-S-bar reversal, is not.

Prints, for each window: the PP held-out error, the S/S-bar symbol count and
angles (spec: 128T at 45/135 starting 45, then 16T at 225/315 starting 225),
and the first symbols after PP.  With --compare, slices 16-point TRN after
two Phase 3 PPs and reports the rotation relating them.

Usage:
  v34_hdx_training_phase.py TAP.g711 --window START END [--window ...] [--baud 3429]
  v34_hdx_training_phase.py --compare A.g711 A_START B.g711 B_START
numpy required.  PCMU taps only (A-law: pass --alaw).
"""
import argparse
import numpy as np



def linear(path, alaw):
    b = np.frombuffer(open(path, 'rb').read(), np.uint8).astype(int)
    if alaw:
        a = b ^ 0x55
        seg = (a >> 4) & 7
        mag = ((a & 15) << 4) + 8
        mag = np.where(seg > 0, (mag + 256) << np.maximum(seg - 1, 0), mag)
        return np.where(a & 128, mag, -mag).astype(float)
    u = (~b) & 255
    x = (((u & 15)*8 + 132) << ((u >> 4) & 7)) - 132
    return np.where(u & 128, -x, x).astype(float)


def pp_sequence():
    return np.array([np.exp(1j*np.pi*((i//4)*(i % 4) + (4 if (i//4) % 3 == 1 else 0))/6)
                     for i in range(288)])


def equalize(x, baud, carrier, t0, t1, pre, post):
    """Fit a 33-tap T/2 FIR to the PP found in [t0, t1]; return its time, held-out
    MSE and the equalized symbols from `pre` before PP to `post` after it."""
    n = np.arange(len(x))
    bb = x*np.exp(-2j*np.pi*carrier*n/8000)
    t = np.arange(161) - 80
    h = np.sinc(2*1900/8000*t)*np.hamming(161)
    bb = np.convolve(bb, h/h.sum(), 'same')
    first = int(t0*2*baud) - 2*pre - 100
    last = int(t1*2*baud) + 2*post + 600
    pos = np.arange(first, last)*8000/(2*baud)
    base = np.floor(pos).astype(int)
    y = np.zeros(len(pos), complex)
    for k in range(-12, 13):
        y += bb[np.clip(base + k, 0, len(bb) - 1)]*np.sinc(pos - base - k)*np.hamming(25)[k + 12]
    pp = pp_sequence()
    lo = 2*pre + 100
    corr = np.correlate(y[lo:lo + int((t1 - t0)*2*baud) + 576], pp.repeat(2), 'valid')
    peak = lo + int(np.argmax(abs(corr)))
    best = None
    for off in range(peak - 6, peak + 7):
        X = y[off + 2*np.arange(288)[:, None] + np.arange(-16, 17)[None, :]]
        coef = np.linalg.lstsq(X[24:216], pp[24:216], rcond=1e-5)[0]
        err = float(np.mean(abs(X[216:272] @ coef - pp[216:272])**2))
        if best is None or err < best[0]:
            best = (err, off, coef)
    err, off, coef = best
    r = y[off + 2*np.arange(-pre, 288 + post)[:, None] + np.arange(-16, 17)[None, :]] @ coef
    return (first + off)/(2*baud), err, r


def report(args):
    x = linear(args.tap, args.alaw)
    for t0, t1 in args.window:
        when, err, r = equalize(x, args.baud, args.carrier, t0, t1, 200, 16)
        pre = r[:200]
        k = 199
        while k >= 0 and abs(pre[k]) > 0.5:
            k -= 1
        s = np.round(np.degrees(np.angle(pre[k + 1:]))).astype(int)
        names = {45: 'S 45/135', 135: 'S 45/135', 225: 'S-bar 225/315', 315: 'S-bar 225/315',
                 0: 'off-axis 0/90', 90: 'off-axis 0/90', 180: 'off-axis 180/270', 270: 'off-axis 180/270'}
        pair = [names[a % 360] for a in 45*np.round(s/45).astype(int)]
        runs = []
        for p in pair:
            if runs and runs[-1][0] == p:
                runs[-1][1] += 1
            else:
                runs.append([p, 1])
        print(f'PP at {when:.4f} s  held-out MSE {err:.5f}')
        print(f'  {len(s)} symbols before PP: ' + ', '.join(f'{n} x {p}' for p, n in runs[:6])
              + ('' if len(runs) <= 6 else ', ...') + '  (spec: 128 x S 45/135, 16 x S-bar 225/315)')
        print(f'  first 4 {[int(v) for v in s[:4]]}  last 4 {[int(v) for v in s[-4:]]}')
        post = np.round(np.degrees(np.angle(r[200 + 288:]))).astype(int)
        print(f'  first 8 after PP {[int(v) for v in post[:8]]}')


def compare(args):
    pa, ta, pb, tb = args.compare
    out = []
    for path, t in ((pa, float(ta)), (pb, float(tb))):
        _, _, r = equalize(linear(path, args.alaw), args.baud, args.carrier, t, t + 0.7, 0, 8000)
        z = r[288:]
        a = np.sqrt(np.mean(abs(z)**2)/10)
        z = z/a
        out.append(np.clip(2*np.round((z.real - 1)/2) + 1, -3, 3)
                   + 1j*np.clip(2*np.round((z.imag - 1)/2) + 1, -3, 3))
    for rot in range(4):
        print(f'rotation {rot*90:3d}: {np.mean(out[0] == out[1]*(1j)**rot):.3f} of 16-point TRN symbols agree')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('tap', nargs='?')
    ap.add_argument('--window', nargs=2, type=float, action='append', default=[])
    ap.add_argument('--compare', nargs=4)
    ap.add_argument('--baud', type=int, default=3429)
    ap.add_argument('--carrier', type=float)
    ap.add_argument('--alaw', action='store_true')
    args = ap.parse_args()
    args.baud = 24000/7 if args.baud == 3429 else args.baud
    if args.carrier is None:
        if round(args.baud) != 3429:
            ap.error('--carrier is required except at 3429 baud (V.34 5.1: fc = S*d/e)')
        args.carrier = args.baud*4/7
    if args.compare:
        compare(args)
    elif args.tap and args.window:
        report(args)
    else:
        ap.error('give TAP with --window, or --compare')


if __name__ == '__main__':
    main()
