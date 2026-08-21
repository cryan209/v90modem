#!/usr/bin/env python3
"""Check a captured Phase 3 transmission against V.34 10.1.3.6/10.1.3.7 itself.

The peer's equalizer is trained by PP, so "is our PP right on the wire" is the
question that matters, and neither modem's log can answer it -- our own
transmitter reports what it meant to send.  This demodulates the G.711 tap and
correlates the PP interval against the sequence 10.1.3.6 defines.

  v34_phase3_verify.py <tap.g711> <search-start-ms> <search-end-ms> [baud] [carrier]
"""
import sys, math, cmath

def ulaw(b):
    b = ~b & 0xff
    s = b & 0x80; e = (b >> 4) & 7; m = b & 0x0f
    mag = (((m << 1) + 33) << e) - 33
    return -mag if s else mag

TBL = [ulaw(i) for i in range(256)]

def pp_ideal():
    out = []
    for i in range(288):
        k, I = (i // 4) % 12, i % 4
        kx = 4.0 if k % 3 == 1 else 0.0
        out.append(cmath.exp(1j*math.pi*(k*I + kx)/6.0))
    return out

def main():
    path = sys.argv[1]
    t0 = float(sys.argv[2]); t1 = float(sys.argv[3])
    baud = float(sys.argv[4]) if len(sys.argv) > 4 else 2400.0
    fc = float(sys.argv[5]) if len(sys.argv) > 5 else 1600.0
    fs = 8000.0
    x = [TBL[b] for b in open(path, 'rb').read()]
    w = 2*math.pi*fc/fs
    bb = [x[n]*cmath.exp(-1j*w*n) for n in range(len(x))]
    sps = fs/baud
    # crude matched filter: integrate over one symbol
    half = int(sps/2)
    ideal = pp_ideal()
    best = (0.0, None)
    n0, n1 = int(t0*fs/1000.0), int(t1*fs/1000.0)
    for start in range(n0, n1):
        acc = 0j; e = 0.0
        ok = True
        for m in range(288):
            c = start + m*sps
            i = int(round(c))
            if i-half < 0 or i+half+1 > len(bb):
                ok = False; break
            seg = bb[i-half:i+half+1]
            v = sum(seg)/len(seg)
            acc += v*ideal[m].conjugate()
            e += abs(v)**2
        if not ok:
            break
        if e > 0:
            score = abs(acc)/math.sqrt(e*288)
            if score > best[0]:
                best = (score, start)
    score, start = best
    print("best PP correlation %.4f at sample %d (%.1f ms)"
          % (score, start if start else -1, (start/8.0) if start else -1))
    print("1.0 = the transmitted PP is exactly 10.1.3.6's sequence")

main()
