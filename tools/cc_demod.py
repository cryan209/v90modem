#!/usr/bin/env python3
"""Demodulate a V.34 control channel (10.2.4) out of a recorded G.711 tap.

600 baud DPSK, carrier 1200 Hz from the call modem and 2400 Hz from the answer
modem, differential encoder and the clause 7 scrambler both enabled.  This is
deliberately OUTSIDE the modem: when a receiver locks a clean preamble and then
rejects every frame -- or when a peer will not accept a frame we believe we
sent -- the way to tell whose fault it is is to decode the bits somewhere that
shares no code with either end.  Same reasoning as the V.90 9.6 CP work.

  tools/cc_demod.py <tap.g711> --carrier 2400 [--from S] [--to S] [--law ulaw]
"""
import argparse, cmath, math, sys

def ulaw2lin(u):
    u = ~u & 0xFF
    t = ((u & 0x0F) << 3) + 0x84
    t <<= (u & 0x70) >> 4
    return (t - 0x84) if (u & 0x80) else -(t - 0x84)

def alaw2lin(a):
    a ^= 0x55
    t = (a & 0x0F) << 4
    seg = (a & 0x70) >> 4
    if seg == 0:   t += 8
    elif seg == 1: t += 0x108
    else:          t = (t + 0x108) << (seg - 1)
    return -t if (a & 0x80) else t

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tap")
    ap.add_argument("--carrier", type=float, default=2400.0)
    ap.add_argument("--baud", type=float, default=600.0)
    ap.add_argument("--law", default="ulaw")
    ap.add_argument("--from", dest="t0", type=float, default=0.0)
    ap.add_argument("--to", dest="t1", type=float, default=1e9)
    ap.add_argument("--fs", type=float, default=8000.0)
    ap.add_argument("--binary", action="store_true",
                    help="10.1.2.3.1 binary DPSK at 600 bit/s, as the INFO "
                         "sequences use: 1 bit per symbol, 180 degrees for a "
                         "1 and 0 degrees for a 0, unscrambled.  ALT, E and "
                         "MPh instead use 10.2.4's quaternary control channel "
                         "at 1200 bit/s, which is the default here.")
    a = ap.parse_args()

    tbl = [ulaw2lin(i) for i in range(256)] if a.law == "ulaw" else [alaw2lin(i) for i in range(256)]
    raw = open(a.tap, "rb").read()
    i0, i1 = int(a.t0*a.fs), min(len(raw), int(a.t1*a.fs))
    x = [tbl[b] for b in raw[i0:i1]]
    if not x:
        sys.exit("empty window")

    # Mix to baseband and low-pass with a running mean over one symbol.
    sps = a.fs/a.baud
    bb = []
    for n, v in enumerate(x):
        bb.append(v*cmath.exp(-2j*math.pi*a.carrier*(n + i0)/a.fs))
    win = int(round(sps))
    acc = 0j
    lp = []
    for n, v in enumerate(bb):
        acc += v
        if n >= win:
            acc -= bb[n-win]
        lp.append(acc/win)

    # Try every symbol phase; the right one maximises the mean |z| at the
    # sampling instants, because the signal is constant modulus.
    best = None
    for off in range(int(sps)):
        pts = [lp[int(round(off + k*sps))] for k in range(int((len(lp)-off)/sps))]
        if len(pts) < 8: continue
        m = sum(abs(p) for p in pts)/len(pts)
        if best is None or m > best[0]:
            best = (m, off, pts)
    if best is None:
        sys.exit("window too short")
    mean_mag, off, pts = best
    print("carrier %.0f Hz  baud %.0f  window %.3f-%.3f s  %d symbols  phase %d  mean |z| %.0f"
          % (a.carrier, a.baud, a.t0, min(a.t1, len(raw)/a.fs), len(pts), off, mean_mag))

    if a.binary:
        # 10.1.2.3.1: "The transmit point is rotated 180 degrees from the
        # previous point if the transmit bit is a 1, and ... 0 degrees ... if
        # the transmit bit is a 0."  One bit per symbol, and the INFO
        # sequences carry no scrambler -- the frame is sent as it stands.
        bits = []
        for k in range(1, len(pts)):
            if abs(pts[k-1]) < 1e-9:
                bits.append(0)
                continue
            d = pts[k]/pts[k-1]
            bits.append(1 if abs(cmath.phase(d)) > math.pi/2 else 0)
        s = "".join(str(b) for b in bits)
        sync = "01110010"
        hits = [j for j in range(len(s)-8) if s[j:j+8] == sync]
        print("  binary DPSK, unscrambled  ones %5.1f%%  sync at %s"
              % (100.0*sum(bits)/max(1,len(bits)), hits[:12] if hits else "(none)"))
        for j in hits[:4]:
            print("     @%5d (t=%.4fs): %s" % (j, a.t0 + (j + off)/a.baud, s[j:j+56]))
        return

    # Differential decode: 10.2.4 rotates the point by Zn*90 where Zn is the
    # running modulo-4 sum of the dibit.  The dibit is therefore the phase
    # STEP between consecutive symbols.
    bits = []
    for k in range(1, len(pts)):
        if abs(pts[k-1]) < 1e-9: bits += [0, 0]; continue
        d = pts[k]/pts[k-1]
        q = int(round(cmath.phase(d)/(math.pi/2))) & 3
        bits += [q & 1, (q >> 1) & 1]

    # Clause 7 scrambler, self-synchronising: descramble with both taps.
    for tap in (5, 18):
        reg = 0
        out = []
        for b in bits:
            o = b ^ ((reg >> (tap-1)) & 1) ^ ((reg >> 22) & 1)
            out.append(o)
            reg = ((reg << 1) | b) & 0x7FFFFF
        s = "".join(str(b) for b in out)
        # INFOh / INFO0 framing: 4 fill ones then the 8-bit sync 01110010.
        sync = "01110010"
        hits = [j for j in range(len(s)-8) if s[j:j+8] == sync]
        print("  scrambler tap %-2d  ones %5.1f%%  sync '01110010' at %s"
              % (tap, 100.0*sum(out)/len(out), hits[:12] if hits else "(none)"))
        for j in hits[:3]:
            print("     @%5d: %s" % (j, s[j:j+56]))

if __name__ == "__main__":
    main()
