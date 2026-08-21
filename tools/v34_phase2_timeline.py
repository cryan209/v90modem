#!/usr/bin/env python3
"""Segment a captured G.711 stream into V.34 Phase 2 events.

Reads a mu-law tap (one octet per 8 kHz sample) and reports, in call time:
energy, which of the two CC carriers is present (1200 Hz = Tone B / call
modem, 2400 Hz = Tone A / answer modem), the phase of that carrier, and the
instants where it reverses.  That is enough to say what a peer actually put on
the line, independently of either modem's own state machine.
"""
import sys, math, cmath

def ulaw_decode(b):
    b = ~b & 0xff
    sign = b & 0x80
    exp = (b >> 4) & 7
    man = b & 0x0f
    mag = ((man << 1) + 33) << exp
    mag -= 33
    return -mag if sign else mag

TBL = [ulaw_decode(i) for i in range(256)]

def load(path):
    with open(path, 'rb') as f:
        return [TBL[b] for b in f.read()]

def goertzel_phase(x, f, fs=8000.0):
    w = 2*math.pi*f/fs
    acc = 0j
    for n, v in enumerate(x):
        acc += v*cmath.exp(-1j*w*n)
    return acc/len(x)

def main():
    path = sys.argv[1]
    win = int(sys.argv[2]) if len(sys.argv) > 2 else 40      # 5 ms
    x = load(path)
    prev = None
    print("# t_ms  rms   |1200|  |2400|  carrier  phase_deg  event")
    for i in range(0, len(x)-win, win):
        seg = x[i:i+win]
        rms = math.sqrt(sum(v*v for v in seg)/win)
        a12 = goertzel_phase(seg, 1200.0)
        a24 = goertzel_phase(seg, 2400.0)
        m12, m24 = abs(a12), abs(a24)
        if rms < 200:
            car, ph = 'silence', 0.0
            prev = None
        elif m12 >= m24:
            car, ph = '1200', math.degrees(cmath.phase(a12))
        else:
            car, ph = '2400', math.degrees(cmath.phase(a24))
        ev = ''
        if car in ('1200', '2400') and prev is not None and prev[0] == car:
            d = abs((ph - prev[1] + 180) % 360 - 180)
            if d > 120:
                ev = 'REVERSAL'
        if car in ('1200', '2400'):
            prev = (car, ph)
        print("%7.1f %6.0f %7.0f %7.0f  %-7s %8.1f  %s"
              % (i/8.0, rms, m12, m24, car, ph, ev))

main()
