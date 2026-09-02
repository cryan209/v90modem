#!/usr/bin/env python3
"""Measure the HSF coupler's receive channel response from a recorded call.

The far end (the digital modem) transmits V.34's line probing signal during
Phase 2: 21 cosines of EQUAL amplitude at 150 Hz to 3750 Hz (V.34 11.2.3, and
spandsp-master/src/make_v34_probe_signals.c has the table).  Equal amplitudes
are what make it a channel sounder -- the received magnitude of each tone is
the path's response at that frequency, times one constant.

The path measured is everything between the two modems: the far end's G.711
transmit, the SIP leg, the ATA, the two-wire line, and the HSF codec's own
receive filter.  That is exactly what v90_analogue_fse.c has to equalise, and
its whole design turns on how much is left above 3 kHz -- a channel that stops
below 4 kHz cannot be equalised to an ISI-free response at an 8 kHz symbol
rate, and cannot carry a symbol-rate timing tone either.

Usage: hsf_probe_response.py <hsf-rx.raw> [--rate 16000] [--top N]

The input is the raw 16-bit stream the coupler reads from the device.
"""
import argparse
import math
import struct
import sys

TONES = [150, 300, 450, 600, 750, 1050, 1350, 1500, 1650, 1950, 2100,
         2250, 2550, 2700, 2850, 3000, 3150, 3300, 3450, 3600, 3750]


# The transmitted phase of each tone, degrees (V.34 11.2.3; the table is in
# spandsp-master/src/make_v34_probe_signals.c).  Without these the received
# phases say nothing -- half the probe's tones start inverted.
TX_PHASE = {150: 0, 300: 180, 450: 0, 600: 0, 750: 0, 1050: 0, 1350: 0,
            1500: 0, 1650: 180, 1950: 0, 2100: 0, 2250: 180, 2550: 0,
            2700: 180, 2850: 0, 3000: 180, 3150: 180, 3300: 180, 3450: 180,
            3600: 0, 3750: 0}


def goertzel_c(samples, start, n, freq, rate):
    """Complex response at one frequency over samples[start:start+n]."""
    k = 2.0*math.cos(2.0*math.pi*freq/rate)
    s1 = s2 = 0.0
    for i in range(start, start + n):
        s0 = samples[i] + k*s1 - s2
        s2, s1 = s1, s0
    re = s1 - s2*math.cos(2.0*math.pi*freq/rate)
    im = s2*math.sin(2.0*math.pi*freq/rate)
    return (re*2.0/n, im*2.0/n)


def goertzel(samples, start, n, freq, rate):
    """Magnitude of one frequency over samples[start:start+n]."""
    k = 2.0*math.cos(2.0*math.pi*freq/rate)
    s1 = s2 = 0.0
    for i in range(start, start + n):
        s0 = samples[i] + k*s1 - s2
        s2, s1 = s1, s0
    re = s1 - s2*math.cos(2.0*math.pi*freq/rate)
    im = s2*math.sin(2.0*math.pi*freq/rate)
    return math.sqrt(re*re + im*im)*2.0/n


def read_pcm(path):
    with open(path, 'rb') as f:
        data = f.read()
    n = len(data)//2
    return list(struct.unpack('<%dh' % n, data[:2*n]))


def find_probe(samples, rate, win, step):
    """The window whose energy sits most completely on the probe's comb.

    The probe is the only thing on this line that is 21 tones on a 150 Hz grid,
    so the comb fraction separates it from speech, tones and data without
    needing the call's log or its clock -- which matters, because deriving the
    instant from the log needs the engine's clock origin (the ANSam detection)
    and that is one indirection too many to trust.

    Two Goertzels prescreen it.  Nothing else on this line has substantial
    energy at BOTH 150 Hz and 3750 Hz, so the product of those two picks the
    handful of windows worth scoring properly, and the full 21-tone score then
    runs on those alone.
    """
    total_len = len(samples)
    screen = []
    for start in range(0, total_len - win, step):
        lo = goertzel(samples, start, win, 150.0, rate)
        hi = goertzel(samples, start, win, 3750.0, rate)
        screen.append((lo*hi, start))
    screen.sort(reverse=True)

    out = []
    for _, start in screen[:64]:
        energy = 0.0
        for i in range(start, start + win):
            energy += float(samples[i])*samples[i]
        mean_sq = energy/win
        if mean_sq < 100.0:                  # silence carries no probe
            continue
        comb = 0.0
        for f in TONES:
            m = goertzel(samples, start, win, f, rate)
            comb += 0.5*m*m                  # a cosine of amplitude m
        out.append((comb/mean_sq, start))
    out.sort(reverse=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('path')
    ap.add_argument('--rate', type=int, default=16000)
    ap.add_argument('--window-ms', type=float, default=100.0)
    ap.add_argument('--search-from', type=float, default=0.0)
    ap.add_argument('--search-secs', type=float, default=0.0)
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--at', type=float, default=-1.0)
    ap.add_argument('--emit-c', action='store_true')
    args = ap.parse_args()

    samples = read_pcm(args.path)
    rate = args.rate
    win = int(rate*args.window_ms/1000.0)
    win -= win % int(rate/50)                 # whole 20 ms probe periods
    if win <= 0:
        print('window too short', file=sys.stderr)
        return 1

    lo = int(args.search_from*rate)
    hi = len(samples) if args.search_secs <= 0 else lo + int(args.search_secs*rate)
    hi = min(hi, len(samples))
    region = samples[lo:hi]
    if len(region) < win*2:
        print('not enough audio', file=sys.stderr)
        return 1

    if args.at >= 0.0:
        cands = [(0.0, int(args.at*rate) - lo)]
    else:
        cands = find_probe(region, rate, win, int(rate*0.02))
    if not cands:
        print('no probe found', file=sys.stderr)
        return 1
    if args.list:
        for sc, st in cands[:10]:
            print('  candidate %.3f s  comb %.3f' % ((lo + st)/float(rate), sc))
    score, start = cands[0]
    print('%s: probe at %.3f s (comb score %.3f), %d ms window'
          % (args.path, (lo + start)/float(rate), score, int(1000*win/rate)))

    resp = [(f,) + goertzel_c(region, start, win, f, rate) for f in TONES]
    mags = [(f, math.hypot(re, im)) for f, re, im in resp]
    peak = max(m for _, m in mags)
    if peak <= 0.0:
        print('silent', file=sys.stderr)
        return 1

    # A noise reference from frequencies the probe deliberately leaves empty
    # (900, 1200, 1800, 2400, 3900): anything at those is the line, not the
    # far end, so a tone within a few dB of them is not a measurement.
    floor = [goertzel(region, start, win, f, rate)
             for f in (900, 1200, 1800, 2400, 3900)]
    floor_db = 20.0*math.log10(max(sum(floor)/len(floor), 1e-9)/peak)
    print('  empty-bin floor %.1f dB' % floor_db)
    # Phase, with the transmitted phase removed and the bulk delay fitted out.
    # The window start is arbitrary, so an overall linear-in-frequency term is
    # not a property of the channel; what is left after removing it is the
    # group-delay distortion, which is the half of the response an equaliser
    # has to undo and the half a magnitude plot cannot show.
    raw = []
    for f, re, im in resp:
        ang = math.atan2(im, re) - math.pi*TX_PHASE[f]/180.0
        raw.append((f, ang))
    unwrapped = []
    prev = 0.0
    for i, (f, ang) in enumerate(raw):
        if i:
            while ang - prev > math.pi:
                ang -= 2.0*math.pi
            while ang - prev < -math.pi:
                ang += 2.0*math.pi
        unwrapped.append((f, ang))
        prev = ang
    n = len(unwrapped)
    sx = sum(f for f, _ in unwrapped)
    sy = sum(a for _, a in unwrapped)
    sxx = sum(f*f for f, _ in unwrapped)
    sxy = sum(f*a for f, a in unwrapped)
    den = n*sxx - sx*sx
    slope = (n*sxy - sx*sy)/den if den else 0.0
    inter = (sy - slope*sx)/n
    print('  bulk delay %.2f ms removed' % (-slope/(2.0*math.pi)*1000.0))

    print('  freq     dB    phase')
    phases = []
    for (f, m), (_, ang) in zip(mags, unwrapped):
        db = 20.0*math.log10(max(m, 1e-9)/peak)
        res = ang - (slope*f + inter)
        phases.append((f, db, res))
        bar = '#'*max(0, int(40 + db))
        print('  %4d  %6.1f  %+6.1f°  %s%s'
              % (f, db, res*180.0/math.pi, bar,
                 '' if db > floor_db + 6.0 else '   (at the floor)'))

    if args.emit_c:
        print()
        print('/* Measured on %s, probe at %.3f s. */'
              % (args.path, (lo + start)/float(rate)))
        print('static const struct { int freq; double db; double phase; }')
        print('    hsf_measured_response[] = {')
        for f, db, res in phases:
            print('        {%4d, %7.2f, %+8.4f},' % (f, db, res))
        print('    };')
    return 0


if __name__ == '__main__':
    sys.exit(main())
