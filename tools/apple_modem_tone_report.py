#!/usr/bin/env python3
"""Report on a tools/apple_modem_pair_test.sh output directory.

Everything here is measured on the STEADY part of each capture, because the
two processes are started independently and the tone arrives 0.5-0.9 s in;
a whole-file figure is that startup skew, not the path.  The per-tone level
is a coherent (Goertzel) amplitude, so it is the amplitude of a sine and sits
3 dB above that tone's RMS -- do not compare it with a broadband RMS without
allowing for that.

    usage: tools/apple_modem_tone_report.py <out-dir> [rate]

Pure python: no numpy, because the corpus is small and this has to run on a
machine where the system python has none.
"""
import sys, os, struct, math, cmath


def load(path):
    with open(path, 'rb') as f:
        d = f.read()
    x = [v / 32768.0 for v in struct.unpack('<%dh' % (len(d) // 2), d[:len(d) // 2 * 2])]
    m = sum(x) / len(x) if x else 0.0
    return [v - m for v in x]


def db(v):
    return 20 * math.log10(max(v, 1e-12))


def amp(x, f, rate):
    """Coherent amplitude of the line at f, over the samples given."""
    if not x:
        return 0.0
    s = sum(v * cmath.exp(-2j * math.pi * f * n / rate) for n, v in enumerate(x))
    return 2 * abs(s) / len(x)


def rms(x):
    return math.sqrt(sum(v * v for v in x) / len(x)) if x else 0.0


def steady(x, rate):
    """1.0 s to 2.0 s into the capture.  The two processes start independently
    so the tone arrives 0.5-0.9 s in, and it stops before the capture ends --
    both ends of the file are transitions, and only the middle is the path."""
    a, b = int(1.0 * rate), int(2.0 * rate)
    return x[a:b] if len(x) > b else x


def report(d, rate):
    print("baseline (both silent) -- the bridge sends no comfort noise, so this")
    print("is the codec's own floor:")
    for tag in ('A', 'B'):
        p = os.path.join(d, 'base_%s.s16' % tag)
        if os.path.exists(p):
            x = load(p)
            print("  %s: RMS %+6.1f dBFS   peak %+6.1f dBFS" %
                  (tag, db(rms(x)), db(max(abs(v) for v in x))))

    print("\nfrequency response, one way at a time (tx amplitude 0.15 = -16.5 dBFS):")
    print("   %5s %12s %12s" % ("Hz", "A->B (dBFS)", "B->A (dBFS)"))
    for f in (300, 1000, 2000, 3000):
        row = []
        for src, name in (('rxB_%d.s16', 'A->B'), ('rxA_%d.s16', 'B->A')):
            p = os.path.join(d, src % f)
            row.append(db(amp(steady(load(p), rate), f, rate)) if os.path.exists(p) else None)
        print("   %5d %12s %12s" % (f,
              "%.1f" % row[0] if row[0] is not None else "-",
              "%.1f" % row[1] if row[1] is not None else "-"))

    print("\nlevel linearity and distortion, A -> B at 1000 Hz:")
    print("   %7s %9s %9s %8s %8s %8s" % ("tx amp", "tx dBFS", "rx dBFS", "loss dB", "2f", "THD"))
    for a in ('0.02', '0.05', '0.15', '0.35'):
        p = os.path.join(d, 'lin_rx_%s.s16' % a)
        if not os.path.exists(p):
            continue
        s = steady(load(p), rate)
        h = [amp(s, 1000.0 * k, rate) for k in (1, 2, 3, 4, 5)]
        thd = math.sqrt(sum(v * v for v in h[1:])) / h[0] if h[0] > 0 else 0.0
        tx = db(float(a))
        print("   %7s %9.1f %9.1f %8.1f %8.1f %7.2f%%" %
              (a, tx, db(h[0]), tx - db(h[0]), db(h[1]), 100 * thd))

    print("\ndouble talk (A sends 1000 Hz, B sends 1400 Hz, at the same time).")
    print("Each modem's own tone in its own receive is its 2-wire hybrid's echo:")
    for tag, own, far in (('A', 1000.0, 1400.0), ('B', 1400.0, 1000.0)):
        p = os.path.join(d, 'dt_%s.s16' % tag)
        if not os.path.exists(p):
            continue
        s = steady(load(p), rate)
        o, fa = db(amp(s, own, rate)), db(amp(s, far, rate))
        ims = {'f2-f1 400': 400.0, 'f1+f2 2400': 2400.0, '2f1-f2 600': 600.0}
        im = "  ".join("%s %.1f" % (k, db(amp(s, v, rate))) for k, v in ims.items())
        print("  %s: far tone %.1f, own echo %.1f (%.1f dB down)" % (tag, fa, o, fa - o))
        print("     intermodulation: %s" % im)


def main():
    d = sys.argv[1]
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 9600.0
    report(d, rate)


main()
