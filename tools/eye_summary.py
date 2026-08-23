#!/usr/bin/env python3
"""One line of eye measurements per call, for a rate/time matrix.

Every number here is the receiver's own distance from the V.34 lattice, which
is what "the eye" means for this modem: 0.667 is the value for symbols bearing
no relation to the lattice, so it is the ceiling rather than an error bar.

Reports, per call:

  settled   the baseline the receiver reaches once B1 hands it a converged
            filter -- its operating point, and the scale everything else in
            the receiver is judged against
  CINR      mean symbol power over mean square distance to the lattice, in dB
  clean%    windows at or under 0.30
  hold      the LONGEST UNBROKEN stretch of clean windows, in seconds.  This
            is the "time" axis of the matrix: a rate that is clean for eight
            seconds and a rate that is clean for two minutes have the same
            clean% on calls of different lengths, and only this separates them
  t_fail    when the first collapse happened, or "-" if the call never lost it
  lines     intact peer payload lines delivered to our PTY

  tools/eye_summary.py <artifact-dir> [more dirs...] [--tsv]
"""
import math
import os
import re
import struct
import sys

WIN_CLEAN = 0.30


def parse_log(path):
    """Pull the per-window symbol error and its timestamp out of a server log."""
    rows = []
    settled = None
    rate = None
    baud = None
    pat = re.compile(r"t=([0-9.]+)s.*?sym err ([0-9.]+)")
    st = re.compile(r"upstream settled at ([0-9.]+)")
    rt = re.compile(r"startup complete \(upstream V\.34 (\d+) bps")
    bd = re.compile(r"upstream selection: (\d+) baud")
    try:
        f = open(path, "rb")
    except OSError:
        return rows, settled, rate, baud
    with f:
        for raw in f:
            line = raw.decode("utf-8", "replace")
            m = pat.search(line)
            if m:
                rows.append((float(m.group(1)), float(m.group(2))))
                continue
            m = st.search(line)
            if m:
                settled = float(m.group(1))
                continue
            m = rt.search(line)
            if m:
                rate = int(m.group(1))
                continue
            m = bd.search(line)
            if m:
                baud = int(m.group(1))
    if not rows:
        # Plain V.34: the data-mode metric is "distance to grid", logged once
        # per 4096 symbols with no timestamp, so time comes from the window
        # size and the negotiated symbol rate.
        g = re.compile(r"distance to grid ([0-9.]+) per symbol over (\d+) symbols")
        gb = re.compile(r"V\.34 training started \([a-z]+, (\d+) baud, up to (\d+) bps")
        n = 0
        try:
            f2 = open(path, "rb")
        except OSError:
            return rows, settled, rate, baud
        with f2:
            for raw in f2:
                line = raw.decode("utf-8", "replace")
                m = gb.search(line)
                if m:
                    baud = int(m.group(1))
                    rate = int(m.group(2))
                    continue
                m = g.search(line)
                if m:
                    n += int(m.group(2))
                    rows.append((n/float(baud or 3000), float(m.group(1))))
    return rows, settled, rate, baud


def lattice(path, limit=400000):
    """CINR from the received data symbols, if the dump is there."""
    try:
        raw = open(path, "rb").read()
    except OSError:
        return None, None
    n = min(len(raw)//4, limit)
    if n < 1000:
        return None, None
    err = pwr = 0.0
    for i in range(n):
        re_, im_ = struct.unpack_from("<hh", raw, i*4)
        x, y = re_/128.0, im_/128.0
        dx = x - (2.0*math.floor((x - 1)/2.0) + 1.0)
        dy = y - (2.0*math.floor((y - 1)/2.0) + 1.0)
        if dx > 1.0:
            dx -= 2.0
        if dy > 1.0:
            dy -= 2.0
        err += dx*dx + dy*dy
        pwr += x*x + y*y
    err /= n
    pwr /= n
    return err, (10.0*math.log10(pwr/err) if err > 0 else None)


def count_lines(path, prefix=b"U"):
    try:
        raw = open(path, "rb").read()
    except OSError:
        return 0
    # The serial captures are CRLF, so an unanchored-right "$" misses every
    # line: the \r sits between the digits and the newline.
    pat = re.compile(b"^" + prefix + b"[0-9]{7}\r?$", re.M)
    return len(pat.findall(raw))


def summarise(d):
    rows, settled, rate, baud = parse_log(os.path.join(d, "server.log"))
    err, cinr = lattice(os.path.join(d, "frames.answer"))
    lines = 0
    for sub in sorted(os.listdir(d)) if os.path.isdir(d) else []:
        p = os.path.join(d, sub, "rx_pty.bin")
        if os.path.exists(p):
            lines += count_lines(p)
    # A V.90 soak counts the peer's U-lines arriving on our PTY; a plain V.34
    # rate call is the other direction, and its payload lands on the peer's
    # DTE.  Count whichever this recording holds.
    if not lines:
        lines += count_lines(os.path.join(d, "pty_rx.bin"))
    if not lines:
        lines += count_lines(os.path.join(d, "call.out"), b"S")
    clean = sum(1 for _, e in rows if e <= WIN_CLEAN)
    total = len(rows)
    # Longest unbroken clean stretch, and when the first collapse happened.
    best = cur_start = 0.0
    t_fail = None
    run = False
    for t, e in rows:
        if e <= WIN_CLEAN:
            if not run:
                cur_start = t
                run = True
            best = max(best, t - cur_start)
        else:
            if run and t_fail is None:
                t_fail = t
            run = False
    return dict(dir=os.path.basename(os.path.normpath(d)), rate=rate, baud=baud,
                settled=settled, err=err, cinr=cinr,
                clean=(100.0*clean/total if total else None), windows=total,
                hold=best, t_fail=t_fail, lines=lines)


def main():
    # Skip anything that is not a call directory: the matrix harness leaves a
    # <dir>.out beside each one, and a shell glob picks both up.
    args = [a for a in sys.argv[1:]
            if not a.startswith("-") and os.path.isdir(a)]
    tsv = "--tsv" in sys.argv
    if not args:
        sys.exit(__doc__)
    hdr = ("dir", "rate", "baud", "settled", "CINR", "clean%", "hold_s",
           "t_fail_s", "lines")
    out = []
    for d in args:
        r = summarise(d)
        out.append((r["dir"],
                    "-" if r["rate"] is None else str(r["rate"]),
                    "-" if r["baud"] is None else str(r["baud"]),
                    "-" if r["settled"] is None else "%.3f" % r["settled"],
                    "-" if r["cinr"] is None else "%.1f" % r["cinr"],
                    "-" if r["clean"] is None else "%.0f" % r["clean"],
                    "%.1f" % r["hold"],
                    "-" if r["t_fail"] is None else "%.1f" % r["t_fail"],
                    str(r["lines"])))
    if tsv:
        print("\t".join(hdr))
        for row in out:
            print("\t".join(row))
        return
    w = [max(len(hdr[i]), max((len(r[i]) for r in out), default=0))
         for i in range(len(hdr))]
    print("  ".join(h.ljust(w[i]) for i, h in enumerate(hdr)))
    print("  ".join("-"*w[i] for i in range(len(hdr))))
    for row in out:
        print("  ".join(row[i].ljust(w[i]) for i in range(len(hdr))))


if __name__ == "__main__":
    main()
