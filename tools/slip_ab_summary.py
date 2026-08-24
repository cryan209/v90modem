#!/usr/bin/env python3
"""Score a v90_slip_ab.sh run: clean time, longest hold, payload, splices.

Clean TIME, not window counts -- a white stretch emits short windows and a
clean one long windows, so a window-weighted percentage flatters a call that
spent most of its seconds white.
"""
import os
import re
import sys

T = re.compile(r"DATA bits: t=([0-9.]+)s")
E = re.compile(r"sym err ([0-9.]+)")
ADJ = re.compile(r"cr_get_adjustment returned")
OPEN = 0.30


def score(log):
    """(clean_s, total_s, longest_s, splices, reached_data)"""
    prev = clean = total = run = best = 0.0
    splices = 0
    data = False
    with open(log, errors="replace") as fh:
        for line in fh:
            if "enter DATA after B1" in line:
                data = True
                prev = 0.0
            if data and ADJ.search(line):
                splices += 1
            m, me = T.search(line), E.search(line)
            if not (m and me):
                continue
            t, e = float(m.group(1)), float(me.group(1))
            d = max(0.0, t - prev)
            prev = t
            total += d
            if e < OPEN:
                clean += d
                run += d
                best = max(best, run)
            else:
                run = 0.0
    return clean, total, best, splices, data


def payload(d):
    n = 0
    for sub in sorted(os.listdir(d)):
        p = os.path.join(d, sub, "rx_pty.bin")
        if os.path.isfile(p):
            n += len(re.findall(rb"^U[0-9]{7}$",
                                open(p, "rb").read(), re.M))
    return n


def main():
    root = sys.argv[1]
    arms = {}
    rows = []
    for name in sorted(os.listdir(root)):
        d = os.path.join(root, name)
        log = os.path.join(d, "server.log")
        if not os.path.isfile(log):
            continue
        arm = name.split("-")[0]
        cl, tot, best, sp, data = score(log)
        pl = payload(d)
        rows.append((name, arm, cl, best, sp, pl, data))
        a = arms.setdefault(arm, [0.0, 0.0, 0, 0, 0, 0])
        a[0] += cl
        a[1] += best
        a[2] += sp
        a[3] += pl
        a[4] += 1
        a[5] += 1 if data else 0

    print("%-12s %-6s %9s %9s %8s %10s" %
          ("run", "arm", "clean_s", "hold_s", "splices", "U-lines"))
    print("-" * 60)
    for name, arm, cl, best, sp, pl, data in rows:
        print("%-12s %-6s %8.1fs %8.1fs %8d %10d%s" %
              (name, arm, cl, best, sp, pl, "" if data else "   (no data mode)"))

    print()
    print("%-8s %8s %9s %9s %8s %10s" %
          ("arm", "calls", "data", "clean_s", "hold_s", "U-lines"))
    print("-" * 60)
    for arm in ("fixed", "slip"):
        if arm not in arms:
            continue
        cl, best, sp, pl, n, dm = arms[arm]
        print("%-8s %8d %9d %8.1fs %8.1fs %10d   (%d splices)" %
              (arm, n, dm, cl, best, pl, sp))


if __name__ == "__main__":
    main()
