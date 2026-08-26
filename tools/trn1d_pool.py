#!/usr/bin/env python3
"""Pool TRN1d first-attempt results across runs, grouped by the TRN1d length
the log actually reports rather than by directory name.

The claim under test is §33's: a long TRN1d gets the FIRST Phase 4 attempt to
data mode, which the default almost never does.  One afternoon's control arm
cannot carry that (§36 -- the control won one), so the arms are pooled across
every run that measured them.
"""
import os
import re
import sys
from math import comb


def fisher(a, b, c, d):
    n = a + b + c + d
    obs = comb(a + b, a) * comb(c + d, c) / comb(n, a + c)
    tot = 0.0
    for x in range(0, min(a + b, a + c) + 1):
        y, z = a + b - x, a + c - x
        w = c + d - z
        if y < 0 or z < 0 or w < 0:
            continue
        p = comb(a + b, x) * comb(c + d, z) / comb(n, a + c)
        if p <= obs + 1e-12:
            tot += p
    return tot


def score(path):
    trn = win = None
    seen_restart = False
    rate = None
    with open(path, errors="replace") as fh:
        for line in fh:
            m = re.search(r"TRN1d complete \((\d+) symbols\)", line)
            if m and trn is None:
                trn = int(m.group(1))
            if "restarting Phase 2" in line:
                seen_restart = True
            m = re.search(r"V\.90 startup complete .*downstream PCM (\d+)", line)
            if m and win is None:
                win = not seen_restart
                rate = int(m.group(1))
    return trn, win, rate


def main(roots):
    pools = {}
    for root in roots:
        for name in sorted(os.listdir(root)):
            d = os.path.join(root, name)
            log = os.path.join(d, "server.log")
            if not os.path.isdir(d) or not os.path.exists(log):
                continue
            trn, win, rate = score(log)
            if trn is None or win is None:
                continue                       # never reached data mode
            p = pools.setdefault(trn, dict(n=0, wins=0, rates=[]))
            p["n"] += 1
            p["wins"] += 1 if win else 0
            p["rates"].append(rate)

    print(f"{'TRN1d':>7} {'calls':>6} {'1st-attempt wins':>17} {'mean rate':>10}")
    for trn in sorted(pools):
        p = pools[trn]
        mr = sum(p["rates"]) // len(p["rates"])
        print(f"{trn:>7} {p['n']:>6} {p['wins']:>8}/{p['n']:<8} {mr:>10}")

    base = pools.get(2496)
    if base:
        print()
        for trn in sorted(pools):
            if trn == 2496:
                continue
            p = pools[trn]
            pv = fisher(p["wins"], p["n"] - p["wins"],
                        base["wins"], base["n"] - base["wins"])
            print(f"  {trn} vs 2496 control: {p['wins']}/{p['n']} vs "
                  f"{base['wins']}/{base['n']}, Fisher p = {pv:.3f}")
        print(f"\n  (corpus base rate for the default, §33: 7/106 = 6.6%)")


if __name__ == "__main__":
    main(sys.argv[1:] or ["artifacts"])
