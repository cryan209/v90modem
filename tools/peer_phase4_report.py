#!/usr/bin/env python3
"""Read the SmartLink peer's own log for the two numbers that decide a V.90 call.

The peer prints both ends of the Phase 4 grading it performs on our TRN2d
("V90Phase4Demodulator reset & enable linear mapping study in TRN2" ->
"disable linear mapping study" or "retrain requested"), and it prints the
constellation it designed, including the linear value it MEASURED for each of
our Ucodes ("linearMapping of constel").  That second number is checkable
against ground truth, because a Ucode's linear magnitude is defined by G.711:

    mag(u) = (((2*(u & 15) + 33) << (u >> 4)) - 33) * 2

so the ratio measured/true says whether the peer resolved which G.711 chord our
downstream is in.  Against this rig it comes out at either 1.00 or 2.00 -- one
whole chord -- and never in between, so it is a discrete decision of the peer's
level estimator and not a noisy measurement.

Usage: peer_phase4_report.py <dir-or-slm.log> ...
"""
import os
import re
import sys

UCODE_RE = re.compile(r"ucode\[\s*(\d+)\s*\]\s*:\s*(\d+).*?:\s*[\d ]+:\s*(\d+)")
RATIO_RE = re.compile(r"ph4MeanErrorEnergyBeforeToAfterUpdateRatio = \+?([\d.]+)")


def mag(u):
    """G.711 mu-law linear magnitude of V.90 Ucode u, 16-bit scale."""
    return (((2 * (u & 15) + 33) << (u >> 4)) - 33) * 2


def report(path):
    level, studies, cur, size = None, [], None, None
    with open(path, errors="replace") as fh:
        for line in fh:
            if level is None:
                m = UCODE_RE.search(line)
                # Skip ucode[0]: the top of the ladder is the one most likely
                # to be clipped, so take a mid-ladder entry instead.
                if m and int(m.group(1)) >= 5:
                    true = mag(int(m.group(2)))
                    if true:
                        level = int(m.group(3)) / true
            if size is None:
                m = re.search(r"constelation size phase\[0\.\.5\]\s*:\s*(\d+)",
                              line)
                if m:
                    size = int(m.group(1))
            if "enable linear mapping study in TRN2" in line:
                cur = None
            elif "retrain requested" in line:
                studies.append(("retrain", cur))
                cur = None
            elif ("disable linear mapping study" in line
                  and " data" not in line):
                studies.append(("done", cur))
                cur = None
            else:
                m = RATIO_RE.search(line)
                if m:
                    cur = float(m.group(1))
    return level, size, studies


def main(args):
    for a in args:
        logs = ([a] if a.endswith(".log")
                else sorted(os.path.join(r, "slm.log")
                            for r, _, f in os.walk(a) if "slm.log" in f))
        for log in logs:
            level, size, studies = report(log)
            st = " ".join(f"{o}:{r}" for o, r in studies[:5])
            lv = f"{level:.3f}" if level else "  -  "
            print(f"{log:64} level={lv} ucodes={str(size):>4}  {st}")


if __name__ == "__main__":
    main(sys.argv[1:] or ["."])
