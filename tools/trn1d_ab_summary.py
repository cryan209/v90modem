#!/usr/bin/env python3
"""Score a tools/soak/v90_trn1d_ab.sh directory.

The claim under test is about the FIRST Phase 4 attempt, so the ordering of
our own log matters more than any count: a call "wins" only if
"V.90 startup complete" appears before the first "restarting Phase 2".
The cost side of the lead is the downstream rate that first attempt got and
whether the call then held, so both are reported beside it.

The peer grades Phase 4 with a stopwatch it prints, so its own log gives the
study durations; enable -> disable is a completed study, enable -> retrain a
failed one.
"""
import os
import re
import sys

STUDY_RE = re.compile(
    r"<(\d+\.\d+)>.*?(enable linear mapping study|disable linear mapping study"
    r"|retrain requested)")


def peer_studies(path):
    """(duration, outcome) per study the peer ran, in order."""
    if not os.path.exists(path):
        return []
    out, start = [], None
    with open(path, errors="replace") as fh:
        for line in fh:
            m = STUDY_RE.search(line)
            if not m:
                continue
            t, what = float(m.group(1)), m.group(2)
            if what.startswith("enable"):
                start = t
            elif start is not None:
                out.append((t - start, "done" if what.startswith("disable")
                            else "retrain"))
                start = None
    return out


def score_call(d):
    log = os.path.join(d, "server.log")
    trn, first_win, rate, retrains, fell_back = None, False, None, 0, False
    seen_restart = False
    if os.path.exists(log):
        with open(log, errors="replace") as fh:
            for line in fh:
                m = re.search(r"TRN1d complete \((\d+) symbols\)", line)
                if m and trn is None:
                    trn = int(m.group(1))
                if "restarting Phase 2" in line:
                    retrains += 1
                    seen_restart = True
                    if "V.34: peer retrain" in line:
                        fell_back = True
                m = re.search(r"V\.90 startup complete .*downstream PCM (\d+)",
                              line)
                if m:
                    if rate is None:
                        rate = int(m.group(1))
                        first_win = not seen_restart
    ulines = 0
    studies = []
    for root, _, files in os.walk(d):
        if "rx_pty.bin" in files:
            with open(os.path.join(root, "rx_pty.bin"), "rb") as fh:
                ulines += len(re.findall(rb"(?m)^U\d{7}$", fh.read()))
        if "slm.log" in files:
            studies += peer_studies(os.path.join(root, "slm.log"))
    return dict(trn=trn, first_win=first_win, rate=rate, retrains=retrains,
                fell_back=fell_back, ulines=ulines, studies=studies)


def main(outdir):
    arms = {}
    for name in sorted(os.listdir(outdir)):
        d = os.path.join(outdir, name)
        if not os.path.isdir(d) or "-r" not in name:
            continue
        arm = name.rsplit("-r", 1)[0]
        arms.setdefault(arm, []).append((name, score_call(d)))

    for arm in sorted(arms):
        print(f"\n=== {arm}")
        print(f"{'call':16} {'TRN1d':>6} {'1st?':>5} {'rate':>6} "
              f"{'retr':>4} {'U-lines':>8}  peer studies")
        wins = rates = n = 0
        for name, s in arms[arm]:
            st = " ".join(f"{d:.2f}/{o}" for d, o in s["studies"][:4])
            print(f"{name:16} {str(s['trn']):>6} "
                  f"{'YES' if s['first_win'] else 'no':>5} "
                  f"{str(s['rate'] or '-'):>6} {s['retrains']:>4} "
                  f"{s['ulines']:>8}  {st}"
                  + ("  [fell to V.34]" if s["fell_back"] else ""))
            n += 1
            wins += 1 if s["first_win"] else 0
            if s["rate"]:
                rates += s["rate"]
        got = [s for _, s in arms[arm] if s["rate"]]
        print(f"  first-attempt wins {wins}/{n}; reached data {len(got)}/{n}; "
              f"mean downstream {(rates // len(got)) if got else 0} bps; "
              f"U-lines {sum(s['ulines'] for _, s in arms[arm])}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
