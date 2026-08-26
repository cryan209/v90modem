#!/usr/bin/env python3
"""Score a tools/soak/v90_concede_ab.sh directory.

SEGMENT BY CALL.  One run's server.log holds however many calls the peer
dialled -- three in the first run measured -- and the concession is per call by
design, so a run-level summary happily reports "conceded" and "reached V.90
data mode" from two different calls.  "SIP connected as answerer" starts a call.

Two questions, and they are not the same measurement:

  cost -- V.90 data-mode reachability per call.  The concession fires only
          after V.90 has already failed twice, so this must be unchanged; if it
          drops, we are giving up on calls that would have worked.
  buy  -- among calls where V.90 never worked, how soon the call stops flogging
          it, off the engine's own trace clock.
"""
import os
import re
import sys

CALL_START = "SIP connected as answerer"


def trace_ms(line):
    m = re.search(r"\[TRACE \+(\d+)ms\]", line)
    return int(m.group(1)) if m else None


def calls_in(path):
    """Yield one dict per call in a server.log."""
    if not os.path.exists(path):
        return []
    out, cur, last_ms = [], None, None

    def fresh():
        return dict(v90_data=False, rate=None, gave_up_ms=None,
                    gave_up_how=None, retrains=0)

    with open(path, errors="replace") as fh:
        for line in fh:
            if CALL_START in line:
                if cur is not None:
                    out.append(cur)
                cur, last_ms = fresh(), None
                continue
            if cur is None:
                continue
            t = trace_ms(line)
            if t is not None:
                last_ms = t
            if "restarting Phase 2" in line:
                cur["retrains"] += 1
            m = re.search(r"V\.90 startup complete .*downstream PCM (\d+)", line)
            if m:
                cur["v90_data"] = True
                if cur["rate"] is None:
                    cur["rate"] = int(m.group(1))
            if cur["gave_up_ms"] is None:
                if "conceding V.90" in line:
                    cur["gave_up_ms"], cur["gave_up_how"] = last_ms, "concede"
                elif "declined by peer INFO1a" in line:
                    cur["gave_up_ms"], cur["gave_up_how"] = last_ms, "peer"
    if cur is not None:
        out.append(cur)
    return out


def main(outdir):
    arms = {}
    for name in sorted(os.listdir(outdir)):
        p = os.path.join(outdir, name)
        if os.path.isdir(p) and "-r" in name:
            arm = name.rsplit("-r", 1)[0]
            for i, c in enumerate(calls_in(os.path.join(p, "server.log")), 1):
                arms.setdefault(arm, []).append((f"{name}#{i}", c))

    for arm in sorted(arms):
        rows = arms[arm]
        print(f"\n=== {arm}  ({len(rows)} calls)")
        print(f"{'call':18} {'V.90 data':>9} {'rate':>6} {'retr':>4} "
              f"{'gave up':>9} {'how':>8}")
        for name, c in rows:
            g = f"{c['gave_up_ms']/1000:.1f}s" if c["gave_up_ms"] else "-"
            print(f"{name:18} {'YES' if c['v90_data'] else 'no':>9} "
                  f"{str(c['rate'] or '-'):>6} {c['retrains']:>4} "
                  f"{g:>9} {c['gave_up_how'] or '-':>8}")
        got = sum(1 for _, c in rows if c["v90_data"])
        gaves = [c["gave_up_ms"] for _, c in rows if c["gave_up_ms"]]
        print(f"  COST: V.90 data mode {got}/{len(rows)} calls")
        if gaves:
            print(f"  BUY : gave up on V.90 in {len(gaves)}/{len(rows)} calls, "
                  f"mean {sum(gaves)/len(gaves)/1000:.1f}s, "
                  f"min {min(gaves)/1000:.1f}s max {max(gaves)/1000:.1f}s")
        else:
            print("  BUY : never gave up on V.90")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
