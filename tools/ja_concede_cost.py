#!/usr/bin/env python3
"""What conceding on Ja actually costs, measured on the recorded corpus.

The concession (docs/v90_phase3_s_and_rbs_false_positive.md §35k) gives up on
V.90 after two consecutive Phase 3 attempts with no CRC-valid Ja descriptor.
Its only risk is a call that WOULD have recovered on attempt 3 or later, so the
question is:

    given a call that has already failed Ja twice, does it ever succeed after?

Every capture in artifacts/ predates the concession and so was recorded under
exactly the behaviour that answers it -- no control arm, and every call that
reaches the decision point contributes.

Segment by call: one server.log holds however many calls the peer dialled.
"""
import os
import re
import sys

CALL_START = "SIP connected as answerer"
ATTEMPT_END = "restarting Phase 2"
JA_OK = ("parsed Ja DIL descriptor",
         "Ja descriptor recovered",
         "Ja confirmed by CRC-valid DIL descriptor")


def attempts_per_call(path):
    """[[bool per Phase 3 attempt], ...] -- one list per call."""
    calls, cur, got = [], None, False
    with open(path, errors="replace") as fh:
        for line in fh:
            if CALL_START in line:
                if cur is not None:
                    cur.append(got)
                    calls.append(cur)
                cur, got = [], False
                continue
            if cur is None:
                continue
            if any(k in line for k in JA_OK):
                got = True
            elif ATTEMPT_END in line:
                cur.append(got)
                got = False
    if cur is not None:
        cur.append(got)
        calls.append(cur)
    return calls


def main(root):
    reached = recovered = calls_seen = 0
    examples = []
    for d in sorted(os.listdir(root)):
        if d.startswith("concede-ab-"):        # recorded WITH the concession
            continue
        for dirpath, _, files in os.walk(os.path.join(root, d)):
            if "server.log" not in files:
                continue
            for i, att in enumerate(attempts_per_call(
                    os.path.join(dirpath, "server.log")), 1):
                if not att:
                    continue
                calls_seen += 1
                run = 0
                for k, ok in enumerate(att):
                    if ok:
                        # A success AFTER the decision point is the cost.
                        if run >= 2:
                            recovered += 1
                            examples.append(f"{d} call#{i} attempt {k+1}")
                            break
                        run = 0
                    else:
                        run += 1
                        if run == 2:
                            reached += 1
    print(f"calls examined:                              {calls_seen}")
    print(f"calls reaching the decision point (2 fails): {reached}")
    print(f"  of those, recovered Ja on a later attempt: {recovered}")
    if reached:
        print(f"  => cost of conceding at 2: {recovered}/{reached} "
              f"= {100.0*recovered/reached:.1f}% of such calls")
    for e in examples[:10]:
        print("   recovered:", e)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "artifacts")
