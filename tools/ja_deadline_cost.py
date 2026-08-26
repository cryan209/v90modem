#!/usr/bin/env python3
"""Price a mid-attempt deadline trigger for the Ja concession.

The shipped trigger counts Phase 3 attempts, but an attempt only ends when the
PEER retrains -- so our clock is derived from the peer's and we cannot decide
before it does (docs §35m).  The alternative is our own deadline: concede if no
CRC-valid descriptor has arrived T seconds into the call.

Priced the same way as §35l, on captures that all predate the concession, and
per call because one server.log holds many.  For each candidate T:

  fires   -- calls with no descriptor by T
  cost    -- of those, calls that WOULD have recovered a descriptor later
  costly  -- of those, calls that went on to reach V.90 data mode (the real loss)
  saves   -- median seconds earlier than that call actually gave up on V.90
"""
import os
import re
import sys

CALL_START = "SIP connected as answerer"
JA_OK = ("parsed Ja DIL descriptor", "Ja descriptor recovered",
         "Ja confirmed by CRC-valid DIL descriptor")
GAVE_UP = ("declined by peer INFO1a", "conceding V.90")


def calls_in(path):
    out, cur, last = [], None, None
    for line in open(path, errors="replace"):
        if CALL_START in line:
            if cur is not None:
                out.append(cur)
            cur, last = dict(ja_ms=None, data_ms=None, gave_ms=None), None
            continue
        if cur is None:
            continue
        m = re.search(r"\[TRACE \+(\d+)ms\]", line)
        if m:
            last = int(m.group(1))
        if any(k in line for k in JA_OK) and cur["ja_ms"] is None:
            cur["ja_ms"] = last
        if "V.90 startup complete" in line and cur["data_ms"] is None:
            cur["data_ms"] = last
        if any(k in line for k in GAVE_UP) and cur["gave_ms"] is None:
            cur["gave_ms"] = last
    if cur is not None:
        out.append(cur)
    return out


def main(root):
    calls = []
    for d in sorted(os.listdir(root)):
        if d.startswith("concede-ab-"):
            continue
        for dp, _, fs in os.walk(os.path.join(root, d)):
            if "server.log" in fs:
                calls += calls_in(os.path.join(dp, "server.log"))
    calls = [c for c in calls if c["ja_ms"] is not None or c["gave_ms"] is not None
             or c["data_ms"] is not None]
    print(f"calls with any Phase 3 activity: {len(calls)}\n")
    print(f"{'T (s)':>6} {'fires':>6} {'cost':>6} {'costly':>7} {'median saved':>13}")
    for T in (5, 10, 15, 20, 25, 30, 40, 60):
        ms = T * 1000
        fires = cost = costly = 0
        saved = []
        for c in calls:
            if c["ja_ms"] is not None and c["ja_ms"] <= ms:
                continue                      # descriptor already in hand
            fires += 1
            if c["ja_ms"] is not None:        # would have arrived later
                cost += 1
                if c["data_ms"] is not None:
                    costly += 1
            if c["gave_ms"] is not None and c["gave_ms"] > ms:
                saved.append((c["gave_ms"] - ms) / 1000.0)
        med = sorted(saved)[len(saved)//2] if saved else 0.0
        print(f"{T:>6} {fires:>6} {cost:>6} {costly:>7} {med:>12.1f}s")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "artifacts")
