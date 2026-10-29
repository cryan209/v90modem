#!/usr/bin/env python3
"""Read compact [PERF] G.711 engine summaries; preserve each call/phase row.

Usage: python3 tools/media_perf_report.py modem.log.1 modem.log [--json]
Percentiles are per-row upper bounds, never averaged across calls or peers.
Wall time includes scheduling/lock waits and must not be labelled CPU time.
"""

import argparse
import json
import sys

COUNTERS = ("calls", "over_budget", "transitions")
TIMINGS = ("audio_ms", "wall_ms", "mean_us", "p95_us_upper", "p99_us_upper", "max_us")


def read_records(path):
    records = []
    with open(path, errors="replace") as stream:
        for line_no, line in enumerate(stream, 1):
            _, marker, text = line.partition("[PERF] ")
            if not marker:
                continue
            fields = dict(word.split("=", 1) for word in text.split() if "=" in word)
            if fields.get("scope") != "g711":
                continue
            try:
                row = {key: fields[key] for key in ("scope", "dir", "phase")}
                row.update({key: int(fields[key]) for key in COUNTERS})
                row.update({key: float(fields[key]) for key in TIMINGS})
                row["call"] = int(fields.get("call", "0"))
            except (KeyError, ValueError) as exc:
                raise ValueError(f"{path}:{line_no}: incomplete/invalid PERF summary") from exc
            row.update(log=str(path), line=line_no)
            records.append(row)
    return records


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--json", action="store_true", help="emit one JSON array of summary rows")
    args = ap.parse_args()
    records = []
    try:
        for path in args.logs:
            records.extend(read_records(path))
    except (OSError, ValueError) as exc:
        ap.exit(1, f"{exc}\n")
    if not records:
        ap.exit(1, "no G.711 PERF summaries found (call must disconnect or engine shut down cleanly)\n")
    if args.json:
        json.dump(records, sys.stdout, indent=2)
        print()
    else:
        print("call dir phase       calls  audio_ms   wall_ms  p95_us<=  p99_us<=    max_us  over_budget transitions source")
        for row in records:
            print("{call:4d} {dir:3} {phase:8} {calls:8d} {audio_ms:9.1f} {wall_ms:9.1f} "
                  "{p95_us_upper:9.1f} {p99_us_upper:9.1f} {max_us:9.1f} "
                  "{over_budget:12d} {transitions:11d} {log}:{line}".format(**row))
    return 0


if __name__ == "__main__":
    sys.exit(main())
