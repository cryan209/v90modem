#!/usr/bin/env python3
"""Audit retained G.711 RTP observer gaps against candidate fill in the RX tap.

This does not identify where a packet was lost. A sequence gap alone is not
an erasure: the packet might arrive later, or the sender might change ptime.
Match sequence AND timestamp, account for later arrivals, and report timing
rather than claiming that every silence run is concealment. No DSP changes.
"""
import argparse
import csv
import itertools
import json
import re
from pathlib import Path


def audit(directory, law="ulaw", packet_samples=160, jitter_ms=200):
    directory = Path(directory)
    if packet_samples <= 0 or jitter_ms < 0:
        raise ValueError("positive packet_samples and nonnegative jitter_ms required")
    with (directory / "rtp-rx.csv").open() as source:
        rows = list(csv.DictReader(source))
    if not rows:
        raise ValueError("empty RTP receive trace")
    keys = {(int(r["ssrc"]), int(r["seq"]), int(r["timestamp"])) for r in rows}
    # Silence can be real: these are only candidates until correlated with RTP.
    fill = 0xD5 if law == "alaw" else 0xFF
    runs = []
    at = 0
    for value, group in itertools.groupby((directory / "live-rx.g711").read_bytes()):
        length = sum(1 for _ in group)
        if value == fill and length >= packet_samples:
            runs.append({"start_s": at / 8000, "samples": length})
        at += length
    gaps = []
    absent_keys = set()
    epoch = rows[0]
    previous = None
    for row in rows:
        ssrc, seq, ts = int(row["ssrc"]), int(row["seq"]), int(row["timestamp"])
        if previous is None or ssrc != int(previous["ssrc"]):
            epoch = row
        else:
            delta = (seq - int(previous["seq"])) & 0xFFFF
            ts_delta = (ts - int(previous["timestamp"])) & 0xFFFFFFFF
            if 1 < delta < 0x8000 and ts_delta == delta * packet_samples:
                missing = [((int(previous["seq"]) + k) & 0xFFFF,
                            (int(previous["timestamp"]) + k * packet_samples) & 0xFFFFFFFF)
                           for k in range(1, delta)]
                absent = sum((ssrc, n, t) not in keys for n, t in missing)
                absent_keys.update((ssrc, n, t) for n, t in missing
                                   if (ssrc, n, t) not in keys)
                start = ((missing[0][1] - int(epoch["timestamp"])) & 0xFFFFFFFF) / 8000
                expected = start + jitter_ms / 1000
                nearest = min(runs, key=lambda r: abs(r["start_s"] - expected)) if runs else None
                gaps.append({"seq_after_gap": seq, "missing_packets_at_end": absent,
                             "observed_elsewhere": len(missing) - absent,
                             "rtp_gap_start_s": start,
                             "expected_fill_start_s": expected,
                             "nearest_fill_candidate": nearest,
                             "fill_alignment_error_s": None if nearest is None else
                                 nearest["start_s"] - expected})
        previous = row
    log = (directory / "server.log").read_text(errors="replace")
    resyncs = [int(t) / 1000 for t in re.findall(
        r"\[TRACE \+(\d+)ms\] V90 upstream discontinuity resync", log)]
    return {"law": law, "packet_samples": packet_samples, "jitter_ms": jitter_ms,
            "observer_packets": len(rows), "missing_packets_at_end": len(absent_keys),
            "gaps": gaps, "fill_candidate_runs": runs,
            "resync_trace_s": resyncs,
            "caveat": "Observer gaps do not locate loss; silence alone is not evidence of concealment. "
                      "RTP-relative, tap and TRACE epochs must be checked for each capture."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--law", choices=("ulaw", "alaw"), default="ulaw")
    parser.add_argument("--packet-samples", type=int, default=160)
    parser.add_argument("--jitter-ms", type=float, default=200)
    args = parser.parse_args()
    print(json.dumps(audit(args.directory, args.law, args.packet_samples, args.jitter_ms), indent=2))


if __name__ == "__main__":
    main()
