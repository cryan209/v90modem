#!/usr/bin/env python3
"""Scan an rtp-rx.csv/rtp-tx.csv trace for loss, timestamp discontinuities and
arrival jitter, optionally reporting the neighbourhood of given instants.

Two things this exists to get right, both of which gave wrong answers first:

  * SEGMENT BY SSRC.  One server.log and its trace hold several calls.  Read
    unsegmented, the reneg captures report hundreds of millions of "lost
    packets" where the truth is a new SSRC and a fresh sequence space.

  * Jitter is ARRIVAL timing; it reaches the recorded audio only through the
    jitter buffer's own adaptation, because the RX tap is written from the
    buffer's output.  A trace with clean jitter therefore exonerates the
    bearer for anything visible in the recording, but a trace with dirty
    jitter does not by itself convict it.

Usage:
    tools/rtp_jitter_scan.py <rtp.csv> [ts ...]

Each optional ts is an RTP timestamp; the neighbourhood of +/-300 ms around it
is summarised, which is how a receiver-side event is checked against the wire.
Map a recording offset to a timestamp with ts = file_offset + first_timestamp.
"""
import csv
import sys
from collections import Counter, defaultdict

NOMINAL_MS = 20.0


def load(path):
    csv.field_size_limit(10 ** 8)
    rows = []
    with open(path, errors="replace") as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append((int(r["ssrc"]), int(r["seq"]),
                             int(r["timestamp"]), int(r["monotonic_ns"])))
            except (TypeError, ValueError, KeyError):
                continue
    return rows


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    rows = load(sys.argv[1])
    marks = [int(a) for a in sys.argv[2:]]
    if not rows:
        sys.exit("no usable rows")

    by_ssrc = defaultdict(list)
    for r in rows:
        by_ssrc[r[0]].append(r)
    print(f"{len(rows)} packets, {len(by_ssrc)} SSRC(s)")

    for ssrc, pkts in sorted(by_ssrc.items(), key=lambda kv: -len(kv[1])):
        pkts.sort(key=lambda x: x[3])
        lost = disc = 0
        jitter = []
        for i in range(1, len(pkts)):
            sd = (pkts[i][1] - pkts[i - 1][1]) & 0xFFFF
            td = (pkts[i][2] - pkts[i - 1][2]) & 0xFFFFFFFF
            if sd > 1:
                lost += sd - 1
            if td != 160 * max(sd, 1):
                disc += 1
            dt = (pkts[i][3] - pkts[i - 1][3]) / 1e6
            jitter.append((pkts[i][2], dt - td / 8.0))
        mags = sorted(abs(j) for _, j in jitter)
        p = lambda q: mags[min(len(mags) - 1, int(q * len(mags)))]
        print(f"\nssrc {ssrc}: {len(pkts)} packets, ts {pkts[0][2]}..{pkts[-1][2]}, "
              f"lost {lost}, timestamp discontinuities {disc}")
        print(f"  |arrival - media| ms: mean {sum(mags)/len(mags):.2f} "
              f"p50 {p(0.50):.2f} p99 {p(0.99):.2f} max {mags[-1]:.2f}")
        big = [(t, round(j, 1)) for t, j in jitter if abs(j) > 10.0]
        print(f"  packets over 10 ms: {len(big)}" + (f" -> {big[:12]}" if big else ""))
        for m in marks:
            near = [(t, j) for t, j in jitter if abs(t - m) <= 2400]
            if not near:
                continue
            worst = max(near, key=lambda x: abs(x[1]))
            mean = sum(abs(j) for _, j in near) / len(near)
            print(f"  ts {m}: {len(near)} packets within +/-300 ms, "
                  f"worst {worst[1]:+.2f} ms at ts {worst[0]}, mean |j| {mean:.2f} ms")


if __name__ == "__main__":
    main()
