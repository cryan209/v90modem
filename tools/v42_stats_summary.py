#!/usr/bin/env python3
"""Summarise V42_STATS output from a server.log.

    v42_stats_summary.py <server.log> [<server.log> ...]

V42_STATS=<seconds> makes spandsp's V.42 print one [V42STAT] line per period
(spandsp-master/src/v42.c, stats_report()).  This averages the periods in which
this end was actually moving data in at least one direction, and says where the
transmit line time went: I-frames, or idle because the window was closed, the
far end was busy, or there was nothing to send.
"""
import re
import sys

PAT = re.compile(
    r"\[V42STAT\] (\w+) rate=(\d+) k=(\d+)/(\d+) n401=(\d+)/(\d+) \| "
    r"tx I=(\d+) \((\d+) retx\) (\d+) oct/s rx I=(\d+) (\d+) oct/s RR=(\d+) \| "
    r"line: I (\d+)% ctrl (\d+)% idle-window (\d+)% idle-busy (\d+)% "
    r"idle-nodata (\d+)% idle-other (\d+)% \| "
    r"ack rtt ms n=(\d+) avg (\d+) min (\d+) max (\d+) outstanding<=(\d+)")


def summarise(path):
    rows = []
    with open(path, errors="replace") as f:
        for line in f:
            m = PAT.search(line)
            if m:
                rows.append([m.group(1)] + [int(x) for x in m.groups()[1:]])
    busy = [r for r in rows if r[8] > 0 or r[10] > 0]
    print(f"{path}: {len(rows)} periods, {len(busy)} carrying data")
    if not busy:
        return
    # Steady state: drop the first and last carrying periods (ramp up/down).
    steady = busy[1:-1] if len(busy) > 4 else busy
    n = len(steady)

    def avg(i):
        return sum(r[i] for r in steady) / n

    rtt_n = sum(r[18] for r in steady)
    rtt = (sum(r[19] * r[18] for r in steady) / rtt_n) if rtt_n else 0
    print(f"  rate={steady[0][1]} k={steady[0][2]}/{steady[0][3]} "
          f"n401={steady[0][4]}/{steady[0][5]}  steady periods={n}")
    print(f"  tx {avg(8):.0f} oct/s ({avg(8)*8/1000:.1f} kbit/s), "
          f"rx {avg(10):.0f} oct/s ({avg(10)*8/1000:.1f} kbit/s), "
          f"retransmits {sum(r[7] for r in steady)}")
    print(f"  line: I {avg(12):.0f}%  ctrl {avg(13):.0f}%  idle-window {avg(14):.0f}%  "
          f"idle-busy {avg(15):.0f}%  idle-nodata {avg(16):.0f}%  idle-other {avg(17):.0f}%")
    print(f"  ack rtt avg {rtt:.0f} ms, min {min(r[20] for r in steady if r[18]) if rtt_n else 0}"
          f" max {max(r[21] for r in steady)} ms, outstanding<= {max(r[22] for r in steady)}")


for p in sys.argv[1:]:
    summarise(p)
