#!/usr/bin/env python3
"""Per-link delay skew and reassembly queue depth of an MLPPP bundle.

Reads the link-*/mp-rx.tsv files `tools/eicon_mlppp.py --mp-trace` writes
(arrival time, RFC 1990 sequence number, B, E for every fragment off the
line) and reports:

  * each link's lateness: how long after the bundle's other fragments of
    neighbouring sequence numbers its fragments arrive (median and p95, ms);
  * the receive-side reassembly queue a Linux receiver would hold: the
    fragments that have arrived but wait behind a gap in the sequence space.
    ppp_generic.c caps that queue at PPP_MP_MAX_QLEN (128) and, once it is
    that long, gives up on the gap and drops the packets it covered;
  * sequence numbers that never arrived on any link.

Usage: tools/mp_skew.py RUN_DIR [--window 16]
"""
import argparse
import statistics
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run', type=Path)
    ap.add_argument('--window', type=int, default=16,
                    help='neighbouring sequence numbers a fragment is compared with')
    ap.add_argument('--qlen', type=int, default=128, help='PPP_MP_MAX_QLEN')
    args = ap.parse_args()

    arrivals = {}                     # seq -> (t, link)
    names = []
    for i, d in enumerate(sorted(args.run.glob('link-*'), key=lambda p: int(p.name.split('-')[1]))):
        names.append(d.name)
        f = d / 'mp-rx.tsv'
        if not f.exists():
            continue
        for line in f.read_text().split('\n'):
            parts = line.split()
            if len(parts) == 4:
                arrivals.setdefault(int(parts[1]), (float(parts[0]), i))
    if not arrivals:
        raise SystemExit('no mp-rx.tsv data')
    seqs = sorted(arrivals)
    lo, hi = seqs[0], seqs[-1]
    missing = (hi - lo + 1) - len(seqs)

    late = {i: [] for i in range(len(names))}
    w = args.window
    for k, s in enumerate(seqs):
        nb = [arrivals[x][0] for x in seqs[max(0, k - w):k + w + 1] if x != s]
        if nb:
            t, i = arrivals[s]
            late[i].append((t - statistics.median(nb)) * 1000)

    print('link                 frags  late median   p95  (ms, vs neighbouring seqs)')
    for i, n in enumerate(names):
        v = sorted(late[i])
        if v:
            print('%-20s %6d %12.0f %6.0f' % (n, len(v), statistics.median(v), v[int(.95 * (len(v) - 1))]))

    # Replay arrivals in time order through a gap-waiting reassembly queue.
    order = sorted(arrivals.items(), key=lambda kv: kv[1][0])
    have, nxt, qmax, over = set(), lo, 0, 0
    for s, _ in order:
        have.add(s)
        while nxt in have:
            have.discard(nxt)
            nxt += 1
        q = len(have)
        qmax = max(qmax, q)
        if q >= args.qlen:
            over += 1
            nxt = min(have)           # the kernel gives up on the gap
    print('seq %d..%d, %d never arrived; reassembly queue max %d fragments, '
          'at or over %d on %d arrivals' % (lo, hi, missing, qmax, args.qlen, over))


if __name__ == '__main__':
    main()
