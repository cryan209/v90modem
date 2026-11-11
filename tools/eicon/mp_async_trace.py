#!/usr/bin/env python3
"""Did eicon420's ppp stack hand the lost MLPPP fragments to the tty?

Joins a tracefs fprobe trace taken on the sending host with the sequence
numbers that never arrived on our side (tools/eicon_mlppp.py --mp-trace,
then the list of missing seqs).  Set the probes up as root on the sender
(CONFIG_FPROBE_EVENTS + CONFIG_PROBE_EVENTS_BTF_ARGS; nothing to install):

  cd /sys/kernel/tracing
  echo 'f:mpt/asend ppp_async_send chan len=skb->len d=+0(skb->data):x64' >> dynamic_events
  echo 'f:mpt/asend_r ppp_async_send%return ret=$retval:s32' >> dynamic_events
  echo 'f:mpt/iwr isicom_write tty idx=tty->index count' >> dynamic_events
  echo 'f:mpt/iwr_r isicom_write%return ret=$retval:s32' >> dynamic_events
  mkdir instances/mpt; cd instances/mpt; echo 65536 > buffer_size_kb
  for e in asend asend_r iwr iwr_r; do echo 1 > events/mpt/$e/enable; done
  cat trace_pipe > /tmp/mpt.trace      # during the run
  # afterwards: disable, rmdir instances/mpt, echo '-:mpt/<e>' >> dynamic_events

Reports, for each missing seq, whether ppp_async_send accepted it (ret 1)
and on which channel (named by the isicom tty its writes went to; channels
with no isicom writes are the Diva's), and whether the tty was pushing back
(isicom_write returning short) around then, against the base rate.

Usage: tools/eicon/mp_async_trace.py TRACE MISSING_SEQS_FILE
"""
import bisect
import collections
import re
import sys

PAT = re.compile(r'\[(\d+)\].*?\s([\d.]+): (\w+): (.*)$')
KV = re.compile(r'(\w+)=(\S+)')


def mp_seq(d):
    b = int(d, 16).to_bytes(8, 'little')   # skb->data: 00 3d flags seq(24, long form)
    return int.from_bytes(b[3:6], 'big') if b[:2] == b'\x00\x3d' else None


def main():
    trace, missf = sys.argv[1], sys.argv[2]
    missing = [int(x) for x in open(missf).read().split()]
    pend, sends, first_write = {}, collections.defaultdict(list), {}
    chan_tty = collections.defaultdict(collections.Counter)
    short = collections.defaultdict(list)          # tty -> times of short writes
    for line in open(trace, errors='replace'):
        m = PAT.search(line)
        if not m:
            continue
        cpu, t, ev, a = int(m.group(1)), float(m.group(2)), m.group(3), dict(KV.findall(m.group(4)))
        if ev == 'asend':
            pend[cpu] = [t, a['chan'], mp_seq(a['d']), None]
        elif ev == 'iwr':
            pend[('w', cpu)] = (a['tty'], int(a['count']), int(a['idx']))
        elif ev == 'iwr_r':
            w = pend.pop(('w', cpu), None)
            if not w:
                continue
            r = int(a['ret'])
            if r < w[1]:
                short[w[0]].append(t)
            p = pend.get(cpu)
            if p:
                p[3] = w[0]
                chan_tty[p[1]][w[0]] += 1
                if p[2] is not None and p[2] not in first_write:
                    first_write[p[2]] = (t, w[0])
        elif ev == 'asend_r':
            p = pend.pop(cpu, None)
            if p and p[2] is not None:
                sends[p[2]].append((p[1], int(a['ret'])))
    ttys = sorted({w for c in chan_tty.values() for w in c})
    name = {c: 'isicom tty %d' % ttys.index(cnt.most_common(1)[0][0]) for c, cnt in chan_tty.items()}
    print('MP fragments offered to ppp_async_send: %d seqs' % len(sends))
    res, by = collections.Counter(), collections.Counter()
    for s in missing:
        acc = [c for c, r in sends.get(s, []) if r == 1]
        if not sends.get(s):
            res['never offered'] += 1
        elif not acc:
            res['offered, never accepted'] += 1
        else:
            res['accepted'] += 1
            by[name.get(acc[-1], 'non-isicom chan ' + acc[-1][-4:])] += 1
    print('missing %d: %s; accepted ones by channel: %s' % (len(missing), dict(res), dict(by)))

    def near(tty, t, win):
        v = short[tty]
        k = bisect.bisect_left(v, t - win)
        return k < len(v) and v[k] <= t + win
    lost = [first_write[s] for s in missing if s in first_write]
    for win in (0.05, 0.2):
        base = [near(w, t, win) for t, w in first_write.values()]
        hit = [near(w, t, win) for t, w in lost]
        print('short isicom_write within %3.0f ms: all isicom frags %.1f%%, lost ones %.1f%% (%d/%d)'
              % (win * 1000, 100 * sum(base) / max(1, len(base)),
                 100 * sum(hit) / max(1, len(hit)), sum(hit), len(hit)))


if __name__ == '__main__':
    main()
