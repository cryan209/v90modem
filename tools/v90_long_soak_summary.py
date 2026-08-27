#!/usr/bin/env python3
"""Score a tools/soak/v90_long_soak.sh directory.

A long soak asks two different questions and they need reporting separately:

  * did the call CONNECT cleanly -- did the first Phase 4 attempt reach data
    mode, or did it burn retrain cycles first (the thing §38 changed), and
  * did it STAY up -- how long data mode held, and whether both directions
    carried their numbered pattern contiguously for the whole schedule.

Sequence integrity is the honest measure in both directions, not byte counts:
an unlocked receiver emits garbage that inflates a byte total, while a missing
line is a missing line.  Downstream is "D%07d" arriving at the peer's socket,
upstream "U%07d" arriving at our PTY.
"""
import os
import re
import sys

PAT_D = re.compile(rb"(?m)^D(\d{7})$")
PAT_U = re.compile(rb"(?m)^U(\d{7})$")


def seq_stats(path, pat):
    """(lines, first, last, missing, dupes, out-of-order, bytes)."""
    if not os.path.exists(path):
        return None
    data = open(path, "rb").read()
    seqs = [int(m.group(1)) for m in pat.finditer(data)]
    if not seqs:
        return (0, None, None, 0, 0, 0, len(data))
    missing = dupes = ooo = 0
    for prev, cur in zip(seqs, seqs[1:]):
        if cur == prev + 1:
            continue
        if cur > prev + 1:
            missing += cur - prev - 1
        elif cur == prev:
            dupes += 1
        else:
            ooo += 1
    return (len(seqs), seqs[0], seqs[-1], missing, dupes, ooo, len(data))


def call_stats(d):
    log = os.path.join(d, "server.log")
    trn = rate = None
    retrains_before_data = 0
    retrains_total = 0
    seen_data = False
    if os.path.exists(log):
        with open(log, errors="replace") as fh:
            for line in fh:
                m = re.search(r"TRN1d complete \((\d+) symbols\)", line)
                if m and trn is None:
                    trn = int(m.group(1))
                if "restarting Phase 2" in line:
                    retrains_total += 1
                    if not seen_data:
                        retrains_before_data += 1
                m = re.search(r"startup complete .*downstream PCM (\d+)", line)
                if m and rate is None:
                    rate, seen_data = int(m.group(1)), True
    # The attempt directory that actually ran a schedule is the last one.
    att = sorted(x for x in os.listdir(d)
                 if x.startswith("v2attempt")
                 and os.path.isdir(os.path.join(d, x)))
    down = up = None
    hold = 0.0
    if att:
        a = os.path.join(d, att[-1])
        down = seq_stats(os.path.join(a, "rx_sock.bin"), PAT_D)
        up = seq_stats(os.path.join(a, "rx_pty.bin"), PAT_U)
        # Both pumps stamp a wall time against a byte count, so the last stamp
        # is the last moment the link DELIVERED anything -- not how long the
        # schedule ran, which is always SOAK_SECONDS.  A call that completes
        # its schedule but stops carrying at 198 s of 600 is the interesting
        # case and this is the column that shows it.
        for idx in ("rx_sock.idx", "rx_pty.idx"):
            p = os.path.join(a, idx)
            if not os.path.exists(p):
                continue
            for line in open(p, errors="replace"):
                try:
                    hold = max(hold, float(line.split()[0]))
                except (ValueError, IndexError):
                    pass
    return dict(trn=trn, rate=rate, retr_before=retrains_before_data,
                retr_total=retrains_total, hold=hold, down=down, up=up)


def fmt(s):
    if s is None:
        return "no capture"
    n, first, last, miss, dup, ooo, nbytes = s
    if not n:
        return f"0 lines / {nbytes} B"
    return (f"{n} lines {first}..{last} missing={miss} dup={dup} ooo={ooo} "
            f"({100.0 * n * 9 / nbytes:.0f}% of {nbytes} B)")


def main(outdir):
    rows = []
    for name in sorted(os.listdir(outdir)):
        d = os.path.join(outdir, name)
        if os.path.isdir(d) and name.startswith("soak-r"):
            rows.append((name, call_stats(d)))
    if not rows:
        print("no soak-r* directories in", outdir)
        return

    first_clean = sum(1 for _, s in rows
                      if s["rate"] and s["retr_before"] == 0)
    reached = sum(1 for _, s in rows if s["rate"])
    print(f"{'call':10} {'TRN1d':>6} {'rate':>6} {'retr':>5} {'carried':>7}  "
          f"downstream / upstream")
    for name, s in rows:
        print(f"{name:10} {str(s['trn']):>6} {str(s['rate'] or '-'):>6} "
              f"{s['retr_before']:>2}/{s['retr_total']:<2} "
              f"{s['hold']:>6.0f}s  {fmt(s['down'])}")
        print(f"{'':10} {'':6} {'':6} {'':5} {'':7}  {fmt(s['up'])}")
    print()
    print(f"calls {len(rows)}; reached data mode {reached}; "
          f"first Phase 4 attempt clean {first_clean}/{len(rows)}")
    dm = sum(s["down"][3] for _, s in rows if s["down"])
    dl = sum(s["down"][0] for _, s in rows if s["down"])
    um = sum(s["up"][3] for _, s in rows if s["up"])
    ul = sum(s["up"][0] for _, s in rows if s["up"])
    print(f"downstream {dl} lines, {dm} missing; upstream {ul} lines, "
          f"{um} missing; total carried "
          f"{sum(s['hold'] for _, s in rows):.0f}s")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
