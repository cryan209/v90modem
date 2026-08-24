#!/usr/bin/env python3
"""Undo the clock-recovery splices a recorded call injected into its own tap.

sip_modem.c applied me_cr_get_adjustment() to the buffer it handed to
me_rx_g711(), and the RX tap is written inside me_rx_g711() -- so a recording
contains the spliced stream, not the wire.  Every replay in this tree has
therefore been replaying the damage.  The server log records each splice and
its sign, and the DATA-bits lines around it bracket it in receiver time, so
the edits can be undone: a -1 (a dropped codeword) is undone by inserting
one, a +1 (a duplicated codeword) by deleting one.

Position within the bracket does not matter.  A splice is a NET SAMPLE
OFFSET: the compensating edit restores alignment for everything after it
wherever inside the bracket it lands, corrupting only the ~1600 samples in
between.  That is the same property that let an opposite slip recover a
collapsed call in the injection experiment.

  desplice_call.py <call-dir> <out.g711> --handover SEC [--rate BPS]
"""
import argparse
import os
import re

RATE = 8000


def slips(log_path):
    """In-data splices as (receiver_time, sign), from the server log."""
    out, t, in_data = [], None, False
    with open(log_path, errors="replace") as fh:
        for line in fh:
            if "enter DATA after B1" in line:
                in_data = True
            if not in_data:
                continue
            m = re.search(r"DATA bits: t=([0-9.]+)s", line)
            if m:
                t = float(m.group(1))
            m = re.search(r"cr_get_adjustment returned (-?1)", line)
            if m and t is not None:
                out.append((t, int(m.group(1))))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("call_dir")
    p.add_argument("dst")
    p.add_argument("--handover", type=float, required=True,
                   help="B1 handover in seconds, as v90_upstream_replay reports it")
    p.add_argument("--lag", type=float, default=0.05,
                   help="how far past the bracket to place the compensating edit")
    a = p.parse_args()

    ev = slips(os.path.join(a.call_dir, "server.log"))
    data = bytearray(open(os.path.join(a.call_dir, "live-rx.g711"), "rb").read())

    # Descending, so earlier edits keep their offsets.
    edits = sorted(((a.handover + t + a.lag, sign) for t, sign in ev), reverse=True)
    for when, sign in edits:
        i = int(when * RATE)
        if not (1 <= i < len(data)):
            continue
        if sign < 0:
            data[i:i] = bytes([data[i - 1]])   # it dropped one; put one back
        else:
            del data[i:i + 1]                  # it duplicated one; take one away

    open(a.dst, "wb").write(bytes(data))
    print("undid %d splice(s): %s"
          % (len(edits), ", ".join("%+d@%.2fs" % (s, w) for w, s in sorted(edits))))


if __name__ == "__main__":
    main()
