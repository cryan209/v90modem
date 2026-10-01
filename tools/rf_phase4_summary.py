#!/usr/bin/env python3
"""Score a RasFinder TRN2d sweep on what the peer does inside Phase 4.

The peer's Phase 4 outcome against this rig has been deterministic to 3 ms
over four calls whose transmit configurations differed elsewhere, so the
interesting columns are the instants, not a repeat count:

  cpt      our barred-Ri acknowledgement of a CRC-valid CPt
  trn2d/mp the transmit stage changes, stamped on the same clock as the
           retrain ([V90] is unbuffered stderr and [ME] is buffered, so they
           interleave out of order in server.log -- only [TRACE +Nms] is
           safe to read a sequence from)
  retrain  the peer abandoning Phase 4
  mp_gap   retrain - mp: how long the peer had our MP in front of it
  window   retrain - cpt

A call that never reaches Phase 4 is a rig outcome (V.8 or the Ja-parse
blocker), not a result for the knob, and is reported as such rather than
counted against the arm.
"""
import re, sys, pathlib

TRACE = re.compile(r"\[TRACE \+(\d+)ms\] (.*)")

def scan(log):
    ev = {}
    cp_valid = 0
    data_mode = False
    try:
        text = log.read_text(errors="replace")
    except OSError:
        return None
    for line in text.splitlines():
        m = TRACE.search(line)
        if m:
            t, msg = int(m.group(1)), m.group(2)
            if "CP_VALID" in msg and "kind=CPt" in msg and "accepted=1" in msg:
                ev.setdefault("cpt", t)
            elif ("Phase 4 tx stage -> TRN2d" in msg or "V90 tx stage -> TRN2d" in msg):
                ev.setdefault("trn2d", t)
            elif ("Phase 4 tx stage -> MP" in msg or "V90 tx stage -> MP" in msg):
                ev.setdefault("mp", t)
            elif ("Phase 4 tx stage -> Ed" in msg or "V90 tx stage -> Ed" in msg):
                ev.setdefault("ed", t)
            elif ("Phase 4 tx stage -> B1d" in msg or "V90 tx stage -> B1d" in msg):
                ev.setdefault("b1d", t)
            elif "peer retrain detected" in msg:
                ev.setdefault("retrain", t)
        if "kind=CP " in line and "accepted=1" in line:
            cp_valid += 1
        if "V.90 data mode" in line or "entering data mode" in line.lower():
            data_mode = True
    ev["cp_frames"] = cp_valid
    ev["data_mode"] = data_mode
    ev["trn2d_len"] = None
    m = re.search(r"CPt accepted; TRN2d \((\d+) mapped", text)
    if m:
        ev["trn2d_len"] = int(m.group(1))
    return ev

def main(root):
    rows = []
    for d in sorted(pathlib.Path(root).iterdir()):
        log = d / "server.log"
        if not log.is_file():
            continue
        ev = scan(log)
        rows.append((d.name, ev))
    hdr = f"{'call':<16}{'TRN2d':>7}{'cpt':>8}{'mp':>8}{'retrain':>9}{'mp_gap':>8}{'window':>8}{'CP':>4}  outcome"
    print(hdr); print("-"*len(hdr))
    for name, ev in rows:
        if "cpt" not in ev:
            print(f"{name:<16}{'-':>7}{'-':>8}{'-':>8}{'-':>9}{'-':>8}{'-':>8}{'-':>4}  no Phase 4 (rig)")
            continue
        cpt = ev["cpt"]; mp = ev.get("mp"); rt = ev.get("retrain")
        gap = rt - mp if (rt is not None and mp is not None) else None
        win = rt - cpt if rt is not None else None
        out = ("DATA MODE" if ev["data_mode"]
               else f"peer retrained ({ev['cp_frames']} CP)" if rt is not None
               else f"no retrain ({ev['cp_frames']} CP)")
        f = lambda v: "-" if v is None else str(v)
        print(f"{name:<16}{f(ev['trn2d_len']):>7}{cpt:>8}{f(mp):>8}"
              f"{f(rt):>9}{f(gap):>8}{f(win):>8}{ev['cp_frames']:>4}  {out}")

if __name__ == "__main__":
    main(sys.argv[1])
