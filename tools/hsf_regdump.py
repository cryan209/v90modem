#!/usr/bin/env python3
"""Read the HSF controller's internal control register file from the host.

Opcode 0x03 <reg> loads a control register (SFR F4/F5 indexed file) into the
script's result byte, and opcode 0x26 appends that byte to the completion
notification.  Found by sweeping opcodes against a firmware-supplied oracle:
startup does MOV f4h,#2dh / MOV f5h,#c0h, so register 0x2D must read 0xC0, and
0x03 was the only opcode that returned it.

Reads are batched -- many "03 rr 26" pairs in one script -- so the whole file
comes back in a handful of round trips instead of one per register.
"""
import argparse, re, subprocess, sys

PROBE = "./hsf_fxo_probe"
MARK = 0x5A              # leading "27 5a" so a completion can be told from a
                         # stale one left by an earlier script -- reading the
                         # first notification that turns up gave a previous
                         # attempt the wrong answer entirely
PER_SCRIPT = 4           # MEASURED: the completion payload caps at ~6 bytes,
                         # so 16 reads per script silently returned only 5


def read_block(regs, secs=3, tries=3, live=False):
    body = f"27{MARK:02x}" + "".join(f"03{r:02x}26" for r in regs) + "2701" + "28" + "36"
    # The completion can land after a short stream window, so retry rather than
    # reporting a timing miss as a failed read.
    for _ in range(tries):
        cmd = [PROBE, "--raw", body, "--stream", str(secs)]
        if live:
            # Dump the file while the device is actually streaming: bring the
            # session up and open the stream first, and send the read script
            # afterwards.  An idle dump and a streaming one are different
            # measurements and were being conflated.
            cmd = [PROBE, "--raw", body, "--raw-post", "--start-codec",
                   "--stream-open", "--feed", "--stream", str(secs)]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=90)
        o = r.stdout + r.stderr
        if "NOT RESPONDING" in o or "GET_INFROMATION failed" in o:
            return None, False
        for l in o.splitlines():
            if "data=" in l and "wValue=0x0001" in l:
                m = re.search(r"data=([0-9a-f]+)", l)
                if m:
                    b = bytes.fromhex(m.group(1))
                    if not b or b[0] != MARK:
                        continue          # somebody else's completion
                    b = b[1:]
                    if b and b[-1] == 0x01:
                        b = b[:-1]        # the trailing 27 01
                    return b, True
    return None, True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lo", type=lambda x: int(x, 0), default=0x00)
    ap.add_argument("--hi", type=lambda x: int(x, 0), default=0x60)
    ap.add_argument("--write", nargs=2, metavar=("REG", "VAL"),
                    help="0b <reg> <val>, with a read-back either side")
    ap.add_argument("--read", metavar="REGS",
                    help="comma-separated registers to read once")
    ap.add_argument("--live", action="store_true",
                    help="dump while a session is up and streaming")
    ap.add_argument("--verify", action="store_true",
                    help="read 0x2d and 0x01 separately first as a control")
    a = ap.parse_args()

    if a.read:
        rs = [int(x, 0) for x in a.read.split(",")]
        v, alive = read_block(rs, live=a.live)
        print("  " + " ".join(f"{r:02x}={('%02x' % x) if v and i < len(v) else '??'}"
                              for i, (r, x) in enumerate(zip(rs, (v or b'\xff' * len(rs))))))
        return

    if a.write:
        reg, val = (int(x, 0) for x in a.write)
        before, alive = read_block([reg])
        body = f"27{MARK:02x}0b{reg:02x}{val:02x}2701" + "28" + "36"
        r = subprocess.run([PROBE, "--raw", body, "--stream", "3"],
                           capture_output=True, text=True, timeout=60)
        ok = "accepted" in (r.stdout + r.stderr)
        after, alive = read_block([reg])
        print(f"  reg 0x{reg:02x}: before={before.hex() if before else '??'} "
              f"write 0x{val:02x} {'accepted' if ok else 'REJECTED'} "
              f"after={after.hex() if after else '??'}"
              f"{'' if alive else '   DEVICE WEDGED'}")
        return

    if a.verify:
        # A single lucky value proves nothing; two registers that are known to
        # differ, read one at a time, is the actual control.
        for r in (0x2D, 0x01, 0x2D):
            v, alive = read_block([r])
            print(f"  reg 0x{r:02x} -> {v.hex() if v else '(none)'}"
                  f"{'' if alive else '  DEVICE WEDGED'}")
            if not alive:
                sys.exit(1)
        return

    vals = {}
    r = a.lo
    while r < a.hi:
        blk = list(range(r, min(r + PER_SCRIPT, a.hi)))
        b, alive = read_block(blk, live=a.live)
        if not alive:
            print(f"wedged reading 0x{blk[0]:02x}..0x{blk[-1]:02x}; replug")
            break
        if b:
            # Keep partials: a short payload still carries the first N reads.
            for reg, val in zip(blk, b):
                vals[reg] = val
            if len(b) < len(blk):
                print(f"  0x{blk[0]:02x}..0x{blk[-1]:02x}: short "
                      f"({len(b)} of {len(blk)}), kept")
        r += PER_SCRIPT

    for base in range(a.lo, a.hi, 16):
        row = [vals.get(x) for x in range(base, min(base + 16, a.hi))]
        print(f"  {base:02x}: " + " ".join("--" if v is None else f"{v:02x}"
                                           for v in row))


if __name__ == "__main__":
    main()
