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
PER_SCRIPT = 4           # MEASURED: the completion payload caps at ~6 bytes,
                         # so 16 reads per script silently returned only 5


def read_block(regs, secs=3, tries=3):
    body = "".join(f"03{r:02x}26" for r in regs) + "2701" + "28" + "36"
    # The completion can land after a short stream window, so retry rather than
    # reporting a timing miss as a failed read.
    for _ in range(tries):
        r = subprocess.run([PROBE, "--raw", body, "--stream", str(secs)],
                           capture_output=True, text=True, timeout=90)
        o = r.stdout + r.stderr
        if "NOT RESPONDING" in o or "GET_INFROMATION failed" in o:
            return None, False
        for l in o.splitlines():
            if "data=" in l and "wValue=0x0001" in l:
                m = re.search(r"data=([0-9a-f]+)", l)
                if m:
                    b = bytes.fromhex(m.group(1))
                    if b and b[-1] == 0x01:
                        b = b[:-1]        # the trailing 27 01
                    return b, True
    return None, True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lo", type=lambda x: int(x, 0), default=0x00)
    ap.add_argument("--hi", type=lambda x: int(x, 0), default=0x60)
    ap.add_argument("--verify", action="store_true",
                    help="read 0x2d and 0x01 separately first as a control")
    a = ap.parse_args()

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
        b, alive = read_block(blk)
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
