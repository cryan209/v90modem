#!/usr/bin/env python3
"""Take apart the HSF USB modem's controller firmware (hsf_rom_image.bin).

The 7399-byte ROM_IMAGE the bootloader wants is 8051 code loaded at address 0.
The proof is at file offset 0x28, which is a textbook 8051 startup:

    MOV R0,#7fh / CLR A / MOV @R0,A / DJNZ R0   -- clear internal RAM
    MOV 1fh,#00h / MOV f4h,#2dh / MOV f5h,#c0h
    MOV SP,#6ah / LJMP 0c9ah                    -- stack, then main

Two things in here matter for driving the part from the host:

  * a 105-entry LJMP table at 0x62.  105 is 0x69, exactly the size of the
    script opcode space in tools/hsf_scripts.py, so this is very likely the
    script interpreter's dispatch.  Targets at 0xd7xx-0xd9xx are in on-chip
    mask ROM we do not have; 46 of the 105 land inside the uploaded image.
    CAVEAT: several of those 46 land mid-instruction under linear disassembly,
    so either the interpreter enters shared code at computed points or this
    table is not the script dispatch after all.  Not settled.

  * SFR 0xF4 / 0xF5, which are an INDEXED CONTROL REGISTER FILE: write a
    register number to F4, then read or modify F5.  This is the chip's internal
    control plane and it is where a codec enable has to live.  --regs maps it.

    ./tools/hsf_firmware.py hsf_rom_image.bin --regs
    ./tools/hsf_firmware.py hsf_rom_image.bin --dis 0x0c26 0x0c60
"""
import argparse, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d8051 import dis

DISPATCH = 0x62


def dispatch_table(d):
    t, off = {}, DISPATCH
    i = 0
    while off + 3 <= len(d) and d[off] == 0x02:
        t[i] = (d[off + 1] << 8) | d[off + 2]
        off += 3
        i += 1
    return t


def regs(d):
    """Every 'MOV F4,#n' site and what it then does to F5."""
    from collections import defaultdict
    by = defaultdict(list)
    for i in range(len(d) - 3):
        if d[i] == 0x75 and d[i + 1] == 0xF4:
            acts = []
            for s, h, t in dis(d, i + 3, i + 3 + 14):
                if "f5h" in t:
                    acts.append(t)
                if t in ("RET", "RETI"):
                    break
            by[d[i + 2]].append((i, acts))
    return by


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--table", action="store_true", help="script opcode dispatch")
    ap.add_argument("--regs", action="store_true", help="F4/F5 control register map")
    ap.add_argument("--dis", nargs=2, metavar=("FROM", "TO"))
    a = ap.parse_args()
    d = open(a.image, "rb").read()

    if a.table:
        t = dispatch_table(d)
        print(f"{len(t)} entries at 0x{DISPATCH:02x}")
        for i, tgt in sorted(t.items()):
            where = "image" if tgt < len(d) else "mask ROM"
            print(f"  op 0x{i:02x} -> 0x{tgt:04x}  ({where})")
    if a.regs:
        by = regs(d)
        print(f"{sum(len(v) for v in by.values())} 'MOV F4,#n' sites, "
              f"{len(by)} register indices")
        for r in sorted(by):
            acts = sorted({x for _, ops in by[r] for x in ops})
            print(f"  reg 0x{r:02x}: {len(by[r]):2d} site(s)  {acts}")
    if a.dis:
        lo, hi = (int(x, 0) for x in a.dis)
        for s, h, t in dis(d, lo, hi):
            print(f"  {s:04x}: {h:<10s} {t}")


if __name__ == "__main__":
    main()
