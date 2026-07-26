#!/usr/bin/env python3
"""ADSP-2181 disassembler driven by the emulator core's own dispatch tables.

Usage: adsp2181_dis.py <pm.bin> <start> <end>

Decodes every op with the same bit layouts as tools/adsp2181emu/adsp2181_core.c,
so the output always matches what the emulator executes. ALU/MAC/shifter
multifunction ops print the family plus their DM/PM DAG transfers.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

CORE = Path(__file__).resolve().parent / "adsp2181emu" / "adsp2181_core.c"

REG_GROUP0 = ["AX0","AX1","MX0","MX1","AY0","AY1","MY0","MY1",
              "SI","SE","AR","MR0","MR1","MR2","SR0","SR1"]
REG_GROUP1 = ["I0","I1","I2","I3","M0","M1","M2","M3",
              "L0","L1","L2","L3","?","?","PMOVLAY","DMOVLAY"]
REG_GROUP2 = ["I4","I5","I6","I7","M4","M5","M6","M7",
              "L4","L5","L6","L7","?","?","?","?"]
REG_GROUP3 = ["ASTAT","MSTAT","SSTAT","IMASK","ICNTL","CNTR","SB","PX",
              "RX0","?","RX1","?","?","?","?","SP"]
REG_GROUPS = [REG_GROUP0, REG_GROUP1, REG_GROUP2, REG_GROUP3]

COND = ["EQ","NE","GT","LE","LT","GE","AV","NOT AV","MV","NOT MV",
        "NEG","POS","ABS","NOT ABS","FLAG_IN","NOT FLAG_IN","CE","ALWAYS"]


def case_comments() -> dict[int, str]:
    comments: dict[int, str] = {}
    src = CORE.read_text()
    pending: list[int] = []
    for line in src.splitlines():
        for m in re.finditer(r"case (0x[0-9a-f]+):", line):
            pending.append(int(m.group(1), 16))
        cmt = re.search(r"/\* (.+?) \*/", line)
        if cmt and pending:
            comments[pending[-1]] = cmt.group(1)
            pending = []
    # fill ranges: each case in a shared line got only the last one; expand
    out: dict[int, str] = {}
    last = None
    for c in range(0x100):
        if c in comments:
            last = comments[c]
        out[c] = last or ""
    return out


COMMENTS = case_comments()


def load_pm(path: str) -> list[int]:
    data = open(path, "rb").read()
    return [data[a * 3] | (data[a * 3 + 1] << 8) | (data[a * 3 + 2] << 16)
            for a in range(0x4000)]


def disas(op: int) -> str:
    top = op >> 16
    cond = COND[op & 15]
    addr = (op >> 4) & 0x3FFF
    if op == 0:
        return "NOP"
    if top == 0x02:
        if op & 0x8000:
            return "IDLE"
        return f"FLAG_OUT({op & 0xff:02x})"
    if top == 0x0A:
        what = "RTI" if op & 0x10 else "RTS"
        return f"IF {cond} {what}" if (op & 15) != 15 else what
    if top == 0x0B:
        kind = "CALL" if op & 0x10 else "JUMP"
        return f"IF {cond} {kind} (I{4 + ((op >> 6) & 3)})"
    if 0x18 <= top <= 0x1B:
        return f"IF {cond} JUMP ${addr:04X}"
    if 0x1C <= top <= 0x1F:
        return f"IF {cond} CALL ${addr:04X}"
    if 0x80 <= top <= 0x8F:
        grp = (top >> 2) & 3
        return f"{REG_GROUPS[grp][op & 15]} = DM(${addr:04X})"
    if 0x90 <= top <= 0x9F:
        grp = (top >> 2) & 3
        return f"DM(${addr:04X}) = {REG_GROUPS[grp][op & 15]}"
    if 0xA0 <= top <= 0xAF:
        return f"DM(I{(op >> 2) & 3},M{op & 3}) = ${(op >> 4) & 0xFFF:04X}"
    if 0xB0 <= top <= 0xBF:
        return f"DM(I{4 + ((op >> 2) & 3)},M{4 + (op & 3)}) = ${(op >> 4) & 0xFFF:04X}"
    if top == 0x0D:
        return (f"{REG_GROUPS[(op >> 10) & 3][(op >> 4) & 15]} = "
                f"{REG_GROUPS[(op >> 8) & 3][op & 15]}")
    if top == 0x0C:
        return f"MODE_CTL({op & 0xffff:04x})"
    if top == 0x01:
        if op & 0x8000:
            return f"IO(${(op >> 4) & 0x7FF:03X}) = {REG_GROUP0[op & 15]}"
        return f"{REG_GROUP0[op & 15]} = IO(${(op >> 4) & 0x7FF:03X})"
    if top in (0x10, 0x11):
        return f"DO ${op & 0x3FFF:04X} UNTIL {COND[(op >> 4) & 15]}"
    cmt = COMMENTS.get(top, "")
    if cmt:
        desc = cmt.split("  ", 1)[-1].strip() if "  " in cmt else cmt
        # annotate DAG transfers embedded in multifunction ops
        notes = []
        if "data read" in desc or "data write" in desc or "dual" in desc:
            notes.append(f"[dag1 I{(op >> 2) & 3}/M{op & 3}"
                         f" dag2 I{4 + ((op >> 6) & 3)}/M{4 + ((op >> 4) & 3)}]")
        return f"{desc}{''.join(notes)}"
    return f"?{op:06X}"


def main() -> int:
    if len(sys.argv) != 4:
        print(__doc__, file=sys.stderr)
        return 2
    pm = load_pm(sys.argv[1])
    start, end = int(sys.argv[2], 0), int(sys.argv[3], 0)
    for a in range(start, end):
        print(f"{a:04x}: {pm[a] & 0xFFFFFF:06x}  {disas(pm[a])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
