#!/usr/bin/env python3
"""ADSP-2181 disassembler driven by the emulator core's own dispatch tables.

Usage: adsp2181_dis.py <pm.bin> <start> <end>

Decodes every op with the same bit layouts as tools/adsp2181emu/adsp2181_core.c,
so the output always matches what the emulator executes.  ALU/MAC/shifter ops
print their real operands -- the operation, its X and Y registers, and the DAG
transfer travelling with it -- taken from the same tables 2100ops.inc uses.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

EMU = Path(__file__).resolve().parent / "adsp2181emu"
CORE = EMU / "adsp2181_core.c"
OPS = EMU / "2100ops.inc"

REG_GROUP0 = ["AX0","AX1","MX0","MX1","AY0","AY1","MY0","MY1",
              "SI","SE","AR","MR0","MR1","MR2","SR0","SR1"]
REG_GROUP1 = ["I0","I1","I2","I3","M0","M1","M2","M3",
              "L0","L1","L2","L3","?","?","PMOVLAY","DMOVLAY"]
REG_GROUP2 = ["I4","I5","I6","I7","M4","M5","M6","M7",
              "L4","L5","L6","L7","?","?","?","?"]
REG_GROUP3 = ["ASTAT","MSTAT","SSTAT","IMASK","ICNTL","CNTR","SB","PX",
              "RX0","TX0","RX1","TX1","?","?","?","TOPPCSTACK"]
REG_GROUPS = [REG_GROUP0, REG_GROUP1, REG_GROUP2, REG_GROUP3]

# condition_table[] in adsp2181_core.c, in encoding order.  14 is the loop
# counter (CONDITION() decrements CNTR for it); 15 is the unconditional
# encoding -- it is not a FLAG_IN test, which is why every transfer in these
# images that looked like "IF NOT FLAG_IN" is simply unconditional.
COND = ["EQ","NE","GT","LE","LT","GE","AV","NOT AV","AC","NOT AC",
        "NEG","POS","MV","NOT MV","NOT CE","ALWAYS"]

# Operand register files, from adsp2181_core.c's alu_/mac_/shift_xregs setup.
ALU_X = ["AX0","AX1","AR","MR0","MR1","MR2","SR0","SR1"]
ALU_Y = ["AY0","AY1","AF","0"]
MAC_X = ["MX0","MX1","AR","MR0","MR1","MR2","SR0","SR1"]
MAC_Y = ["MY0","MY1","MF","0"]
SHIFT_X = ["SI","SI","AR","MR0","MR1","MR2","SR0","SR1"]

# alu_op_ar_const()'s y operand table (2100ops.inc `constants[]`).
CONSTANTS = [0x0001, 0xfffe, 0x0002, 0xfffd, 0x0004, 0xfffb, 0x0008, 0xfff7,
             0x0010, 0xffef, 0x0020, 0xffdf, 0x0040, 0xffbf, 0x0080, 0xff7f,
             0x0100, 0xfeff, 0x0200, 0xfdff, 0x0400, 0xfbff, 0x0800, 0xf7ff,
             0x1000, 0xefff, 0x2000, 0xdfff, 0x4000, 0xbfff, 0x8000, 0x7fff]


def op_table(function: str, shift: int) -> list[str]:
    """The 16 operation names 2100ops.inc switches on, in encoding order."""
    src = OPS.read_text()
    start = src.index(f"static void {function}(")
    body = src[start:src.index("\nstatic void", start + 10)]
    names = {}
    for m in re.finditer(r"case 0x([0-9a-f]+)<<%d:\s*\n\s*/\* (.+?) \*/" % shift,
                         body):
        names[int(m.group(1), 16)] = m.group(2).split("  ")[0].strip()
    return [names.get(i, "?") for i in range(16)]


ALU_OPS = op_table("alu_op_ar", 13)
MAC_OPS = op_table("mac_op_mr", 13)
SHIFT_OPS = op_table("shift_op", 11)


def operands(text: str, x: str, y: str) -> str:
    """Substitute the X and Y placeholders, not the X inside XOR."""
    return re.sub(r"\bX\b", x, re.sub(r"\bY\b", y, text))


def alu(op: int, dest: str, const: bool = False) -> str:
    y = (f"${CONSTANTS[((op >> 5) & 7) | ((op >> 8) & 0x18)]:04X}" if const
         else ALU_Y[(op >> 11) & 3])
    text = operands(ALU_OPS[(op >> 13) & 15], ALU_X[(op >> 8) & 7], y)
    return f"{dest} = {text}"


def mac(op: int, dest: str) -> str:
    text = MAC_OPS[(op >> 13) & 15]
    if text == "no-op":
        return "NOP (MAC)"
    text = operands(text, MAC_X[(op >> 8) & 7], MAC_Y[(op >> 11) & 3])
    return f"{dest} = {text}"


def shift(op: int, immediate: bool = False) -> str:
    text = SHIFT_OPS[(op >> 11) & 15].replace("(", f"{SHIFT_X[(op >> 8) & 7]} (")
    if immediate:
        sc = op & 0xFF
        return f"SR = {text} BY {sc - 256 if sc > 127 else sc}"
    return f"SR = {text}"


def dag(op: int, second: bool) -> str:
    base = 4 if second else 0
    return f"I{base + ((op >> 2) & 3)},M{base + (op & 3)}"


def compute(op: int, kind: str) -> str:
    """The compute half of a multifunction op, by destination."""
    if kind == "MR":
        return mac(op, "MR")
    if kind == "MF":
        return mac(op, "MF")
    if kind == "AR":
        return alu(op, "AR")
    return alu(op, "AF")


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


def guard(op: int, text: str) -> str:
    """Prefix a transfer with its condition; 15 is the unconditional form."""
    return text if (op & 15) == 15 else f"IF {COND[op & 15]} {text}"


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
        return what if (op & 15) == 15 else f"IF {cond} {what}"
    if top == 0x0B:
        kind = "CALL" if op & 0x10 else "JUMP"
        return guard(op, f"{kind} (I{4 + ((op >> 6) & 3)})")
    if 0x18 <= top <= 0x1B:
        return guard(op, f"JUMP ${addr:04X}")
    if 0x1C <= top <= 0x1F:
        return guard(op, f"CALL ${addr:04X}")
    if 0x80 <= top <= 0x8F:
        grp = (top >> 2) & 3
        return f"{REG_GROUPS[grp][op & 15]} = DM(${addr:04X})"
    if 0x90 <= top <= 0x9F:
        grp = (top >> 2) & 3
        return f"DM(${addr:04X}) = {REG_GROUPS[grp][op & 15]}"
    if 0xA0 <= top <= 0xAF:
        return f"DM(I{(op >> 2) & 3},M{op & 3}) = ${(op >> 4) & 0xFFFF:04X}"
    if 0xB0 <= top <= 0xBF:
        return f"DM(I{4 + ((op >> 2) & 3)},M{4 + (op & 3)}) = ${(op >> 4) & 0xFFFF:04X}"
    if top == 0x04:
        if (op & 0xFFFF) == 0x0040:
            return "DIS INTS"
        if (op & 0xFFFF) == 0x0060:
            return "ENA INTS"
        actions = []
        if op & 0x10: actions.append("POP PC")
        if op & 0x08: actions.append("POP LOOP")
        if op & 0x04: actions.append("POP CNTR")
        if op & 0x02: actions.append("POP STS" if op & 1 else "PUSH STS")
        return ", ".join(actions) if actions else "STACK CONTROL"
    if top == 0x0D:
        return (f"{REG_GROUPS[(op >> 10) & 3][(op >> 4) & 15]} = "
                f"{REG_GROUPS[(op >> 8) & 3][op & 15]}")
    if top == 0x0C:
        return f"MODE_CTL({op & 0xffff:04x})"
    if top == 0x01:
        if op & 0x8000:
            return f"IO(${(op >> 4) & 0x7FF:03X}) = {REG_GROUP0[op & 15]}"
        return f"{REG_GROUP0[op & 15]} = IO(${(op >> 4) & 0x7FF:03X})"
    if 0x14 <= top <= 0x17:
        # loop_stack_push(op & 0x3ffff): end address in bits 17-4, cond in 3-0.
        return f"DO ${(op >> 4) & 0x3FFF:04X} UNTIL {cond}"
    if top == 0x10:
        return (f"{shift(op)}, "
                f"{REG_GROUP0[(op >> 4) & 15]} = {REG_GROUP0[op & 15]}")
    if top == 0x11:
        reg = REG_GROUP0[(op >> 4) & 15]
        if op & 0x8000:
            return f"PM({dag(op, True)}) = {reg}, {shift(op)}"
        return f"{shift(op)}, {reg} = PM({dag(op, True)})"
    if top == 0x0E:
        return f"IF {cond} {shift(op)}"
    if top == 0x0F:
        return shift(op, immediate=True)
    if top in (0x12, 0x13):
        second = top == 0x13
        reg = REG_GROUP0[(op >> 4) & 15]
        if op & 0x8000:
            return f"DM({dag(op, second)}) = {reg}, {shift(op)}"
        return f"{shift(op)}, {reg} = DM({dag(op, second)})"
    if 0x20 <= top <= 0x27:                     # conditional compute
        kind = ("MR", "AR", "MF", "AF")[(top - 0x20) >> 1]
        const = kind in ("AR", "AF") and (op & 0x10)
        text = (alu(op, kind, const=True) if const else compute(op, kind))
        # Condition field 15 is how the assembler encodes an unconditional
        # compute; FLAG_IN is deasserted on this card, so it always holds.
        return text if (op & 15) == 15 else f"IF {cond} {text}"
    if 0x28 <= top <= 0x2F:                     # compute + register move
        kind = ("MR", "AR", "MF", "AF")[(top - 0x28) >> 1]
        if kind == "AR" and (op & 0xFF) == 0xAA:
            return alu(op, "AR") + "   (flags only)"
        return (f"{compute(op, kind)}, "
                f"{REG_GROUP0[(op >> 4) & 15]} = {REG_GROUP0[op & 15]}")
    if 0x50 <= top <= 0x5F:                     # compute + PM transfer (DAG2)
        kind = ("MR", "AR", "MF", "AF")[((top - 0x50) >> 1) & 3]
        reg = REG_GROUP0[(op >> 4) & 15]
        if top >= 0x58:
            return f"PM({dag(op, True)}) = {reg}, {compute(op, kind)}"
        return f"{compute(op, kind)}, {reg} = PM({dag(op, True)})"
    if 0x60 <= top <= 0x7F:                     # compute + DM transfer
        kind = ("MR", "AR", "MF", "AF")[((top - 0x60) >> 1) & 3]
        second = top >= 0x70
        write = ((top - 0x60) >> 3) & 1
        reg = REG_GROUP0[(op >> 4) & 15]
        if write:
            return f"DM({dag(op, second)}) = {reg}, {compute(op, kind)}"
        return f"{compute(op, kind)}, {reg} = DM({dag(op, second)})"
    if top >= 0xC0:                             # compute + DM read + PM read
        kind = "AR" if (top >> 1) & 1 else "MR"
        cmt = COMMENTS.get(top, "")
        m = re.search(r"data read to (\w+) & pgm read to (\w+)", cmt)
        dm_reg, pm_reg = m.groups() if m else ("?", "?")
        return (f"{compute(op, kind)}, {dm_reg} = DM({dag(op, False)}), "
                f"{pm_reg} = PM({dag(op >> 4, True)})")
    if 0x30 <= top <= 0x3F:
        grp = (top - 0x30) >> 2
        # (INT32)(op << 14) >> 18 in the core: 32-bit truncation, then an
        # arithmetic shift, so the 14-bit field is signed.  M and L registers
        # are strides and lengths, and reading them unsigned is misleading.
        val = (op << 14) & 0xFFFFFFFF
        if val & 0x80000000:
            val -= 0x100000000
        val >>= 18
        name = REG_GROUPS[grp][op & 15]
        if val < 0:
            # I and L registers are 14-bit unsigned; M registers are strides.
            return f"{name} = {val}" if name[0] == "M" else \
                   f"{name} = ${val & 0x3FFF:04X}"
        return f"{name} = ${val:04X}" if name[0] != "M" else f"{name} = {val}"
    if 0x40 <= top <= 0x4F:
        return f"{REG_GROUP0[op & 15]} = ${(op >> 4) & 0xFFFF:04X}"
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
