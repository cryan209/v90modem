#!/usr/bin/env python3
"""Disassemble a file range of the Eicon MIPS protocol image (te_dmlt.pm).

Usage: mips_dis.py <image> <file_start> <file_end> [--bytes]

Runtime addresses are 0x80011000 + file offset (build 107-79 bias).
"""

from __future__ import annotations

import argparse
import sys

from capstone import Cs, CS_ARCH_MIPS, CS_MODE_LITTLE_ENDIAN, CS_MODE_MIPS32

RUNTIME_FILE_BIAS = 0x80011000


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image")
    parser.add_argument("start", type=lambda s: int(s, 0))
    parser.add_argument("end", type=lambda s: int(s, 0))
    parser.add_argument("--bytes", action="store_true", help="show raw words")
    args = parser.parse_args()

    data = open(args.image, "rb").read()
    md = Cs(CS_ARCH_MIPS, CS_MODE_MIPS32 | CS_MODE_LITTLE_ENDIAN)
    code = data[args.start:args.end]
    for ins in md.disasm(code, RUNTIME_FILE_BIAS + args.start):
        raw = ""
        if args.bytes:
            raw = " ".join(f"{b:02x}" for b in ins.bytes).ljust(11)
        print(f"{ins.address:08x}: {raw}{ins.mnemonic:10s} {ins.op_str}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
