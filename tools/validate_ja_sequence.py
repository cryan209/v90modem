#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
from pathlib import Path


def longest_run_ones(bits: str) -> tuple[int, int]:
    best = 0
    best_start = -1
    cur = 0
    cur_start = 0
    for i, b in enumerate(bits):
        if b == "1":
            if cur == 0:
                cur_start = i
            cur += 1
            if cur > best:
                best = cur
                best_start = cur_start
        else:
            cur = 0
    return best, best_start


def main() -> int:
    ap = argparse.ArgumentParser(description="Validate Ja bitstream structure")
    ap.add_argument("path", type=Path)
    ap.add_argument("--label", type=str, default="nat_bits", help="bit label inside file")
    ap.add_argument("--dil-len", type=int, default=276)
    args = ap.parse_args()

    txt = args.path.read_text(encoding="ascii", errors="ignore")
    m = re.search(rf"{re.escape(args.label)}\(\d+\):\n([01\s]+)", txt)
    if not m:
        print(f"label={args.label} not found")
        return 1
    bits = "".join(m.group(1).split())
    run, run_start = longest_run_ones(bits)
    print(f"label={args.label} len={len(bits)}")
    print(f"longest_ones_run={run} at={run_start}")
    print(f"multiple_of_12={1 if (len(bits) % 12) == 0 else 0}")

    anchor = bits.find("1" * 24)
    if anchor < 0:
        print("prefix_24_ones_found=0")
        return 0
    print(f"prefix_24_ones_found=1 at={anchor}")

    rem = bits[anchor + 24 :]
    n = len(rem) // args.dil_len
    print(f"complete_{args.dil_len}b_blocks_after_prefix={n}")
    if n >= 2:
        b0 = rem[: args.dil_len]
        for i in range(1, n):
            bi = rem[i * args.dil_len : (i + 1) * args.dil_len]
            eq = sum(1 for a, b in zip(b0, bi) if a == b)
            print(f"block0_vs_block{i}_match={eq}/{args.dil_len} ({eq/args.dil_len:.3f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

