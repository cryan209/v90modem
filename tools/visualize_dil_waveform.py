#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path


def extract_label_bits(path: Path, label: str) -> str:
    txt = path.read_text(encoding="ascii", errors="ignore")
    m = re.search(rf"{re.escape(label)}\(\d+\):\n([01\s]+)", txt)
    if not m:
        raise ValueError(f"label {label!r} not found in {path}")
    return "".join(m.group(1).split())


def find_anchor(bits: str, prefix_ones: int) -> int:
    return bits.find("1" * prefix_ones)


def diff_phase_states(bits: str, init_state: int = 0) -> list[int]:
    st = init_state & 1
    out = []
    for b in bits:
        if b == "1":
            st ^= 1
        out.append(st)
    return out


def synth_bpsk_wave(states: list[int], sps: int) -> list[float]:
    wave: list[float] = []
    for st in states:
        phase = 0.0 if st == 0 else math.pi
        for n in range(sps):
            t = (n + 0.5) / sps
            wave.append(math.cos(2.0 * math.pi * t + phase))
    return wave


def ascii_plot(values: list[float], width: int = 96, height: int = 12) -> str:
    if not values:
        return "(empty)"
    if width < 4:
        width = 4
    if height < 4:
        height = 4

    n = len(values)
    col_vals: list[float] = []
    for c in range(width):
        i0 = (c * n) // width
        i1 = ((c + 1) * n) // width
        if i1 <= i0:
            i1 = min(n, i0 + 1)
        seg = values[i0:i1]
        col_vals.append(sum(seg) / len(seg))

    vmin = min(col_vals)
    vmax = max(col_vals)
    if vmax <= vmin:
        vmax = vmin + 1.0
    grid = [[" " for _ in range(width)] for _ in range(height)]

    def y_of(v: float) -> int:
        u = (v - vmin) / (vmax - vmin)
        y = int(round((1.0 - u) * (height - 1)))
        if y < 0:
            y = 0
        if y >= height:
            y = height - 1
        return y

    y0 = y_of(0.0)
    for x in range(width):
        grid[y0][x] = "-"
    for x, v in enumerate(col_vals):
        y = y_of(v)
        grid[y][x] = "*"

    lines = ["".join(row) for row in grid]
    return "\n".join(lines)


def grouped(bits: str, group: int) -> str:
    return " ".join(bits[i:i + group] for i in range(0, len(bits), group))


def main() -> int:
    ap = argparse.ArgumentParser(description="Visualize Ja->DIL candidate waveform in terminal")
    ap.add_argument("path", type=Path, help="bit dump text file")
    ap.add_argument("--label", default="dbpsk_bits_inverted")
    ap.add_argument("--prefix-ones", type=int, default=24)
    ap.add_argument("--dil-len", type=int, default=276)
    ap.add_argument("--view-bits", type=int, default=180)
    ap.add_argument("--group", type=int, default=12)
    ap.add_argument("--sps", type=int, default=16, help="samples per symbol for synthesized waveform")
    ap.add_argument("--plot-width", type=int, default=96)
    ap.add_argument("--plot-height", type=int, default=12)
    args = ap.parse_args()

    bits = extract_label_bits(args.path, args.label)
    anchor = find_anchor(bits, args.prefix_ones)
    if anchor < 0:
        print(f"label={args.label} len={len(bits)}")
        print(f"anchor_{args.prefix_ones}ones=not_found")
        return 1
    dil_start = anchor + args.prefix_ones
    if dil_start + args.dil_len > len(bits):
        print(f"label={args.label} len={len(bits)}")
        print(f"anchor_{args.prefix_ones}ones={anchor}")
        print("dil_bits=insufficient")
        return 1

    dil = bits[dil_start:dil_start + args.dil_len]
    rem = bits[dil_start + args.dil_len:]
    rep = rem[:args.dil_len] if len(rem) >= args.dil_len else ""
    rep_match = 0
    if rep:
        rep_match = sum(1 for a, b in zip(dil, rep) if a == b)

    view = dil[:args.view_bits]
    states = diff_phase_states(view, init_state=0)
    wave = synth_bpsk_wave(states, args.sps)

    phase_row = "".join("1" if s else "0" for s in states)
    match_row = ""
    if rep:
        r = rep[:args.view_bits]
        match_row = "".join("|" if a == b else "x" for a, b in zip(view, r))

    print(f"file={args.path}")
    print(f"label={args.label} total_bits={len(bits)}")
    print(f"anchor_{args.prefix_ones}ones={anchor} dil_start={dil_start} dil_len={args.dil_len}")
    if rep:
        print(f"dil_rep_match={rep_match}/{args.dil_len} ({rep_match/args.dil_len:.3f})")
    else:
        print("dil_rep_match=n/a")
    print("")
    print("DIL bits (view):")
    print(grouped(view, args.group))
    print("")
    print("Differential phase state (0/1) from DIL bits:")
    print(grouped(phase_row, args.group))
    if match_row:
        print("")
        print("Repeat block match (| match, x mismatch):")
        print(grouped(match_row, args.group))
    print("")
    print("Synthesized DIL waveform (ASCII):")
    print(ascii_plot(wave, width=args.plot_width, height=args.plot_height))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
