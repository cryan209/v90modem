#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path


def extract_label_bits(path: Path, label: str) -> str:
    txt = path.read_text(encoding="ascii", errors="ignore")
    m = re.search(rf"{re.escape(label)}\(\d+\):\n([01\s]+)", txt)
    if not m:
        raise ValueError(f"label {label!r} not found in {path}")
    return "".join(m.group(1).split())


def find_anchor(bits: str, prefix_ones: int) -> int:
    return bits.find("1" * prefix_ones)


def grouped(bits: str, group: int) -> str:
    return " ".join(bits[i:i + group] for i in range(0, len(bits), group))


def bit_at(bits: str, i: int) -> int:
    return 1 if bits[i] == "1" else 0


def bits_le(bits: str, pos: int, n: int) -> int:
    v = 0
    for i in range(n):
        v |= (bit_at(bits, pos + i) << i)
    return v


def expect_zero(bits: str, pos: int, n: int = 1) -> bool:
    return pos + n <= len(bits) and all(ch == "0" for ch in bits[pos:pos + n])


def copy_framed_pattern(bits: str, start_pos: int, out_len: int) -> tuple[list[int], int] | None:
    pos = start_pos
    out: list[int] = []
    while len(out) < out_len:
        chunk = min(16, out_len - len(out))
        if not expect_zero(bits, pos, 1):
            return None
        pos += 1
        if pos + chunk > len(bits):
            return None
        for i in range(chunk):
            out.append(bit_at(bits, pos + i))
        pos += chunk
        pad = 16 - chunk
        if pad > 0:
            if not expect_zero(bits, pos, pad):
                return None
            pos += pad
    return out, pos


def parse_byte_pairs(bits: str, start_pos: int, out_count: int) -> tuple[list[int], int] | None:
    pos = start_pos
    out: list[int] = []
    while len(out) < out_count:
        if not expect_zero(bits, pos, 1):
            return None
        pos += 1
        if pos + 8 > len(bits):
            return None
        out.append(bits_le(bits, pos, 7))
        pos += 7
        if len(out) < out_count:
            if not expect_zero(bits, pos, 1):
                return None
            pos += 1
            if pos + 8 > len(bits):
                return None
            out.append(bits_le(bits, pos, 7))
            pos += 7
            if not expect_zero(bits, pos, 1):
                return None
            pos += 1
        else:
            if not expect_zero(bits, pos, 9):
                return None
            pos += 9
    return out, pos


def crc16_bits(bits: str, bit_count: int) -> int:
    crc = 0xFFFF
    for i in range(bit_count):
        b = bit_at(bits, i)
        fb = ((crc >> 15) ^ b) & 1
        crc = ((crc << 1) & 0xFFFF)
        if fb:
            crc ^= 0x8005
    return crc & 0xFFFF


@dataclass
class DilDesc:
    n: int
    lsp: int
    ltp: int
    sp: list[int]
    tp: list[int]
    h: list[int]
    ref: list[int]
    train_u: list[int]
    bit_len: int


def parse_dil_descriptor(bits: str) -> DilDesc | None:
    if len(bits) < 206:
        return None
    if any(ch != "1" for ch in bits[:17]):
        return None
    if not expect_zero(bits, 17, 1):
        return None

    n = bits_le(bits, 18, 8)
    if not expect_zero(bits, 26, 8):
        return None
    if not expect_zero(bits, 34, 1):
        return None
    lsp = bits_le(bits, 35, 7) + 1
    if not expect_zero(bits, 42, 1):
        return None
    ltp = bits_le(bits, 43, 7) + 1
    if not expect_zero(bits, 50, 1):
        return None
    if n == 0 and (lsp != 1 or ltp != 1):
        return None

    alpha = ((lsp + 15) // 16) * 17
    beta = alpha + ((ltp + 15) // 16) * 17
    training_start = 187 + beta
    training_bits = ((n + 1) // 2) * 17
    crc_start = training_start + training_bits
    descriptor_bits = crc_start + 18
    if len(bits) < descriptor_bits:
        return None

    sp_res = copy_framed_pattern(bits, 51, lsp)
    if sp_res is None:
        return None
    sp, _ = sp_res
    tp_res = copy_framed_pattern(bits, 51 + alpha, ltp)
    if tp_res is None:
        return None
    tp, _ = tp_res

    h_res = parse_byte_pairs(bits, 51 + beta, 8)
    if h_res is None:
        return None
    h, _ = h_res
    ref_res = parse_byte_pairs(bits, 119 + beta, 8)
    if ref_res is None:
        return None
    ref, _ = ref_res

    train_u: list[int] = []
    pos = training_start
    while len(train_u) < n:
        if not expect_zero(bits, pos, 1):
            return None
        pos += 1
        if pos + 8 > len(bits):
            return None
        train_u.append(bits_le(bits, pos, 7))
        pos += 7
        if len(train_u) < n:
            if not expect_zero(bits, pos, 1):
                return None
            pos += 1
            if pos + 8 > len(bits):
                return None
            train_u.append(bits_le(bits, pos, 7))
            pos += 7
            if not expect_zero(bits, pos, 1):
                return None
            pos += 1
        else:
            if not expect_zero(bits, pos, 9):
                return None
            pos += 9

    if not expect_zero(bits, pos, 1):
        return None
    pos += 1
    if pos + 16 > len(bits):
        return None
    crc_field = bits_le(bits, pos, 16)
    pos += 16
    if crc_field != crc16_bits(bits, crc_start):
        return None
    if not expect_zero(bits, pos, 1):
        return None

    return DilDesc(
        n=n,
        lsp=lsp,
        ltp=ltp,
        sp=sp,
        tp=tp,
        h=h,
        ref=ref,
        train_u=train_u,
        bit_len=descriptor_bits,
    )


def consensus_block(bits: str, start: int, block_len: int, max_blocks: int) -> tuple[str, int]:
    if block_len <= 0 or start < 0 or start >= len(bits):
        return "", 0
    available = len(bits) - start
    blocks = min(max_blocks, available // block_len)
    if blocks <= 0:
        return "", 0
    out = []
    for i in range(block_len):
        ones = 0
        for b in range(blocks):
            if bits[start + b * block_len + i] == "1":
                ones += 1
        out.append("1" if ones * 2 >= blocks else "0")
    return "".join(out), blocks


def uchord_idx(training_ucode: int) -> int:
    idx = training_ucode >> 4
    if idx < 0:
        return 0
    if idx > 7:
        return 7
    return idx


def signed_ucode(ucode: int, sp_bit: int) -> int:
    mag = ucode & 0x7F
    return mag if sp_bit else -mag


def build_dil_symbols(desc: DilDesc, max_segments: int) -> tuple[list[float], list[tuple[int, int, int, int, int]], list[int], list[int], list[int]]:
    symbols: list[float] = []
    seg_rows: list[tuple[int, int, int, int, int]] = []
    sp_bits: list[int] = []
    tp_bits: list[int] = []
    signed_ucodes: list[int] = []
    if desc.n <= 0:
        return symbols, seg_rows, sp_bits, tp_bits, signed_ucodes

    seg_count = min(desc.n, max_segments)
    lsp = max(1, min(128, desc.lsp))
    ltp = max(1, min(128, desc.ltp))
    for seg in range(seg_count):
        tu = desc.train_u[seg] & 0x7F
        ui = uchord_idx(tu)
        h = desc.h[ui] & 0x7F
        lc = (h + 1) * 6
        r = desc.ref[ui] & 0x7F
        seg_rows.append((seg, ui + 1, tu, r, lc))
        for pos in range(lc):
            sp = 1 if desc.sp[pos % lsp] else 0
            tp = 1 if desc.tp[pos % ltp] else 0
            u = tu if tp else r
            su = signed_ucode(u, sp)
            symbols.append(su / 127.0)
            sp_bits.append(sp)
            tp_bits.append(tp)
            signed_ucodes.append(su)
    return symbols, seg_rows, sp_bits, tp_bits, signed_ucodes


def fallback_symbols_from_bits(bits: str) -> tuple[list[float], list[int]]:
    st = 0
    states: list[int] = []
    for b in bits:
        if b == "1":
            st ^= 1
        states.append(st)
    symbols = [1.0 if s else -1.0 for s in states]
    return symbols, states


def expand_hold(values: list[float], sps: int) -> list[float]:
    out: list[float] = []
    for v in values:
        for _ in range(sps):
            out.append(v)
    return out


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
    return "\n".join("".join(row) for row in grid)


def main() -> int:
    ap = argparse.ArgumentParser(description="Visualize Ja descriptor and synthesized DIL waveform in terminal")
    ap.add_argument("path", type=Path, help="bit dump text file")
    ap.add_argument("--label", default="dbpsk_bits_inverted")
    ap.add_argument("--prefix-ones", type=int, default=24)
    ap.add_argument("--dil-len", type=int, default=276,
                    help="Fallback length when descriptor parse fails; 276 only applies to V.92 when N=0")
    ap.add_argument("--view-bits", type=int, default=180)
    ap.add_argument("--group", type=int, default=12)
    ap.add_argument("--sps", type=int, default=16, help="samples per symbol for synthesized waveform")
    ap.add_argument("--max-segments", type=int, default=8, help="Max DIL segments to synthesize for preview")
    ap.add_argument("--plot-width", type=int, default=96)
    ap.add_argument("--plot-height", type=int, default=12)
    args = ap.parse_args()

    bits = extract_label_bits(args.path, args.label)
    anchor = find_anchor(bits, args.prefix_ones)
    if anchor < 0:
        print(f"label={args.label} len={len(bits)}")
        print(f"anchor_{args.prefix_ones}ones=not_found")
        return 1

    desc_start = anchor + args.prefix_ones
    if desc_start + args.dil_len > len(bits):
        print(f"label={args.label} len={len(bits)}")
        print(f"anchor_{args.prefix_ones}ones={anchor}")
        print("descriptor_bits=insufficient")
        return 1

    desc = parse_dil_descriptor(bits[desc_start:])
    parser_mode = "ok" if desc else "fallback_len_only"
    descriptor_len = desc.bit_len if desc else args.dil_len
    if desc is None:
        cons_bits, cons_blocks = consensus_block(bits, desc_start, args.dil_len, 64)
        if cons_bits:
            cons_desc = parse_dil_descriptor(cons_bits)
            if cons_desc is not None:
                desc = cons_desc
                parser_mode = f"consensus_{cons_blocks}blocks"
                descriptor_len = desc.bit_len
    block0 = bits[desc_start:desc_start + descriptor_len]
    rem = bits[desc_start + descriptor_len:]
    rep = rem[:descriptor_len] if len(rem) >= descriptor_len else ""
    rep_match = sum(1 for a, b in zip(block0, rep) if a == b) if rep else 0

    print(f"file={args.path}")
    print(f"label={args.label} total_bits={len(bits)}")
    print(f"anchor_{args.prefix_ones}ones={anchor} descriptor_start={desc_start}")
    print(f"descriptor_len={descriptor_len} parser={parser_mode}")
    if rep:
        print(f"descriptor_rep_match={rep_match}/{descriptor_len} ({rep_match/descriptor_len:.3f})")
    else:
        print("descriptor_rep_match=n/a")

    if not desc:
        print("")
        print("Descriptor parse failed; raw descriptor bits (view):")
        print(grouped(block0[:args.view_bits], args.group))
        raw_view = block0[:args.view_bits]
        sym, st = fallback_symbols_from_bits(raw_view)
        phase_row = "".join("1" if v else "0" for v in st)
        view_match = ""
        if rep:
            r = rep[:len(raw_view)]
            view_match = "".join("|" if a == c else "x" for a, c in zip(raw_view, r))
        wave = expand_hold(sym, args.sps)
        print("")
        print("Fallback differential phase state from descriptor bits:")
        print(grouped(phase_row, args.group))
        if view_match:
            print("")
            print("Descriptor repeat match (| match, x mismatch):")
            print(grouped(view_match, args.group))
        print("")
        print("Fallback descriptor-driven waveform (ASCII):")
        print(ascii_plot(wave, width=args.plot_width, height=args.plot_height))
        return 0

    print("")
    print(f"DIL descriptor fields: N={desc.n} LSP={desc.lsp} LTP={desc.ltp}")
    print("H[1..8]=" + ",".join(str(v) for v in desc.h))
    print("REF[1..8]=" + ",".join(str(v) for v in desc.ref))
    if desc.n > 0:
        preview_u = ",".join(str(v) for v in desc.train_u[: min(24, desc.n)])
        print(f"train_u_first={preview_u}")
    else:
        print("train_u_first=n/a (N=0)")

    symbols, seg_rows, sp_bits, tp_bits, signed_ucodes = build_dil_symbols(desc, args.max_segments)
    print("")
    print(f"Synthesized DIL preview: segments={len(seg_rows)} symbols={len(symbols)}")
    print("seg uchord train_u ref_u Lc")
    for seg, ui, tu, ru, lc in seg_rows:
        print(f"{seg:3d} {ui:6d} {tu:7d} {ru:5d} {lc:3d}")

    view_syms = min(args.view_bits, len(symbols))
    if view_syms <= 0:
        print("")
        print("No DIL symbols synthesized (N=0).")
        return 0

    view_sp = "".join("1" if b else "0" for b in sp_bits[:view_syms])
    view_tp = "".join("1" if b else "0" for b in tp_bits[:view_syms])
    view_match = ""
    if rep:
        r = rep[:view_syms]
        b = block0[:view_syms]
        view_match = "".join("|" if a == c else "x" for a, c in zip(b, r))

    wave = expand_hold(symbols[:view_syms], args.sps)

    print("")
    print("SP bits (restart each segment):")
    print(grouped(view_sp, args.group))
    print("")
    print("TP bits (0=REF,1=TRAIN):")
    print(grouped(view_tp, args.group))
    print("")
    print("Signed Ucodes (spec DIL output, first symbols):")
    show_ucodes = signed_ucodes[:view_syms]
    print(" ".join(f"{v:+03d}" for v in show_ucodes))
    if view_match:
        print("")
        print("Descriptor repeat match (| match, x mismatch):")
        print(grouped(view_match, args.group))
    print("")
    print("Synthesized DIL symbol waveform (ASCII):")
    print(ascii_plot(wave, width=args.plot_width, height=args.plot_height))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
