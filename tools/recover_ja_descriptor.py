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


def bit_at(bits: str, i: int) -> int:
    return 1 if bits[i] == "1" else 0


def bits_le(bits: str, pos: int, n: int) -> int:
    v = 0
    for i in range(n):
        v |= bit_at(bits, pos + i) << i
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


def descramble_bits(bits: str, tap: int = 4) -> str:
    reg = 0
    out = []
    for ch in bits:
        b = 1 if ch == "1" else 0
        o = (b ^ ((reg >> tap) & 1) ^ ((reg >> 22) & 1)) & 1
        reg = ((reg << 1) | b) & ((1 << 23) - 1)
        out.append("1" if o else "0")
    return "".join(out)


@dataclass
class DilDesc:
    variant: str
    n: int
    lsp: int
    ltp: int
    h: list[int]
    ref: list[int]
    train_u: list[int]
    bit_len: int


def parse_dil_descriptor(bits: str) -> DilDesc | None:
    if len(bits) < 206:
        return None
    if any(ch != "1" for ch in bits[:17]) or not expect_zero(bits, 17, 1):
        return None
    n = bits_le(bits, 18, 8)
    if not expect_zero(bits, 26, 8) or not expect_zero(bits, 34, 1):
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
    tp_res = copy_framed_pattern(bits, 51 + alpha, ltp)
    h_res = parse_byte_pairs(bits, 51 + beta, 8)
    ref_res = parse_byte_pairs(bits, 119 + beta, 8)
    if not sp_res or not tp_res or not h_res or not ref_res:
        return None
    h, _ = h_res
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
    # Legacy V.90 tail
    if expect_zero(bits, pos, 1) and (pos + 17) <= len(bits):
        p = pos + 1
        if p + 16 <= len(bits):
            crc_field = bits_le(bits, p, 16)
            if crc_field == crc16_bits(bits, crc_start):
                p += 16
                if expect_zero(bits, p, 1):
                    return DilDesc(variant="v90", n=n, lsp=lsp, ltp=ltp, h=h, ref=ref, train_u=train_u, bit_len=descriptor_bits)

    # V.92 Table 20 tail
    p = pos
    if not expect_zero(bits, p, 1):
        return None
    p += 1
    if p + 16 > len(bits):
        return None
    _ds_low = bits_le(bits, p, 16)
    p += 16
    if not expect_zero(bits, p, 1):
        return None
    p += 1
    if p + 16 > len(bits):
        return None
    _ds_high = bits_le(bits, p, 16)
    p += 16
    crc_start_v92 = p
    if not expect_zero(bits, p, 1):
        return None
    p += 1
    if p + 16 > len(bits):
        return None
    crc_field = bits_le(bits, p, 16)
    if crc_field != crc16_bits(bits, crc_start_v92):
        return None
    p += 16
    if not expect_zero(bits, p, 1):
        return None
    p += 1
    while (p % 12) != 0:
        if not expect_zero(bits, p, 1):
            return None
        p += 1
    return DilDesc(variant="v92", n=n, lsp=lsp, ltp=ltp, h=h, ref=ref, train_u=train_u, bit_len=p)


def consensus_block(bits: str, start: int, block_len: int, blocks: int) -> str:
    out = []
    for i in range(block_len):
        ones = 0
        for b in range(blocks):
            if bits[start + b * block_len + i] == "1":
                ones += 1
        out.append("1" if ones * 2 >= blocks else "0")
    return "".join(out)


def block_match(bits: str, start: int, block_len: int, blocks: int) -> float:
    if blocks < 2:
        return 0.0
    base = bits[start:start + block_len]
    acc = 0.0
    for b in range(1, blocks):
        cur = bits[start + b * block_len:start + (b + 1) * block_len]
        eq = sum(1 for x, y in zip(base, cur) if x == y)
        acc += eq / block_len
    return acc / (blocks - 1)


def main() -> int:
    ap = argparse.ArgumentParser(description="Recover Ja DIL descriptor by block-period consensus")
    ap.add_argument("bitdump", type=Path)
    ap.add_argument("--label", default="dbpsk_bits_inverted")
    ap.add_argument("--prefix-ones", type=int, default=24)
    ap.add_argument("--anchor-min", type=int, default=0)
    ap.add_argument("--anchor-max", type=int, default=4096)
    ap.add_argument("--len-min", type=int, default=206)
    ap.add_argument("--len-max", type=int, default=1200)
    ap.add_argument("--len-step", type=int, default=1)
    ap.add_argument("--min-blocks", type=int, default=3)
    ap.add_argument("--max-blocks", type=int, default=20)
    ap.add_argument("--top", type=int, default=20)
    ap.add_argument("--stream-mode", choices=("raw", "descr4", "descr17", "auto"), default="auto")
    args = ap.parse_args()

    bits = extract_label_bits(args.bitdump, args.label)
    streams = []
    if args.stream_mode == "raw":
        streams = [("raw", bits)]
    elif args.stream_mode == "descr4":
        streams = [("descr4", descramble_bits(bits, 4))]
    elif args.stream_mode == "descr17":
        streams = [("descr17", descramble_bits(bits, 17))]
    else:
        streams = [
            ("raw", bits),
            ("descr4", descramble_bits(bits, 4)),
            ("descr17", descramble_bits(bits, 17)),
        ]

    candidates = []
    for sname, sbits in streams:
        anchors = []
        patt = "1" * args.prefix_ones
        i = sbits.find(patt)
        while i >= 0:
            if args.anchor_min <= i <= args.anchor_max:
                anchors.append(i)
            i = sbits.find(patt, i + 1)

        for a in anchors:
            ds = a + args.prefix_ones
            for L in range(args.len_min, args.len_max + 1, args.len_step):
                avail = len(sbits) - ds
                maxb = min(args.max_blocks, avail // L)
                if maxb < args.min_blocks:
                    continue
                bmatch = block_match(sbits, ds, L, maxb)
                # Prefer lengths with stronger repeat.
                score = bmatch * 1000.0 - abs(L - 276) * 0.03
                cons = consensus_block(sbits, ds, L, maxb)
                d = parse_dil_descriptor(cons)
                if d is not None:
                    score += 1000.0
                candidates.append((score, sname, a, L, maxb, bmatch, d))

    candidates.sort(key=lambda x: x[0], reverse=True)
    top = candidates[: args.top]
    print("rank\tscore\tstream\tanchor\tL\tblocks\tmatch\tparse\tvar\tN\tLSP\tLTP\tbit_len")
    for i, (s, sname, a, L, b, m, d) in enumerate(top, start=1):
        if d is None:
            print(f"{i}\t{s:.2f}\t{sname}\t{a}\t{L}\t{b}\t{m:.4f}\t0\t-\t-\t-\t-\t-")
        else:
            print(f"{i}\t{s:.2f}\t{sname}\t{a}\t{L}\t{b}\t{m:.4f}\t1\t{d.variant}\t{d.n}\t{d.lsp}\t{d.ltp}\t{d.bit_len}")
    if not top:
        print("no_candidates")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
