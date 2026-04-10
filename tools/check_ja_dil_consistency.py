#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def ulaw_to_lin(u: int) -> int:
    u = (~u) & 0xFF
    sign = u & 0x80
    exp = (u >> 4) & 7
    man = u & 0x0F
    x = ((man << 3) + 0x84) << exp
    x -= 0x84
    return -x if sign else x


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
    sp: list[int]
    tp: list[int]
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
    sp, _ = sp_res
    tp, _ = tp_res
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
    # Try legacy V.90 tail: start, CRC16, fill
    if expect_zero(bits, pos, 1) and (pos + 17) <= len(bits):
        p = pos + 1
        if p + 16 <= len(bits):
            crc_field = bits_le(bits, p, 16)
            if crc_field == crc16_bits(bits, crc_start):
                p += 16
                if expect_zero(bits, p, 1):
                    return DilDesc(variant="v90", n=n, lsp=lsp, ltp=ltp, sp=sp, tp=tp, h=h, ref=ref, train_u=train_u, bit_len=descriptor_bits)

    # Try V.92 Table 20 tail:
    # start(0), 16-bit DS-rate mask low, start(0), 16-bit DS-rate mask high,
    # start(0), CRC16, fill(0), then zero fill to /12.
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
    return DilDesc(variant="v92", n=n, lsp=lsp, ltp=ltp, sp=sp, tp=tp, h=h, ref=ref, train_u=train_u, bit_len=p)


def consensus_block(bits: str, start: int, block_len: int, max_blocks: int) -> tuple[str, int]:
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


def dil_cycle_len(desc: DilDesc) -> int:
    if desc.n <= 0:
        return 0
    total = 0
    for i in range(desc.n):
        ui = uchord_idx(desc.train_u[i] & 0x7F)
        total += (int(desc.h[ui] & 0x7F) + 1) * 6
    return total


def norm_corr(x: np.ndarray, lag: int) -> float:
    if lag <= 0 or lag >= len(x) - 8:
        return 0.0
    a = x[:-lag]
    b = x[lag:]
    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na == 0 or nb == 0:
        return 0.0
    return float(np.dot(a, b) / (na * nb))


def best_repeat_lengths(bits: str, start: int, lmin: int, lmax: int, blocks: int, top: int = 8) -> list[tuple[float, int, int]]:
    out: list[tuple[float, int, int]] = []
    for L in range(lmin, lmax + 1):
        avail = len(bits) - start
        b = min(blocks, avail // L)
        if b < 2:
            continue
        base = bits[start:start + L]
        acc = 0.0
        for i in range(1, b):
            cur = bits[start + i * L:start + (i + 1) * L]
            eq = sum(1 for x, y in zip(base, cur) if x == y)
            acc += eq / L
        score = acc / (b - 1)
        out.append((score, L, b))
    out.sort(reverse=True)
    return out[:top]


def main() -> int:
    ap = argparse.ArgumentParser(description="Check Ja descriptor vs observed DIL window consistency")
    ap.add_argument("--bitdump", type=Path, required=True)
    ap.add_argument("--label", default="dbpsk_bits_inverted")
    ap.add_argument("--g711", type=Path, required=True)
    ap.add_argument("--dil-start-ms", type=float, required=True)
    ap.add_argument("--dil-end-ms", type=float, required=True)
    ap.add_argument("--prefix-ones", type=int, default=24)
    ap.add_argument("--descriptor-len", type=int, default=276)
    ap.add_argument("--consensus-blocks", type=int, default=64)
    ap.add_argument("--stream-mode", choices=("raw", "descr4", "descr17"), default="descr4")
    args = ap.parse_args()

    bits = extract_label_bits(args.bitdump, args.label)
    if args.stream_mode == "descr4":
        bits = descramble_bits(bits, 4)
    elif args.stream_mode == "descr17":
        bits = descramble_bits(bits, 17)

    anchor = find = bits.find("1" * args.prefix_ones)
    if find < 0:
        print("ja_anchor=not_found")
        return 1
    ds = anchor + args.prefix_ones
    d = parse_dil_descriptor(bits[ds:])
    mode = "direct"
    if d is None:
        cons, b = consensus_block(bits, ds, args.descriptor_len, args.consensus_blocks)
        if cons:
            d = parse_dil_descriptor(cons)
            if d is not None:
                mode = f"consensus_{b}blocks"
    print(f"ja_anchor_bit={anchor} descriptor_start_bit={ds} stream={args.stream_mode}")
    if d is None:
        print("descriptor_parse=failed")
        tops = best_repeat_lengths(bits, ds, 206, 500, 16, 8)
        print("top_repeat_lengths(score,len,blocks):")
        for s, L, b in tops:
            print(f"  {s:.4f} {L} {b}")
        return 2
    print(f"descriptor_parse=ok mode={mode} variant={d.variant} bit_len={d.bit_len}")
    print(f"N={d.n} LSP={d.lsp} LTP={d.ltp}")
    cyc = dil_cycle_len(d)
    print(f"predicted_dil_cycle_symbols={cyc}")

    raw = np.frombuffer(args.g711.read_bytes(), dtype=np.uint8)
    fs = 8000
    s0 = int(args.dil_start_ms * fs / 1000.0)
    s1 = int(args.dil_end_ms * fs / 1000.0)
    if s0 < 0:
        s0 = 0
    if s1 > len(raw):
        s1 = len(raw)
    w = raw[s0:s1]
    x = np.array([ulaw_to_lin(int(v)) for v in w], dtype=np.float64)
    print(f"dil_window_samples={len(w)} duration_ms={len(w)/fs*1000:.1f}")

    if cyc > 8 and cyc < len(x) - 8:
        r = norm_corr(x, cyc)
        print(f"corr_at_predicted_cycle({cyc})={r:+.4f} abs={abs(r):.4f}")
    else:
        print("corr_at_predicted_cycle=n/a")

    # Default-v90 sanity comparison
    r1500 = norm_corr(x, 1500) if len(x) > 1520 else 0.0
    print(f"corr_at_1500={r1500:+.4f} abs={abs(r1500):.4f}")
    print("consistency_hint=good" if (abs(r1500) > 0.45 or (cyc > 8 and abs(norm_corr(x, cyc)) > 0.45)) else "consistency_hint=weak")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
