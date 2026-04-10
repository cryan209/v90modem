#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def ulaw_byte_to_linear(u: int) -> int:
    u = (~u) & 0xFF
    sign = u & 0x80
    exponent = (u >> 4) & 0x07
    mantissa = u & 0x0F
    sample = ((mantissa << 3) + 0x84) << exponent
    sample -= 0x84
    return -sample if sign else sample


def interp(xs: np.ndarray, t: float) -> float:
    if t < 0 or t >= len(xs) - 1:
        return 0.0
    i = int(t)
    f = t - i
    return float(xs[i] * (1.0 - f) + xs[i + 1] * f)


def demod_dbpsk_bits(
    x: np.ndarray,
    fs: int,
    start_ms: float,
    offset_samples: float,
    carrier_hz: float,
    sym_rate: float,
    bit_count: int,
) -> str:
    step = fs / sym_rate
    start = start_ms * fs / 1000.0 + offset_samples
    n_syms = bit_count + 1
    syms = np.zeros(n_syms, dtype=np.complex128)
    for n in range(n_syms):
        c = start + n * step
        acc = 0j
        for k in range(-2, 3):
            t = c + k
            s = interp(x, t)
            acc += s * np.exp(-1j * 2.0 * math.pi * carrier_hz * (t / fs))
        syms[n] = acc
    dph = syms[1:] * np.conj(syms[:-1])
    return "".join("1" if float(np.real(v)) < 0.0 else "0" for v in dph)


def bit_at(bits: str, i: int) -> int:
    return 1 if bits[i] == "1" else 0


def bits_le(bits: str, pos: int, n: int) -> int:
    v = 0
    for i in range(n):
        v |= bit_at(bits, pos + i) << i
    return v


def popcount16(x: int) -> int:
    x &= 0xFFFF
    c = 0
    while x:
        x &= x - 1
        c += 1
    return c


def crc16_bits(bits: str, bit_count: int) -> int:
    crc = 0xFFFF
    for i in range(bit_count):
        b = bit_at(bits, i)
        fb = ((crc >> 15) ^ b) & 1
        crc = ((crc << 1) & 0xFFFF)
        if fb:
            crc ^= 0x8005
    return crc & 0xFFFF


def bit_hamming(a: str, b: str) -> int:
    n = min(len(a), len(b))
    d = 0
    for i in range(n):
        if a[i] != b[i]:
            d += 1
    d += abs(len(a) - len(b))
    return d


def descriptor_nearness(bits: str) -> tuple[int, int, int, int, int]:
    sync_ref = ("1" * 17) + "0"
    sync_hd = bit_hamming(bits[:18], sync_ref) if len(bits) >= 18 else 18

    fixed_zero_viol = 0
    fixed_zero_positions = [17, 34, 42, 50] + list(range(26, 34))
    for p in fixed_zero_positions:
        if p < len(bits) and bits[p] != "0":
            fixed_zero_viol += 1

    fs12_pos = bits.find("111101110010")

    if len(bits) < 51:
        return sync_hd, fixed_zero_viol, 16, 16, fs12_pos

    n = bits_le(bits, 18, 8) if len(bits) >= 26 else 0
    lsp = (bits_le(bits, 35, 7) + 1) if len(bits) >= 42 else 1
    ltp = (bits_le(bits, 43, 7) + 1) if len(bits) >= 50 else 1
    lsp = min(128, max(1, lsp))
    ltp = min(128, max(1, ltp))
    n = min(255, max(0, n))

    alpha = ((lsp + 15) // 16) * 17
    beta = alpha + ((ltp + 15) // 16) * 17
    training_start = 187 + beta
    training_bits = ((n + 1) // 2) * 17
    crc_start = training_start + training_bits

    crc_hd_v90 = 16
    if crc_start + 17 <= len(bits):
        crc_field = bits_le(bits, crc_start + 1, 16)
        crc_calc = crc16_bits(bits, crc_start)
        crc_hd_v90 = popcount16(crc_field ^ crc_calc)

    crc_hd_v92 = 16
    p = training_start + training_bits
    p += 1 + 16 + 1 + 16 + 1
    if p + 16 <= len(bits):
        crc_field = bits_le(bits, p, 16)
        crc_calc = crc16_bits(bits, p - 1)
        crc_hd_v92 = popcount16(crc_field ^ crc_calc)

    return sync_hd, fixed_zero_viol, crc_hd_v90, crc_hd_v92, fs12_pos


def framing_zero_viol(bits: str, start: int = 51, step: int = 17, count: int = 40) -> int:
    v = 0
    for k in range(count):
        i = start + k * step
        if i >= len(bits):
            break
        if bits[i] != "0":
            v += 1
    return v


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


def consensus_block(bits: str, start: int, block_len: int, blocks: int) -> str:
    out = []
    for i in range(block_len):
        ones = 0
        for b in range(blocks):
            if bits[start + b * block_len + i] == "1":
                ones += 1
        out.append("1" if ones * 2 >= blocks else "0")
    return "".join(out)


def frange(start: float, stop: float, step: float) -> list[float]:
    out = []
    v = start
    while v <= stop + 1e-12:
        out.append(round(v, 6))
        v += step
    return out


@dataclass
class Cand:
    score: float
    start_ms: float
    carrier_hz: float
    sym_rate: float
    offset_samples: float
    inv: int
    anchor: int
    L: int
    blocks: int
    rep: float
    sync_hd: int
    zero_viol: int
    frame17_viol: int
    crc90_hd: int
    crc92_hd: int
    fs12_pos: int


def main() -> int:
    ap = argparse.ArgumentParser(description="Refine Ja lock using framing/CRC nearness metrics")
    ap.add_argument("path", type=Path)
    ap.add_argument("--fs", type=int, default=8000)
    ap.add_argument("--start-ms", type=float, default=5695.0)
    ap.add_argument("--start-span-ms", type=float, default=20.0)
    ap.add_argument("--start-step-ms", type=float, default=5.0)
    ap.add_argument("--carrier-center", type=float, default=1920.0)
    ap.add_argument("--carrier-span", type=float, default=80.0)
    ap.add_argument("--carrier-step", type=float, default=20.0)
    ap.add_argument("--rate-center", type=float, default=3429.0)
    ap.add_argument("--rate-span", type=float, default=80.0)
    ap.add_argument("--rate-step", type=float, default=20.0)
    ap.add_argument("--offset-min", type=float, default=-1.0)
    ap.add_argument("--offset-max", type=float, default=1.0)
    ap.add_argument("--offset-step", type=float, default=0.5)
    ap.add_argument("--bits", type=int, default=8192)
    ap.add_argument("--prefix-ones", type=int, default=24)
    ap.add_argument("--anchor-max", type=int, default=1024)
    ap.add_argument("--len-min", type=int, default=206)
    ap.add_argument("--len-max", type=int, default=420)
    ap.add_argument("--blocks", type=int, default=10)
    ap.add_argument("--top", type=int, default=20)
    args = ap.parse_args()

    raw = args.path.read_bytes()
    x = np.asarray([ulaw_byte_to_linear(b) for b in raw], dtype=np.float64)

    starts = frange(args.start_ms - args.start_span_ms, args.start_ms + args.start_span_ms, args.start_step_ms)
    carriers = frange(args.carrier_center - args.carrier_span, args.carrier_center + args.carrier_span, args.carrier_step)
    rates = frange(args.rate_center - args.rate_span, args.rate_center + args.rate_span, args.rate_step)
    offsets = frange(args.offset_min, args.offset_max, args.offset_step)

    cands: list[Cand] = []
    for s in starts:
        for car in carriers:
            for rate in rates:
                for off in offsets:
                    raw_bits = demod_dbpsk_bits(x, args.fs, s, off, car, rate, args.bits)
                    for inv in (0, 1):
                        bits = raw_bits if inv == 0 else "".join("0" if b == "1" else "1" for b in raw_bits)
                        a = bits.find("1" * args.prefix_ones)
                        if a < 0 or a > args.anchor_max:
                            continue
                        ds = a + args.prefix_ones
                        avail = len(bits) - ds
                        best = None
                        for L in range(args.len_min, args.len_max + 1):
                            b = min(args.blocks, avail // L)
                            if b < 3:
                                continue
                            rep = block_match(bits, ds, L, b)
                            cons = consensus_block(bits, ds, L, b)
                            sync_hd, zviol, crc90, crc92, fs12 = descriptor_nearness(cons)
                            f17 = framing_zero_viol(cons, 51, 17, 40)
                            score = (
                                rep * 1000.0
                                - 12.0 * sync_hd
                                - 10.0 * zviol
                                - 5.0 * f17
                                - 3.0 * min(crc90, crc92)
                                + (40.0 if fs12 >= 0 else 0.0)
                            )
                            if best is None or score > best[0]:
                                best = (score, L, b, rep, sync_hd, zviol, f17, crc90, crc92, fs12)
                        if best is None:
                            continue
                        cands.append(
                            Cand(
                                score=best[0],
                                start_ms=s,
                                carrier_hz=car,
                                sym_rate=rate,
                                offset_samples=off,
                                inv=inv,
                                anchor=a,
                                L=best[1],
                                blocks=best[2],
                                rep=best[3],
                                sync_hd=best[4],
                                zero_viol=best[5],
                                frame17_viol=best[6],
                                crc90_hd=best[7],
                                crc92_hd=best[8],
                                fs12_pos=best[9],
                            )
                        )

    cands.sort(key=lambda c: c.score, reverse=True)
    top = cands[: max(1, args.top)]
    print("rank\tscore\tstart_ms\tcarrier\trate\toffset\tinv\tanchor\tL\trep\tsync_hd\tzviol\tframe17\tcrc90\tcrc92\tfs12")
    for i, c in enumerate(top, 1):
        print(
            f"{i}\t{c.score:.1f}\t{c.start_ms:.1f}\t{c.carrier_hz:.1f}\t{c.sym_rate:.1f}\t{c.offset_samples:.2f}\t"
            f"{c.inv}\t{c.anchor}\t{c.L}\t{c.rep:.4f}\t{c.sync_hd}\t{c.zero_viol}\t{c.frame17_viol}\t{c.crc90_hd}\t{c.crc92_hd}\t{c.fs12_pos}"
        )
    if not top:
        print("no_candidates")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

