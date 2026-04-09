#!/usr/bin/env python3
"""
Auto-search V.92 Table 17 (INFO1d) in a G.711 u-law capture.

Strict frame:
  0:3    fill 1111
  4:11   sync 01110010
  12:104 payload+crc (93 bits), where 89:104 is CRC over 12:88
  105:108 fill 1111
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


FSYNC12 = "111101110010"
TAIL4 = "1111"
FRAME_BITS = 109


def ulaw_byte_to_linear(u: int) -> int:
    u = (~u) & 0xFF
    sign = u & 0x80
    exponent = (u >> 4) & 0x07
    mantissa = u & 0x0F
    sample = ((mantissa << 3) + 0x84) << exponent
    sample -= 0x84
    return -sample if sign else sample


def ulaw_decode(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    return np.asarray([ulaw_byte_to_linear(b) for b in raw], dtype=np.float64)


def b2i_le(s: str) -> int:
    v = 0
    for i, ch in enumerate(s):
        if ch == "1":
            v |= 1 << i
    return v


def signed10(v: int) -> int:
    return -(v ^ 0x3FF) - 1 if (v & 0x200) else v


def crc_itu16_bits_from_str(bitstr: str) -> int:
    crc = 0xFFFF
    for ch in bitstr:
        bit = 1 if ch == "1" else 0
        if ((crc ^ bit) & 1) != 0:
            crc = (crc >> 1) ^ 0x8408
        else:
            crc >>= 1
        crc &= 0xFFFF
    return crc & 0xFFFF


def parse_table17_payload93(payload93: str) -> dict[str, int]:
    d: dict[str, int] = {}
    d["b12_14_min_pwr_red"] = b2i_le(payload93[0:3])
    d["b15_17_addl_pwr_red"] = b2i_le(payload93[3:6])
    d["b18_24_md"] = b2i_le(payload93[6:13])
    d["b70_pcm_upstream_not_supported"] = b2i_le(payload93[58:59])
    fr = b2i_le(payload93[67:77])
    d["b79_88_freq_raw"] = fr
    d["b79_88_freq_signed"] = signed10(fr)
    d["crc_field"] = b2i_le(payload93[77:93])   # bits 89:104
    d["crc_calc"] = crc_itu16_bits_from_str(payload93[0:77])  # bits 12:88
    d["crc_match"] = 1 if d["crc_field"] == d["crc_calc"] else 0
    return d


def interp(xs: np.ndarray, t: float) -> float:
    if t < 0 or t >= len(xs) - 1:
        return 0.0
    i = int(t)
    f = t - i
    return float(xs[i] * (1.0 - f) + xs[i + 1] * f)


def demod_bits(
    samples: np.ndarray,
    fs: int,
    carrier_hz: float,
    rate_bps: float,
    offset_samples: float,
    invert_diff: bool,
) -> str:
    step = fs / rate_bps
    nmax = int((len(samples) - 16) / step)
    if nmax < 20:
        return ""

    syms: list[complex] = []
    for n in range(nmax):
        center = offset_samples + n * step
        if center + 8 >= len(samples):
            break
        i_acc = 0.0
        q_acc = 0.0
        for k in range(-6, 7):
            tt = center + k
            x = interp(samples, tt)
            ang = -2.0 * math.pi * carrier_hz * (tt / fs)
            i_acc += x * math.cos(ang)
            q_acc += x * math.sin(ang)
        syms.append(complex(i_acc, q_acc))

    if len(syms) < 2:
        return ""

    bits: list[str] = []
    prev = syms[0]
    for z in syms[1:]:
        d = prev.conjugate() * z
        bit = 1 if d.real < 0 else 0
        if invert_diff:
            bit ^= 1
        bits.append(str(bit))
        prev = z
    return "".join(bits)


def find_table17_frames(bits: str) -> list[tuple[int, str]]:
    out: list[tuple[int, str]] = []
    n = len(bits)
    for i in range(0, n - FRAME_BITS + 1):
        fr = bits[i:i + FRAME_BITS]
        if fr[:12] != FSYNC12:
            continue
        if fr[105:109] != TAIL4:
            continue
        out.append((i, fr))
    return out


@dataclass
class Candidate:
    score: int
    crc_hits: int
    full_hits: int
    off: float
    car: float
    rate: float
    inv: bool
    frames: list[tuple[int, str]]


def main() -> int:
    ap = argparse.ArgumentParser(description="Auto-tune strict V.92 Table17 INFO1d search")
    ap.add_argument("path", type=Path)
    ap.add_argument("--fs", type=int, default=8000)
    ap.add_argument("--window-start-ms", type=float, default=3500.0)
    ap.add_argument("--window-end-ms", type=float, default=9000.0)
    ap.add_argument("--offset-step", type=float, default=0.5)
    ap.add_argument("--carrier-center", type=float, default=2400.0)
    ap.add_argument("--carrier-span", type=float, default=1.0)
    ap.add_argument("--carrier-step", type=float, default=1.0)
    ap.add_argument("--rate-span", type=float, default=0.15)
    ap.add_argument("--rate-step", type=float, default=0.15)
    ap.add_argument("--top", type=int, default=20)
    args = ap.parse_args()

    x = ulaw_decode(args.path)
    s0 = max(0, int(args.window_start_ms * args.fs / 1000.0))
    s1 = min(len(x), int(args.window_end_ms * args.fs / 1000.0))
    if s1 <= s0:
        raise SystemExit("empty window")
    seg = x[s0:s1]
    print(f"window={args.window_start_ms:.1f}-{args.window_end_ms:.1f} ms samples={len(seg)}")

    step = args.fs / 600.0
    offsets = np.arange(0.0, step, args.offset_step)
    carriers = np.arange(args.carrier_center - args.carrier_span,
                         args.carrier_center + args.carrier_span + 1e-9,
                         args.carrier_step)
    rates = np.arange(600.0 - args.rate_span, 600.0 + args.rate_span + 1e-9, args.rate_step)

    cands: list[Candidate] = []
    for off in offsets:
        for car in carriers:
            for rate in rates:
                for inv in (False, True):
                    bits = demod_bits(seg, args.fs, float(car), float(rate), float(off), inv)
                    if len(bits) < FRAME_BITS:
                        continue
                    frames = find_table17_frames(bits)
                    if not frames:
                        continue
                    crc_hits = 0
                    for _, fr in frames:
                        d = parse_table17_payload93(fr[12:105])
                        crc_hits += int(d["crc_match"])
                    full_hits = len(frames)
                    score = crc_hits * 100 + full_hits
                    cands.append(Candidate(score, crc_hits, full_hits, float(off), float(car), float(rate), inv, frames))

    if not cands:
        print("no strict Table17 frames found in sweep")
        return 0

    cands.sort(key=lambda c: (c.score, c.crc_hits, c.full_hits), reverse=True)
    top = cands[: max(1, args.top)]
    print(f"candidates_with_hits={len(cands)}")
    for i, c in enumerate(top, start=1):
        print(
            f"[{i}] score={c.score} crc_hits={c.crc_hits} full_hits={c.full_hits} "
            f"off={c.off:.3f} car={c.car:.3f} rate={c.rate:.3f} inv={int(c.inv)}"
        )
        for j, (bidx, fr) in enumerate(c.frames[:4], start=1):
            d = parse_table17_payload93(fr[12:105])
            t_ms = (s0 + (c.off + (bidx + 1) * (args.fs / c.rate))) * 1000.0 / args.fs
            print(
                f"    frame{j}: bit={bidx} t~{t_ms:.2f}ms md={d['b18_24_md']} "
                f"b70_no_pcm_up={d['b70_pcm_upstream_not_supported']} freq={d['b79_88_freq_signed']} "
                f"crc_field=0x{d['crc_field']:04X} crc_calc=0x{d['crc_calc']:04X} crc={d['crc_match']}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
