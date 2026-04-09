#!/usr/bin/env python3
"""
Auto-search V.92 Table 15 (INFO0d) in a G.711 u-law capture.

Strict frame definition:
  bits 0:3   fill = 1111
  bits 4:11  sync = 01110010 (LSB:MSB table notation, bit 0 first on wire)
  bits 12:57 payload(46) where bits 42:57 are CRC over bits 12:41
  bits 58:61 fill = 1111
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


FSYNC12 = "111101110010"
TAIL4 = "1111"
FRAME_BITS = 62


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


def parse_table15_payload46(payload46: str) -> dict[str, int]:
    d: dict[str, int] = {}
    # 12..28 equivalent capability/control
    d["b12"] = b2i_le(payload46[0:1])
    d["b13"] = b2i_le(payload46[1:2])
    d["b14"] = b2i_le(payload46[2:3])
    d["b15"] = b2i_le(payload46[3:4])
    d["b16"] = b2i_le(payload46[4:5])
    d["b17"] = b2i_le(payload46[5:6])
    d["b18"] = b2i_le(payload46[6:7])
    d["b19"] = b2i_le(payload46[7:8])
    d["b20"] = b2i_le(payload46[8:9])
    d["b21_23"] = b2i_le(payload46[9:12])
    d["b24_cme"] = b2i_le(payload46[12:13])
    d["b25_1664"] = b2i_le(payload46[13:14])
    d["b26_short_p2_req"] = b2i_le(payload46[14:15])
    d["b27_v92_cap"] = b2i_le(payload46[15:16])
    d["b28_ack_info0a"] = b2i_le(payload46[16:17])
    # 29..41
    d["b29_32_nominal_pwr"] = b2i_le(payload46[17:21])
    d["b33_37_max_pwr"] = b2i_le(payload46[21:26])
    d["b38_codec_out_measure"] = b2i_le(payload46[26:27])
    d["b39_pcm_coding"] = b2i_le(payload46[27:28])  # 0=ulaw, 1=alaw
    d["b40_upstream_3429"] = b2i_le(payload46[28:29])
    d["b41_reserved"] = b2i_le(payload46[29:30])
    # CRC 42..57 over bits 12..41 (payload[0:30])
    d["crc_field"] = b2i_le(payload46[30:46])
    d["crc_calc"] = crc_itu16_bits_from_str(payload46[0:30])
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


def find_table15_frames(bits: str) -> list[tuple[int, str]]:
    out: list[tuple[int, str]] = []
    n = len(bits)
    for i in range(0, n - FRAME_BITS + 1):
        fr = bits[i:i + FRAME_BITS]
        if fr[:12] != FSYNC12:
            continue
        if fr[58:62] != TAIL4:
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
    ap = argparse.ArgumentParser(description="Auto-tune strict V.92 Table15 INFO0d search")
    ap.add_argument("path", type=Path)
    ap.add_argument("--fs", type=int, default=8000)
    ap.add_argument("--window-start-ms", type=float, default=4950.0)
    ap.add_argument("--window-end-ms", type=float, default=6200.0)
    ap.add_argument("--offset-step", type=float, default=0.25)
    ap.add_argument("--carrier-span", type=float, default=8.0)
    ap.add_argument("--carrier-step", type=float, default=1.0)
    ap.add_argument("--rate-span", type=float, default=0.8)
    ap.add_argument("--rate-step", type=float, default=0.1)
    ap.add_argument("--top", type=int, default=12)
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
    carriers = np.arange(2400.0 - args.carrier_span, 2400.0 + args.carrier_span + 1e-9, args.carrier_step)
    rates = np.arange(600.0 - args.rate_span, 600.0 + args.rate_span + 1e-9, args.rate_step)

    cands: list[Candidate] = []
    for off in offsets:
        for car in carriers:
            for rate in rates:
                for inv in (False, True):
                    bits = demod_bits(seg, args.fs, float(car), float(rate), float(off), inv)
                    if len(bits) < FRAME_BITS:
                        continue
                    frames = find_table15_frames(bits)
                    if not frames:
                        continue
                    crc_hits = 0
                    for _, fr in frames:
                        p = fr[12:58]
                        d = parse_table15_payload46(p)
                        crc_hits += int(d["crc_match"])
                    full_hits = len(frames)
                    score = crc_hits * 100 + full_hits
                    cands.append(Candidate(score, crc_hits, full_hits, float(off), float(car), float(rate), inv, frames))

    if not cands:
        print("no strict Table15 frames found in sweep")
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
            p = fr[12:58]
            d = parse_table15_payload46(p)
            t_ms = (s0 + (c.off + (bidx + 1) * (args.fs / c.rate))) * 1000.0 / args.fs
            print(
                f"    frame{j}: bit={bidx} t~{t_ms:.2f}ms "
                f"b27_v92={d['b27_v92_cap']} b26_short={d['b26_short_p2_req']} b39_pcm={d['b39_pcm_coding']} "
                f"crc_field=0x{d['crc_field']:04X} crc_calc=0x{d['crc_calc']:04X} crc={d['crc_match']}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

