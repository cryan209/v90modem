#!/usr/bin/env python3
"""
Auto-search Table 16 (INFO0a) frames in a G.711 u-law capture by:
1) Finding the guard/data region where 1800 Hz and 2400 Hz are both present.
2) Demodulating 2400 Hz DBPSK at ~600 bps across that full region.
3) Sweeping timing/carrier/rate to maximize strict 49-bit Table 16 matches and CRC hits.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import check_info_tables as t16


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


def tone_metric_windows(
    x: np.ndarray,
    fs: int,
    win_ms: float = 20.0,
    hop_ms: float = 5.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    w = max(32, int(round(win_ms * fs / 1000.0)))
    h = max(8, int(round(hop_ms * fs / 1000.0)))
    n = len(x)
    if n < w:
        return np.array([]), np.array([]), np.array([])

    t1800 = np.exp(-1j * 2.0 * np.pi * 1800.0 * np.arange(w) / fs)
    t2400 = np.exp(-1j * 2.0 * np.pi * 2400.0 * np.arange(w) / fs)

    centers: list[float] = []
    n1800: list[float] = []
    n2400: list[float] = []
    eps = 1e-12
    for s in range(0, n - w + 1, h):
        seg = x[s:s + w]
        p = float(np.dot(seg, seg)) + eps
        e18 = float(np.abs(np.vdot(seg, t1800)) ** 2) / p
        e24 = float(np.abs(np.vdot(seg, t2400)) ** 2) / p
        centers.append((s + w * 0.5) / fs)
        n1800.append(e18)
        n2400.append(e24)
    return np.asarray(centers), np.asarray(n1800), np.asarray(n2400)


def find_active_region(
    t_s: np.ndarray,
    n1800: np.ndarray,
    n2400: np.ndarray,
    start_ms: float,
    min_len_ms: float = 120.0,
) -> tuple[float, float] | None:
    if len(t_s) == 0:
        return None
    start_s = start_ms / 1000.0
    m = t_s >= start_s
    if not np.any(m):
        return None

    t = t_s[m]
    a = n1800[m]
    b = n2400[m]

    thr18 = max(0.02, 0.35 * float(np.max(a)))
    thr24 = max(0.01, 0.25 * float(np.max(b)))
    mask = (a >= thr18) & (b >= thr24)
    if not np.any(mask):
        return None

    # Longest contiguous active run.
    runs: list[tuple[int, int]] = []
    i = 0
    while i < len(mask):
        if not mask[i]:
            i += 1
            continue
        j = i + 1
        while j < len(mask) and mask[j]:
            j += 1
        runs.append((i, j))
        i = j
    if not runs:
        return None

    best = max(runs, key=lambda r: r[1] - r[0])
    i0, i1 = best
    t0 = float(t[i0])
    t1 = float(t[i1 - 1])
    if (t1 - t0) * 1000.0 < min_len_ms:
        return None
    # Pad slightly for edges.
    return (max(start_s, t0 - 0.02), t1 + 0.02)


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
    bits: str


def main() -> int:
    ap = argparse.ArgumentParser(description="Auto-tune Table16 decode from 1800/2400 region")
    ap.add_argument("path", type=Path, help="Input .g711 (u-law)")
    ap.add_argument("--fs", type=int, default=8000)
    ap.add_argument("--start-ms", type=float, default=4800.0, help="Search active region from this time")
    ap.add_argument("--window-start-ms", type=float, default=None, help="Optional fixed demod window start (ms)")
    ap.add_argument("--window-end-ms", type=float, default=None, help="Optional fixed demod window end (ms)")
    ap.add_argument("--offset-step", type=float, default=0.25, help="Sample step for timing sweep")
    ap.add_argument("--carrier-span", type=float, default=3.0, help="+/- Hz around 2400")
    ap.add_argument("--carrier-step", type=float, default=1.0, help="Hz step")
    ap.add_argument("--rate-span", type=float, default=0.30, help="+/- bps around 600")
    ap.add_argument("--rate-step", type=float, default=0.15, help="bps step")
    ap.add_argument("--top", type=int, default=12, help="How many top candidates to print")
    args = ap.parse_args()

    x = ulaw_decode(args.path)
    t_s, n1800, n2400 = tone_metric_windows(x, args.fs)
    if args.window_start_ms is not None and args.window_end_ms is not None:
        t0 = args.window_start_ms / 1000.0
        t1 = args.window_end_ms / 1000.0
    else:
        region = find_active_region(t_s, n1800, n2400, args.start_ms)
        if region is None:
            raise SystemExit("could not find a clear 1800/2400 active region")
        t0, t1 = region

    s0 = max(0, int(t0 * args.fs))
    s1 = min(len(x), int(t1 * args.fs))
    seg = x[s0:s1]
    print(f"active_region={t0*1000:.1f}-{t1*1000:.1f} ms samples={len(seg)}")

    step = args.fs / 600.0
    offsets = np.arange(0.0, step, args.offset_step)
    carriers = np.arange(2400.0 - args.carrier_span, 2400.0 + args.carrier_span + 1e-9, args.carrier_step)
    rates = np.arange(600.0 - args.rate_span, 600.0 + args.rate_span + 1e-9, args.rate_step)

    best: list[Candidate] = []
    for off in offsets:
        for car in carriers:
            for rate in rates:
                for inv in (False, True):
                    bits = demod_bits(seg, args.fs, float(car), float(rate), float(off), inv)
                    if len(bits) < 80:
                        continue
                    frames = t16.find_table16_full_frames(bits)
                    if not frames:
                        continue
                    crc_hits = 0
                    for _, frame in frames:
                        d = t16.decode_table16_info0a(frame[12:45])
                        crc_hits += int(d["crc_match"])
                    full_hits = len(frames)
                    score = crc_hits * 100 + full_hits
                    cand = Candidate(
                        score=score,
                        crc_hits=crc_hits,
                        full_hits=full_hits,
                        off=float(off),
                        car=float(car),
                        rate=float(rate),
                        inv=inv,
                        frames=frames,
                        bits=bits,
                    )
                    best.append(cand)

    if not best:
        print("no full 49-bit Table16 matches found in sweep")
        return 0

    best.sort(key=lambda c: (c.score, c.crc_hits, c.full_hits), reverse=True)
    topn = best[: max(1, args.top)]
    print(f"candidates_tested_with_hits={len(best)}")
    for i, c in enumerate(topn, start=1):
        print(
            f"[{i}] score={c.score} crc_hits={c.crc_hits} full_hits={c.full_hits} "
            f"off={c.off:.3f} car={c.car:.3f} rate={c.rate:.3f} inv={int(c.inv)}"
        )
        for j, (bidx, frame) in enumerate(c.frames[:4], start=1):
            d = t16.decode_table16_info0a(frame[12:45])
            t_ms = (s0 + (c.off + (bidx + 1) * (args.fs / c.rate))) * 1000.0 / args.fs
            print(
                f"    frame{j}: bit={bidx} t~{t_ms:.2f}ms frame49={frame} "
                f"crc_field=0x{d['crc_field']:04X} crc_calc=0x{d['crc_calc']:04X} crc={d['crc_match']}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
