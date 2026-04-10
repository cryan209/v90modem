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


@dataclass
class Candidate:
    score: float
    start_ms: float
    carrier_hz: float
    sym_rate: float
    offset_samples: float
    inverted: int
    longest_run: int
    longest_at: int
    prefix24_at: int
    rep_match: int
    rep_total: int
    bits_preview: str


def demod_dbpsk_bits(x: np.ndarray,
                     fs: int,
                     start_ms: float,
                     offset_samples: float,
                     carrier_hz: float,
                     sym_rate: float,
                     bit_count: int) -> str:
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


def score_bits(bits: str, dil_len: int, prefix_ones: int) -> tuple[float, int, int, int, int, int]:
    run, run_at = longest_run_ones(bits)
    p = bits.find("1" * prefix_ones)
    rep_match = 0
    rep_total = 0
    if p >= 0:
        rem = bits[p + prefix_ones:]
        if len(rem) >= 2 * dil_len:
            b0 = rem[:dil_len]
            b1 = rem[dil_len:2 * dil_len]
            rep_total = dil_len
            rep_match = sum(1 for a, b in zip(b0, b1) if a == b)
    score = float(run) * 2.0
    if p >= 0:
        score += 200.0
        score += max(0.0, 100.0 - p * 0.2)
    if rep_total > 0:
        score += 200.0 * (rep_match / rep_total)
    return score, run, run_at, p, rep_match, rep_total


def frange(start: float, stop: float, step: float) -> list[float]:
    out = []
    v = start
    while v <= stop + 1e-12:
        out.append(round(v, 6))
        v += step
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Auto-search Ja DBPSK candidates")
    ap.add_argument("path", type=Path)
    ap.add_argument("--fs", type=int, default=8000)
    ap.add_argument("--start-ms", type=float, default=5725.0)
    ap.add_argument("--start-span-ms", type=float, default=40.0)
    ap.add_argument("--start-step-ms", type=float, default=5.0)
    ap.add_argument("--carrier-center", type=float, default=1800.0)
    ap.add_argument("--carrier-span", type=float, default=120.0)
    ap.add_argument("--carrier-step", type=float, default=10.0)
    ap.add_argument("--rates", type=str, default="2400,2743,2800,3000,3200,3429")
    ap.add_argument("--offset-min", type=float, default=0.0)
    ap.add_argument("--offset-max", type=float, default=2.0)
    ap.add_argument("--offset-step", type=float, default=0.5)
    ap.add_argument("--bits", type=int, default=4096)
    ap.add_argument("--prefix-ones", type=int, default=24)
    ap.add_argument("--dil-len", type=int, default=276)
    ap.add_argument("--top", type=int, default=20)
    args = ap.parse_args()

    raw = args.path.read_bytes()
    x = np.asarray([ulaw_byte_to_linear(b) for b in raw], dtype=np.float64)

    starts = frange(args.start_ms - args.start_span_ms, args.start_ms + args.start_span_ms, args.start_step_ms)
    carriers = frange(args.carrier_center - args.carrier_span, args.carrier_center + args.carrier_span, args.carrier_step)
    rates = [float(v.strip()) for v in args.rates.split(",") if v.strip()]
    offsets = frange(args.offset_min, args.offset_max, args.offset_step)

    cands: list[Candidate] = []
    for s in starts:
        for car in carriers:
            for rate in rates:
                for off in offsets:
                    bits = demod_dbpsk_bits(x, args.fs, s, off, car, rate, args.bits)
                    for inv in (0, 1):
                        b = bits if inv == 0 else "".join("0" if ch == "1" else "1" for ch in bits)
                        score, run, run_at, p, rep_m, rep_t = score_bits(b, args.dil_len, args.prefix_ones)
                        cands.append(Candidate(
                            score=score,
                            start_ms=s,
                            carrier_hz=car,
                            sym_rate=rate,
                            offset_samples=off,
                            inverted=inv,
                            longest_run=run,
                            longest_at=run_at,
                            prefix24_at=p,
                            rep_match=rep_m,
                            rep_total=rep_t,
                            bits_preview=b[:256],
                        ))

    cands.sort(key=lambda c: c.score, reverse=True)
    top = cands[: max(1, args.top)]

    print(
        "rank\tscore\tstart_ms\tcarrier\trate\toffset\tinv\tlongest1\tlongest_at\tprefix24_at\trep_match"
    )
    for i, c in enumerate(top, start=1):
        rep = f"{c.rep_match}/{c.rep_total}" if c.rep_total > 0 else "n/a"
        print(
            f"{i}\t{c.score:.1f}\t{c.start_ms:.1f}\t{c.carrier_hz:.1f}\t{c.sym_rate:.1f}\t"
            f"{c.offset_samples:.2f}\t{c.inverted}\t{c.longest_run}\t{c.longest_at}\t{c.prefix24_at}\t{rep}"
        )
    if top:
        print("best_bits_preview_256=" + top[0].bits_preview)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
