#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
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


def analyze_carrier(seg: np.ndarray, fs: int, s0: int, f0: float) -> tuple[float, float, float, int] | None:
    sym_rate = 3200.0
    step = fs / sym_rate
    n_syms = int((len(seg) - 8) / step)
    if n_syms < 400:
        return None
    syms = np.zeros(n_syms, dtype=np.complex128)
    for n in range(n_syms):
        c = n * step
        acc = 0j
        for k in range(-2, 3):
            t = c + k
            s = interp(seg, t)
            acc += s * np.exp(-1j * 2.0 * math.pi * f0 * (t / fs))
        syms[n] = acc

    win = 128
    hop = 16
    times = []
    occ = []
    mags = []
    for i in range(0, max(1, n_syms - win), hop):
        wsy = syms[i:i + win]
        if len(wsy) < win:
            break
        ph = np.angle(wsy)
        bins = ((ph + math.pi) * (8.0 / (2.0 * math.pi))).astype(int) % 8
        h = np.bincount(bins, minlength=8)
        occ_bins = int(np.sum(h >= max(3, int(0.04 * win))))
        t_ms = (s0 + (i + win * 0.5) * step) * 1000.0 / fs
        times.append(t_ms)
        occ.append(occ_bins)
        mags.append(float(np.mean(np.abs(wsy))))
    times = np.asarray(times)
    occ = np.asarray(occ)
    mags = np.asarray(mags)
    if len(times) < 8:
        return None

    # TRN1u-like = sustained low phase occupancy.
    low = occ <= 3
    if not np.any(low):
        return None
    # pick longest contiguous low run
    best_i0 = -1
    best_i1 = -1
    i = 0
    while i < len(low):
        if not low[i]:
            i += 1
            continue
        j = i + 1
        while j < len(low) and low[j]:
            j += 1
        if (j - i) > (best_i1 - best_i0):
            best_i0, best_i1 = i, j
        i = j
    if best_i0 < 0:
        return None
    trn_ms = float(times[best_i0])
    run_len = best_i1 - best_i0

    ja_ms = -1.0
    for i in range(best_i1, len(occ) - 1):
        if occ[i] >= 6 and occ[i + 1] >= 6:
            ja_ms = float(times[i])
            break
    return trn_ms, ja_ms, float(mags[best_i0]), run_len


def detect(path: Path, fs: int, start_ms: float, end_ms: float) -> int:
    raw = path.read_bytes()
    x = np.asarray([ulaw_byte_to_linear(b) for b in raw], dtype=np.float64)
    s0 = max(0, int(start_ms * fs / 1000.0))
    s1 = min(len(x), int(end_ms * fs / 1000.0))
    if s1 - s0 < 2000:
        print("window too short")
        return 1
    seg = x[s0:s1]

    # Estimate dominant carrier in 1.6..2.8 kHz (for context only).
    nfft = 4096
    w = np.hanning(min(len(seg), nfft))
    z = np.fft.rfft(seg[: len(w)] * w, n=nfft)
    freqs = np.fft.rfftfreq(nfft, d=1.0 / fs)
    m = (freqs >= 1600.0) & (freqs <= 2800.0)
    f0 = float(freqs[m][np.argmax(np.abs(z[m]))])

    best = None
    for car in (1800.0, 2400.0, f0):
        r = analyze_carrier(seg, fs, s0, float(car))
        if r is None:
            continue
        trn_ms, ja_ms, mag, run_len = r
        # prefer longer TRN-like run and valid Ja transition
        score = run_len + (1000 if ja_ms > 0 else 0)
        cand = (score, car, trn_ms, ja_ms, mag, run_len)
        if best is None or cand[0] > best[0]:
            best = cand
    print(f"carrier_est_hz={f0:.1f}")
    if best is None:
        print("trn1u_candidate_ms=not_found")
        print("ja_candidate_ms=not_found")
        return 0
    _, car, trn_ms, ja_ms, mag, run_len = best
    print(f"analysis_carrier_hz={car:.1f}")
    print(f"trn1u_candidate_ms={trn_ms:.1f} run_windows={run_len} mag={mag:.1f}")
    if ja_ms > 0:
        print(f"ja_candidate_ms={ja_ms:.1f}")
    else:
        print("ja_candidate_ms=not_found")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", type=Path)
    ap.add_argument("--fs", type=int, default=8000)
    ap.add_argument("--start-ms", type=float, default=5200.0)
    ap.add_argument("--end-ms", type=float, default=9000.0)
    args = ap.parse_args()
    return detect(args.path, args.fs, args.start_ms, args.end_ms)


if __name__ == "__main__":
    raise SystemExit(main())
