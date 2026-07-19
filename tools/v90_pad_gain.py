#!/usr/bin/env python3
"""Replicate the analogue modem's pad-gain search against our downstream PCM.

A V.90 analogue modem must recover the digital pad attenuation before it can
design a Phase 4 constellation.  It knows which Ucodes the digital modem was
asked to transmit (it authored the DIL descriptor), measures what actually
arrived, and fits a single gain g so that received ~= g * level(Ucode).
SmartLink's blob calls this findPadGain(); when no candidate gain fits,
V90TRN2Design() reports "constelation design failed" and the modem retrains --
which is the failure seen live against the d-modem rig (see rig/README.md).

This tool performs the same fit offline, so a per-Ucode residual can be
attributed to either the codewords we transmit or the transport in between.

Transports:

  ideal   straight DS0 -- what a real digital modem puts on the wire.  This is
          the control: it must return gain 1.0 with zero per-Ucode error.
  rig     the d-modem bridge's 8 kHz <-> 9.6 kHz conversion (rig/d-modem/
          d-modem.c), then a best-case receiver (perfect timing recovery and
          ideal band-limited interpolation back onto the 8 kHz symbol grid).

Generate an input with a known DIL using the truth-capture encoder:

    ./vpcm_encode --law ulaw --out /tmp/truth.g711     # DIL at ~7000-7560 ms
    tools/v90_pad_gain.py /tmp/truth.g711 --start-ms 7000 --end-ms 7560 \
        --transport rig

or point it at a live tap (VPCM_G711_TAP_DIR=... live-tx.g711) over the window
where DIL was transmitted.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

RS_TAPS = 257
RS_HALF = 128
RS_PHASES = 6
PIPELINE_DELAY_IN = 160  # d-modem emits input frame N as output frame N+1
INT16_MAX = 32767


def ulaw_decode(codewords: np.ndarray) -> np.ndarray:
    value = np.bitwise_not(codewords)
    magnitude = ((value & 0x0F).astype(np.int32) << 3) + 0x84
    magnitude <<= (value >> 4) & 0x07
    magnitude -= 0x84
    return np.where((value & 0x80) != 0, -magnitude, magnitude).astype(np.float64)


def ulaw_ucode(codewords: np.ndarray) -> np.ndarray:
    """Ucode (0..127) per codeword.

    V.90 carries the sign in the MSB, so the positive codeword for Ucode u is
    0xFF - u and the negative one is 0x7F - u (+0 = 0xFF and -0 = 0x7F are
    distinct codewords -- never round-trip these through linear WAV).
    """
    return np.where(codewords >= 0x80, 0xFF - codewords, 0x7F - codewords).astype(np.int32)


def ucode_levels() -> np.ndarray:
    """Ideal positive linear level for each Ucode 0..127."""
    return np.abs(ulaw_decode((0xFF - np.arange(128)).astype(np.uint8)))


def build_kernel() -> np.ndarray:
    """Exactly rig/d-modem/d-modem.c's windowed-sinc polyphase kernel."""
    kernel = np.empty((RS_PHASES, RS_TAPS), dtype=np.float64)
    cutoff = 4000.0 / (8000.0 / 2.0)
    for phase in range(RS_PHASES):
        fraction = phase / RS_PHASES
        for tap in range(RS_TAPS):
            x = (tap - RS_HALF) - fraction
            y = cutoff * x
            sinc = cutoff if abs(y) < 1.0e-9 else cutoff * math.sin(math.pi * y) / (math.pi * y)
            window = 0.5 - 0.5 * math.cos(2.0 * math.pi * tap / (RS_TAPS - 1))
            kernel[phase, tap] = sinc * window
        kernel[phase] /= kernel[phase].sum()
    return kernel


def dmodem_resample(samples: np.ndarray, kernel: np.ndarray) -> np.ndarray:
    """Vectorised d-modem 8 kHz -> 9.6 kHz converter, before the int16 write.

    Closed form of the reference frame loop in analyze_dmodem_pcm_path.py:
    output m is centred on input floor(m*5/6) - PIPELINE_DELAY_IN with
    polyphase phase (5*m) % 6.  Verified sample-exact against that reference.
    """
    out_len = (len(samples) // 160) * 192
    lead = PIPELINE_DELAY_IN + RS_HALF
    padded = np.concatenate((np.zeros(lead), samples, np.zeros(RS_TAPS + 400)))
    windows = np.lib.stride_tricks.sliding_window_view(padded, RS_TAPS)
    output = np.zeros(out_len)
    for residue in range(RS_PHASES):
        m = np.arange(residue, out_len, RS_PHASES)
        if not len(m):
            continue
        start = (m * 5) // 6 - RS_HALF + lead - PIPELINE_DELAY_IN
        valid = (start >= 0) & (start + RS_TAPS <= len(padded))
        output[m[valid]] = windows[start[valid]] @ kernel[(5 * residue) % 6]
    return output


def sinc_sample(signal: np.ndarray, positions: np.ndarray, half: int = 32) -> np.ndarray:
    """Ideal band-limited resampling at fractional positions (best-case Rx)."""
    base = np.floor(positions).astype(np.int64)
    frac = positions - base
    taps = np.arange(-half + 1, half + 1)
    x = frac[:, None] - taps[None, :]
    weights = np.sinc(x) * (0.5 + 0.5 * np.cos(np.pi * np.clip(x / half, -1, 1)))
    idx = np.clip(base[:, None] + taps[None, :], 0, len(signal) - 1)
    return (signal[idx] * weights).sum(axis=1)


def fit_pad_gain(ucode: np.ndarray, measured: np.ndarray, levels: np.ndarray):
    """Least-squares single-gain fit plus the per-Ucode residual profile."""
    present = np.unique(ucode)
    mean_measured = np.array([measured[ucode == u].mean() for u in present])
    counts = np.array([(ucode == u).sum() for u in present])
    ideal = levels[present]
    gain = float((mean_measured * ideal).sum() / (ideal * ideal).sum())
    predicted = gain * ideal
    rel = np.where(predicted > 0, (mean_measured - predicted) / np.maximum(predicted, 1e-12), 0.0)
    return present, counts, ideal, mean_measured, gain, rel


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("g711", type=Path, help="transmitted mu-law codeword stream")
    ap.add_argument("--start-ms", type=float, required=True, help="window start (DIL)")
    ap.add_argument("--end-ms", type=float, required=True, help="window end")
    ap.add_argument("--transport", choices=["ideal", "rig"], default="rig")
    ap.add_argument("--headroom", type=float, default=1.0,
                    help="pre-scale applied before the rig's int16 write; "
                         "<=0.45 keeps the interpolator from clipping")
    args = ap.parse_args()

    codewords = np.frombuffer(args.g711.read_bytes(), dtype=np.uint8)
    linear = ulaw_decode(codewords)
    ucode_all = ulaw_ucode(codewords)
    levels = ucode_levels()

    begin, end = int(args.start_ms * 8.0), int(args.end_ms * 8.0)
    if begin >= end or end > len(codewords):
        raise SystemExit(f"window {begin}..{end} outside the {len(codewords)}-codeword tap")

    print(f"{args.g711.name}: {len(codewords)} codewords, window "
          f"{args.start_ms:.1f}-{args.end_ms:.1f} ms, transport={args.transport}"
          + (f", headroom={args.headroom:g}" if args.transport == "rig" else ""))

    if args.transport == "ideal":
        recovered = np.abs(linear)
    else:
        kernel = build_kernel()
        raw = dmodem_resample(linear * args.headroom, kernel)
        clipped = int((np.abs(raw) > INT16_MAX).sum())
        active = int((np.abs(raw) > 1000).sum())
        print(f"  interpolator peak {np.abs(raw).max():.0f} "
              f"({np.abs(raw).max() / INT16_MAX:.2f}x int16 full scale); "
              f"clipped {clipped} samples "
              f"({100.0 * clipped / max(active, 1):.3f}% of active)")
        signal = np.clip(np.round(raw), -32768, INT16_MAX)
        positions = np.arange(len(linear)) * 6.0 / 5.0 + PIPELINE_DELAY_IN * 6.0 / 5.0
        recovered = np.abs(sinc_sample(signal, positions)) / args.headroom

    u, m = ucode_all[begin:end], recovered[begin:end]
    present, counts, ideal, meas, gain, rel = fit_pad_gain(u, m, levels)

    print(f"  Ucodes present: {len(present)}   symbols: {len(u)}")
    print(f"  best-fit pad gain: {gain:.6f}")
    print(f"  per-Ucode relative error: rms={np.sqrt((rel ** 2).mean()) * 100:.4f}%  "
          f"max={np.abs(rel).max() * 100:.3f}%")
    for lo, hi in [(0, 15), (16, 63), (64, 95), (96, 127)]:
        sel = (present >= lo) & (present <= hi)
        if sel.sum():
            print(f"    Ucode {lo:3d}-{hi:3d} (n={sel.sum():2d}): "
                  f"rms={np.sqrt((rel[sel] ** 2).mean()) * 100:7.4f}%  "
                  f"max={np.abs(rel[sel]).max() * 100:7.3f}%")

    print("\n   Ucode    n      ideal     measured    rel.err")
    for i in sorted(np.argsort(-np.abs(rel))[:10]):
        print(f"   {present[i]:5d} {counts[i]:5d} {ideal[i]:10.1f} {meas[i]:11.2f} "
              f"{rel[i] * 100:+9.3f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
