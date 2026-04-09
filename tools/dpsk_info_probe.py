#!/usr/bin/env python3
"""
Probe likely INFO sequences in a G.711 u-law capture using a simple 2400 Hz
DBPSK demodulator. This is intentionally lightweight and dependency-free.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path


def ulaw_byte_to_linear(u: int) -> int:
    u = (~u) & 0xFF
    sign = u & 0x80
    exponent = (u >> 4) & 0x07
    mantissa = u & 0x0F
    sample = ((mantissa << 3) + 0x84) << exponent
    sample -= 0x84
    return -sample if sign else sample


def interp(xs: list[int], t: float) -> float:
    if t < 0 or t >= len(xs) - 1:
        return 0.0
    i = int(t)
    f = t - i
    return xs[i] * (1.0 - f) + xs[i + 1] * f


def demod_bits(
    samples: list[int],
    fs: int,
    carrier_hz: float,
    rate_bps: float,
    offset_samples: float,
    invert_diff: bool = False,
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
            t = center + k
            x = interp(samples, t)
            ang = -2.0 * math.pi * carrier_hz * (t / fs)
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


def find_all(haystack: str, needle: str) -> list[int]:
    out: list[int] = []
    pos = 0
    while True:
        idx = haystack.find(needle, pos)
        if idx < 0:
            return out
        out.append(idx)
        pos = idx + 1


def parse_offsets(text: str) -> list[float]:
    parts = [p.strip() for p in text.split(",") if p.strip()]
    return [float(p) for p in parts]


def chunk_bits(bits: str, width: int) -> list[str]:
    out: list[str] = []
    for i in range(0, len(bits), width):
        out.append(bits[i:i + width])
    return out


def dump_all_bits(bits: str, width: int = 64) -> str:
    lines: list[str] = []
    for i, chunk in enumerate(chunk_bits(bits, width)):
        lo = i * width
        hi = lo + len(chunk) - 1
        lines.append(f"{lo:04d}:{hi:04d} {chunk}")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="Probe 2400 Hz DBPSK INFO candidates")
    ap.add_argument("path", type=Path, help="Input .g711 (u-law) file")
    ap.add_argument("--start-ms", type=float, default=4950.0)
    ap.add_argument("--end-ms", type=float, default=5300.0)
    ap.add_argument("--fs", type=int, default=8000)
    ap.add_argument("--carrier-hz", type=float, default=2400.0)
    ap.add_argument("--rate-bps", type=float, default=600.0)
    ap.add_argument("--offsets", type=str, default="2.375,7.625")
    ap.add_argument(
        "--dump-dir",
        type=Path,
        default=None,
        help="Optional directory to write full decoded bit dumps",
    )
    ap.add_argument(
        "--dump-width",
        type=int,
        default=64,
        help="Bits per line in full dumps",
    )
    args = ap.parse_args()

    raw = args.path.read_bytes()
    samples = [ulaw_byte_to_linear(b) for b in raw]

    s0 = int(args.start_ms * args.fs / 1000.0)
    s1 = int(args.end_ms * args.fs / 1000.0)
    if s0 < 0:
        s0 = 0
    if s1 > len(samples):
        s1 = len(samples)
    if s1 <= s0:
        raise SystemExit("empty analysis window")
    seg = samples[s0:s1]

    pattern_372 = format(0x372, "010b")
    pattern_4ef = format(0x4EF, "011b")

    print(
        f"window={args.start_ms:.1f}-{args.end_ms:.1f}ms "
        f"samples={len(seg)} carrier={args.carrier_hz:.1f}Hz rate={args.rate_bps:.3f}bps"
    )

    if args.dump_dir is not None:
        args.dump_dir.mkdir(parents=True, exist_ok=True)

    for off in parse_offsets(args.offsets):
        for inv in (False, True):
            bits = demod_bits(
                seg,
                args.fs,
                args.carrier_hz,
                args.rate_bps,
                off,
                invert_diff=inv,
            )
            if not bits:
                continue
            hits_372 = find_all(bits, pattern_372)
            hits_4ef = find_all(bits, pattern_4ef)
            if not hits_372 and not hits_4ef:
                continue

            label = "inv" if inv else "norm"
            print(f"\noffset={off:.3f} mode={label} bits={len(bits)}")
            print(f"  0x372 hits: {hits_372}")
            print(f"  0x4EF hits: {hits_4ef}")

            if args.dump_dir is not None:
                out_path = args.dump_dir / f"bits_off_{off:.3f}_{label}.txt"
                out_path.write_text(
                    dump_all_bits(bits, width=args.dump_width) + "\n",
                    encoding="ascii",
                )
                print(f"  full bit dump: {out_path}")

            for hit in hits_372[:4]:
                t_ms = args.start_ms + (off + (hit + 1) * (args.fs / args.rate_bps)) * 1000.0 / args.fs
                print(f"  hit@bit={hit} t~{t_ms:.2f}ms")
                for plen in (33, 46, 54, 93):
                    lo = hit + 10
                    hi = lo + plen
                    if hi <= len(bits):
                        print(f"    payload{plen}: {bits[lo:hi]}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
