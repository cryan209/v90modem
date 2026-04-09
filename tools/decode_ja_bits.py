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


def chunk(s: str, n: int) -> str:
    return " ".join(s[i:i + n] for i in range(0, len(s), n))


def main() -> int:
    ap = argparse.ArgumentParser(description="Decode raw Ja-region symbol bits around an anchor")
    ap.add_argument("path", type=Path)
    ap.add_argument("--fs", type=int, default=8000)
    ap.add_argument("--carrier-hz", type=float, default=1800.0)
    ap.add_argument("--sym-rate", type=float, default=3200.0)
    ap.add_argument("--start-ms", type=float, required=True, help="Ja start anchor in ms")
    ap.add_argument("--symbols", type=int, default=256, help="Number of symbols to decode")
    ap.add_argument("--offset-samples", type=float, default=0.0, help="Extra symbol timing offset in samples")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    raw = args.path.read_bytes()
    x = np.asarray([ulaw_byte_to_linear(b) for b in raw], dtype=np.float64)
    step = args.fs / args.sym_rate
    start = args.start_ms * args.fs / 1000.0 + args.offset_samples

    syms = np.zeros(args.symbols, dtype=np.complex128)
    for n in range(args.symbols):
        c = start + n * step
        acc = 0j
        for k in range(-2, 3):
            t = c + k
            s = interp(x, t)
            acc += s * np.exp(-1j * 2.0 * math.pi * args.carrier_hz * (t / args.fs))
        syms[n] = acc

    # Quantize to 4-phase states (QPSK-like). This is a raw symbol view.
    phases = np.angle(syms)
    states = (((phases + math.pi) / (math.pi / 2.0)).astype(int)) & 3

    # Differential symbol transitions (mod-4); useful for unknown absolute phase.
    d = (states[1:] - states[:-1]) & 3

    # Two representations for each diff symbol.
    # Natural: 0->00 1->01 2->10 3->11
    nat_bits = "".join(f"{int(v):02b}" for v in d)
    # Gray:    0->00 1->01 2->11 3->10
    gray_map = {0: "00", 1: "01", 2: "11", 3: "10"}
    gray_bits = "".join(gray_map[int(v)] for v in d)

    out = []
    out.append(f"file={args.path}")
    out.append(
        f"start_ms={args.start_ms:.3f} carrier_hz={args.carrier_hz:.3f} "
        f"sym_rate={args.sym_rate:.3f} symbols={args.symbols} offset_samples={args.offset_samples:.3f}"
    )
    out.append(f"state_symbols({len(states)}): {''.join(str(int(v)) for v in states)}")
    out.append(f"diff_symbols({len(d)}):  {''.join(str(int(v)) for v in d)}")
    out.append(f"nat_bits({len(nat_bits)}):")
    out.append(chunk(nat_bits, 64))
    out.append(f"gray_bits({len(gray_bits)}):")
    out.append(chunk(gray_bits, 64))
    txt = "\n".join(out) + "\n"

    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(txt, encoding="ascii")
        print(args.out)
    else:
        print(txt, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

