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
    ap = argparse.ArgumentParser(description="Decode Ja/TRN bitstreams around an anchor")
    ap.add_argument("path", type=Path)
    ap.add_argument("--fs", type=int, default=8000)
    ap.add_argument("--carrier-hz", type=float, default=1800.0)
    ap.add_argument("--sym-rate", type=float, default=3200.0)
    ap.add_argument("--start-ms", type=float, required=True, help="Ja start anchor in ms")
    ap.add_argument("--mode", choices=("sign-diff", "carrier-qpsk", "carrier-dbpsk"), default="sign-diff")
    ap.add_argument("--symbols", type=int, default=0, help="Number of symbols to decode in carrier-qpsk mode (0=all)")
    ap.add_argument("--bits", type=int, default=0, help="Number of bits to decode in sign-diff mode (0=all)")
    ap.add_argument("--offset-samples", type=float, default=0.0, help="Extra symbol timing offset in samples")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    raw = args.path.read_bytes()
    x = np.asarray([ulaw_byte_to_linear(b) for b in raw], dtype=np.float64)
    start = args.start_ms * args.fs / 1000.0 + args.offset_samples

    out = []
    out.append(f"file={args.path}")
    out.append(
        f"start_ms={args.start_ms:.3f} mode={args.mode} carrier_hz={args.carrier_hz:.3f} "
        f"sym_rate={args.sym_rate:.3f} offset_samples={args.offset_samples:.3f}"
    )

    if args.mode in ("carrier-qpsk", "carrier-dbpsk"):
        step = args.fs / args.sym_rate
        max_syms = int(max(0, (len(x) - 2 - start) / step))
        n_syms = max_syms if args.symbols <= 0 else min(args.symbols, max_syms)

        syms = np.zeros(n_syms, dtype=np.complex128)
        for n in range(n_syms):
            c = start + n * step
            acc = 0j
            for k in range(-2, 3):
                t = c + k
                s = interp(x, t)
                acc += s * np.exp(-1j * 2.0 * math.pi * args.carrier_hz * (t / args.fs))
            syms[n] = acc

        out.append(f"symbols={n_syms}")
        if args.mode == "carrier-qpsk":
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

            out.append(f"state_symbols({len(states)}): {''.join(str(int(v)) for v in states)}")
            out.append(f"diff_symbols({len(d)}):  {''.join(str(int(v)) for v in d)}")
            out.append(f"nat_bits({len(nat_bits)}):")
            out.append(chunk(nat_bits, 64))
            out.append(f"gray_bits({len(gray_bits)}):")
            out.append(chunk(gray_bits, 64))
        else:
            if n_syms < 2:
                raise SystemExit("need at least 2 symbols for differential BPSK")
            dph = syms[1:] * np.conj(syms[:-1])
            # Differential BPSK bit: 1 for ~pi rotation (negative projection), 0 for ~0.
            dbits = "".join("1" if float(np.real(v)) < 0.0 else "0" for v in dph)
            dbits_inv = "".join("0" if b == "1" else "1" for b in dbits)
            rot = "".join("1" if float(np.imag(v)) >= 0.0 else "0" for v in dph)
            out.append(f"dbpsk_bits({len(dbits)}):")
            out.append(chunk(dbits, 128))
            out.append(f"dbpsk_bits_inverted({len(dbits_inv)}):")
            out.append(chunk(dbits_inv, 128))
            out.append(f"dbpsk_quadrant_imag_sign({len(rot)}):")
            out.append(chunk(rot, 128))
    else:
        s0 = int(round(start))
        if s0 < 1:
            s0 = 1
        signs = [1 if (b & 0x80) else 0 for b in raw]
        max_bits = len(signs) - s0
        bit_count = max_bits if args.bits <= 0 else min(args.bits, max_bits)
        if bit_count < 1:
            raise SystemExit("requested window is empty")

        raw_sign = ["1" if signs[s0 + i] else "0" for i in range(bit_count)]
        scr = []
        prev = int(signs[s0 - 1])
        for i in range(bit_count):
            cur = int(signs[s0 + i])
            b = cur ^ prev
            scr.append(b)
            prev = cur

        def descr(bits: list[int], tap: int) -> str:
            reg = 0
            out_bits = []
            for b in bits:
                o = (b ^ ((reg >> tap) & 1) ^ ((reg >> 22) & 1)) & 1
                reg = ((reg << 1) | b) & ((1 << 23) - 1)
                out_bits.append("1" if o else "0")
            return "".join(out_bits)

        scr_bits = "".join("1" if b else "0" for b in scr)
        descr4 = descr(scr, 4)
        descr17 = descr(scr, 17)

        out.append(f"bits={bit_count}")
        out.append(f"raw_sign_bits({len(raw_sign)}):")
        out.append(chunk("".join(raw_sign), 128))
        out.append(f"diff_scrambled_bits({len(scr_bits)}):")
        out.append(chunk(scr_bits, 128))
        out.append(f"descrambled_tap4_bits({len(descr4)}):")
        out.append(chunk(descr4, 128))
        out.append(f"descrambled_tap17_bits({len(descr17)}):")
        out.append(chunk(descr17, 128))
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
