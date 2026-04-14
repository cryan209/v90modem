#!/usr/bin/env python3
"""Emit a V.32bis startup trace as pulse-shaped complex baseband samples."""

from __future__ import annotations

import argparse
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.v32bis_ref.rate_signal import rate_mask_from_list
from tools.v32bis_ref.startup import generate_answer_startup_trace, generate_call_startup_trace
from tools.v32bis_ref.tx import startup_trace_to_complex_symbols
from tools.v32bis_ref.tx_waveform import symbols_to_baseband


def parse_rates(text: str) -> int:
    rates = [int(part.strip()) for part in text.split(",") if part.strip()]
    return rate_mask_from_list(rates)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Emit V.32bis startup complex baseband samples")
    parser.add_argument("--role", choices=("answer", "call"), default="answer")
    parser.add_argument("--r1-rates", default="4800,7200,9600,12000")
    parser.add_argument("--r2-rates", default="4800,9600")
    parser.add_argument("--selected-rate", type=int, default=9600)
    parser.add_argument("--r1-repetitions", type=int, default=2)
    parser.add_argument("--r2-repetitions", type=int, default=2)
    parser.add_argument("--r3-repetitions", type=int, default=2)
    parser.add_argument("--trn-length", type=int, default=1280)
    parser.add_argument("--samples-per-symbol", type=int, default=10)
    parser.add_argument("--beta", type=float, default=0.5)
    parser.add_argument("--span-symbols", type=int, default=8)
    parser.add_argument("--limit", type=int, default=20)
    return parser


def main(argv: list[str]) -> int:
    args = build_parser().parse_args(argv)

    if args.role == "answer":
        trace = generate_answer_startup_trace(
            r1_mask=parse_rates(args.r1_rates),
            r2_mask=parse_rates(args.r2_rates),
            r3_selected_rate=args.selected_rate,
            trn_length=args.trn_length,
            r1_repetitions=args.r1_repetitions,
            r3_repetitions=args.r3_repetitions,
        )
    else:
        trace = generate_call_startup_trace(
            r1_mask=parse_rates(args.r1_rates),
            r2_mask=parse_rates(args.r2_rates),
            r3_selected_rate=args.selected_rate,
            trn_length=args.trn_length,
            r2_repetitions=args.r2_repetitions,
        )

    symbols = startup_trace_to_complex_symbols(trace)
    waveform = symbols_to_baseband(
        symbols,
        samples_per_symbol=args.samples_per_symbol,
        beta=args.beta,
        span_symbols=args.span_symbols,
    )

    for index, sample in enumerate(waveform.samples[: args.limit]):
        print(f"{index:05d} {sample.real:9.5f} {sample.imag:9.5f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
