#!/usr/bin/env python3
"""Run a clean TX->RX round trip for V.32bis startup symbols."""

from __future__ import annotations

import argparse
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.v32bis_ref.rate_signal import rate_mask_from_list
from tools.v32bis_ref.rx_frontend import recover_symbols_ideal, recover_symbols_with_timing_offset, search_symbol_timing
from tools.v32bis_ref.startup import generate_answer_startup_trace
from tools.v32bis_ref.tx import startup_trace_to_complex_symbols
from tools.v32bis_ref.tx_passband import baseband_to_passband
from tools.v32bis_ref.tx_waveform import symbols_to_baseband


def parse_rates(text: str) -> int:
    rates = [int(part.strip()) for part in text.split(",") if part.strip()]
    return rate_mask_from_list(rates)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a clean V.32bis startup TX/RX round trip")
    parser.add_argument("--r1-rates", default="4800,7200,9600,12000")
    parser.add_argument("--r2-rates", default="4800,9600")
    parser.add_argument("--selected-rate", type=int, default=9600)
    parser.add_argument("--trn-length", type=int, default=1280)
    parser.add_argument("--samples-per-symbol", type=int, default=10)
    parser.add_argument("--sample-rate", type=int, default=24000)
    parser.add_argument("--carrier-hz", type=float, default=1800.0)
    parser.add_argument("--timing-offset", type=int, default=0)
    parser.add_argument("--search-timing", action="store_true")
    parser.add_argument("--limit", type=int, default=20)
    return parser


def main(argv: list[str]) -> int:
    args = build_parser().parse_args(argv)

    trace = generate_answer_startup_trace(
        r1_mask=parse_rates(args.r1_rates),
        r2_mask=parse_rates(args.r2_rates),
        r3_selected_rate=args.selected_rate,
        trn_length=args.trn_length,
    )
    transmitted = startup_trace_to_complex_symbols(trace)
    baseband = symbols_to_baseband(transmitted, samples_per_symbol=args.samples_per_symbol)
    passband = baseband_to_passband(baseband, sample_rate=args.sample_rate, carrier_hz=args.carrier_hz)
    if args.search_timing:
        search = search_symbol_timing(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=args.samples_per_symbol,
        )
        print(f"# best_offset={search.offset} metric={search.metric:.6f}")
        recovered = search.recovered
    elif args.timing_offset != 0:
        recovered = recover_symbols_with_timing_offset(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=args.samples_per_symbol,
            offset=args.timing_offset,
        )
    else:
        recovered = recover_symbols_ideal(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=args.samples_per_symbol,
        )

    for index, symbol in enumerate(recovered[: args.limit]):
        print(
            f"{index:05d} "
            f"{symbol.source_name:>12} "
            f"{symbol.expected_symbol:>3} "
            f"{symbol.decided_symbol:>3} "
            f"{symbol.point.real:8.3f} "
            f"{symbol.point.imag:8.3f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
