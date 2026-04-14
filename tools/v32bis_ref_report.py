#!/usr/bin/env python3
"""Run and report a V.32bis logical startup scenario."""

from __future__ import annotations

import argparse
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.v32bis_ref.receiver import V32bisLogicalReceiver
from tools.v32bis_ref.startup import generate_answer_startup_trace
from tools.v32bis_ref.stream import (
    flatten_startup_trace,
    impair_stream,
    impair_stream_burst,
    impair_stream_insert,
    impair_stream_q_neighbor,
    impair_stream_random,
)
from tools.v32bis_ref.rate_signal import rate_mask_from_list


def parse_rates(text: str) -> int:
    rates = [int(part.strip()) for part in text.split(",") if part.strip()]
    return rate_mask_from_list(rates)


def format_events(events) -> list[str]:
    lines = []
    for event in events:
        parts = [event.name]
        if event.rate_mask is not None:
            parts.append(f"mask=0x{event.rate_mask:04x}")
        if event.selected_rate is not None:
            parts.append(f"rate={event.selected_rate}")
        if event.repetitions is not None:
            parts.append(f"reps={event.repetitions}")
        lines.append(" ".join(parts))
    return lines


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Report a V.32bis logical startup run")
    parser.add_argument("--answer-rates", default="4800,7200,9600,12000", help="comma-separated answerer supported rates")
    parser.add_argument("--r3-rates", default="4800,9600", help="comma-separated answerer-selected R3 rate set")
    parser.add_argument("--selected-rate", type=int, default=9600, help="final selected rate carried by E/B1")
    parser.add_argument("--r1-repetitions", type=int, default=2)
    parser.add_argument("--r3-repetitions", type=int, default=2)
    parser.add_argument("--impair-burst-start", type=int)
    parser.add_argument("--impair-burst-length", type=int, default=8)
    parser.add_argument("--impair-burst-symbol", default="X")
    parser.add_argument("--impair-random-prob", type=float)
    parser.add_argument("--impair-random-seed", type=int, default=1)
    parser.add_argument("--impair-q-neighbor", type=int)
    parser.add_argument("--impair-insert-index", type=int)
    parser.add_argument("--impair-insert-symbol", default="Q1")
    parser.add_argument("--impair-drop-index", type=int)
    return parser


def main(argv: list[str]) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    trace = generate_answer_startup_trace(
        r1_mask=parse_rates(args.answer_rates),
        r2_mask=parse_rates(args.r3_rates),
        r3_selected_rate=args.selected_rate,
        r1_repetitions=args.r1_repetitions,
        r3_repetitions=args.r3_repetitions,
    )
    stream = flatten_startup_trace(trace)

    if args.impair_burst_start is not None:
        stream = impair_stream_burst(
            stream,
            start=args.impair_burst_start,
            length=args.impair_burst_length,
            replacement=args.impair_burst_symbol,
        )
    if args.impair_random_prob is not None:
        stream = impair_stream_random(
            stream,
            flip_probability=args.impair_random_prob,
            seed=args.impair_random_seed,
        )
    if args.impair_q_neighbor is not None:
        stream = impair_stream_q_neighbor(stream, every_nth_q=args.impair_q_neighbor)
    if args.impair_insert_index is not None:
        stream = impair_stream_insert(stream, index=args.impair_insert_index, symbol=args.impair_insert_symbol)
    if args.impair_drop_index is not None:
        stream = impair_stream(stream, drops={args.impair_drop_index})

    receiver = V32bisLogicalReceiver()
    events = receiver.ingest_all(stream)
    stats = receiver.stats()

    print("Events:")
    for line in format_events(events):
        print(f"  {line}")

    print("Stats:")
    print(f"  q_candidates_tested={stats.q_candidates_tested}")
    print(f"  q_invalid_candidates={stats.q_invalid_candidates}")
    print(f"  q_resync_shifts={stats.q_resync_shifts}")
    print(f"  q_valid_words={stats.q_valid_words}")
    print(f"  r_words_detected={stats.r_words_detected}")
    print(f"  e_words_detected={stats.e_words_detected}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
