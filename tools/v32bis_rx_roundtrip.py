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
from tools.v32bis_ref.receiver import V32bisLogicalReceiver
from tools.v32bis_ref.rx_frontend import (
    recover_startup_with_decision_directed_tracking,
    recover_symbols_ideal,
    recover_symbols_with_carrier_tracking,
    recover_symbols_with_frontend,
    recovered_to_metadata_free_observable_stream,
    recovered_to_observable_stream,
    recover_symbols_with_timing_loop,
    recover_symbols_with_timing_tracking,
    recover_symbols_with_timing_offset,
    recover_symbols_with_tracking,
    search_carrier_frequency,
    search_timing_and_carrier,
    search_symbol_timing,
)
from tools.v32bis_ref.startup import generate_answer_startup_trace
from tools.v32bis_ref.tx import startup_trace_to_complex_symbols
from tools.v32bis_ref.tx_passband import (
    baseband_to_passband,
    impair_passband_awgn,
    impair_passband_carrier_drift,
    impair_passband_echo,
    impair_passband_fir,
    impair_passband_gain,
)
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
    parser.add_argument("--channel-gain", type=float, default=1.0)
    parser.add_argument("--channel-snr-db", type=float)
    parser.add_argument("--channel-noise-seed", type=int, default=1)
    parser.add_argument("--channel-drift-hz-per-sample", type=float, default=0.0)
    parser.add_argument("--channel-fir", type=str)
    parser.add_argument("--channel-echo-delay", type=int, default=0)
    parser.add_argument("--channel-echo-gain", type=float, default=0.0)
    parser.add_argument("--rx-carrier-hz", type=float)
    parser.add_argument("--search-carrier", action="store_true")
    parser.add_argument("--search-both", action="store_true")
    parser.add_argument("--track-carrier", action="store_true")
    parser.add_argument("--track-timing", action="store_true")
    parser.add_argument("--track-both", action="store_true")
    parser.add_argument("--timing-loop", action="store_true")
    parser.add_argument("--carrier-search-span", type=float, default=10.0)
    parser.add_argument("--carrier-search-step", type=float, default=5.0)
    parser.add_argument("--carrier-phase-gain", type=float, default=0.2)
    parser.add_argument("--timing-offset", type=int, default=0)
    parser.add_argument("--search-timing", action="store_true")
    parser.add_argument("--timing-step", type=int, default=1)
    parser.add_argument("--timing-gain", type=float, default=0.02)
    parser.add_argument("--early-late-spacing", type=float, default=0.5)
    parser.add_argument("--decision-directed", action="store_true")
    parser.add_argument("--emit-events", action="store_true")
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
    if args.channel_gain != 1.0:
        passband = impair_passband_gain(passband, gain=args.channel_gain)
    if args.channel_snr_db is not None:
        passband = impair_passband_awgn(
            passband,
            snr_db=args.channel_snr_db,
            seed=args.channel_noise_seed,
        )
    if args.channel_drift_hz_per_sample != 0.0:
        passband = impair_passband_carrier_drift(
            passband,
            drift_hz_per_sample=args.channel_drift_hz_per_sample,
        )
    if args.channel_fir:
        taps = [float(part.strip()) for part in args.channel_fir.split(",") if part.strip()]
        passband = impair_passband_fir(passband, taps=taps)
    if args.channel_echo_delay > 0 and args.channel_echo_gain != 0.0:
        passband = impair_passband_echo(
            passband,
            delay_samples=args.channel_echo_delay,
            gain=args.channel_echo_gain,
        )
    if args.search_both:
        center = args.rx_carrier_hz if args.rx_carrier_hz is not None else args.carrier_hz
        candidates = []
        current = center - args.carrier_search_span
        while current <= center + args.carrier_search_span + 1e-9:
            candidates.append(round(current, 6))
            current += args.carrier_search_step
        search = search_timing_and_carrier(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=args.samples_per_symbol,
            carrier_candidates_hz=candidates,
        )
        print(
            f"# best_carrier_hz={search.carrier_hz:.6f} "
            f"best_offset={search.timing_offset} "
            f"metric={search.metric:.6f}"
        )
        recovered = search.recovered
    elif args.track_both:
        if args.decision_directed:
            tracking = recover_startup_with_decision_directed_tracking(
                passband,
                transmitted_symbols=transmitted,
                taps=baseband.taps,
                samples_per_symbol=args.samples_per_symbol,
                timing_offset=float(args.timing_offset),
                carrier_hz=args.rx_carrier_hz,
                phase_gain=args.carrier_phase_gain,
                timing_gain=args.timing_gain,
                early_late_spacing=args.early_late_spacing,
            )
            print(
                f"# selected_mode={tracking.mode} "
                f"events={','.join(tracking.event_names)} "
                f"metric={tracking.metric:.6f}"
            )
            recovered = tracking.recovered
        else:
            tracking = recover_symbols_with_tracking(
                passband,
                transmitted_symbols=transmitted,
                taps=baseband.taps,
                samples_per_symbol=args.samples_per_symbol,
                timing_offset=float(args.timing_offset),
                carrier_hz=args.rx_carrier_hz,
                phase_gain=args.carrier_phase_gain,
                timing_gain=args.timing_gain,
                early_late_spacing=args.early_late_spacing,
                decision_directed=args.decision_directed,
            )
            print(
                f"# final_phase_rad={tracking.final_phase_rad:.6f} "
                f"final_offset={tracking.final_offset} "
                f"metric={tracking.metric:.6f}"
            )
            recovered = tracking.recovered
    elif args.search_carrier:
        center = args.rx_carrier_hz if args.rx_carrier_hz is not None else args.carrier_hz
        candidates = []
        current = center - args.carrier_search_span
        while current <= center + args.carrier_search_span + 1e-9:
            candidates.append(round(current, 6))
            current += args.carrier_search_step
        search = search_carrier_frequency(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=args.samples_per_symbol,
            timing_offset=args.timing_offset,
            carrier_candidates_hz=candidates,
        )
        print(f"# best_carrier_hz={search.carrier_hz:.6f} metric={search.metric:.6f}")
        recovered = search.recovered
    elif args.track_carrier:
        tracking = recover_symbols_with_carrier_tracking(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=args.samples_per_symbol,
            timing_offset=args.timing_offset,
            carrier_hz=args.rx_carrier_hz,
            phase_gain=args.carrier_phase_gain,
            decision_directed=args.decision_directed,
        )
        print(
            f"# final_phase_rad={tracking.final_phase_rad:.6f} "
            f"metric={tracking.metric:.6f}"
        )
        recovered = tracking.recovered
    elif args.search_timing:
        search = search_symbol_timing(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=args.samples_per_symbol,
        )
        print(f"# best_offset={search.offset} metric={search.metric:.6f}")
        recovered = search.recovered
    elif args.timing_loop:
        tracking = recover_symbols_with_timing_loop(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=args.samples_per_symbol,
            timing_offset=float(args.timing_offset),
            carrier_hz=args.rx_carrier_hz,
            timing_gain=args.timing_gain,
            early_late_spacing=args.early_late_spacing,
            decision_directed=args.decision_directed,
        )
        print(f"# final_offset={tracking.final_offset:.6f} metric={tracking.metric:.6f}")
        recovered = tracking.recovered
    elif args.track_timing:
        tracking = recover_symbols_with_timing_tracking(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=args.samples_per_symbol,
            timing_offset=args.timing_offset,
            carrier_hz=args.rx_carrier_hz,
            timing_step=args.timing_step,
        )
        print(f"# final_offset={tracking.final_offset} metric={tracking.metric:.6f}")
        recovered = tracking.recovered
    elif args.timing_offset != 0:
        recovered = recover_symbols_with_timing_offset(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=args.samples_per_symbol,
            offset=args.timing_offset,
        )
    elif args.rx_carrier_hz is not None:
        recovered = recover_symbols_with_frontend(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=args.samples_per_symbol,
            carrier_hz=args.rx_carrier_hz,
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
    if args.emit_events:
        receiver = V32bisLogicalReceiver()
        events = receiver.ingest_all(recovered_to_metadata_free_observable_stream(recovered))
        print("# events")
        for event in events:
            parts = [event.name]
            if event.rate_mask is not None:
                parts.append(f"rate_mask=0x{event.rate_mask:04x}")
            if event.selected_rate is not None:
                parts.append(f"selected_rate={event.selected_rate}")
            if event.repetitions is not None:
                parts.append(f"repetitions={event.repetitions}")
            print(" ".join(parts))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
