#!/usr/bin/env python3
"""Sweep combined V.32bis startup channel impairments and report recovery."""

from __future__ import annotations

import argparse
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.v32bis_ref.rate_signal import rate_mask_from_list
from tools.v32bis_ref.rx_frontend import recover_startup_with_decision_directed_tracking
from tools.v32bis_ref.startup import generate_answer_startup_trace
from tools.v32bis_ref.tx import startup_trace_to_complex_symbols
from tools.v32bis_ref.tx_passband import (
    baseband_to_passband,
    impair_passband_awgn,
    impair_passband_carrier_drift,
    impair_passband_fir,
    impair_passband_gain,
)
from tools.v32bis_ref.tx_waveform import symbols_to_baseband


def parse_rates(text: str) -> int:
    rates = [int(part.strip()) for part in text.split(",") if part.strip()]
    return rate_mask_from_list(rates)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Sweep combined V.32bis startup channel impairments")
    parser.add_argument("--r1-rates", default="4800,7200,9600,12000")
    parser.add_argument("--r2-rates", default="4800,9600")
    parser.add_argument("--selected-rate", type=int, default=9600)
    parser.add_argument("--trn-length", type=int, default=1280)
    parser.add_argument("--samples-per-symbol", type=int, default=10)
    parser.add_argument("--sample-rate", type=int, default=24000)
    parser.add_argument("--carrier-hz", type=float, default=1800.0)
    parser.add_argument("--rx-carrier-hz", type=float, default=1801.0)
    parser.add_argument("--carrier-phase-gain", type=float, default=0.05)
    parser.add_argument("--timing-offset", type=float, default=1.0)
    parser.add_argument("--timing-gain", type=float, default=0.005)
    parser.add_argument("--early-late-spacing", type=float, default=0.5)
    parser.add_argument("--channel-gain", type=float, default=0.8)
    parser.add_argument("--channel-drift-hz-per-sample", type=float, default=1e-6)
    parser.add_argument("--channel-fir", type=str, default="0.9,0.25,-0.1")
    parser.add_argument("--snr-start", type=float, default=34.0)
    parser.add_argument("--snr-stop", type=float, default=16.0)
    parser.add_argument("--snr-step", type=float, default=2.0)
    parser.add_argument("--noise-seed", type=int, default=11)
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
    fir_taps = [float(part.strip()) for part in args.channel_fir.split(",") if part.strip()]

    print("# snr_db mode metric events")
    snr = args.snr_start
    while snr >= args.snr_stop - 1e-9:
        passband = baseband_to_passband(baseband, sample_rate=args.sample_rate, carrier_hz=args.carrier_hz)
        passband = impair_passband_gain(passband, gain=args.channel_gain)
        passband = impair_passband_awgn(passband, snr_db=snr, seed=args.noise_seed)
        passband = impair_passband_carrier_drift(passband, drift_hz_per_sample=args.channel_drift_hz_per_sample)
        passband = impair_passband_fir(passband, taps=fir_taps)
        tracked = recover_startup_with_decision_directed_tracking(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=args.samples_per_symbol,
            timing_offset=args.timing_offset,
            carrier_hz=args.rx_carrier_hz,
            phase_gain=args.carrier_phase_gain,
            timing_gain=args.timing_gain,
            early_late_spacing=args.early_late_spacing,
        )
        events = ",".join(tracked.event_names) if tracked.event_names else "-"
        print(f"{snr:6.1f} {tracked.mode:10} {tracked.metric:10.3f} {events}")
        snr -= args.snr_step

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
