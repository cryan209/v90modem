#!/usr/bin/env python3
"""Run the runtime-oriented V.32bis datapump startup skeleton."""

from __future__ import annotations

import argparse
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.v32bis_datapump import ChannelConfig, RxConfig, TxConfig, V32bisDatapump


def _parse_multi_echo(text: str | None) -> tuple[tuple[int, float], ...]:
    if not text:
        return ()
    paths: list[tuple[int, float]] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        delay_text, gain_text = part.split(":")
        paths.append((int(delay_text), float(gain_text)))
    return tuple(paths)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the V.32bis startup datapump skeleton")
    parser.add_argument("--channel-gain", type=float, default=1.0)
    parser.add_argument("--channel-snr-db", type=float)
    parser.add_argument("--channel-noise-seed", type=int, default=1)
    parser.add_argument("--channel-drift-hz-per-sample", type=float, default=0.0)
    parser.add_argument("--channel-fir", type=str)
    parser.add_argument("--channel-echo-delay", type=int, default=0)
    parser.add_argument("--channel-echo-gain", type=float, default=0.0)
    parser.add_argument("--channel-multi-echo", type=str)
    parser.add_argument("--near-end-echo", type=str)
    parser.add_argument("--cancel-near-end-echo", action="store_true")
    parser.add_argument("--near-end-echo-estimate", type=str)
    parser.add_argument("--adaptive-near-end-echo-cancel", action="store_true")
    parser.add_argument("--adaptive-echo-tap-count", type=int, default=16)
    parser.add_argument("--adaptive-echo-step-size", type=float, default=0.1)
    parser.add_argument("--blind-runtime", action="store_true")
    parser.add_argument("--rx-carrier-hz", type=float, default=1801.0)
    parser.add_argument("--timing-offset", type=float, default=1.0)
    parser.add_argument("--carrier-phase-gain", type=float, default=0.05)
    parser.add_argument("--timing-gain", type=float, default=0.005)
    return parser


def main(argv: list[str]) -> int:
    args = build_parser().parse_args(argv)
    fir_taps = tuple(float(part.strip()) for part in args.channel_fir.split(",") if part.strip()) if args.channel_fir else ()
    near_end_paths = _parse_multi_echo(args.near_end_echo)
    near_end_estimate = _parse_multi_echo(args.near_end_echo_estimate)
    datapump = V32bisDatapump(
        tx_config=TxConfig(),
        rx_config=RxConfig(
            rx_carrier_hz=args.rx_carrier_hz,
            timing_offset=args.timing_offset,
            phase_gain=args.carrier_phase_gain,
            timing_gain=args.timing_gain,
        ),
        channel_config=ChannelConfig(
            gain=args.channel_gain,
            snr_db=args.channel_snr_db,
            noise_seed=args.channel_noise_seed,
            drift_hz_per_sample=args.channel_drift_hz_per_sample,
            fir_taps=fir_taps,
            echo_delay=args.channel_echo_delay,
            echo_gain=args.channel_echo_gain,
            multi_echo_paths=_parse_multi_echo(args.channel_multi_echo),
            near_end_echo_paths=near_end_paths,
            cancel_near_end_echo=args.cancel_near_end_echo,
            near_end_echo_estimate_paths=near_end_estimate,
            adaptive_near_end_echo_cancel=args.adaptive_near_end_echo_cancel,
            adaptive_echo_tap_count=args.adaptive_echo_tap_count,
            adaptive_echo_step_size=args.adaptive_echo_step_size,
        ),
        blind_runtime=args.blind_runtime,
    )
    result = datapump.run_startup()
    print(f"# mode={result.recovery.mode} metric={result.recovery.metric:.6f}")
    print(f"# expected={','.join(result.oracle.expected)}")
    print(f"# actual={','.join(result.oracle.actual)}")
    print(f"# oracle_match={str(result.oracle.matches).lower()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
