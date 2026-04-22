#!/usr/bin/env python3
"""Generate and analyze simple V.32bis WAV fixtures.

This harness is meant to be a practical bridge between the current offline
reference/datapump stack and real captured audio. It supports:

- generating startup WAVs
- generating post-convergence data-mode WAVs
- analyzing startup WAVs with either oracle-assisted or blind recovery

It intentionally keeps scope narrow: mono PCM16 WAV only, and no resampling.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import wave
from pathlib import Path

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.v32bis_datapump.data import generate_data_waveform
from tools.v32bis_datapump.rx import RxConfig, recover_startup, recover_startup_blind
from tools.v32bis_datapump.tx import (
    TxConfig,
    generate_answer_startup_waveform,
    generate_call_startup_waveform,
)
from tools.v32bis_ref.tx_passband import PassbandWaveform


EXPECTED_STARTUP_EVENT_NAMES = ["S", "R1", "S", "R3", "E", "B1"]


def write_pcm16_wav(path: Path, waveform: PassbandWaveform) -> None:
    """Write a mono PCM16 WAV from a floating-point passband waveform."""

    peak = max((abs(sample) for sample in waveform.samples), default=0.0)
    scale = 32767.0 / peak if peak > 1.0 else 32767.0
    pcm = bytearray()
    for sample in waveform.samples:
        clipped = max(-32768, min(32767, int(round(sample * scale))))
        pcm.extend(int(clipped).to_bytes(2, byteorder="little", signed=True))

    with wave.open(str(path), "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(waveform.sample_rate)
        handle.writeframes(bytes(pcm))


def read_pcm16_wav(path: Path) -> PassbandWaveform:
    """Read a mono PCM16 WAV into a floating-point passband waveform."""

    with wave.open(str(path), "rb") as handle:
        channels = handle.getnchannels()
        width = handle.getsampwidth()
        sample_rate = handle.getframerate()
        frame_count = handle.getnframes()
        if channels != 1:
            raise ValueError(f"expected mono WAV, got {channels} channels")
        if width != 2:
            raise ValueError(f"expected 16-bit PCM WAV, got sample width {width}")
        raw = handle.readframes(frame_count)

    samples = []
    for index in range(0, len(raw), 2):
        value = int.from_bytes(raw[index:index + 2], byteorder="little", signed=True)
        samples.append(value / 32768.0)
    return PassbandWaveform(samples=samples, sample_rate=sample_rate, carrier_hz=1800.0)


def build_tx_config(args: argparse.Namespace) -> TxConfig:
    return TxConfig(
        r1_rates=tuple(args.r1_rates),
        r2_rates=tuple(args.r2_rates),
        selected_rate=args.selected_rate,
        trn_length=args.trn_length,
        samples_per_symbol=args.samples_per_symbol,
        sample_rate=args.sample_rate,
        carrier_hz=args.carrier_hz,
    )


def build_rx_config(args: argparse.Namespace) -> RxConfig:
    return RxConfig(
        samples_per_symbol=args.samples_per_symbol,
        rx_carrier_hz=args.rx_carrier_hz,
        timing_offset=args.timing_offset,
        phase_gain=args.phase_gain,
        timing_gain=args.timing_gain,
        early_late_spacing=args.early_late_spacing,
    )


def generate_startup_wav(path: Path, *, role: str, config: TxConfig) -> dict[str, object]:
    waveform = (
        generate_answer_startup_waveform(config)
        if role == "answer"
        else generate_call_startup_waveform(config)
    )
    write_pcm16_wav(path, waveform.passband)
    return {
        "mode": "generate-startup",
        "path": str(path),
        "role": role,
        "sample_rate": waveform.passband.sample_rate,
        "sample_count": len(waveform.passband.samples),
        "selected_rate": config.selected_rate,
    }


def generate_data_wav(
    path: Path,
    *,
    bit_rate: int,
    n_symbols: int,
    seed: int,
    calling_party: bool,
    config: TxConfig,
) -> dict[str, object]:
    waveform = generate_data_waveform(
        config,
        bit_rate=bit_rate,
        n_symbols=n_symbols,
        seed=seed,
        calling_party=calling_party,
    )
    write_pcm16_wav(path, waveform.passband)
    return {
        "mode": "generate-data",
        "path": str(path),
        "sample_rate": waveform.passband.sample_rate,
        "sample_count": len(waveform.passband.samples),
        "bit_rate": bit_rate,
        "n_symbols": n_symbols,
        "n_bits": len(waveform.tx_bits),
        "calling_party": calling_party,
    }


def analyze_startup_wav(
    path: Path,
    *,
    role: str,
    tx_config: TxConfig,
    rx_config: RxConfig,
    blind: bool,
    search_carrier_hz: float = 0.0,
    search_carrier_step_hz: float = 2.0,
    search_all_timing: bool = False,
) -> dict[str, object]:
    waveform = read_pcm16_wav(path)
    if waveform.sample_rate != tx_config.sample_rate:
        raise ValueError(
            f"WAV sample rate {waveform.sample_rate} does not match expected {tx_config.sample_rate}"
        )

    expected = (
        generate_answer_startup_waveform(tx_config)
        if role == "answer"
        else generate_call_startup_waveform(tx_config)
    )
    received = PassbandWaveform(
        samples=waveform.samples,
        sample_rate=waveform.sample_rate,
        carrier_hz=tx_config.carrier_hz,
    )
    if search_carrier_step_hz <= 0.0:
        raise ValueError("search_carrier_step_hz must be positive")

    def _event_score(event_names: list[str]) -> tuple[int, int, int]:
        expected_index = 0
        matched = 0
        for name in event_names:
            if expected_index < len(EXPECTED_STARTUP_EVENT_NAMES) and name == EXPECTED_STARTUP_EVENT_NAMES[expected_index]:
                matched += 1
                expected_index += 1
        extras = len(event_names) - matched
        exact = int(event_names == EXPECTED_STARTUP_EVENT_NAMES)
        return exact, matched, -extras

    carrier_candidates = [rx_config.rx_carrier_hz]
    if search_carrier_hz > 0.0:
        width_steps = int(round(search_carrier_hz / search_carrier_step_hz))
        carrier_candidates = [
            rx_config.rx_carrier_hz + step * search_carrier_step_hz
            for step in range(-width_steps, width_steps + 1)
        ]
    timing_candidates = (
        [float(offset) for offset in range(tx_config.samples_per_symbol)]
        if search_all_timing
        else [rx_config.timing_offset]
    )

    best_recovery = None
    best_rx_config = None
    tried_candidates: list[dict[str, object]] = []
    for carrier_hz in carrier_candidates:
        for timing_offset in timing_candidates:
            candidate_config = RxConfig(
                samples_per_symbol=rx_config.samples_per_symbol,
                rx_carrier_hz=carrier_hz,
                timing_offset=timing_offset,
                phase_gain=rx_config.phase_gain,
                timing_gain=rx_config.timing_gain,
                early_late_spacing=rx_config.early_late_spacing,
            )
            try:
                if blind:
                    recovery = recover_startup_blind(
                        received,
                        matched_filter_taps=expected.baseband.taps,
                        config=candidate_config,
                    )
                else:
                    recovery = recover_startup(
                        received,
                        transmitted_symbols=expected.transmitted_symbols,
                        matched_filter_taps=expected.baseband.taps,
                        config=candidate_config,
                    )
            except ValueError as exc:
                tried_candidates.append(
                    {
                        "rx_carrier_hz": carrier_hz,
                        "timing_offset": timing_offset,
                        "mode": "error",
                        "metric": float("inf"),
                        "event_count": -1,
                        "error": str(exc),
                    }
                )
                continue
            tried_candidates.append(
                {
                    "rx_carrier_hz": carrier_hz,
                    "timing_offset": timing_offset,
                    "mode": recovery.mode,
                    "metric": recovery.metric,
                    "event_count": len(recovery.events),
                    "event_score": _event_score([event.name for event in recovery.events]),
                }
            )
            if best_recovery is None or (
                _event_score([event.name for event in recovery.events]),
                -recovery.metric,
            ) > (
                _event_score([event.name for event in best_recovery.events]),
                -best_recovery.metric,
            ):
                best_recovery = recovery
                best_rx_config = candidate_config

    if best_recovery is None or best_rx_config is None:
        raise RuntimeError("no valid startup recovery candidates were found")
    return {
        "mode": "analyze-startup",
        "path": str(path),
        "role": role,
        "blind": blind,
        "recovery_mode": best_recovery.mode,
        "metric": best_recovery.metric,
        "selected_rx_carrier_hz": best_rx_config.rx_carrier_hz,
        "selected_timing_offset": best_rx_config.timing_offset,
        "candidate_count": len(tried_candidates),
        "top_candidates": sorted(
            tried_candidates,
            key=lambda candidate: (candidate.get("event_score", (-1, -1, -999999)), -candidate["metric"]),
            reverse=True,
        )[:5],
        "events": [
            {
                "name": event.name,
                "rate_mask": event.rate_mask,
                "selected_rate": event.selected_rate,
                "repetitions": event.repetitions,
            }
            for event in best_recovery.events
        ],
    }


def _parse_rates(text: str) -> tuple[int, ...]:
    return tuple(int(part.strip()) for part in text.split(",") if part.strip())


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate and analyze simple V.32bis WAV fixtures")
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_common_tx_options(subparser: argparse.ArgumentParser) -> None:
        subparser.add_argument("--r1-rates", type=_parse_rates, default=(4800, 7200, 9600, 12000))
        subparser.add_argument("--r2-rates", type=_parse_rates, default=(4800, 9600))
        subparser.add_argument("--selected-rate", type=int, default=9600)
        subparser.add_argument("--trn-length", type=int, default=1280)
        subparser.add_argument("--samples-per-symbol", type=int, default=10)
        subparser.add_argument("--sample-rate", type=int, default=24000)
        subparser.add_argument("--carrier-hz", type=float, default=1800.0)

    gen_startup = subparsers.add_parser("generate-startup", help="write a startup WAV")
    gen_startup.add_argument("--output", type=Path, required=True)
    gen_startup.add_argument("--role", choices=("answer", "call"), default="answer")
    add_common_tx_options(gen_startup)

    gen_data = subparsers.add_parser("generate-data", help="write a data-mode WAV")
    gen_data.add_argument("--output", type=Path, required=True)
    gen_data.add_argument("--bit-rate", type=int, default=9600)
    gen_data.add_argument("--n-symbols", type=int, default=512)
    gen_data.add_argument("--seed", type=int, default=42)
    gen_data.add_argument("--calling-party", action="store_true")
    add_common_tx_options(gen_data)

    analyze = subparsers.add_parser("analyze-startup", help="recover startup events from a WAV")
    analyze.add_argument("--input", type=Path, required=True)
    analyze.add_argument("--role", choices=("answer", "call"), default="answer")
    analyze.add_argument("--blind", action="store_true")
    add_common_tx_options(analyze)
    analyze.add_argument("--rx-carrier-hz", type=float, default=1801.0)
    analyze.add_argument("--timing-offset", type=float, default=1.0)
    analyze.add_argument("--phase-gain", type=float, default=0.05)
    analyze.add_argument("--timing-gain", type=float, default=0.005)
    analyze.add_argument("--early-late-spacing", type=float, default=0.5)
    analyze.add_argument("--search-carrier-hz", type=float, default=0.0)
    analyze.add_argument("--search-carrier-step-hz", type=float, default=2.0)
    analyze.add_argument("--search-all-timing", action="store_true")

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.command == "generate-startup":
        result = generate_startup_wav(args.output, role=args.role, config=build_tx_config(args))
    elif args.command == "generate-data":
        result = generate_data_wav(
            args.output,
            bit_rate=args.bit_rate,
            n_symbols=args.n_symbols,
            seed=args.seed,
            calling_party=args.calling_party,
            config=build_tx_config(args),
        )
    else:
        result = analyze_startup_wav(
            args.input,
            role=args.role,
            tx_config=build_tx_config(args),
            rx_config=build_rx_config(args),
            blind=args.blind,
            search_carrier_hz=args.search_carrier_hz,
            search_carrier_step_hz=args.search_carrier_step_hz,
            search_all_timing=args.search_all_timing,
        )

    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
