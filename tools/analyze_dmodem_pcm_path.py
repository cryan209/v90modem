#!/usr/bin/env python3
"""Compare a live G.711 TX tap with the PCM consumed by d-modem.

The d-modem bridge converts 8 kHz PCMU to the SmartLink DSP's 9.6 kHz
linear-PCM interface.  This tool reproduces that converter sample-for-sample,
finds V.90 Sd in the local codeword tap, and correlates the complete
Sd/S-bar/TRN1d transition against the remote DSP tap.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np


INPUT_FRAME = 160
OUTPUT_FRAME = 192
RS_TAPS = 12
RS_HALF = 5
RS_PHASES = 6


def ulaw_decode(codewords: bytes) -> np.ndarray:
    encoded = np.frombuffer(codewords, dtype=np.uint8)
    value = np.bitwise_not(encoded)
    magnitude = (((value & 0x0F).astype(np.int32) << 3) + 0x84)
    magnitude <<= (value >> 4) & 0x07
    magnitude -= 0x84
    magnitude[(value & 0x80) != 0] *= -1
    return magnitude.astype(np.int16)


def build_kernel() -> np.ndarray:
    kernel = np.empty((RS_PHASES, RS_TAPS), dtype=np.float32)
    cutoff = 3950.0 / (8000.0 / 2.0)

    for phase in range(RS_PHASES):
        fraction = phase / RS_PHASES
        for tap in range(RS_TAPS):
            x = (tap - RS_HALF) - fraction
            y = cutoff * x
            sinc = cutoff if abs(y) < 1.0e-9 else cutoff * math.sin(math.pi * y) / (math.pi * y)
            window = 0.5 - 0.5 * math.cos(2.0 * math.pi * tap / (RS_TAPS - 1))
            kernel[phase, tap] = np.float32(sinc * window)
        kernel[phase] /= np.float32(kernel[phase].sum())
    return kernel


def dmodem_resample(samples: np.ndarray) -> np.ndarray:
    """Reproduce rig/d-modem/d-modem.c, including its one-frame pipeline."""
    kernel = build_kernel()
    frame_count = len(samples) // INPUT_FRAME
    output = np.zeros(frame_count * OUTPUT_FRAME, dtype=np.int16)
    history = np.zeros(RS_TAPS, dtype=np.int16)
    previous: np.ndarray | None = None

    for frame_index in range(frame_count):
        current = samples[frame_index * INPUT_FRAME:(frame_index + 1) * INPUT_FRAME]
        if previous is not None:
            work = np.concatenate((history, previous, current[:RS_TAPS]))
            frame_output = output[frame_index * OUTPUT_FRAME:(frame_index + 1) * OUTPUT_FRAME]
            for k in range(OUTPUT_FRAME):
                numerator = k * 5
                center = numerator // 6
                phase = numerator % 6
                base = RS_TAPS + center - RS_HALF
                accumulator = np.float32(0.0)
                for tap in range(RS_TAPS):
                    accumulator = np.float32(
                        accumulator
                        + np.float32(work[base + tap]) * kernel[phase, tap]
                    )
                accumulator = np.clip(accumulator, -32768.0, 32767.0)
                frame_output[k] = int(
                    accumulator + np.float32(0.5)
                    if accumulator >= 0
                    else accumulator - np.float32(0.5)
                )
            history = previous[-RS_TAPS:].copy()
        previous = current.copy()
    return output


def sd_pattern(u_info: int) -> bytes:
    w_ucode = (16 + u_info) & 0x7F
    pos_w = 0xFF - w_ucode
    return bytes((pos_w | 0x80, 0xFF, pos_w | 0x80,
                  pos_w & 0x7F, 0x7F, pos_w & 0x7F))


def find_sd_starts(codewords: bytes, u_info: int) -> list[tuple[int, int]]:
    pattern = sd_pattern(u_info)
    needle = pattern * 4
    starts: list[tuple[int, int]] = []
    position = 0

    while True:
        position = codewords.find(needle, position)
        if position < 0:
            break
        if position < 6 or codewords[position - 6:position] != pattern:
            repetitions = 0
            while codewords[
                position + repetitions * 6:position + (repetitions + 1) * 6
            ] == pattern:
                repetitions += 1
            # S-bar is Sd shifted by three symbols.  Requiring a substantial
            # run excludes that seven-repetition alias and incidental matches.
            if repetitions >= 32:
                starts.append((position, repetitions))
        position += 6
    return starts


def best_match(template: np.ndarray, actual: np.ndarray, begin: int, end: int) -> tuple[int, float, float]:
    begin = max(0, begin)
    end = min(len(actual) - len(template), end)
    if begin > end:
        raise ValueError("correlation search window is outside the remote capture")

    search = actual[begin:end + len(template)].astype(np.float64)
    reference = template.astype(np.float64)
    dots = np.correlate(search, reference, mode="valid")
    energy = np.concatenate(([0.0], np.cumsum(search * search)))
    window_energy = energy[len(reference):] - energy[:-len(reference)]
    denominator = np.linalg.norm(reference) * np.sqrt(window_energy)
    scores = np.divide(dots, denominator, out=np.zeros_like(dots), where=denominator > 0)
    best_offset = int(np.argmax(scores))
    remote_start = begin + best_offset
    rms_ratio = float(
        np.sqrt(np.mean(actual[remote_start:remote_start + len(template)].astype(np.float64) ** 2))
        / np.sqrt(np.mean(reference ** 2))
    )
    return remote_start, float(scores[best_offset]), rms_ratio


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("local_g711", type=Path, help="local live-tx.g711")
    parser.add_argument("remote_pcm", type=Path, help="remote dm_to_dsp.raw (s16le/9600 Hz)")
    parser.add_argument(
        "--call-offset",
        type=int,
        default=0,
        help="8 kHz samples to skip before the call represented by remote_pcm",
    )
    parser.add_argument("--u-info", type=int, default=78)
    parser.add_argument("--search-ms", type=float, default=600.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    all_codewords = args.local_g711.read_bytes()
    if args.call_offset < 0 or args.call_offset >= len(all_codewords):
        raise SystemExit("--call-offset is outside the local G.711 tap")
    codewords = all_codewords[args.call_offset:]
    local = ulaw_decode(codewords)
    predicted = dmodem_resample(local)
    remote = np.fromfile(args.remote_pcm, dtype="<i2")
    starts = find_sd_starts(codewords, args.u_info)

    print(
        f"local={len(local)} samples ({len(local) / 8000.0:.3f} s) "
        f"predicted={len(predicted)} samples remote={len(remote)} samples "
        f"({len(remote) / 9600.0:.3f} s)"
    )
    if not starts:
        print(f"no exact Sd sequence found for U_INFO={args.u_info}")
        return 1

    search_samples = round(args.search_ms * 9.6)
    # Include the distinctive Sd, S-bar, TRN1d, and first 256 Jd symbols.
    template_input_samples = 64 * 6 + 8 * 6 + 2040 + 256
    template_output_samples = round(template_input_samples * 6 / 5)

    for number, (local_start, repetitions) in enumerate(starts, 1):
        # Input frame N is emitted as output frame N+1 by d-modem.
        predicted_start = round(local_start * 6 / 5) + OUTPUT_FRAME
        predicted_end = min(len(predicted), predicted_start + template_output_samples)
        template = predicted[predicted_start:predicted_end]
        remote_start, score, rms_ratio = best_match(
            template,
            remote,
            predicted_start - search_samples,
            predicted_start + search_samples,
        )
        delay = remote_start - predicted_start
        print(
            f"Sd#{number}: local={local_start / 8000.0:.6f} s reps={repetitions} "
            f"predicted={predicted_start / 9600.0:.6f} s "
            f"remote={remote_start / 9600.0:.6f} s "
            f"delay={delay} samples ({delay / 9.6:.3f} ms) "
            f"correlation={score:.6f} rms_ratio={rms_ratio:.4f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
