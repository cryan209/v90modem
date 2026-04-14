"""Reference V.32bis baseband waveform generation."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .tx import TransmittedSymbol


def rrc_taps(beta: float, samples_per_symbol: int, span_symbols: int) -> list[float]:
    """Generate unit-energy root-raised-cosine taps."""

    if not 0.0 <= beta <= 1.0:
        raise ValueError("beta must be in range 0..1")
    if samples_per_symbol <= 0:
        raise ValueError("samples_per_symbol must be positive")
    if span_symbols <= 0:
        raise ValueError("span_symbols must be positive")

    ntaps = span_symbols * samples_per_symbol + 1
    center = ntaps // 2
    taps: list[float] = []

    for n in range(ntaps):
        t = (n - center) / samples_per_symbol
        if abs(t) < 1e-12:
            value = 1.0 + beta * ((4.0 / math.pi) - 1.0)
        elif beta > 0.0 and abs(abs(t) - 1.0 / (4.0 * beta)) < 1e-12:
            term1 = (1.0 + 2.0 / math.pi) * math.sin(math.pi / (4.0 * beta))
            term2 = (1.0 - 2.0 / math.pi) * math.cos(math.pi / (4.0 * beta))
            value = (beta / math.sqrt(2.0)) * (term1 + term2)
        else:
            numerator = (
                math.sin(math.pi * t * (1.0 - beta))
                + 4.0 * beta * t * math.cos(math.pi * t * (1.0 + beta))
            )
            denominator = math.pi * t * (1.0 - (4.0 * beta * t) ** 2)
            value = numerator / denominator
        taps.append(value)

    energy = math.sqrt(sum(tap * tap for tap in taps))
    return [tap / energy for tap in taps]


@dataclass(frozen=True)
class BasebandWaveform:
    samples: list[complex]
    taps: list[float]
    samples_per_symbol: int


def symbols_to_baseband(
    symbols: list[TransmittedSymbol],
    *,
    samples_per_symbol: int = 10,
    beta: float = 0.5,
    span_symbols: int = 8,
) -> BasebandWaveform:
    """Pulse-shape symbol-domain points into complex baseband samples."""

    taps = rrc_taps(beta, samples_per_symbol, span_symbols)
    upsampled_len = len(symbols) * samples_per_symbol
    upsampled = [0j] * upsampled_len
    for index, symbol in enumerate(symbols):
        upsampled[index * samples_per_symbol] = symbol.point

    samples = [0j] * (upsampled_len + len(taps) - 1)
    for i, sample in enumerate(upsampled):
        if sample == 0j:
            continue
        for j, tap in enumerate(taps):
            samples[i + j] += sample * tap

    return BasebandWaveform(
        samples=samples,
        taps=taps,
        samples_per_symbol=samples_per_symbol,
    )
