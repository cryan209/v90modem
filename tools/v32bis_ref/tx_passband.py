"""Reference V.32bis passband waveform generation."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .tx_waveform import BasebandWaveform


@dataclass(frozen=True)
class PassbandWaveform:
    samples: list[float]
    sample_rate: int
    carrier_hz: float


def baseband_to_passband(
    waveform: BasebandWaveform,
    *,
    sample_rate: int,
    carrier_hz: float = 1800.0,
) -> PassbandWaveform:
    """Upconvert complex baseband samples to a real passband waveform."""

    if sample_rate <= 0:
        raise ValueError("sample_rate must be positive")

    samples: list[float] = []
    for index, sample in enumerate(waveform.samples):
        phase = 2.0 * math.pi * carrier_hz * index / sample_rate
        carrier = complex(math.cos(phase), math.sin(phase))
        upconverted = sample * carrier
        samples.append(upconverted.real)

    return PassbandWaveform(
        samples=samples,
        sample_rate=sample_rate,
        carrier_hz=carrier_hz,
    )
