"""Reference V.32bis passband waveform generation."""

from __future__ import annotations

import math
import random
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


def impair_passband_gain(
    waveform: PassbandWaveform,
    *,
    gain: float,
) -> PassbandWaveform:
    """Scale passband amplitude by a fixed gain."""

    if gain <= 0.0:
        raise ValueError("gain must be positive")
    return PassbandWaveform(
        samples=[gain * sample for sample in waveform.samples],
        sample_rate=waveform.sample_rate,
        carrier_hz=waveform.carrier_hz,
    )


def impair_passband_awgn(
    waveform: PassbandWaveform,
    *,
    snr_db: float,
    seed: int,
) -> PassbandWaveform:
    """Add deterministic white Gaussian noise to a passband waveform."""

    if not waveform.samples:
        return waveform

    signal_power = sum(sample * sample for sample in waveform.samples) / len(waveform.samples)
    if signal_power <= 0.0:
        return waveform

    noise_power = signal_power / (10.0 ** (snr_db / 10.0))
    noise_stddev = math.sqrt(noise_power)
    rng = random.Random(seed)
    return PassbandWaveform(
        samples=[sample + rng.gauss(0.0, noise_stddev) for sample in waveform.samples],
        sample_rate=waveform.sample_rate,
        carrier_hz=waveform.carrier_hz,
    )


def impair_passband_carrier_drift(
    waveform: PassbandWaveform,
    *,
    drift_hz_per_sample: float,
) -> PassbandWaveform:
    """Apply a slow linear carrier drift to the real passband waveform."""

    if drift_hz_per_sample == 0.0:
        return waveform

    drifted: list[float] = []
    phase = 0.0
    for index, sample in enumerate(waveform.samples):
        instantaneous_offset = drift_hz_per_sample * index
        phase += 2.0 * math.pi * instantaneous_offset / waveform.sample_rate
        drifted.append(sample * math.cos(phase))

    return PassbandWaveform(
        samples=drifted,
        sample_rate=waveform.sample_rate,
        carrier_hz=waveform.carrier_hz,
    )
