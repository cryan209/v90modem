"""Runtime channel helpers for the V.32bis datapump."""

from __future__ import annotations

from dataclasses import dataclass, field

from tools.v32bis_ref.tx_passband import (
    PassbandWaveform,
    impair_passband_awgn,
    impair_passband_carrier_drift,
    impair_passband_echo,
    impair_passband_fir,
    impair_passband_gain,
    impair_passband_multi_echo,
)


@dataclass(frozen=True)
class ChannelConfig:
    gain: float = 1.0
    snr_db: float | None = None
    noise_seed: int = 1
    drift_hz_per_sample: float = 0.0
    fir_taps: tuple[float, ...] = ()
    echo_delay: int = 0
    echo_gain: float = 0.0
    multi_echo_paths: tuple[tuple[int, float], ...] = field(default_factory=tuple)
    near_end_echo_paths: tuple[tuple[int, float], ...] = field(default_factory=tuple)
    cancel_near_end_echo: bool = False
    near_end_echo_estimate_paths: tuple[tuple[int, float], ...] = field(default_factory=tuple)
    adaptive_near_end_echo_cancel: bool = False
    adaptive_echo_tap_count: int = 16
    adaptive_echo_step_size: float = 0.1


def apply_channel(waveform: PassbandWaveform, config: ChannelConfig) -> PassbandWaveform:
    """Apply configured channel impairments to a passband waveform."""

    impaired = waveform
    if config.gain != 1.0:
        impaired = impair_passband_gain(impaired, gain=config.gain)
    if config.snr_db is not None:
        impaired = impair_passband_awgn(impaired, snr_db=config.snr_db, seed=config.noise_seed)
    if config.drift_hz_per_sample != 0.0:
        impaired = impair_passband_carrier_drift(
            impaired,
            drift_hz_per_sample=config.drift_hz_per_sample,
        )
    if config.fir_taps:
        impaired = impair_passband_fir(impaired, taps=list(config.fir_taps))
    if config.echo_delay > 0 and config.echo_gain != 0.0:
        impaired = impair_passband_echo(
            impaired,
            delay_samples=config.echo_delay,
            gain=config.echo_gain,
        )
    if config.multi_echo_paths:
        impaired = impair_passband_multi_echo(impaired, paths=list(config.multi_echo_paths))
    return impaired


def synthesize_near_end_echo(
    local_waveform: PassbandWaveform,
    *,
    paths: tuple[tuple[int, float], ...],
) -> PassbandWaveform:
    """Build a near-end leakage waveform from the local transmitter."""

    return impair_passband_multi_echo(local_waveform, paths=list(paths))


def mix_passbands(left: PassbandWaveform, right: PassbandWaveform) -> PassbandWaveform:
    """Sum two passband waveforms."""

    length = max(len(left.samples), len(right.samples))
    samples = [0.0] * length
    for index in range(length):
        if index < len(left.samples):
            samples[index] += left.samples[index]
        if index < len(right.samples):
            samples[index] += right.samples[index]
    return PassbandWaveform(
        samples=samples,
        sample_rate=left.sample_rate,
        carrier_hz=left.carrier_hz,
    )


def subtract_passbands(left: PassbandWaveform, right: PassbandWaveform) -> PassbandWaveform:
    """Subtract one passband waveform from another."""

    length = max(len(left.samples), len(right.samples))
    samples = [0.0] * length
    for index in range(length):
        if index < len(left.samples):
            samples[index] += left.samples[index]
        if index < len(right.samples):
            samples[index] -= right.samples[index]
    return PassbandWaveform(
        samples=samples,
        sample_rate=left.sample_rate,
        carrier_hz=left.carrier_hz,
    )


def adaptive_cancel_near_end_echo(
    received_waveform: PassbandWaveform,
    local_waveform: PassbandWaveform,
    *,
    tap_count: int,
    step_size: float,
) -> tuple[PassbandWaveform, list[float]]:
    """Estimate and cancel near-end echo with a block least-squares filter."""

    if tap_count < 1:
        raise ValueError("tap_count must be positive")
    if step_size <= 0.0:
        raise ValueError("step_size must be positive")

    length = max(len(received_waveform.samples), len(local_waveform.samples))
    received = list(received_waveform.samples) + [0.0] * (length - len(received_waveform.samples))
    local = list(local_waveform.samples) + [0.0] * (length - len(local_waveform.samples))
    correlation = [[0.0 for _ in range(tap_count)] for _ in range(tap_count)]
    projection = [0.0 for _ in range(tap_count)]

    for index in range(length):
        window = [local[index - tap] if index - tap >= 0 else 0.0 for tap in range(tap_count)]
        for row in range(tap_count):
            projection[row] += received[index] * window[row]
            for col in range(tap_count):
                correlation[row][col] += window[row] * window[col]

    regularization = step_size
    for row in range(tap_count):
        correlation[row][row] += regularization

    taps = _solve_linear_system(correlation, projection)
    cancelled = [0.0] * length
    for index in range(length):
        window = [local[index - tap] if index - tap >= 0 else 0.0 for tap in range(tap_count)]
        estimate = sum(tap * sample for tap, sample in zip(taps, window))
        cancelled[index] = received[index] - estimate

    return (
        PassbandWaveform(
            samples=cancelled,
            sample_rate=received_waveform.sample_rate,
            carrier_hz=received_waveform.carrier_hz,
        ),
        taps,
    )


def _solve_linear_system(matrix: list[list[float]], vector: list[float]) -> list[float]:
    """Solve a dense linear system with Gaussian elimination."""

    size = len(vector)
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]

    for pivot in range(size):
        best_row = max(range(pivot, size), key=lambda row: abs(augmented[row][pivot]))
        augmented[pivot], augmented[best_row] = augmented[best_row], augmented[pivot]
        pivot_value = augmented[pivot][pivot]
        if abs(pivot_value) < 1e-12:
            continue
        inv_pivot = 1.0 / pivot_value
        for col in range(pivot, size + 1):
            augmented[pivot][col] *= inv_pivot
        for row in range(size):
            if row == pivot:
                continue
            factor = augmented[row][pivot]
            if factor == 0.0:
                continue
            for col in range(pivot, size + 1):
                augmented[row][col] -= factor * augmented[pivot][col]

    return [augmented[row][size] for row in range(size)]
