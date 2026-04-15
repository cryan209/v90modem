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
