"""RX datapump entry points for startup recovery."""

from __future__ import annotations

from dataclasses import dataclass

from tools.v32bis_ref.receiver import DetectedEvent, V32bisLogicalReceiver
from tools.v32bis_ref.rx_frontend import (
    recovered_to_metadata_free_observable_stream,
    recover_startup_with_decision_directed_tracking,
)
from tools.v32bis_ref.tx import TransmittedSymbol
from tools.v32bis_ref.tx_passband import PassbandWaveform


@dataclass(frozen=True)
class RxConfig:
    samples_per_symbol: int = 10
    rx_carrier_hz: float = 1801.0
    timing_offset: float = 1.0
    phase_gain: float = 0.05
    timing_gain: float = 0.005
    early_late_spacing: float = 0.5


@dataclass(frozen=True)
class StartupRecovery:
    mode: str
    metric: float
    events: list[DetectedEvent]


def recover_startup(
    waveform: PassbandWaveform,
    *,
    transmitted_symbols: list[TransmittedSymbol],
    matched_filter_taps: list[float],
    config: RxConfig,
) -> StartupRecovery:
    """Recover startup events from a passband waveform."""

    tracked = recover_startup_with_decision_directed_tracking(
        waveform,
        transmitted_symbols=transmitted_symbols,
        taps=matched_filter_taps,
        samples_per_symbol=config.samples_per_symbol,
        timing_offset=config.timing_offset,
        carrier_hz=config.rx_carrier_hz,
        phase_gain=config.phase_gain,
        timing_gain=config.timing_gain,
        early_late_spacing=config.early_late_spacing,
    )
    receiver = V32bisLogicalReceiver()
    events = receiver.ingest_all(recovered_to_metadata_free_observable_stream(tracked.recovered))
    return StartupRecovery(
        mode=tracked.mode,
        metric=tracked.metric,
        events=events,
    )
