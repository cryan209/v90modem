"""High-level runtime datapump facade."""

from __future__ import annotations

from dataclasses import dataclass

from .channel import ChannelConfig, apply_channel
from .oracle import OracleComparison, compare_events
from .rx import RxConfig, StartupRecovery, recover_startup
from .tx import StartupWaveform, TxConfig, generate_answer_startup_waveform


@dataclass(frozen=True)
class DatapumpResult:
    startup_waveform: StartupWaveform
    recovery: StartupRecovery
    oracle: OracleComparison


class V32bisDatapump:
    """Minimal runtime datapump wrapper for startup-mode experiments."""

    def __init__(
        self,
        *,
        tx_config: TxConfig | None = None,
        rx_config: RxConfig | None = None,
        channel_config: ChannelConfig | None = None,
    ) -> None:
        self.tx_config = tx_config or TxConfig()
        self.rx_config = rx_config or RxConfig()
        self.channel_config = channel_config or ChannelConfig()

    def run_startup(self) -> DatapumpResult:
        startup = generate_answer_startup_waveform(self.tx_config)
        impaired = apply_channel(startup.passband, self.channel_config)
        recovery = recover_startup(
            impaired,
            transmitted_symbols=startup.transmitted_symbols,
            matched_filter_taps=startup.baseband.taps,
            config=self.rx_config,
        )
        oracle = compare_events(recovery.events)
        return DatapumpResult(
            startup_waveform=startup,
            recovery=recovery,
            oracle=oracle,
        )
