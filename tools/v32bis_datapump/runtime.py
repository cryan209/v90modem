"""High-level runtime datapump facade."""

from __future__ import annotations

from dataclasses import dataclass

from .channel import (
    ChannelConfig,
    adaptive_cancel_near_end_echo,
    apply_channel,
    mix_passbands,
    subtract_passbands,
    synthesize_near_end_echo,
)
from .oracle import OracleComparison, compare_events
from .rx import RxConfig, StartupRecovery, recover_startup, recover_startup_blind
from .tx import (
    StartupWaveform,
    TxConfig,
    generate_answer_startup_waveform,
    generate_call_startup_waveform,
)


@dataclass(frozen=True)
class DatapumpResult:
    startup_waveform: StartupWaveform
    local_waveform: StartupWaveform
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
        blind_runtime: bool = False,
    ) -> None:
        self.tx_config = tx_config or TxConfig()
        self.rx_config = rx_config or RxConfig()
        self.channel_config = channel_config or ChannelConfig()
        self.blind_runtime = blind_runtime

    def run_startup(self) -> DatapumpResult:
        startup = generate_answer_startup_waveform(self.tx_config)
        local = generate_call_startup_waveform(self.tx_config)
        impaired = apply_channel(startup.passband, self.channel_config)
        if self.channel_config.near_end_echo_paths:
            leaked = synthesize_near_end_echo(
                local.passband,
                paths=self.channel_config.near_end_echo_paths,
            )
            impaired = mix_passbands(impaired, leaked)
            if self.channel_config.adaptive_near_end_echo_cancel:
                impaired, _estimated_taps = adaptive_cancel_near_end_echo(
                    impaired,
                    local.passband,
                    tap_count=self.channel_config.adaptive_echo_tap_count,
                    step_size=self.channel_config.adaptive_echo_step_size,
                )
            elif self.channel_config.cancel_near_end_echo:
                estimate_paths = (
                    self.channel_config.near_end_echo_estimate_paths
                    or self.channel_config.near_end_echo_paths
                )
                estimated = synthesize_near_end_echo(
                    local.passband,
                    paths=estimate_paths,
                )
                impaired = subtract_passbands(impaired, estimated)
        if self.blind_runtime:
            recovery = recover_startup_blind(
                impaired,
                matched_filter_taps=startup.baseband.taps,
                config=self.rx_config,
            )
        else:
            recovery = recover_startup(
                impaired,
                transmitted_symbols=startup.transmitted_symbols,
                matched_filter_taps=startup.baseband.taps,
                config=self.rx_config,
            )
        oracle = compare_events(recovery.events)
        return DatapumpResult(
            startup_waveform=startup,
            local_waveform=local,
            recovery=recovery,
            oracle=oracle,
        )
