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
from .channel import apply_channel as _apply_channel_to_passband
from .data import DataResult, DataWaveform, generate_data_waveform, measure_ber, recover_data
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

    def run_data(
        self,
        *,
        bit_rate: int | None = None,
        n_symbols: int = 512,
        seed: int = 42,
        calling_party: bool = True,
    ) -> DataResult:
        """Run a data-mode BER simulation at *bit_rate* bps.

        Generates a random payload, scrambles it, TCM-encodes it, applies the
        configured channel impairments, then demodulates and measures BER/SER.

        Parameters
        ----------
        bit_rate:
            One of 4800, 7200, 9600, 12000, 14400 bps.  Defaults to
            ``tx_config.selected_rate``.
        n_symbols:
            Number of data symbols to transmit.
        seed:
            RNG seed for deterministic payload generation.
        calling_party:
            Selects the scrambler polynomial (True = calling party, tap=17).

        Returns
        -------
        DataResult
            BER, SER, symbol/bit counts, residual carrier phase error, and
            full waveform/recovery objects for further inspection.
        """
        rate = bit_rate if bit_rate is not None else self.tx_config.selected_rate

        # 1. Generate TX waveform.
        waveform = generate_data_waveform(
            self.tx_config,
            bit_rate=rate,
            n_symbols=n_symbols,
            seed=seed,
            calling_party=calling_party,
        )

        # 2. Apply channel impairments.
        impaired = _apply_channel_to_passband(waveform.passband, self.channel_config)
        if self.channel_config.near_end_echo_paths:
            from .channel import (
                adaptive_cancel_near_end_echo,
                mix_passbands,
                subtract_passbands,
                synthesize_near_end_echo,
            )
            # For data mode, near-end echo uses the same TX waveform as the
            # local transmitter (self-interference).
            leaked = synthesize_near_end_echo(
                waveform.passband,
                paths=self.channel_config.near_end_echo_paths,
            )
            impaired = mix_passbands(impaired, leaked)
            if self.channel_config.adaptive_near_end_echo_cancel:
                impaired, _taps = adaptive_cancel_near_end_echo(
                    impaired,
                    waveform.passband,
                    tap_count=self.channel_config.adaptive_echo_tap_count,
                    step_size=self.channel_config.adaptive_echo_step_size,
                )
            elif self.channel_config.cancel_near_end_echo:
                estimate_paths = (
                    self.channel_config.near_end_echo_estimate_paths
                    or self.channel_config.near_end_echo_paths
                )
                estimated = synthesize_near_end_echo(
                    waveform.passband,
                    paths=estimate_paths,
                )
                impaired = subtract_passbands(impaired, estimated)

        # 3. Demodulate.
        enable_equalizer = bool(
            self.channel_config.fir_taps
            or (self.channel_config.echo_delay > 0 and self.channel_config.echo_gain != 0.0)
            or self.channel_config.multi_echo_paths
            or self.channel_config.near_end_echo_paths
        )
        if enable_equalizer and rate >= 12000:
            phase_gain = min(self.rx_config.phase_gain, 0.015 if rate == 12000 else 0.02)
            equalizer_step_size = 0.0005
            equalizer_training_symbols = 256
        else:
            phase_gain = self.rx_config.phase_gain
            equalizer_step_size = 0.0008
            equalizer_training_symbols = 128
        recovery = recover_data(
            impaired,
            matched_filter_taps=waveform.baseband.taps,
            n_symbols=n_symbols,
            bit_rate=rate,
            rx_carrier_hz=self.tx_config.carrier_hz,
            samples_per_symbol=self.tx_config.samples_per_symbol,
            timing_offset=(
                max(0, min(int(round(self.rx_config.timing_offset)), self.tx_config.samples_per_symbol - 1))
                if enable_equalizer else 0
            ),
            phase_gain=phase_gain,
            calling_party=calling_party,
            enable_equalizer=enable_equalizer,
            search_timing=enable_equalizer,
            training_points=waveform.tx_points,
            equalizer_step_size=equalizer_step_size,
            equalizer_training_symbols=equalizer_training_symbols,
        )

        # 4. Measure BER / SER.
        from tools.v32bis_ref.data_mode import DataModeEncoder
        bpg = DataModeEncoder(rate).bits_per_group
        bit_errors, ber, sym_errors, ser = measure_ber(
            waveform.tx_bits,
            recovery.decoded_bits,
            bits_per_group=bpg,
        )

        return DataResult(
            bit_rate=rate,
            n_symbols=n_symbols,
            n_bits=len(waveform.tx_bits),
            bit_errors=bit_errors,
            ber=ber,
            symbol_errors=sym_errors,
            ser=ser,
            snr_db=self.channel_config.snr_db,
            carrier_phase_error_rad=recovery.carrier_phase_error_rad,
            waveform=waveform,
            recovery=recovery,
        )
