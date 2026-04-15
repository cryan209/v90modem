"""TX datapump entry points for startup waveform generation."""

from __future__ import annotations

from dataclasses import dataclass

from tools.v32bis_ref.rate_signal import rate_mask_from_list
from tools.v32bis_ref.startup import generate_answer_startup_trace
from tools.v32bis_ref.tx import TransmittedSymbol, startup_trace_to_complex_symbols
from tools.v32bis_ref.tx_passband import PassbandWaveform, baseband_to_passband
from tools.v32bis_ref.tx_waveform import BasebandWaveform, symbols_to_baseband


@dataclass(frozen=True)
class TxConfig:
    r1_rates: tuple[int, ...] = (4800, 7200, 9600, 12000)
    r2_rates: tuple[int, ...] = (4800, 9600)
    selected_rate: int = 9600
    trn_length: int = 1280
    samples_per_symbol: int = 10
    sample_rate: int = 24000
    carrier_hz: float = 1800.0


@dataclass(frozen=True)
class StartupWaveform:
    transmitted_symbols: list[TransmittedSymbol]
    baseband: BasebandWaveform
    passband: PassbandWaveform


def generate_answer_startup_waveform(config: TxConfig) -> StartupWaveform:
    """Generate an answer-mode startup waveform for the datapump."""

    trace = generate_answer_startup_trace(
        r1_mask=rate_mask_from_list(list(config.r1_rates)),
        r2_mask=rate_mask_from_list(list(config.r2_rates)),
        r3_selected_rate=config.selected_rate,
        trn_length=config.trn_length,
    )
    transmitted = startup_trace_to_complex_symbols(trace)
    baseband = symbols_to_baseband(transmitted, samples_per_symbol=config.samples_per_symbol)
    passband = baseband_to_passband(
        baseband,
        sample_rate=config.sample_rate,
        carrier_hz=config.carrier_hz,
    )
    return StartupWaveform(
        transmitted_symbols=transmitted,
        baseband=baseband,
        passband=passband,
    )
