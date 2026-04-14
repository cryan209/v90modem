"""Reference V.32bis RX front end for clean synthetic waveforms."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .tx import SYNC_STATE_TO_INDEX, TransmittedSymbol, startup_symbol_to_point
from .tx_passband import PassbandWaveform
from .tx_waveform import BasebandWaveform
from .training import STATE_A, STATE_B, STATE_C, STATE_D


@dataclass(frozen=True)
class RecoveredSymbol:
    """Recovered symbol decision from a clean waveform."""

    point: complex
    decided_symbol: str
    source_name: str
    source_instance: int
    expected_symbol: str


@dataclass(frozen=True)
class TimingSearchResult:
    offset: int
    metric: float
    recovered: list[RecoveredSymbol]


def passband_to_baseband(
    waveform: PassbandWaveform,
    *,
    carrier_hz: float | None = None,
) -> list[complex]:
    """Mix a real passband waveform down to complex baseband."""

    carrier = waveform.carrier_hz if carrier_hz is None else carrier_hz
    samples: list[complex] = []
    for index, sample in enumerate(waveform.samples):
        phase = 2.0 * math.pi * carrier * index / waveform.sample_rate
        lo = complex(math.cos(phase), -math.sin(phase))
        samples.append(2.0 * sample * lo)
    return samples


def matched_filter(samples: list[complex], taps: list[float]) -> list[complex]:
    """Filter complex samples with real matched-filter taps."""

    output = [0j] * (len(samples) + len(taps) - 1)
    for i, sample in enumerate(samples):
        if sample == 0j:
            continue
        for j, tap in enumerate(taps):
            output[i + j] += sample * tap
    return output


def ideal_symbol_samples(
    filtered_samples: list[complex],
    *,
    symbol_count: int,
    samples_per_symbol: int,
    filter_len: int,
    offset: int = 0,
) -> list[complex]:
    """Sample matched-filter output at ideal symbol instants."""

    if not 0 <= offset < samples_per_symbol:
        raise ValueError("offset must be within one symbol period")

    start = filter_len - 1 + offset
    recovered: list[complex] = []
    for index in range(symbol_count):
        recovered.append(filtered_samples[start + index * samples_per_symbol])
    return recovered


def _candidate_symbols_for_expected(expected_symbol: str) -> list[str]:
    if expected_symbol in {STATE_A, STATE_B, STATE_C, STATE_D}:
        return [STATE_A, STATE_B, STATE_C, STATE_D]
    if expected_symbol == "B1":
        return ["B1", STATE_A, STATE_B, STATE_C, STATE_D]
    if expected_symbol.startswith("Q"):
        return ["Q0", "Q1", "Q2", "Q3"]
    raise ValueError(f"unsupported expected symbol: {expected_symbol}")


def nearest_symbol_label(point: complex, expected_symbol: str) -> str:
    """Choose the nearest allowable startup symbol label for a recovered point."""

    candidates = _candidate_symbols_for_expected(expected_symbol)
    best_label = candidates[0]
    best_distance = float("inf")
    for label in candidates:
        target = startup_symbol_to_point(label)
        distance = (point.real - target.real) ** 2 + (point.imag - target.imag) ** 2
        if distance < best_distance:
            best_distance = distance
            best_label = label
    return best_label


def recover_symbols_ideal(
    passband: PassbandWaveform,
    *,
    transmitted_symbols: list[TransmittedSymbol],
    taps: list[float],
    samples_per_symbol: int,
) -> list[RecoveredSymbol]:
    """Recover symbol decisions from a clean synthetic passband waveform."""

    baseband = passband_to_baseband(passband)
    filtered = matched_filter(baseband, taps)
    sampled = ideal_symbol_samples(
        filtered,
        symbol_count=len(transmitted_symbols),
        samples_per_symbol=samples_per_symbol,
        filter_len=len(taps),
    )

    recovered: list[RecoveredSymbol] = []
    for point, transmitted in zip(sampled, transmitted_symbols):
        decided = nearest_symbol_label(point, transmitted.symbol)
        recovered.append(
            RecoveredSymbol(
                point=point,
                decided_symbol=decided,
                source_name=transmitted.source_name,
                source_instance=transmitted.source_instance,
                expected_symbol=transmitted.symbol,
            )
        )
    return recovered


def symbol_error_metric(recovered: list[RecoveredSymbol]) -> float:
    """Compute squared-distance metric against expected startup points."""

    metric = 0.0
    for symbol in recovered:
        target = startup_symbol_to_point(symbol.expected_symbol)
        metric += (symbol.point.real - target.real) ** 2 + (symbol.point.imag - target.imag) ** 2
    return metric


def recover_symbols_with_timing_offset(
    passband: PassbandWaveform,
    *,
    transmitted_symbols: list[TransmittedSymbol],
    taps: list[float],
    samples_per_symbol: int,
    offset: int,
) -> list[RecoveredSymbol]:
    """Recover symbols using a specified timing offset within the symbol period."""

    baseband = passband_to_baseband(passband)
    filtered = matched_filter(baseband, taps)
    sampled = ideal_symbol_samples(
        filtered,
        symbol_count=len(transmitted_symbols),
        samples_per_symbol=samples_per_symbol,
        filter_len=len(taps),
        offset=offset,
    )

    recovered: list[RecoveredSymbol] = []
    for point, transmitted in zip(sampled, transmitted_symbols):
        decided = nearest_symbol_label(point, transmitted.symbol)
        recovered.append(
            RecoveredSymbol(
                point=point,
                decided_symbol=decided,
                source_name=transmitted.source_name,
                source_instance=transmitted.source_instance,
                expected_symbol=transmitted.symbol,
            )
        )
    return recovered


def search_symbol_timing(
    passband: PassbandWaveform,
    *,
    transmitted_symbols: list[TransmittedSymbol],
    taps: list[float],
    samples_per_symbol: int,
) -> TimingSearchResult:
    """Search over symbol-phase offsets and return the best one."""

    best_offset = 0
    best_metric = float("inf")
    best_recovered: list[RecoveredSymbol] = []
    for offset in range(samples_per_symbol):
        recovered = recover_symbols_with_timing_offset(
            passband,
            transmitted_symbols=transmitted_symbols,
            taps=taps,
            samples_per_symbol=samples_per_symbol,
            offset=offset,
        )
        metric = symbol_error_metric(recovered)
        if metric < best_metric:
            best_metric = metric
            best_offset = offset
            best_recovered = recovered

    return TimingSearchResult(
        offset=best_offset,
        metric=best_metric,
        recovered=best_recovered,
    )
