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


@dataclass(frozen=True)
class CarrierSearchResult:
    carrier_hz: float
    metric: float
    recovered: list[RecoveredSymbol]


@dataclass(frozen=True)
class JointSearchResult:
    carrier_hz: float
    timing_offset: int
    metric: float
    recovered: list[RecoveredSymbol]


@dataclass(frozen=True)
class CarrierTrackingResult:
    final_phase_rad: float
    metric: float
    recovered: list[RecoveredSymbol]


@dataclass(frozen=True)
class TimingTrackingResult:
    final_offset: int
    metric: float
    recovered: list[RecoveredSymbol]


@dataclass(frozen=True)
class TimingLoopResult:
    final_offset: float
    metric: float
    recovered: list[RecoveredSymbol]


@dataclass(frozen=True)
class FrontendTrackingResult:
    final_phase_rad: float
    final_offset: int
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


def _rotate(point: complex, phase_rad: float) -> complex:
    return point * complex(math.cos(phase_rad), math.sin(phase_rad))


def _phase_error(point: complex, target: complex) -> float:
    return math.atan2(
        point.imag * target.real - point.real * target.imag,
        point.real * target.real + point.imag * target.imag,
    )


def _symbol_metric(point: complex, target: complex) -> float:
    return (point.real - target.real) ** 2 + (point.imag - target.imag) ** 2


def _decide_rotated_symbol(point: complex, expected_symbol: str) -> tuple[str, complex, float, float]:
    decided = nearest_symbol_label(point, expected_symbol)
    target = startup_symbol_to_point(decided)
    return decided, target, _phase_error(point, target), _symbol_metric(point, target)


def _offset_lookahead_metric(
    filtered: list[complex],
    transmitted_symbols: list[TransmittedSymbol],
    *,
    start_index: int,
    samples_per_symbol: int,
    phase_offset: int,
    lookahead_symbols: int,
    phase_estimate: float = 0.0,
) -> float:
    metric = 0.0
    start = start_index + phase_offset
    stop = min(len(transmitted_symbols), lookahead_symbols)
    for local_index in range(stop):
        point = filtered[start + local_index * samples_per_symbol]
        corrected = _rotate(point, -phase_estimate)
        _decided, target, _phase_error_unused, candidate_metric = _decide_rotated_symbol(
            corrected,
            transmitted_symbols[local_index].symbol,
        )
        metric += candidate_metric
    return metric


def _interpolated_sample(filtered: list[complex], sample_index: float) -> complex:
    if sample_index < 0.0 or sample_index > len(filtered) - 1:
        raise ValueError("sample_index out of range")
    left_index = int(math.floor(sample_index))
    right_index = min(left_index + 1, len(filtered) - 1)
    fraction = sample_index - left_index
    left = filtered[left_index]
    right = filtered[right_index]
    return left + (right - left) * fraction


def _acquire_timing_offset(
    filtered: list[complex],
    transmitted_symbols: list[TransmittedSymbol],
    *,
    start_index: int,
    samples_per_symbol: int,
    lookahead_symbols: int,
    phase_estimate: float = 0.0,
) -> int:
    best_offset = 0
    best_metric = float("inf")
    for candidate_offset in range(samples_per_symbol):
        candidate_metric = _offset_lookahead_metric(
            filtered,
            transmitted_symbols,
            start_index=start_index,
            samples_per_symbol=samples_per_symbol,
            phase_offset=candidate_offset,
            lookahead_symbols=lookahead_symbols,
            phase_estimate=phase_estimate,
        )
        if candidate_metric < best_metric:
            best_offset = candidate_offset
            best_metric = candidate_metric
    return best_offset


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


def recover_symbols_with_frontend(
    passband: PassbandWaveform,
    *,
    transmitted_symbols: list[TransmittedSymbol],
    taps: list[float],
    samples_per_symbol: int,
    timing_offset: int = 0,
    carrier_hz: float | None = None,
) -> list[RecoveredSymbol]:
    """Recover symbols using specified timing and carrier choices."""

    baseband = passband_to_baseband(passband, carrier_hz=carrier_hz)
    filtered = matched_filter(baseband, taps)
    sampled = ideal_symbol_samples(
        filtered,
        symbol_count=len(transmitted_symbols),
        samples_per_symbol=samples_per_symbol,
        filter_len=len(taps),
        offset=timing_offset,
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


def search_carrier_frequency(
    passband: PassbandWaveform,
    *,
    transmitted_symbols: list[TransmittedSymbol],
    taps: list[float],
    samples_per_symbol: int,
    timing_offset: int = 0,
    carrier_candidates_hz: list[float],
) -> CarrierSearchResult:
    """Search over RX carrier frequencies and return the best one."""

    if not carrier_candidates_hz:
        raise ValueError("carrier_candidates_hz must not be empty")

    best_carrier = carrier_candidates_hz[0]
    best_metric = float("inf")
    best_recovered: list[RecoveredSymbol] = []
    for carrier_hz in carrier_candidates_hz:
        recovered = recover_symbols_with_frontend(
            passband,
            transmitted_symbols=transmitted_symbols,
            taps=taps,
            samples_per_symbol=samples_per_symbol,
            timing_offset=timing_offset,
            carrier_hz=carrier_hz,
        )
        metric = symbol_error_metric(recovered)
        if metric < best_metric:
            best_metric = metric
            best_carrier = carrier_hz
            best_recovered = recovered

    return CarrierSearchResult(
        carrier_hz=best_carrier,
        metric=best_metric,
        recovered=best_recovered,
    )


def search_timing_and_carrier(
    passband: PassbandWaveform,
    *,
    transmitted_symbols: list[TransmittedSymbol],
    taps: list[float],
    samples_per_symbol: int,
    carrier_candidates_hz: list[float],
) -> JointSearchResult:
    """Jointly search over timing phase and carrier frequency."""

    if not carrier_candidates_hz:
        raise ValueError("carrier_candidates_hz must not be empty")

    best_carrier = carrier_candidates_hz[0]
    best_offset = 0
    best_metric = float("inf")
    best_recovered: list[RecoveredSymbol] = []

    for carrier_hz in carrier_candidates_hz:
        for offset in range(samples_per_symbol):
            recovered = recover_symbols_with_frontend(
                passband,
                transmitted_symbols=transmitted_symbols,
                taps=taps,
                samples_per_symbol=samples_per_symbol,
                timing_offset=offset,
                carrier_hz=carrier_hz,
            )
            metric = symbol_error_metric(recovered)
            if metric < best_metric:
                best_metric = metric
                best_carrier = carrier_hz
                best_offset = offset
                best_recovered = recovered

    return JointSearchResult(
        carrier_hz=best_carrier,
        timing_offset=best_offset,
        metric=best_metric,
        recovered=best_recovered,
    )


def recover_symbols_with_carrier_tracking(
    passband: PassbandWaveform,
    *,
    transmitted_symbols: list[TransmittedSymbol],
    taps: list[float],
    samples_per_symbol: int,
    timing_offset: int = 0,
    carrier_hz: float | None = None,
    phase_gain: float = 0.2,
) -> CarrierTrackingResult:
    """Recover symbols with a simple decision-directed carrier phase tracker."""

    baseband = passband_to_baseband(passband, carrier_hz=carrier_hz)
    filtered = matched_filter(baseband, taps)
    sampled = ideal_symbol_samples(
        filtered,
        symbol_count=len(transmitted_symbols),
        samples_per_symbol=samples_per_symbol,
        filter_len=len(taps),
        offset=timing_offset,
    )

    phase_estimate = 0.0
    recovered: list[RecoveredSymbol] = []
    for point, transmitted in zip(sampled, transmitted_symbols):
        corrected = _rotate(point, -phase_estimate)
        decided, _target, phase_error, _metric = _decide_rotated_symbol(corrected, transmitted.symbol)
        recovered.append(
            RecoveredSymbol(
                point=corrected,
                decided_symbol=decided,
                source_name=transmitted.source_name,
                source_instance=transmitted.source_instance,
                expected_symbol=transmitted.symbol,
            )
        )
        phase_estimate += phase_gain * phase_error

    return CarrierTrackingResult(
        final_phase_rad=phase_estimate,
        metric=symbol_error_metric(recovered),
        recovered=recovered,
    )


def recover_symbols_with_timing_tracking(
    passband: PassbandWaveform,
    *,
    transmitted_symbols: list[TransmittedSymbol],
    taps: list[float],
    samples_per_symbol: int,
    timing_offset: int,
    carrier_hz: float | None = None,
    timing_step: int = 1,
    lookahead_symbols: int = 8,
    acquisition_symbols: int = 32,
) -> TimingTrackingResult:
    """Recover symbols with a simple greedy timing tracker."""

    if timing_step < 1:
        raise ValueError("timing_step must be positive")
    if lookahead_symbols < 1:
        raise ValueError("lookahead_symbols must be positive")
    if acquisition_symbols < 1:
        raise ValueError("acquisition_symbols must be positive")

    baseband = passband_to_baseband(passband, carrier_hz=carrier_hz)
    filtered = matched_filter(baseband, taps)

    start = len(taps) - 1
    phase_offset = _acquire_timing_offset(
        filtered,
        transmitted_symbols[:acquisition_symbols],
        start_index=start,
        samples_per_symbol=samples_per_symbol,
        lookahead_symbols=min(lookahead_symbols, acquisition_symbols),
    )
    recovered: list[RecoveredSymbol] = []
    for index, transmitted in enumerate(transmitted_symbols):
        nominal = start + index * samples_per_symbol
        best_offset = phase_offset
        best_point = filtered[nominal + phase_offset]
        best_metric = _offset_lookahead_metric(
            filtered,
            transmitted_symbols[index:],
            start_index=nominal,
            samples_per_symbol=samples_per_symbol,
            phase_offset=phase_offset,
            lookahead_symbols=lookahead_symbols,
        )

        for delta in range(-timing_step, timing_step + 1):
            candidate_offset = phase_offset + delta
            if not 0 <= candidate_offset < samples_per_symbol:
                continue
            candidate_point = filtered[nominal + candidate_offset]
            candidate_metric = _offset_lookahead_metric(
                filtered,
                transmitted_symbols[index:],
                start_index=nominal,
                samples_per_symbol=samples_per_symbol,
                phase_offset=candidate_offset,
                lookahead_symbols=lookahead_symbols,
            )
            if candidate_metric < best_metric:
                best_offset = candidate_offset
                best_point = candidate_point
                best_metric = candidate_metric

        phase_offset = best_offset
        decided = nearest_symbol_label(best_point, transmitted.symbol)
        recovered.append(
            RecoveredSymbol(
                point=best_point,
                decided_symbol=decided,
                source_name=transmitted.source_name,
                source_instance=transmitted.source_instance,
                expected_symbol=transmitted.symbol,
            )
        )

    return TimingTrackingResult(
        final_offset=phase_offset,
        metric=symbol_error_metric(recovered),
        recovered=recovered,
    )


def recover_symbols_with_tracking(
    passband: PassbandWaveform,
    *,
    transmitted_symbols: list[TransmittedSymbol],
    taps: list[float],
    samples_per_symbol: int,
    timing_offset: int = 0,
    carrier_hz: float | None = None,
    phase_gain: float = 0.2,
    timing_step: int = 1,
    lookahead_symbols: int = 8,
    acquisition_symbols: int = 32,
) -> FrontendTrackingResult:
    """Recover symbols with simple joint timing and carrier tracking."""

    if timing_step < 1:
        raise ValueError("timing_step must be positive")
    if lookahead_symbols < 1:
        raise ValueError("lookahead_symbols must be positive")
    if acquisition_symbols < 1:
        raise ValueError("acquisition_symbols must be positive")

    baseband = passband_to_baseband(passband, carrier_hz=carrier_hz)
    filtered = matched_filter(baseband, taps)

    phase_estimate = 0.0
    start = len(taps) - 1
    phase_offset = _acquire_timing_offset(
        filtered,
        transmitted_symbols[:acquisition_symbols],
        start_index=start,
        samples_per_symbol=samples_per_symbol,
        lookahead_symbols=min(lookahead_symbols, acquisition_symbols),
    )
    recovered: list[RecoveredSymbol] = []
    for index, transmitted in enumerate(transmitted_symbols):
        nominal = start + index * samples_per_symbol
        best_offset = phase_offset
        best_corrected = _rotate(filtered[nominal + phase_offset], -phase_estimate)
        best_decided, _best_target, best_phase_error, _best_symbol_metric = _decide_rotated_symbol(
            best_corrected,
            transmitted.symbol,
        )
        best_metric = _offset_lookahead_metric(
            filtered,
            transmitted_symbols[index:],
            start_index=nominal,
            samples_per_symbol=samples_per_symbol,
            phase_offset=phase_offset,
            lookahead_symbols=lookahead_symbols,
            phase_estimate=phase_estimate,
        )

        for delta in range(-timing_step, timing_step + 1):
            candidate_offset = phase_offset + delta
            if not 0 <= candidate_offset < samples_per_symbol:
                continue
            candidate_corrected = _rotate(filtered[nominal + candidate_offset], -phase_estimate)
            candidate_decided, _target, candidate_phase_error, _candidate_symbol_metric = _decide_rotated_symbol(
                candidate_corrected,
                transmitted.symbol,
            )
            candidate_metric = _offset_lookahead_metric(
                filtered,
                transmitted_symbols[index:],
                start_index=nominal,
                samples_per_symbol=samples_per_symbol,
                phase_offset=candidate_offset,
                lookahead_symbols=lookahead_symbols,
                phase_estimate=phase_estimate,
            )
            if candidate_metric < best_metric:
                best_offset = candidate_offset
                best_corrected = candidate_corrected
                best_decided = candidate_decided
                best_phase_error = candidate_phase_error
                best_metric = candidate_metric

        phase_offset = best_offset
        recovered.append(
            RecoveredSymbol(
                point=best_corrected,
                decided_symbol=best_decided,
                source_name=transmitted.source_name,
                source_instance=transmitted.source_instance,
                expected_symbol=transmitted.symbol,
            )
        )
        phase_estimate += phase_gain * best_phase_error

    return FrontendTrackingResult(
        final_phase_rad=phase_estimate,
        final_offset=phase_offset,
        metric=symbol_error_metric(recovered),
        recovered=recovered,
    )


def recover_symbols_with_timing_loop(
    passband: PassbandWaveform,
    *,
    transmitted_symbols: list[TransmittedSymbol],
    taps: list[float],
    samples_per_symbol: int,
    timing_offset: float = 0.0,
    carrier_hz: float | None = None,
    timing_gain: float = 0.01,
    early_late_spacing: float = 0.5,
) -> TimingLoopResult:
    """Recover symbols with a simple early/late timing error detector loop."""

    if not 0.0 <= timing_offset < samples_per_symbol:
        raise ValueError("timing_offset must be within one symbol period")
    if timing_gain <= 0.0:
        raise ValueError("timing_gain must be positive")
    if early_late_spacing <= 0.0:
        raise ValueError("early_late_spacing must be positive")

    baseband = passband_to_baseband(passband, carrier_hz=carrier_hz)
    filtered = matched_filter(baseband, taps)

    phase_offset = timing_offset
    start = len(taps) - 1
    recovered: list[RecoveredSymbol] = []
    for index, transmitted in enumerate(transmitted_symbols):
        nominal = start + index * samples_per_symbol + phase_offset
        point = _interpolated_sample(filtered, nominal)
        early = _interpolated_sample(filtered, nominal - early_late_spacing)
        late = _interpolated_sample(filtered, nominal + early_late_spacing)

        decided = nearest_symbol_label(point, transmitted.symbol)
        target = startup_symbol_to_point(transmitted.symbol)
        early_metric = _symbol_metric(early, target)
        late_metric = _symbol_metric(late, target)
        phase_offset += timing_gain * (early_metric - late_metric)
        phase_offset %= samples_per_symbol

        recovered.append(
            RecoveredSymbol(
                point=point,
                decided_symbol=decided,
                source_name=transmitted.source_name,
                source_instance=transmitted.source_instance,
                expected_symbol=transmitted.symbol,
            )
        )

    return TimingLoopResult(
        final_offset=phase_offset,
        metric=symbol_error_metric(recovered),
        recovered=recovered,
    )
