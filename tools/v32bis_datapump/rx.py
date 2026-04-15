"""RX datapump entry points for startup recovery."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from tools.v32bis_ref.negotiation import decode_e_rate
from tools.v32bis_ref.receiver import V32bisLogicalReceiver
from tools.v32bis_ref.rate_signal import decode_rate_sequence_symbols, is_e_sequence_bits, is_rate_signal_bits
from tools.v32bis_ref.receiver import DetectedEvent
from tools.v32bis_ref.rx_frontend import (
    ideal_symbol_samples,
    matched_filter,
    passband_to_baseband,
    recovered_to_metadata_free_observable_stream,
    recover_startup_with_decision_directed_tracking,
)
from tools.v32bis_ref.tx import startup_symbol_to_point
from tools.v32bis_ref.tx_passband import PassbandWaveform
from tools.v32bis_ref.tx import TransmittedSymbol


STATE_POINTS = [
    startup_symbol_to_point("Q0"),
    startup_symbol_to_point("Q1"),
    startup_symbol_to_point("Q2"),
    startup_symbol_to_point("Q3"),
]


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


@dataclass(frozen=True)
class BlindStateSequence:
    states: list[int]
    metric: float


def _nearest_state(point: complex) -> tuple[int, float]:
    best_index = 0
    best_distance = float("inf")
    for index, target in enumerate(STATE_POINTS):
        distance = (point.real - target.real) ** 2 + (point.imag - target.imag) ** 2
        if distance < best_distance:
            best_distance = distance
            best_index = index
    return best_index, best_distance


def _recover_raw_points(
    waveform: PassbandWaveform,
    *,
    matched_filter_taps: list[float],
    config: RxConfig,
) -> list[complex]:
    baseband = passband_to_baseband(waveform, carrier_hz=config.rx_carrier_hz)
    filtered = matched_filter(baseband, matched_filter_taps)
    start = len(matched_filter_taps) - 1 + int(config.timing_offset)
    symbol_count = max(0, (len(filtered) - start) // config.samples_per_symbol)
    return ideal_symbol_samples(
        filtered,
        symbol_count=symbol_count,
        samples_per_symbol=config.samples_per_symbol,
        filter_len=len(matched_filter_taps),
        offset=int(config.timing_offset),
    )


def _equalize_points(points: list[complex], *, tap_count: int, step_size: float) -> BlindStateSequence:
    half = tap_count // 2
    padded = [0j] * half + points + [0j] * half
    taps = [0j] * tap_count
    taps[half] = 1.0 + 0.0j
    states: list[int] = []
    metric = 0.0
    for index in range(len(points)):
        window = padded[index:index + tap_count]
        output = sum(tap * sample for tap, sample in zip(taps, window))
        state, distance = _nearest_state(output)
        target = STATE_POINTS[state]
        error = target - output
        for tap_index in range(tap_count):
            taps[tap_index] += step_size * error * window[tap_index].conjugate()
        states.append(state)
        metric += distance
    return BlindStateSequence(states=states, metric=metric)


def _equalize_points_dfe(
    points: list[complex],
    *,
    feedforward_taps: int,
    feedback_taps: int,
    step_size: float,
) -> BlindStateSequence:
    ff_half = feedforward_taps // 2
    padded = [0j] * ff_half + points + [0j] * ff_half
    ff = [0j] * feedforward_taps
    ff[ff_half] = 1.0 + 0.0j
    fb = [0j] * feedback_taps
    past_targets = [0j] * feedback_taps
    states: list[int] = []
    metric = 0.0
    for index in range(len(points)):
        ff_window = padded[index:index + feedforward_taps]
        output = sum(t * s for t, s in zip(ff, ff_window)) - sum(t * s for t, s in zip(fb, past_targets))
        state, distance = _nearest_state(output)
        target = STATE_POINTS[state]
        error = target - output
        for tap_index in range(feedforward_taps):
            ff[tap_index] += step_size * error * ff_window[tap_index].conjugate()
        for tap_index in range(feedback_taps):
            fb[tap_index] -= step_size * error * past_targets[tap_index].conjugate()
        past_targets = [target] + past_targets[:-1]
        states.append(state)
        metric += distance
    return BlindStateSequence(states=states, metric=metric)


def _decode_rate_bits_from_states(states: list[int], *, remote_calling_party: bool) -> list[int]:
    return decode_rate_sequence_symbols(
        [f"Q{state}" for state in states],
        calling_party=remote_calling_party,
        initial_diff_state=1,
    )


def _detect_events_from_states(states: list[int], *, remote_calling_party: bool) -> list[DetectedEvent]:
    events: list[DetectedEvent] = []
    recent_ab: deque[int] = deque(maxlen=256)
    in_s_run = False
    q_run: deque[int] = deque()
    rate_sequences: list[list[int]] = []
    rate_signal_count = 0
    seen_e = False
    b1_run = 0
    b1_emitted = False

    for state in states:
        if state in {0, 1}:
            q_run.clear()
            rate_sequences.clear()
            b1_run = 0
            b1_emitted = False
            recent_ab.append(state)
            detected = len(recent_ab) == 256 and all(recent_ab[i] == (i % 2) for i in range(256))
            if detected and not in_s_run:
                in_s_run = True
                events.append(DetectedEvent(name="S"))
            elif not detected:
                in_s_run = False
            continue

        recent_ab.clear()
        in_s_run = False

        if seen_e and state == 3:
            q_run.clear()
            rate_sequences.clear()
            b1_run += 1
            if b1_run >= 24 and not b1_emitted:
                b1_emitted = True
                events.append(DetectedEvent(name="B1", repetitions=b1_run))
            continue

        b1_run = 0
        b1_emitted = False
        q_run.append(state)
        while len(q_run) >= 8:
            candidate = list(q_run)[:8]
            try:
                bits = _decode_rate_bits_from_states(candidate, remote_calling_party=remote_calling_party)
            except ValueError:
                q_run.popleft()
                continue
            if is_e_sequence_bits(bits) and not seen_e:
                seen_e = True
                for _ in range(8):
                    q_run.popleft()
                events.append(DetectedEvent(name="E", selected_rate=decode_e_rate(bits)))
                break
            if is_rate_signal_bits(bits):
                for _ in range(8):
                    q_run.popleft()
                rate_sequences.append(bits)
                if len(rate_sequences) >= 2 and rate_sequences[-1] == rate_sequences[-2]:
                    rate_signal_count += 1
                    rate_mask_bits = bits
                    rate_mask = 0
                    if rate_mask_bits[5]:
                        rate_mask |= 0x0020
                    if rate_mask_bits[9]:
                        rate_mask |= 0x0040
                    if rate_mask_bits[6]:
                        rate_mask |= 0x0200
                    if rate_mask_bits[10]:
                        rate_mask |= 0x0400
                    if rate_mask_bits[12]:
                        rate_mask |= 0x1000
                    name = "R1" if rate_signal_count == 1 else "R3" if rate_signal_count == 2 else f"R{rate_signal_count}"
                    events.append(DetectedEvent(name=name, rate_mask=rate_mask, repetitions=2))
                    rate_sequences.clear()
                continue
            q_run.popleft()
    return events


def recover_startup(
    waveform: PassbandWaveform,
    *,
    transmitted_symbols: list[TransmittedSymbol],
    matched_filter_taps: list[float],
    config: RxConfig,
) -> StartupRecovery:
    """Recover startup events with the validated runtime/reference hybrid path."""

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


def recover_startup_blind(
    waveform: PassbandWaveform,
    *,
    matched_filter_taps: list[float],
    config: RxConfig,
) -> StartupRecovery:
    """Recover startup events from a passband waveform without remote-symbol oracle input."""

    points = _recover_raw_points(waveform, matched_filter_taps=matched_filter_taps, config=config)
    candidates = [
        ("raw", BlindStateSequence(states=[_nearest_state(point)[0] for point in points], metric=sum(_nearest_state(point)[1] for point in points))),
        ("eq7", _equalize_points(points, tap_count=7, step_size=0.0015)),
        ("eq9", _equalize_points(points, tap_count=9, step_size=0.001)),
        ("dfe9x4", _equalize_points_dfe(points, feedforward_taps=9, feedback_taps=4, step_size=0.0008)),
    ]

    best_mode = candidates[0][0]
    best_events = _detect_events_from_states(candidates[0][1].states, remote_calling_party=False)
    best_metric = candidates[0][1].metric
    for mode, candidate in candidates[1:]:
        events = _detect_events_from_states(candidate.states, remote_calling_party=False)
        if (len(events), -candidate.metric) > (len(best_events), -best_metric):
            best_mode = mode
            best_events = events
            best_metric = candidate.metric

    return StartupRecovery(
        mode=best_mode,
        metric=best_metric,
        events=best_events,
    )
