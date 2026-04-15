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


@dataclass(frozen=True)
class BlindCandidate:
    mode: str
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
    timing_offset: int | None = None,
    carrier_hz: float | None = None,
) -> list[complex]:
    offset = int(config.timing_offset) if timing_offset is None else timing_offset
    carrier = config.rx_carrier_hz if carrier_hz is None else carrier_hz
    baseband = passband_to_baseband(waveform, carrier_hz=carrier)
    filtered = matched_filter(baseband, matched_filter_taps)
    start = len(matched_filter_taps) - 1 + offset
    symbol_count = max(0, (len(filtered) - start) // config.samples_per_symbol)
    return ideal_symbol_samples(
        filtered,
        symbol_count=symbol_count,
        samples_per_symbol=config.samples_per_symbol,
        filter_len=len(matched_filter_taps),
        offset=offset,
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


def _rate_mask_from_bits(rate_mask_bits: list[int]) -> int:
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
    return rate_mask


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
                    rate_mask = _rate_mask_from_bits(bits)
                    name = "R1" if rate_signal_count == 1 else "R3" if rate_signal_count == 2 else f"R{rate_signal_count}"
                    events.append(DetectedEvent(name=name, rate_mask=rate_mask, repetitions=2))
                    rate_sequences.clear()
                continue
            q_run.popleft()
    return events


def _decode_best_word_near(
    states: list[int],
    *,
    center_index: int,
    search_radius: int,
    remote_calling_party: bool,
    predicate,
) -> list[int] | None:
    best_bits: list[int] | None = None
    for offset in range(-search_radius, search_radius + 1):
        start = center_index + offset
        if start < 0 or start + 8 > len(states):
            continue
        candidate = states[start:start + 8]
        try:
            bits = _decode_rate_bits_from_states(candidate, remote_calling_party=remote_calling_party)
        except ValueError:
            continue
        if predicate(bits):
            best_bits = bits
            break
    return best_bits


def _detect_events_from_startup_windows(states: list[int], *, remote_calling_party: bool) -> list[DetectedEvent]:
    """Detect startup events using the expected answer-mode startup timing windows."""

    events: list[DetectedEvent] = []
    conditioning_symbols = 1552
    rate_symbols = 16
    e_symbols = 8

    def has_s_window_near(center: int, radius: int) -> bool:
        for offset in range(-radius, radius + 1):
            start = center + offset
            if start < 0 or start + 256 > len(states):
                continue
            window = states[start:start + 256]
            if all(window[i] == (i % 2) for i in range(256)):
                return True
        return False

    if has_s_window_near(0, 32):
        events.append(DetectedEvent(name="S"))

    r1_bits = _decode_best_word_near(
        states,
        center_index=conditioning_symbols,
        search_radius=8,
        remote_calling_party=remote_calling_party,
        predicate=is_rate_signal_bits,
    )
    if r1_bits is not None:
        events.append(DetectedEvent(name="R1", rate_mask=_rate_mask_from_bits(r1_bits), repetitions=2))

    second_conditioning = conditioning_symbols + rate_symbols
    if has_s_window_near(second_conditioning, 32):
        events.append(DetectedEvent(name="S"))

    r3_bits = _decode_best_word_near(
        states,
        center_index=second_conditioning + conditioning_symbols,
        search_radius=8,
        remote_calling_party=remote_calling_party,
        predicate=is_rate_signal_bits,
    )
    if r3_bits is not None:
        events.append(DetectedEvent(name="R3", rate_mask=_rate_mask_from_bits(r3_bits), repetitions=2))

    e_bits = _decode_best_word_near(
        states,
        center_index=second_conditioning + conditioning_symbols + rate_symbols,
        search_radius=8,
        remote_calling_party=remote_calling_party,
        predicate=is_e_sequence_bits,
    )
    if e_bits is not None:
        events.append(DetectedEvent(name="E", selected_rate=decode_e_rate(e_bits)))

    b1_start = second_conditioning + conditioning_symbols + rate_symbols + e_symbols
    if b1_start + 24 <= len(states):
        b1_window = states[b1_start:b1_start + 24]
        if b1_window.count(3) >= 16:
            events.append(DetectedEvent(name="B1", repetitions=24))

    return events


def _best_alternation_matches(
    states: list[int],
    *,
    center_index: int,
    search_radius: int,
    window_length: int = 256,
) -> int:
    best_matches = 0
    for offset in range(-search_radius, search_radius + 1):
        start = center_index + offset
        if start < 0 or start + window_length > len(states):
            continue
        window = states[start:start + window_length]
        matches = sum(1 for index, state in enumerate(window) if state == (index % 2))
        if matches > best_matches:
            best_matches = matches
    return best_matches


def _best_rate_event_from_candidates(
    candidates: list[BlindCandidate],
    *,
    center_index: int,
    search_radius: int,
    remote_calling_party: bool,
    predicate,
    name: str,
) -> tuple[DetectedEvent | None, str | None]:
    best_event: DetectedEvent | None = None
    best_mode: str | None = None
    best_metric = float("inf")
    for candidate in candidates:
        bits = _decode_best_word_near(
            candidate.states,
            center_index=center_index,
            search_radius=search_radius,
            remote_calling_party=remote_calling_party,
            predicate=predicate,
        )
        if bits is None:
            continue
        if predicate is is_rate_signal_bits:
            event = DetectedEvent(name=name, rate_mask=_rate_mask_from_bits(bits), repetitions=2)
        else:
            event = DetectedEvent(name=name, selected_rate=decode_e_rate(bits))
        if candidate.metric < best_metric:
            best_event = event
            best_mode = candidate.mode
            best_metric = candidate.metric
    return best_event, best_mode


def _best_b1_from_candidates(
    candidates: list[BlindCandidate],
    *,
    start_index: int,
    search_radius: int,
    required_count: int,
    window_length: int = 24,
) -> tuple[DetectedEvent | None, str | None]:
    best_event: DetectedEvent | None = None
    best_mode: str | None = None
    best_metric = float("inf")
    for candidate in candidates:
        for offset in range(-search_radius, search_radius + 1):
            start = start_index + offset
            if start < 0 or start + window_length > len(candidate.states):
                continue
            window = candidate.states[start:start + window_length]
            if window.count(3) >= required_count and candidate.metric < best_metric:
                best_event = DetectedEvent(name="B1", repetitions=window_length)
                best_mode = candidate.mode
                best_metric = candidate.metric
                break
    return best_event, best_mode


def _detect_events_from_candidate_bank(
    candidates: list[BlindCandidate],
    *,
    remote_calling_party: bool,
    s_threshold: int = 224,
) -> tuple[list[DetectedEvent], list[str]]:
    conditioning_symbols = 1552
    rate_symbols = 16
    e_symbols = 8

    events: list[DetectedEvent] = []
    contributors: list[str] = []

    best_s1 = max(
        candidates,
        key=lambda candidate: (_best_alternation_matches(candidate.states, center_index=0, search_radius=64), -candidate.metric),
    )
    if _best_alternation_matches(best_s1.states, center_index=0, search_radius=64) >= s_threshold:
        events.append(DetectedEvent(name="S"))
        contributors.append(f"S1={best_s1.mode}")

    r1_event, r1_mode = _best_rate_event_from_candidates(
        candidates,
        center_index=conditioning_symbols,
        search_radius=16,
        remote_calling_party=remote_calling_party,
        predicate=is_rate_signal_bits,
        name="R1",
    )
    if r1_event is not None and r1_mode is not None:
        events.append(r1_event)
        contributors.append(f"R1={r1_mode}")

    second_conditioning = conditioning_symbols + rate_symbols
    best_s2 = max(
        candidates,
        key=lambda candidate: (
            _best_alternation_matches(candidate.states, center_index=second_conditioning, search_radius=64),
            -candidate.metric,
        ),
    )
    if _best_alternation_matches(best_s2.states, center_index=second_conditioning, search_radius=64) >= s_threshold:
        events.append(DetectedEvent(name="S"))
        contributors.append(f"S2={best_s2.mode}")

    r3_event, r3_mode = _best_rate_event_from_candidates(
        candidates,
        center_index=second_conditioning + conditioning_symbols,
        search_radius=16,
        remote_calling_party=remote_calling_party,
        predicate=is_rate_signal_bits,
        name="R3",
    )
    if r3_event is not None and r3_mode is not None:
        events.append(r3_event)
        contributors.append(f"R3={r3_mode}")

    e_event, e_mode = _best_rate_event_from_candidates(
        candidates,
        center_index=second_conditioning + conditioning_symbols + rate_symbols,
        search_radius=16,
        remote_calling_party=remote_calling_party,
        predicate=is_e_sequence_bits,
        name="E",
    )
    if e_event is not None and e_mode is not None:
        events.append(e_event)
        contributors.append(f"E={e_mode}")

    b1_event, b1_mode = _best_b1_from_candidates(
        candidates,
        start_index=second_conditioning + conditioning_symbols + rate_symbols + e_symbols,
        search_radius=16,
        required_count=16,
    )
    if b1_event is not None and b1_mode is not None:
        events.append(b1_event)
        contributors.append(f"B1={b1_mode}")

    return events, contributors


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

    raw_candidates: list[BlindCandidate] = []
    carrier_candidates = [config.rx_carrier_hz + delta for delta in (-8.0, -4.0, -2.0, 0.0, 2.0, 4.0, 8.0)]
    for carrier_hz in carrier_candidates:
        for timing_offset in range(config.samples_per_symbol):
            points = _recover_raw_points(
                waveform,
                matched_filter_taps=matched_filter_taps,
                config=config,
                timing_offset=timing_offset,
                carrier_hz=carrier_hz,
            )
            raw_states = [_nearest_state(point)[0] for point in points]
            raw_metric = sum(_nearest_state(point)[1] for point in points)
            raw_candidates.append(
                BlindCandidate(
                    mode=f"raw@{carrier_hz:.1f}/{timing_offset}",
                    states=raw_states,
                    metric=raw_metric,
                )
            )

    raw_event_map = {
        candidate.mode: _detect_events_from_startup_windows(candidate.states, remote_calling_party=False)
        for candidate in raw_candidates
    }
    best_single = raw_candidates[0]
    best_single_events = raw_event_map[best_single.mode]
    for candidate in raw_candidates[1:]:
        events = raw_event_map[candidate.mode]
        if (len(events), -candidate.metric) > (len(best_single_events), -best_single.metric):
            best_single = candidate
            best_single_events = events

    def top_modes_by(key_fn, count: int) -> list[str]:
        return [candidate.mode for candidate in sorted(raw_candidates, key=key_fn, reverse=True)[:count]]

    seed_modes = set()
    seed_modes.update(
        top_modes_by(
            lambda candidate: (
                len(raw_event_map[candidate.mode]),
                -candidate.metric,
            ),
            4,
        )
    )
    seed_modes.update(top_modes_by(lambda candidate: -candidate.metric, 4))
    seed_modes.update(
        top_modes_by(
            lambda candidate: _best_alternation_matches(candidate.states, center_index=0, search_radius=64),
            4,
        )
    )
    seed_modes.update(
        top_modes_by(
            lambda candidate: _best_alternation_matches(candidate.states, center_index=1552 + 16, search_radius=64),
            4,
        )
    )

    seed_candidates = [candidate for candidate in raw_candidates if candidate.mode in seed_modes]
    candidate_bank: dict[str, BlindCandidate] = {candidate.mode: candidate for candidate in seed_candidates}
    for seed in seed_candidates:
        carrier_hz = float(seed.mode.split("@", 1)[1].split("/", 1)[0])
        timing_offset = int(seed.mode.rsplit("/", 1)[1])
        points = _recover_raw_points(
            waveform,
            matched_filter_taps=matched_filter_taps,
            config=config,
            timing_offset=timing_offset,
            carrier_hz=carrier_hz,
        )
        eq7 = _equalize_points(points, tap_count=7, step_size=0.0015)
        eq9 = _equalize_points(points, tap_count=9, step_size=0.001)
        dfe9x4 = _equalize_points_dfe(points, feedforward_taps=9, feedback_taps=4, step_size=0.0008)
        candidate_bank[f"{seed.mode}+eq7"] = BlindCandidate(
            mode=f"{seed.mode}+eq7",
            states=eq7.states,
            metric=eq7.metric,
        )
        candidate_bank[f"{seed.mode}+eq9"] = BlindCandidate(
            mode=f"{seed.mode}+eq9",
            states=eq9.states,
            metric=eq9.metric,
        )
        candidate_bank[f"{seed.mode}+dfe9x4"] = BlindCandidate(
            mode=f"{seed.mode}+dfe9x4",
            states=dfe9x4.states,
            metric=dfe9x4.metric,
        )

    bank_events, contributors = _detect_events_from_candidate_bank(
        list(candidate_bank.values()),
        remote_calling_party=False,
    )

    bank_metric = min(candidate.metric for candidate in candidate_bank.values())
    if (len(bank_events), -bank_metric) > (len(best_single_events), -best_single.metric):
        best_mode = "bank[" + ",".join(contributors) + "]"
        best_events = bank_events
        best_metric = bank_metric
    else:
        best_mode = best_single.mode
        best_events = best_single_events
        best_metric = best_single.metric

    return StartupRecovery(
        mode=best_mode,
        metric=best_metric,
        events=best_events,
    )
