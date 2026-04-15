"""Logical receive-side event detection for V.32bis startup symbol streams."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from .negotiation import (
    decode_e_rate,
    detect_repeated_rate_signal,
    detect_s_sequence,
)
from .rate_signal import (
    decode_rate_sequence_symbols,
    is_e_sequence_bits,
    is_rate_signal_bits,
)
from .stream import ObservableSymbol
from .training import STATE_A, STATE_B


@dataclass(frozen=True)
class DetectedEvent:
    name: str
    rate_mask: int | None = None
    selected_rate: int | None = None
    repetitions: int | None = None


@dataclass(frozen=True)
class ReceiverStats:
    q_candidates_tested: int
    q_invalid_candidates: int
    q_resync_shifts: int
    q_valid_words: int
    r_words_detected: int
    e_words_detected: int


class V32bisLogicalReceiver:
    """Consume a continuous symbol stream and emit logical protocol events."""

    def __init__(self) -> None:
        self._recent_symbols: deque[str] = deque(maxlen=256)
        self._q_run_symbols: deque[str] = deque()
        self._rate_sequences: list[list[int]] = []
        self._b1_run_length = 0
        self._in_s_run = False
        self._b1_emitted_current_run = False
        self._rate_signal_count = 0
        self._seen_e_in_run = False
        self._q_candidates_tested = 0
        self._q_invalid_candidates = 0
        self._q_resync_shifts = 0
        self._q_valid_words = 0
        self._r_words_detected = 0
        self._e_words_detected = 0

    def _reset_q_run(self) -> None:
        self._q_run_symbols.clear()
        self._rate_sequences.clear()
        self._seen_e_in_run = False

    def _rate_event_name(self, observable: ObservableSymbol) -> str:
        if observable.source_name in {"R1", "R2", "R3"}:
            return observable.source_name
        self._rate_signal_count += 1
        if self._rate_signal_count == 1:
            return "R1"
        if self._rate_signal_count == 2:
            return "R3"
        return f"R{self._rate_signal_count}"

    def ingest(self, observable: ObservableSymbol) -> list[DetectedEvent]:
        events: list[DetectedEvent] = []

        if observable.symbol not in {STATE_A, STATE_B, "B1"} and not observable.symbol.startswith("Q"):
            self._reset_q_run()
            self._recent_symbols.clear()
            self._in_s_run = False
            self._b1_run_length = 0
            self._b1_emitted_current_run = False
            return events

        if observable.symbol in {STATE_A, STATE_B}:
            self._reset_q_run()
            self._b1_run_length = 0
            self._b1_emitted_current_run = False
            self._recent_symbols.append(observable.symbol)
            detected = len(self._recent_symbols) == 256 and detect_s_sequence(list(self._recent_symbols))
            if detected and not self._in_s_run:
                self._in_s_run = True
                events.append(DetectedEvent(name="S"))
            elif not detected:
                self._in_s_run = False
            return events

        if observable.symbol.startswith("Q"):
            self._recent_symbols.clear()
            self._in_s_run = False
            self._b1_run_length = 0
            self._b1_emitted_current_run = False
            self._q_run_symbols.append(observable.symbol)
            while len(self._q_run_symbols) >= 8:
                candidate = list(self._q_run_symbols)[:8]
                self._q_candidates_tested += 1
                try:
                    decoded_bits = decode_rate_sequence_symbols(
                        candidate,
                        calling_party=observable.tx_calling_party,
                        initial_diff_state=1,
                    )
                except ValueError:
                    self._q_invalid_candidates += 1
                    self._q_resync_shifts += 1
                    self._q_run_symbols.popleft()
                    continue
                if is_e_sequence_bits(decoded_bits) and not self._seen_e_in_run:
                    self._q_valid_words += 1
                    self._e_words_detected += 1
                    for _ in range(8):
                        self._q_run_symbols.popleft()
                    self._seen_e_in_run = True
                    events.append(
                        DetectedEvent(
                            name="E",
                            selected_rate=decode_e_rate(decoded_bits),
                        )
                    )
                    break
                if is_rate_signal_bits(decoded_bits):
                    self._q_valid_words += 1
                    for _ in range(8):
                        self._q_run_symbols.popleft()
                    self._rate_sequences.append(decoded_bits)
                    self._r_words_detected += 1
                    if len(self._rate_sequences) >= 2:
                        rate_mask = detect_repeated_rate_signal(self._rate_sequences[-2:])
                        if rate_mask is not None:
                            name = self._rate_event_name(observable)
                            self._rate_sequences.clear()
                            events.append(
                                DetectedEvent(
                                    name=name,
                                    rate_mask=rate_mask,
                                    repetitions=2,
                                )
                            )
                    continue
                self._q_invalid_candidates += 1
                self._q_resync_shifts += 1
                self._q_run_symbols.popleft()
            return events

        if observable.symbol == "B1":
            self._reset_q_run()
            self._recent_symbols.clear()
            self._in_s_run = False
            self._b1_run_length += 1
            if self._b1_run_length >= 24 and not self._b1_emitted_current_run:
                self._b1_emitted_current_run = True
                events.append(
                    DetectedEvent(
                        name="B1",
                        selected_rate=observable.selected_rate,
                        repetitions=self._b1_run_length,
                    )
                )
        return events

    def ingest_all(self, stream: list[ObservableSymbol]) -> list[DetectedEvent]:
        events: list[DetectedEvent] = []
        for observable in stream:
            events.extend(self.ingest(observable))
        return events

    def stats(self) -> ReceiverStats:
        return ReceiverStats(
            q_candidates_tested=self._q_candidates_tested,
            q_invalid_candidates=self._q_invalid_candidates,
            q_resync_shifts=self._q_resync_shifts,
            q_valid_words=self._q_valid_words,
            r_words_detected=self._r_words_detected,
            e_words_detected=self._e_words_detected,
        )
