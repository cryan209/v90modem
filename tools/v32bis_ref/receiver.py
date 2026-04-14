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


class V32bisLogicalReceiver:
    """Consume a continuous symbol stream and emit logical protocol events."""

    def __init__(self) -> None:
        self._recent_symbols: deque[str] = deque(maxlen=256)
        self._q_run_symbols: dict[int, list[str]] = {}
        self._rate_sequences: dict[int, list[list[int]]] = {}
        self._b1_run_lengths: dict[int, int] = {}
        self._emitted_instances: set[tuple[str, int]] = set()

    def ingest(self, observable: ObservableSymbol) -> list[DetectedEvent]:
        events: list[DetectedEvent] = []
        instance_key = (observable.source_name, observable.source_instance)

        if observable.symbol not in {STATE_A, STATE_B, "B1"} and not observable.symbol.startswith("Q"):
            self._q_run_symbols.pop(observable.source_instance, None)
            return events

        if observable.symbol in {STATE_A, STATE_B}:
            self._recent_symbols.append(observable.symbol)
            if (
                instance_key not in self._emitted_instances
                and len(self._recent_symbols) == 256
                and detect_s_sequence(list(self._recent_symbols))
            ):
                self._emitted_instances.add(instance_key)
                events.append(DetectedEvent(name="S"))
            return events

        if observable.symbol.startswith("Q"):
            run_key = observable.source_instance
            symbol_run = self._q_run_symbols.setdefault(run_key, [])
            symbol_run.append(observable.symbol)
            if len(symbol_run) == 8:
                try:
                    decoded_bits = decode_rate_sequence_symbols(
                        symbol_run,
                        calling_party=observable.tx_calling_party,
                        initial_diff_state=1,
                    )
                except ValueError:
                    self._q_run_symbols[run_key] = []
                    return events
                self._q_run_symbols[run_key] = []
                if is_e_sequence_bits(decoded_bits) and instance_key not in self._emitted_instances:
                    self._emitted_instances.add(instance_key)
                    events.append(
                        DetectedEvent(
                            name="E",
                            selected_rate=decode_e_rate(decoded_bits),
                        )
                    )
                elif is_rate_signal_bits(decoded_bits):
                    sequences = self._rate_sequences.setdefault(run_key, [])
                    sequences.append(decoded_bits)
                    if len(sequences) >= 2:
                        rate_mask = detect_repeated_rate_signal(sequences[:2])
                        if rate_mask is not None and instance_key not in self._emitted_instances:
                            self._emitted_instances.add(instance_key)
                            events.append(
                                DetectedEvent(
                                    name=observable.source_name,
                                    rate_mask=rate_mask,
                                    repetitions=2,
                                )
                            )
            return events

        if observable.symbol == "B1":
            run_key = observable.source_instance
            self._b1_run_lengths[run_key] = self._b1_run_lengths.get(run_key, 0) + 1
            if self._b1_run_lengths[run_key] >= 24 and instance_key not in self._emitted_instances:
                self._emitted_instances.add(instance_key)
                events.append(
                    DetectedEvent(
                        name="B1",
                        selected_rate=observable.selected_rate,
                        repetitions=self._b1_run_lengths[run_key],
                    )
                )
        return events

    def ingest_all(self, stream: list[ObservableSymbol]) -> list[DetectedEvent]:
        events: list[DetectedEvent] = []
        for observable in stream:
            events.extend(self.ingest(observable))
        return events
