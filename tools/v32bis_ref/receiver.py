"""Logical receive-side event detection for V.32bis startup symbol streams."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from .negotiation import (
    decode_e_rate,
    detect_repeated_rate_signal,
    detect_s_sequence,
)
from .stream import ObservableSymbol


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
        self._rate_sequences: dict[str, list[list[int]]] = {}
        self._emitted_instances: set[tuple[str, int]] = set()

    def ingest(self, observable: ObservableSymbol) -> list[DetectedEvent]:
        events: list[DetectedEvent] = []
        instance_key = (observable.source_kind, observable.source_instance)

        if observable.source_kind in {"conditioning", "s_hold"}:
            self._recent_symbols.append(observable.symbol)
            if (
                instance_key not in self._emitted_instances
                and len(self._recent_symbols) == 256
                and detect_s_sequence(list(self._recent_symbols))
            ):
                self._emitted_instances.add(instance_key)
                events.append(DetectedEvent(name="S"))
            return events

        if observable.source_kind == "rate_signal" and observable.word_bits is not None and observable.word_index is not None:
            rate_key = f"{observable.source_name}:{observable.source_instance}"
            sequences = self._rate_sequences.setdefault(rate_key, [])
            while len(sequences) <= observable.word_index:
                sequences.append(list(observable.word_bits))
            if observable.word_index >= 1:
                rate_mask = detect_repeated_rate_signal(sequences[: observable.word_index + 1])
                if rate_mask is not None and instance_key not in self._emitted_instances and observable.word_index == 1:
                    self._emitted_instances.add(instance_key)
                    events.append(
                        DetectedEvent(
                            name=observable.source_name,
                            rate_mask=rate_mask,
                            repetitions=observable.word_index + 1,
                        )
                    )
            return events

        if observable.source_kind == "sequence_e" and observable.word_bits is not None and instance_key not in self._emitted_instances:
            self._emitted_instances.add(instance_key)
            events.append(
                DetectedEvent(
                    name="E",
                    selected_rate=decode_e_rate(list(observable.word_bits)),
                )
            )
            return events

        if observable.source_kind == "scrambled_ones" and instance_key not in self._emitted_instances:
            self._emitted_instances.add(instance_key)
            events.append(
                DetectedEvent(
                    name="B1",
                    selected_rate=observable.selected_rate,
                )
            )
        return events

    def ingest_all(self, stream: list[ObservableSymbol]) -> list[DetectedEvent]:
        events: list[DetectedEvent] = []
        for observable in stream:
            events.extend(self.ingest(observable))
        return events
