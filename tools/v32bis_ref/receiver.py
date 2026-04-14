"""Logical receive-side event detection for V.32bis startup symbol streams."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from .negotiation import (
    decode_e_rate,
    detect_repeated_rate_signal,
    detect_s_sequence,
)
from .rate_signal import decode_rate_sequence_symbols
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
        self._rate_word_symbols: dict[str, list[str]] = {}
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

        if observable.source_kind == "rate_signal":
            rate_key = f"{observable.source_name}:{observable.source_instance}"
            symbol_run = self._rate_word_symbols.setdefault(rate_key, [])
            symbol_run.append(observable.symbol)
            if len(symbol_run) == 8:
                sequences = self._rate_sequences.setdefault(rate_key, [])
                decoded_bits = decode_rate_sequence_symbols(
                    symbol_run,
                    calling_party=observable.tx_calling_party,
                    initial_diff_state=1,
                )
                sequences.append(decoded_bits)
                self._rate_word_symbols[rate_key] = []
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

        if observable.source_kind == "sequence_e" and instance_key not in self._emitted_instances:
            rate_key = f"{observable.source_name}:{observable.source_instance}"
            symbol_run = self._rate_word_symbols.setdefault(rate_key, [])
            symbol_run.append(observable.symbol)
            if len(symbol_run) == 8:
                decoded_bits = decode_rate_sequence_symbols(
                    symbol_run,
                    calling_party=observable.tx_calling_party,
                    initial_diff_state=1,
                )
                self._rate_word_symbols[rate_key] = []
                self._emitted_instances.add(instance_key)
                events.append(
                    DetectedEvent(
                        name="E",
                        selected_rate=decode_e_rate(decoded_bits),
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
