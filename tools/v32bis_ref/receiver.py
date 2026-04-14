"""Logical receive-side event detection for V.32bis startup traces."""

from __future__ import annotations

from dataclasses import dataclass

from .negotiation import (
    decode_e_rate,
    detect_repeated_rate_signal,
    detect_s_sequence,
)
from .startup import StartupSegment


@dataclass(frozen=True)
class DetectedEvent:
    name: str
    rate_mask: int | None = None
    selected_rate: int | None = None
    repetitions: int | None = None


class V32bisLogicalReceiver:
    """Consume logical startup segments and emit detected protocol events."""

    def detect(self, segment: StartupSegment) -> list[DetectedEvent]:
        events: list[DetectedEvent] = []

        if segment.kind == "s_hold":
            events.append(DetectedEvent(name="S"))
            return events

        if segment.kind == "conditioning" and segment.symbols and detect_s_sequence(segment.symbols):
            events.append(DetectedEvent(name="S"))
            return events

        if segment.kind == "rate_signal" and segment.bits is not None and segment.repetitions is not None:
            sequences = [segment.bits for _ in range(segment.repetitions)]
            rate_mask = detect_repeated_rate_signal(sequences)
            if rate_mask is not None:
                events.append(
                    DetectedEvent(
                        name=segment.name,
                        rate_mask=rate_mask,
                        repetitions=segment.repetitions,
                    )
                )
            return events

        if segment.kind == "sequence_e" and segment.bits is not None:
            events.append(
                DetectedEvent(
                    name="E",
                    selected_rate=decode_e_rate(segment.bits),
                )
            )
            return events

        if segment.kind == "scrambled_ones":
            events.append(
                DetectedEvent(
                    name="B1",
                    selected_rate=segment.bit_rate,
                    repetitions=segment.repetitions,
                )
            )
        return events
