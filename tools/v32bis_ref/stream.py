"""Continuous observable symbol streams for V.32bis startup traces."""

from __future__ import annotations

from dataclasses import dataclass

from .startup import StartupSegment


@dataclass(frozen=True)
class ObservableSymbol:
    """A single logical receive-side symbol observation."""

    symbol: str
    source_name: str
    source_kind: str
    source_instance: int
    word_bits: tuple[int, ...] | None = None
    word_index: int | None = None
    selected_rate: int | None = None


def flatten_startup_trace(trace: list[StartupSegment]) -> list[ObservableSymbol]:
    """Flatten startup segments into one continuous observable symbol stream."""

    stream: list[ObservableSymbol] = []
    for source_instance, segment in enumerate(trace):
        if segment.kind in {"conditioning", "s_hold"} and segment.symbols:
            for symbol in segment.symbols:
                stream.append(
                    ObservableSymbol(
                        symbol=symbol,
                        source_name=segment.name,
                        source_kind=segment.kind,
                        source_instance=source_instance,
                    )
                )
            continue

        if segment.kind in {"rate_signal", "sequence_e"} and segment.symbols:
            word_len = len(segment.symbols) if segment.kind == "sequence_e" else len(segment.symbols) // (segment.repetitions or 1)
            repetitions = 1 if segment.kind == "sequence_e" else (segment.repetitions or 1)
            word_bits = tuple(segment.bits or [])
            for repetition in range(repetitions):
                start = repetition * word_len
                end = start + word_len
                for symbol in segment.symbols[start:end]:
                    stream.append(
                        ObservableSymbol(
                            symbol=symbol,
                            source_name=segment.name,
                            source_kind=segment.kind,
                            source_instance=source_instance,
                            word_bits=word_bits,
                            word_index=repetition,
                            selected_rate=segment.bit_rate,
                        )
                    )
            continue

        if segment.kind == "scrambled_ones":
            for _ in range(segment.repetitions or 0):
                stream.append(
                    ObservableSymbol(
                        symbol="B1",
                        source_name=segment.name,
                        source_kind=segment.kind,
                        source_instance=source_instance,
                        selected_rate=segment.bit_rate,
                    )
                )
    return stream
