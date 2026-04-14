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
    tx_calling_party: bool
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
                        tx_calling_party=segment.tx_calling_party,
                    )
                )
            continue

        if segment.symbols and segment.kind in {"rate_signal", "sequence_e"}:
            for symbol in segment.symbols:
                stream.append(
                    ObservableSymbol(
                        symbol=symbol,
                        source_name=segment.name,
                        source_kind="q_run",
                        source_instance=source_instance,
                        tx_calling_party=segment.tx_calling_party,
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
                        tx_calling_party=segment.tx_calling_party,
                        selected_rate=segment.bit_rate,
                    )
                )
    return stream
