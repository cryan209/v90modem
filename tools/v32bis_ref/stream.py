"""Continuous observable symbol streams for V.32bis startup traces."""

from __future__ import annotations

from dataclasses import dataclass
import random

from .startup import StartupSegment


@dataclass(frozen=True)
class ObservableSymbol:
    """A single logical receive-side symbol observation."""

    symbol: str
    source_name: str
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
                        source_instance=source_instance,
                        tx_calling_party=segment.tx_calling_party,
                        selected_rate=segment.bit_rate,
                    )
                )
    return stream


def impair_stream(
    stream: list[ObservableSymbol],
    *,
    replacements: dict[int, str] | None = None,
    drops: set[int] | None = None,
) -> list[ObservableSymbol]:
    """Apply simple symbol-level impairments to an observable stream.

    `replacements` maps absolute symbol indexes to replacement symbol strings.
    `drops` removes absolute symbol indexes from the stream.
    """

    replacements = replacements or {}
    drops = drops or set()

    impaired: list[ObservableSymbol] = []
    for index, observable in enumerate(stream):
        if index in drops:
            continue
        if index in replacements:
            impaired.append(
                ObservableSymbol(
                    symbol=replacements[index],
                    source_name=observable.source_name,
                    source_instance=observable.source_instance,
                    tx_calling_party=observable.tx_calling_party,
                    selected_rate=observable.selected_rate,
                )
            )
        else:
            impaired.append(observable)
    return impaired


def impair_stream_burst(
    stream: list[ObservableSymbol],
    *,
    start: int,
    length: int,
    replacement: str = "X",
) -> list[ObservableSymbol]:
    """Replace a contiguous burst of symbols with a fixed invalid symbol."""

    if length < 0:
        raise ValueError("length must be non-negative")
    replacements = {index: replacement for index in range(start, start + length)}
    return impair_stream(stream, replacements=replacements)


def impair_stream_random(
    stream: list[ObservableSymbol],
    *,
    replacement_choices: tuple[str, ...] = ("X", "Q9"),
    flip_probability: float,
    seed: int,
) -> list[ObservableSymbol]:
    """Apply deterministic random symbol substitutions across a stream."""

    if not 0.0 <= flip_probability <= 1.0:
        raise ValueError("flip_probability must be in range 0..1")
    rng = random.Random(seed)
    replacements: dict[int, str] = {}
    for index, _observable in enumerate(stream):
        if rng.random() < flip_probability:
            replacements[index] = replacement_choices[rng.randrange(len(replacement_choices))]
    return impair_stream(stream, replacements=replacements)
