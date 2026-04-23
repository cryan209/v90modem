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
        if segment.kind in {"conditioning", "s_hold", "aa", "cc", "ac", "ca"} and segment.symbols:
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


def impair_stream_q_neighbor(
    stream: list[ObservableSymbol],
    *,
    every_nth_q: int,
) -> list[ObservableSymbol]:
    """Confuse every Nth observed Q-state with a nearby Q-state.

    This is a crude symbol-decision model: `Q0->Q1`, `Q1->Q2`, `Q2->Q3`,
    `Q3->Q2`.
    """

    if every_nth_q <= 0:
        raise ValueError("every_nth_q must be positive")

    replacements: dict[int, str] = {}
    q_seen = 0
    neighbor_map = {
        "Q0": "Q1",
        "Q1": "Q2",
        "Q2": "Q3",
        "Q3": "Q2",
    }

    for index, observable in enumerate(stream):
        if observable.symbol.startswith("Q"):
            q_seen += 1
            if q_seen % every_nth_q == 0 and observable.symbol in neighbor_map:
                replacements[index] = neighbor_map[observable.symbol]

    return impair_stream(stream, replacements=replacements)


def impair_stream_insert(
    stream: list[ObservableSymbol],
    *,
    index: int,
    symbol: str,
) -> list[ObservableSymbol]:
    """Insert one observable symbol into the stream at the given index."""

    if not 0 <= index <= len(stream):
        raise ValueError("index out of range")
    if not stream:
        raise ValueError("cannot insert into an empty stream")

    template = stream[min(index, len(stream) - 1)]
    inserted = ObservableSymbol(
        symbol=symbol,
        source_name=template.source_name,
        source_instance=template.source_instance,
        tx_calling_party=template.tx_calling_party,
        selected_rate=template.selected_rate,
    )
    return stream[:index] + [inserted] + stream[index:]
