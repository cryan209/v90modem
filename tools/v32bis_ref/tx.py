"""Reference V.32bis symbol-domain transmitter."""

from __future__ import annotations

from dataclasses import dataclass

from .constellation import symbol_to_point
from .stream import ObservableSymbol, flatten_startup_trace
from .training import STATE_A, STATE_B, STATE_C, STATE_D


SYNC_STATE_TO_INDEX = {
    STATE_A: 0,
    STATE_B: 1,
    STATE_C: 3,
    STATE_D: 2,
}


@dataclass(frozen=True)
class TransmittedSymbol:
    """One transmitted complex symbol and its source metadata."""

    point: complex
    source_name: str
    source_instance: int
    symbol: str
    bit_rate: int
    tx_calling_party: bool
    selected_rate: int | None = None


def startup_symbol_to_point(symbol: str) -> complex:
    """Map a startup/training symbol label to its exact complex point."""

    if symbol in SYNC_STATE_TO_INDEX:
        re, im = symbol_to_point(4800, SYNC_STATE_TO_INDEX[symbol])
        return complex(re, im)

    if symbol == "B1":
        re, im = symbol_to_point(4800, SYNC_STATE_TO_INDEX[STATE_C])
        return complex(re, im)

    if symbol.startswith("Q"):
        index = int(symbol[1:])
        re, im = symbol_to_point(4800, index)
        return complex(re, im)

    raise ValueError(f"unsupported startup symbol label: {symbol}")


def observable_to_transmitted_symbol(observable: ObservableSymbol) -> TransmittedSymbol:
    point = startup_symbol_to_point(observable.symbol)
    return TransmittedSymbol(
        point=point,
        source_name=observable.source_name,
        source_instance=observable.source_instance,
        symbol=observable.symbol,
        bit_rate=4800,
        tx_calling_party=observable.tx_calling_party,
        selected_rate=observable.selected_rate,
    )


def startup_trace_to_complex_symbols(trace) -> list[TransmittedSymbol]:
    """Flatten a startup trace and map it to exact complex symbols."""

    return [observable_to_transmitted_symbol(observable) for observable in flatten_startup_trace(trace)]
