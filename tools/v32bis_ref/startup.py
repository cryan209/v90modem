"""V.32bis startup trace generator.

This module models the transmitter-side ordering of the main start-up exchanges
from section 6. It is intentionally a logical trace, not a waveform model.
"""

from __future__ import annotations

from dataclasses import dataclass

from .rate_signal import (
    encode_rate_sequence_bits,
    e_sequence_bits,
    rate_signal_bits,
)
from .training import ConditioningSignal, generate_conditioning_signal


@dataclass(frozen=True)
class StartupSegment:
    name: str
    kind: str
    symbols: list[str] | None = None
    bits: list[int] | None = None
    repetitions: int | None = None
    bit_rate: int | None = None


def _encode_states_as_labels(bits: list[int], calling_party: bool, initial_diff_state: int) -> list[str]:
    encoded = encode_rate_sequence_bits(
        bits,
        calling_party=calling_party,
        initial_diff_state=initial_diff_state,
    )
    return [f"Q{state}" for state in encoded.differential_states]


def _conditioning_segment(calling_party: bool, trn_length: int) -> StartupSegment:
    conditioning = generate_conditioning_signal(calling_party, trn_length)
    return StartupSegment(
        name="conditioning",
        kind="conditioning",
        symbols=conditioning.symbols,
    )


def _rate_segment(name: str, rate_mask: int, *, calling_party: bool, initial_diff_state: int, repetitions: int) -> StartupSegment:
    bits = rate_signal_bits(rate_mask)
    one_sequence = _encode_states_as_labels(bits, calling_party, initial_diff_state)
    return StartupSegment(
        name=name,
        kind="rate_signal",
        bits=bits,
        symbols=one_sequence * repetitions,
        repetitions=repetitions,
    )


def _e_segment(selected_rate: int, *, calling_party: bool, initial_diff_state: int) -> StartupSegment:
    bits = e_sequence_bits(selected_rate)
    return StartupSegment(
        name="E",
        kind="sequence_e",
        bit_rate=selected_rate,
        bits=bits,
        symbols=_encode_states_as_labels(bits, calling_party, initial_diff_state),
    )


def _b1_segment(bit_rate: int, symbols: int) -> StartupSegment:
    return StartupSegment(
        name="B1",
        kind="scrambled_ones",
        bit_rate=bit_rate,
        repetitions=symbols,
    )


def generate_call_startup_trace(
    *,
    r1_mask: int,
    r2_mask: int,
    r3_selected_rate: int,
    trn_length: int = 1280,
    r2_repetitions: int = 2,
    b1_symbols: int = 128,
) -> list[StartupSegment]:
    """Generate the caller's transmitter-side start-up trace after detecting R1."""

    return [
        StartupSegment(name="S_NT", kind="s_hold"),
        _conditioning_segment(calling_party=True, trn_length=trn_length),
        _rate_segment("R2", r2_mask & r1_mask, calling_party=True, initial_diff_state=1, repetitions=r2_repetitions),
        _e_segment(r3_selected_rate, calling_party=True, initial_diff_state=1),
        _b1_segment(r3_selected_rate, b1_symbols),
    ]


def generate_answer_startup_trace(
    *,
    r1_mask: int,
    r2_mask: int,
    r3_selected_rate: int,
    trn_length: int = 1280,
    r1_repetitions: int = 2,
    r3_repetitions: int = 2,
    b1_symbols: int = 128,
) -> list[StartupSegment]:
    """Generate the answerer's transmitter-side start-up trace from section 6.2."""

    return [
        _conditioning_segment(calling_party=False, trn_length=trn_length),
        _rate_segment("R1", r1_mask, calling_party=False, initial_diff_state=1, repetitions=r1_repetitions),
        _conditioning_segment(calling_party=False, trn_length=trn_length),
        _rate_segment("R3", r2_mask, calling_party=False, initial_diff_state=1, repetitions=r3_repetitions),
        _e_segment(r3_selected_rate, calling_party=False, initial_diff_state=1),
        _b1_segment(r3_selected_rate, b1_symbols),
    ]
