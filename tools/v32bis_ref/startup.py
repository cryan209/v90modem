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
from .spec_policy import (
    startup_diff_state_from_final_trn_symbol,
    startup_scrambler_register_from_trn,
)
from .training import ConditioningSignal, generate_conditioning_signal


@dataclass(frozen=True)
class StartupSegment:
    name: str
    kind: str
    tx_calling_party: bool
    symbols: list[str] | None = None
    bits: list[int] | None = None
    repetitions: int | None = None
    bit_rate: int | None = None


def _encode_states_as_labels(
    bits: list[int],
    calling_party: bool,
    initial_diff_state: int,
    initial_scrambler_register: int,
) -> list[str]:
    encoded = encode_rate_sequence_bits(
        bits,
        calling_party=calling_party,
        initial_diff_state=initial_diff_state,
        initial_scrambler_register=initial_scrambler_register,
    )
    return [f"Q{state}" for state in encoded.differential_states]


def _conditioning_segment(calling_party: bool, conditioning: ConditioningSignal) -> StartupSegment:
    return StartupSegment(
        name="conditioning",
        kind="conditioning",
        tx_calling_party=calling_party,
        symbols=conditioning.symbols,
    )


def _rate_segment(
    name: str,
    rate_mask: int,
    *,
    calling_party: bool,
    initial_diff_state: int,
    initial_scrambler_register: int,
    repetitions: int,
) -> StartupSegment:
    bits = rate_signal_bits(rate_mask)
    one_sequence = _encode_states_as_labels(bits, calling_party, initial_diff_state, initial_scrambler_register)
    return StartupSegment(
        name=name,
        kind="rate_signal",
        tx_calling_party=calling_party,
        bits=bits,
        symbols=one_sequence * repetitions,
        repetitions=repetitions,
    )


def _e_segment(
    selected_rate: int,
    *,
    calling_party: bool,
    initial_diff_state: int,
    initial_scrambler_register: int,
) -> StartupSegment:
    bits = e_sequence_bits(selected_rate)
    return StartupSegment(
        name="E",
        kind="sequence_e",
        tx_calling_party=calling_party,
        bit_rate=selected_rate,
        bits=bits,
        symbols=_encode_states_as_labels(bits, calling_party, initial_diff_state, initial_scrambler_register),
    )


def _b1_segment(bit_rate: int, symbols: int, *, calling_party: bool) -> StartupSegment:
    return StartupSegment(
        name="B1",
        kind="scrambled_ones",
        tx_calling_party=calling_party,
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
    spec_derived_startup_state: bool = True,
) -> list[StartupSegment]:
    """Generate the caller's transmitter-side start-up trace after detecting R1."""

    conditioning = generate_conditioning_signal(True, trn_length)
    initial_diff_state = 1
    initial_scrambler_register = 0
    if spec_derived_startup_state:
        initial_diff_state = startup_diff_state_from_final_trn_symbol(conditioning.final_trn_symbol)
        initial_scrambler_register = startup_scrambler_register_from_trn(conditioning.trn_final_scrambler_register)
    return [
        StartupSegment(name="S_NT", kind="s_hold", tx_calling_party=True),
        _conditioning_segment(calling_party=True, conditioning=conditioning),
        _rate_segment(
            "R2",
            r2_mask & r1_mask,
            calling_party=True,
            initial_diff_state=initial_diff_state,
            initial_scrambler_register=initial_scrambler_register,
            repetitions=r2_repetitions,
        ),
        _e_segment(
            r3_selected_rate,
            calling_party=True,
            initial_diff_state=initial_diff_state,
            initial_scrambler_register=initial_scrambler_register,
        ),
        _b1_segment(r3_selected_rate, b1_symbols, calling_party=True),
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
    spec_derived_startup_state: bool = True,
) -> list[StartupSegment]:
    """Generate the answerer's transmitter-side start-up trace from section 6.2."""

    r1_conditioning = generate_conditioning_signal(False, trn_length)
    r3_conditioning = generate_conditioning_signal(False, trn_length)
    r1_initial_diff_state = 1
    r3_initial_diff_state = 1
    r1_initial_scrambler_register = 0
    r3_initial_scrambler_register = 0
    if spec_derived_startup_state:
        r1_initial_diff_state = startup_diff_state_from_final_trn_symbol(r1_conditioning.final_trn_symbol)
        r3_initial_diff_state = startup_diff_state_from_final_trn_symbol(r3_conditioning.final_trn_symbol)
        r1_initial_scrambler_register = startup_scrambler_register_from_trn(r1_conditioning.trn_final_scrambler_register)
        r3_initial_scrambler_register = startup_scrambler_register_from_trn(r3_conditioning.trn_final_scrambler_register)
    return [
        _conditioning_segment(calling_party=False, conditioning=r1_conditioning),
        _rate_segment(
            "R1",
            r1_mask,
            calling_party=False,
            initial_diff_state=r1_initial_diff_state,
            initial_scrambler_register=r1_initial_scrambler_register,
            repetitions=r1_repetitions,
        ),
        _conditioning_segment(calling_party=False, conditioning=r3_conditioning),
        _rate_segment(
            "R3",
            r2_mask,
            calling_party=False,
            initial_diff_state=r3_initial_diff_state,
            initial_scrambler_register=r3_initial_scrambler_register,
            repetitions=r3_repetitions,
        ),
        _e_segment(
            r3_selected_rate,
            calling_party=False,
            initial_diff_state=r3_initial_diff_state,
            initial_scrambler_register=r3_initial_scrambler_register,
        ),
        _b1_segment(r3_selected_rate, b1_symbols, calling_party=False),
    ]
