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
    StartupTransmitState,
    startup_state_from_trn,
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
    initial_tx_state: StartupTransmitState | None = None
    final_tx_state: StartupTransmitState | None = None


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


def _advance_startup_state(
    bits: list[int],
    *,
    calling_party: bool,
    initial_tx_state: StartupTransmitState,
    repetitions: int = 1,
) -> tuple[list[str], StartupTransmitState]:
    symbols: list[str] = []
    state = initial_tx_state
    for _ in range(repetitions):
        encoded = encode_rate_sequence_bits(
            bits,
            calling_party=calling_party,
            initial_diff_state=state.diff_state,
            initial_scrambler_register=state.scrambler_register,
        )
        symbols.extend(f"Q{diff_state}" for diff_state in encoded.differential_states)
        state = StartupTransmitState(
            scrambler_register=encoded.final_scrambler_register,
            diff_state=encoded.final_state,
            convolution_state=state.convolution_state,
        )
    return symbols, state


def _rate_segment(
    name: str,
    rate_mask: int,
    *,
    calling_party: bool,
    initial_tx_state: StartupTransmitState,
    repetitions: int,
) -> StartupSegment:
    bits = rate_signal_bits(rate_mask)
    one_word_symbols, final_tx_state = _advance_startup_state(
        bits,
        calling_party=calling_party,
        initial_tx_state=initial_tx_state,
        repetitions=1,
    )
    return StartupSegment(
        name=name,
        kind="rate_signal",
        tx_calling_party=calling_party,
        bits=bits,
        symbols=one_word_symbols * repetitions,
        repetitions=repetitions,
        initial_tx_state=initial_tx_state,
        final_tx_state=final_tx_state,
    )


def _e_segment(
    selected_rate: int,
    *,
    calling_party: bool,
    initial_tx_state: StartupTransmitState,
) -> StartupSegment:
    bits = e_sequence_bits(selected_rate)
    symbols, final_tx_state = _advance_startup_state(
        bits,
        calling_party=calling_party,
        initial_tx_state=initial_tx_state,
        repetitions=1,
    )
    return StartupSegment(
        name="E",
        kind="sequence_e",
        tx_calling_party=calling_party,
        bit_rate=selected_rate,
        bits=bits,
        symbols=symbols,
        initial_tx_state=initial_tx_state,
        final_tx_state=final_tx_state,
    )


def _b1_segment(
    bit_rate: int,
    symbols: int,
    *,
    calling_party: bool,
    initial_tx_state: StartupTransmitState,
) -> StartupSegment:
    return StartupSegment(
        name="B1",
        kind="scrambled_ones",
        tx_calling_party=calling_party,
        bit_rate=bit_rate,
        repetitions=symbols,
        initial_tx_state=initial_tx_state,
        final_tx_state=initial_tx_state,
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
    startup_state = StartupTransmitState(scrambler_register=0, diff_state=1)
    if spec_derived_startup_state:
        startup_state = startup_state_from_trn(
            conditioning.final_trn_symbol,
            conditioning.trn_final_scrambler_register,
        )
    r2_segment = _rate_segment(
        "R2",
        r2_mask & r1_mask,
        calling_party=True,
        initial_tx_state=startup_state,
        repetitions=r2_repetitions,
    )
    e_segment = _e_segment(
        r3_selected_rate,
        calling_party=True,
        initial_tx_state=startup_state,
    )
    b1_state = StartupTransmitState(
        scrambler_register=(e_segment.final_tx_state or startup_state).scrambler_register,
        diff_state=(e_segment.final_tx_state or startup_state).diff_state,
        convolution_state=0,
    )
    return [
        StartupSegment(name="S_NT", kind="s_hold", tx_calling_party=True),
        _conditioning_segment(calling_party=True, conditioning=conditioning),
        r2_segment,
        e_segment,
        _b1_segment(r3_selected_rate, b1_symbols, calling_party=True, initial_tx_state=b1_state),
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
    r1_startup_state = StartupTransmitState(scrambler_register=0, diff_state=1)
    r3_startup_state = StartupTransmitState(scrambler_register=0, diff_state=1)
    if spec_derived_startup_state:
        r1_startup_state = startup_state_from_trn(
            r1_conditioning.final_trn_symbol,
            r1_conditioning.trn_final_scrambler_register,
        )
        r3_startup_state = startup_state_from_trn(
            r3_conditioning.final_trn_symbol,
            r3_conditioning.trn_final_scrambler_register,
        )
    r1_segment = _rate_segment(
        "R1",
        r1_mask,
        calling_party=False,
        initial_tx_state=r1_startup_state,
        repetitions=r1_repetitions,
    )
    r3_segment = _rate_segment(
        "R3",
        r2_mask,
        calling_party=False,
        initial_tx_state=r3_startup_state,
        repetitions=r3_repetitions,
    )
    e_segment = _e_segment(
        r3_selected_rate,
        calling_party=False,
        initial_tx_state=r3_startup_state,
    )
    b1_state = StartupTransmitState(
        scrambler_register=(e_segment.final_tx_state or r3_startup_state).scrambler_register,
        diff_state=(e_segment.final_tx_state or r3_startup_state).diff_state,
        convolution_state=0,
    )
    return [
        _conditioning_segment(calling_party=False, conditioning=r1_conditioning),
        r1_segment,
        _conditioning_segment(calling_party=False, conditioning=r3_conditioning),
        r3_segment,
        e_segment,
        _b1_segment(r3_selected_rate, b1_symbols, calling_party=False, initial_tx_state=b1_state),
    ]
