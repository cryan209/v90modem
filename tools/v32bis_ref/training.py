"""V.32bis transmitter-side training and conditioning sequences."""

from __future__ import annotations

from dataclasses import dataclass

from .scrambler import Scrambler, scrambler_tap
from .spec_policy import TRN_INITIAL_SCRAMBLER_REGISTER


STATE_A = "A"
STATE_B = "B"
STATE_C = "C"
STATE_D = "D"

TRN_DIRECT_MAP = {
    0b00: STATE_A,
    0b01: STATE_B,
    0b11: STATE_C,
    0b10: STATE_D,
}


def alternating_states(first: str, second: str, length: int) -> list[str]:
    if length < 0:
        raise ValueError("length must be non-negative")
    return [first if i % 2 == 0 else second for i in range(length)]


def generate_s_segment(length: int = 256) -> list[str]:
    """Generate receiver-conditioning segment S = ABAB..."""

    return alternating_states(STATE_A, STATE_B, length)


def generate_s_bar_segment(length: int = 16) -> list[str]:
    """Generate receiver-conditioning segment S-bar = CDCD..."""

    return alternating_states(STATE_C, STATE_D, length)


def generate_trn_bits(calling_party: bool, symbol_count: int) -> list[tuple[int, int]]:
    """Generate the scrambled dibits that underlie the TRN segment.

    V.32bis section 5.2.3 explicitly initializes the TRN scrambler to all
    zeros, so this helper always starts from register 0.
    """

    if symbol_count < 0:
        raise ValueError("symbol_count must be non-negative")
    scrambler = Scrambler(
        scrambler_tap(calling_party, transmit=True),
        register=TRN_INITIAL_SCRAMBLER_REGISTER,
    )
    dibits = []
    for _ in range(symbol_count):
        b0 = scrambler.process_bit(1)
        b1 = scrambler.process_bit(1)
        dibits.append((b0, b1))
    return dibits


def trn_final_scrambler_register(calling_party: bool, symbol_count: int) -> int:
    """Return the scrambler register after emitting `symbol_count` TRN symbols."""

    if symbol_count < 0:
        raise ValueError("symbol_count must be non-negative")
    scrambler = Scrambler(
        scrambler_tap(calling_party, transmit=True),
        register=TRN_INITIAL_SCRAMBLER_REGISTER,
    )
    for _ in range(symbol_count * 2):
        scrambler.process_bit(1)
    return scrambler.register


def generate_trn_segment(calling_party: bool, symbol_count: int = 1280) -> list[str]:
    """Generate the TRN signal-state sequence from section 5.2.3."""

    if symbol_count < 256:
        raise ValueError("TRN segment must be at least 256 symbols long")

    dibits = generate_trn_bits(calling_party, symbol_count)
    states: list[str] = []
    for i, (b0, b1) in enumerate(dibits):
        if i < 256:
            states.append(STATE_A if b0 == 0 else STATE_C)
        else:
            states.append(TRN_DIRECT_MAP[b0 | (b1 << 1)])
    return states


@dataclass(frozen=True)
class ConditioningSignal:
    s: list[str]
    s_bar: list[str]
    trn: list[str]
    trn_final_scrambler_register: int

    @property
    def symbols(self) -> list[str]:
        return self.s + self.s_bar + self.trn

    @property
    def final_trn_symbol(self) -> str:
        return self.trn[-1]


def generate_conditioning_signal(calling_party: bool, trn_length: int = 1280) -> ConditioningSignal:
    """Generate the full receiver-conditioning signal from section 5.2."""

    return ConditioningSignal(
        s=generate_s_segment(),
        s_bar=generate_s_bar_segment(),
        trn=generate_trn_segment(calling_party, trn_length),
        trn_final_scrambler_register=trn_final_scrambler_register(calling_party, trn_length),
    )
