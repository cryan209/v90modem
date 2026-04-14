"""V.32bis scrambler and descrambler reference logic."""

from __future__ import annotations

from dataclasses import dataclass


GPC_TAP = 17
GPA_TAP = 4
REGISTER_LENGTH = 23


def scrambler_tap(calling_party: bool, transmit: bool) -> int:
    """Return the active scrambler tap for the given modem direction."""

    if calling_party:
        return GPC_TAP if transmit else GPA_TAP
    return GPA_TAP if transmit else GPC_TAP


@dataclass
class Scrambler:
    """Self-synchronizing V.32bis scrambler."""

    tap: int
    register: int = 0

    def process_bit(self, in_bit: int) -> int:
        in_bit &= 1
        out_bit = (in_bit ^ (self.register >> self.tap) ^ (self.register >> (REGISTER_LENGTH - 1))) & 1
        self.register = ((self.register << 1) | out_bit) & ((1 << REGISTER_LENGTH) - 1)
        return out_bit

    def process_bits(self, bits: list[int] | tuple[int, ...]) -> list[int]:
        return [self.process_bit(bit) for bit in bits]


@dataclass
class Descrambler:
    """Self-synchronizing V.32bis descrambler."""

    tap: int
    register: int = 0

    def process_bit(self, in_bit: int) -> int:
        in_bit &= 1
        out_bit = (in_bit ^ (self.register >> self.tap) ^ (self.register >> (REGISTER_LENGTH - 1))) & 1
        self.register = ((self.register << 1) | in_bit) & ((1 << REGISTER_LENGTH) - 1)
        return out_bit

    def process_bits(self, bits: list[int] | tuple[int, ...]) -> list[int]:
        return [self.process_bit(bit) for bit in bits]
