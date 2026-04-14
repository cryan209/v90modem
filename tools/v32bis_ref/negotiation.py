"""Receive-side logical decoding and negotiation helpers for V.32bis."""

from __future__ import annotations

from dataclasses import dataclass

from .rate_signal import (
    RATE_12000,
    RATE_14400,
    RATE_4800,
    RATE_7200,
    RATE_9600,
    list_from_rate_mask,
)
from .training import STATE_A, STATE_B, generate_s_segment


SYNC_RATE_BITS = {0: 0, 1: 0, 2: 0, 3: 0, 7: 1, 11: 0, 15: 0}
SYNC_E_BITS = {0: 1, 1: 1, 2: 1, 3: 1, 4: 1, 7: 1, 8: 1, 11: 1, 13: 0, 14: 0, 15: 1}


def detect_s_sequence(symbols: list[str], min_length: int = 256) -> bool:
    """Return True if the sequence begins with a valid S pattern."""

    if len(symbols) < min_length:
        return False
    expected = generate_s_segment(min_length)
    return symbols[:min_length] == expected


def validate_rate_signal_bits(bits: list[int]) -> bool:
    """Validate the fixed synchronization positions from Table 5."""

    if len(bits) != 16:
        return False
    return all(bits[index] == value for index, value in SYNC_RATE_BITS.items())


def validate_e_sequence_bits(bits: list[int]) -> bool:
    """Validate the fixed synchronization positions from Table 6."""

    if len(bits) != 16:
        return False
    return all(bits[index] == value for index, value in SYNC_E_BITS.items())


def decode_rate_mask(bits: list[int]) -> int:
    if not validate_rate_signal_bits(bits):
        raise ValueError("bits do not form a valid V.32bis rate signal")

    mask = 0
    if bits[5]:
        mask |= RATE_4800
    if bits[6]:
        mask |= RATE_9600
    if bits[9]:
        mask |= RATE_7200
    if bits[10]:
        mask |= RATE_12000
    if bits[12]:
        mask |= RATE_14400
    return mask


def decode_e_rate(bits: list[int]) -> int:
    if not validate_e_sequence_bits(bits):
        raise ValueError("bits do not form a valid V.32bis E sequence")

    selected = []
    if bits[5]:
        selected.append(4800)
    if bits[6]:
        selected.append(9600)
    if bits[9]:
        selected.append(7200)
    if bits[10]:
        selected.append(12000)
    if bits[12]:
        selected.append(14400)
    if len(selected) != 1:
        raise ValueError("E sequence must indicate exactly one data rate")
    return selected[0]


def detect_repeated_rate_signal(sequences: list[list[int]]) -> int | None:
    """Return the decoded rate mask if two consecutive identical R sequences are present."""

    if len(sequences) < 2:
        return None
    first, second = sequences[0], sequences[1]
    if first != second:
        return None
    if not validate_rate_signal_bits(first):
        return None
    return decode_rate_mask(first)


def highest_common_rate(left_mask: int, right_mask: int) -> int | None:
    common = left_mask & right_mask
    for rate in (14400, 12000, 9600, 7200, 4800):
        if rate in list_from_rate_mask(common):
            return rate
    return None


@dataclass(frozen=True)
class NegotiationResult:
    agreed_rate: int | None
    cleardown: bool


def negotiate_startup_rate(r2_mask: int, r3_mask: int) -> NegotiationResult:
    """Select the agreed startup rate from the exchanged R2/R3 capabilities."""

    agreed = highest_common_rate(r2_mask, r3_mask)
    return NegotiationResult(agreed_rate=agreed, cleardown=agreed is None)


def negotiate_renegotiation_rate(r4_mask: int, r5_mask: int) -> NegotiationResult:
    """Select the agreed renegotiated rate from R4/R5."""

    agreed = highest_common_rate(r4_mask, r5_mask)
    return NegotiationResult(agreed_rate=agreed, cleardown=agreed is None)
