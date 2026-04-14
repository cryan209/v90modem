"""Clause-locked V.32bis coding helpers.

This module implements the bit-to-symbol coding path used by V.32bis data
modes. The goal is to make the exact coding logic testable before we add the
rest of the modem stack.
"""

from __future__ import annotations

from dataclasses import dataclass


SUPPORTED_BIT_RATES = (4800, 7200, 9600, 12000, 14400)

BITS_PER_SYMBOL = {
    4800: 2,
    7200: 3,
    9600: 4,
    12000: 5,
    14400: 6,
}

V32BIS_4800_DIFFERENTIAL_ENCODER = (
    (2, 3, 0, 1),
    (0, 2, 1, 3),
    (3, 1, 2, 0),
    (1, 0, 3, 2),
)

TRELLIS_DIFFERENTIAL_ENCODER = (
    (0, 1, 2, 3),
    (1, 2, 3, 0),
    (2, 3, 0, 1),
    (3, 0, 1, 2),
)

TRELLIS_CONVOLUTIONAL_ENCODER = (
    (0, 2, 3, 1),
    (4, 7, 5, 6),
    (1, 3, 2, 0),
    (7, 4, 6, 5),
    (2, 0, 1, 3),
    (6, 5, 7, 4),
    (3, 1, 0, 2),
    (5, 6, 4, 7),
)


def bits_per_symbol(bit_rate: int) -> int:
    """Return the number of payload bits per 2400-symbol baud."""

    try:
        return BITS_PER_SYMBOL[bit_rate]
    except KeyError as exc:
        raise ValueError(f"unsupported V.32bis bit rate: {bit_rate}") from exc


def differential_encode(previous_state: int, dibit: int, bit_rate: int) -> int:
    """Encode the first two bits of a symbol group.

    `dibit` is interpreted as the two least-significant bits of the group:
    bit 0 is Q1n and bit 1 is Q2n.
    """

    if not 0 <= previous_state <= 3:
        raise ValueError("previous_state must be in range 0..3")
    if not 0 <= dibit <= 3:
        raise ValueError("dibit must be in range 0..3")

    if bit_rate == 4800:
        return V32BIS_4800_DIFFERENTIAL_ENCODER[previous_state][dibit]
    return TRELLIS_DIFFERENTIAL_ENCODER[previous_state][dibit]


def convolution_encode(previous_state: int, diff_state: int) -> int:
    """Advance the 3-bit trellis state and return the new state."""

    if not 0 <= previous_state <= 7:
        raise ValueError("previous_state must be in range 0..7")
    if not 0 <= diff_state <= 3:
        raise ValueError("diff_state must be in range 0..3")
    return TRELLIS_CONVOLUTIONAL_ENCODER[previous_state][diff_state]


@dataclass(frozen=True)
class EncoderState:
    """Minimal transmitter-side coding state."""

    diff_state: int = 1
    conv_state: int = 0


@dataclass(frozen=True)
class EncodedSymbol:
    """Result of encoding one V.32bis symbol group."""

    symbol_index: int
    state: EncoderState


def encode_symbol_bits(bit_rate: int, bits: int, state: EncoderState) -> EncodedSymbol:
    """Encode one symbol group's worth of already-scrambled bits.

    The bit ordering follows the same LSB-first convention used in SpanDSP's
    V.17/V.32bis transmitter path:

    - bit 0 -> Q1n
    - bit 1 -> Q2n
    - bit 2 -> Q3n
    - ...
    """

    bps = bits_per_symbol(bit_rate)
    if bits < 0 or bits >= (1 << bps):
        raise ValueError(f"bits must be in range 0..{(1 << bps) - 1} for {bit_rate}")

    dibit = bits & 0x03
    diff_state = differential_encode(state.diff_state, dibit, bit_rate)

    if bit_rate == 4800:
        return EncodedSymbol(
            symbol_index=diff_state,
            state=EncoderState(diff_state=diff_state, conv_state=state.conv_state),
        )

    conv_state = convolution_encode(state.conv_state, diff_state)
    symbol_index = ((bits << 1) & 0x78) | (diff_state << 1) | ((conv_state >> 2) & 0x01)
    return EncodedSymbol(
        symbol_index=symbol_index,
        state=EncoderState(diff_state=diff_state, conv_state=conv_state),
    )
