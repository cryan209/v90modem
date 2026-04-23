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

V32BIS_4800_DIFFERENTIAL_DECODER = tuple(
    tuple(row.index(output_state) for output_state in range(4))
    for row in V32BIS_4800_DIFFERENTIAL_ENCODER
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


def differential_decode(previous_state: int, output_state: int, bit_rate: int) -> int:
    """Invert the differential encoder for one dibit/state transition."""

    if not 0 <= previous_state <= 3:
        raise ValueError("previous_state must be in range 0..3")
    if not 0 <= output_state <= 3:
        raise ValueError("output_state must be in range 0..3")

    if bit_rate == 4800:
        return V32BIS_4800_DIFFERENTIAL_DECODER[previous_state][output_state]
    return TRELLIS_DIFFERENTIAL_ENCODER[previous_state].index(output_state)


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


# ---------------------------------------------------------------------------
# V.32 non-redundant 9600 bit/s (16-QAM, no trellis)
# ---------------------------------------------------------------------------

def encode_v32_nr_symbol_bits(bits: int, state: EncoderState) -> EncodedSymbol:
    """Encode 4 scrambled data bits using V.32 non-redundant (16-QAM) coding.

    Differentially encodes Q1n, Q2n → Y1n, Y2n via Table 1/V.32 (same table
    as 4800 bit/s differential encoding).  Q3n and Q4n pass through unchanged.

    Symbol index = Y1·8 + Y2·4 + Q3·2 + Q4  (range 0–15).

    Input bit ordering (LSB-first, same convention as :func:`encode_symbol_bits`):
      bit 0 → Q1n, bit 1 → Q2n, bit 2 → Q3n, bit 3 → Q4n
    """
    if not 0 <= bits <= 15:
        raise ValueError("bits must be in range 0..15 for V.32 non-redundant 9600")

    dibit = bits & 0x03
    diff_state = differential_encode(state.diff_state, dibit, 4800)

    # Y1 = MSB of diff_state, Y2 = LSB of diff_state (see Table 1/V.32 mapping)
    y1 = (diff_state >> 1) & 1
    y2 = diff_state & 1
    q3 = (bits >> 2) & 1
    q4 = (bits >> 3) & 1
    symbol_index = (y1 << 3) | (y2 << 2) | (q3 << 1) | q4

    return EncodedSymbol(
        symbol_index=symbol_index,
        state=EncoderState(diff_state=diff_state, conv_state=state.conv_state),
    )


def decode_v32_nr_symbol_index(
    symbol_index: int, state: EncoderState
) -> tuple[int, EncoderState]:
    """Decode a V.32 non-redundant 9600 symbol index (0–15) to 4 data bits.

    Returns ``(bits, new_state)`` where ``bits`` is the recovered 4-bit group
    in the same LSB-first ordering as :func:`encode_v32_nr_symbol_bits`.
    """
    if not 0 <= symbol_index <= 15:
        raise ValueError("symbol_index must be in range 0..15")

    y1 = (symbol_index >> 3) & 1
    y2 = (symbol_index >> 2) & 1
    q3 = (symbol_index >> 1) & 1
    q4 = symbol_index & 1

    diff_state = (y1 << 1) | y2
    dibit = differential_decode(state.diff_state, diff_state, 4800)
    bits = dibit | (q3 << 2) | (q4 << 3)

    return bits, EncoderState(diff_state=diff_state, conv_state=state.conv_state)
