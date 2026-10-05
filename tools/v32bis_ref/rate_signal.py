"""V.32bis rate signal and sequence E helpers."""

from __future__ import annotations

from dataclasses import dataclass

from .coding import EncoderState, differential_decode, differential_encode
from .scrambler import Descrambler, Scrambler, scrambler_tap


# Rate masks double as the Table 5 word bits (B0 is bit 0): B5 = 4800,
# B6 = 9600, B9 = 7200, B10 = 12000, B12 = 14400.  Same values as the C
# datapump's V32BIS_RATE_* (spandsp/v32bis.h).
RATE_14400 = 0x1000
RATE_12000 = 0x0400
RATE_9600 = 0x0040
RATE_7200 = 0x0200
RATE_4800 = 0x0020

SUPPORTED_RATE_MASK = RATE_14400 | RATE_12000 | RATE_9600 | RATE_7200 | RATE_4800

# Table 5: B0-B3 = 0 and B7, B11, B15 = 1; 5.3.1 detects a rate signal on
# exactly these seven bits.
SYNC_BITS = {
    0: 0,
    1: 0,
    2: 0,
    3: 0,
    7: 1,
    11: 1,
    15: 1,
}

# Table 6's fixed bits, as transmitted.  B8 is 0 at 4800 under Note 1's V.32
# interworking, and Note 2 has B13/B14 ignored on reception, so E is
# recognised on E_SYNC_BITS alone.
E_PREFIX_BITS = {
    0: 1,
    1: 1,
    2: 1,
    3: 1,
    4: 1,
    7: 1,
    8: 1,
    11: 1,
    13: 0,
    14: 0,
    15: 1,
}

E_SYNC_BITS = {
    0: 1,
    1: 1,
    2: 1,
    3: 1,
    7: 1,
    11: 1,
    15: 1,
}


def bits_to_word(bits: list[int]) -> int:
    """Pack a 16-bit sequence into an integer, B0 as bit 0."""

    if len(bits) != 16:
        raise ValueError("startup word must contain exactly 16 bits")
    return sum(bit << index for index, bit in enumerate(bits))


def word_to_bits(word: int) -> list[int]:
    """Unpack a 16-bit integer into B0..B15."""

    return [(word >> index) & 1 for index in range(16)]


def rate_mask_from_list(bit_rates: list[int] | tuple[int, ...]) -> int:
    mask = 0
    for bit_rate in bit_rates:
        if bit_rate == 4800:
            mask |= RATE_4800
        elif bit_rate == 7200:
            mask |= RATE_7200
        elif bit_rate == 9600:
            mask |= RATE_9600
        elif bit_rate == 12000:
            mask |= RATE_12000
        elif bit_rate == 14400:
            mask |= RATE_14400
        else:
            raise ValueError(f"unsupported V.32bis bit rate: {bit_rate}")
    return mask


def list_from_rate_mask(mask: int) -> list[int]:
    rates = []
    if mask & RATE_4800:
        rates.append(4800)
    if mask & RATE_7200:
        rates.append(7200)
    if mask & RATE_9600:
        rates.append(9600)
    if mask & RATE_12000:
        rates.append(12000)
    if mask & RATE_14400:
        rates.append(14400)
    return rates


def rate_signal_bits(rate_mask: int, v32_compatible: bool = True) -> list[int]:
    """Build the 16-bit R-sequence from Table 5/V.32bis."""

    bits = [0] * 16
    for index, value in SYNC_BITS.items():
        bits[index] = value

    if v32_compatible:
        bits[4] = 1
        bits[8] = 1

    bits[5] = 1 if rate_mask & RATE_4800 else 0
    bits[6] = 1 if rate_mask & RATE_9600 else 0
    bits[9] = 1 if rate_mask & RATE_7200 else 0
    bits[10] = 1 if rate_mask & RATE_12000 else 0
    bits[12] = 1 if rate_mask & RATE_14400 else 0
    return bits


def e_sequence_bits(selected_rate: int, v32_compatible: bool = True) -> list[int]:
    """Build the 16-bit E sequence from Table 6/V.32bis."""

    bits = [0] * 16
    for index, value in E_PREFIX_BITS.items():
        bits[index] = value

    if not v32_compatible:
        bits[8] = 0

    if selected_rate == 4800:
        bits[5] = 1
    elif selected_rate == 7200:
        bits[9] = 1
    elif selected_rate == 9600:
        bits[6] = 1
    elif selected_rate == 12000:
        bits[10] = 1
    elif selected_rate == 14400:
        bits[12] = 1
    else:
        raise ValueError(f"unsupported V.32bis bit rate: {selected_rate}")
    return bits


@dataclass(frozen=True)
class RateSignalEncoding:
    scrambled_bits: list[int]
    output_dibits: list[int]
    differential_states: list[int]
    final_state: int
    initial_scrambler_register: int
    final_scrambler_register: int


def decode_rate_stream_symbols(
    symbols: list[str],
    *,
    calling_party: bool,
    initial_diff_state: int,
    initial_scrambler_register: int = 0,
) -> list[int]:
    """Descramble any run of 4800 startup symbols as one continuous stream.

    5.3's rate signal is a single scrambled, differentially encoded stream,
    so a word that follows others is recovered by decoding through them; the
    self-synchronising descrambler is right after 23 bits whatever its seed.
    """

    descrambler = Descrambler(
        scrambler_tap(calling_party, transmit=True),
        register=initial_scrambler_register,
    )
    diff_state = initial_diff_state
    bits: list[int] = []
    for symbol in symbols:
        if not symbol.startswith("Q"):
            raise ValueError(f"unexpected startup symbol label: {symbol}")
        output_state = int(symbol[1:])
        dibit = differential_decode(diff_state, output_state, 4800)
        diff_state = output_state
        bits.extend(descrambler.process_bits([dibit & 0x01, (dibit >> 1) & 0x01]))
    return bits


def decode_rate_sequence_symbols(
    symbols: list[str],
    *,
    calling_party: bool,
    initial_diff_state: int,
    initial_scrambler_register: int = 0,
) -> list[int]:
    """Recover a 16-bit R or E word from eight observed 4800 startup symbols."""

    if len(symbols) != 8:
        raise ValueError("rate sequence symbol run must contain exactly 8 symbols")
    return decode_rate_stream_symbols(
        symbols,
        calling_party=calling_party,
        initial_diff_state=initial_diff_state,
        initial_scrambler_register=initial_scrambler_register,
    )


def is_rate_signal_bits(bits: list[int]) -> bool:
    """Return True if the decoded 16-bit word matches Table 5 sync bits."""

    return len(bits) == 16 and all(bits[index] == value for index, value in SYNC_BITS.items())


def is_e_sequence_bits(bits: list[int]) -> bool:
    """Return True if the decoded 16-bit word matches Table 6 sync bits."""

    return len(bits) == 16 and all(bits[index] == value for index, value in E_SYNC_BITS.items())


def encode_rate_sequence_bits(
    bits: list[int],
    *,
    calling_party: bool,
    initial_diff_state: int,
    initial_scrambler_register: int = 0,
) -> RateSignalEncoding:
    """Scramble and 4800-differentially encode a 16-bit R or E sequence.

    5.3 makes the rate signal one continuously scrambled, differentially
    encoded stream: a caller chaining repeated words and E passes each
    word's ``final_scrambler_register`` and ``final_state`` into the next.
    Renegotiation (5.3.2) restarts the scrambler from zero.
    """

    if len(bits) != 16:
        raise ValueError("rate sequence must contain exactly 16 bits")

    scrambler = Scrambler(
        scrambler_tap(calling_party, transmit=True),
        register=initial_scrambler_register,
    )
    scrambled = scrambler.process_bits(bits)

    states = []
    dibits = []
    diff_state = initial_diff_state
    for i in range(0, 16, 2):
        dibit = scrambled[i] | (scrambled[i + 1] << 1)
        dibits.append(dibit)
        diff_state = differential_encode(diff_state, dibit, 4800)
        states.append(diff_state)

    return RateSignalEncoding(
        scrambled_bits=scrambled,
        output_dibits=dibits,
        differential_states=states,
        final_state=diff_state,
        initial_scrambler_register=initial_scrambler_register,
        final_scrambler_register=scrambler.register,
    )
