"""V.32bis rate signal and sequence E helpers."""

from __future__ import annotations

from dataclasses import dataclass

from .coding import EncoderState, differential_decode, differential_encode
from .scrambler import Descrambler, Scrambler, scrambler_tap


RATE_14400 = 0x1000
RATE_12000 = 0x0400
RATE_9600 = 0x0200
RATE_7200 = 0x0040
RATE_4800 = 0x0020

SUPPORTED_RATE_MASK = RATE_14400 | RATE_12000 | RATE_9600 | RATE_7200 | RATE_4800

SYNC_BITS = {
    0: 0,
    1: 0,
    2: 0,
    3: 0,
    7: 1,
    11: 0,
    15: 0,
}

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


def decode_rate_sequence_symbols(
    symbols: list[str],
    *,
    calling_party: bool,
    initial_diff_state: int,
) -> list[int]:
    """Recover a 16-bit R or E word from eight observed 4800 startup symbols."""

    if len(symbols) != 8:
        raise ValueError("rate sequence symbol run must contain exactly 8 symbols")

    descrambler = Descrambler(scrambler_tap(calling_party, transmit=True))
    diff_state = initial_diff_state
    bits: list[int] = []
    for symbol in symbols:
        if not symbol.startswith("Q"):
            raise ValueError(f"unexpected startup symbol label: {symbol}")
        output_state = int(symbol[1:])
        dibit = differential_decode(diff_state, output_state, 4800)
        diff_state = output_state
        scrambled_bits = [dibit & 0x01, (dibit >> 1) & 0x01]
        bits.extend(descrambler.process_bits(scrambled_bits))
    return bits


def encode_rate_sequence_bits(
    bits: list[int],
    *,
    calling_party: bool,
    initial_diff_state: int,
) -> RateSignalEncoding:
    """Scramble and 4800-differentially encode a 16-bit R or E sequence."""

    if len(bits) != 16:
        raise ValueError("rate sequence must contain exactly 16 bits")

    scrambler = Scrambler(scrambler_tap(calling_party, transmit=True))
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
    )
