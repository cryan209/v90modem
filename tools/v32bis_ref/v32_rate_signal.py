"""V.32 rate signal and sequence E helpers.

V.32 (Table 6/V.32) uses different 16-bit sync positions than V.32bis:

  R-sequence sync: B0-3=0, B7=1, B11=1, B15=1
  E-sequence sync: B0-3=1, B7=1, B11=1, B15=1

Capability bits in the R sequence:
  B4: 2400 bit/s (for further study; normally 0)
  B5: 4800 bit/s
  B6: 9600 bit/s
  B4-6=000: GSTN cleardown request
  B8: trellis coding available at the highest indicated rate
  B9-14: special modes (all 0 = none; B11=1 is already the fixed sync bit)

Note: B4=1 together with B8=1 signals V.32bis capability (Table 6, Note 1).

The encoding process (GPC/GPA scramble + 4800-baud differential encode) is
identical to V.32bis. Only the bit content and sync positions differ.
"""

from __future__ import annotations

from .rate_signal import (
    RateSignalEncoding,
    decode_rate_sequence_symbols,
    encode_rate_sequence_bits,
)

# Capability-flag bit-mask constants for V.32 rate events.
V32_CAP_4800 = 0x01
V32_CAP_9600 = 0x02
V32_CAP_TRELLIS = 0x04
V32_CAP_CLEARDOWN = 0x08

# Fixed sync positions (bit index → required value) per Table 6/V.32.
V32_RATE_SYNC_BITS: dict[int, int] = {0: 0, 1: 0, 2: 0, 3: 0, 7: 1, 11: 1, 15: 1}

# Fixed sync positions per Table 7/V.32 (E sequence).
V32_E_SYNC_BITS: dict[int, int] = {0: 1, 1: 1, 2: 1, 3: 1, 7: 1, 11: 1, 15: 1}


def v32_rate_signal_bits(
    *,
    support_4800: bool = True,
    support_9600: bool = True,
    trellis: bool = True,
    cleardown: bool = False,
) -> list[int]:
    """Build a 16-bit V.32 R-sequence (Table 6/V.32).

    B5=4800, B6=9600, B8=trellis; B4-6=000 requests a GSTN cleardown.
    """
    bits = [0] * 16
    for index, value in V32_RATE_SYNC_BITS.items():
        bits[index] = value
    if cleardown:
        bits[4] = bits[5] = bits[6] = 0
    else:
        bits[5] = 1 if support_4800 else 0
        bits[6] = 1 if support_9600 else 0
        bits[8] = 1 if trellis and (support_4800 or support_9600) else 0
    return bits


def v32_e_sequence_bits(
    selected_rate: int,
    *,
    trellis: bool = True,
) -> list[int]:
    """Build a 16-bit V.32 E-sequence (Table 7/V.32)."""
    if selected_rate not in (4800, 9600):
        raise ValueError(f"unsupported V.32 data rate: {selected_rate}")
    bits = [0] * 16
    for index, value in V32_E_SYNC_BITS.items():
        bits[index] = value
    if selected_rate == 4800:
        bits[5] = 1
    else:
        bits[6] = 1
        bits[8] = 1 if trellis else 0
    return bits


def is_v32_rate_signal_bits(bits: list[int]) -> bool:
    """Return True if the 16-bit word matches V.32 Table 6 sync positions."""
    return len(bits) == 16 and all(bits[i] == v for i, v in V32_RATE_SYNC_BITS.items())


def is_v32_e_sequence_bits(bits: list[int]) -> bool:
    """Return True if the 16-bit word matches V.32 Table 7 sync positions."""
    return len(bits) == 16 and all(bits[i] == v for i, v in V32_E_SYNC_BITS.items())


def decode_v32_rate_signal(bits: list[int]) -> tuple[bool, bool, bool, bool]:
    """Decode a V.32 R-sequence bit word.

    Returns ``(support_4800, support_9600, trellis, cleardown)``.
    """
    if not is_v32_rate_signal_bits(bits):
        raise ValueError("bits do not form a valid V.32 rate signal")
    cleardown = not bits[4] and not bits[5] and not bits[6]
    return bool(bits[5]), bool(bits[6]), bool(bits[8]), cleardown


def decode_v32_e_sequence(bits: list[int]) -> tuple[int, bool]:
    """Decode a V.32 E-sequence bit word.

    Returns ``(selected_rate, trellis)``.
    """
    if not is_v32_e_sequence_bits(bits):
        raise ValueError("bits do not form a valid V.32 E sequence")
    if bits[5] and not bits[6]:
        return 4800, False
    if bits[6]:
        return 9600, bool(bits[8])
    raise ValueError("V.32 E sequence indicates no data rate")


def encode_v32_rate_sequence_bits(
    bits: list[int],
    *,
    calling_party: bool,
    initial_diff_state: int,
    initial_scrambler_register: int = 0,
) -> RateSignalEncoding:
    """Scramble and differentially encode a 16-bit V.32 R or E sequence.

    Delegates to the same GPC/GPA scrambler and 4800-baud differential encoder
    used by V.32bis — only the bit content differs between the two standards.
    """
    return encode_rate_sequence_bits(
        bits,
        calling_party=calling_party,
        initial_diff_state=initial_diff_state,
        initial_scrambler_register=initial_scrambler_register,
    )


def decode_v32_rate_sequence_symbols(
    symbols: list[str],
    *,
    calling_party: bool,
    initial_diff_state: int,
) -> list[int]:
    """Recover a 16-bit V.32 R or E word from eight 4800-baud startup symbols.

    Delegates to the same decoding machinery as V.32bis.
    """
    return decode_rate_sequence_symbols(
        symbols,
        calling_party=calling_party,
        initial_diff_state=initial_diff_state,
    )


def v32_cap_mask(
    *,
    support_4800: bool,
    support_9600: bool,
    trellis: bool,
    cleardown: bool = False,
) -> int:
    """Pack V.32 capability flags into a compact bitmask (V32_CAP_* constants)."""
    mask = 0
    if support_4800:
        mask |= V32_CAP_4800
    if support_9600:
        mask |= V32_CAP_9600
    if trellis:
        mask |= V32_CAP_TRELLIS
    if cleardown:
        mask |= V32_CAP_CLEARDOWN
    return mask


def v32_highest_common_rate(
    call_4800: bool,
    call_9600: bool,
    ans_4800: bool,
    ans_9600: bool,
) -> int | None:
    """Select the highest V.32 data rate supported by both sides."""
    if call_9600 and ans_9600:
        return 9600
    if call_4800 and ans_4800:
        return 4800
    return None
