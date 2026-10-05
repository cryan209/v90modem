"""V.32 startup trace generator.

Produces transmitter-side segment sequences for the V.32 start-up procedure
(§5.4/V.32, Figure 4).  The segment structure differs from V.32bis in two ways:

  1. The calling modem prefixes its conditioning with an AA segment (carrier
     state A repeated) followed by a CC segment (state C repeated).  These are
     used for echo-canceller training and round-trip timing measurement.

  2. The answering modem prefixes its conditioning with AC → CA → AC segments
     (alternating A/C and C/A) for the same purpose.

After those prefixes both sides proceed with the usual S, S̄, TRN receiver-
conditioning signal and then exchange R and E sequences encoded using V.32 bit
formats (see :mod:`.v32_rate_signal`).

Nominal segment lengths follow the minimum values from Figure 4 and §5.4:
  AA: 64 T    (64 ± 2 per spec, model uses 64)
  CC: 64 T    (duration until second phase reversal detected; model uses 64)
  AC: 128 T   (≥ 128, even)
  CA: 64 T    (64 ± 2, even)
"""

from __future__ import annotations

from .rate_signal import encode_rate_sequence_bits
from .spec_policy import (
    startup_state_from_trn,
)
from .startup import StartupSegment
from .training import (
    STATE_A,
    STATE_C,
    ConditioningSignal,
    alternating_states,
    generate_conditioning_signal,
)
from .v32_rate_signal import v32_e_sequence_bits, v32_rate_signal_bits


def _encode_chain(
    bits: list[int],
    *,
    calling_party: bool,
    initial_diff_state: int,
    initial_scrambler_register: int,
    repetitions: int = 1,
) -> tuple[list[str], int, int]:
    """Encode *repetitions* words as one continuous rate-signal stream (5.3).

    Returns the state labels and the final (diff state, scrambler register),
    from which the next word or E carries on.
    """

    labels: list[str] = []
    diff_state = initial_diff_state
    register = initial_scrambler_register
    for _ in range(repetitions):
        encoded = encode_rate_sequence_bits(
            bits,
            calling_party=calling_party,
            initial_diff_state=diff_state,
            initial_scrambler_register=register,
        )
        labels.extend(f"Q{state}" for state in encoded.differential_states)
        diff_state = encoded.final_state
        register = encoded.final_scrambler_register
    return labels, diff_state, register


def _rate_segment(
    name: str,
    bits: list[int],
    *,
    calling_party: bool,
    initial_diff_state: int,
    initial_scrambler_register: int,
    repetitions: int,
) -> tuple[StartupSegment, int, int]:
    labels, diff_state, register = _encode_chain(
        bits,
        calling_party=calling_party,
        initial_diff_state=initial_diff_state,
        initial_scrambler_register=initial_scrambler_register,
        repetitions=repetitions,
    )
    segment = StartupSegment(
        name=name,
        kind="rate_signal",
        tx_calling_party=calling_party,
        bits=bits,
        symbols=labels,
        repetitions=repetitions,
    )
    return segment, diff_state, register


def _e_segment(
    selected_rate: int,
    *,
    trellis: bool,
    calling_party: bool,
    initial_diff_state: int,
    initial_scrambler_register: int,
) -> StartupSegment:
    bits = v32_e_sequence_bits(selected_rate, trellis=trellis)
    labels, _, _ = _encode_chain(
        bits,
        calling_party=calling_party,
        initial_diff_state=initial_diff_state,
        initial_scrambler_register=initial_scrambler_register,
    )
    return StartupSegment(
        name="E",
        kind="sequence_e",
        tx_calling_party=calling_party,
        bit_rate=selected_rate,
        bits=bits,
        symbols=labels,
    )


def _conditioning(calling_party: bool, conditioning: ConditioningSignal) -> StartupSegment:
    return StartupSegment(
        name="conditioning",
        kind="conditioning",
        tx_calling_party=calling_party,
        symbols=conditioning.symbols,
    )


def _b1(bit_rate: int, count: int, *, calling_party: bool) -> StartupSegment:
    return StartupSegment(
        name="B1",
        kind="scrambled_ones",
        tx_calling_party=calling_party,
        bit_rate=bit_rate,
        repetitions=count,
    )


# ---------------------------------------------------------------------------
# Public segment generators
# ---------------------------------------------------------------------------

def generate_aa_segment(length: int = 64, *, calling_party: bool) -> StartupSegment:
    """Carrier state A repeated — calling modem before echo-canceller training."""
    if length < 1:
        raise ValueError("length must be positive")
    return StartupSegment(
        name="AA",
        kind="aa",
        tx_calling_party=calling_party,
        symbols=[STATE_A] * length,
    )


def generate_cc_segment(length: int = 64, *, calling_party: bool) -> StartupSegment:
    """Carrier state C repeated — calling modem after first phase reversal."""
    if length < 1:
        raise ValueError("length must be positive")
    return StartupSegment(
        name="CC",
        kind="cc",
        tx_calling_party=calling_party,
        symbols=[STATE_C] * length,
    )


def generate_ac_segment(length: int = 128, *, calling_party: bool) -> StartupSegment:
    """Alternating A, C — answering modem initial and post-CA signal (≥128, even)."""
    if length < 2 or length % 2 != 0:
        raise ValueError("AC segment length must be a positive even number")
    return StartupSegment(
        name="AC",
        kind="ac",
        tx_calling_party=calling_party,
        symbols=alternating_states(STATE_A, STATE_C, length),
    )


def generate_ca_segment(length: int = 64, *, calling_party: bool) -> StartupSegment:
    """Alternating C, A — answering modem after first phase reversal (64±2, even)."""
    if length < 2 or length % 2 != 0:
        raise ValueError("CA segment length must be a positive even number")
    return StartupSegment(
        name="CA",
        kind="ca",
        tx_calling_party=calling_party,
        symbols=alternating_states(STATE_C, STATE_A, length),
    )


# ---------------------------------------------------------------------------
# Startup trace generators
# ---------------------------------------------------------------------------

def generate_v32_call_startup_trace(
    *,
    support_4800: bool = True,
    support_9600: bool = True,
    trellis: bool = True,
    selected_rate: int,
    aa_length: int = 64,
    cc_length: int = 64,
    trn_length: int = 1280,
    r2_repetitions: int = 2,
    b1_symbols: int = 128,
    spec_derived_startup_state: bool = True,
) -> list[StartupSegment]:
    """Generate the V.32 calling-modem startup trace (Figure 4/V.32).

    Sequence: AA → CC → conditioning(S, S̄, TRN) → R2 → E → B1

    Parameters
    ----------
    selected_rate:
        Data rate agreed for B1 and data: 4800 or 9600.
    trellis:
        Whether to signal trellis coding in R2 and E (9600 only).
    aa_length, cc_length:
        Symbol counts for the AA and CC segments.
    spec_derived_startup_state:
        If True, derive initial_diff_state and scrambler register from the
        final TRN symbol rather than using fixed defaults.
    """
    if selected_rate not in (4800, 9600):
        raise ValueError(f"unsupported V.32 rate: {selected_rate}")

    conditioning = generate_conditioning_signal(True, trn_length)
    initial_diff_state = 1
    initial_scrambler_register = 0
    if spec_derived_startup_state:
        startup_state = startup_state_from_trn(
            conditioning.final_trn_symbol,
            conditioning.trn_final_scrambler_register,
        )
        initial_diff_state = startup_state.diff_state
        initial_scrambler_register = startup_state.scrambler_register

    r2_bits = v32_rate_signal_bits(
        support_4800=support_4800,
        support_9600=support_9600,
        trellis=trellis,
    )
    r2_segment, e_diff, e_scr = _rate_segment(
        "R2",
        r2_bits,
        calling_party=True,
        initial_diff_state=initial_diff_state,
        initial_scrambler_register=initial_scrambler_register,
        repetitions=r2_repetitions,
    )
    return [
        generate_aa_segment(aa_length, calling_party=True),
        generate_cc_segment(cc_length, calling_party=True),
        _conditioning(True, conditioning),
        r2_segment,
        _e_segment(
            selected_rate,
            trellis=trellis and selected_rate == 9600,
            calling_party=True,
            initial_diff_state=e_diff,
            initial_scrambler_register=e_scr,
        ),
        _b1(selected_rate, b1_symbols, calling_party=True),
    ]


def generate_v32_answer_startup_trace(
    *,
    support_4800: bool = True,
    support_9600: bool = True,
    trellis: bool = True,
    selected_rate: int,
    ac_length: int = 128,
    ca_length: int = 64,
    trn_length: int = 1280,
    r1_repetitions: int = 2,
    r3_repetitions: int = 2,
    b1_symbols: int = 128,
    spec_derived_startup_state: bool = True,
) -> list[StartupSegment]:
    """Generate the V.32 answering-modem startup trace (Figure 4/V.32).

    Sequence: AC → CA → AC → conditioning(S,S̄,TRN) → R1 →
              conditioning(S,S̄,TRN) → R3 → E → B1

    The second AC segment uses half the ac_length of the first (the answerer
    switches back to AC after detecting the caller's second phase reversal).

    Parameters
    ----------
    selected_rate:
        Data rate selected for R3, E, B1, and data: 4800 or 9600.
    trellis:
        Whether to signal trellis coding.
    ac_length:
        Length of the first and third AC segments (first AC is full length;
        third uses ac_length // 2).
    ca_length:
        Length of the CA segment (64 ± 2 per spec).
    """
    if selected_rate not in (4800, 9600):
        raise ValueError(f"unsupported V.32 rate: {selected_rate}")

    r1_cond = generate_conditioning_signal(False, trn_length)
    r3_cond = generate_conditioning_signal(False, trn_length)

    r1_diff = 1
    r3_diff = 1
    r1_scr = 0
    r3_scr = 0
    if spec_derived_startup_state:
        r1_startup_state = startup_state_from_trn(
            r1_cond.final_trn_symbol,
            r1_cond.trn_final_scrambler_register,
        )
        r3_startup_state = startup_state_from_trn(
            r3_cond.final_trn_symbol,
            r3_cond.trn_final_scrambler_register,
        )
        r1_diff = r1_startup_state.diff_state
        r3_diff = r3_startup_state.diff_state
        r1_scr = r1_startup_state.scrambler_register
        r3_scr = r3_startup_state.scrambler_register

    r1_bits = v32_rate_signal_bits(
        support_4800=support_4800,
        support_9600=support_9600,
        trellis=trellis,
    )
    r3_bits = v32_rate_signal_bits(
        support_4800=(selected_rate == 4800),
        support_9600=(selected_rate == 9600),
        trellis=trellis and selected_rate == 9600,
    )

    r1_segment, _, _ = _rate_segment(
        "R1",
        r1_bits,
        calling_party=False,
        initial_diff_state=r1_diff,
        initial_scrambler_register=r1_scr,
        repetitions=r1_repetitions,
    )
    r3_segment, e_diff, e_scr = _rate_segment(
        "R3",
        r3_bits,
        calling_party=False,
        initial_diff_state=r3_diff,
        initial_scrambler_register=r3_scr,
        repetitions=r3_repetitions,
    )
    return [
        generate_ac_segment(ac_length, calling_party=False),
        generate_ca_segment(ca_length, calling_party=False),
        generate_ac_segment(ac_length // 2, calling_party=False),
        _conditioning(False, r1_cond),
        r1_segment,
        _conditioning(False, r3_cond),
        r3_segment,
        _e_segment(
            selected_rate,
            trellis=trellis and selected_rate == 9600,
            calling_party=False,
            initial_diff_state=e_diff,
            initial_scrambler_register=e_scr,
        ),
        _b1(selected_rate, b1_symbols, calling_party=False),
    ]
