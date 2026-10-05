"""Normative V.32bis startup-state policy notes and helpers.

This module separates rules stated explicitly in the Recommendation from
startup-state behavior we currently infer when modelling a normal startup
handoff in the reference transmitter.
"""

from __future__ import annotations

from dataclasses import dataclass

TRN_INITIAL_SCRAMBLER_REGISTER = 0
RENEGOTIATION_INITIAL_SCRAMBLER_REGISTER = 0
POST_E_INITIAL_CONVOLUTION_STATE = 0

# The Recommendation zeroes the scrambler for TRN (5.2.3) and for
# renegotiation (5.3.2).  5.3 makes the rate signal one continuously
# scrambled, differentially encoded stream, so the scrambler and the
# differential encoder run on from TRN through every repeated R word, E and
# into B1 -- nothing resets between them.  (An earlier model reseeded every
# word from the end of TRN; slmodemd's R1 descrambled continuously reads as
# identical valid words, and it never answered the reseeded form.  See the
# "Against slmodemd" section of docs/v32bis_compliance_plan.md.)
NORMAL_STARTUP_SCRAMBLER_RESET_AFTER_E = False

# Figure 2-5: A = 00, D = 10, B = 01, C = 11 by Y1Y2, i.e. 4800 table indices
# 0, 1, 2, 3 with b0 = Y1.
_TRN_STATE_TO_DIFF_STATE = {
    "A": 0,
    "D": 1,
    "B": 2,
    "C": 3,
}


@dataclass(frozen=True)
class StartupTransmitState:
    """Reference transmitter state used when leaving TRN for startup signalling.

    It seeds the first word of a rate signal; later words and E continue
    from the state the previous word left (5.3):
    - differential state is derived from the final transmitted TRN symbol
    - scrambler register continuity is carried from the end of TRN
    - the trellis/convolution state is explicitly zero at B1 entry
    """

    scrambler_register: int
    diff_state: int
    convolution_state: int = POST_E_INITIAL_CONVOLUTION_STATE


def startup_diff_state_from_final_trn_symbol(symbol: str) -> int:
    """Map the final TRN state label to the startup differential state."""

    try:
        return _TRN_STATE_TO_DIFF_STATE[symbol]
    except KeyError as exc:
        raise ValueError(f"unsupported TRN state label: {symbol}") from exc


def startup_scrambler_register_from_trn(final_trn_scrambler_register: int) -> int:
    """Return the carried scrambler register for normal startup modelling."""

    return final_trn_scrambler_register


def startup_state_from_trn(
    final_trn_symbol: str,
    final_trn_scrambler_register: int,
) -> StartupTransmitState:
    """Build the reference startup transmitter state from the end of TRN."""

    return StartupTransmitState(
        scrambler_register=startup_scrambler_register_from_trn(final_trn_scrambler_register),
        diff_state=startup_diff_state_from_final_trn_symbol(final_trn_symbol),
        convolution_state=POST_E_INITIAL_CONVOLUTION_STATE,
    )
