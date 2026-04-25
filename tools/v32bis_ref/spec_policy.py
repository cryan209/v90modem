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

# The Recommendation is explicit about zeroing the scrambler for TRN and
# renegotiation, but it does not clearly say to reset the scrambler between
# normal startup segments after TRN. We therefore model scrambler continuity
# across TRN -> R -> E -> B1 as an interoperability assumption, not a proven
# normative rule.
NORMAL_STARTUP_SCRAMBLER_RESET_AFTER_E: bool | None = None

_TRN_STATE_TO_DIFF_STATE = {
    "A": 0,
    "B": 1,
    "C": 3,
    "D": 2,
}


@dataclass(frozen=True)
class StartupTransmitState:
    """Reference transmitter state used when leaving TRN for startup signalling.

    This models the ITU-oriented startup policy used by the reference code:
    - differential state is derived from the final transmitted TRN symbol
    - scrambler register continuity is carried from the end of TRN
    - the trellis/convolution state is explicitly zero at B1 entry

    The scrambler carry-forward remains an interoperability assumption rather
    than a fully explicit normative statement from the Recommendation.
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
