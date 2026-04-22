"""Normative V.32bis startup-state policy notes and helpers.

This module separates rules stated explicitly in the Recommendation from
startup-state behavior we currently infer when modelling a normal startup
handoff in the reference transmitter.
"""

from __future__ import annotations

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


def startup_diff_state_from_final_trn_symbol(symbol: str) -> int:
    """Map the final TRN state label to the startup differential state."""

    try:
        return _TRN_STATE_TO_DIFF_STATE[symbol]
    except KeyError as exc:
        raise ValueError(f"unsupported TRN state label: {symbol}") from exc


def startup_scrambler_register_from_trn(final_trn_scrambler_register: int) -> int:
    """Return the carried scrambler register for normal startup modelling."""

    return final_trn_scrambler_register
