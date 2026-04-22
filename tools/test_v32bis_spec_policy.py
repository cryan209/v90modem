"""Focused tests for the V.32bis startup-state policy helpers."""

from __future__ import annotations

import unittest

from tools.v32bis_ref import (
    POST_E_INITIAL_CONVOLUTION_STATE,
    NORMAL_STARTUP_SCRAMBLER_RESET_AFTER_E,
    RATE_7200,
    RATE_9600,
    RATE_12000,
    TRN_INITIAL_SCRAMBLER_REGISTER,
    encode_rate_sequence_bits,
    generate_call_startup_trace,
    generate_conditioning_signal,
    rate_signal_bits,
    startup_diff_state_from_final_trn_symbol,
    startup_scrambler_register_from_trn,
)
from tools.v32bis_ref.scrambler import Scrambler, scrambler_tap


class V32bisSpecPolicyTests(unittest.TestCase):
    def test_trn_policy_starts_from_zero_register(self) -> None:
        conditioning = generate_conditioning_signal(True, 260)
        scrambler = Scrambler(
            scrambler_tap(calling_party=True, transmit=True),
            register=TRN_INITIAL_SCRAMBLER_REGISTER,
        )
        for _ in range(260 * 2):
            scrambler.process_bit(1)
        self.assertEqual(conditioning.trn_final_scrambler_register, scrambler.register)

    def test_startup_trace_uses_final_trn_symbol_and_scrambler_state(self) -> None:
        conditioning = generate_conditioning_signal(True, 260)
        expected = encode_rate_sequence_bits(
            rate_signal_bits(RATE_7200 | RATE_9600),
            calling_party=True,
            initial_diff_state=startup_diff_state_from_final_trn_symbol(conditioning.final_trn_symbol),
            initial_scrambler_register=startup_scrambler_register_from_trn(
                conditioning.trn_final_scrambler_register
            ),
        )
        trace = generate_call_startup_trace(
            r1_mask=RATE_7200 | RATE_9600 | RATE_12000,
            r2_mask=RATE_7200 | RATE_9600,
            r3_selected_rate=9600,
            trn_length=260,
            r2_repetitions=1,
            spec_derived_startup_state=True,
        )
        self.assertEqual(trace[2].symbols, [f"Q{state}" for state in expected.differential_states])

    def test_post_e_convolution_policy_is_explicit_zero(self) -> None:
        self.assertEqual(POST_E_INITIAL_CONVOLUTION_STATE, 0)

    def test_normal_startup_scrambler_reset_remains_unresolved(self) -> None:
        self.assertIsNone(NORMAL_STARTUP_SCRAMBLER_RESET_AFTER_E)


if __name__ == "__main__":
    unittest.main()
