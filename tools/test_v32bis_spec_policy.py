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
    generate_answer_startup_trace,
    generate_conditioning_signal,
    rate_signal_bits,
    startup_state_from_trn,
    startup_diff_state_from_final_trn_symbol,
    startup_scrambler_register_from_trn,
)
from tools.v32bis_ref.scrambler import Scrambler, scrambler_tap
from tools.v32bis_ref.spec_policy import StartupTransmitState


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

    def _assert_continuous(self, rate, e, b1, *, calling_party, repetitions) -> None:
        """5.3: the R words and E are one scrambled, differentially encoded
        stream, and B1 carries on from where E left it."""

        state = rate.initial_tx_state
        expected: list[str] = []
        for bits in [rate.bits] * repetitions + [e.bits]:
            encoded = encode_rate_sequence_bits(
                bits,
                calling_party=calling_party,
                initial_diff_state=state.diff_state,
                initial_scrambler_register=state.scrambler_register,
            )
            expected.extend(f"Q{x}" for x in encoded.differential_states)
            state = StartupTransmitState(encoded.final_scrambler_register, encoded.final_state)
        self.assertEqual(rate.symbols + e.symbols, expected)
        # Not reseeded per word, so a repeated word is not repeated symbols.
        self.assertNotEqual(rate.symbols[:8], rate.symbols[8:16])
        self.assertEqual(e.initial_tx_state, rate.final_tx_state)
        self.assertEqual(b1.initial_tx_state.diff_state, e.final_tx_state.diff_state)
        self.assertEqual(b1.initial_tx_state.scrambler_register, e.final_tx_state.scrambler_register)
        self.assertEqual(b1.initial_tx_state.convolution_state, POST_E_INITIAL_CONVOLUTION_STATE)

    def test_call_trace_r2_and_e_are_one_continuous_stream(self) -> None:
        trace = generate_call_startup_trace(
            r1_mask=RATE_7200 | RATE_9600 | RATE_12000,
            r2_mask=RATE_7200 | RATE_9600,
            r3_selected_rate=9600,
            trn_length=260,
            r2_repetitions=2,
        )
        self._assert_continuous(trace[2], trace[3], trace[4], calling_party=True, repetitions=2)

    def test_answer_trace_r3_and_e_are_one_continuous_stream(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_7200 | RATE_9600 | RATE_12000,
            r2_mask=RATE_7200 | RATE_9600,
            r3_selected_rate=9600,
            trn_length=260,
            r1_repetitions=2,
            r3_repetitions=2,
        )
        self._assert_continuous(trace[3], trace[4], trace[5], calling_party=False, repetitions=2)

    def test_post_e_convolution_policy_is_explicit_zero(self) -> None:
        self.assertEqual(POST_E_INITIAL_CONVOLUTION_STATE, 0)

    def test_startup_state_from_trn_centralizes_reference_handoff(self) -> None:
        conditioning = generate_conditioning_signal(True, 260)
        state = startup_state_from_trn(
            conditioning.final_trn_symbol,
            conditioning.trn_final_scrambler_register,
        )
        self.assertEqual(
            state.diff_state,
            startup_diff_state_from_final_trn_symbol(conditioning.final_trn_symbol),
        )
        self.assertEqual(
            state.scrambler_register,
            startup_scrambler_register_from_trn(conditioning.trn_final_scrambler_register),
        )
        self.assertEqual(state.convolution_state, POST_E_INITIAL_CONVOLUTION_STATE)

    def test_normal_startup_does_not_reset_the_scrambler(self) -> None:
        self.assertIs(NORMAL_STARTUP_SCRAMBLER_RESET_AFTER_E, False)


if __name__ == "__main__":
    unittest.main()
