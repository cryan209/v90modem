"""Unit tests for the spec-locked V.32bis coding helpers."""

from __future__ import annotations

import unittest

from tools.v32bis_ref.coding import (
    EncoderState,
    bits_per_symbol,
    convolution_encode,
    differential_encode,
    encode_symbol_bits,
)
from tools.v32bis_ref.constellation import symbol_to_point
from tools.v32bis_ref.rate_signal import (
    RATE_12000,
    RATE_14400,
    RATE_4800,
    RATE_7200,
    RATE_9600,
    e_sequence_bits,
    encode_rate_sequence_bits,
    list_from_rate_mask,
    rate_mask_from_list,
    rate_signal_bits,
)
from tools.v32bis_ref.negotiation import (
    decode_e_rate,
    decode_rate_mask,
    detect_repeated_rate_signal,
    detect_s_sequence,
    highest_common_rate,
    negotiate_renegotiation_rate,
    negotiate_startup_rate,
    validate_e_sequence_bits,
    validate_rate_signal_bits,
)
from tools.v32bis_ref.receiver import V32bisLogicalReceiver
from tools.v32bis_ref.rx_frontend import (
    nearest_symbol_label,
    passband_to_baseband,
    recover_symbols_with_carrier_tracking,
    recover_symbols_with_frontend,
    recover_symbols_ideal,
    recover_symbols_with_timing_loop,
    recover_symbols_with_timing_tracking,
    recover_symbols_with_timing_offset,
    recover_symbols_with_tracking,
    search_carrier_frequency,
    search_timing_and_carrier,
    search_symbol_timing,
    symbol_error_metric,
)
from tools.v32bis_ref.scrambler import Descrambler, Scrambler, scrambler_tap
from tools.v32bis_ref.stream import (
    flatten_startup_trace,
    impair_stream,
    impair_stream_burst,
    impair_stream_insert,
    impair_stream_q_neighbor,
    impair_stream_random,
)
from tools.v32bis_ref.simulator import simulate_startup
from tools.v32bis_ref.startup import generate_answer_startup_trace, generate_call_startup_trace
from tools.v32bis_ref.tx import startup_symbol_to_point, startup_trace_to_complex_symbols
from tools.v32bis_ref.tx_passband import baseband_to_passband
from tools.v32bis_ref.tx_waveform import rrc_taps, symbols_to_baseband
from tools.v32bis_ref.training import (
    STATE_A,
    STATE_B,
    STATE_C,
    STATE_D,
    generate_conditioning_signal,
    generate_s_bar_segment,
    generate_s_segment,
    generate_trn_bits,
    generate_trn_segment,
)


class BitRateTests(unittest.TestCase):
    def test_supported_rates_have_expected_bits_per_symbol(self) -> None:
        self.assertEqual(bits_per_symbol(4800), 2)
        self.assertEqual(bits_per_symbol(7200), 3)
        self.assertEqual(bits_per_symbol(9600), 4)
        self.assertEqual(bits_per_symbol(12000), 5)
        self.assertEqual(bits_per_symbol(14400), 6)


class DifferentialTableTests(unittest.TestCase):
    def test_trellis_table_matches_v32bis_table_1(self) -> None:
        expected = {
            (0, 0, 0): 0,
            (0, 0, 1): 1,
            (0, 0, 2): 2,
            (0, 0, 3): 3,
            (1, 0, 0): 1,
            (1, 0, 1): 2,
            (1, 0, 2): 3,
            (1, 0, 3): 0,
            (2, 0, 0): 2,
            (2, 0, 1): 3,
            (2, 0, 2): 0,
            (2, 0, 3): 1,
            (3, 0, 0): 3,
            (3, 0, 1): 0,
            (3, 0, 2): 1,
            (3, 0, 3): 2,
        }

        for previous, dibit, output in (
            (prev, dibit, out)
            for (dibit, _unused, prev), out in expected.items()
        ):
            self.assertEqual(differential_encode(previous, dibit, 7200), output)

    def test_uncoded_4800_mode_uses_its_own_differential_table(self) -> None:
        self.assertEqual(differential_encode(0, 0, 4800), 2)
        self.assertEqual(differential_encode(0, 1, 4800), 3)
        self.assertEqual(differential_encode(0, 2, 4800), 0)
        self.assertEqual(differential_encode(0, 3, 4800), 1)


class ConvolutionTests(unittest.TestCase):
    def test_known_convolution_state_transitions(self) -> None:
        self.assertEqual(convolution_encode(0, 0), 0)
        self.assertEqual(convolution_encode(0, 1), 2)
        self.assertEqual(convolution_encode(0, 2), 3)
        self.assertEqual(convolution_encode(0, 3), 1)
        self.assertEqual(convolution_encode(4, 2), 1)


class SymbolEncodingTests(unittest.TestCase):
    def test_4800_symbol_index_is_just_differential_output(self) -> None:
        encoded = encode_symbol_bits(4800, 0b00, EncoderState(diff_state=1, conv_state=0))
        self.assertEqual(encoded.symbol_index, 0)
        self.assertEqual(encoded.state.diff_state, 0)

    def test_7200_symbol_encoding_matches_reference_logic(self) -> None:
        encoded = encode_symbol_bits(7200, 0b101, EncoderState(diff_state=1, conv_state=0))
        self.assertEqual(encoded.symbol_index, 0x0C)
        self.assertEqual(encoded.state.diff_state, 2)
        self.assertEqual(encoded.state.conv_state, 3)

    def test_9600_symbol_encoding_matches_reference_logic(self) -> None:
        encoded = encode_symbol_bits(9600, 0b1101, EncoderState(diff_state=1, conv_state=0))
        self.assertEqual(encoded.symbol_index, 0x1C)
        self.assertEqual(encoded.state.diff_state, 2)
        self.assertEqual(encoded.state.conv_state, 3)


class ConstellationTests(unittest.TestCase):
    def test_4800_exact_points(self) -> None:
        self.assertEqual(symbol_to_point(4800, 0), (-6.0, -2.0))
        self.assertEqual(symbol_to_point(4800, 3), (6.0, 2.0))

    def test_7200_exact_points(self) -> None:
        self.assertEqual(symbol_to_point(7200, 0), (6.0, -6.0))
        self.assertEqual(symbol_to_point(7200, 7), (6.0, 6.0))
        self.assertEqual(symbol_to_point(7200, 15), (-2.0, -2.0))

    def test_9600_exact_points(self) -> None:
        self.assertEqual(symbol_to_point(9600, 0), (-8.0, 2.0))
        self.assertEqual(symbol_to_point(9600, 31), (-2.0, 8.0))

    def test_12000_exact_points(self) -> None:
        self.assertEqual(symbol_to_point(12000, 0), (7.0, 1.0))
        self.assertEqual(symbol_to_point(12000, 31), (7.0, -1.0))
        self.assertEqual(symbol_to_point(12000, 63), (3.0, -5.0))

    def test_14400_exact_points(self) -> None:
        self.assertEqual(symbol_to_point(14400, 0), (-8.0, -3.0))
        self.assertEqual(symbol_to_point(14400, 63), (-1.0, 0.0))
        self.assertEqual(symbol_to_point(14400, 127), (-5.0, 0.0))


class ScramblerTests(unittest.TestCase):
    def test_call_and_answer_use_opposite_polynomials(self) -> None:
        self.assertEqual(scrambler_tap(calling_party=True, transmit=True), 17)
        self.assertEqual(scrambler_tap(calling_party=True, transmit=False), 4)
        self.assertEqual(scrambler_tap(calling_party=False, transmit=True), 4)
        self.assertEqual(scrambler_tap(calling_party=False, transmit=False), 17)

    def test_scrambler_and_descrambler_round_trip(self) -> None:
        message = [1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0]
        tx = Scrambler(scrambler_tap(calling_party=True, transmit=True))
        rx = Descrambler(scrambler_tap(calling_party=False, transmit=False))
        scrambled = tx.process_bits(message)
        recovered = rx.process_bits(scrambled)
        self.assertEqual(recovered, message)


class RateSignalTests(unittest.TestCase):
    def test_rate_mask_conversion_round_trip(self) -> None:
        mask = rate_mask_from_list([4800, 9600, 14400])
        self.assertEqual(mask, RATE_4800 | RATE_9600 | RATE_14400)
        self.assertEqual(list_from_rate_mask(mask), [4800, 9600, 14400])

    def test_rate_signal_table_5_layout(self) -> None:
        bits = rate_signal_bits(RATE_4800 | RATE_7200 | RATE_14400)
        self.assertEqual(bits, [0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0])

    def test_e_sequence_table_6_layout(self) -> None:
        bits = e_sequence_bits(12000)
        self.assertEqual(bits, [1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1])

    def test_rate_sequence_encoding_produces_8_differential_states(self) -> None:
        bits = rate_signal_bits(RATE_4800 | RATE_9600 | RATE_12000)
        encoded = encode_rate_sequence_bits(bits, calling_party=True, initial_diff_state=1)
        self.assertEqual(len(encoded.scrambled_bits), 16)
        self.assertEqual(len(encoded.output_dibits), 8)
        self.assertEqual(len(encoded.differential_states), 8)
        self.assertEqual(encoded.final_state, encoded.differential_states[-1])

    def test_different_scrambler_direction_changes_rate_symbol_stream(self) -> None:
        bits = e_sequence_bits(14400)
        call = encode_rate_sequence_bits(bits, calling_party=True, initial_diff_state=1)
        answer = encode_rate_sequence_bits(bits, calling_party=False, initial_diff_state=1)
        self.assertNotEqual(call.scrambled_bits, answer.scrambled_bits)


class TrainingTests(unittest.TestCase):
    def test_s_segment_is_ab_alternation(self) -> None:
        self.assertEqual(generate_s_segment(8), [STATE_A, STATE_B, STATE_A, STATE_B, STATE_A, STATE_B, STATE_A, STATE_B])

    def test_s_bar_segment_is_cd_alternation(self) -> None:
        self.assertEqual(generate_s_bar_segment(6), [STATE_C, STATE_D, STATE_C, STATE_D, STATE_C, STATE_D])

    def test_trn_first_symbols_match_spec_examples(self) -> None:
        call_trn = generate_trn_segment(True, 271)
        answer_trn = generate_trn_segment(False, 271)
        self.assertEqual(call_trn[:15], [STATE_C, STATE_C, STATE_C, STATE_C, STATE_C, STATE_C, STATE_C, STATE_C, STATE_C, STATE_A, STATE_A, STATE_A, STATE_C, STATE_C, STATE_C])
        self.assertEqual(answer_trn[:15], [STATE_C, STATE_C, STATE_C, STATE_A, STATE_A, STATE_C, STATE_C, STATE_C, STATE_A, STATE_A, STATE_C, STATE_C, STATE_A, STATE_C, STATE_C])

    def test_trn_after_first_256_uses_direct_dibit_mapping(self) -> None:
        dibits = generate_trn_bits(True, 260)
        trn = generate_trn_segment(True, 260)
        direct = {
            (0, 0): STATE_A,
            (1, 0): STATE_B,
            (1, 1): STATE_C,
            (0, 1): STATE_D,
        }
        for offset in range(256, 260):
            self.assertEqual(trn[offset], direct[dibits[offset]])

    def test_conditioning_signal_lengths(self) -> None:
        conditioning = generate_conditioning_signal(True, 1280)
        self.assertEqual(len(conditioning.s), 256)
        self.assertEqual(len(conditioning.s_bar), 16)
        self.assertEqual(len(conditioning.trn), 1280)
        self.assertEqual(len(conditioning.symbols), 1552)


class StartupTraceTests(unittest.TestCase):
    def test_call_trace_order(self) -> None:
        trace = generate_call_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_7200 | RATE_9600,
            r3_selected_rate=9600,
        )
        self.assertEqual([segment.name for segment in trace], ["S_NT", "conditioning", "R2", "E", "B1"])
        self.assertEqual(trace[2].bits, rate_signal_bits(RATE_7200 | RATE_9600))
        self.assertEqual(trace[3].bits, e_sequence_bits(9600))

    def test_answer_trace_order(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600 | RATE_12000,
            r2_mask=RATE_7200 | RATE_9600,
            r3_selected_rate=7200,
        )
        self.assertEqual([segment.name for segment in trace], ["conditioning", "R1", "conditioning", "R3", "E", "B1"])
        self.assertEqual(trace[1].bits, rate_signal_bits(RATE_4800 | RATE_7200 | RATE_9600 | RATE_12000))
        self.assertEqual(trace[3].bits, rate_signal_bits(RATE_7200 | RATE_9600))
        self.assertEqual(trace[4].bits, e_sequence_bits(7200))


class ReceiverTests(unittest.TestCase):
    def test_receiver_detects_conditioning_s(self) -> None:
        receiver = V32bisLogicalReceiver()
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        events = receiver.ingest_all(flatten_startup_trace([trace[0]]))
        self.assertEqual([event.name for event in events], ["S"])

    def test_receiver_detects_rate_and_e_segments(self) -> None:
        receiver = V32bisLogicalReceiver()
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        r1_events = receiver.ingest_all(flatten_startup_trace([trace[1]]))
        e_events = receiver.ingest_all(flatten_startup_trace([trace[4]]))
        self.assertEqual(r1_events[0].name, "R1")
        self.assertEqual(r1_events[0].rate_mask, RATE_4800 | RATE_7200 | RATE_9600)
        self.assertEqual(e_events[0].name, "E")
        self.assertEqual(e_events[0].selected_rate, 9600)

    def test_receiver_detects_events_from_continuous_stream(self) -> None:
        receiver = V32bisLogicalReceiver()
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        events = receiver.ingest_all(flatten_startup_trace(trace))
        self.assertEqual([event.name for event in events], ["S", "R1", "S", "R3", "E", "B1"])
        stats = receiver.stats()
        self.assertGreaterEqual(stats.q_valid_words, 5)
        self.assertEqual(stats.e_words_detected, 1)

    def test_receiver_loses_e_when_e_word_is_corrupted(self) -> None:
        receiver = V32bisLogicalReceiver()
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        stream = flatten_startup_trace(trace)
        e_start = next(i for i, obs in enumerate(stream) if obs.source_name == "E")
        impaired = impair_stream(stream, replacements={e_start: "X"})
        events = receiver.ingest_all(impaired)
        self.assertEqual([event.name for event in events], ["S", "R1", "S", "R3", "B1"])
        self.assertEqual(receiver.stats().e_words_detected, 0)

    def test_receiver_loses_r1_when_first_rate_word_is_corrupted(self) -> None:
        receiver = V32bisLogicalReceiver()
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        stream = flatten_startup_trace(trace)
        r1_start = next(i for i, obs in enumerate(stream) if obs.source_name == "R1")
        impaired = impair_stream(stream, replacements={r1_start: "Q9"})
        events = receiver.ingest_all(impaired)
        self.assertNotIn("R1", [event.name for event in events])
        self.assertIn("R3", [event.name for event in events])

    def test_receiver_still_detects_b1_with_one_dropped_symbol(self) -> None:
        receiver = V32bisLogicalReceiver()
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        stream = flatten_startup_trace(trace)
        b1_start = next(i for i, obs in enumerate(stream) if obs.source_name == "B1")
        impaired = impair_stream(stream, drops={b1_start})
        events = receiver.ingest_all(impaired)
        b1_events = [event for event in events if event.name == "B1"]
        self.assertEqual(len(b1_events), 1)
        self.assertGreaterEqual(b1_events[0].repetitions or 0, 24)

    def test_receiver_loses_r1_when_entire_first_word_is_burst_corrupted(self) -> None:
        receiver = V32bisLogicalReceiver()
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
            r1_repetitions=2,
        )
        stream = flatten_startup_trace(trace)
        r1_start = next(i for i, obs in enumerate(stream) if obs.source_name == "R1")
        impaired = impair_stream_burst(stream, start=r1_start, length=8, replacement="X")
        events = receiver.ingest_all(impaired)
        self.assertNotIn("R1", [event.name for event in events])

    def test_receiver_can_recover_r1_when_later_pair_is_clean(self) -> None:
        receiver = V32bisLogicalReceiver()
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
            r1_repetitions=3,
        )
        stream = flatten_startup_trace(trace)
        r1_start = next(i for i, obs in enumerate(stream) if obs.source_name == "R1")
        impaired = impair_stream_burst(stream, start=r1_start, length=8, replacement="X")
        events = receiver.ingest_all(impaired)
        r1_events = [event for event in events if event.name == "R1"]
        self.assertEqual(len(r1_events), 1)
        self.assertEqual(r1_events[0].rate_mask, RATE_4800 | RATE_7200 | RATE_9600)

    def test_random_light_corruption_still_leaves_some_protocol_events(self) -> None:
        receiver = V32bisLogicalReceiver()
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
            r1_repetitions=3,
            r3_repetitions=3,
        )
        stream = flatten_startup_trace(trace)
        impaired = impair_stream_random(stream, flip_probability=0.002, seed=7)
        events = receiver.ingest_all(impaired)
        event_names = [event.name for event in events]
        self.assertIn("S", event_names)
        self.assertIn("B1", event_names)

    def test_neighbor_q_confusions_can_preserve_some_protocol(self) -> None:
        receiver = V32bisLogicalReceiver()
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
            r1_repetitions=3,
            r3_repetitions=3,
        )
        stream = flatten_startup_trace(trace)
        impaired = impair_stream_q_neighbor(stream, every_nth_q=17)
        events = receiver.ingest_all(impaired)
        event_names = [event.name for event in events]
        self.assertIn("S", event_names)
        self.assertIn("B1", event_names)

    def test_q_insertion_can_break_initial_r1_but_allow_later_reacquisition(self) -> None:
        receiver = V32bisLogicalReceiver()
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
            r1_repetitions=3,
        )
        stream = flatten_startup_trace(trace)
        r1_start = next(i for i, obs in enumerate(stream) if obs.source_name == "R1")
        impaired = impair_stream_insert(stream, index=r1_start + 3, symbol="Q1")
        events = receiver.ingest_all(impaired)
        r1_events = [event for event in events if event.name == "R1"]
        self.assertEqual(len(r1_events), 1)
        self.assertEqual(r1_events[0].rate_mask, RATE_4800 | RATE_7200 | RATE_9600)
        self.assertGreater(receiver.stats().q_resync_shifts, 0)

    def test_q_deletion_can_break_initial_r1_but_allow_later_reacquisition(self) -> None:
        receiver = V32bisLogicalReceiver()
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
            r1_repetitions=3,
        )
        stream = flatten_startup_trace(trace)
        r1_start = next(i for i, obs in enumerate(stream) if obs.source_name == "R1")
        impaired = impair_stream(stream, drops={r1_start + 3})
        events = receiver.ingest_all(impaired)
        r1_events = [event for event in events if event.name == "R1"]
        self.assertEqual(len(r1_events), 1)
        self.assertEqual(r1_events[0].rate_mask, RATE_4800 | RATE_7200 | RATE_9600)
        self.assertGreater(receiver.stats().q_resync_shifts, 0)


class NegotiationTests(unittest.TestCase):
    def test_detect_s_sequence(self) -> None:
        self.assertTrue(detect_s_sequence([STATE_A, STATE_B] * 128))
        self.assertFalse(detect_s_sequence([STATE_A, STATE_C] * 128))

    def test_validate_and_decode_rate_signal(self) -> None:
        bits = rate_signal_bits(RATE_4800 | RATE_9600 | RATE_14400)
        self.assertTrue(validate_rate_signal_bits(bits))
        self.assertEqual(decode_rate_mask(bits), RATE_4800 | RATE_9600 | RATE_14400)

    def test_validate_and_decode_e_signal(self) -> None:
        bits = e_sequence_bits(7200)
        self.assertTrue(validate_e_sequence_bits(bits))
        self.assertEqual(decode_e_rate(bits), 7200)

    def test_repeated_rate_signal_detection_requires_two_identical_sequences(self) -> None:
        bits = rate_signal_bits(RATE_4800 | RATE_7200)
        self.assertEqual(detect_repeated_rate_signal([bits, bits]), RATE_4800 | RATE_7200)
        self.assertIsNone(detect_repeated_rate_signal([bits, rate_signal_bits(RATE_4800 | RATE_9600)]))

    def test_highest_common_rate(self) -> None:
        left = RATE_4800 | RATE_7200 | RATE_9600 | RATE_12000
        right = RATE_4800 | RATE_9600 | RATE_14400
        self.assertEqual(highest_common_rate(left, right), 9600)
        self.assertIsNone(highest_common_rate(RATE_7200, RATE_14400))

    def test_startup_negotiation(self) -> None:
        result = negotiate_startup_rate(
            RATE_4800 | RATE_7200 | RATE_9600 | RATE_12000,
            RATE_4800 | RATE_9600,
        )
        self.assertEqual(result.agreed_rate, 9600)
        self.assertFalse(result.cleardown)

    def test_startup_negotiation_cleardown_when_no_common_rate(self) -> None:
        result = negotiate_startup_rate(RATE_7200, RATE_14400)
        self.assertIsNone(result.agreed_rate)
        self.assertTrue(result.cleardown)

    def test_renegotiation_negotiates_highest_common_rate(self) -> None:
        result = negotiate_renegotiation_rate(
            RATE_4800 | RATE_7200 | RATE_9600,
            RATE_4800 | RATE_7200,
        )
        self.assertEqual(result.agreed_rate, 7200)
        self.assertFalse(result.cleardown)


class StartupSimulationTests(unittest.TestCase):
    def test_simulator_converges_on_highest_common_rate(self) -> None:
        result = simulate_startup(
            answer_supported_mask=RATE_4800 | RATE_7200 | RATE_9600 | RATE_12000,
            call_supported_mask=RATE_4800 | RATE_9600 | RATE_14400,
        )
        self.assertFalse(result.cleardown)
        self.assertEqual(result.answer_r1_mask, RATE_4800 | RATE_7200 | RATE_9600 | RATE_12000)
        self.assertEqual(result.call_r2_mask, RATE_4800 | RATE_9600)
        self.assertEqual(result.answer_r3_mask, RATE_4800 | RATE_9600)
        self.assertEqual(result.selected_rate, 9600)
        self.assertEqual(result.caller_e_rate, 9600)
        self.assertEqual(result.answerer_e_rate, 9600)

    def test_simulator_honors_answerer_subset_choice(self) -> None:
        result = simulate_startup(
            answer_supported_mask=RATE_4800 | RATE_7200 | RATE_9600 | RATE_12000,
            call_supported_mask=RATE_4800 | RATE_7200 | RATE_9600,
            answer_selected_mask=RATE_4800 | RATE_7200,
        )
        self.assertFalse(result.cleardown)
        self.assertEqual(result.answer_r3_mask, RATE_4800 | RATE_7200)
        self.assertEqual(result.selected_rate, 7200)

    def test_simulator_clears_down_without_common_rate(self) -> None:
        result = simulate_startup(
            answer_supported_mask=RATE_7200,
            call_supported_mask=RATE_14400,
        )
        self.assertTrue(result.cleardown)
        self.assertIsNone(result.selected_rate)
        self.assertIsNone(result.caller_e_rate)
        self.assertIsNone(result.answerer_e_rate)


class TxSymbolTests(unittest.TestCase):
    def test_startup_symbol_to_point_for_sync_states(self) -> None:
        self.assertEqual(startup_symbol_to_point("A"), complex(-6.0, -2.0))
        self.assertEqual(startup_symbol_to_point("B"), complex(-2.0, 6.0))
        self.assertEqual(startup_symbol_to_point("C"), complex(6.0, 2.0))
        self.assertEqual(startup_symbol_to_point("D"), complex(2.0, -6.0))

    def test_startup_symbol_to_point_for_q_state(self) -> None:
        self.assertEqual(startup_symbol_to_point("Q3"), complex(6.0, 2.0))

    def test_startup_trace_maps_to_complex_symbols(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        transmitted = startup_trace_to_complex_symbols(trace)
        self.assertGreater(len(transmitted), 0)
        self.assertEqual(transmitted[0].source_name, "conditioning")
        self.assertEqual(transmitted[0].symbol, "A")
        self.assertEqual(transmitted[0].point, complex(-6.0, -2.0))


class TxWaveformTests(unittest.TestCase):
    def test_rrc_taps_are_symmetric(self) -> None:
        taps = rrc_taps(0.5, 10, 8)
        self.assertEqual(len(taps), 81)
        for left, right in zip(taps, reversed(taps)):
            self.assertAlmostEqual(left, right, places=12)

    def test_rrc_taps_have_unit_energy(self) -> None:
        taps = rrc_taps(0.5, 10, 8)
        energy = sum(tap * tap for tap in taps)
        self.assertAlmostEqual(energy, 1.0, places=12)

    def test_symbols_to_baseband_returns_expected_length(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        symbols = startup_trace_to_complex_symbols(trace[:1])
        waveform = symbols_to_baseband(symbols, samples_per_symbol=10, beta=0.5, span_symbols=8)
        self.assertEqual(
            len(waveform.samples),
            len(symbols) * 10 + len(waveform.taps) - 1,
        )
        self.assertEqual(waveform.samples_per_symbol, 10)
        self.assertTrue(any(sample != 0j for sample in waveform.samples))

    def test_baseband_to_passband_returns_real_samples(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        symbols = startup_trace_to_complex_symbols(trace[:1])
        waveform = symbols_to_baseband(symbols, samples_per_symbol=10, beta=0.5, span_symbols=8)
        passband = baseband_to_passband(waveform, sample_rate=24000, carrier_hz=1800.0)
        self.assertEqual(len(passband.samples), len(waveform.samples))
        self.assertEqual(passband.sample_rate, 24000)
        self.assertEqual(passband.carrier_hz, 1800.0)
        self.assertTrue(any(abs(sample) > 0.0 for sample in passband.samples))


class RxFrontendTests(unittest.TestCase):
    def test_nearest_symbol_label_for_clean_sync_point(self) -> None:
        self.assertEqual(nearest_symbol_label(complex(-6.0, -2.0), "A"), "A")
        self.assertEqual(nearest_symbol_label(complex(6.0, 2.0), "Q3"), "Q3")

    def test_passband_to_baseband_returns_complex_samples(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        transmitted = startup_trace_to_complex_symbols(trace[:1])
        baseband = symbols_to_baseband(transmitted, samples_per_symbol=10, beta=0.5, span_symbols=8)
        passband = baseband_to_passband(baseband, sample_rate=24000, carrier_hz=1800.0)
        remixed = passband_to_baseband(passband)
        self.assertEqual(len(remixed), len(passband.samples))
        self.assertTrue(any(sample != 0j for sample in remixed))

    def test_clean_roundtrip_recovers_startup_symbols(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        transmitted = startup_trace_to_complex_symbols(trace[:2])
        baseband = symbols_to_baseband(transmitted, samples_per_symbol=10, beta=0.5, span_symbols=8)
        passband = baseband_to_passband(baseband, sample_rate=24000, carrier_hz=1800.0)
        recovered = recover_symbols_ideal(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
        )
        self.assertEqual(len(recovered), len(transmitted))
        self.assertEqual([symbol.decided_symbol for symbol in recovered[:16]], [symbol.symbol for symbol in transmitted[:16]])

    def test_timing_offset_changes_error_metric(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        transmitted = startup_trace_to_complex_symbols(trace[:1])
        baseband = symbols_to_baseband(transmitted, samples_per_symbol=10, beta=0.5, span_symbols=8)
        passband = baseband_to_passband(baseband, sample_rate=24000, carrier_hz=1800.0)
        recovered_0 = recover_symbols_with_timing_offset(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            offset=0,
        )
        recovered_5 = recover_symbols_with_timing_offset(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            offset=5,
        )
        self.assertLess(symbol_error_metric(recovered_0), symbol_error_metric(recovered_5))

    def test_timing_search_finds_best_offset(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        transmitted = startup_trace_to_complex_symbols(trace[:1])
        baseband = symbols_to_baseband(transmitted, samples_per_symbol=10, beta=0.5, span_symbols=8)
        passband = baseband_to_passband(baseband, sample_rate=24000, carrier_hz=1800.0)
        search = search_symbol_timing(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
        )
        self.assertEqual(search.offset, 0)
        self.assertEqual(len(search.recovered), len(transmitted))

    def test_carrier_mismatch_changes_error_metric(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        transmitted = startup_trace_to_complex_symbols(trace[:1])
        baseband = symbols_to_baseband(transmitted, samples_per_symbol=10, beta=0.5, span_symbols=8)
        passband = baseband_to_passband(baseband, sample_rate=24000, carrier_hz=1800.0)
        recovered_1800 = recover_symbols_with_frontend(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            carrier_hz=1800.0,
        )
        recovered_1810 = recover_symbols_with_frontend(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            carrier_hz=1810.0,
        )
        self.assertLess(symbol_error_metric(recovered_1800), symbol_error_metric(recovered_1810))

    def test_carrier_search_finds_best_frequency(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        transmitted = startup_trace_to_complex_symbols(trace[:1])
        baseband = symbols_to_baseband(transmitted, samples_per_symbol=10, beta=0.5, span_symbols=8)
        passband = baseband_to_passband(baseband, sample_rate=24000, carrier_hz=1800.0)
        search = search_carrier_frequency(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            carrier_candidates_hz=[1790.0, 1800.0, 1810.0],
        )
        self.assertEqual(search.carrier_hz, 1800.0)
        self.assertEqual(len(search.recovered), len(transmitted))

    def test_joint_search_finds_best_timing_and_carrier(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        transmitted = startup_trace_to_complex_symbols(trace[:1])
        baseband = symbols_to_baseband(transmitted, samples_per_symbol=10, beta=0.5, span_symbols=8)
        passband = baseband_to_passband(baseband, sample_rate=24000, carrier_hz=1800.0)
        search = search_timing_and_carrier(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            carrier_candidates_hz=[1790.0, 1800.0, 1810.0],
        )
        self.assertEqual(search.carrier_hz, 1800.0)
        self.assertEqual(search.timing_offset, 0)
        self.assertEqual(len(search.recovered), len(transmitted))

    def test_carrier_tracking_improves_mismatched_carrier_metric(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        transmitted = startup_trace_to_complex_symbols(trace[:1])
        baseband = symbols_to_baseband(transmitted, samples_per_symbol=10, beta=0.5, span_symbols=8)
        passband = baseband_to_passband(baseband, sample_rate=24000, carrier_hz=1800.0)
        mismatched = recover_symbols_with_frontend(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            carrier_hz=1810.0,
        )
        tracked = recover_symbols_with_carrier_tracking(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            carrier_hz=1810.0,
            phase_gain=0.3,
        )
        self.assertLess(tracked.metric, symbol_error_metric(mismatched))

    def test_timing_tracking_improves_bad_initial_offset(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        transmitted = startup_trace_to_complex_symbols(trace[:1])
        baseband = symbols_to_baseband(transmitted, samples_per_symbol=10, beta=0.5, span_symbols=8)
        passband = baseband_to_passband(baseband, sample_rate=24000, carrier_hz=1800.0)
        fixed = recover_symbols_with_timing_offset(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            offset=5,
        )
        tracked = recover_symbols_with_timing_tracking(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            timing_offset=5,
            timing_step=1,
        )
        self.assertLess(tracked.metric, symbol_error_metric(fixed))
        self.assertLessEqual(abs(tracked.final_offset), 5)

    def test_joint_tracking_improves_combined_timing_and_carrier_mismatch(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        transmitted = startup_trace_to_complex_symbols(trace[:1])
        baseband = symbols_to_baseband(transmitted, samples_per_symbol=10, beta=0.5, span_symbols=8)
        passband = baseband_to_passband(baseband, sample_rate=24000, carrier_hz=1800.0)
        fixed = recover_symbols_with_frontend(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            timing_offset=5,
            carrier_hz=1810.0,
        )
        tracked = recover_symbols_with_tracking(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            timing_offset=5,
            carrier_hz=1810.0,
            phase_gain=0.3,
            timing_step=1,
        )
        self.assertLess(tracked.metric, symbol_error_metric(fixed))

    def test_timing_loop_improves_bad_initial_offset_without_search(self) -> None:
        trace = generate_answer_startup_trace(
            r1_mask=RATE_4800 | RATE_7200 | RATE_9600,
            r2_mask=RATE_4800 | RATE_9600,
            r3_selected_rate=9600,
        )
        transmitted = startup_trace_to_complex_symbols(trace[:1])
        baseband = symbols_to_baseband(transmitted, samples_per_symbol=10, beta=0.5, span_symbols=8)
        passband = baseband_to_passband(baseband, sample_rate=24000, carrier_hz=1800.0)
        fixed = recover_symbols_with_timing_offset(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            offset=5,
        )
        loop = recover_symbols_with_timing_loop(
            passband,
            transmitted_symbols=transmitted,
            taps=baseband.taps,
            samples_per_symbol=10,
            timing_offset=5.0,
            timing_gain=0.02,
            early_late_spacing=0.5,
        )
        self.assertLess(loop.metric, symbol_error_metric(fixed))
        self.assertTrue(0.0 <= loop.final_offset < 10.0)


if __name__ == "__main__":
    unittest.main()
