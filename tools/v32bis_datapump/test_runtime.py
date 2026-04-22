"""Runtime datapump tests."""

from __future__ import annotations

import unittest

from tools.v32bis_datapump import ChannelConfig, RxConfig, TxConfig, V32bisDatapump


class DatapumpRuntimeTests(unittest.TestCase):
    def test_datapump_startup_matches_oracle_on_clean_channel(self) -> None:
        datapump = V32bisDatapump()
        result = datapump.run_startup()
        self.assertTrue(result.oracle.matches)
        self.assertTrue(result.recovery.mode.startswith("carrier"))

    def test_datapump_startup_matches_oracle_on_combined_channel(self) -> None:
        datapump = V32bisDatapump(
            tx_config=TxConfig(),
            rx_config=RxConfig(),
            channel_config=ChannelConfig(
                gain=0.8,
                snr_db=28.0,
                noise_seed=11,
                drift_hz_per_sample=1e-6,
                fir_taps=(0.9, 0.25, -0.1),
                echo_delay=6,
                echo_gain=0.25,
            ),
        )
        result = datapump.run_startup()
        self.assertTrue(result.oracle.matches)

    def test_datapump_startup_matches_oracle_on_multi_echo_channel(self) -> None:
        datapump = V32bisDatapump(
            channel_config=ChannelConfig(
                gain=0.8,
                snr_db=6.0,
                noise_seed=11,
                drift_hz_per_sample=1e-6,
                fir_taps=(0.9, 0.25, -0.1),
                multi_echo_paths=((4, 0.2), (11, -0.1)),
            ),
        )
        result = datapump.run_startup()
        self.assertTrue(result.oracle.matches)

    def test_datapump_near_end_echo_breaks_startup_without_cancellation(self) -> None:
        datapump = V32bisDatapump(
            channel_config=ChannelConfig(
                gain=0.8,
                snr_db=6.0,
                noise_seed=1,
                drift_hz_per_sample=1e-6,
                fir_taps=(0.9, 0.25, -0.1),
                near_end_echo_paths=((3, 0.9), (9, 0.45)),
            ),
        )
        result = datapump.run_startup()
        self.assertFalse(result.oracle.matches)

    def test_datapump_near_end_echo_cancellation_restores_startup(self) -> None:
        datapump = V32bisDatapump(
            channel_config=ChannelConfig(
                gain=0.8,
                snr_db=6.0,
                noise_seed=1,
                drift_hz_per_sample=1e-6,
                fir_taps=(0.9, 0.25, -0.1),
                near_end_echo_paths=((3, 0.9), (9, 0.45)),
                cancel_near_end_echo=True,
            ),
        )
        result = datapump.run_startup()
        self.assertTrue(result.oracle.matches)

    def test_datapump_near_end_echo_cancellation_tolerates_small_path_error(self) -> None:
        datapump = V32bisDatapump(
            channel_config=ChannelConfig(
                gain=0.8,
                snr_db=6.0,
                noise_seed=1,
                drift_hz_per_sample=1e-6,
                fir_taps=(0.9, 0.25, -0.1),
                near_end_echo_paths=((3, 0.9), (9, 0.45)),
                cancel_near_end_echo=True,
                near_end_echo_estimate_paths=((3, 0.82), (9, 0.4)),
            ),
        )
        result = datapump.run_startup()
        self.assertTrue(result.oracle.matches)

    def test_datapump_adaptive_near_end_echo_cancellation_restores_startup(self) -> None:
        datapump = V32bisDatapump(
            channel_config=ChannelConfig(
                gain=0.8,
                snr_db=6.0,
                noise_seed=1,
                drift_hz_per_sample=1e-6,
                fir_taps=(0.9, 0.25, -0.1),
                near_end_echo_paths=((3, 0.9), (9, 0.45)),
                adaptive_near_end_echo_cancel=True,
            ),
        )
        result = datapump.run_startup()
        self.assertTrue(result.oracle.matches)

    def test_datapump_adaptive_near_end_echo_cancellation_uses_configured_filter(self) -> None:
        datapump = V32bisDatapump(
            channel_config=ChannelConfig(
                gain=0.8,
                snr_db=6.0,
                noise_seed=1,
                drift_hz_per_sample=1e-6,
                fir_taps=(0.9, 0.25, -0.1),
                near_end_echo_paths=((3, 0.9), (9, 0.45)),
                adaptive_near_end_echo_cancel=True,
                adaptive_echo_tap_count=24,
                adaptive_echo_step_size=0.001,
            ),
        )
        result = datapump.run_startup()
        self.assertTrue(result.oracle.matches)

    def test_blind_runtime_recovers_full_startup_on_clean_distorted_channel(self) -> None:
        datapump = V32bisDatapump(
            channel_config=ChannelConfig(
                gain=0.8,
                snr_db=28.0,
                noise_seed=1,
                drift_hz_per_sample=1e-6,
                fir_taps=(0.9, 0.25, -0.1),
            ),
            blind_runtime=True,
        )
        result = datapump.run_startup()
        names = [event.name for event in result.recovery.events]
        self.assertEqual(names, ["S", "R1", "S", "R3", "E", "B1"])
        self.assertTrue(result.oracle.matches)

    def test_blind_runtime_recovers_full_startup_on_multi_echo_channel(self) -> None:
        datapump = V32bisDatapump(
            channel_config=ChannelConfig(
                gain=0.8,
                snr_db=6.0,
                noise_seed=1,
                drift_hz_per_sample=1e-6,
                fir_taps=(0.9, 0.25, -0.1),
                multi_echo_paths=((4, 0.2), (11, -0.1)),
            ),
            blind_runtime=True,
        )
        result = datapump.run_startup()
        names = [event.name for event in result.recovery.events]
        self.assertEqual(names, ["S", "R1", "S", "R3", "E", "B1"])
        self.assertTrue(result.oracle.matches)

    def test_blind_runtime_recovers_full_startup_with_adaptive_near_end_echo_cancellation(self) -> None:
        datapump = V32bisDatapump(
            channel_config=ChannelConfig(
                gain=0.8,
                snr_db=6.0,
                noise_seed=1,
                drift_hz_per_sample=1e-6,
                fir_taps=(0.9, 0.25, -0.1),
                near_end_echo_paths=((3, 0.9), (9, 0.45)),
                adaptive_near_end_echo_cancel=True,
            ),
            blind_runtime=True,
        )
        result = datapump.run_startup()
        names = [event.name for event in result.recovery.events]
        self.assertEqual(names, ["S", "R1", "S", "R3", "E", "B1"])
        self.assertTrue(result.oracle.matches)

    def test_datapump_data_mode_clean_channel_has_zero_ber_at_all_rates(self) -> None:
        datapump = V32bisDatapump()
        for rate in (4800, 7200, 9600, 12000, 14400):
            result = datapump.run_data(bit_rate=rate, n_symbols=256, seed=7)
            self.assertEqual(result.bit_errors, 0, f"clean-channel BER failed at {rate} bps")

    def test_datapump_data_mode_awgn16_stays_within_rate_specific_ber_targets(self) -> None:
        datapump = V32bisDatapump(
            channel_config=ChannelConfig(
                snr_db=16.0,
                noise_seed=11,
            ),
        )
        ber_limits = {
            4800: 0.0,
            7200: 0.0,
            9600: 0.0,
            12000: 0.0,
            14400: 0.005,
        }
        for rate, max_ber in ber_limits.items():
            result = datapump.run_data(bit_rate=rate, n_symbols=512, seed=7)
            self.assertLessEqual(
                result.ber,
                max_ber,
                f"AWGN BER target failed at {rate} bps: got {result.ber:.6f}",
            )

    def test_datapump_data_mode_block_agc_handles_gain_scaled_awgn(self) -> None:
        datapump = V32bisDatapump(
            channel_config=ChannelConfig(
                gain=0.8,
                snr_db=20.0,
                noise_seed=11,
            ),
        )
        for rate in (4800, 7200, 9600, 12000, 14400):
            result = datapump.run_data(bit_rate=rate, n_symbols=512, seed=7)
            self.assertEqual(result.bit_errors, 0, f"gain-scaled BER failed at {rate} bps")
            self.assertGreater(result.recovery.agc_gain, 1.15)
            self.assertLess(result.recovery.agc_gain, 1.35)

    def test_datapump_data_mode_equalizer_handles_mild_combined_channel(self) -> None:
        datapump = V32bisDatapump(
            channel_config=ChannelConfig(
                gain=0.8,
                snr_db=28.0,
                noise_seed=11,
                drift_hz_per_sample=1e-6,
                fir_taps=(0.9, 0.25, -0.1),
                echo_delay=6,
                echo_gain=0.25,
            ),
        )
        ber_limits = {
            4800: 0.0,
            7200: 0.0,
            9600: 0.0,
            12000: 0.0,
            14400: 0.006,
        }
        for rate, max_ber in ber_limits.items():
            result = datapump.run_data(bit_rate=rate, n_symbols=512, seed=7)
            self.assertGreaterEqual(result.recovery.equalizer_training_symbols, 0)
            self.assertLessEqual(
                result.ber,
                max_ber,
                f"combined-channel BER target failed at {rate} bps: got {result.ber:.6f}",
            )

    def test_datapump_data_mode_equalizer_recovers_multipath_channel_up_to_12000(self) -> None:
        datapump = V32bisDatapump(
            channel_config=ChannelConfig(
                gain=0.8,
                snr_db=20.0,
                noise_seed=11,
                fir_taps=(0.9, 0.25, -0.1),
                multi_echo_paths=((4, 0.2), (11, -0.1)),
            ),
        )
        ber_limits = {
            4800: 0.0,
            7200: 0.0,
            9600: 0.0,
            12000: 0.0,
        }
        for rate, max_ber in ber_limits.items():
            result = datapump.run_data(bit_rate=rate, n_symbols=512, seed=7)
            self.assertEqual(result.recovery.equalizer_training_symbols, 0 if rate == 4800 else 128)
            self.assertLessEqual(
                result.ber,
                max_ber,
                f"multipath BER target failed at {rate} bps: got {result.ber:.6f}",
            )


if __name__ == "__main__":
    unittest.main()
