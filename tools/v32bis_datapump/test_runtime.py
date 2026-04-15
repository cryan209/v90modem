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


if __name__ == "__main__":
    unittest.main()
