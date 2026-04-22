"""Smoke tests for the V.32bis WAV interop harness."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from tools.v32bis_wav_harness import (
    analyze_startup_wav,
    generate_data_wav,
    generate_startup_wav,
    read_pcm16_wav,
)
from tools.v32bis_datapump.tx import TxConfig
from tools.v32bis_datapump.rx import RxConfig


class V32bisWavHarnessTests(unittest.TestCase):
    def test_generate_and_analyze_startup_wav(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "startup.wav"
            tx_config = TxConfig()
            generate_startup_wav(path, role="answer", config=tx_config)
            result = analyze_startup_wav(
                path,
                role="answer",
                tx_config=tx_config,
                rx_config=RxConfig(),
                blind=False,
            )
            self.assertEqual([event["name"] for event in result["events"]], ["S", "R1", "S", "R3", "E", "B1"])

    def test_generate_data_wav_creates_samples(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "data.wav"
            result = generate_data_wav(
                path,
                bit_rate=9600,
                n_symbols=64,
                seed=7,
                calling_party=True,
                config=TxConfig(),
            )
            waveform = read_pcm16_wav(path)
            self.assertGreater(result["sample_count"], 0)
            self.assertEqual(len(waveform.samples), result["sample_count"])

    def test_analyze_startup_search_recovers_from_bad_hints(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "startup.wav"
            tx_config = TxConfig()
            generate_startup_wav(path, role="answer", config=tx_config)
            result = analyze_startup_wav(
                path,
                role="answer",
                tx_config=tx_config,
                rx_config=RxConfig(rx_carrier_hz=1810.0, timing_offset=5.0),
                blind=False,
                search_carrier_hz=12.0,
                search_carrier_step_hz=2.0,
                search_all_timing=True,
            )
            self.assertEqual([event["name"] for event in result["events"]], ["S", "R1", "S", "R3", "E", "B1"])
            self.assertGreater(result["candidate_count"], 1)


if __name__ == "__main__":
    unittest.main()
