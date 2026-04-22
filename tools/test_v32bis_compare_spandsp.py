"""Tests for the local SpanDSP V.32bis comparison harness."""

from __future__ import annotations

import unittest
from pathlib import Path

from tools.v32bis_compare_spandsp import (
    REPO_ROOT,
    build_report,
    parse_spandsp_constellation_maps,
    parse_spandsp_scrambler_taps,
    python_scrambler_taps,
    simulate_spandsp_tx_symbols,
)


class CompareSpanDSPTests(unittest.TestCase):
    def test_parses_expected_constellation_table_sizes(self) -> None:
        header = REPO_ROOT / "spandsp-master/src/v17_v32bis_tx_constellation_maps.h"
        parsed = parse_spandsp_constellation_maps(header)
        self.assertEqual(len(parsed[14400]), 128)
        self.assertEqual(len(parsed[12000]), 64)
        self.assertEqual(len(parsed[9600]), 32)
        self.assertEqual(len(parsed[7200]), 16)
        self.assertEqual(len(parsed[4800]), 4)

    def test_scrambler_direction_assignments_match_python_reference(self) -> None:
        c_path = REPO_ROOT / "spandsp-master/src/v32bis.c"
        self.assertEqual(parse_spandsp_scrambler_taps(c_path), python_scrambler_taps())

    def test_build_report_includes_all_rates(self) -> None:
        report = build_report(
            REPO_ROOT / "spandsp-master/src/v17_v32bis_tx_constellation_maps.h",
            REPO_ROOT / "spandsp-master/src/v32bis.c",
        )
        self.assertEqual([row["rate"] for row in report["constellations"]], [14400, 12000, 9600, 7200, 4800])
        self.assertEqual([row["rate"] for row in report["emitted_symbols"]], [14400, 12000, 9600, 7200, 4800])
        self.assertEqual([row["rate"] for row in report["emitted_symbol_diagnostics"]], [14400, 12000, 9600, 7200, 4800])
        self.assertEqual(report["stage_diagnostics"]["rate"], 12000)
        self.assertEqual(report["startup_handoff_diagnostics"]["rate"], 12000)
        self.assertTrue(report["scrambler_taps"]["same"])

    def test_simulated_spandsp_stream_has_expected_length(self) -> None:
        header = REPO_ROOT / "spandsp-master/src/v17_v32bis_tx_constellation_maps.h"
        maps = parse_spandsp_constellation_maps(header)
        bits = [0, 1, 1, 0, 1, 0, 0, 1]
        symbols = simulate_spandsp_tx_symbols(4800, bits, calling_party=True, spandsp_maps=maps)
        self.assertEqual(len(symbols), 4)


if __name__ == "__main__":
    unittest.main()
