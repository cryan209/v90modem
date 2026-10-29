#!/usr/bin/env python3
"""PERF parser tests and a framed-bearer engine smoke test (no hardware)."""

from pathlib import Path
import os
import struct
import subprocess
import tempfile
import unittest

from media_perf_report import read_records

ROOT = Path(__file__).resolve().parent.parent


class MediaPerfReportTest(unittest.TestCase):
    def test_parser_and_multiple_calls(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "modem.log"
            row = ("[PERF] scope=g711 dir=rx phase=data calls=10 audio_ms=200.000 "
                   "wall_ms=5.000 mean_us=500.000 p95_us_upper=750.000 "
                   "p99_us_upper=750.000 max_us=750.000 over_budget=0 transitions=1\n")
            log.write_text("unrelated message\n" + row + "[ME] note\n" + row)
            records = read_records(log)
            self.assertEqual(len(records), 2)
            self.assertEqual([r["line"] for r in records], [2, 4])
            self.assertEqual(records[0]["calls"], 10)
            self.assertEqual(records[0]["max_us"], 750.0)

    def test_incomplete_record_is_not_silently_scored(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "modem.log"
            log.write_text("[PERF] scope=g711 dir=rx phase=data calls=10\n")
            with self.assertRaisesRegex(ValueError, "incomplete/invalid"):
                read_records(log)

    def test_engine_records_requested_block_sizes(self):
        self.check_engine(quiet=False)

    def test_quiet_engine_still_reports(self):
        quiet = self.check_engine(quiet=True)
        verbose = self.check_engine(quiet=False)
        self.assertEqual(quiet, verbose, "diagnostic gating must not change transmitted octets")

    def check_engine(self, quiet):
        # The engine peer frames PCM codewords but otherwise uses the real
        # engine, including call startup, teardown and summary output.
        with tempfile.TemporaryDirectory(prefix="v90-perf-") as tmp:
            sizes = [80, 160, 320, 160, 80]
            bearer = b"".join(struct.pack("<H", n) + b"\xff" * n for n in sizes)
            result = subprocess.run(
                [str(ROOT / "v90_engine_peer"), str(Path(tmp) / "pty")],
                input=bearer, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                cwd=ROOT, env=dict(os.environ, TMPDIR=tmp, ME_DUMP_DIR=tmp,
                                  ME_MODE="v21", ME_SLOW_CALLBACK_US="1",
                                  ENGINE_PEER_QUIET=str(int(quiet))),
                timeout=30, check=True)
            self.assertEqual(len(result.stdout), sum(sizes))
            log = Path(tmp) / "peer.log"
            log.write_bytes(result.stderr)
            rows = read_records(log)
            self.assertTrue(rows)
            for direction in ("rx", "tx"):
                selected = [r for r in rows if r["dir"] == direction]
                self.assertEqual(sum(r["calls"] for r in selected), len(sizes))
                self.assertAlmostEqual(sum(r["audio_ms"] for r in selected), sum(sizes) / 8)
                self.assertTrue(all(r["p99_us_upper"] <= r["max_us"] for r in selected))
            slow_lines = result.stderr.count(b"[ME] slow media callback:")
            self.assertLessEqual(slow_lines, 4)
            if quiet:
                self.assertEqual(slow_lines, 0)
            return result.stdout


if __name__ == "__main__":
    unittest.main()
