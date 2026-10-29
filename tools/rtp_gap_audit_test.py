#!/usr/bin/env python3
"""Synthetic metadata tests; not a modem or a network-loss qualification."""
import csv
import tempfile
import unittest
from pathlib import Path
from rtp_gap_audit import audit


class GapAuditTest(unittest.TestCase):
    def run_audit(self, packets, law="ulaw", fill=None):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            with (path / "rtp-rx.csv").open("w", newline="") as out:
                writer = csv.writer(out)
                writer.writerow(["ssrc", "seq", "timestamp"])
                writer.writerows(packets)
            samples = bytearray(b"\x55" * 8000)
            if fill is not None:
                samples[1760:2080] = bytes([fill]) * 320
            (path / "live-rx.g711").write_bytes(samples)
            (path / "server.log").write_text(
                "[TRACE +278ms] V90 upstream discontinuity resync\n")
            return audit(path, law)

    def test_gap_and_fill(self):
        result = self.run_audit([(9, 0, 0), (9, 3, 480)], fill=255)
        self.assertEqual(result["missing_packets_at_end"], 2)
        gap = result["gaps"][0]
        self.assertEqual(gap["nearest_fill_candidate"]["samples"], 320)
        self.assertAlmostEqual(gap["fill_alignment_error_s"], 0)
        self.assertEqual(result["resync_trace_s"], [.278])

    def test_late_packet_is_not_missing(self):
        result = self.run_audit([(9, 0, 0), (9, 2, 320), (9, 1, 160)])
        self.assertEqual(result["missing_packets_at_end"], 0)
        self.assertEqual(result["gaps"][0]["observed_elsewhere"], 1)

    def test_overlapping_gaps_are_not_counted_twice(self):
        result = self.run_audit([(9, 0, 0), (9, 3, 480), (9, 1, 160), (9, 4, 640)])
        self.assertEqual(result["missing_packets_at_end"], 1)

    def test_wrap(self):
        result = self.run_audit([(9, 65534, 0xFFFFFF00), (9, 1, 224)], fill=255)
        self.assertEqual(result["missing_packets_at_end"], 2)
        self.assertAlmostEqual(result["gaps"][0]["fill_alignment_error_s"], 0)

    def test_silence_alone_is_not_loss(self):
        result = self.run_audit([(9, 0, 0), (9, 1, 160)], fill=255)
        self.assertEqual(len(result["fill_candidate_runs"]), 1)
        self.assertEqual(result["gaps"], [])

    def test_timestamp_disagreement(self):
        result = self.run_audit([(9, 0, 0), (9, 3, 640)])
        self.assertEqual(result["gaps"], [])

    def test_ssrc_change(self):
        result = self.run_audit([(9, 0, 0), (10, 3, 480)])
        self.assertEqual(result["gaps"], [])

    def test_alaw(self):
        packets = [(9, 0, 0), (9, 3, 480)]
        self.assertEqual(len(self.run_audit(packets, "alaw", 0xD5)["fill_candidate_runs"]), 1)
        self.assertEqual(self.run_audit(packets, "alaw", 0xFF)["fill_candidate_runs"], [])


if __name__ == "__main__":
    unittest.main()
