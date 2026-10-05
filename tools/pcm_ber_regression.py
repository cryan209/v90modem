#!/usr/bin/env python3
"""Check PCM measurement failure detection and callback-boundary invariance."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / "pcm_ber_test"


class PCMMeasurements(unittest.TestCase):
    def measure(self, mode, *args):
        proc = subprocess.run([str(BINARY), "--mode", mode, *map(str, args)],
                              cwd=ROOT, capture_output=True, timeout=30)
        return proc.returncode, json.loads(proc.stdout)

    def test_callback_boundaries_do_not_change_payload(self):
        for mode in ("v90-downstream", "v92-upstream"):
            reference = None
            for chunk in (1, 37, 80, 160):
                code, row = self.measure(mode, "--seed", 7, "--delay", 83,
                                         "--chunk", chunk, "--bits", 16003)
                self.assertEqual(code, 0, row)
                measured = {k: row[k] for k in ("checked_bits", "bit_errors", "startup_complete", "rejected_frames")}
                self.assertEqual(row["checked_bits"], 16003)
                if reference is not None:
                    self.assertEqual(measured, reference)
                reference = measured

    def test_wire_corruption_is_never_forgiven(self):
        for mode, index in (("v90-downstream", 3100), ("v92-upstream", 700)):
            code, row = self.measure(mode, "--corrupt-sample", index)
            self.assertEqual(code, 1, row)
            self.assertEqual(row["status"], "errors")
            self.assertGreater(row["bit_errors"], 0)

    def test_seeded_analog_noise_is_repeatable(self):
        args = ("--noise-rms", 120, "--seed", 7)
        code, first = self.measure("v92-upstream", *args)
        self.assertEqual(code, 1, first)
        self.assertNotEqual(first["status"], "pass")
        self.assertEqual((code, first), self.measure("v92-upstream", *args))

    def test_invalid_configurations(self):
        for args in (("--mode", "v92-upstream", "--drn", "20"),
                     ("--noise-rms", "1"), ("--chunk", "0"),
                     ("--seed", ""), ("--corrupt-sample", "20000000"),
                     ("--unknown", "1")):
            proc = subprocess.run([str(BINARY), *args], cwd=ROOT, capture_output=True, timeout=30)
            self.assertEqual(proc.returncode, 2, args)

    def test_sweep_strict_failure_and_timeout(self):
        with tempfile.TemporaryDirectory(prefix="pcm-regression-") as tmp:
            common = ["python3", str(ROOT / "tools/pcm_sweep.py"), "--modes", "v92-upstream",
                      "--laws", "ulaw", "--v92-drns", "19"]
            for name, extra, expected in (("strict", ["--require-clean"], 1),
                                          ("timeout", ["--timeout", "0.000001"], 2)):
                out = Path(tmp) / name
                proc = subprocess.run(common + extra + ["--output", str(out)],
                                      cwd=ROOT, capture_output=True, timeout=30)
                self.assertEqual(proc.returncode, expected, proc.stderr)
                rows = [json.loads(line) for line in (out / "results.jsonl").read_text().splitlines()]
                self.assertEqual(len(rows), 1)
                self.assertNotEqual(rows[0]["status"], "pass")
                if expected == 1:
                    self.assertIsNone(rows[0]["ber"])
                    self.assertEqual(rows[0]["checked_bits"], 0)


if __name__ == "__main__":
    unittest.main()
