#!/usr/bin/env python3
"""Regression tests for bounded logging; no DSP/hardware dependencies."""

import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest

from rotate_log import RotatingLog
from run_tests import run_one


class LogCaptureTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "capture.log"

    def test_small_log_is_unchanged(self):
        with RotatingLog(self.path, 8) as out:
            out.write(b"hello\n")
        self.assertEqual(self.path.read_bytes(), b"hello\n")
        self.assertFalse(Path(str(self.path) + ".1").exists())

    def test_rotation_and_large_write(self):
        with RotatingLog(self.path, 8) as out:
            out.write(b"abcdefgh")
            self.assertFalse(Path(str(self.path) + ".1").exists())
            out.write(b"ijklmnopqrstuvwxyz")
        self.assertEqual(self.path.read_bytes(), b"yz")
        self.assertEqual(Path(str(self.path) + ".1").read_bytes(), b"qrstuvwx")

    def test_new_run_removes_old_backup(self):
        with RotatingLog(self.path, 2) as out:
            out.write(b"abcdef")
        with RotatingLog(self.path, 2) as out:
            out.write(b"z")
        self.assertEqual(self.path.read_bytes(), b"z")
        self.assertFalse(Path(str(self.path) + ".1").exists())

    def test_invalid_limit(self):
        with self.assertRaises(ValueError):
            RotatingLog(self.path, 0)

    def run_command(self, script, timeout=5):
        cmd = shlex.join([sys.executable, "-c", script])
        result = run_one(0, ("test.list", 1, cmd), self.tmp.name, timeout, 1024)
        self.addCleanup(shutil.rmtree, result[-1], True)
        log = Path(result[-2])
        segments = [p for p in (Path(str(log) + ".1"), log) if p.exists()]
        self.assertTrue(all(p.stat().st_size <= 1024 for p in segments))
        return result, b"".join(p.read_bytes() for p in segments)

    def test_noisy_success_keeps_tail(self):
        result, data = self.run_command(
            "import os; os.write(1, b'x' * 1000000); os.write(2, b'FINAL ERROR DETAIL\\n')")
        self.assertEqual(result[2:4], ("PASS", 0))
        self.assertTrue(data.endswith(b"FINAL ERROR DETAIL\n"))
        self.assertFalse(os.path.exists(result[-1]))

    def test_failure_preserves_status_and_scratch(self):
        result, data = self.run_command("import sys; print('failure'); sys.exit(7)")
        self.assertEqual(result[2:4], ("FAIL", 7))
        self.assertIn(b"failure", data)
        self.assertTrue(os.path.isdir(result[-1]))

    def test_timeout_drains_output(self):
        result, _ = self.run_command(
            "import os\nwhile True: os.write(1, b'x' * 65536)", timeout=0.2)
        self.assertEqual(result[2:4], ("TIMEOUT", None))

    def test_closed_stdout_does_not_end_test(self):
        result, _ = self.run_command(
            "import os, time; os.close(1); os.close(2); time.sleep(5)", timeout=0.2)
        self.assertEqual(result[2:4], ("TIMEOUT", None))

    def test_cli(self):
        subprocess.run([sys.executable, str(Path(__file__).with_name("rotate_log.py")),
                        str(self.path), "--segment-mib", str(8 / 1024**2)],
                       input=b"abcdefghijklmnopqrstuvwxyz", check=True)
        self.assertEqual(self.path.read_bytes(), b"yz")
        self.assertEqual(Path(str(self.path) + ".1").read_bytes(), b"qrstuvwx")


if __name__ == "__main__":
    unittest.main()
