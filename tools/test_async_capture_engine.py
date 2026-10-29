#!/usr/bin/env python3
"""Background tap checks; build v90_engine_peer and v90_engine_replay first."""

import json
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parent.parent


class AsyncCaptureEngineTest(unittest.TestCase):
    sizes = (80, 160, 320, 160, 80)
    octets = bytes((i * 73 + 11) % 256 for i in range(sum(sizes)))

    def run_peer(self, directory, alaw=False, capture=False, mode="v21", sizes=None, octets=None):
        directory = Path(directory)
        sizes = self.sizes if sizes is None else sizes
        octets = self.octets if octets is None else octets
        self.assertEqual(sum(sizes), len(octets))
        env = dict(os.environ, TMPDIR=str(directory), ME_DUMP_DIR=str(directory),
                   ME_MODE=mode, ENGINE_PEER_QUIET="1")
        env.pop("VPCM_G711_TAP_DIR", None)
        env.pop("ME_G711_CAPTURE", None)
        if capture:
            env.update(VPCM_G711_TAP_DIR=str(directory / "taps"),
                       ME_G711_CAPTURE=str(directory / "prefix"))
        pos = 0
        blocks = []
        for size in sizes:
            blocks.append(struct.pack("<H", size) + octets[pos:pos + size])
            pos += size
        cmd = [str(ROOT / "v90_engine_peer"), str(directory / "pty")]
        if alaw:
            cmd.append("--alaw")
        return subprocess.run(cmd, input=b"".join(blocks), cwd=ROOT, env=env,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              timeout=30, check=True)

    def check_capture(self, path, expected):
        self.assertEqual(path.read_bytes(), expected)
        status = json.loads(Path(str(path) + ".status.json").read_text())
        self.assertEqual(status["state"], "complete")
        self.assertEqual(status["error"], 0)
        self.assertEqual(status["rejected_bytes"], 0)
        for field in ("offered_bytes", "accepted_bytes", "written_bytes"):
            self.assertEqual(status[field], len(expected))
        self.assertTrue(status["prefix_only"])

    def test_both_laws_are_byte_exact_and_do_not_change_transmit(self):
        for mode in ("v21", "clear", "x2-symm"):
            for alaw in (False, True):
                with self.subTest(mode=mode, alaw=alaw), tempfile.TemporaryDirectory(prefix="v90-taps-") as tmp:
                    baseline = self.run_peer(tmp, alaw=alaw, mode=mode)
                    captured = self.run_peer(tmp, alaw=alaw, capture=True, mode=mode)
                    self.assertEqual(captured.stdout, baseline.stdout)
                    self.assertEqual(len(captured.stdout), len(self.octets))
                    directory = Path(tmp)
                    self.check_capture(directory / "taps/live-rx.g711", self.octets)
                    self.check_capture(directory / "taps/live-tx.g711", captured.stdout)
                    self.check_capture(directory / "prefix.rx.ulaw", self.octets)

    def test_long_fast_bearer_preserves_complete_captures(self):
        with tempfile.TemporaryDirectory(prefix="v90-taps-") as tmp:
            sizes = (160,) * 4000  # 80 s simulated DS0; well over two queue lengths.
            octets = b"\xff" * sum(sizes)
            result = self.run_peer(tmp, capture=True, mode="clear", sizes=sizes, octets=octets)
            self.assertEqual(len(result.stdout), len(octets))
            directory = Path(tmp)
            self.check_capture(directory / "taps/live-rx.g711", octets)
            self.check_capture(directory / "taps/live-tx.g711", result.stdout)
            self.check_capture(directory / "prefix.rx.ulaw", octets)

    def test_fast_replay_batches_normal_and_recorded_schedules(self):
        for scheduled in (False, True):
            with self.subTest(scheduled=scheduled), tempfile.TemporaryDirectory(prefix="v90-taps-") as tmp:
                directory = Path(tmp)
                octets = b"\xff" * (160 * 4000)
                tap = directory / "input.g711"
                tap.write_bytes(octets)
                cmd = [str(ROOT / "v90_engine_replay"), str(tap), "--fast", "--from", "0",
                       "--pty-link", str(directory / "pty")]
                if scheduled:
                    schedule = directory / "schedule.bin"
                    records = []
                    for i in range(4000):
                        for kind, offset in ((b"R", 0), (b"T", 1000000)):
                            records.append(struct.pack("=c3xiQ", kind, 160, (i + 1) * 20000000 + offset))
                    schedule.write_bytes(b"".join(records))
                    cmd.extend(["--schedule", str(schedule)])
                else:
                    cmd.append("--split")
                env = dict(os.environ, TMPDIR=tmp, ME_DUMP_DIR=tmp, ME_MODE="clear",
                           VPCM_G711_TAP_DIR=str(directory / "taps"),
                           ME_G711_CAPTURE=str(directory / "prefix"))
                subprocess.run(cmd, cwd=ROOT, env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, timeout=30, check=True)
                self.check_capture(directory / "taps/live-rx.g711", octets)
                self.check_capture(directory / "prefix.rx.ulaw", octets)
                tx = directory / "taps/live-tx.g711"
                self.assertEqual(tx.stat().st_size, len(octets))
                self.check_capture(tx, tx.read_bytes())

    def test_tap_setup_failure_does_not_break_engine_or_independent_prefix(self):
        with tempfile.TemporaryDirectory(prefix="v90-taps-") as tmp:
            baseline = self.run_peer(tmp)
            directory = Path(tmp)
            (directory / "taps/live-rx.g711.status.json").mkdir(parents=True)
            result = self.run_peer(tmp, capture=True)
            self.assertEqual(result.stdout, baseline.stdout)
            self.assertIn(b"Unable to open live G.711 taps", result.stderr)
            self.check_capture(directory / "prefix.rx.ulaw", self.octets)
            self.check_capture(directory / "taps/live-tx.g711", result.stdout)
            self.assertFalse((directory / "taps/live-rx.g711").exists())

    def test_forced_kill_does_not_leave_a_completion_certificate(self):
        with tempfile.TemporaryDirectory(prefix="v90-taps-") as tmp:
            directory = Path(tmp)
            env = dict(os.environ, TMPDIR=tmp, ME_DUMP_DIR=tmp,
                       VPCM_G711_TAP_DIR=str(directory / "taps"),
                       ENGINE_PEER_QUIET="1")
            env.pop("ME_G711_CAPTURE", None)
            proc = subprocess.Popen([str(ROOT / "v90_engine_peer"), str(directory / "pty")],
                                    cwd=ROOT, env=env, stdin=subprocess.PIPE,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                status_path = directory / "taps/live-tx.g711.status.json"
                deadline = time.monotonic() + 5
                while True:
                    if status_path.exists():
                        try:
                            status = json.loads(status_path.read_text())
                            if status["state"] == "recording":
                                break
                        except json.JSONDecodeError:
                            pass
                    self.assertIsNone(proc.poll(), "engine exited before capture setup")
                    self.assertLess(time.monotonic(), deadline, "capture setup timed out")
                    time.sleep(0.01)
                proc.kill()
                proc.wait(timeout=5)
                status = json.loads(status_path.read_text())
                self.assertEqual(status["state"], "recording")
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.wait()
                proc.stdin.close()


if __name__ == "__main__":
    unittest.main()
