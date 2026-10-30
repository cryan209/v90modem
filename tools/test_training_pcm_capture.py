#!/usr/bin/env python3
"""Training PCM diagnostics; build engine_pair_test, peer and replay first.

Generate private inputs with a complete V.34 call, then compare deterministic
training replays with/without PCM capture. This is not hardware qualification.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
RECORD = struct.Struct("=c3xiQ")


class TrainingPCMTests(unittest.TestCase):
    def env(self, directory):
        env = dict(os.environ, TMPDIR=str(directory), ME_MODE="v34")
        for name in ("ME_DUMP_DIR", "VPCM_G711_TAP_DIR", "ME_G711_CAPTURE", "ME_IO_SCHEDULE", "ME_GUI_PORT",
                     "ME_V34_SPAN_FLOW_LOG", "V34_DUPLEX_LOG"):
            env.pop(name, None)
        return env

    def capture(self, path, expected=None):
        raw = path.read_bytes()
        status = json.loads(Path(str(path) + ".status.json").read_text())
        self.assertEqual(status["state"], "complete", status)
        self.assertEqual(status["error"], 0, status)
        self.assertEqual(status["rejected_bytes"], 0, status)
        for field in ("offered_bytes", "accepted_bytes", "written_bytes"):
            self.assertEqual(status[field], len(raw), status)
        if expected is not None:
            self.assertEqual(len(raw), len(expected))
            self.assertEqual(hashlib.sha256(raw).digest(), hashlib.sha256(expected).digest())
        return raw

    def peer(self, directory, dump):
        env = self.env(directory)
        env.update(ME_MODE="clear", ENGINE_PEER_QUIET="1")
        if dump is not None:
            env["ME_DUMP_DIR"] = str(dump)
        return subprocess.run([str(ROOT / "v90_engine_peer"), str(directory / "pty")],
                              input=struct.pack("<H", 160) + b"\xff" * 160,
                              env=env, cwd=ROOT, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, timeout=15, check=True)

    def test_setup_is_opt_in_and_independent(self):
        with tempfile.TemporaryDirectory(prefix="v90-pcm-") as tmp:
            root = Path(tmp)
            for value in (None, ""):
                result = self.peer(root, value)
                self.assertFalse((root / "v34_rx.raw").exists())
                self.assertFalse((root / "v34_tx.raw").exists())
            baseline = result.stdout
            # Reject RX metadata before touching its old raw recording. TX is
            # still usable, and engine output must not depend on this failure.
            (root / "v34_rx.raw").write_bytes(b"old")
            (root / "v34_rx.raw.status.json").mkdir()
            result = self.peer(root, root)
            self.assertEqual(result.stdout, baseline)
            self.assertIn(b"Cannot open training PCM capture", result.stderr)
            self.assertEqual((root / "v34_rx.raw").read_bytes(), b"old")
            self.capture(root / "v34_tx.raw", b"")

    def test_active_engine_keeps_its_capture_files(self):
        with tempfile.TemporaryDirectory(prefix="v90-pcm-") as tmp:
            root = Path(tmp)
            env = self.env(root)
            env.update(ME_MODE="clear", ME_DUMP_DIR=tmp, ENGINE_PEER_QUIET="1")
            first = subprocess.Popen([str(ROOT / "v90_engine_peer"), str(root / "held-pty")],
                                     env=env, cwd=ROOT, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                deadline = time.monotonic() + 5
                while not (root / "held-pty").exists():
                    self.assertIsNone(first.poll(), "first peer exited during setup")
                    self.assertLess(time.monotonic(), deadline, "first peer setup timed out")
                    time.sleep(0.01)
                second = self.peer(root, root)
                self.assertIn(b"Cannot open training PCM capture", second.stderr)
                for direction in ("rx", "tx"):
                    path = root / ("v34_" + direction + ".raw.status.json")
                    self.assertEqual(json.loads(path.read_text())["state"], "recording")
                output, error = first.communicate(struct.pack("<H", 160) + b"\xff" * 160, timeout=15)
                self.assertEqual(first.returncode, 0, error[-12000:])
                self.assertEqual(output, second.stdout)
                self.capture(root / "v34_rx.raw", b"")
                self.capture(root / "v34_tx.raw", b"")
            finally:
                if first.poll() is None:
                    first.kill()
                    first.communicate(timeout=5)

    def check_marks(self, directory, text):
        for direction in ("RX", "TX"):
            path = directory / ("v34_" + direction.lower() + ".raw")
            raw = self.capture(path)
            self.assertGreater(len(raw), 1600)
            self.assertEqual(len(raw) % 2, 0)
            marks = [int(x) for x in re.findall(r"\[ME\] " + direction + r" dump mark: sample=(\d+)", text)]
            self.assertTrue(marks, direction)
            self.assertTrue(all(b - a >= 800 for a, b in zip([0] + marks, marks)))
            self.assertLessEqual(marks[-1], len(raw) // 2)
            self.assertLess(len(raw) // 2 - marks[-1], 800)

    def test_v34_payload_and_training_replay_in_both_laws(self):
        for alaw in (False, True):
            with self.subTest(alaw=alaw), tempfile.TemporaryDirectory(prefix="v90-pcm-") as tmp:
                root = Path(tmp)
                env = self.env(root)
                env.update(ENGINE_PEER_QUIET="0", ENGINE_PAIR_KEEP_LOGS="1")
                cmd = [str(ROOT / "engine_pair_test")] + (["--alaw"] if alaw else [])
                cmd += ["--both-at", "AT+MS=V34", "--expect", "V34", "--payload-printable", "PCM",
                        "--payload-reps", "16", "--hold-seconds", "2", "--seconds", "40"]
                for role, flag in (("call", "--call-env"), ("answer", "--answer-env")):
                    directory = root / role
                    directory.mkdir()
                    for setting in (f"ME_DUMP_DIR={directory}", f"VPCM_G711_TAP_DIR={directory}",
                                    f"ME_IO_SCHEDULE={directory}/io.bin"):
                        cmd.extend([flag, setting])
                result = subprocess.run(cmd, env=env, cwd=ROOT, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True, timeout=90)
                self.assertEqual(result.returncode, 0, result.stdout[-12000:])
                logs = re.search(r"  logs: (\S+) (\S+)", result.stdout)
                self.assertIsNotNone(logs, result.stdout)
                # The existing driver keeps logs outside TMPDIR. Read only the
                # paths returned by this invocation, then remove our own logs.
                for role, path in zip(("call", "answer"), logs.groups()):
                    log = Path(path)
                    try:
                        self.check_marks(root / role, log.read_text(errors="replace"))
                    finally:
                        log.unlink()
                # Limit replay comparison to the first 10 s: no DTE payload
                # or data-link wall-clock retransmission timers in this window.
                source = root / "answer"
                schedule = bytearray()
                received = 0
                for kind, count, ns in RECORD.iter_unpack((source / "io.bin").read_bytes()):
                    if kind == b"R":
                        if received + count > 80000:
                            break
                        received += count
                    schedule.extend(RECORD.pack(kind, count, ns))
                tap = root / "input.g711"
                tap.write_bytes((source / "live-rx.g711").read_bytes()[:received])
                timing = root / "input.bin"
                timing.write_bytes(schedule)
                outputs = []
                for enabled in (False, True):
                    directory = root / ("enabled" if enabled else "disabled")
                    directory.mkdir()
                    replay_env = self.env(directory)
                    replay_env.update(ME_DUMP_DIR=str(directory) if enabled else "",
                                      VPCM_G711_TAP_DIR=str(directory))
                    cmd = [str(ROOT / "v90_engine_replay"), str(tap), "alaw" if alaw else "ulaw",
                           "--fast", "--sync", "--from", "0", "--schedule", str(timing),
                           "--at", "AT+MS=V34", "--pty-link", str(directory / "pty")]
                    replay = subprocess.run(cmd, env=replay_env, cwd=ROOT, stdout=subprocess.PIPE,
                                            stderr=subprocess.PIPE, timeout=30, check=True)
                    self.capture(directory / "live-rx.g711", tap.read_bytes())
                    outputs.append(self.capture(directory / "live-tx.g711"))
                    if enabled:
                        self.check_marks(directory, (replay.stdout + replay.stderr).decode(errors="replace"))
                    else:
                        self.assertFalse((directory / "v34_rx.raw").exists())
                        self.assertFalse((directory / "v34_tx.raw").exists())
                        for message in (b"RX PCM dump:", b"TX PCM dump:", b"RX dump mark:", b"TX dump mark:"):
                            self.assertNotIn(message, replay.stdout + replay.stderr)
                self.assertEqual(len(outputs[0]), len(outputs[1]))
                self.assertEqual(hashlib.sha256(outputs[0]).digest(), hashlib.sha256(outputs[1]).digest(),
                                 "PCM capture changed replayed TX codewords")


if __name__ == "__main__":
    unittest.main()
