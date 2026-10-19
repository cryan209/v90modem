#!/usr/bin/env python3
"""Engine-level timer regression, not K56flex interop evidence.

Non-idle noise cannot satisfy a peer gate. The preflight/probe must exhaust its own
bounded waits before ordinary V.8 receives a fresh timeout budget (Draft 0.23
clause 4.1: V.8bis precedes V.8, not concurrent V.8 negotiation).
"""
import os
from pathlib import Path
import re
import random
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    with tempfile.TemporaryDirectory(prefix="k56flex-timeout-") as scratch:
        directory = Path(scratch)
        tap = directory / "noise.g711"
        rng = random.Random(56)
        # Non-idle input exercises the engine's receive-side phase timer;
        # all-idle input takes an early return and cannot reproduce this bug.
        levels = (0xb0, 0x30, 0xc0, 0x40)
        tap.write_bytes(bytes(levels[rng.getrandbits(2)] for _ in range(36 * 8000)))
        env = dict(os.environ, ME_K56FLEX="probe", ME_K56FLEX_GATE_MS="8000",
                   ME_VOICE_CAPTURE_HOLD="0", ME_MODE="v90", ME_V8="1")
        run = subprocess.run(
            [str(ROOT / "v90_engine_replay"), str(tap), "ulaw", "--sync",
             "--from", "0", "--dial", "--sip-answer", "--pty-link", str(directory / "dte")],
            cwd=ROOT, env=env, capture_output=True, text=True, timeout=60)
        log = run.stdout + run.stderr
        print(log)
        if run.returncode:
            raise SystemExit(f"replay failed: {run.returncode}")
        gate = re.search(r"\[TRACE \+(\d+)ms\] K56flex probe gave up", log)
        timeout = re.search(r"\[TRACE \+(\d+)ms\] V8 timeout after (\d+)ms", log)
        if not gate or not timeout:
            raise SystemExit("missing gate timeout/fallback or fresh V.8 timeout")
        if int(gate[1]) < 16000 or int(timeout[1]) - int(gate[1]) < 15000:
            raise SystemExit("V.8 timeout stole time from K56flex probe/fallback")
        if int(timeout[2]) > 15100:
            raise SystemExit("fallback V.8 timeout was not reset")
        print("PASS: K56flex probe uses its own wait; fallback V.8 gets a fresh 15 s")


if __name__ == "__main__":
    main()
