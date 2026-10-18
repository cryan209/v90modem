#!/usr/bin/env python3
"""Hardware-free ANSam reference ownership and explicit ppm override checks."""
import array
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="apple-clock-reference-") as scratch:
        tone = Path(scratch) / "answer-tone.s16"
        # Known +200 ppm input clock. V.90 5.2's network timing must not be
        # inferred from our own reflected ANSam when this frontend answers.
        rate = 9600
        samples = array.array("h", (round(5000 * math.sin(
            2 * math.pi * (2100 / 1.0002) * n / rate)) for n in range(6 * rate)))
        if sys.byteorder != "little":
            samples.byteswap()
        tone.write_bytes(samples.tobytes())
        base = dict(os.environ, ME_MODE="v90", ME_V90_ROLE="digital",
                    ME_DUMP_DIR=scratch, APPLE_MODEM_EC="0")
        for key in ("APPLE_MODEM_CLOCK_PPM", "APPLE_MODEM_LOOP_TIMING"):
            base.pop(key, None)
        cases = [("caller", [], {}), ("answer", ["--answer-now"], {}),
                 ("forced", ["--answer-now"], {"APPLE_MODEM_CLOCK_PPM": "-34"})]
        for name, flags, overrides in cases:
            result = subprocess.run(
                [str(root / "apple_usb_modem_coupler"), *flags, "--rate", str(rate),
                 "--rx-replay", str(tone)], cwd=root, env=dict(base, **overrides),
                capture_output=True, text=True, timeout=15, check=True)
            corrections = re.findall(r"loop timing: codec clock ([+-][\d.]+) ppm",
                                     result.stderr)
            if name == "caller":
                assert len(corrections) == 1 and abs(float(corrections[0]) - 200) < 1
            elif name == "answer":
                assert not corrections and "answerer has no remote ANSam reference" in result.stderr
            else:
                assert corrections == ["-34.0"]
    print("apple_clock_reference_test: caller reference, answer echo rejection, forced ppm OK")


if __name__ == "__main__":
    main()
