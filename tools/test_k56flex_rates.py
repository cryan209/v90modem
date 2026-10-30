#!/usr/bin/env python3
"""Qualify every K56flex downstream rate, both laws and both framing modes.

Draft 0.23 clause 4.12 / V.250 6.4.1: exact downstream bounds, AT-only
negotiation, and independent full-octet binary payloads in both directions.
Runs through run_tests.py so each engine pair has its own scratch directory.
--peer native-hsf instead tests the independent native receiver at all
32000..56000 rates (mu-law/V.14), plus 58/60k server-ceiling fallback checks.
This is closed-loop qualification, not a hardware or noise-margin test.
"""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent


def commands(payload_bytes, seconds):
    for rate in range(32000, 60001, 2000):
        for alaw in (False, True):
            for v14 in (False, True):
                argv = ["./engine_pair_test"]
                if alaw:
                    argv += ["--alaw"]
                if v14:
                    argv += ["--both-at", "AT+ES=1"]
                argv += [
                    "--call-at", f"AT+MS=K56,1,0,31200,{rate},{rate}",
                    "--answer-at", f"AT+MS=K56,1,{rate},{rate},0,31200",
                    "--both-at", "ATS2=255",
                    "--expect-connect", str(rate),
                    "--both-expect", f"CONNECT {rate}",
                    "--payload-random-bytes", str(payload_bytes),
                    "--seconds", str(seconds),
                ]
                yield shlex.join(argv)


def native_hsf_commands(payload_bytes, seconds):
    base = ["python3", "tools/k56flex_hsf_pair.py",
            "--max-upstream-rate", "28800", "--seconds", str(seconds),
            "--payload-bytes", str(payload_bytes), "--require-payload",
            "--require-audio-gates"]
    for rate in range(32000, 56001, 2000):
        yield shlex.join(base + ["--downstream-rate", str(rate)]) + ' --output "$TMPDIR/hsf"'
    # Do not force unsupported native RX configurations: its wrapper's
    # maximum is 56000. Unconstrained HSF selects 52000 on this path.
    for ceiling in (58000, 60000):
        yield (f"ME_K56FLEX_RATE={ceiling} "
               + shlex.join(base + ["--expect-downstream-rate", "52000"])
               + ' --output "$TMPDIR/hsf"')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-j", "--jobs", type=int, default=4)
    parser.add_argument("--peer", choices=("engine", "native-hsf"), default="engine")
    parser.add_argument("--payload-bytes", type=int)
    parser.add_argument("--seconds", type=int)
    parser.add_argument("--logdir", type=Path)
    args = parser.parse_args()
    native = args.peer == "native-hsf"
    if args.payload_bytes is None:
        args.payload_bytes = 8192 if native else 32768
    if args.seconds is None:
        args.seconds = 40 if native else 90
    if native and args.seconds > 120:
        parser.error("native-hsf seconds must be 1..120")
    if args.logdir is None:
        args.logdir = (Path(os.environ.get("ME_DUMP_DIR", ROOT / "test-logs"))
                       / ("k56flex-hsf-rates" if native else "k56flex-rates"))
    if args.jobs < 1 or not 1 <= args.payload_bytes <= 65536 or args.seconds < 1:
        parser.error("jobs/seconds must be positive; payload-bytes must be 1..65536")
    # The normal runner deletes its log directory before a run. Keep that
    # operation confined to a dedicated matrix directory, never a caller's root.
    logdir = args.logdir.resolve()
    if logdir == ROOT or logdir == ROOT / "test-logs" or logdir == Path("/"):
        parser.error("logdir must be a dedicated matrix directory")
    generator = native_hsf_commands if native else commands
    rows = list(generator(args.payload_bytes, args.seconds))
    coverage = ("native HSF, 32000..56000 plus 58/60k ceiling fallbacks, mu-law/V.14"
                if native else "32000..60000, A/mu law, LAPM/V.14")
    print(f"K56flex: {len(rows)} pairs, {coverage}, "
          f"{args.payload_bytes} random bytes each way", flush=True)
    with tempfile.TemporaryDirectory(prefix="k56flex-rate-list-") as scratch:
        path = Path(scratch) / "k56flex-rates.list"
        path.write_text("\n".join(rows) + "\n")
        return subprocess.call([
            sys.executable, str(ROOT / "tools/run_tests.py"),
            "-j", str(args.jobs), "--logdir", str(logdir), str(path),
        ], cwd=ROOT)


if __name__ == "__main__":
    sys.exit(main())
