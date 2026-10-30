#!/usr/bin/env python3
"""Measure native HSF K56flex RX impairment limits at 44/52/56k.

Impairments are post-DAC analogue samples, not modified G.711 codewords.
Expected receiver failures in a characterization sweep are reported as FAIL,
not silently accepted as successful calls. Select qualified profiles for
regression use. Each pair has independent scratch space via run_tests.py.
"""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent
# Supplemental fine-noise and delay profiles are opt-in. The default sweep
# deliberately includes failures to characterize a boundary, not pass a CI tier.
DEFAULT_PROFILES = ["clean", "noise-80", "noise-70", "noise-60", "noise-50", "noise-40",
                    "gain-3", "gain-6", "clock-10", "clock+10"]
PROFILES = {
    "clean": [],
    **{f"noise{db}": ["--rx-noise-dbfs", str(db)] for db in (-90, -85, -80, -70, -65, -60, -50, -40)},
    **{f"gain{db}": ["--rx-gain-db", str(db)] for db in (-3, -6)},
    **{f"clock{ppm:+d}": ["--rx-clock-ppm", str(ppm)] for ppm in (-10, 10)},
    "delay4ms": ["--rx-delay-samples", "64"],
    "delay16ms": ["--rx-delay-samples", "256"],
}


def cases(profiles, seconds, payload_bytes):
    for rate in (44000, 52000, 56000):
        for profile in profiles:
            argv = ["python3", "tools/k56flex_hsf_pair.py",
                    "--downstream-rate", str(rate), "--max-upstream-rate", "28800",
                    "--seconds", str(seconds), "--payload-bytes", str(payload_bytes),
                    "--require-audio-gates", "--require-payload",
                    "--rx-delay-samples", "32"] + PROFILES[profile]
            yield rate, profile, shlex.join(argv) + ' --output "$TMPDIR/hsf"'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", action="append", choices=PROFILES)
    parser.add_argument("-j", "--jobs", type=int, default=3)
    parser.add_argument("--seconds", type=int, default=40)
    parser.add_argument("--payload-bytes", type=int, default=8192)
    parser.add_argument("--logdir", type=Path, default=Path(os.environ.get("ME_DUMP_DIR", ROOT / "test-logs")) / "k56flex-hsf-impairments")
    args = parser.parse_args()
    if args.jobs < 1 or not 1 <= args.seconds <= 120 or not 1 <= args.payload_bytes <= 65536:
        parser.error("positive jobs, seconds 1..120, payload-bytes 1..65536 required")
    profiles = args.profile or DEFAULT_PROFILES
    rows = list(cases(profiles, args.seconds, args.payload_bytes))
    out = args.logdir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    logs = out / "pairs"
    print(f"Native HSF: {len(rows)} impairment cases, 32-sample matched analogue delay, "
          f"{args.payload_bytes} random bytes each way", flush=True)
    with tempfile.TemporaryDirectory(prefix="k56flex-impairment-list-") as scratch:
        path = Path(scratch) / "k56flex-hsf-impairments.list"
        path.write_text("\n".join(row[2] for row in rows) + "\n")
        status = subprocess.call([sys.executable, str(ROOT / "tools/run_tests.py"),
                                  "-j", str(args.jobs), "--logdir", str(logs), str(path)], cwd=ROOT)
    statuses = [line.split("\t", 3)[1] for line in (logs / "timings.tsv").read_text().splitlines()]
    summary = []
    for index, (rate, profile, command) in enumerate(rows):
        text = (logs / f"{index:04d}.log").read_text(errors="replace")
        report = None
        start = text.find("{\n")
        if start >= 0:
            try:
                report, _ = json.JSONDecoder().raw_decode(text[start:])
            except ValueError:
                pass
        passed = bool(statuses[index] == "PASS" and report and report.get("verified_connection")
                      and report.get("downstream_rate_exact"))
        summary.append(dict(rate=rate, profile=profile, passed=passed, status=statuses[index], report=report,
                            command=command, log=str(logs / f"{index:04d}.log")))
        actual = report.get("client_rates") if report else None
        reason = report.get("end_reason") if report else "no report (harness failure)"
        print(f"{rate} {profile:>9}: {'PASS' if passed else 'FAIL'} actual={actual} {reason}")
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return status or int(any(not row["passed"] for row in summary))


if __name__ == "__main__":
    sys.exit(main())
