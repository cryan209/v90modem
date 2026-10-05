#!/usr/bin/env python3
"""Run reproducible synthetic-line V.34 BER measurements; retain every failure."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from itertools import product
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cases", default="2400:9600,3200:21600", help="baud:rate pairs")
    p.add_argument("--laws", default="ulaw,alaw")
    p.add_argument("--snr", default="off,40,30,24", help="comma-separated dB values or off")
    p.add_argument("--delays", default="0,80", help="one-way samples at 8 kHz")
    p.add_argument("--channels", default="off", help="off or AD:EDD pairs; AD=1,5,6,7,8,9; EDD=1,2,3")
    p.add_argument("--seeds", default="1")
    p.add_argument("--bits", type=int, default=1_000_000)
    p.add_argument("--seconds", type=int, default=240, help="simulated deadline")
    p.add_argument("--loss-db", type=float, default=0)
    p.add_argument("--echo-db", type=float)
    p.add_argument("--echo-delay", type=int, default=2136)
    p.add_argument("--timeout", type=float, default=120, help="wall seconds per child")
    p.add_argument("--output", type=Path)
    p.add_argument("--require-clean", action="store_true", help="exit 1 if any line test fails")
    args = p.parse_args()
    try:
        cases = [tuple(map(int, c.split(":"))) for c in args.cases.split(",")]
        if not all(len(c) == 2 for c in cases):
            raise ValueError("expected baud:rate pairs")
        laws = args.laws.split(",")
        snrs = [None if s == "off" else float(s) for s in args.snr.split(",")]
        delays = [int(d) for d in args.delays.split(",")]
        channels = [(0,0) if c == "off" else tuple(map(int,c.split(":"))) for c in args.channels.split(",")]
        seeds = [int(s) for s in args.seeds.split(",")]
        import math
        valid = [all(l in ("ulaw", "alaw") for l in laws),
                 all(c[0] in (2400,2743,2800,3000,3200,3429) and 2400 <= c[1] <= 33600 and c[1] % 2400 == 0 for c in cases),
                 all(s is None or math.isfinite(s) and 0 <= s <= 100 for s in snrs),
                 all(0 <= d < 8192 for d in delays),
                 all(len(c)==2 and (c==(0,0) or c[0] in (1,5,6,7,8,9) and c[1] in (1,2,3)) for c in channels),
                 all(d <= 7679 for d in delays) if any(c!=(0,0) for c in channels) else True,
                 all(1 <= s <= 1_000_000 for s in seeds),
                 1 <= args.bits <= 100_000_000 and 1 <= args.seconds <= 86400,
                 math.isfinite(args.loss_db) and 0 <= args.loss_db <= 60,
                 args.echo_db is None or math.isfinite(args.echo_db) and 0 <= args.echo_db <= 120,
                 0 <= args.echo_delay <= 6000 and math.isfinite(args.timeout) and args.timeout > 0]
        if not all(valid):
            raise ValueError("unsupported sweep value")
    except ValueError:
        p.error("invalid sweep values")
    binary = ROOT / "v56_loopback_test"
    if not binary.is_file():
        p.error("build first with make v56_loopback_test")
    out = args.output or ROOT / "artifacts/v56" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    out.mkdir(parents=True, exist_ok=False)
    # The DSP has diagnostic ME_/V34_ switches. A sweep must not unknowingly
    # inherit a previous experiment and call it the declared line profile.
    env = {k: v for k, v in os.environ.items() if not k.startswith(("ME_", "V34_"))}
    counts: Counter[str] = Counter()
    infrastructure_failed = False
    with (out / "results.jsonl").open("w") as results:
        for (baud, rate), law, snr, delay, seed, (ad, edd) in product(cases,laws,snrs,delays,seeds,channels):
            cmd = [str(binary), "--json", "--baud", str(baud), "--rate", str(rate),
                   "--law", law, "--bits", str(args.bits), "--seconds", str(args.seconds),
                   "--loss-db", str(args.loss_db), "--delay", str(delay), "--seed", str(seed)]
            if ad:
                cmd += ["--ad", str(ad), "--edd", str(edd)]
            if snr is not None:
                cmd += ["--snr-db", str(snr)]
            if args.echo_db is not None:
                cmd += ["--echo-db", str(args.echo_db), "--echo-delay", str(args.echo_delay)]
            name = f"{sum(counts.values()):04d}"
            record = {"command": cmd, "baud": baud, "requested_bps": rate, "law": law,
                      "snr_db": snr, "delay_samples": delay, "seed": seed, "ad": ad, "edd": edd}
            try:
                proc = subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True,
                                      text=True, timeout=args.timeout)
                (out / f"{name}.log").write_text(proc.stderr)
                (out / f"{name}.stdout").write_text(proc.stdout)
                measured = json.loads(proc.stdout)
                expected_exit = 0 if measured["status"] == "pass" else 1
                if measured["status"] not in ("pass", "errors", "timeout", "carrier_lost", "rate_mismatch") or proc.returncode != expected_exit:
                    raise ValueError("unexpected child status/exit")
                record.update(measured)
                for direction in ("ab", "ba"):
                    n = record[f"{direction}_bits"]
                    record[f"{direction}_ber"] = record[f"{direction}_errors"] / n if n else None
                record["exit_code"] = proc.returncode
            except subprocess.TimeoutExpired as exc:
                record.update(status="wall_timeout", wall_timeout_s=args.timeout)
                (out / f"{name}.log").write_bytes(exc.stderr or b"")
                (out / f"{name}.stdout").write_bytes(exc.stdout or b"")
                infrastructure_failed = True
            except (ValueError, KeyError, TypeError, OSError) as exc:
                record.update(status="harness_error", error=str(exc))
                infrastructure_failed = True
            counts[record["status"]] += 1
            results.write(json.dumps(record, allow_nan=False) + "\n")
            results.flush()
            print(f"{baud}/{rate}/{law} ad={ad} edd={edd} snr={snr} delay={delay} seed={seed}: "
                  f"{record['status']} A->B {record.get('ab_errors', '-')}/{record.get('ab_bits', '-')} "
                  f"B->A {record.get('ba_errors', '-')}/{record.get('ba_bits', '-')}", flush=True)
    print(f"Results: {out / 'results.jsonl'}; {dict(counts)}")
    if infrastructure_failed:
        return 2
    return int(args.require_clean and counts["pass"] != sum(counts.values()))


if __name__ == "__main__":
    raise SystemExit(main())
