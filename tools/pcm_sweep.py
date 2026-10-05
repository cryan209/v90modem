#!/usr/bin/env python3
"""Retain directed V.90 downstream/V.92 PCM upstream datapump measurements."""
from __future__ import annotations
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
from itertools import product
import json
import math
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def csv_ints(value: str, low: int, high: int) -> list[int]:
    values = [int(v) for v in value.split(",")]
    if not values or any(v < low or v > high for v in values):
        raise ValueError(value)
    return values


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--modes", default="v90-downstream,v92-upstream")
    p.add_argument("--laws", default="ulaw,alaw")
    p.add_argument("--v90-drns", default="1,9,22")
    p.add_argument("--v92-drns", default="1,9,19")
    p.add_argument("--shaping", default="0,1,2,3")
    p.add_argument("--seeds", default="1")
    p.add_argument("--chunks", default="37", help="RX callback sizes, 1..160 samples")
    p.add_argument("--delays", default="0", help="leading sample delay, 0..8192")
    p.add_argument("--noise-rms", default="0", help="V.92 only: linear units at simulated network A/D")
    p.add_argument("--bits", type=int, default=16000)
    p.add_argument("--timeout", type=float, default=120)
    p.add_argument("--all-rates", action="store_true")
    p.add_argument("--require-clean", action="store_true")
    p.add_argument("--output", type=Path)
    args = p.parse_args()
    try:
        modes = args.modes.split(",")
        laws = args.laws.split(",")
        if any(m not in ("v90-downstream", "v92-upstream") for m in modes):
            raise ValueError("mode")
        if any(l not in ("ulaw", "alaw") for l in laws):
            raise ValueError("law")
        rates = {"v90-downstream": list(range(1, 23)) if args.all_rates else csv_ints(args.v90_drns, 1, 22),
                 "v92-upstream": list(range(1, 20)) if args.all_rates else csv_ints(args.v92_drns, 1, 19)}
        shaping = csv_ints(args.shaping, 0, 3)
        seeds = csv_ints(args.seeds, 1, 1000000)
        chunks = csv_ints(args.chunks, 1, 160)
        delays = csv_ints(args.delays, 0, 8192)
        noises = [float(v) for v in args.noise_rms.split(",")]
        if any(not math.isfinite(n) or not 0 <= n <= 10000 for n in noises):
            raise ValueError("noise")
        if not 1 <= args.bits <= 10000000 or not math.isfinite(args.timeout) or args.timeout <= 0:
            raise ValueError("limit")
    except ValueError as exc:
        p.error(f"invalid matrix configuration: {exc}")
    binary = ROOT / "pcm_ber_test"
    if not binary.is_file():
        p.error("build first: make pcm_ber_test")
    out = args.output or ROOT / "artifacts/pcm" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    out.mkdir(parents=True, exist_ok=False)
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(("ME_", "V34_", "V90_", "V92_", "VPCM_"))}
    metadata = {"arguments": vars(args) | {"output": str(out)},
                "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
                "coverage": "directed components; no V.8/full V.92 call or hardware claim"}
    (out / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n")
    counts: Counter[str] = Counter()
    broken = False
    with (out / "results.jsonl").open("w") as results:
        for mode in modes:
            shapes = shaping if mode == "v90-downstream" else [0]
            noise_values = [0.0] if mode == "v90-downstream" else noises
            for law, drn, sr, seed, chunk, delay, noise in product(
                    laws, rates[mode], shapes, seeds, chunks, delays, noise_values):
                cmd = [str(binary), "--mode", mode, "--law", law, "--drn", str(drn),
                       "--sr", str(sr), "--seed", str(seed), "--chunk", str(chunk),
                       "--delay", str(delay), "--noise-rms", str(noise), "--bits", str(args.bits)]
                name = f"{sum(counts.values()):04d}"
                record = {"command": cmd, "mode": mode, "law": law, "drn": drn,
                          "sr": sr, "seed": seed, "chunk_samples": chunk, "delay_samples": delay,
                          "noise_rms": noise, "target_bits": args.bits}
                start = time.monotonic()
                try:
                    proc = subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True,
                                          timeout=args.timeout)
                    (out / f"{name}.log").write_bytes(proc.stderr)
                    (out / f"{name}.stdout").write_bytes(proc.stdout)
                    measured = json.loads(proc.stdout)
                    status = measured["status"]
                    if status not in ("pass", "errors", "no_acquisition", "short_payload"):
                        raise ValueError(f"unexpected child status: {status}")
                    if proc.returncode != (0 if status == "pass" else 1):
                        raise ValueError("child exit/status mismatch")
                    for key in ("mode", "law", "drn", "sr", "seed", "chunk_samples", "delay_samples", "noise_rms", "target_bits"):
                        if measured[key] != record[key]:
                            raise ValueError(f"child configuration mismatch: {key}")
                    if measured["full_call"] is not False or measured["rate_source"] != "configured_profile":
                        raise ValueError("unexpected coverage/rate source")
                    if type(measured["startup_complete"]) is not bool:
                        raise ValueError("invalid startup flag")
                    for key in ("rejected_frames", "b1_bit_errors", "clips"):
                        if type(measured[key]) is not int or measured[key] < 0:
                            raise ValueError(f"invalid count: {key}")
                    for key in ("rate_bps", "b1_correlation"):
                        if type(measured[key]) not in (int, float) or not math.isfinite(measured[key]):
                            raise ValueError(f"invalid measurement: {key}")
                    bits, errors = measured["checked_bits"], measured["bit_errors"]
                    if type(bits) is not int or type(errors) is not int or not 0 <= errors <= bits <= args.bits:
                        raise ValueError("invalid bit counts")
                    if status == "pass" and (bits != args.bits or errors or not measured["startup_complete"]
                                             or measured["rejected_frames"] or measured["b1_bit_errors"]):
                        raise ValueError("unsubstantiated pass")
                    record.update(measured, ber=errors / bits if bits else None, exit_code=proc.returncode)
                except subprocess.TimeoutExpired as exc:
                    (out / f"{name}.log").write_bytes(exc.stderr or b"")
                    (out / f"{name}.stdout").write_bytes(exc.stdout or b"")
                    record.update(status="wall_timeout")
                    broken = True
                except (ValueError, KeyError, TypeError, OSError) as exc:
                    record.update(status="harness_error", error=str(exc))
                    broken = True
                record["wall_seconds"] = time.monotonic() - start
                counts[record["status"]] += 1
                results.write(json.dumps(record, allow_nan=False) + "\n")
                results.flush()
                print(f"{mode}/{law}/drn={drn}/sr={sr}/seed={seed}/chunk={chunk}/delay={delay}/noise={noise}: "
                      f"{record['status']} {record.get('bit_errors', '-')}/{record.get('checked_bits', '-')}", flush=True)
    summary = {"counts": dict(counts), "rows": sum(counts.values()), "infrastructure_failed": broken}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Results: {out / 'results.jsonl'}; {dict(counts)}")
    return 2 if broken else int(args.require_clean and counts["pass"] != sum(counts.values()))


if __name__ == "__main__":
    raise SystemExit(main())
