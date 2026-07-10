#!/usr/bin/env python3
"""Run one bounded real-modem V.90 attempt and preserve reproducible evidence."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import platform
import signal
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def sanitize_args(args: list[str]) -> list[str]:
    sanitized: list[str] = []
    redact_next = False
    for arg in args:
        if redact_next:
            sanitized.append("<redacted>")
            redact_next = False
        elif arg == "--password":
            sanitized.append(arg)
            redact_next = True
        elif arg.startswith("--password="):
            sanitized.append("--password=<redacted>")
        else:
            sanitized.append(arg)
    return sanitized


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_head() -> str | None:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture a bounded V.90 hardware-interoperability attempt"
    )
    parser.add_argument("--label", required=True, help="modem/gateway label")
    parser.add_argument("--duration", type=int, default=180, help="run seconds")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / "artifacts" / "v90-hardware",
    )
    parser.add_argument("--binary", type=Path, default=ROOT / "sip_v90_modem")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "modem_args",
        nargs=argparse.REMAINDER,
        help="arguments for sip_v90_modem; place them after --",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    modem_args = args.modem_args[1:] if args.modem_args[:1] == ["--"] else args.modem_args
    if args.duration < 1:
        raise SystemExit("--duration must be at least one second")
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    safe_label = "".join(c if c.isalnum() or c in "-_" else "_" for c in args.label)
    output_dir = args.output_root / f"{stamp}-{safe_label}"
    command = [str(args.binary), *modem_args]

    print(f"evidence_dir={output_dir}")
    print("command=" + " ".join(sanitize_args(command)))
    if args.dry_run:
        return 0
    if not args.binary.is_file():
        raise SystemExit(f"modem binary not found: {args.binary}; run make first")

    output_dir.mkdir(parents=True, exist_ok=False)
    log_path = output_dir / "modem.log"
    environment = os.environ.copy()
    environment["VPCM_G711_TAP_DIR"] = str(output_dir)

    with log_path.open("wb") as log_stream:
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=environment,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
        )
        timed_out = False
        try:
            return_code = process.wait(timeout=args.duration)
        except subprocess.TimeoutExpired:
            timed_out = True
            process.send_signal(signal.SIGINT)
            try:
                return_code = process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    return_code = process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    return_code = process.wait(timeout=5)

    summary_path = output_dir / "summary.json"
    summary_process = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "summarize_modem_trace.py"), "--json", str(log_path)],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if summary_process.returncode == 0:
        summary_path.write_text(summary_process.stdout)

    files: dict[str, dict[str, object]] = {}
    for path in sorted(output_dir.iterdir()):
        if path.is_file() and path.name != "manifest.json":
            files[path.name] = {"bytes": path.stat().st_size, "sha256": sha256(path)}
    manifest = {
        "schema": 1,
        "label": args.label,
        "started_utc": stamp,
        "duration_limit_seconds": args.duration,
        "stopped_at_duration_limit": timed_out,
        "process_return_code": return_code,
        "command": sanitize_args(command),
        "git_head": git_head(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "files": files,
    }
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"manifest={output_dir / 'manifest.json'}")
    print(f"summary={summary_path if summary_path.exists() else 'unavailable'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
