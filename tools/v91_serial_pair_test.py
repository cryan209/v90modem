#!/usr/bin/env python3
"""Launch two sip_v90_modem instances and verify bidirectional PTY payload."""

from __future__ import annotations

import argparse
import os
import pathlib
import select
import signal
import subprocess
import sys
import tempfile
import time
import tty


def wait_path(path: pathlib.Path, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if path.exists():
            return
        time.sleep(0.05)
    raise RuntimeError(f"PTY link did not appear: {path}")


def open_raw(path: pathlib.Path) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    return fd


def drain(fds: list[int], seconds: float = 0.5) -> dict[int, bytes]:
    output = {fd: b"" for fd in fds}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        ready, _, _ = select.select(fds, [], [], 0.05)
        for fd in ready:
            try:
                output[fd] += os.read(fd, 65536)
            except BlockingIOError:
                pass
    return output


def wait_connect(fds: list[int], timeout: float) -> dict[int, bytes]:
    output = {fd: b"" for fd in fds}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ready, _, _ = select.select(fds, [], [], 0.2)
        for fd in ready:
            try:
                output[fd] += os.read(fd, 65536)
            except BlockingIOError:
                pass
        if all(b"CONNECT" in output[fd] for fd in fds):
            return output
        if any(b"NO CARRIER" in output[fd] for fd in fds):
            break
    raise RuntimeError(f"V.91 connection did not reach CONNECT: {output!r}")


def expect_payload(src: int, dst: int, payload: bytes, timeout: float) -> bytes:
    os.write(src, payload)
    received = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ready, _, _ = select.select([dst], [], [], 0.2)
        if ready:
            try:
                received += os.read(dst, 65536)
            except BlockingIOError:
                pass
            if payload in received:
                return received
    raise RuntimeError(
        f"payload not received (expected={payload!r}, received={received!r})"
    )


def terminate(proc: subprocess.Popen[bytes]) -> None:
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.terminate()
        proc.wait(timeout=5)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", default="./sip_v90_modem")
    parser.add_argument("--caller-port", type=int, default=5091)
    parser.add_argument("--answerer-port", type=int, default=5092)
    parser.add_argument("--timeout", type=float, default=25.0)
    parser.add_argument(
        "--robbed-phase", type=int, default=None,
        help="simulate a robbed-bit trunk on both receive paths at this cadence phase (0-5)")
    args = parser.parse_args()

    child_env = dict(os.environ)
    label = "bidirectional PTY payload over SIP/RTP"
    if args.robbed_phase is not None:
        child_env["V91_SIM_ROBBED_RX"] = str(args.robbed_phase)
        label += f", simulated robbed-bit trunk phase={args.robbed_phase}"

    binary = str(pathlib.Path(args.binary).resolve())
    with tempfile.TemporaryDirectory(prefix="v91-serial-pair-") as temp_dir:
        temp = pathlib.Path(temp_dir)
        caller_pty = temp / "caller-pty"
        answerer_pty = temp / "answerer-pty"
        caller_log_path = temp / "caller.log"
        answerer_log_path = temp / "answerer.log"
        caller_log = caller_log_path.open("wb")
        answerer_log = answerer_log_path.open("wb")
        answerer = subprocess.Popen(
            [binary, "--local-port", str(args.answerer_port),
             "--pty-link", str(answerer_pty), "--verbose"],
            stdout=answerer_log,
            stderr=subprocess.STDOUT,
            env=child_env,
        )
        caller = subprocess.Popen(
            [binary, "--sip-server", f"127.0.0.1:{args.answerer_port}",
             "--local-port", str(args.caller_port),
             "--pty-link", str(caller_pty), "--verbose"],
            stdout=caller_log,
            stderr=subprocess.STDOUT,
            env=child_env,
        )
        caller_fd = -1
        answerer_fd = -1
        try:
            wait_path(caller_pty, 8)
            wait_path(answerer_pty, 8)
            caller_fd = open_raw(caller_pty)
            answerer_fd = open_raw(answerer_pty)
            drain([caller_fd, answerer_fd])
            os.write(caller_fd, f"ATD{args.answerer_port}\r".encode("ascii"))
            wait_connect([caller_fd, answerer_fd], args.timeout)
            drain([caller_fd, answerer_fd])

            c2a = b"V91-CALLER-TO-ANSWERER-0123456789\r\n"
            a2c = b"V91-ANSWERER-TO-CALLER-9876543210\r\n"
            expect_payload(caller_fd, answerer_fd, c2a, 5)
            expect_payload(answerer_fd, caller_fd, a2c, 5)
            print(f"v91_serial_pair_test: OK ({label})")
            return 0
        except Exception as exc:
            print(f"v91_serial_pair_test: FAIL: {exc}", file=sys.stderr)
            terminate(caller)
            terminate(answerer)
            caller_log.close()
            answerer_log.close()
            print("--- caller log ---", file=sys.stderr)
            print(caller_log_path.read_text(errors="replace"), file=sys.stderr)
            print("--- answerer log ---", file=sys.stderr)
            print(answerer_log_path.read_text(errors="replace"), file=sys.stderr)
            return 1
        finally:
            if caller_fd >= 0:
                os.close(caller_fd)
            if answerer_fd >= 0:
                os.close(answerer_fd)
            terminate(caller)
            terminate(answerer)
            if not caller_log.closed:
                caller_log.close()
            if not answerer_log.closed:
                answerer_log.close()


if __name__ == "__main__":
    raise SystemExit(main())
