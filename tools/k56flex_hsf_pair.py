#!/usr/bin/env python3
"""Bounded whole-engine K56flex call against cruft's native legacy HSF client.

Direct byte-exact PCMU socket, no SIP/PBX or forced gates. The peer uses its
normal Flex-only V.8bis -> V.8 -> K56 startup. This probe reports carrier
observations separately from verified payload (which it does not qualify).
"""
import argparse
import errno
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import tty

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def drain(fd, output):
    while True:
        try:
            data = os.read(fd, 4096)
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EIO):
                return
            raise
        if not data:
            return
        output.extend(data)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cruft", type=Path, default=ROOT.parent / "cruft")
    ap.add_argument("--seconds", type=int, default=35)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    if not 1 <= args.seconds <= 120:
        ap.error("--seconds must be 1..120")
    cruft = args.cruft.resolve()
    sys.path.insert(0, str(cruft))
    from native.hsf.k56_client import K56Client, PCMUStream
    from native.hsf.pcm import LIBRARY

    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    executable = ROOT / "audio_sock_modem"
    env = dict(os.environ, ME_K56FLEX="probe", ME_MODE="k56", ME_V8="1",
               ME_MEDIA_CLOCK="1", VPCM_G711_TAP_DIR=str(out))
    client = K56Client(modulation="flex", max_upstream_rate=9600)
    stream = PCMUStream(client)
    upstream, downstream, dte = bytearray(), bytearray(), bytearray()
    fd, frames, end_reason = None, 0, "duration limit"
    start_wall = time.monotonic()
    final_state = None
    proc = None
    try:
        # Keep socket paths short enough for macOS sockaddr_un. All paths
        # are unique; there are no fixed scratch-file names across runs.
        with tempfile.TemporaryDirectory(prefix="k56hsf-") as scratch:
            sock_path = Path(scratch) / "audio"
            pty_path = Path(scratch) / "dte"
            with (out / "server.log").open("wb") as log:
                proc = subprocess.Popen(
                    [str(executable), "--listen", str(sock_path),
                     "--pty-link", str(pty_path), "--verbose"],
                    cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 15
                    while not (sock_path.exists() and pty_path.exists()):
                        if proc.poll() is not None:
                            raise RuntimeError("engine exited during startup; see server.log")
                        if time.monotonic() > deadline:
                            raise RuntimeError("engine socket/PTY startup timed out")
                        time.sleep(0.02)
                    fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                    tty.setraw(fd)
                    os.write(fd, b"ATD1\r")
                    # Let the DTE thread accept ATD before connecting. The
                    # socket log is checked below: incoming/answer role is
                    # not a valid substitute for the intended digital caller.
                    time.sleep(0.2)
                    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                        sock.settimeout(15)
                        sock.connect(str(sock_path))
                        tx = b"\xff" * 160
                        for _ in range(args.seconds * 50):
                            if time.monotonic() - start_wall > 180:
                                raise RuntimeError("wall-clock limit")
                            try:
                                sock.sendall(tx)
                                incoming = bytearray()
                                while len(incoming) < 160:
                                    data = sock.recv(160 - len(incoming))
                                    if not data:
                                        break
                                    incoming.extend(data)
                            except (BrokenPipeError, ConnectionResetError):
                                end_reason = "engine closed audio socket"
                                break
                            if len(incoming) != 160:
                                end_reason = "engine closed audio socket"
                                break
                            upstream.extend(tx)
                            downstream.extend(incoming)
                            tx = stream.exchange(bytes(incoming))
                            frames += 1
                            drain(fd, dte)
                            if client.failure:
                                end_reason = client.failure
                                break
                    drain(fd, dte)
                finally:
                    if fd is not None:
                        os.close(fd)
                        fd = None
                    proc.terminate()
                    try:
                        proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
        final_state = client.pump.state() if client.pump else None
        (out / "client-tx.g711").write_bytes(upstream)
        (out / "server-tx.g711").write_bytes(downstream)
        (out / "dte.bin").write_bytes(dte)
        (out / "client-events.json").write_text(json.dumps(client.events, indent=2) + "\n")
        server_log = (out / "server.log").read_text(errors="replace")
        if "outgoing (completes ATD)" not in server_log:
            raise RuntimeError("wrong engine role; see server.log")
        notices = [notice for event in client.events
                   for notice in event.get("notifications", [])]
        report = {
            "transport": "lockstep 8 kHz PCMU, whole audio_sock_modem engine",
            "peer": "cruft native legacy HSF, Flex-only (no V.90/V.34 fallback)",
            "audio_seconds": frames / 50, "end_reason": end_reason,
            "wall_seconds": time.monotonic() - start_wall,
            "flex_agreed": client.flex_agreed,
            "v8_started": client.v8 is not None,
            "v8_modulations": client.v8_modulations,
            "selected_modulation": client.selected_modulation,
            "final_client_state": final_state, "failure": client.failure,
            "client_tx_training_completed": any(n[0] == 1000 for n in notices),
            "client_rx_training_completed": any(n[0] == 1001 for n in notices),
            "server_dte_connect": b"CONNECT" in dte,
            "verified_connection": False,
            "limits": "Carrier diagnostics only; no checked duplex payload qualification.",
            "inputs": {str(executable): digest(executable), str(LIBRARY): digest(LIBRARY),
                       str(Path(__file__).resolve()): digest(Path(__file__))},
        }
        (out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
    finally:
        client.close()
        if proc is not None and proc.poll() is None:
            proc.kill()
            proc.wait()


if __name__ == "__main__":
    main()
