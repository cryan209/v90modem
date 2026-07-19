#!/usr/bin/env python3
"""Drive the remote d-modem/slmodemd rig's AT interface over SSH+socat.

Bridges /dev/ttySL0 inside the `d-modem` container on `root@tower` to this
process's stdin/stdout via `docker exec ... socat`, and speaks AT commands
over that pipe the same way tools/cx_at.py does for a local serial port.
"""
import argparse
import subprocess
import sys
import time

HOST = "root@tower"
CONTAINER = "d-modem"
TTY = "/dev/ttySL0"


def open_bridge():
    cmd = [
        "ssh", HOST,
        f"docker exec -i {CONTAINER} socat {TTY},raw,echo=0,b115200 -",
    ]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL, bufsize=0)


def send(proc, cmd: str, wait: float = 3.0, quiet: bool = False) -> str:
    proc.stdin.write((cmd + "\r").encode())
    proc.stdin.flush()
    deadline = time.time() + wait
    buf = b""
    import select
    while time.time() < deadline:
        r, _, _ = select.select([proc.stdout], [], [], 0.1)
        if r:
            chunk = proc.stdout.read(4096)
            if chunk:
                buf += chunk
                text = buf.decode(errors="replace")
                if any(t in text for t in ("OK", "ERROR", "CONNECT", "NO CARRIER",
                                           "NO DIALTONE", "BUSY", "NO ANSWER")):
                    time.sleep(0.2)
                    r2, _, _ = select.select([proc.stdout], [], [], 0.2)
                    if r2:
                        buf += proc.stdout.read(4096)
                    break
    text = buf.decode(errors="replace")
    if not quiet:
        print(f">>> {cmd}")
        print(text.strip() or "(no response)")
        print("-" * 40)
    return text


def monitor(proc, wait: float):
    import select
    deadline = time.time() + wait
    buf = b""
    while time.time() < deadline:
        r, _, _ = select.select([proc.stdout], [], [], 0.2)
        if r:
            chunk = proc.stdout.read(4096)
            if chunk:
                buf += chunk
                sys.stdout.write(chunk.decode(errors="replace"))
                sys.stdout.flush()
                text = buf.decode(errors="replace")
                if any(t in text for t in ("NO CARRIER", "NO DIALTONE", "BUSY", "NO ANSWER", "ERROR")):
                    break
                if "CONNECT" in text:
                    time.sleep(2.0)
                    r2, _, _ = select.select([proc.stdout], [], [], 1.0)
                    if r2:
                        sys.stdout.write(proc.stdout.read(4096).decode(errors="replace"))
                    break
    print("\n[monitor done]")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["probe", "cmd", "dial"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--wait", type=float, default=90.0)
    opts = ap.parse_args()

    proc = open_bridge()
    time.sleep(1.0)
    try:
        if opts.mode == "probe":
            send(proc, "AT", wait=2.0)
            send(proc, "ATI", wait=2.0)
            send(proc, "ATX3", wait=2.0)
        elif opts.mode == "cmd":
            for c in opts.args:
                send(proc, c, wait=4.0)
        elif opts.mode == "dial":
            number = opts.args[0]
            send(proc, "ATX3", wait=2.0)
            send(proc, "ATE1V1Q0", wait=2.0)
            print(f">>> ATD{number}  (monitoring {opts.wait:.0f}s)")
            proc.stdin.write(f"ATD{number}\r".encode())
            proc.stdin.flush()
            monitor(proc, opts.wait)
            time.sleep(1.0)
            proc.stdin.write(b"+++")
            proc.stdin.flush()
            time.sleep(1.5)
            send(proc, "ATH", wait=3.0)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
