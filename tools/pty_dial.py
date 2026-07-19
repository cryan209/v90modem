#!/usr/bin/env python3
"""Dial a number via the sip_v90_modem PTY (AT interface) and log the result.

Usage: pty_dial.py <number> [--wait SECONDS] [--pty /tmp/v90modem]
"""
import argparse
import os
import sys
import termios
import time
import tty


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("number")
    ap.add_argument("--pty", default="/tmp/v90modem")
    ap.add_argument("--wait", type=float, default=90.0)
    opts = ap.parse_args()

    fd = os.open(opts.pty, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd, termios.TCSANOW)
    os.set_blocking(fd, False)

    t0 = time.time()

    def send(s: str) -> None:
        os.write(fd, s.encode())
        print(f">>> {s.strip()!r}", flush=True)

    def drain(duration: float) -> str:
        deadline = time.time() + duration
        buf = b""
        while time.time() < deadline:
            try:
                chunk = os.read(fd, 4096)
                if chunk:
                    buf += chunk
                    text = buf.decode(errors="replace")
                    sys.stdout.write(chunk.decode(errors="replace"))
                    sys.stdout.flush()
                    if any(k in text for k in ("NO CARRIER", "NO DIALTONE", "BUSY", "ERROR")):
                        time.sleep(0.3)
                        try:
                            buf += os.read(fd, 4096)
                        except BlockingIOError:
                            pass
                        break
                    if "CONNECT" in text:
                        break
            except BlockingIOError:
                time.sleep(0.05)
        return buf.decode(errors="replace")

    send("AT\r")
    time.sleep(1)
    drain(1.0)
    send(f"ATD{opts.number}\r")
    print(f"[{time.time()-t0:.1f}s] dialing, monitoring {opts.wait:.0f}s...", flush=True)
    result = drain(opts.wait)

    if "CONNECT" in result:
        print(f"\n[{time.time()-t0:.1f}s] CONNECTED -- monitoring another 10s of data mode", flush=True)
        drain(10.0)

    time.sleep(0.5)
    send("+++")
    time.sleep(1.5)
    send("ATH\r")
    time.sleep(1)
    drain(2.0)
    print(f"[{time.time()-t0:.1f}s] done", flush=True)
    os.close(fd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
