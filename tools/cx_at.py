#!/usr/bin/env python3
"""Minimal AT driver for the Conexant CX93010 USB modem.

Usage:
  cx_at.py probe                     — reset + identify
  cx_at.py cmd 'ATI3' 'ATI6' ...     — run arbitrary commands
  cx_at.py dial 6001 --wait 60       — dial and log everything until CONNECT/NO CARRIER/timeout
  cx_at.py answer --wait 60          — ATA (manual answer) and log
"""
import argparse
import sys
import time

import serial


def open_port(dev: str) -> serial.Serial:
    return serial.Serial(dev, 115200, timeout=0.2, rtscts=False, dsrdtr=False)


def send(port: serial.Serial, cmd: str, wait: float = 2.0, quiet: bool = False) -> str:
    port.reset_input_buffer()
    port.write((cmd + "\r").encode())
    port.flush()
    deadline = time.time() + wait
    buf = b""
    while time.time() < deadline:
        chunk = port.read(256)
        if chunk:
            buf += chunk
            text = buf.decode(errors="replace")
            if any(t in text for t in ("OK", "ERROR", "CONNECT", "NO CARRIER",
                                       "NO DIALTONE", "NO DIAL TONE", "BUSY", "NO ANSWER")):
                # allow trailing bytes to arrive
                time.sleep(0.1)
                buf += port.read(1024)
                break
    text = buf.decode(errors="replace")
    if not quiet:
        print(f">>> {cmd}")
        print(text.strip() or "(no response)")
        print("-" * 40)
    return text


def monitor(port: serial.Serial, wait: float) -> None:
    deadline = time.time() + wait
    buf = b""
    while time.time() < deadline:
        chunk = port.read(256)
        if chunk:
            buf += chunk
            sys.stdout.write(chunk.decode(errors="replace"))
            sys.stdout.flush()
            text = buf.decode(errors="replace")
            if any(t in text for t in ("NO CARRIER", "NO DIALTONE", "NO DIAL TONE",
                                       "BUSY", "NO ANSWER", "ERROR")):
                time.sleep(0.5)
                sys.stdout.write(port.read(4096).decode(errors="replace"))
                break
            if "CONNECT" in text:
                # stay connected a bit to observe
                time.sleep(2.0)
                sys.stdout.write(port.read(4096).decode(errors="replace"))
                break
    print("\n[monitor done]")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/cu.usbmodem123456781")
    ap.add_argument("mode", choices=["probe", "cmd", "dial", "answer"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--wait", type=float, default=60.0)
    ap.add_argument("--setup", action="append", default=[],
                    help="extra AT command(s) before dialing")
    opts = ap.parse_args()

    port = open_port(opts.dev)
    try:
        if opts.mode == "probe":
            send(port, "AT", wait=1.0)
            send(port, "ATZ", wait=3.0)
            send(port, "ATE1V1Q0")
            for c in ("ATI0", "ATI1", "ATI2", "ATI3", "ATI4", "ATI5", "ATI6", "ATI7",
                      "AT+GMM", "AT+GMR", "AT+FCLASS=?", "AT+MS=?"):
                send(port, c, wait=2.0)
        elif opts.mode == "cmd":
            for c in opts.args:
                send(port, c, wait=5.0)
        elif opts.mode == "dial":
            number = opts.args[0]
            send(port, "ATZ", wait=3.0)
            send(port, "ATE1V1Q0X3")
            for c in opts.setup:
                send(port, c, wait=3.0)
            port.reset_input_buffer()
            port.write((f"ATDT{number}\r").encode())
            port.flush()
            print(f">>> ATDT{number}  (monitoring {opts.wait:.0f}s)")
            monitor(port, opts.wait)
            # hang up: DTR drop + +++ATH
            time.sleep(1.0)
            port.write(b"+++")
            port.flush()
            time.sleep(1.5)
            send(port, "ATH", wait=3.0)
        elif opts.mode == "answer":
            for c in opts.setup:
                send(port, c, wait=3.0)
            port.reset_input_buffer()
            port.write(b"ATA\r")
            port.flush()
            print(f">>> ATA  (monitoring {opts.wait:.0f}s)")
            monitor(port, opts.wait)
            time.sleep(1.0)
            port.write(b"+++")
            port.flush()
            time.sleep(1.5)
            send(port, "ATH", wait=3.0)
    finally:
        port.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
