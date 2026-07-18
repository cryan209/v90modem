#!/usr/bin/env python3
"""Reverse-direction test: set Conexant to auto-answer, then dial 6311 from
the v90modem PTY and log both sides with timestamps."""
import os
import sys
import termios
import threading
import time

import serial
import tty

T0 = time.time()

def log(tag, text):
    print(f"[{time.time()-T0:7.2f}s] {tag}: {text}", flush=True)

# 1. Conexant: reset + auto-answer after 1 ring
mdm = serial.Serial("/dev/cu.usbmodem123456781", 115200, timeout=0.2)
def at(cmd, wait=3.0):
    mdm.reset_input_buffer()
    mdm.write((cmd + "\r").encode())
    deadline = time.time() + wait
    buf = b""
    while time.time() < deadline:
        chunk = mdm.read(256)
        if chunk:
            buf += chunk
            if b"OK" in buf or b"ERROR" in buf:
                break
    log("CX", f"{cmd} -> {buf.decode(errors='replace').strip()!r}")

at("ATZ")
at("ATE1V1Q0X3")
at("ATS0=1")

def cx_monitor():
    deadline = time.time() + 100
    while time.time() < deadline:
        chunk = mdm.read(256)
        if chunk:
            log("CX", repr(chunk.decode(errors="replace")))

threading.Thread(target=cx_monitor, daemon=True).start()

# 2. PTY: dial 6311 (raw mode — otherwise slave-side echo loops our writes back)
fd = os.open("/tmp/v90modem", os.O_RDWR | os.O_NOCTTY)
tty.setraw(fd, termios.TCSANOW)
os.set_blocking(fd, False)

def pty_write(s):
    os.write(fd, s.encode())
    log("PTY>", s.strip())

def pty_monitor():
    deadline = time.time() + 100
    while time.time() < deadline:
        try:
            data = os.read(fd, 256)
            if data:
                log("PTY<", repr(data.decode(errors="replace")))
        except BlockingIOError:
            time.sleep(0.05)

threading.Thread(target=pty_monitor, daemon=True).start()

time.sleep(1)
pty_write("AT\r")
time.sleep(1)
pty_write("ATD6311\r")

time.sleep(95)
pty_write("+++")
time.sleep(1.5)
pty_write("ATH\r")
time.sleep(2)
mdm.write(b"+++")
time.sleep(1.5)
mdm.write(b"ATH\r")
time.sleep(1)
mdm.write(b"ATS0=0\r")
time.sleep(1)
log("DONE", "test complete")
