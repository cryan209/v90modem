#!/usr/bin/env python3
"""Numbered-line writer for tools/soak/v34_rate_call.sh.

Waits for the AT layer's CONNECT on the server's PTY, then writes
"S%07d\n" lines at a rate the slowest V.34 profile under test can carry, and
records everything that comes back.  Line counts are the honest measure here
(see docs/v90_upstream_data_path.md): a byte-level "percent clean" figure
counts the garbage emitted while the link is not carrying anything.
"""
import os, sys, time, select, termios

PTY = "/tmp/v90modem"
OUT = sys.argv[1]
RATE = int(os.environ.get("V34_RATE_PTY_BPS", "600"))   # bytes/s
HOLD = float(os.environ.get("V34_RATE_PTY_SECONDS", "600"))

fd = None
for _ in range(60):
    try:
        fd = os.open(PTY, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        break
    except OSError:
        time.sleep(1)
if fd is None:
    sys.exit("no PTY")

# Without this the lines written here come straight back on the same fd and
# pty_rx.bin records our own echo rather than the peer -- which reads exactly
# like a working receive direction and is not one.
try:
    a = termios.tcgetattr(fd)
    a[3] &= ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, a)
except termios.error:
    pass

rx = open(os.path.join(OUT, "pty_rx.bin"), "wb")
buf = b""
connected = False
n = 0
start = time.time()
next_send = start
while time.time() - start < HOLD:
    r, _, _ = select.select([fd], [], [], 0.05)
    if r:
        try:
            d = os.read(fd, 4096)
        except OSError:
            d = b""
        if d:
            rx.write(d); rx.flush()
            buf += d
            if not connected and b"CONNECT" in buf:
                connected = True
                print("PTY: CONNECT seen", flush=True)
    if connected and time.time() >= next_send:
        n += 1
        line = b"S%07d\n" % n
        try:
            os.write(fd, line)
        except OSError:
            pass
        next_send += len(line)/RATE
print("PTY: sent %d lines" % n, flush=True)
