#!/usr/bin/env python3
"""PTY-side pump for the V.90 data-mode soak test.

Opens the server's PTY link, waits for the AT layer's CONNECT, then runs a
three-phase schedule keyed to its own CONNECT sighting:
  Phase A (0-35s):   send downstream pattern lines, read whatever arrives
  Phase B (35-70s):  read-only (upstream test)
  Phase C (70-105s): send + read (bidirectional)
Pattern lines are "D%07d\n" (9 bytes).  All received bytes go to rx_pty.bin
with a timestamped index in rx_pty.idx; progress on stdout every 5 s.
"""
import os, sys, time, termios, select

PTY = "/tmp/modem0"
OUTDIR = sys.argv[1]
SEND_RATE = 5000          # B/s, under the 5200 B/s V.14 payload capacity of 52000 bps
CHUNK = 512
PHASES = [(0, 35, True), (35, 70, False), (70, 105, True)]  # (start, end, sending)
END_T = 110

fd = os.open(PTY, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
attr = termios.tcgetattr(fd)
# raw mode
attr[0] = 0; attr[1] = 0; attr[3] = 0
attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
termios.tcsetattr(fd, termios.TCSANOW, attr)

rxf = open(os.path.join(OUTDIR, "rx_pty.bin"), "wb")
idxf = open(os.path.join(OUTDIR, "rx_pty.idx"), "w")

def read_avail():
    try:
        return os.read(fd, 4096)
    except BlockingIOError:
        return b""

# --- wait for CONNECT ---
buf = b""
t_wait = time.time()
while True:
    r, _, _ = select.select([fd], [], [], 1.0)
    if r:
        d = read_avail()
        buf += d
        if b"CONNECT" in buf:
            print(f"PTY: CONNECT seen: {buf[-80:]!r}", flush=True)
            break
    if time.time() - t_wait > 240:
        print("PTY: no CONNECT within 240s; giving up", flush=True)
        sys.exit(1)

t0 = time.time()
seq = 0
tx_bytes = 0
rx_bytes = 0
send_credit = 0.0
last_credit_t = t0
last_report = t0
pending = b""

while True:
    now = time.time()
    t = now - t0
    if t >= END_T:
        break
    sending = any(a <= t < b and s for a, b, s in PHASES)

    # read
    r, _, _ = select.select([fd], [], [], 0.02)
    if r:
        d = read_avail()
        if d:
            rxf.write(d)
            idxf.write(f"{t:.3f} {len(d)}\n")
            rx_bytes += len(d)
            if b"NO CARRIER" in d:
                print(f"PTY: NO CARRIER at t={t:.1f}s", flush=True)
                break

    # rate-limited send
    send_credit += (now - last_credit_t) * SEND_RATE
    send_credit = min(send_credit, 4 * CHUNK)
    last_credit_t = now
    if sending and send_credit >= CHUNK:
        while len(pending) < CHUNK:
            pending += b"D%07d\n" % seq
            seq += 1
        chunk, pending = pending[:CHUNK], pending[CHUNK:]
        try:
            n = os.write(fd, chunk)
            tx_bytes += n
            send_credit -= n
            if n < len(chunk):
                pending = chunk[n:] + pending
        except BlockingIOError:
            pending = chunk + pending
            time.sleep(0.01)

    if now - last_report >= 5:
        print(f"PTY t={t:5.1f} tx={tx_bytes} rx={rx_bytes}", flush=True)
        last_report = now

rxf.flush(); rxf.close(); idxf.close()
print(f"PTY DONE: tx={tx_bytes} rx={rx_bytes} (lines sent: {seq})", flush=True)
