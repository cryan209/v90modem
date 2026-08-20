#!/usr/bin/env python3
"""Socket-side (d-modem DTE) pump for the V.90 data-mode soak test.

Connects to the tower socat AT bridge, dials, waits for CONNECT, then runs
the mirror of soak_pty.py's schedule keyed to its own CONNECT sighting:
  Phase A (0-35s):   read-only (downstream test)
  Phase B (35-70s):  send upstream pattern lines
  Phase C (70-105s): send + read (bidirectional)
Pattern lines are "U%07d\n".  Received bytes -> rx_sock.bin + rx_sock.idx.
"""
import os, sys, time, socket, select

HOST, PORT = "tower.net.cryan.nz", 5556
OUTDIR = sys.argv[1]
SEND_RATE = int(os.environ.get("SOAK_SEND_RATE", "3000"))  # B/s
# A low SOAK_SEND_RATE leaves the upstream mostly idle, so a decode fault is
# obvious: idle marks read as ones and the occasional character stands out.
CHUNK = 512
PHASES = [(0, 35, False), (35, 70, True), (70, 105, True)]
if os.environ.get("SOAK_SOCK_ALWAYS"):
    # Send for the whole call rather than only the last two thirds.  To ask
    # whether payload flows correctly for minutes, payload has to be flowing
    # for minutes -- otherwise most of the capture is idle marks and a clean
    # result says only that the line is quiet.
    PHASES = [(0, 105, True)]
# SOAK_SECONDS stretches the three-phase schedule for a long correctness run
# (the default 105 s proves the path; minutes prove it stays up).
SOAK_SCALE = max(1.0, float(os.environ.get("SOAK_SECONDS", "105"))/105.0)
PHASES = [(a*SOAK_SCALE, b*SOAK_SCALE, c) for (a, b, c) in PHASES]
END_T = 110*SOAK_SCALE

s = socket.create_connection((HOST, PORT), timeout=20)
s.setblocking(False)

def send_at(cmd, wait=2.0):
    s.sendall(cmd.encode() + b"\r")
    end = time.time() + wait
    got = b""
    while time.time() < end:
        r, _, _ = select.select([s], [], [], 0.2)
        if r:
            try:
                got += s.recv(4096)
            except BlockingIOError:
                pass
    print(f"SOCK: {cmd} -> {got!r}", flush=True)
    return got

send_at("ATZ"); send_at("ATX3"); send_at("AT\\N0"); send_at("AT+MS=90,1,300,56000")

# Drain anything still queued on the pty (socat fork children can deliver a
# previous call's trailing NO CARRIER here, which used to abort the soak
# before the dial had even been placed).
drain_end = time.time() + 1.0
while time.time() < drain_end:
    r, _, _ = select.select([s], [], [], 0.2)
    if r:
        try:
            s.recv(4096)
        except BlockingIOError:
            pass

rxf = open(os.path.join(OUTDIR, "rx_sock.bin"), "wb")
idxf = open(os.path.join(OUTDIR, "rx_sock.idx"), "w")

# A dial placed within a few seconds of a rig bounce comes straight back as
# NO CARRIER (the d-modem child has not finished registering with the SIP
# server yet).  A redial a few seconds later goes through, so treat an
# immediate NO CARRIER as "not ready" rather than as a failed soak.
MAX_DIALS = 12
dials = 0

def place_dial():
    global dials, buf
    dials += 1
    buf = b""
    s.sendall(b"ATD6001\r")
    print(f"SOCK: dialing (attempt {dials})", flush=True)

buf = b""
place_dial()
t_wait = time.time()
while True:
    r, _, _ = select.select([s], [], [], 1.0)
    if r:
        try:
            d = s.recv(4096)
        except BlockingIOError:
            d = b""
        if d == b"" and r:
            print("SOCK: socket closed while waiting for CONNECT", flush=True)
            sys.exit(1)
        buf += d
        # Only judge what follows the dial's own echo.
        if b"ATD6001" in buf:
            buf = buf.split(b"ATD6001", 1)[1]
        if b"CONNECT" in buf:
            print(f"SOCK: CONNECT seen: {buf[-120:]!r}", flush=True)
            break
        if b"NO CARRIER" in buf or b"BUSY" in buf or b"ERROR" in buf:
            # A genuine dial cannot fail this fast: the SIP call takes about
            # six seconds just to reach CONNECTING.  Anything arriving sooner
            # is a previous call's queued result overtaking our dial echo on
            # the shared pty -- swallow it and keep waiting.
            if time.time() - t_wait < 5.0:
                print(f"SOCK: ignoring stale {buf[-40:]!r} "
                      f"{time.time() - t_wait:.1f}s after dialing", flush=True)
                buf = b""
                continue
            print(f"SOCK: dial failed: {buf[-120:]!r}", flush=True)
            if dials >= MAX_DIALS:
                sys.exit(1)
            time.sleep(8)
            place_dial()
            t_wait = time.time()
            continue
    if time.time() - t_wait > 240:
        print("SOCK: no CONNECT within 240s; giving up", flush=True)
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
    sending = any(a <= t < b and snd for a, b, snd in PHASES)

    r, _, _ = select.select([s], [], [], 0.02)
    if r:
        try:
            d = s.recv(4096)
        except BlockingIOError:
            d = b""
        if d == b"":
            print(f"SOCK: socket closed at t={t:.1f}s", flush=True)
            break
        rxf.write(d)
        idxf.write(f"{t:.3f} {len(d)}\n")
        rx_bytes += len(d)
        if b"NO CARRIER" in d:
            print(f"SOCK: NO CARRIER at t={t:.1f}s", flush=True)
            break

    send_credit += (now - last_credit_t) * SEND_RATE
    send_credit = min(send_credit, 4 * CHUNK)
    last_credit_t = now
    if sending and send_credit >= CHUNK:
        while len(pending) < CHUNK:
            pending += b"U%07d\n" % seq
            seq += 1
        chunk, pending = pending[:CHUNK], pending[CHUNK:]
        try:
            n = s.send(chunk)
            tx_bytes += n
            send_credit -= n
            if n < len(chunk):
                pending = chunk[n:] + pending
        except (BlockingIOError, BrokenPipeError) as e:
            if isinstance(e, BrokenPipeError):
                print(f"SOCK: broken pipe at t={t:.1f}s", flush=True)
                break
            pending = chunk + pending
            time.sleep(0.01)

    if now - last_report >= 5:
        print(f"SOCK t={t:5.1f} tx={tx_bytes} rx={rx_bytes}", flush=True)
        last_report = now

rxf.flush(); rxf.close(); idxf.close()
print(f"SOCK DONE: tx={tx_bytes} rx={rx_bytes} (lines sent: {seq})", flush=True)
try:
    s.close()
except Exception:
    pass
