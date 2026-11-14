#!/usr/bin/env python3
"""One call from the hardware Courier on this Mac to our server on vmhost CT 117.

  courier_serial_call.py <name> [--hold 60] [--lines 40] [--at CMD ...]
                         [--env K=V ...] [--dial 2902] [--dev /dev/cu.usbserial-...]

The modem (USRobotics Courier V.Everything, ID_SDL 4.03d, x2 and V.90) is on
/dev/cu.usbserial-FT4TQOFT at 115200 and its line is Cisco 2911 FXS 0/3/0,
caller 7800.  The server half is tools/soak/courier_serial_server.sh in the
CT's /root/x2live worktree (account 2902, u-law, SIP port 5098), started over
ssh; --env passes K=V to it, e.g. --env MODE=v90 --env 'SRV_INIT=ATE0;ATS0=1'.

The modem is set from AT&F1 (never written to NVRAM; ATZ afterwards restores
the stored profile).  After CONNECT each side sends --lines numbered lines and
the lines that arrive exactly are counted.  After the call ATI6 and ATI11 (the
Courier's own link diagnostics) are saved, and the server's log, G.711 taps
and RTP CSVs are pulled into artifacts/<name>/.  Needs pyserial (a venv).
"""
import argparse
import os
import re
import subprocess
import sys
import threading
import time

import serial

SSH = ["ssh", "-o", "HostKeyAlias=vmhost", "root@vmhost.net.cryan.nz"]
REMOTE = "/root/x2live"
LINE = b" the quick brown fox jumps over the lazy dog 0123456789"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name")
    ap.add_argument("--hold", type=int, default=60)
    ap.add_argument("--lines", type=int, default=40)
    ap.add_argument("--at", action="append", default=[])
    ap.add_argument("--env", action="append", default=[])
    ap.add_argument("--dial", default="2902")
    ap.add_argument("--dev", default="/dev/cu.usbserial-FT4TQOFT")
    ap.add_argument("--no-server", action="store_true",
                    help="only dial: something else answers --dial (e.g. the emulated I-modem)")
    a = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    out = os.path.join(root, "artifacts", a.name)
    os.makedirs(out, exist_ok=True)
    log = open(os.path.join(out, "modem.log"), "w")
    t0 = time.time()

    def note(s):
        line = f"[{time.time() - t0:7.2f}] {s}"
        print(line, flush=True)
        log.write(line + "\n")
        log.flush()

    # The remote shell splits this once: quote only what must stay one word.
    envs = " ".join("'" + e.replace("'", "'\\''") + "'" for e in [f"LINES={a.lines}"] + a.env)
    srv = None if a.no_server else subprocess.Popen(SSH + [f"pct exec 117 -- bash -c \"cd {REMOTE} && env {envs} "
                                  f"bash tools/soak/courier_serial_server.sh {a.name} {a.hold}\""],
                           stdout=subprocess.PIPE, text=True)
    srvout = []

    def reader():
        for line in srv.stdout:
            srvout.append(line.strip())
            note("srv: " + line.strip())
    if srv:
        threading.Thread(target=reader, daemon=True).start()

    p = serial.Serial(a.dev, 115200, timeout=0.2, rtscts=True)

    def send(c, wait=2.5):
        p.reset_input_buffer()
        p.write(c.encode() + b"\r")
        end = time.time() + wait
        buf = b""
        while time.time() < end:
            d = p.read(1024)
            if d:
                buf += d
                end = max(end, time.time() + 0.5)
                if buf.rstrip().endswith((b"OK", b"ERROR")):
                    break
        note(f"> {c}  < {buf.decode('latin1').strip()!r}")
        return buf

    for c in ["AT&F1", "ATE1V1Q0X4S0=0"] + a.at:
        send(c)
    if srv:
        while "READY" not in srvout and srv.poll() is None:
            time.sleep(0.5)
        if "READY" not in srvout:
            sys.exit("server did not start")
        time.sleep(6)                               # registration
    p.reset_input_buffer()
    p.write(f"ATDT{a.dial}\r".encode())

    buf = b""
    connect_t = None
    sent = 0
    last = 0.0
    end = time.time() + 100
    while time.time() < end:
        d = p.read(1024)
        if d:
            buf += d
            note(f"rx {d!r}")
        if connect_t is None and re.search(rb"CONNECT[^\r]*\r", buf):
            connect_t = time.time()
            end = connect_t + a.hold
            note("CONNECT")
        if connect_t is None and any(x in buf for x in (b"NO CARRIER", b"BUSY", b"NO ANSWER", b"NO DIAL", b"ERROR")):
            break
        if connect_t and b"NO CARRIER" in buf[buf.find(b"CONNECT"):]:
            break
        if connect_t and time.time() - connect_t > 3 and sent < a.lines and time.time() - last > 0.5:
            p.write(b"cli %05d" % sent + LINE + b"\r\n")
            sent += 1
            last = time.time()
    open(os.path.join(out, "modem-rx.bin"), "wb").write(buf)
    got = len(set(re.findall(rb"srv \d{5}" + LINE, buf)))
    note(f"modem: {got} of {a.lines} srv lines exact; sent {sent}")

    time.sleep(1.2)
    p.write(b"+++")
    time.sleep(1.5)
    p.read(4096)
    for c in ["ATI6", "ATI11"]:
        open(os.path.join(out, c + ".txt"), "wb").write(send(c, 4))
    send("ATH0")
    send("ATZ")
    p.close()

    if not srv:
        return 0
    subprocess.run(SSH + ["pct exec 117 -- touch /tmp/x2hw-stop"])
    try:
        srv.wait(timeout=30)
    except subprocess.TimeoutExpired:
        srv.kill()
    r = subprocess.run(SSH + [f"pct exec 117 -- tar -c -C {REMOTE}/artifacts {a.name}"], capture_output=True)
    subprocess.run(["tar", "-x", "-C", os.path.join(root, "artifacts")], input=r.stdout)
    note(f"artifacts in artifacts/{a.name}")


if __name__ == "__main__":
    sys.exit(main())
