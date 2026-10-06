#!/usr/bin/env python3
"""One call from a serial AT modem on this host to our server on tower.

  serial_dial_call.py <name> [--dial 2901] [--hold 40] [--dev /dev/cu.usbserial-...]
                      [--env ME_V8BIS=1 ...] [--at "AT+MS=V90,1,300,33600,300,56000" ...]

Starts sip_v90_modem in the v90modem-sip container on tower as account --dial
(2900-2904 are u-law, 2905-2909 A-law; the container build must be current),
dials that account from the modem, logs every byte the modem prints with a
timestamp, holds the call, and pulls the server log and G.711 taps back to
artifacts/<name>/.  After CONNECT the server writes numbered lines into its
PTY so a data path that decodes is proved by the bytes arriving on the modem's
serial side.  Needs pyserial.  Leave ~90 s between calls.
"""
import argparse
import os
import subprocess
import sys
import time

import serial

TOWER = "tower.net.cryan.nz"
CONTAINER = "v90modem-sip"


def tower(cmd, **kw):
    return subprocess.run(["ssh", TOWER, cmd], capture_output=True, text=True, **kw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("--dial", default="2901")
    ap.add_argument("--hold", type=int, default=40)
    ap.add_argument("--dev", default="/dev/cu.usbserial-FT4TQOFT")
    ap.add_argument("--env", action="append", default=[])
    ap.add_argument("--at", action="append", default=[])
    ap.add_argument("--wait", type=int, default=90, help="seconds to wait for CONNECT")
    a = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    out = os.path.join(root, "artifacts", a.name)
    os.makedirs(out, exist_ok=True)
    rdir = f"/root/v90modem/artifacts/{a.name}"
    law = "SIP_FORCE_PCMA=1" if a.dial in ("2905", "2906", "2907", "2908", "2909") else "SIP_FORCE_PCMU=1"
    envs = " ".join(f"-e {e}" for e in a.env + [law, "ME_LAPM_XID_OPTION_OCTETS=auto", f"VPCM_G711_TAP_DIR={rdir}",
                                                   "VPCM_ME_VERBOSE=1"])
    log = open(os.path.join(out, "modem.log"), "w")
    t0 = time.time()

    def note(s):
        line = f"[{time.time() - t0:7.2f}] {s}"
        print(line)
        log.write(line + "\n")
        log.flush()

    # the bracket keeps pkill -f from matching (and killing) the shell that runs it
    tower(f"docker exec {CONTAINER} sh -c 'pkill -f \"[l]ocal-port 5074\"; true'")
    tower(f"docker exec {CONTAINER} mkdir -p {rdir}")
    tower(f"docker exec -d {envs} {CONTAINER} sh -c 'cd /root/v90modem && timeout {a.hold + a.wait + 60} "
          f"./sip_v90_modem --sip-server asterisk.net.cryan.nz --username {a.dial} --password {a.dial} "
          f"--local-port 5074 --rtp-port 14100 --pty-link /tmp/v90ser --verbose > {rdir}/server.log 2>&1'")
    time.sleep(7)                                   # registration
    tower(f"docker exec {CONTAINER} sh -c 'stty -F /tmp/v90ser raw -echo; (cat /tmp/v90ser > {rdir}/pty-rx.bin &)'")
    note(f"server started on tower as {a.dial} ({law}); env {a.env}")

    p = serial.Serial(a.dev, 115200, timeout=0.2)

    def send(c, wait=1.5):
        p.reset_input_buffer()
        p.write(c.encode() + b"\r")
        end = time.time() + wait
        buf = b""
        while time.time() < end:
            d = p.read(256)
            if d:
                buf += d
                if b"OK" in buf or b"ERROR" in buf:
                    break
        note(f"> {c}   < {buf.decode(errors='replace').strip()!r}")
        return buf

    p.write(b"\x03")
    time.sleep(0.3)
    p.read(4096)
    for c in ["ATZ", "ATE1V1Q0X3", "AT\\N0"] + a.at:
        send(c)
    send(f"ATDT{a.dial}", 0.5)

    buf = b""
    state = "dialling"
    end = time.time() + a.wait
    connect_t = None
    pay = None
    while time.time() < end:
        d = p.read(512)
        if not d:
            if connect_t and time.time() - connect_t > a.hold:
                break
            continue
        buf += d
        note(f"rx {d!r}")
        if b"CONNECT" in buf and connect_t is None:
            connect_t = time.time()
            end = connect_t + a.hold + 5
            note("CONNECT; starting numbered lines from the server's DTE")
            tower(f"docker exec -d {CONTAINER} sh -c 'i=0; e=$(( $(date +%s) + {a.hold} )); "
                  f"while [ $(date +%s) -lt $e ]; do printf \"D%07d\\r\\n\" $i > /tmp/v90ser; i=$((i+1)); sleep 0.05; done'")
        if any(x in buf for x in (b"NO CARRIER", b"BUSY", b"NO ANSWER", b"NO DIALTONE", b"NO DIAL TONE", b"ERROR")) \
                and connect_t is None:
            break
        if connect_t and (b"NO CARRIER" in buf[buf.find(b"CONNECT"):]):
            break
    note("hanging up")
    time.sleep(1.2)
    p.write(b"+++")
    time.sleep(1.5)
    p.write(b"ATH0\r")
    time.sleep(1)
    p.write(b"ATZ\r")
    time.sleep(0.5)
    open(os.path.join(out, "modem-rx.bin"), "wb").write(buf)
    p.close()

    tower(f"docker exec {CONTAINER} sh -c 'pkill -f \"[l]ocal-port 5074\"; true'")
    time.sleep(1)
    r = subprocess.run(["ssh", TOWER, f"docker exec {CONTAINER} tar -c -C /root/v90modem/artifacts {a.name}"],
                       capture_output=True)
    subprocess.run(["tar", "-x", "-C", os.path.join(root, "artifacts")], input=r.stdout, capture_output=True)
    note(f"artifacts in artifacts/{a.name}")
    lines = [l for l in buf.decode(errors="replace").splitlines() if "D0" in l]
    note(f"data lines from the server seen on the modem: {len(lines)}")


if __name__ == "__main__":
    sys.exit(main())
