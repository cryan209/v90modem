#!/usr/bin/env python3
"""One call between slmodemd (SmartLink soft modem) and audio_sock_modem, locally.

No SIP, no rig: slmodemd runs rig/slm_bridge/slm_bridge as its -e program,
which connects slmodemd's 9600 Hz audio socket to audio_sock_modem's G.711
socket (see both files' headers).  Each DTE is driven over its PTY, numbered
lines are sent both ways once CONNECT is up, and the call is graded on lines
that arrive intact and in order -- the same measure v32bis_engine_pair_test
uses.

  tools/slm_local_pair.py --slmodemd <d-modem>/slmodemd/slmodemd \\
      --slm-ms 132,0,4800,14400 [--originate slm|ours] [--ours-at 'AT+MS=V32B']

--slm-ms is slmodemd's AT+MS (SmartLink numbering: 132 = V.32bis, 34 = V.34,
56 = K56flex, 90 = V.90, ...).  --originate says who dials; the other end
answers (ours auto-answers a connection; slmodemd answers with ATA, which is
what makes it run the bridge with an empty dial string).

Needs root for slmodemd's /dev/ttySL<n> symlink.
"""
import argparse
import os
import re
import select
import signal
import subprocess
import sys
import termios
import time
import tty

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def open_pty(path, timeout=10.0):
    deadline = time.time() + timeout
    while not os.path.exists(path):
        if time.time() > deadline:
            raise SystemExit(f"{path} never appeared")
        time.sleep(0.05)
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[3] &= ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


class Dte:
    def __init__(self, name, fd):
        self.name = name
        self.fd = fd
        self.buf = b""

    def poll(self, wait=0.0):
        r, _, _ = select.select([self.fd], [], [], wait)
        if r:
            try:
                chunk = os.read(self.fd, 65536)
            except BlockingIOError:
                chunk = b""
            self.buf += chunk

    def write(self, data):
        view = memoryview(data)
        while view:
            try:
                n = os.write(self.fd, view)
                view = view[n:]
            except BlockingIOError:
                time.sleep(0.005)

    def cmd(self, text, want=(b"OK", b"ERROR"), timeout=5.0):
        start = len(self.buf)
        self.write(text.encode() + b"\r")
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.poll(0.05)
            tail = self.buf[start:]
            for w in want:
                if w in tail:
                    print(f"  {self.name}: {text!r} -> {tail.decode(errors='replace').strip()!r}")
                    return tail
        print(f"  {self.name}: {text!r} -> (timeout) {self.buf[start:]!r}")
        return self.buf[start:]


def count_lines(data, peer, total):
    """Intact peer lines in order, and bytes that are not part of one."""
    expected = 1
    intact = 0
    for m in re.finditer(rb"%s(\d{7})\r\n" % peer, data):
        if int(m.group(1)) == expected:
            intact += 1
            expected += 1
    return intact, len(data) - intact * 10


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--slmodemd", required=True)
    ap.add_argument("--slm-ms", default="132,0,4800,14400",
                    help="slmodemd AT+MS value (default: V.32bis 4800-14400)")
    ap.add_argument("--slm-init", default="ATX3",
                    help="extra slmodemd init (default ATX3: no dial tone on this line)")
    ap.add_argument("--ours-at", action="append", default=[],
                    help="AT commands for our modem before the call")
    ap.add_argument("--originate", choices=("slm", "ours"), default="slm")
    ap.add_argument("--alaw", action="store_true")
    ap.add_argument("--lines", type=int, default=150)
    ap.add_argument("--connect-timeout", type=float, default=60.0)
    ap.add_argument("--outdir", default="/tmp/slm-pair")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--slm-rx-gain-db", type=float, default=-6.0,
                    help="gain on the line audio into slmodemd (default -6: a short loop; at 0 slmodemd overloads on our -11 dBm0 and retrains at 12000+)")
    ap.add_argument("--slm-echo-db", type=float, default=None,
                    help="near end hybrid echo of slmodemd's own transmit, in dB (default none)")
    ap.add_argument("--ours-env", action="append", default=[],
                    help="NAME=VALUE for audio_sock_modem's environment")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    sock = os.path.join(args.outdir, "line.sock")
    ours_pty = os.path.join(args.outdir, "ours-pty")
    procs = []
    env = dict(os.environ)
    for kv in args.ours_env:
        k, v = kv.split("=", 1)
        env[k] = v
    ours_cmd = [os.path.join(ROOT, "audio_sock_modem"), "--listen", sock,
                "--pty-link", ours_pty]
    if args.alaw:
        ours_cmd.append("--alaw")
    if args.verbose:
        ours_cmd.append("--verbose")
    ours_log = open(os.path.join(args.outdir, "ours.log"), "wb")
    procs.append(subprocess.Popen(ours_cmd, stdout=ours_log, stderr=subprocess.STDOUT,
                                  env=env, cwd=args.outdir))
    while not os.path.exists(sock):
        time.sleep(0.05)
    # slmodemd drops root before it runs the bridge.
    os.chmod(args.outdir, 0o777)
    os.chmod(sock, 0o666)

    slm_env = dict(os.environ)
    slm_env["SLM_BRIDGE_SOCKET"] = sock
    slm_env["SLM_BRIDGE_TAP_DIR"] = args.outdir
    slm_env["SLM_BRIDGE_RX_GAIN_DB"] = str(args.slm_rx_gain_db)
    if args.slm_echo_db is not None:
        slm_env["SLM_BRIDGE_ECHO_DB"] = str(args.slm_echo_db)
    if args.alaw:
        slm_env["SLM_BRIDGE_ALAW"] = "1"
    slm_log = open(os.path.join(args.outdir, "slmodemd.log"), "wb")
    for stale in ("/dev/ttySL0",):
        if os.path.islink(stale):
            os.unlink(stale)
    procs.append(subprocess.Popen([args.slmodemd, "-d9", "-e", os.path.join(ROOT, "slm_bridge")],
                                  stdout=slm_log, stderr=subprocess.STDOUT, env=slm_env,
                                  cwd=args.outdir))
    rc = 1
    try:
        ours = Dte("ours", open_pty(ours_pty))
        slm = Dte("slm ", open_pty("/dev/ttySL0"))
        time.sleep(0.5)
        ours.cmd("ATE0")
        for c in args.ours_at:
            ours.cmd(c)
        slm.cmd("ATE0")
        for c in args.slm_init.split(";"):
            if c:
                slm.cmd(c)
        if args.slm_ms:
            slm.cmd("AT+MS=" + args.slm_ms)

        if args.originate == "slm":
            # Our modem answers the bridge's connection on its own.
            slm.write(b"ATD1234\r")
        else:
            ours.write(b"ATD1234\r")
            time.sleep(0.5)
            slm.write(b"ATA\r")

        t0 = time.time()
        rates = {}
        ends = {"ours": ours, "slm ": slm}
        while time.time() - t0 < args.connect_timeout and len(rates) < 2:
            for name, d in ends.items():
                d.poll(0.02)
                m = re.search(rb"CONNECT[ ]?(\d*)[^\r\n]*\r\n", d.buf)
                if m and name not in rates:
                    rates[name] = m.group(0).decode().strip()
                    d.data_from = m.end()
                    print(f"  {name}: {rates[name]} at {time.time() - t0:.1f} s")
                if b"NO CARRIER" in d.buf and name not in rates:
                    print(f"  {name}: NO CARRIER before CONNECT at {time.time() - t0:.1f} s")
                    raise SystemExit(1)
        if len(rates) < 2:
            print("  no CONNECT on both ends")
            raise SystemExit(1)

        # Numbered lines both ways, one every 40 ms (2 kbit/s of DTE text),
        # as v32bis_engine_pair_test does.
        sent = 0
        next_t = time.time()
        while sent < args.lines:
            if time.time() >= next_t:
                ours.write(b"O%07d\r\n" % (sent + 1))
                slm.write(b"S%07d\r\n" % (sent + 1))
                sent += 1
                next_t += 0.04
            ours.poll(0.005)
            slm.poll(0.0)
        settle = time.time() + 5.0
        while time.time() < settle:
            ours.poll(0.05)
            slm.poll(0.0)
            a, _ = count_lines(ours.buf[ours.data_from:], b"S", args.lines)
            b, _ = count_lines(slm.buf[slm.data_from:], b"O", args.lines)
            if a == args.lines and b == args.lines:
                break
        ours_in, ours_stray = count_lines(ours.buf[ours.data_from:], b"S", args.lines)
        slm_in, slm_stray = count_lines(slm.buf[slm.data_from:], b"O", args.lines)
        print(f"  slm -> ours: {ours_in}/{args.lines} lines intact and in order, "
              f"{ours_stray} other bytes")
        print(f"  ours -> slm: {slm_in}/{args.lines} lines intact and in order, "
              f"{slm_stray} other bytes")
        with open(os.path.join(args.outdir, "ours-dte.bin"), "wb") as f:
            f.write(ours.buf)
        with open(os.path.join(args.outdir, "slm-dte.bin"), "wb") as f:
            f.write(slm.buf)
        rc = 0 if (ours_in == args.lines and slm_in == args.lines) else 1
        print("RESULT:", "PASS" if rc == 0 else "FAIL", rates)
    finally:
        for p in reversed(procs):
            p.send_signal(signal.SIGTERM)
        for p in procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
    sys.exit(rc)


if __name__ == "__main__":
    main()
