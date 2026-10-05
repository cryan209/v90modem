#!/usr/bin/env python3
"""One call between TWO slmodemd instances, as a calibration for slm_local_pair.py.

When this engine and slmodemd fail to hold a call, the first question is
whether slmodemd holds the same call against itself.  Here two slmodemd
processes each run rig/slm_bridge/slm_bridge as their -e program, and the two
bridges connect to a relay in this script instead of to audio_sock_modem.  The
relay is the line: each bridge writes one 20 ms frame of G.711 codewords and
then reads one, so the relay reads a frame from each side and hands each to
the other -- byte-exact, one frame of delay each way, no channel at all.

  tools/slm_slm_pair.py --slmodemd <d-modem>/slmodemd/slmodemd \\
      --slm-ms 132,0,7200,7200 [--slm-init 'ATX3;AT\\N0'] [--outdir DIR]

slmodemd in socket mode names its tty /dev/ttySL<device.num> with device.num
hard-wired to 0 (a FIXME in modem_main.c), so two instances need a patched
slmodemd that takes the number from SLM_DEV_NUM; the caller is ttySL0 and the
answerer ttySL1.  Needs root, like slm_local_pair.py.  Each bridge writes its
taps into its own subdirectory (call/, answer/).
"""
import argparse
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from slm_local_pair import Dte, open_pty, count_lines  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRAME = 160


def read_full(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def relay(sock_path, stop):
    """Accept two bridges and swap their frames until either hangs up."""
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    os.chmod(sock_path, 0o666)
    srv.listen(2)
    srv.settimeout(1.0)
    conns = []
    while len(conns) < 2 and not stop.is_set():
        try:
            c, _ = srv.accept()
            conns.append(c)
        except socket.timeout:
            pass
    if len(conns) < 2:
        return
    a, b = conns
    while not stop.is_set():
        fa = read_full(a, FRAME)
        fb = read_full(b, FRAME)
        if fa is None or fb is None:
            break
        try:
            a.sendall(fb)
            b.sendall(fa)
        except OSError:
            break
    for c in conns:
        c.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--slmodemd", required=True)
    ap.add_argument("--slm-ms", default="132,0,4800,14400")
    ap.add_argument("--slm-init", default="ATX3")
    ap.add_argument("--lines", type=int, default=150)
    ap.add_argument("--connect-timeout", type=float, default=60.0)
    ap.add_argument("--outdir", default="/tmp/slm-slm-pair")
    ap.add_argument("--slm-rx-gain-db", type=float, default=-6.0,
                    help="loss into each slmodemd (as slm_local_pair.py)")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    os.chmod(args.outdir, 0o777)
    sock = os.path.join(args.outdir, "line.sock")
    if os.path.exists(sock):
        os.unlink(sock)
    stop = threading.Event()
    t = threading.Thread(target=relay, args=(sock, stop), daemon=True)
    t.start()
    while not os.path.exists(sock):
        time.sleep(0.05)

    procs = []
    ttys = {}
    for num, role in ((0, "call"), (1, "answer")):
        d = os.path.join(args.outdir, role)
        os.makedirs(d, exist_ok=True)
        os.chmod(d, 0o777)
        env = dict(os.environ)
        env["SLM_DEV_NUM"] = str(num)
        env["SLM_BRIDGE_SOCKET"] = sock
        env["SLM_BRIDGE_TAP_DIR"] = d
        env["SLM_BRIDGE_RX_GAIN_DB"] = str(args.slm_rx_gain_db)
        link = "/dev/ttySL%d" % num
        if os.path.islink(link):
            os.unlink(link)
        log = open(os.path.join(d, "slmodemd.log"), "wb")
        procs.append(subprocess.Popen([args.slmodemd, "-d9", "-e", os.path.join(ROOT, "slm_bridge")],
                                      stdout=log, stderr=subprocess.STDOUT, env=env, cwd=d))
        ttys[role] = link
    rc = 1
    try:
        call = Dte("call  ", open_pty(ttys["call"]))
        answer = Dte("answer", open_pty(ttys["answer"]))
        time.sleep(0.5)
        for d in (call, answer):
            d.cmd("ATE0")
            for c in args.slm_init.split(";"):
                if c:
                    d.cmd(c)
            if args.slm_ms:
                d.cmd("AT+MS=" + args.slm_ms)
        call.write(b"ATD1234\r")
        time.sleep(0.5)
        answer.write(b"ATA\r")

        t0 = time.time()
        rates = {}
        ends = {"call": call, "answer": answer}
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

        sent = 0
        next_t = time.time()
        while sent < args.lines:
            if time.time() >= next_t:
                call.write(b"C%07d\r\n" % (sent + 1))
                answer.write(b"A%07d\r\n" % (sent + 1))
                sent += 1
                next_t += 0.04
            call.poll(0.005)
            answer.poll(0.0)
        settle = time.time() + 5.0
        while time.time() < settle:
            call.poll(0.05)
            answer.poll(0.0)
        a_in, a_stray = count_lines(answer.buf[answer.data_from:], b"C", args.lines)
        c_in, c_stray = count_lines(call.buf[call.data_from:], b"A", args.lines)
        print(f"  call -> answer: {a_in}/{args.lines} lines intact and in order, {a_stray} other bytes")
        print(f"  answer -> call: {c_in}/{args.lines} lines intact and in order, {c_stray} other bytes")
        with open(os.path.join(args.outdir, "call-dte.bin"), "wb") as f:
            f.write(call.buf)
        with open(os.path.join(args.outdir, "answer-dte.bin"), "wb") as f:
            f.write(answer.buf)
        rc = 0 if (a_in == args.lines and c_in == args.lines) else 1
        print("RESULT:", "PASS" if rc == 0 else "FAIL", rates)
    finally:
        stop.set()
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
