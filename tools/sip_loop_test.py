#!/usr/bin/env python3
"""Two sip_v90_modem instances calling each other over SIP on 127.0.0.1.

No registrar: the caller's --sip-server is the answerer's own address, so
ATD<n> becomes an INVITE straight to it.  This is the one test of the paths
that live only in sip_modem.c -- ringing, V.253 caller ID after the first
RING, answering by S0 (--auto-answer) and by ATA, CONNECT on both sides,
the far end's NO CARRIER after ATH, and a dial to an address where nothing
answers ending in a result code instead of silence.  V.22bis keeps each call
to a few seconds.  Exit status is the number of failed checks.

    make sip-loop-test        (or: python3 tools/sip_loop_test.py)
"""
import os
import select
import subprocess
import sys
import tempfile
import time
import tty

failures = 0


def check(ok, what):
    global failures
    print(("  ok   " if ok else "  FAIL ") + what)
    if not ok:
        failures += 1


def open_pty(path, wait=10.0):
    t0 = time.time()
    while not os.path.exists(path):
        if time.time() - t0 > wait:
            raise SystemExit("no pty at " + path)
        time.sleep(0.1)
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    return fd


def pump(bufs, secs, until=None):
    end = time.time() + secs
    while time.time() < end:
        r, _, _ = select.select(list(bufs), [], [], 0.1)
        for fd in r:
            try:
                bufs[fd] += os.read(fd, 4096)
            except BlockingIOError:
                pass
        if until and until(bufs):
            return True
    return False


def modem(args, log):
    env = dict(os.environ, ME_MODE="v22")
    return subprocess.Popen(["./sip_v90_modem", "--bind-addr", "127.0.0.1"] + args, env=env,
                            stdout=open(log, "w"), stderr=subprocess.STDOUT)


def call(tmp, answer_by_ata):
    name = "ATA" if answer_by_ata else "S0=2"
    print(f"call answered by {name}:")
    b_args = ["--local-port", "5070", "--rtp-port", "41000", "--pty-link", tmp + "/b"]
    if answer_by_ata:
        b_args += ["--auto-answer", "0"]
    pb = modem(b_args, tmp + "/b.log")
    pa = modem(["--local-port", "5080", "--rtp-port", "42000", "--sip-server", "127.0.0.1:5070",
                "--pty-link", tmp + "/a"], tmp + "/a.log")
    try:
        a, b = open_pty(tmp + "/a"), open_pty(tmp + "/b")
        bufs = {a: b"", b: b""}
        time.sleep(1.0)
        os.write(b, b"ATE0+VCID=1\r")
        os.write(a, b"ATE0\r")
        pump(bufs, 1.0)
        bufs[a] = bufs[b] = b""
        os.write(a, b"ATD6004\r")
        if answer_by_ata:
            check(pump(bufs, 15, lambda x: x[b].count(b"RING") >= 2), "the answerer rings")
            check(b"CONNECT" not in bufs[b], "S0=0: it does not answer by itself")
            time.sleep(0.2)
            os.write(b, b"ATA\r")
        ok = pump(bufs, 45, lambda x: b"CONNECT" in x[a] and b"CONNECT" in x[b])
        check(ok, f"CONNECT on both sides (answered by {name})")
        text = bufs[b].decode(errors="replace")
        check("NMBR = modem" in text and "DATE = " in text and text.find("RING") < text.find("DATE"),
              "caller ID after the first RING (V.253 9.2.3.1)")
        if not answer_by_ata:
            check(text.count("RING") == 2, "S0=2: answered on the second ring")
        time.sleep(1.2)
        os.write(a, b"+++")
        time.sleep(1.2)
        os.write(a, b"ATI6\r")
        pump(bufs, 2)
        check(b"Originate, in progress" in bufs[a], "ATI6 mid-call on the caller")
        os.write(a, b"ATH\r")
        check(pump(bufs, 8, lambda x: b"NO CARRIER" in x[b]), "the answerer reports NO CARRIER after ATH")
    finally:
        pa.terminate()
        pb.terminate()
        pa.wait()
        pb.wait()


def dead(tmp):
    print("dial with nothing answering:")
    pa = modem(["--local-port", "5080", "--rtp-port", "42000", "--sip-server", "127.0.0.1:5999",
                "--pty-link", tmp + "/a"], tmp + "/dead.log")
    try:
        a = open_pty(tmp + "/a")
        bufs = {a: b""}
        time.sleep(1.0)
        os.write(a, b"ATE0\r")
        pump(bufs, 0.5)
        bufs[a] = b""
        os.write(a, b"ATD6004\r")
        ok = pump(bufs, 45, lambda x: b"NO CARRIER" in x[a] or b"NO DIALTONE" in x[a])
        check(ok, "a final result code (it used to be silence)")
        bufs[a] = b""
        os.write(a, b"ATI6\r")
        pump(bufs, 1)
        check(b"failed before data mode" in bufs[a] and b"SIP 408" in bufs[a], "ATI6 names the SIP failure")
        bufs[a] = b""
        os.write(a, b"ATD6004\r")
        time.sleep(1.0)
        log = open(tmp + "/dead.log").read()
        check(log.count("state: CALLING") >= 2, "a second ATD places a call (the engine is not wedged)")
    finally:
        pa.terminate()
        pa.wait()


def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)) + "/..")
    with tempfile.TemporaryDirectory() as tmp:
        call(tmp, False)
        call(tmp, True)
        dead(tmp)
    print("PASSED" if failures == 0 else f"FAILED ({failures})")
    return failures


if __name__ == "__main__":
    sys.exit(main())
