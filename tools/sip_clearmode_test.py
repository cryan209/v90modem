#!/usr/bin/env python3
"""Two sip_v90_modem instances over SIP on 127.0.0.1 in V.120: the SDP must
offer and select RFC 4040 CLEARMODE/8000 when ME_CLEARMODE=1, PCMU otherwise,
and the DTE text must cross byte-exact either way (the octets are the same;
only the codec name on the wire changes).

    make sip-clearmode-test
"""
import os
import re
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sip_loop_test import open_pty, pump, check  # noqa: E402
import sip_loop_test  # noqa: E402


def run(tmp, clearmode, tag):
    env = dict(os.environ, ME_MODE="v22", ME_CLEARMODE="1" if clearmode else "0")
    def modem(args, log):
        return subprocess.Popen(["./sip_v90_modem", "--bind-addr", "127.0.0.1"] + args, env=env,
                                stdout=open(log, "w"), stderr=subprocess.STDOUT)
    pb = modem(["--local-port", "5070", "--rtp-port", "41000", "--pty-link", tmp + "/b",
                "--auto-answer", "1"], tmp + f"/b-{tag}.log")
    pa = modem(["--local-port", "5080", "--rtp-port", "42000", "--sip-server", "127.0.0.1:5070",
                "--pty-link", tmp + "/a"], tmp + f"/a-{tag}.log")
    try:
        a, b = open_pty(tmp + "/a"), open_pty(tmp + "/b")
        bufs = {a: b"", b: b""}
        time.sleep(1.0)
        for fd in (a, b):
            os.write(fd, b"ATE0\r")
            pump(bufs, 0.5)
            os.write(fd, b"AT+MS=V120\r")
            pump(bufs, 0.5)
        check(bufs[a].count(b"OK") >= 2 and bufs[b].count(b"OK") >= 2, f"[{tag}] AT+MS=V120 on both")
        bufs[a] = bufs[b] = b""
        os.write(a, b"ATD6004\r")
        ok = pump(bufs, 30, lambda x: b"CONNECT 64000" in x[a] and b"CONNECT 64000" in x[b])
        check(ok, f"[{tag}] CONNECT 64000 on both sides")
        time.sleep(0.5)
        bufs[a] = bufs[b] = b""
        lines = b"".join(b"A%04d the quick brown fox\r\n" % i for i in range(200))
        sent = 0
        t0 = time.time()
        while sent < len(lines) and time.time() - t0 < 20:
            try:
                sent += os.write(a, lines[sent:sent + 512])
            except BlockingIOError:
                pass
            pump(bufs, 0.1)
        pump(bufs, 10, lambda x: x[b].count(b"\r\n") >= 200)
        check(bufs[b] == lines, f"[{tag}] 200 numbered lines byte-exact through the call")
    finally:
        pa.terminate()
        pb.terminate()
        pa.wait()
        pb.wait()
    la = open(tmp + f"/a-{tag}.log", errors="replace").read()
    lb = open(tmp + f"/b-{tag}.log", errors="replace").read()
    return la, lb


def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)) + "/..")
    with tempfile.TemporaryDirectory() as tmp:
        la, lb = run(tmp, True, "clearmode")
        for who, log in (("caller", la), ("answerer", lb)):
            check("CLEARMODE" in re.findall(r"a=rtpmap:\d+ (\w+)/8000", log) or "Codec: CLEARMODE" in log,
                  f"[clearmode] {who} negotiated CLEARMODE")
        la, lb = run(tmp, False, "pcmu")
        for who, log in (("caller", la), ("answerer", lb)):
            check("Codec: PCMU" in log and "CLEARMODE" not in log.replace("Codec: CLEARMODE", "X") + "",
                  f"[pcmu] {who} stays on PCMU with the knob off")
    print("PASSED" if sip_loop_test.failures == 0 else f"FAILED ({sip_loop_test.failures})")
    return sip_loop_test.failures


if __name__ == "__main__":
    sys.exit(main())
