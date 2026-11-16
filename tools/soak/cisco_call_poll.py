#!/usr/bin/env python3
"""Log the TopRouter Cisco's per-call voice counters while a live batch runs.

Logs in once over telnet (`login local`; the operator types the username and
password at a getpass prompt, they are never stored or echoed), then every
--interval seconds runs `show call active voice brief` and appends the output,
stamped with host wall-clock time, to --out.  The VoIP leg of each active call
carries the router's own receive counters for our RTP (lost/early/late
packets, playout delay), which our taps cannot see: they show what reached
the FXS port that feeds the analogue modem.

    python3 tools/soak/cisco_call_poll.py --out artifacts/cisco-poll.log

Stop with Ctrl-C.  Host 192.168.88.62 (see rig/v90ras/README.md).
"""

import argparse
import os
import getpass
import socket
import sys
import time

IAC, DONT, DO, WONT, WILL, SB, SE = 255, 254, 253, 252, 251, 250, 240


class Telnet:
    def __init__(self, host, port=23):
        self.s = socket.create_connection((host, port), timeout=15)
        self.buf = b""

    def _recv(self):
        data = self.s.recv(4096)
        if not data:
            raise EOFError("connection closed")
        out, i = bytearray(), 0
        while i < len(data):
            b = data[i]
            if b == IAC and i + 1 < len(data):
                cmd = data[i + 1]
                if cmd in (DO, DONT, WILL, WONT) and i + 2 < len(data):
                    opt = data[i + 2]
                    reply = WONT if cmd in (DO, DONT) else DONT
                    self.s.sendall(bytes([IAC, reply, opt]))
                    i += 3
                    continue
                if cmd == SB:
                    j = data.find(bytes([IAC, SE]), i)
                    i = len(data) if j < 0 else j + 2
                    continue
                i += 2
                continue
            out.append(b)
            i += 1
        self.buf += bytes(out)

    def read_until(self, tokens, timeout=20):
        end = time.time() + timeout
        while True:
            for t in tokens:
                k = self.buf.find(t)
                if k >= 0:
                    got, self.buf = self.buf[:k + len(t)], self.buf[k + len(t):]
                    return got
            if time.time() > end:
                raise TimeoutError(f"waiting for {tokens!r}; have {self.buf[-200:]!r}")
            self.s.settimeout(max(0.1, end - time.time()))
            self._recv()

    def send(self, line):
        self.s.sendall(line.encode() + b"\r\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.88.62")
    ap.add_argument("--out", required=True)
    ap.add_argument("--interval", type=float, default=2.0)
    ap.add_argument("--cmd-file",
                    help="each poll, run the show commands in this file "
                         "(one per line; only lines starting with 'show'), "
                         "log their output, then delete the file")
    ap.add_argument("--config-file",
                    help="each poll, apply this file's lines in 'configure "
                         "terminal' (not saved), log the echo, then delete it")
    a = ap.parse_args()

    user = input("Cisco username: ")
    pw = getpass.getpass("Cisco password: ")
    t = Telnet(a.host)
    t.read_until([b"Username:"])
    t.send(user)
    t.read_until([b"Password:"])
    t.send(pw)
    pw = None
    prompt = t.read_until([b">", b"#"]).splitlines()[-1].strip()
    t.send("terminal length 0")
    t.read_until([prompt])
    print(f"logged in ({prompt.decode(errors='replace')}); polling every "
          f"{a.interval:g} s into {a.out}; Ctrl-C to stop", flush=True)
    n = 0
    with open(a.out, "a") as f:
        while True:
            stamp = time.time()
            if a.cmd_file and os.path.exists(a.cmd_file):
                with open(a.cmd_file) as cf:
                    cmds = [c.strip() for c in cf if c.strip().startswith("show ")]
                os.remove(a.cmd_file)
                for c in cmds:
                    t.send(c)
                    out = t.read_until([prompt]).decode(errors="replace")
                    f.write(f"=== {time.time():.3f} CMD {c}\n{out}\n")
                f.flush()
            if a.config_file and os.path.exists(a.config_file):
                with open(a.config_file) as cf:
                    lines = [c.strip() for c in cf if c.strip()]
                os.remove(a.config_file)
                cfg_prompt = prompt[:-1] + b"(config)#"
                t.send("configure terminal")
                log = t.read_until([cfg_prompt]).decode(errors="replace")
                for c in lines:
                    t.send(c)
                    log += t.read_until([b"#"]).decode(errors="replace")
                t.send("end")
                log += t.read_until([prompt]).decode(errors="replace")
                f.write(f"=== {time.time():.3f} CONFIG\n{log}\n")
                f.flush()
            t.send("show call active voice brief")
            out = t.read_until([prompt]).decode(errors="replace")
            f.write(f"=== {stamp:.3f} {time.strftime('%H:%M:%S', time.localtime(stamp))}\n")
            f.write(out.replace("\r", "") + "\n")
            f.flush()
            n += 1
            if n % 30 == 0:
                print(f"{n} polls", flush=True)
            time.sleep(max(0.0, a.interval - (time.time() - stamp)))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
