#!/usr/bin/env python3
"""Pass numbered lines both ways across two modem PTYs and grade the result.

Both ends of this rig are v90modem PTYs, so the same script drives them: it
waits for each side to reach data mode (the modem sends CONNECT), then writes
its own numbered pattern and counts the peer's lines that arrive intact and in
sequence.  Line counts, not byte percentages: an unlocked receiver emits
garbage that inflates a byte total.
"""
import os, re, select, sys, threading, time

def open_pty(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    return fd

class End:
    def __init__(self, path, tag):
        self.fd = open_pty(path); self.tag = tag
        self.rx = b""; self.connected = False; self.lines = []
    def poll(self):
        r,_,_ = select.select([self.fd], [], [], 0)
        if r:
            try: self.rx += os.read(self.fd, 65536)
            except OSError: pass

def main():
    a = End(sys.argv[1], sys.argv[2])
    b = End(sys.argv[3], sys.argv[4])
    secs = float(sys.argv[5]) if len(sys.argv) > 5 else 60.0
    # Seconds between 10-byte lines per direction.  At 2400 bit/s V.14 gives
    # 240 char/s, so 0.05 s offers 200 char/s and loads the link to 83%;
    # keep the default well under that so losses measure the link, not queueing.
    gap = float(sys.argv[6]) if len(sys.argv) > 6 else 0.12
    t0 = time.time(); n = {a.tag: 0, b.tag: 0}; last = 0.0
    while time.time() - t0 < secs:
        for e in (a, b):
            e.poll()
            if not e.connected and b"CONNECT" in e.rx:
                e.connected = True
                print("[%s] CONNECT at %.1fs" % (e.tag, time.time()-t0), flush=True)
                e.rx = e.rx.split(b"CONNECT", 1)[1]
        if a.connected and b.connected and time.time() - last > gap:
            last = time.time()
            for e in (a, b):
                msg = ("%s%07d\r\n" % (e.tag[0].upper(), n[e.tag])).encode()
                try: os.write(e.fd, msg); n[e.tag] += 1
                except OSError: pass
        time.sleep(0.005)
    for e in (a, b):
        e.poll()
    for e, peer in ((a, b), (b, a)):
        pat = re.compile((peer.tag[0].upper() + r"(\d{7})").encode())
        got = [int(m.group(1)) for m in pat.finditer(e.rx)]
        seq = sum(1 for i in range(1, len(got)) if got[i] == got[i-1] + 1)
        print("[%s] sent %d, received %d intact %s-lines, %d in sequence, %d bytes"
              % (e.tag, n[e.tag], len(got), peer.tag[0].upper(), seq, len(e.rx)), flush=True)

main()
