#!/usr/bin/env python3
"""Pass 160-octet G.711 blocks between mica_trace and a peer, recording both ways."""
import os, subprocess, sys
up_path, down_path, argv = sys.argv[1], sys.argv[2], sys.argv[3:]
child = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
up, down = open(up_path, 'wb', buffering=0), open(down_path, 'wb', buffering=0)
def full(f, n):
    b = b''
    while len(b) < n:
        k = f.read(n - len(b))
        if not k: return None
        b += k
    return b
stdin, stdout = sys.stdin.buffer.raw, sys.stdout.buffer.raw
GAIN = float(os.environ.get('TEE_UP_GAIN', '1'))
def ulaw_dec(b):
    b = ~b & 0xff; e = (b >> 4) & 7; m = b & 15
    v = (((m << 3) + 0x84) << e) - 0x84
    return -v if b & 0x80 else v
def ulaw_enc(x):
    x = max(-32124, min(32124, int(round(x))))
    sign = 0x80 if x < 0 else 0
    x = min(abs(x) + 0x84, 0x7fff)
    e = 7
    while e > 0 and not (x & (0x4000 >> (7 - e))): e -= 1
    m = (x >> (e + 3)) & 15
    return ~(sign | (e << 4) | m) & 0xff
DEC = [ulaw_dec(i) for i in range(256)]
assert all(ulaw_enc(DEC[i]) == i or DEC[i] == 0 for i in range(256))
def scale(block):
    return bytes(ulaw_enc(DEC[c] * GAIN) for c in block) if GAIN != 1 else block
while True:
    block = full(stdin, 160)
    if block is None: break
    down.write(block)
    try:
        child.stdin.write(block)
    except BrokenPipeError:
        break
    reply = full(child.stdout, 160)
    if reply is None: break
    reply = scale(reply)
    up.write(reply)
    stdout.write(reply)
child.stdin.close(); child.wait()
