#!/usr/bin/env python3
"""Dial from an AT modem (the CX93001), hold the call and send numbered lines.

    cx_hold_dial.py <tty> <number> <hold-seconds> [AT setup ...]

ATZ, the setup commands (e.g. 'AT\\N2' 'AT+MS=V22B', or 'AT+ES=4,4' for
MNP only), ATDT<number>; after CONNECT one line a second for 20 s, then
+++ATH and AT&V1 (the CX's last-call report: protocol, compression,
termination reason).  Everything the modem says is echoed and kept in
cx_hold_dial.out.  The partner of tools/soak/answer_ec_call.sh.
"""
import sys, time, serial

dev, number, hold = sys.argv[1], sys.argv[2], float(sys.argv[3])
setup = sys.argv[4:]
p = serial.Serial(dev, 115200, timeout=0.1, rtscts=True)
log = open('cx_hold_dial.out', 'wb')

def rd(t):
    end, buf = time.time() + t, b''
    while time.time() < end:
        b = p.read(4096)
        if b:
            buf += b
            log.write(b); log.flush()
            sys.stdout.write(b.decode(errors='replace')); sys.stdout.flush()
    return buf

def cmd(c, t=2.0):
    p.write((c + '\r').encode()); return rd(t)

cmd('ATZ', 3)
for c in ['ATE1V1Q0X3', 'ATW2'] + setup:
    cmd(c)
p.write(('ATDT%s\r' % number).encode())
buf, t0 = b'', time.time()
while time.time() - t0 < 90:
    buf += rd(0.5)
    if b'CONNECT' in buf or b'NO CARRIER' in buf or b'BUSY' in buf or b'NO DIAL' in buf:
        break
if b'CONNECT' not in buf:
    sys.exit(1)
t0, n = time.time(), 0
while time.time() - t0 < hold:
    if n < 20 and time.time() - t0 > 3 + n:
        n += 1
        p.write(b'cx %02d pack my box with five dozen liquor jugs 9876543210\r\n' % n)
    buf += rd(0.5)
    if b'NO CARRIER' in buf[-200:]:
        break
time.sleep(1)
p.write(b'+++'); rd(2); cmd('ATH', 3)
cmd('AT&V1', 3)
