#!/usr/bin/env python3
"""Dial from an AT modem (the CX93001), hold the call and send numbered lines.

    cx_hold_dial.py <tty> <number> <hold-seconds> [AT setup ...]

ATZ, the setup commands (e.g. 'AT\\N2' 'AT+MS=V22B', or 'AT+ES=4,4' for
MNP only), ATDT<number>; after CONNECT one line a second for 20 s, then
+++ATH and AT&V1 (the CX's last-call report: protocol, compression,
termination reason).  Everything the modem says is echoed and kept in
cx_hold_dial.out.  The partner of tools/soak/answer_ec_call.sh.
"""
import sys
import time


def dial_and_hold(port, number, hold, setup, read, command):
    """Run one call; finish in command mode and retain its last-call report."""
    dial_attempted = False
    try:
        command('ATZ', 3)
        for c in ['ATE1V1Q0X3', 'ATW2'] + setup:
            command(c)
        dial_attempted = True
        port.write(('ATDT%s\r' % number).encode())
        buf, started = b'', time.monotonic()
        while time.monotonic() - started < 90:
            buf += read(0.5)
            if any(result in buf for result in
                   (b'CONNECT', b'NO CARRIER', b'BUSY', b'NO DIAL')):
                break
        if b'CONNECT' not in buf:
            return 1
        started, n = time.monotonic(), 0
        while time.monotonic() - started < hold:
            if n < 20 and time.monotonic() - started > 3 + n:
                n += 1
                port.write(b'cx %02d pack my box with five dozen liquor jugs 9876543210\r\n' % n)
            buf += read(0.5)
            if b'NO CARRIER' in buf[-200:]:
                break
        return 0
    finally:
        if dial_attempted:
            # A timeout may leave the modem in data mode. Preserve the guard
            # interval before escaping, then hang up on every result path.
            time.sleep(1)
            port.write(b'+++')
            read(2)
            # A failed dial may already be in command mode, where +++ is
            # an unfinished command. Terminate it before sending ATH.
            port.write(b'\r')
            command('ATH', 3)
            command('AT&V1', 3)


def main():
    import serial

    if len(sys.argv) < 4:
        print(__doc__, file=sys.stderr)
        return 2
    dev, number, hold = sys.argv[1], sys.argv[2], float(sys.argv[3])
    setup = sys.argv[4:]
    with serial.Serial(dev, 115200, timeout=0.1, rtscts=True) as port, \
            open('cx_hold_dial.out', 'wb') as log:
        def read(duration):
            end, buf = time.monotonic() + duration, b''
            while time.monotonic() < end:
                data = port.read(4096)
                if data:
                    buf += data
                    log.write(data)
                    log.flush()
                    sys.stdout.write(data.decode(errors='replace'))
                    sys.stdout.flush()
            return buf

        def command(text, duration=2.0):
            port.write((text + '\r').encode())
            return read(duration)

        return dial_and_hold(port, number, hold, setup, read, command)


if __name__ == '__main__':
    raise SystemExit(main())
