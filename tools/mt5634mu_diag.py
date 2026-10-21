#!/usr/bin/env python3
"""One bounded MT5634MU call through its bridge; stdlib only, no reset/NVRAM.

Run on an idle bridge PTY. Capture ATI11 before any reset; consume its pager
before sending another command. No USB requests or audio processing here.
"""
import argparse
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import select
import termios
import time
import tty


RESULT = re.compile(rb"(?:^|[\r\n])(OK|ERROR|CONNECT[^\r\n]*|NO CARRIER|NO DIAL ?TONE|BUSY|NO ANSWER)[\r\n]")
PAGER = re.compile(rb"press any key", re.I)


def analyze_cdr(path, number):
    """Asterisk Master.csv: group legs by uniqueid, never trust ANSWERED alone."""
    with path.open(newline='') as stream:
        rows = [row for row in csv.reader(stream)
                if len(row) >= 17 and row[1] == '6004' and row[2] == number]
    if not rows:
        raise RuntimeError(f'no 6004 -> {number} records in CDR snapshot')
    call_id = rows[-1][16]
    legs = [dict(application=r[7], target=r[8], disposition=r[14],
                 start=r[9], answer=r[10], end=r[11])
            for r in rows if r[16] == call_id]
    dial = [leg for leg in legs if leg['application'] == 'Dial']
    voicemail = any(leg['application'] == 'VoiceMail' for leg in legs)
    result = 'unresolved'
    if voicemail:
        result = 'voicemail_after_' + (dial[-1]['disposition'].lower() if dial else 'unknown')
    elif dial and dial[-1]['disposition'] == 'ANSWERED':
        result = 'modem_route_answered'  # SIP answer alone does not prove training.
    elif dial:
        result = 'modem_route_' + dial[-1]['disposition'].lower()
    return dict(number=number, source='6004', call_id=call_id,
                route_result=result, legs=legs)


class Port:
    def __init__(self, fd, log):
        self.fd, self.log = fd, log
        self.start = time.monotonic()

    def record(self, direction, data):
        self.log.write(json.dumps(dict(t=round(time.monotonic() - self.start, 6),
                                       direction=direction, hex=data.hex(),
                                       text=data.decode('ascii', errors='replace'))) + '\n')
        self.log.flush()

    def write(self, data):
        self.record('tx', data)
        view = memoryview(data)
        while view:
            view = view[os.write(self.fd, view):]
        termios.tcdrain(self.fd)

    def read(self, seconds, terminal=True, pager=False):
        deadline = time.monotonic() + seconds
        buf, pages = b'', 0
        while time.monotonic() < deadline:
            if not select.select([self.fd], [], [], max(0, deadline - time.monotonic()))[0]:
                break
            chunk = os.read(self.fd, 4096)
            if not chunk:
                raise RuntimeError('bridge PTY closed')
            self.record('rx', chunk)
            buf += chunk
            if pager:
                count = len(PAGER.findall(buf))
                while pages < count:
                    if pages >= 8:
                        raise RuntimeError('modem pager exceeded eight pages')
                    self.write(b' ')
                    pages += 1
            match = RESULT.search(buf)
            if terminal and match:
                return buf, match.group(1).decode('ascii')
        return buf, None

    def command(self, command, pager=False):
        self.write(command.encode('ascii') + b'\r')
        data, result = self.read(5, pager=pager)
        if result != 'OK':
            raise RuntimeError(f'{command}: expected OK, got {result!r}')
        return data.decode('ascii', errors='replace')


def diagnose(port, number, wait, hold):
    summary = dict(number=number, identity={}, result=None)
    port.command('AT')
    port.command('ATE1V1Q0')
    port.command('ATM0')  # V.250 (07/2003) 6.3.14, Table 12: speaker off.
    for cmd in ('ATI3', 'AT+GMR'):
        summary['identity'][cmd] = port.command(cmd)
    if 'MT5634MU' not in summary['identity']['ATI3']:
        raise RuntimeError('not an MT5634MU; refusing to dial')
    # Read active profile, including modulation/error-control policy. No ATZ,
    # &F, &W or undocumented register writes: keep the failing setup intact.
    summary['profile'] = port.command('AT&V', pager=True)
    start = time.monotonic()
    port.write(f'ATDT{number}\r'.encode('ascii'))
    dial_data, result = port.read(wait)
    summary.update(result=result or 'TIMEOUT', dial_seconds=round(time.monotonic() - start, 3))
    if result and result.startswith('CONNECT'):
        data, _ = port.read(hold, terminal=False)
        summary['data_preview'] = (dial_data + data).decode('ascii', errors='replace')
        # This modem's documented escape guard. PTY close is not DTR; hang up while
        # still open so ATI11 describes this call, not a subsequent reset.
        time.sleep(1.1)
        port.write(b'+++')
        time.sleep(1.1)
        _, escaped = port.read(2)
        if escaped not in ('OK', 'NO CARRIER'):
            raise RuntimeError('escape unconfirmed; refusing to send ATH into DATA')
        port.command('ATH')
    elif result is None:
        # A close with carrier OFF does not cancel training in the bridge.
        # V.250 (07/2003) 5.6.1 / 6.3.1: a character requests dial abort;
        # require confirmation before ATH (abort may fail during training).
        port.write(b'\r')
        _, aborted = port.read(5)
        if aborted not in ('OK', 'NO CARRIER'):
            raise RuntimeError('dial abort unconfirmed; inspect modem before another call')
        port.command('ATH')
    summary['ati11'] = port.command('ATI11', pager=True)
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--dev', default='/tmp/mt5634mu')
    ap.add_argument('--number', required=True, choices=('6311', '6312'))
    ap.add_argument('--out', required=True, type=Path, help='new output directory')
    ap.add_argument('--wait', type=float, default=80)
    ap.add_argument('--hold', type=float, default=5)
    ap.add_argument('--cdr-csv', type=Path,
                    help='analyze latest 6004 call in an Asterisk Master.csv snapshot; do not dial')
    args = ap.parse_args()
    if args.wait <= 0 or args.hold < 0:
        ap.error('wait must be positive and hold nonnegative')
    args.out.mkdir(parents=True, exist_ok=False)
    summary = dict(number=args.number, dev=args.dev,
                   started_utc=datetime.now(timezone.utc).isoformat())
    fd = None
    try:
        if args.cdr_csv:
            summary.update(analyze_cdr(args.cdr_csv, args.number))
        else:
            with (args.out / 'transcript.jsonl').open('w') as log:
                fd = os.open(args.dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                os.set_blocking(fd, True)
                tty.setraw(fd)
                summary.update(diagnose(Port(fd, log), args.number, args.wait, args.hold))
    except (OSError, RuntimeError, KeyboardInterrupt) as exc:
        summary['error'] = str(exc) or 'interrupted; inspect modem before redialling'
    finally:
        if fd is not None:
            os.close(fd)
        (args.out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return int('error' in summary or summary.get('result') == 'TIMEOUT')


if __name__ == '__main__':
    raise SystemExit(main())
