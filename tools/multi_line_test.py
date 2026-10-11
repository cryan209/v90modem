#!/usr/bin/env python3
"""Multi-line server test: `sip_v90_modem --lines N` over local SIP.

Starts one supervisor with N lines (answering, no registrar: each worker
listens on its own port) and N ordinary caller instances, each dialling a
different line, all at once.  Every line's console echoes what it receives
after CONNECT; every caller sends its own payload and must get it back
byte-exact while all N calls are up together.  With --recycle it then
checks that each line got a fresh worker process and runs a second round.
Finally the supervisor is stopped and no worker may survive it.

  tools/multi_line_test.py [--lines N] [--mode v22] [--recycle] [--rounds R]
"""
import argparse
import os
import re
import signal
import socket
import subprocess
import sys
import tempfile
import termios
import threading
import time
import tty
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BINARY = str(ROOT / 'sip_v90_modem')


def free_port_block(n, step=1):
    """A base port with base, base+step, ... base+(n-1)*step free for UDP."""
    for _ in range(200):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(('127.0.0.1', 0))
            base = s.getsockname()[1]
        if base + n * step >= 65000:
            continue
        socks = []
        try:
            for i in range(n):
                t = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                socks.append(t)
                t.bind(('127.0.0.1', base + i * step))
            return base
        except OSError:
            pass
        finally:
            for t in socks:
                t.close()
    raise SystemExit('no free port block')


def open_pty(path, timeout=20):
    deadline = time.time() + timeout
    while not os.path.exists(path):
        if time.time() > deadline:
            raise SystemExit(f'{path} never appeared')
        time.sleep(0.1)
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[3] &= ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def read_some(fd):
    try:
        return os.read(fd, 4096)
    except BlockingIOError:
        return b''
    except OSError:
        return b''


class Pump(threading.Thread):
    """Collect a process's output so its pipe never fills."""

    def __init__(self, proc):
        super().__init__(daemon=True)
        self.proc, self.lines = proc, []

    def run(self):
        for raw in self.proc.stdout:
            self.lines.append(raw.decode(errors='replace'))

    def text(self):
        return ''.join(self.lines)


def line_echo(path, stop, connected, idx):
    """A line's DTE: after CONNECT, echo every byte back."""
    fd = None
    seen = b''
    while not stop.is_set():
        if fd is None:
            try:
                fd = open_pty(path, timeout=1)
            except SystemExit:
                continue
        data = read_some(fd)
        if not data:
            if not os.path.exists(path):      # recycled worker: new PTY
                os.close(fd)
                fd, seen = None, b''
            time.sleep(0.01)
            continue
        if b'CONNECT' not in seen:
            seen += data
            if b'CONNECT' in seen:
                connected[idx] = time.time()
                tail = seen.split(b'CONNECT', 1)[1]
                tail = tail.split(b'\n', 1)[1] if b'\n' in tail else b''
                if tail:
                    os.write(fd, tail)
            continue
        if b'NO CARRIER' in data:
            seen = b''
            continue
        os.write(fd, data)


def one_call(i, args, tmp, line_port, rtp_base, results):
    """Caller i: dial line i, send a payload, wait for the echo."""
    port = free_port_block(1)
    pty = str(tmp / f'call.{i}')
    proc = subprocess.Popen(
        [BINARY, '--bind-addr', '127.0.0.1', '--local-port', str(port),
         '--rtp-port', str(rtp_base + 100 * i),
         '--sip-server', f'127.0.0.1:{line_port + i}', '--mode', args.mode,
         '--auto-answer', '0', '--pty-link', pty],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        env=dict(os.environ, ME_DATA_FRAMING='v14', **caller_env(args),
                 **({'ME_IO_SCHEDULE': str(tmp / f'call.{i}.sched')} if args.io_schedule else {})))
    pump = Pump(proc)
    pump.start()
    r = {'ok': False, 'why': '', 'connect': None, 'done': None}
    results[i] = r
    try:
        fd = open_pty(pty)
        os.write(fd, b'ATD1234\r')
        got = b''
        deadline = time.time() + args.connect_timeout
        while b'CONNECT' not in got and time.time() < deadline:
            got += read_some(fd)
            if b'NO CARRIER' in got:
                break
            time.sleep(0.02)
        if b'CONNECT' not in got:
            r['why'] = 'no CONNECT: ' + got.decode(errors='replace')[-120:]
            return
        r['connect'] = time.time()
        m = re.search(rb'CONNECT (\d+)', got)
        r['rate'] = int(m.group(1)) if m else 0
        time.sleep(1.0)                       # let both ends settle
        payload = (f'line {i + args.payload_offset} '.encode() + bytes(range(32, 127)) + b'\r\n') * 4
        os.write(fd, payload)
        echo = b''
        deadline = time.time() + args.echo_timeout
        while payload not in echo and time.time() < deadline:
            echo += read_some(fd)
            time.sleep(0.02)
        if payload not in echo:
            r['why'] = f'echo incomplete: {len(echo)} of {len(payload)} bytes'
            (tmp / f'echo.{i}.bin').write_bytes(echo)
            (tmp / f'sent.{i}.bin').write_bytes(payload)
            return
        r['done'] = time.time()
        r['ok'] = True
    finally:
        try:
            proc.send_signal(signal.SIGTERM)
            proc.wait(10)
        except subprocess.TimeoutExpired:
            proc.kill()
        r['log'] = pump.text()


def caller_env(args):
    # V.90/V.92 have an analogue and a digital end: the lines are the
    # digital (server) side, so the callers take the analogue role.
    return {'ME_V90_ROLE': 'analogue'} if args.mode in ('v90', 'v92') else {}


def cpu_seconds(pid):
    out = subprocess.run(['ps', '-o', 'time=', '-p', str(pid)],
                         capture_output=True, text=True).stdout.strip()
    if not out:
        return None
    parts = [float(x) for x in out.replace('-', ':').split(':')]
    total = 0.0
    for x in parts:
        total = total * 60 + x
    return total


def schedule_summary(path):
    """Longest gap between successive media calls, and the RX/TX sample balance."""
    import struct
    data = path.read_bytes()
    recs = [struct.unpack_from('<B3xiQ', data, k) for k in range(0, len(data) - 15, 16)]
    if len(recs) < 2:
        return 'empty'
    worst, at = 0, 0
    for a, b in zip(recs, recs[1:]):
        if b[2] - a[2] > worst:
            worst, at = b[2] - a[2], (a[2] - recs[0][2]) / 1e9
    rx = sum(r[1] for r in recs if r[0] == ord('R'))
    tx = sum(r[1] for r in recs if r[0] == ord('T'))
    return f'longest gap {worst / 1e6:.0f} ms at {at:.1f} s; RX {rx} TX {tx} samples'


def worker_pids(text):
    return re.findall(r'line (\d+): worker pid (\d+) started', text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--lines', type=int, default=3)
    ap.add_argument('--mode', default='v22')
    ap.add_argument('--recycle', action='store_true')
    ap.add_argument('--rounds', type=int, default=1)
    ap.add_argument('--payload-offset', type=int, default=0,
                    help='number the payloads from this instead of 0')
    ap.add_argument('--io-schedule', action='store_true',
                    help='record every process\'s RX/TX timing (ME_IO_SCHEDULE) and report its longest gap')
    ap.add_argument('--connect-timeout', type=float, default=60)
    ap.add_argument('--echo-timeout', type=float, default=30)
    args = ap.parse_args()
    if args.recycle and args.rounds < 2:
        args.rounds = 2

    tmp = Path(tempfile.mkdtemp(prefix='multi-line-'))
    line_port = free_port_block(args.lines)
    line_rtp = free_port_block(args.lines, 100)
    call_rtp = free_port_block(args.lines, 100)
    sup_args = [BINARY, '--lines', str(args.lines), '--bind-addr', '127.0.0.1',
                '--local-port', str(line_port), '--rtp-port', str(line_rtp),
                '--auto-answer', '1', '--mode', args.mode,
                '--pty-link', str(tmp / 'line')]
    if args.recycle:
        sup_args.append('--recycle')
    line_env = {'ME_DATA_FRAMING': 'v14'}
    if args.mode in ('v90', 'v92'):
        # Our own analogue caller against our digital lines: the strict Ja
        # handling the GUI's V.90 loopback fixture also uses.
        line_env['ME_V90_JA_HEURISTIC_FALLBACK_MS'] = '0'
    if args.io_schedule:
        line_env['ME_IO_SCHEDULE'] = str(tmp / 'line.%L.sched')
    sup = subprocess.Popen(sup_args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           env=dict(os.environ, **line_env))
    sup_out = Pump(sup)
    sup_out.start()
    stop = threading.Event()
    connected = {}
    echoes = [threading.Thread(target=line_echo, args=(str(tmp / f'line.{i}'), stop, connected, i),
                               daemon=True) for i in range(args.lines)]
    for t in echoes:
        t.start()
    failures = 0
    try:
        for i in range(args.lines):
            open_pty(str(tmp / f'line.{i}'))
        for rnd in range(args.rounds):
            results = {}
            callers = [threading.Thread(target=one_call,
                                        args=(i, args, tmp, line_port, call_rtp, results))
                       for i in range(args.lines)]
            for t in callers:
                t.start()
            for t in callers:
                t.join()
            ok = [results[i]['ok'] for i in range(args.lines)]
            for i in range(args.lines):
                r = results[i]
                state = 'ok' if r['ok'] else 'FAIL ' + r['why']
                rate = r.get('rate', 0)
                print(f'round {rnd} line {i}: CONNECT {rate}, {state}')
                if not r['ok']:
                    failures += 1
                    (tmp / f'caller.{i}.round{rnd}.log').write_text(r.get('log', ''))
                    print(f'  caller log: {tmp}/caller.{i}.round{rnd}.log')
            live = {}
            for idx, pid in worker_pids(sup_out.text()):
                live[int(idx)] = int(pid)
            cpu = [cpu_seconds(live[i]) for i in sorted(live)]
            if all(c is not None for c in cpu):
                print(f'round {rnd}: worker CPU so far (s): ' + ', '.join(f'{c:.1f}' for c in cpu))
            if all(ok):
                # All N calls were up at the same time: the latest CONNECT
                # came before the earliest echo completed.
                last_connect = max(results[i]['connect'] for i in range(args.lines))
                first_done = min(results[i]['done'] for i in range(args.lines))
                overlap = first_done - last_connect
                print(f'round {rnd}: all {args.lines} calls up together for {overlap:.1f} s')
                if overlap <= 0:
                    failures += 1
                    print('  FAIL: the calls did not overlap')
            if args.recycle:
                # Every line's worker must be replaced after its call.
                deadline = time.time() + 20
                want = (rnd + 2) * args.lines
                while len(worker_pids(sup_out.text())) < want and time.time() < deadline:
                    time.sleep(0.2)
                starts = worker_pids(sup_out.text())
                if len(starts) < want:
                    failures += 1
                    print(f'  FAIL: {len(starts)} worker starts, wanted {want}')
                else:
                    print(f'round {rnd}: every line recycled into a fresh worker')
                for i in range(args.lines):
                    open_pty(str(tmp / f'line.{i}'))
    finally:
        stop.set()
        pids = [int(p) for _, p in worker_pids(sup_out.text())]
        sup.send_signal(signal.SIGTERM)
        try:
            sup.wait(15)
        except subprocess.TimeoutExpired:
            sup.kill()
            failures += 1
            print('FAIL: supervisor did not stop')
        time.sleep(0.5)
        alive = [p for p in pids if subprocess.run(['kill', '-0', str(p)],
                                                   capture_output=True).returncode == 0]
        if alive:
            failures += 1
            print(f'FAIL: workers still running after the supervisor: {alive}')
            for p in alive:
                os.kill(p, signal.SIGKILL)
        (tmp / 'supervisor.log').write_text(sup_out.text())
        if failures:
            print(f'supervisor log (all lines, prefixed [Li]): {tmp / "supervisor.log"}')
    if args.io_schedule:
        for f in sorted(tmp.glob('*.sched')):
            print(f'{f.name}: {schedule_summary(f)}')
    print('multi_line_test: ' + ('FAILED' if failures else
          f'OK ({args.lines} lines, {args.rounds} round(s){", recycled" if args.recycle else ""})'))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
