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
import hashlib
import os
import re
import select
import shutil
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


def freeze_binary(source, destination):
    """Use one executable image even if another build replaces the source."""
    source, destination = Path(source), Path(destination)
    for _ in range(3):
        before = hashlib.sha256(source.read_bytes()).hexdigest()
        shutil.copy2(source, destination)
        digest = hashlib.sha256(destination.read_bytes()).hexdigest()
        if before == digest == hashlib.sha256(source.read_bytes()).hexdigest():
            return digest
    raise RuntimeError('executable changed while taking its snapshot')


def free_port_block(n, step=1, width=1, avoid=(), even=False):
    """Check disjoint UDP allocations, including the adjacent RTCP ports."""
    avoid = set(avoid)
    for _ in range(200):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(('127.0.0.1', 0))
            base = s.getsockname()[1]
        if even:
            base &= ~1
        ports = [base + i * step + k for i in range(n) for k in range(width)]
        if max(ports) >= 65000 or not avoid.isdisjoint(ports):
            continue
        socks = []
        try:
            for port in ports:
                t = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                socks.append(t)
                t.bind(('127.0.0.1', port))
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


def write_all(fd, data, timeout=10):
    """PTYs are nonblocking; a short write must not silently truncate a probe."""
    pending = memoryview(data)
    deadline = time.monotonic() + timeout
    while pending:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('PTY write timed out')
        if not select.select([], [fd], [], remaining)[1]:
            continue
        try:
            count = os.write(fd, pending)
        except BlockingIOError:
            continue
        if count <= 0:
            raise OSError('PTY write returned no bytes')
        pending = pending[count:]


def line_echo(path, stop, connected, idx):
    """A line's DTE: after CONNECT, echo every byte back."""
    fd = None
    seen = b''
    try:
        while not stop.is_set():
            if fd is None:
                try:
                    fd = open_pty(path, timeout=1)
                except (SystemExit, OSError):
                    continue
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                time.sleep(0.01)
                continue
            except OSError:
                data = b''
            if not data:
                # EOF/EIO means the old master is gone even when a recycled
                # worker has already replaced the symlink. Do not keep the
                # dead descriptor just because that pathname exists again.
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
                        write_all(fd, tail)
                continue
            if b'NO CARRIER' in data:
                seen = b''
                continue
            write_all(fd, data)
    finally:
        if fd is not None:
            os.close(fd)


def one_call(i, args, tmp, line_port, rtp_base, results, barrier, caller_sip_base=None,
             cancel=None):
    """Caller i: dial line i, send a payload, wait for the echo."""
    r = {'ok': False, 'why': '', 'connect': None, 'done': None, 'bytes_echoed': 0}
    results[i] = r
    fd = proc = pump = None

    def check_cancelled():
        if cancel is not None and cancel.is_set():
            r['cancelled'] = True
            raise InterruptedError('original caller stopped for intentional worker-crash test')

    try:
        port = caller_sip_base + i if caller_sip_base is not None else free_port_block(1)
        pty = str(tmp / f'call.{i}')
        dumps = tmp / f'caller.{i}.dumps'
        dumps.mkdir(exist_ok=True)
        proc = subprocess.Popen(
            [BINARY, '--bind-addr', '127.0.0.1', '--local-port', str(port),
             '--rtp-port', str(rtp_base + 100 * i),
             '--sip-server', f'127.0.0.1:{line_port + i}', '--mode', args.mode,
             '--auto-answer', '0', '--pty-link', pty],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=dict(os.environ, ME_DATA_FRAMING='v14', ME_DUMP_DIR=str(dumps),
                     **caller_env(args),
                     **({'VPCM_G711_TAP_DIR': str(dumps / 'g711')}
                        if os.environ.get('VPCM_G711_TAP_DIR') else {}),
                     **({'ME_IO_SCHEDULE': str(tmp / f'call.{i}.sched')} if args.io_schedule else {})))
        pump = Pump(proc)
        pump.start()
        fd = open_pty(pty)
        write_all(fd, b'ATD1234\r')
        got = b''
        deadline = time.time() + args.connect_timeout
        while b'CONNECT' not in got and time.time() < deadline:
            check_cancelled()
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
        # Do not let an early caller finish before the slowest has trained.
        barrier.wait(timeout=args.connect_timeout)
        for block in range(args.payload_rounds):
            check_cancelled()
            label = f'line {i + args.payload_offset} '
            if block:
                label += f'block {block} '
            payload = (label.encode() + bytes(range(32, 127)) + b'\r\n') * 4
            write_all(fd, payload, args.echo_timeout)
            echo = b''
            deadline = time.monotonic() + args.echo_timeout
            while len(echo) < len(payload) and time.monotonic() < deadline:
                check_cancelled()
                echo += read_some(fd)
                if proc.poll() is not None:
                    break
                time.sleep(0.02)
            if echo != payload:
                r['why'] = f'block {block}: echo mismatch: {len(echo)} of {len(payload)} bytes'
                (tmp / f'echo.{i}.block{block}.bin').write_bytes(echo)
                (tmp / f'sent.{i}.block{block}.bin').write_bytes(payload)
                return
            r['bytes_echoed'] += len(payload)
        r['done'] = time.time()
        r['ok'] = True
        try:
            barrier.wait(timeout=args.echo_timeout)
        except threading.BrokenBarrierError:
            pass  # Keep this channel's exact result if a different one failed.
    except (OSError, TimeoutError, SystemExit, threading.BrokenBarrierError) as exc:
        r['why'] = str(exc) or type(exc).__name__
    finally:
        if not r['ok']:
            barrier.abort()
        if fd is not None:
            os.close(fd)
        if proc is not None:
            try:
                proc.send_signal(signal.SIGTERM)
                proc.wait(10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(5)
        if pump is not None:
            pump.join(timeout=2)
            r['log'] = pump.text()


def restart_active_line(line, args, tmp, results, callers, cancellations,
                        sup_out, line_port, call_rtp, call_port):
    """Kill one owned active worker, preserve peers, and redial the fresh line.

    The original call is intentionally destroyed, never graded as a pass.
    Other calls must keep their original caller and complete exact payloads.
    """
    deadline = time.monotonic() + args.connect_timeout + args.echo_timeout
    while not results.get(line, {}).get('bytes_echoed'):
        if time.monotonic() >= deadline or not callers[line].is_alive():
            raise RuntimeError('fault target did not reach exact payload')
        time.sleep(.02)
    if results[line].get('done') is not None:
        raise RuntimeError('fault target finished before injection; use more payload rounds')
    before = dict(worker_pids(sup_out.text()))
    old_pid = int(before[str(line)])
    print(f'fault: killing owned worker line {line} pid {old_pid} during payload')
    os.kill(old_pid, signal.SIGKILL)
    cancellations[line].set()
    callers[line].join(timeout=15)
    if callers[line].is_alive():
        raise RuntimeError('original fault-target caller did not stop')
    original = results[line]
    (tmp / f'intentionally-faulted-caller.{line}.log').write_text(original.get('log', ''))
    if original.get('ok'):
        raise RuntimeError('fault did not interrupt the intended active caller')
    deadline = time.monotonic() + 15
    while True:
        text = sup_out.text()
        after = dict(worker_pids(text))
        pid = int(after.get(str(line), old_pid))
        marker = f'line {line}: worker pid {pid} started'
        tail = text[text.rfind(marker):] if marker in text else ''
        ready = re.search(rf'^\[L{line}\].*SIP V\.90 modem ready', tail, re.MULTILINE)
        if pid != old_pid and ready:
            break
        if time.monotonic() >= deadline:
            raise RuntimeError('replacement worker did not become ready')
        time.sleep(.02)
    for other in range(args.lines):
        if other != line and before[str(other)] != after.get(str(other)):
            raise RuntimeError(f'unrelated worker {other} restarted')
    replacement = threading.Thread(target=one_call,
                                   args=(line, args, tmp, line_port, call_rtp,
                                         results, threading.Barrier(1), call_port))
    replacement.start()
    callers[line] = replacement
    print(f'fault: line {line} restarted as pid {pid}; other worker PIDs unchanged')
    return original['bytes_echoed']


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
    global BINARY
    ap = argparse.ArgumentParser()
    ap.add_argument('--binary', type=Path, default=Path(BINARY),
                    help='freeze/select the executable under test')
    ap.add_argument('--lines', type=int, default=3)
    ap.add_argument('--mode', default='v22')
    ap.add_argument('--recycle', action='store_true')
    ap.add_argument('--rounds', type=int, default=1)
    ap.add_argument('--restart-line', type=int,
                    help='kill this active worker and redial it while peers continue; needs >=64 payload rounds')
    ap.add_argument('--payload-rounds', type=int, default=1,
                    help='distinct payload blocks per call, checked byte-for-byte')
    ap.add_argument('--payload-offset', type=int, default=0,
                    help='number the payloads from this instead of 0')
    ap.add_argument('--io-schedule', action='store_true',
                    help='record every process\'s RX/TX timing (ME_IO_SCHEDULE) and report its longest gap')
    ap.add_argument('--connect-timeout', type=float, default=60)
    ap.add_argument('--echo-timeout', type=float, default=30)
    args = ap.parse_args()
    BINARY = str(args.binary.resolve())
    if not 1 <= args.lines <= 64 or args.payload_rounds < 1 or args.rounds < 1:
        ap.error('lines must be 1..64; payload-rounds and rounds must be positive')
    if args.restart_line is not None:
        if (not 0 <= args.restart_line < args.lines or args.lines < 2
                or args.recycle or args.rounds != 1 or args.payload_rounds < 64):
            ap.error('restart-line needs >=2 lines, >=64 payload-rounds, one round and no recycle')
    if args.recycle and args.rounds < 2:
        args.rounds = 2

    source_binary = BINARY
    tmp = Path(tempfile.mkdtemp(prefix='multi-line-'))
    BINARY = str(tmp / 'sip_v90_modem')
    binary_hash = freeze_binary(source_binary, BINARY)
    print(f'binary: {source_binary} frozen={BINARY} sha256={binary_hash}')
    line_port = free_port_block(args.lines)
    allocated = set(range(line_port, line_port + args.lines))
    line_rtp = free_port_block(args.lines, 100, width=2, avoid=allocated, even=True)
    allocated.update(line_rtp + 100*i + k for i in range(args.lines) for k in range(2))
    call_rtp = free_port_block(args.lines, 100, width=2, avoid=allocated, even=True)
    allocated.update(call_rtp + 100*i + k for i in range(args.lines) for k in range(2))
    call_port = free_port_block(args.lines, avoid=allocated)
    sup_args = [BINARY, '--lines', str(args.lines), '--bind-addr', '127.0.0.1',
                '--local-port', str(line_port), '--rtp-port', str(line_rtp),
                '--auto-answer', '1', '--mode', args.mode,
                '--pty-link', str(tmp / 'line')]
    if args.recycle:
        sup_args.append('--recycle')
    line_env = {'ME_DATA_FRAMING': 'v14', 'ME_DUMP_DIR': str(tmp / 'line-dumps')}
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
            os.close(open_pty(str(tmp / f'line.{i}')))
        for rnd in range(args.rounds):
            results = {}
            barrier = threading.Barrier(args.lines)
            cancellations = [threading.Event() for _ in range(args.lines)]
            callers = [threading.Thread(target=one_call,
                                        args=(i, args, tmp, line_port, call_rtp, results,
                                              barrier, call_port, cancellations[i]))
                       for i in range(args.lines)]
            for t in callers:
                t.start()
            if args.restart_line is not None:
                try:
                    interrupted_bytes = restart_active_line(
                        args.restart_line, args, tmp, results, callers, cancellations,
                        sup_out, line_port, call_rtp, call_port)
                    print(f'fault: original call intentionally lost after {interrupted_bytes} exact bytes; '
                          'replacement must pass independently')
                except (OSError, RuntimeError) as exc:
                    failures += 1
                    print(f'FAIL: active-worker isolation: {exc}')
            for t in callers:
                t.join()
            ok = [results[i]['ok'] for i in range(args.lines)]
            for i in range(args.lines):
                r = results[i]
                state = 'ok' if r['ok'] else 'FAIL ' + r['why']
                rate = r.get('rate', 0)
                print(f'round {rnd} line {i}: CONNECT {rate}, {state}, '
                      f'{r["bytes_echoed"]} exact bytes')
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
                    os.close(open_pty(str(tmp / f'line.{i}')))
    finally:
        stop.set()
        for t in echoes:
            t.join(timeout=2)
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
    if hashlib.sha256(Path(BINARY).read_bytes()).hexdigest() != binary_hash:
        failures += 1
        print('FAIL: executable changed during the run; use --binary with a frozen copy')
    fault_note = '; one intentional call loss, replacement validated' if args.restart_line is not None else ''
    print('multi_line_test: ' + ('FAILED' if failures else
          f'OK ({args.lines} lines, {args.rounds} round(s){", recycled" if args.recycle else ""}{fault_note})'))
    print(f'logs/captures: {tmp}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
