#!/usr/bin/env python3
"""Bounded Eicon modem calls with exact echo checks and per-call evidence.

Runs sequentially, preserves G.711 law, and never changes the gateway or peer.
Use the isolated 5078/14600 SIP/RTP ports; no other probe may use them at once.
"""
import argparse
import json
import os
from pathlib import Path
import select
import subprocess
import termios
import time

ROOT = Path(__file__).resolve().parents[1]
MODE_CARRIERS = {'v34': 'V34', 'v90': 'V90', 'v92': 'V92',
                 'v32bis': 'V32B', 'v32': 'V32', 'v22': 'V22B',
                 'v22-1200': 'V22'}


def call(directory, mode, law, lines, startup_seconds, echo_timeout=20):
    directory.mkdir(parents=True, exist_ok=False)
    ext, account = ('7900', '2905') if law == 'alaw' else ('7910', '2900')
    pty = '/tmp/eicon-batch-' + directory.name
    env = os.environ.copy()
    env.pop('SIP_FORCE_PCMA', None)
    env.pop('SIP_FORCE_PCMU', None)
    env.pop('ME_V34_SPAN_FLOW_LOG', None)  # per-symbol logging stalls live audio
    env.update(ME_MODE=mode, ME_V90_ROLE='analogue', VPCM_ME_VERBOSE='1',
               VPCM_G711_TAP_DIR=str(directory.resolve()),
               ME_DUMP_DIR=str(directory.resolve()),
               ME_IO_SCHEDULE=str((directory/'io.bin').resolve()))
    env['SIP_FORCE_PCMA' if law == 'alaw' else 'SIP_FORCE_PCMU'] = '1'
    result = dict(mode=mode, law=law, extension=ext, stage='launch',
                  echo_timeout_seconds=echo_timeout,
                  started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                  events=[], echoes=[], settings={k: v for k, v in env.items()
                  if k.startswith(('ME_', 'SIP_FORCE_'))})
    proc = None
    fd = None
    rx = bytearray()
    start = time.monotonic()
    with (directory/'server.log').open('wb') as log, \
            (directory/'pty-rx.bin').open('wb') as output:
        def event(kind, **fields):
            result['events'].append(dict(t_seconds=round(time.monotonic()-start, 3),
                                         kind=kind, **fields))

        def read_until(token, timeout):
            offset = len(rx)
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    event('modem_exit', status=proc.returncode)
                    return False
                if select.select([fd], [], [], .2)[0]:
                    try:
                        chunk = os.read(fd, 65536)
                    except BlockingIOError:
                        continue
                    if not chunk:
                        return False
                    rx.extend(chunk)
                    output.write(chunk)
                    output.flush()
                    if token in rx[offset:]:
                        return True
                    if b'NO CARRIER' in rx[offset:]:
                        event('no_carrier')
                        return False
            event('timeout', expected=token[:80].decode('ascii', 'replace'))
            return False

        try:
            proc = subprocess.Popen([str(ROOT/'sip_v90_modem'), '--sip-server',
                'asterisk.net.cryan.nz', '--username', account, '--password',
                os.environ.get('EICON_SIP_PASSWORD', account), '--local-port',
                '5078', '--rtp-port', '14600', '--pty-link', pty, '--verbose'],
                cwd=ROOT, env=env, stdout=log, stderr=log)
            for _ in range(100):
                if proc.poll() is not None:
                    raise RuntimeError('modem exited before PTY opened')
                if os.path.exists(pty):
                    break
                time.sleep(.1)
            fd = os.open(pty, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attrs = termios.tcgetattr(fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] |= termios.CLOCAL | termios.CREAD
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            time.sleep(3)
            for command in ('ATE0', 'AT+MS=' + MODE_CARRIERS[mode] + ',0'):
                result['stage'] = 'configure'
                os.write(fd, command.encode()+b'\r')
                if not read_until(b'OK', 3):
                    raise RuntimeError('AT configuration rejected: '+command)
            result['stage'] = 'startup'
            event('dial', number=ext)
            os.write(fd, ('ATD'+ext+'\r').encode())
            result['menu'] = read_until(b'Select: ', startup_seconds)
            if not result['menu']:
                return result
            event('menu')
            result['stage'] = 'echo_selection'
            os.write(fd, b'1\r')
            result['echo_selected'] = read_until(b'/menu exits.', 12)
            if not result['echo_selected']:
                return result
            result['stage'] = 'echo_data'
            for sequence in range(lines):
                payload = ('EICON-%s-%04d-' % (mode.upper(), sequence)
                    + '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz'*3).encode()
                event('send_line', sequence=sequence, bytes=len(payload))
                os.write(fd, payload+b'\r')
                ok = read_until(b'ECHO: '+payload+b'\r\n', echo_timeout)
                result['echoes'].append(dict(sequence=sequence, bytes=len(payload), ok=ok))
                event('echo_result', sequence=sequence, ok=ok)
                print(directory.name, 'echo', sequence, ok, flush=True)
                if not ok:
                    return result
                time.sleep(2)
            result['stage'] = 'complete'
            return result
        except Exception as exc:
            result['error'] = str(exc)
            return result
        finally:
            if proc is not None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
            if fd is not None:
                os.close(fd)
            result['elapsed_seconds'] = round(time.monotonic()-start, 3)
            (directory/'summary.json').write_text(json.dumps(result, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--lines', type=int, default=10)
    parser.add_argument('--startup-seconds', type=float, default=70)
    parser.add_argument('--echo-timeout', type=float, default=20)
    parser.add_argument('--mode', choices=tuple(MODE_CARRIERS))
    parser.add_argument('--law', choices=('ulaw', 'alaw'))
    args = parser.parse_args()
    if args.repeats < 1 or args.lines < 1 or min(args.startup_seconds, args.echo_timeout) <= 0:
        parser.error('repeats, lines and timeouts must be positive')
    if bool(args.mode) != bool(args.law):
        parser.error('--mode and --law must be used together')
    args.output.mkdir(parents=True, exist_ok=False)
    plan = [(args.mode, args.law)] if args.mode else [
        ('v34', 'ulaw'), ('v90', 'alaw'), ('v90', 'ulaw'), ('v34', 'alaw')]
    ended = {}
    results = []
    for repetition in range(1, args.repeats+1):
        for mode, law in plan:
            # Avoid close redials to a controller; unrelated controller can run meanwhile.
            delay = max(0, ended.get(law, 0)+65-time.monotonic())
            if delay:
                print('controller cooldown', law, round(delay, 1), 'seconds', flush=True)
                time.sleep(delay)
            name = '%02d-%s-%s' % (len(results)+1, mode, law)
            print('CALL', name, 'repetition', repetition, flush=True)
            result = call(args.output/name, mode, law, args.lines, args.startup_seconds,
                          args.echo_timeout)
            ended[law] = time.monotonic()
            results.append(dict(directory=name, **result))
            (args.output/'summary.json').write_text(json.dumps(results, indent=2)+'\n')
            print('RESULT', name, result['stage'], 'echoes',
                  sum(x['ok'] for x in result['echoes']), 'error', result.get('error'), flush=True)
    print('BATCH COMPLETE', args.output, flush=True)


if __name__ == '__main__':
    main()
