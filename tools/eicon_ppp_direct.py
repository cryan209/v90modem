#!/usr/bin/env python3
"""PPP straight from CONNECT on an Eicon endpoint that answers with LCP.

7910 stopped offering the echo menu (it sends LCP Configure-Requests as soon
as the modem connects), so eicon_soak_test.py's `Select:` wait never
completes.  This dials, waits for CONNECT, hands the PTY to pppd (the peer
assigns the addresses), then pings the peer and optionally fetches HTTP.
Evidence: summary.json, server.log, pppd.log, ping.log.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import select
import subprocess
import termios
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('output', type=Path)
    ap.add_argument('--law', choices=('ulaw', 'alaw'), default='ulaw')
    ap.add_argument('--mode', choices=('v90', 'v34'), default='v34')
    ap.add_argument('--ms', help='full AT+MS command, overrides --mode')
    ap.add_argument('--pings', type=int, default=30)
    ap.add_argument('--local-port', type=int, default=5078)
    ap.add_argument('--rtp-port', type=int, default=14600)
    args = ap.parse_args()
    d = args.output
    d.mkdir(parents=True, exist_ok=False)
    ext, account = ('7900', '2905') if args.law == 'alaw' else ('7910', '2900')
    pty = '/tmp/eicon-ppp-' + d.name
    env = os.environ.copy()
    env.pop('ME_V34_SPAN_FLOW_LOG', None)
    env.update(ME_MODE=args.mode, ME_V90_ROLE='analogue', VPCM_ME_VERBOSE='1',
               VPCM_G711_TAP_DIR=str(d.resolve()), ME_DUMP_DIR=str(d.resolve()))
    env['SIP_FORCE_PCMA' if args.law == 'alaw' else 'SIP_FORCE_PCMU'] = '1'
    result = dict(extension=ext, mode=args.mode, ok=False)
    proc = child = None
    fd = None
    start = time.monotonic()
    try:
        with (d / 'server.log').open('wb') as log:
            proc = subprocess.Popen(
                [str(ROOT / 'sip_v90_modem'), '--sip-server', 'asterisk.net.cryan.nz',
                 '--username', account, '--password', os.environ.get('EICON_SIP_PASSWORD', account),
                 '--local-port', str(args.local_port), '--rtp-port', str(args.rtp_port),
                 '--pty-link', pty, '--verbose'], cwd=ROOT, env=env, stdout=log, stderr=log)
            for _ in range(100):
                if proc.poll() is not None:
                    raise RuntimeError('modem exited before PTY opened')
                if os.path.exists(pty):
                    break
                time.sleep(.1)
            fd = os.open(pty, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            a = termios.tcgetattr(fd)
            a[0] = a[1] = a[3] = 0
            a[2] |= termios.CLOCAL | termios.CREAD
            termios.tcsetattr(fd, termios.TCSANOW, a)
            time.sleep(3)
            rx = bytearray()

            def read_until(token, timeout):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    if select.select([fd], [], [], .2)[0]:
                        try:
                            chunk = os.read(fd, 65536)
                        except BlockingIOError:
                            continue
                        rx.extend(chunk)
                        if token in rx:
                            return True
                        if b'NO CARRIER' in rx:
                            return False
                return False

            for cmd in ('ATE0', args.ms or 'AT+MS=%s,0' % args.mode.upper()):
                os.write(fd, cmd.encode() + b'\r')
                if not read_until(b'OK', 3):
                    raise RuntimeError('AT rejected: ' + cmd)
                del rx[:]
            os.write(fd, ('ATD' + ext + '\r').encode())
            if not read_until(b'CONNECT', 90):
                raise RuntimeError('no CONNECT')
            m = re.search(rb'CONNECT (\d+)', rx)
            result['connect_rate'] = int(m.group(1)) if m else None
            result['lcp_seen'] = False
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)
            with (d / 'pppd.log').open('wb') as out:
                child = subprocess.Popen(
                    ['/usr/sbin/pppd', 'notty', 'nodetach', 'local', 'noauth', 'nodefaultroute',
                     'noproxyarp', 'noipv6', 'noccp', 'novj', 'nopersist', 'logfd', '2',
                     'ipcp-accept-local', 'ipcp-accept-remote', 'lcp-echo-interval', '10',
                     'lcp-echo-failure', '6', 'debug', '0.0.0.0:'],
                    stdin=fd, stdout=fd, stderr=out)
                deadline = time.monotonic() + 60
                peer_ip = None
                while time.monotonic() < deadline:
                    if child.poll() is not None:
                        raise RuntimeError('pppd exited')
                    text = (d / 'pppd.log').read_text(errors='replace')
                    result['lcp_seen'] = 'rcvd [LCP' in text
                    m = re.search(r'remote IP address (\S+)', text)
                    if m:
                        peer_ip = m.group(1)
                        break
                    time.sleep(.5)
                if not peer_ip:
                    raise TimeoutError('IPCP negotiation')
                result['peer_ip'] = peer_ip
                ping = subprocess.run(['ping', '-n', '-c', str(args.pings), '-i', '1', '-W', '5', peer_ip],
                                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=args.pings * 6 + 30)
                (d / 'ping.log').write_bytes(ping.stdout)
                result['ping_status'] = ping.returncode
                loss = re.search(rb'(\d+(?:\.\d+)?)% packet loss', ping.stdout)
                result['loss_percent'] = float(loss.group(1)) if loss else None
                result['ok'] = ping.returncode == 0
    except Exception as exc:
        result['error'] = str(exc)
        print('ERROR', exc, flush=True)
    finally:
        for p in (child, proc):
            if p is not None:
                p.terminate()
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
        if fd is not None:
            os.close(fd)
        result['elapsed_seconds'] = round(time.monotonic() - start, 1)
        (d / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
