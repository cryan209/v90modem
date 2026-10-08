#!/usr/bin/env python3
"""Timed download, upload, simultaneous binary tests, or PPP over an Eicon call."""
import argparse
import fcntl
import hashlib
import urllib.request
import json
import os
from pathlib import Path
import select
import subprocess
import termios
import time

from eicon.stream_test import run as stream_run

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('output', type=Path)
    ap.add_argument('--law', choices=('ulaw', 'alaw'), default='ulaw')
    ap.add_argument('--mode', choices=('v90', 'v34'), default='v90')
    ap.add_argument('--local-port', type=int, default=5078)
    ap.add_argument('--rtp-port', type=int, default=14600)
    ap.add_argument('--jitter-buffer-ms', type=int, help='fixed RTP prefetch; 40 ms tested on Tower LAN')
    ap.add_argument('--test', choices=('soak', 'probe', 'ppp'), default='soak')
    ap.add_argument('--http-bytes', type=int, default=0, help='HTTP download and POST upload over PPP')
    ap.add_argument('--http-port', type=int, default=19890)
    ap.add_argument('--download-seconds', type=int, default=610)
    ap.add_argument('--upload-seconds', type=int, default=610)
    ap.add_argument('--duplex-seconds', type=int, default=310)
    args = ap.parse_args()
    if args.jitter_buffer_ms is not None and not 1 <= args.jitter_buffer_ms <= 2000:
        ap.error('jitter-buffer-ms must be 1..2000')
    for value in (args.download_seconds, args.upload_seconds, args.duplex_seconds):
        if not 1 <= value <= 3600: ap.error('durations must be 1..3600')
    if not 0 <= args.http_bytes <= 16*1024*1024: ap.error('http-bytes must be 0..16777216')
    if args.http_bytes and args.test != 'ppp': ap.error('http-bytes requires --test ppp')
    d = args.output.resolve()
    d.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.pop('ME_V34_SPAN_FLOW_LOG', None)
    env.pop('SIP_FORCE_PCMA', None)
    env.pop('SIP_FORCE_PCMU', None)
    env.update(ME_MODE=args.mode, ME_V90_ROLE='analogue', VPCM_ME_VERBOSE='1',
               ME_DUMP_DIR=str(d), VPCM_G711_TAP_DIR=str(d), ME_IO_SCHEDULE=str(d/'io.bin'))
    env['SIP_FORCE_PCMU' if args.law == 'ulaw' else 'SIP_FORCE_PCMA'] = '1'
    if args.jitter_buffer_ms is not None:
        env['ME_JB_MS'] = str(args.jitter_buffer_ms)
    account, ext = ('2900', '7910') if args.law == 'ulaw' else ('2905', '7900')
    pty = '/tmp/eicon-soak-'+str(os.getpid())
    result = dict(mode=args.mode, law=args.law, test=args.test,
                  jitter_buffer_ms=env.get('ME_JB_MS', '200'))
    fd = None
    proc = None
    start = time.monotonic()
    with (d/'server.log').open('wb') as log, (d/'menu.bin').open('wb') as transcript:
        def read_until(token, timeout=30):
            buf = bytearray()
            end = time.monotonic()+timeout
            # Read exactly through the textual prompt: don't consume ZMODEM headers.
            while time.monotonic() < end:
                if proc.poll() is not None:
                    raise RuntimeError('modem exited')
                if select.select([fd], [], [], .2)[0]:
                    try:
                        chunk = os.read(fd, 1)
                    except BlockingIOError:
                        continue
                    if not chunk:
                        raise RuntimeError('PTY closed')
                    buf.extend(chunk)
                    transcript.write(chunk)
                    if buf.endswith(token):
                        transcript.flush()
                        return bytes(buf)
                    if buf.endswith(b'NO CARRIER'):
                        raise RuntimeError('NO CARRIER')
            raise TimeoutError('waiting for '+repr(token))

        def send(text):
            os.write(fd, text.encode()+b'\r')

        def timed(kind, seconds):
            send('6')
            read_until(b'Test download/upload/duplex: ')
            send(kind)
            read_until(b'maximum 3600: ')
            send(str(seconds))
            read_until(b'STREAM READY\r\n')
            def progress(stats):
                print(kind, stats, flush=True)
                (d/(kind+'-progress.json')).write_text(json.dumps(stats, indent=2)+'\n')
            local = stream_run(fd, 'caller', kind, seconds, progress=progress)
            text = read_until(b'Select: ', 60)
            line = next(line for line in text.splitlines() if line.startswith(b'STREAM RESULT '))
            peer = json.loads(line[len(b'STREAM RESULT '):])
            minimum = max(0, seconds-2) if args.test == 'probe' else (300 if kind == 'duplex' else 600)
            receivers = [local] if kind == 'download' else [peer] if kind == 'upload' else [local, peer]
            ok = (local['ok'] and peer['ok'] and local['tx_bytes'] == peer['rx_bytes']
                  and local['rx_bytes'] == peer['tx_bytes']
                  and all(row['rx_bytes'] > 0 and row['rx_active_seconds'] >= minimum for row in receivers))
            result[kind] = dict(local=local, peer=peer, minimum_active_seconds=minimum, ok=ok)
            (d/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
            print(kind, result[kind], flush=True)
            if not ok: raise RuntimeError(kind+' integrity or minimum duration failed')

        def ppp():
            send('7')
            banner = read_until(b'\r\n', 10)
            # The menu echoes the choice before its PPP banner.
            while b'PPP READY ' not in banner:
                if b'unavailable' in banner: raise RuntimeError(banner.decode())
                banner = read_until(b'\r\n', 10)
            fields = banner.decode().strip().split()
            peer_ip, local_ip = fields[-2:]
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)
            child = None
            try:
                with (d/'pppd.log').open('wb') as output:
                    child = subprocess.Popen(['/usr/sbin/pppd', 'notty', 'nodetach', 'local', 'noauth',
                        'nodefaultroute', 'noproxyarp', 'noipv6', 'noccp', 'novj', 'nopersist',
                        'logfd', '2', 'lcp-echo-interval', '10', 'lcp-echo-failure', '6',
                        local_ip+':'+peer_ip], stdin=fd, stdout=fd, stderr=output)
                    deadline = time.monotonic()+60
                    while time.monotonic() < deadline:
                        if child.poll() is not None: raise RuntimeError('pppd exited')
                        if ('remote IP address '+peer_ip) in (d/'pppd.log').read_text(errors='replace'): break
                        time.sleep(.5)
                    else: raise TimeoutError('PPP IPCP negotiation')
                    ping = subprocess.run(['ping', '-n', '-c', '30', '-i', '1', '-W', '5', peer_ip],
                                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=180)
                    (d/'ping.log').write_bytes(ping.stdout)
                    result['ppp'] = dict(local_ip=local_ip, peer_ip=peer_ip, ping_status=ping.returncode,
                                         ok=ping.returncode == 0 and b' 0% packet loss' in ping.stdout)
                    if not result['ppp']['ok']: raise RuntimeError('PPP ping failed')
                    if args.http_bytes:
                        from eicon.http_test_server import payload
                        data = payload(args.http_bytes)
                        expected = hashlib.sha256(data).hexdigest()
                        url = f'http://{peer_ip}:{args.http_port}'
                        # Ignore proxy environment variables: every byte must cross PPP.
                        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
                        result['http'] = {}
                        for direction in ('download', 'upload'):
                            t = time.monotonic()
                            request = urllib.request.Request(url+f'/download/{args.http_bytes}') if direction == 'download' else urllib.request.Request(
                                url+'/upload', data=data, method='POST', headers={'Content-Type': 'application/octet-stream'})
                            with opener.open(request, timeout=900) as response:
                                body = response.read()
                                status = response.status
                            elapsed = time.monotonic()-t
                            (d/(direction+'-http.bin')).write_bytes(body)
                            if direction == 'download':
                                actual = hashlib.sha256(body).hexdigest()
                                count = len(body)
                            else:
                                received = json.loads(body)
                                actual, count = received['sha256'], received['bytes']
                            entry = dict(status=status, bytes=count, seconds=round(elapsed, 3),
                                         bytes_per_second=round(count/elapsed, 1), sha256=actual,
                                         expected_sha256=expected, ok=status == 200 and count == args.http_bytes and actual == expected)
                            result['http'][direction] = entry
                            (d/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
                            print('HTTP', direction, entry, flush=True)
                            if not entry['ok']: raise RuntimeError('HTTP '+direction+' failed integrity')

            finally:
                if child and child.poll() is None:
                    child.terminate()
                    try: child.wait(timeout=5)
                    except subprocess.TimeoutExpired: child.kill(); child.wait()
                fcntl.fcntl(fd, fcntl.F_SETFL, flags)

        try:
            proc = subprocess.Popen([str(ROOT/'sip_v90_modem'), '--sip-server', 'asterisk.net.cryan.nz',
                '--username', account, '--password', os.environ.get('EICON_SIP_PASSWORD', account),
                '--local-port', str(args.local_port), '--rtp-port', str(args.rtp_port), '--pty-link', pty, '--verbose'],
                env=env, cwd=ROOT, stdout=log, stderr=log)
            for _ in range(100):
                if proc.poll() is not None:
                    raise RuntimeError('modem launch failed')
                if os.path.exists(pty):
                    break
                time.sleep(.1)
            fd = os.open(pty, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            a = termios.tcgetattr(fd)
            a[0] = a[1] = a[3] = 0
            a[2] |= termios.CLOCAL | termios.CREAD
            a[6][termios.VMIN] = 1
            a[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, a)
            time.sleep(3)
            send('ATE0')
            read_until(b'OK', 5)
            send('AT+MS='+args.mode.upper()+',0')
            read_until(b'OK', 5)
            send('ATD'+ext)
            read_until(b'Select: ', 70)
            if args.test == 'ppp':
                ppp()
            else:
                for kind, seconds in (('download', args.download_seconds), ('upload', args.upload_seconds),
                                      ('duplex', args.duplex_seconds)):
                    timed(kind, seconds)
                send('q')
            result['ok'] = True
        except Exception as exc:
            result['ok'] = False
            result['error'] = str(exc)
            print('ERROR', exc, flush=True)
        finally:
            if proc:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
            if fd is not None:
                os.close(fd)
            result['elapsed_seconds'] = round(time.monotonic()-start, 3)
            (d/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    raise SystemExit(0 if result['ok'] else 1)


if __name__ == '__main__':
    main()
