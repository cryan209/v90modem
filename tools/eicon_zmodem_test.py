#!/usr/bin/env python3
"""Test both menu ZMODEM directions over a dedicated Eicon modem call."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import select
import subprocess
import termios
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('output', type=Path)
    ap.add_argument('--law', choices=('ulaw', 'alaw'), default='ulaw')
    ap.add_argument('--mode', choices=('v90', 'v34'), default='v90')
    ap.add_argument('--bytes', type=int, default=65536)
    args = ap.parse_args()
    if not 1 <= args.bytes <= 1048576:
        ap.error('bytes must be 1..1048576')
    d = args.output.resolve()
    d.mkdir(parents=True, exist_ok=False)
    (d/'download').mkdir()
    # Deterministic binary data; the prefix exercises every byte value.
    data = (bytes(range(256)) + b''.join(hashlib.sha256(str(i).encode()).digest()
            for i in range((args.bytes+31)//32)))[:args.bytes]
    upload = d/'binary-upload.bin'
    upload.write_bytes(data)
    expected = hashlib.sha256(data).hexdigest()
    env = os.environ.copy()
    env.pop('ME_V34_SPAN_FLOW_LOG', None)
    env.pop('SIP_FORCE_PCMA', None)
    env.pop('SIP_FORCE_PCMU', None)
    env.update(ME_MODE=args.mode, ME_V90_ROLE='analogue', VPCM_ME_VERBOSE='1',
               ME_DUMP_DIR=str(d), VPCM_G711_TAP_DIR=str(d), ME_IO_SCHEDULE=str(d/'io.bin'))
    env['SIP_FORCE_PCMU' if args.law == 'ulaw' else 'SIP_FORCE_PCMA'] = '1'
    account, ext = ('2900', '7910') if args.law == 'ulaw' else ('2905', '7900')
    pty = '/tmp/eicon-zmodem-test'
    result = dict(mode=args.mode, law=args.law, bytes=args.bytes, upload_sha256=expected)
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

        def transfer(direction):
            send('2' if direction == 'upload' else '3')
            read_until(b'[Z]: ')
            send('z')
            if direction == 'download':
                read_until(b'maximum 1048576: ')
                send(str(args.bytes))
            read_until(b'start your terminal transfer now.\r\n')
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)
            cmd = ['/usr/bin/sz', '--zmodem', '--binary', '--quiet', str(upload)] if direction == 'upload' else [
                '/usr/bin/rz', '--zmodem', '--binary', '--overwrite', '--quiet']
            t = time.monotonic()
            try:
                with (d/(direction+'-lrzsz.log')).open('wb') as err:
                    child = subprocess.Popen(cmd, stdin=fd, stdout=fd, stderr=err, cwd=d/'download')
                    try:
                        status = child.wait(timeout=180)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
                        raise TimeoutError(direction+' ZMODEM timeout')
            finally:
                fcntl.fcntl(fd, fcntl.F_SETFL, flags)
            text = read_until(b'Select: ', 30)
            entry = dict(status=status, seconds=round(time.monotonic()-t, 3))
            if direction == 'upload':
                entry['hash_match'] = expected.encode() in text
            else:
                received = (d/'download/test-payload.bin').read_bytes()
                pattern = b'0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz\r\n'
                wanted = (pattern*((args.bytes+len(pattern)-1)//len(pattern)))[:args.bytes]
                entry.update(received_bytes=len(received), sha256=hashlib.sha256(received).hexdigest(),
                             hash_match=received == wanted)
            entry['ok'] = status == 0 and entry['hash_match'] and b'Transfer complete' in text
            result[direction] = entry
            (d/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
            print(direction, entry, flush=True)
            if not entry['ok']:
                raise RuntimeError(direction+' failed verification')

        try:
            proc = subprocess.Popen([str(ROOT/'sip_v90_modem'), '--sip-server', 'asterisk.net.cryan.nz',
                '--username', account, '--password', os.environ.get('EICON_SIP_PASSWORD', account),
                '--local-port', '5078', '--rtp-port', '14600', '--pty-link', pty, '--verbose'],
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
            transfer('upload')
            transfer('download')
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
