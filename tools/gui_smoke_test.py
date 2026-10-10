#!/usr/bin/env python3
"""Real localhost SIP GUI check (requires permission to bind local sockets)."""
import base64
import json
import os
from pathlib import Path
import selectors
import socket
import subprocess
import tempfile
import time
import tty
import urllib.request


def wait_for(fn, timeout=25):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        value = fn()
        if value:
            return value
        time.sleep(.1)
    raise AssertionError('Timed out waiting for GUI/connection/payload')


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def main():
    binary = str(Path(__file__).resolve().parents[1]/'sip_v90_modem')
    with tempfile.TemporaryDirectory(prefix='gui-smoke-') as temp:
        at, data = str(Path(temp)/'at'), str(Path(temp)/'data')
        env = dict(os.environ, BROWSER='true', ME_DATA_FRAMING='v14')
        a_port, b_port = free_port(), free_port()
        children, fds = [], []
        try:
            with open(Path(temp)/'peer.log', 'wb') as log:
                peer = subprocess.Popen([binary, '--bind-addr', '127.0.0.1', '--mode', 'v22',
                    '--local-port', str(b_port), '--auto-answer', '1',
                    '--control-link', at, '--data-link', data], env=env, stdout=log, stderr=log)
                children.append(peer)
                wait_for(lambda: os.path.exists(at))
                for path in (at, data):
                    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                    tty.setraw(fd)
                    fds.append(fd)
                gui = subprocess.Popen([binary, '--gui-web', '--bind-addr', '127.0.0.1',
                    '--mode', 'v22', '--sip-server', f'127.0.0.1:{b_port}', '--local-port', str(a_port)],
                    env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                children.append(gui)
                ready = selectors.DefaultSelector()
                ready.register(gui.stdout, selectors.EVENT_READ)
                assert ready.select(10), 'GUI did not start'
                url = gui.stdout.readline().decode().strip().removeprefix('Modem GUI: ')
                ready.close()
                origin = url.split('/', 3)[:3]
                origin = '/'.join(origin)

                def state():
                    with urllib.request.urlopen(url+'state', timeout=2) as r:
                        return json.load(r)

                def send(port, value):
                    req = urllib.request.Request(url+'send', data=json.dumps({'port': port,
                        'data': base64.b64encode(value).decode()}).encode(),
                        headers={'Content-Type': 'application/json', 'Origin': origin})
                    with urllib.request.urlopen(req, timeout=2) as r:
                        assert r.status == 200

                wait_for(lambda: state().get('ports'))
                time.sleep(.3)
                send('at', b'AT+MS?\r')
                def at_ok():
                    return any(b'OK' in base64.b64decode(data) for _, data in state()['streams']['at'])
                wait_for(at_ok)
                send('at', b'ATD6004\r')
                def connected():
                    s = state()
                    return s if s.get('state') == 4 and s.get('age', 9) < 1 and len(s.get('iq', [[]])[0]) == 256 else None
                s = wait_for(connected)
                assert all(len(a) == 512 for a in s['audio'])
                assert all(p['count'] > 0 for p in s['pcm'])
                assert all(len(p['hex']) == 6400 for p in s['listen'])
                assert 2390 < s['rx_carrier'] < 2410 and s['tx_carrier'] == 1200
                assert all(w['runs'] for w in s['wire'])
                assert len(json.dumps(s)) > 9216, 'Must exercise large telemetry datagrams'
                send('data', b'GUI-to-peer\x00\xff\r\n')
                received = bytearray()
                def peer_payload():
                    try:
                        received.extend(os.read(fds[1], 4096))
                    except BlockingIOError:
                        pass
                    return b'GUI-to-peer\x00\xff\r\n' in received
                wait_for(peer_payload)
                os.write(fds[1], b'peer-to-GUI\x00\xfe\r\n')
                def gui_payload():
                    data = b''.join(base64.b64decode(b) for _, b in state()['streams']['data'])
                    return b'peer-to-GUI\x00\xfe\r\n' in data
                wait_for(gui_payload)
                s = state()
                assert all(w['count'] > 0 for w in s['wire'])
                assert s['events'] and s['age'] < 1
                send('at', b'ATH\r')
                wait_for(lambda: state().get('state') == 0)
                print('gui_smoke_test: OK (AT, live large snapshots, QAM, wire bytes, bidirectional binary serial data, hangup)')
        finally:
            for child in reversed(children):
                if child.poll() is None:
                    child.terminate()
                    try:
                        child.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
            for fd in fds:
                os.close(fd)


if __name__ == '__main__':
    main()
