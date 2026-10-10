#!/usr/bin/env python3
"""Exercise the self-contained GUI echo pair and profile changes over SIP."""
import base64
import json
import os
from pathlib import Path
import selectors
import subprocess
import sys
import time
import urllib.request

from gui_smoke_test import wait_for


def main():
    binary = str(Path(__file__).resolve().parents[1]/'sip_v90_modem')
    env = dict(os.environ, BROWSER='true')
    child = subprocess.Popen([binary, '--gui-web', '--loopback'], env=env,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        with selectors.DefaultSelector() as ready:
            ready.register(child.stdout, selectors.EVENT_READ)
            assert ready.select(10), 'Loopback launcher did not start'
            url = child.stdout.readline().decode().strip().removeprefix('Modem GUI: ')
        origin = '/'.join(url.split('/', 3)[:3])

        def state():
            with urllib.request.urlopen(url+'state', timeout=2) as response:
                return json.load(response)

        def post(path, body):
            req = urllib.request.Request(url+path, data=json.dumps(body).encode(),
                headers={'Origin': origin, 'Content-Type': 'application/json'})
            with urllib.request.urlopen(req, timeout=2) as response:
                assert response.status == 200

        def send(port, payload):
            post('send', {'port': port, 'data': base64.b64encode(payload).decode()})

        wait_for(lambda: state().get('loopback', {}).get('ready'))
        paths = dict(state()['ports'])
        profiles = sys.argv[1:] or [p[0] for p in state()['loopback']['profiles']]
        for index,profile in enumerate(profiles):
            wait_for(lambda: state().get('state', 0) == 0 and state()['loopback']['peer_state'] == 0)
            print('Checking '+profile, flush=True)
            post('loopback', {'profile': profile})
            wait_for(lambda: state().get('data_ready'), timeout=45)
            wait_for(lambda: state()['loopback']['pattern_ready'])
            s = state()
            assert s['age'] < 1 and len(s['audio'][0]) == 512
            if profile not in ('v21','v90','v91','v92'):
                assert len(s['eye']) == 128, 'Recovered samples missing'
            if profile == 'v21': assert s['modulation'] == 8 and s['rx_baud'] == 300
            if profile == 'v90': assert s['modulation'] == 2 and s['rx_pcm'] and not s['tx_pcm']
            if profile == 'v92': assert s['modulation'] == 2 and s['rx_pcm'] and s['tx_pcm']
            if profile == 'v91': assert s['modulation'] == 1 and s['rx_pcm'] and s['tx_pcm']
            if profile == 'v92': assert s['v92'], 'V.92 PCM upstream was not negotiated'
            start = max((ident for ident,_ in s['streams']['data']), default=0)
            payload = bytes(range(256))+b'local-loopback\x00\xff\r\n'
            send('data', payload)

            received = bytearray(); seen = start
            def returned():
                nonlocal seen
                s = state()
                chunks = [base64.b64decode(chunk) for ident,chunk in s['streams']['data'] if ident > seen]
                # Caller write echoes in the GUI are distinct from actual RX.
                seen = max((ident for ident,_ in s['streams']['data']), default=seen)
                received.extend(b''.join(chunk for chunk in chunks if not chunk.startswith(b'\n> ')))
                return payload in received and s['loopback']['echoed'] >= len(payload)

            wait_for(returned, timeout=30 if profile == 'v21' else 8)
            if index != len(profiles)-1:
                send('at', b'ATH\r')
                wait_for(lambda: state().get('state') == 0)
            print(profile+' byte-exact echo OK', flush=True)
        print('gui_loopback_test: OK (all profiles, real SIP, binary echo and repeated calls)', flush=True)
    except Exception:
        if 'state' in locals():
            s = state()
            print('Failed state:', {k:s.get(k) for k in ('state','rx_signal','tx_signal','loopback')}, flush=True)
            print('Serial:', b''.join(base64.b64decode(c) for _,c in s['streams']['data'])[-1200:].hex(), flush=True)
            print('AT:', b''.join(base64.b64decode(c) for _,c in s['streams']['at'])[-2000:], flush=True)
            print('Log:', b''.join(base64.b64decode(c) for _,c in s['streams']['log'])[-5000:], flush=True)
        raise
    finally:
        child.terminate()
        try:
            child.wait(timeout=12)
        except subprocess.TimeoutExpired:
            child.kill(); child.wait()
        if 'paths' in locals():
            assert all(not os.path.lexists(path) for path in paths.values()), 'Temporary PTYs were not removed'


if __name__ == '__main__':
    main()
