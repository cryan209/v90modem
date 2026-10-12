#!/usr/bin/env python3
"""Hardware-free Apple GUI check: on-hook idle, AT control and fresh sessions."""
import base64
import json
import os
from pathlib import Path
import selectors
import subprocess
import sys
import tempfile
import time
import urllib.request


def wait_for(fn, timeout=15):
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        value = fn()
        if value: return value
        time.sleep(.1)
    raise AssertionError('Apple GUI timed out')


def main():
    repo = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='gui-apple-test-') as temp:
        env = dict(os.environ, BROWSER='true', ME_MODE='v22', APPLE_MODEM_ADDR='0:0')
        with (Path(temp)/'launcher.log').open('wb') as log:
            child = subprocess.Popen([sys.executable,str(repo/'tools/modem_gui.py'),
                '--web','--apple',str(repo/'apple_usb_modem_coupler'), '--hold','1'],
                env=env,stdout=subprocess.PIPE,stderr=log)
            try:
                with selectors.DefaultSelector() as selector:
                    selector.register(child.stdout,selectors.EVENT_READ)
                    assert selector.select(20), 'Launcher did not publish its URL'
                    url = child.stdout.readline().decode().strip().removeprefix('Modem GUI: ')
                assert url.startswith('http://127.0.0.1:'), (Path(temp)/'launcher.log').read_text()
                origin = url.split('/',3)[:3]
                origin = '/'.join(origin)
                def state():
                    with urllib.request.urlopen(url+'state',timeout=2) as response:
                        return json.load(response)
                def send(command):
                    payload = json.dumps({'port':'at','data':base64.b64encode((command+'\r').encode()).decode()}).encode()
                    request = urllib.request.Request(url+'send',payload,headers={'Origin':origin,'Content-Type':'application/json'})
                    with urllib.request.urlopen(request,timeout=2) as response: assert response.status == 200
                initial = wait_for(lambda: (s if s.get('hardware_status','').startswith('Idle') else None) if (s:=state()) else None)
                assert initial['backend'] == 'Apple USB modem' and initial['state'] == 0
                assert initial['hardware']['usb_address'] == '0:0'
                wait_for(lambda: state().get('interfaces_ready'))
                send('AT+MS?')
                def transcript():
                    return b''.join(base64.b64decode(chunk) for _,chunk in state()['streams']['at'])
                wait_for(lambda: b'+MS:' in transcript())
                send('AT+MS=V34,0,9600,9600')
                wait_for(lambda: b'OK' in transcript())
                # Address 0:0 cannot identify a configured USB modem. Exercise
                # the failure path without seizing any physical telephone line.
                send('ATD1')
                next_call = wait_for(lambda: s if (s:=state()).get('epoch',0)>initial.get('epoch',0) and s.get('hardware_status','').startswith('Idle') and s.get('interfaces_ready') else None)
                assert next_call['state'] == 0
                time.sleep(.2)
                send('ATH')
                time.sleep(.3)
                after_hangup = state()
                assert after_hangup['epoch'] == next_call['epoch'] and after_hangup['state'] == 0
                send('ATA')
                wait_for(lambda: (s:=state()).get('epoch',0)>next_call['epoch'] and s.get('hardware_status','').startswith('Idle') and s.get('interfaces_ready'))
                print('Apple GUI: on-hook idle, +MS, dial/answer missing-device recovery and idle ATH passed')
            finally:
                child.terminate()
                try: child.wait(timeout=8)
                except subprocess.TimeoutExpired: child.kill(); child.wait()


if __name__ == '__main__': main()
