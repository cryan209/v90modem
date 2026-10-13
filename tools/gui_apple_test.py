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
import struct
import urllib.error
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
        env = dict(os.environ, BROWSER='true', ME_MODE='v22', APPLE_MODEM_ADDR='0:0', APPLE_GUI_HOME=str(Path(temp)/'settings'))
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
                def action(name, **values):
                    payload = json.dumps(dict(action=name,**values)).encode()
                    request = urllib.request.Request(url+'apple',payload,headers={'Origin':origin,'Content-Type':'application/json'})
                    with urllib.request.urlopen(request,timeout=5) as response: assert response.status == 200
                def idle_after(epoch):
                    return wait_for(lambda: s if (s:=state()).get('epoch',0)>epoch and s.get('state') == 0 and s.get('interfaces_ready') and not s['hardware'].get('replay') else None)
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
                action('save_settings')
                profile = Path(initial['hardware']['profile_path'])
                assert 'V34' in profile.read_text()
                send('ATD1')
                next_call = wait_for(lambda: s if (s:=state()).get('epoch',0)>initial.get('epoch',0) and s.get('hardware_status','').startswith('Idle') and s.get('interfaces_ready') else None)
                assert next_call['state'] == 0
                start = state()['streams']['at'][-1][0]
                send('AT+MS?')
                wait_for(lambda: b'V34' in b''.join(base64.b64decode(chunk) for ident,chunk in state()['streams']['at'] if ident>start))
                time.sleep(.2)
                send('ATH')
                time.sleep(.3)
                after_hangup = state()
                assert after_hangup['epoch'] == next_call['epoch'] and after_hangup['state'] == 0
                send('ATA')
                wait_for(lambda: (s:=state()).get('epoch',0)>next_call['epoch'] and s.get('hardware_status','').startswith('Idle') and s.get('interfaces_ready'))
                action('refresh')
                try:
                    action('configure',usb_address='not-a-device',audio_uid='')
                    raise AssertionError('Invalid device was accepted')
                except urllib.error.HTTPError as error: assert error.code == 400
                action('probe')
                wait_for(lambda: state().get('hardware_status') == 'No selected USB modem')
                action('monitor',enabled=True)
                time.sleep(.2)
                assert not state()['line_health']['monitoring']
                action('auto_answer',rings=2)
                assert state()['hardware']['auto_answer'] == 2
                action('auto_answer',rings=0)
                previous = state()['epoch']
                action('capture',enabled=True)
                capture_state = idle_after(previous)
                assert capture_state['hardware']['capture']
                metadata = Path(capture_state['hardware']['capture_path'])/'session.json'
                assert json.loads(metadata.read_text())['rate'] == 9600
                previous = capture_state['epoch']
                action('capture',enabled=False)
                idle_after(previous)
                fixture = Path(temp)/'settings/captures/fixture'; fixture.mkdir(parents=True)
                (fixture/'session.json').write_text(json.dumps({'created':'Fixture','rate':8000}))
                (fixture/'rx.s16').write_bytes(struct.pack('<h',1000)*48000)
                action('replay',id='fixture')
                replay_state = wait_for(lambda: s if (s:=state())['hardware'].get('replay') and s.get('hardware_audio',{}).get('listen',[{}])[0].get('count',0)>512 else None)
                assert replay_state['hardware_audio']['rate'] == 8000
                assert replay_state['hardware_audio']['metrics'][0]['dc'] == 1000
                assert len(replay_state['hardware_audio']['audio'][0]) == 512
                assert len(replay_state['hardware_audio']['listen'][0]['hex']) <= 6400
                action('stop')
                restored = idle_after(replay_state['epoch'])
                wait_for(lambda: state().get('hardware_audio',{}).get('rate') == 9600)
                try:
                    action('replay',id='../outside')
                    raise AssertionError('Invalid replay path was accepted')
                except urllib.error.HTTPError as error: assert error.code == 400
                print('Apple GUI: stored settings, device validation, line/monitor failures, auto-answer settings, capture metadata, bounded raw replay, stop and rate restoration passed')
            finally:
                child.terminate()
                try: child.wait(timeout=8)
                except subprocess.TimeoutExpired: child.kill(); child.wait()


if __name__ == '__main__': main()
