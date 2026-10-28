#!/usr/bin/env python3
"""Hardware-free: load the opt-in Apple/Eicon profiles through the real AT parser.

USB address 0:0 deliberately cannot select a physical modem; never dial/answer.
Tests stored-profile restore, not hardware interoperability (V.250 Table 20).
"""
import errno
import os
from pathlib import Path
import re
import select
import subprocess
import sys
import tempfile
import time
import tty


def check(backend, profile, carrier, ceiling):
    original = profile.read_bytes()
    with tempfile.TemporaryDirectory(prefix='apple-eicon-profile-') as tmp:
        directory = Path(tmp)
        control, data = directory/'control', directory/'data'
        env = dict(os.environ, APPLE_MODEM_ADDR='0:0', ME_MODE='v90',
                   APPLE_GUI_PROFILE=str(profile))
        fd = None
        with (directory/'backend.log').open('wb') as log:
            child = subprocess.Popen([str(backend), '--gui-control',
                '--control-link', str(control), '--data-link', str(data)],
                env=env, stdin=subprocess.PIPE, stdout=log, stderr=log)
            try:
                deadline = time.monotonic()+10
                while not (control.exists() and data.exists()):
                    assert child.poll() is None, 'Backend exited before PTYs appeared'
                    assert time.monotonic() < deadline, 'PTY startup timed out'
                    time.sleep(.02)
                fd = os.open(control, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                tty.setraw(fd)

                def command(text):
                    wire = (text+'\r').encode()
                    assert os.write(fd, wire) == len(wire)
                    response = bytearray()
                    deadline = time.monotonic()+3
                    while time.monotonic() < deadline:
                        if select.select([fd], [], [], .05)[0]:
                            try:
                                response.extend(os.read(fd, 8192))
                            except OSError as exc:
                                if exc.errno == errno.EAGAIN:
                                    continue
                                raise
                            if re.search(rb'(?:\r|\n)(?:OK|ERROR)\r?\n', response):
                                break
                    assert re.search(rb'(?:\r|\n)OK\r?\n', response), (text, bytes(response))
                    return bytes(response)

                def active_preset():
                    answer = command('AT+MS?')
                    assert ('+MS: '+carrier+',0,').encode() in answer, answer
                    if ceiling:
                        assert (',300,%d,300,%d' % (ceiling, ceiling)).encode() in answer, answer
                    assert b'+ES: 2,3,5' in command('AT+ES?')

                command('ATE0')
                active_preset()
                command('AT+MS=V34,0')
                command('AT+ES=3,0,2')
                command('ATZ0')
                active_preset()
                command('AT&F')
                assert b'+ES: 3,0,2' in command('AT+ES?')
                command('ATZ0')
                active_preset()
            finally:
                if fd is not None:
                    os.close(fd)
                if child.poll() is None:
                    child.terminate()
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
                child.stdin.close()
        text = (directory/'backend.log').read_text()
        assert 'refused; ignored' not in text, text
        assert 'profiles not loaded' not in text, text
        assert 'hook_request' not in text, 'No hook operation is expected'
    assert profile.read_bytes() == original, 'The shipped profile was modified'
    print(profile.name+': loaded, queried, ATZ restored, factory reset preserved; no line seized')


def main():
    root = Path(__file__).resolve().parents[1]
    backend = Path(sys.argv[1]).resolve() if len(sys.argv)>1 else root/'apple_usb_modem_coupler'
    check(backend, root/'profiles/apple-eicon-v32bis-9600.at', 'V32B', 9600)
    check(backend, root/'profiles/apple-eicon-v22bis.at', 'V22B', None)


if __name__ == '__main__':
    main()
