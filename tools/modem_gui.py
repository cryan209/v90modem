#!/usr/bin/env python3
"""Local live modem console. Python standard library only; --gui invokes this."""
import base64
import collections
import http.server
import json
import os
from pathlib import Path
import secrets
import plistlib
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
import webbrowser


def main():
    args = sys.argv[1:]
    web = bool(args and args[0] == "--web")
    if web:
        args.pop(0)
    if not args:
        raise SystemExit('Usage: tools/modem_gui.py ./sip_v90_modem [modem options]')
    if '--pty-link' in args:
        raise SystemExit('--gui requires split consoles; use --control-link and --data-link instead of --pty-link')
    stop = threading.Event()
    lock = threading.Lock()
    telemetry = {}
    streams = {name: collections.deque(maxlen=256) for name in ('at', 'data', 'log')}
    seq = {name: 0 for name in streams}
    ports = {}
    token = secrets.token_urlsafe(24)

    def record(name, data):
        with lock:
            for offset in range(0, len(data), 4096):
                seq[name] += 1
                streams[name].append([seq[name], base64.b64encode(data[offset:offset+4096]).decode()])

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', 0))
    udp.settimeout(.2)
    env = dict(os.environ, ME_GUI_PORT=str(udp.getsockname()[1]))
    temp = tempfile.TemporaryDirectory(prefix='modem-gui-')
    for name, flag in [('at', '--control-link'), ('data', '--data-link')]:
        if flag in args:
            ports[name] = args[args.index(flag)+1]
        else:
            ports[name] = str(Path(temp.name)/name)
            args += [flag, ports[name]]
    native = None
    native_binary = Path(__file__).resolve().parents[1]/'modem_gui_native'
    native_source = Path(__file__).with_name('modem_gui_native.swift')
    if not web and sys.platform == 'darwin' and (not native_binary.exists() or native_binary.stat().st_mtime < native_source.stat().st_mtime):
        with tempfile.TemporaryDirectory(prefix='modem-gui-build-') as cache:
            subprocess.run(['swiftc', '-module-cache-path', cache, '-O',
                str(Path(__file__).with_name('modem_gui_native.swift')), '-o', str(native_binary)], check=True)
    child = subprocess.Popen(args, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    fds = {}

    def reader():
        while not stop.is_set():
            try:
                payload, _ = udp.recvfrom(65535)
                value = json.loads(payload)
                with lock:
                    telemetry.clear()
                    telemetry.update(value, received=time.monotonic())
            except socket.timeout:
                pass
            except (ValueError, OSError):
                if stop.is_set():
                    break

    def log_reader():
        while data := child.stdout.readline(4096):
            record('log', data)

    def port_reader(name):
        while not stop.is_set() and child.poll() is None:
            try:
                fd = os.open(ports[name], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                tty.setraw(fd, termios.TCSANOW)
                fds[name] = fd
                break
            except OSError:
                time.sleep(.1)
        else:
            return
        while not stop.is_set():
            try:
                data = os.read(fd, 4096)
                if data:
                    record(name, data)
                else:
                    time.sleep(.02)
            except BlockingIOError:
                time.sleep(.02)
            except OSError:
                break

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def respond(self, code, data, kind='application/json'):
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/'+token+'/':
                self.respond(200, Path(__file__).with_name('modem_gui.html').read_bytes(), 'text/html; charset=utf-8')
            elif self.path == '/'+token+'/state':
                with lock:
                    value = dict(telemetry)
                    value['age'] = time.monotonic()-value.pop('received', time.monotonic())
                    value['streams'] = {k: list(v) for k, v in streams.items()}
                    value['ports'] = ports
                    value['exit'] = child.poll()
                self.respond(200, json.dumps(value).encode())
            else:
                self.respond(404, b'{}')

        def do_POST(self):
            # Capability URL + same-origin check: arbitrary websites cannot operate PTYs.
            origin = self.headers.get('Origin')
            expected = f'http://127.0.0.1:{self.server.server_port}'
            if self.path != '/'+token+'/send' or origin != expected:
                return self.respond(403, b'{}')
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 16384:
                    raise ValueError('Invalid message size')
                req = json.loads(self.rfile.read(length))
                name = req['port']
                data = base64.b64decode(req['data'], validate=True)
                if name not in fds:
                    raise ValueError('Port not ready')
                written = os.write(fds[name], data)
                if written != len(data):
                    raise ValueError(f'Only {written} of {len(data)} bytes sent')
                record(name, b'\n> '+data+b'\n')
                self.respond(200, b'{}')
            except (KeyError, ValueError, OSError) as e:
                self.respond(400, json.dumps({'error': str(e)}).encode())

    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    for target in [reader, log_reader, lambda: port_reader('at'), lambda: port_reader('data')]:
        threading.Thread(target=target, daemon=True).start()
    url = f'http://127.0.0.1:{server.server_port}/{token}/'
    print(f'Modem GUI: {url}', flush=True)
    if not web and sys.platform == 'darwin':
        bundle = native_binary.with_name('Modem GUI.app')
        macos = bundle/'Contents'/'MacOS'
        macos.mkdir(parents=True, exist_ok=True)
        executable = macos/native_binary.name
        if not executable.exists() or executable.stat().st_mtime < native_binary.stat().st_mtime:
            shutil.copy2(native_binary, executable)
        with (bundle/'Contents'/'Info.plist').open('wb') as info:
            plistlib.dump({'CFBundleExecutable': native_binary.name,
                'CFBundleIdentifier': 'nz.cryan.v90modem.gui', 'CFBundleName': 'Modem GUI',
                'CFBundlePackageType': 'APPL', 'NSPrincipalClass': 'NSApplication',
                'NSHighResolutionCapable': True}, info)
        native = subprocess.Popen([str(executable), url])
    else:
        webbrowser.open(url)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    def shutdown(*_):
        stop.set()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        while child.poll() is None and not stop.wait(.2):
            if native is not None and native.poll() is not None:
                break
    finally:
        stop.set()
        if native is not None and native.poll() is None:
            native.terminate()
            native.wait(timeout=5)
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        server.shutdown()
        server.server_close()
        udp.close()
        for fd in fds.values():
            os.close(fd)
        temp.cleanup()


if __name__ == '__main__':
    main()
