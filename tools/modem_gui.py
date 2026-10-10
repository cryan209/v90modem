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


LOOPBACK_PROFILES = {
    'v22-1200': ('V.22 · 1200', 'V22B,0,1200,1200'),
    'v22': ('V.22bis · 2400', 'V22B,0,2400,2400'),
    'v32': ('V.32 · 9600', 'V32,0,9600,9600'),
    'v32bis': ('V.32bis · 14400', 'V32B,0,14400,14400'),
    'v34-9600': ('V.34 · 9600', 'V34,0,9600,9600'),
    'v34-21600': ('V.34 · 21600', 'V34,0,21600,21600'),
}

def local_sip_ports():
    # Reserve both sockets together so the allocator cannot return one twice.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as a, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as b:
        a.bind(('127.0.0.1', 0)); b.bind(('127.0.0.1', 0))
        return a.getsockname()[1], b.getsockname()[1]

def main():
    args = sys.argv[1:]
    web = bool(args and args[0] == "--web")
    if web:
        args.pop(0)
    if not args:
        raise SystemExit('Usage: tools/modem_gui.py ./sip_v90_modem [modem options]')
    loopback = '--loopback' in args
    if loopback:
        args.remove('--loopback')
        conflicts = ('--sip-server', '--username', '--password', '--local-port', '--bind-addr', '--rtp-port', '--auto-answer', '--profile')
        if any(flag in args for flag in conflicts):
            raise SystemExit('Loopback owns SIP/network settings; omit server, credentials, ports, profile and auto-answer options')
    if '--pty-link' in args:
        raise SystemExit('--gui requires split consoles; use --control-link and --data-link instead of --pty-link')
    stop = threading.Event()
    lock = threading.Lock()
    telemetry = {}
    peer_telemetry = {}
    test = {"busy": False, "status": "Ready · local peer echoes serial bytes", "echoed": 0}
    condition = threading.Condition(lock)
    test_lock = threading.Lock()
    streams = {name: collections.deque(maxlen=256) for name in ('at', 'data', 'log')}
    if loopback: streams['peer_at'] = collections.deque(maxlen=256)
    seq = {name: 0 for name in streams}
    ports = {}
    token = secrets.token_urlsafe(24)

    def record(name, data):
        with lock:
            for offset in range(0, len(data), 4096):
                seq[name] += 1
                streams[name].append([seq[name], base64.b64encode(data[offset:offset+4096]).decode()])
            condition.notify_all()

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', 0))
    udp.settimeout(.2)
    env = dict(os.environ, ME_GUI_PORT=str(udp.getsockname()[1]))
    temp = tempfile.TemporaryDirectory(prefix='modem-gui-')
    peer = None; peer_udp = None
    if loopback:
        env.setdefault('ME_DATA_FRAMING', 'v14')
        mode = args[args.index('--mode')+1] if '--mode' in args else 'v22'
        if mode not in ('v22', 'v22-1200', 'v32', 'v32bis', 'v34'):
            raise SystemExit('GUI loopback supports v22, v22-1200, v32, v32bis and v34; PCM roles require a different test rig')
        if '--mode' not in args: args += ['--mode', mode]
        caller_port, peer_port = local_sip_ports()
        args += ['--bind-addr', '127.0.0.1', '--local-port', str(caller_port), '--sip-server', f'127.0.0.1:{peer_port}', '--auto-answer', '0']
        ports['peer_at'] = str(Path(temp.name)/'peer-at')
        ports['peer_data'] = str(Path(temp.name)/'peer-data')
        peer_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        peer_udp.bind(('127.0.0.1', 0)); peer_udp.settimeout(.2)
        peer_env = dict(env, ME_GUI_PORT=str(peer_udp.getsockname()[1]))
        peer_args = [args[0], '--mode', mode, '--bind-addr', '127.0.0.1', '--local-port', str(peer_port), '--auto-answer', '1', '--control-link', ports['peer_at'], '--data-link', ports['peer_data']]
        test['profiles'] = [[key, label] for key,(label,_) in LOOPBACK_PROFILES.items()]

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
    if loopback: peer = subprocess.Popen(peer_args, env=peer_env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        child = subprocess.Popen(args, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    except Exception:
        if peer is not None: peer.terminate(); peer.wait(timeout=5)
        raise
    fds = {}
    generation = 0
    port_stop = threading.Event()
    port_threads = []
    lifecycle = threading.Lock()

    def reader(sock=udp, destination=telemetry):
        while not stop.is_set():
            try:
                payload, _ = sock.recvfrom(65535)
                value = json.loads(payload)
                with lock:
                    destination.clear()
                    value["epoch"] = value.get("epoch", 0)+generation*1000000
                    destination.update(value, received=time.monotonic())
            except socket.timeout:
                pass
            except (ValueError, OSError):
                if stop.is_set():
                    break

    def log_reader(process=child, prefix=b''):
        while data := process.stdout.readline(4096):
            record('log', prefix+data)

    def port_reader(name, process, halt):
        while not stop.is_set() and not halt.is_set() and process.poll() is None:
            try:
                fd = os.open(ports[name], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                tty.setraw(fd, termios.TCSANOW)
                fds[name] = fd
                break
            except OSError:
                time.sleep(.1)
        else:
            return
        while not stop.is_set() and not halt.is_set():
            try:
                data = os.read(fd, 4096)
                if data:
                    if name == 'peer_data':
                        # Echo at most one 4 KiB block, with no growing queue.
                        pending = memoryview(data)
                        while pending and not stop.is_set() and not halt.is_set():
                            try:
                                n = os.write(fd, pending); pending = pending[n:]
                                with lock: test['echoed'] += n
                            except BlockingIOError:
                                stop.wait(.02)
                    else:
                        record(name, data)
                else:
                    time.sleep(.02)
            except BlockingIOError:
                time.sleep(.02)
            except OSError:
                break

    def at_transaction(name, command):
        with condition:
            start = seq[name]
            if name not in fds: raise ValueError('Local modem interfaces are still starting')
            payload = (command+'\r').encode()
            if os.write(fds[name], payload) != len(payload): raise ValueError('Partial AT command write')
        if name == 'at': record(name, b'\n> '+payload+b'\n')
        deadline = time.monotonic()+5
        with condition:
            while not stop.is_set():
                response = b''.join(base64.b64decode(chunk) for ident,chunk in streams[name] if ident > start)
                if b'ERROR' in response: raise ValueError(f'{name}: {command} rejected')
                if b'OK' in response: return
                remaining = deadline-time.monotonic()
                if remaining <= 0: raise ValueError(f'{name}: AT response timed out')
                condition.wait(min(.2,remaining))
        raise ValueError('Loopback stopped')

    def launch_ports():
        nonlocal port_stop, port_threads
        port_stop = threading.Event(); port_threads = []
        names = [('at',child),('data',child)]
        if peer is not None: names += [('peer_at',peer),('peer_data',peer)]
        for name,process in names:
            thread = threading.Thread(target=port_reader,args=(name,process,port_stop),daemon=True)
            thread.start(); port_threads.append(thread)

    def terminate_pair():
        for process in (child,peer):
            if process is not None and process.poll() is None: process.terminate()
        for process in (child,peer):
            if process is not None:
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired: process.kill(); process.wait()

    def start_test(profile):
        nonlocal child, peer, generation
        try:
            # Test isolation: a profile starts with fresh DSP/SIP instances.
            # This does not reset or alter the production receiver algorithms.
            with lifecycle:
                port_stop.set()
                for thread in port_threads: thread.join(timeout=1)
                terminate_pair()
                for fd in list(fds.values()): os.close(fd)
                fds.clear()
                if stop.is_set(): raise ValueError('Loopback stopped')
                with lock:
                    generation += 1; telemetry.clear(); peer_telemetry.clear()
                    test['status'] = 'Starting fresh local pair · '+LOOPBACK_PROFILES[profile][0]
                child = subprocess.Popen(args,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
                peer = subprocess.Popen(peer_args,env=peer_env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
                for process,prefix in [(child,b''),(peer,b'[peer] ')]:
                    threading.Thread(target=log_reader,args=(process,prefix),daemon=True).start()
                launch_ports()
                deadline = time.monotonic()+5
                while not stop.is_set() and not all(name in fds for name in ('at','data','peer_at','peer_data')):
                    if time.monotonic()>deadline: raise ValueError('Local modem interfaces did not start')
                    stop.wait(.05)
                at_transaction('peer_at', 'AT+MS='+LOOPBACK_PROFILES[profile][1])
                at_transaction('at', 'AT+MS='+LOOPBACK_PROFILES[profile][1])
                payload = b'ATD6004\r'
                if os.write(fds['at'], payload) != len(payload): raise ValueError('Partial dial command')
                record('at', b'\n> '+payload+b'\n')
                with lock: test['status'] = 'Dialling local echo peer · '+LOOPBACK_PROFILES[profile][0]
        except (ValueError,OSError) as error:
            with lock: test['status'] = str(error)
        finally:
            with lock: test['busy'] = False
            test_lock.release()

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
                    value['exit'] = None if test['busy'] else child.poll()
                    value['loopback'] = dict(test, ready=all(name in fds for name in ('at','data','peer_at','peer_data')), peer_exit=peer.poll(), peer_state=peer_telemetry.get('state', 0)) if peer is not None else None
                self.respond(200, json.dumps(value).encode())
            else:
                self.respond(404, b'{}')

        def do_POST(self):
            # Capability URL + same-origin check: arbitrary websites cannot operate PTYs.
            origin = self.headers.get('Origin')
            expected = f'http://127.0.0.1:{self.server.server_port}'
            if self.path not in ('/'+token+'/send', '/'+token+'/loopback') or origin != expected:
                return self.respond(403, b'{}')
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 16384:
                    raise ValueError('Invalid message size')
                req = json.loads(self.rfile.read(length))
                if self.path.endswith('/loopback'):
                    profile = req.get('profile')
                    if not loopback or profile not in LOOPBACK_PROFILES: raise ValueError('Unknown loopback profile')
                    if not test_lock.acquire(blocking=False): raise ValueError('Loopback setup is already running')
                    with lock:
                        if telemetry.get('state', 0) != 0 or peer_telemetry.get('state', 0) != 0 or peer.poll() is not None:
                            test_lock.release(); raise ValueError('Hang up first; local peer must be running')
                        test.update(busy=True, status='Configuring both local modems', echoed=0)
                    threading.Thread(target=start_test,args=(profile,),daemon=True).start()
                    return self.respond(200, b'{}')
                name = req['port']
                if name not in ('at','data'): raise ValueError('Use the caller AT or serial interface')
                if test.get('busy'): raise ValueError('Wait for loopback setup to finish')
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
    for target in [reader, log_reader]:
        threading.Thread(target=target, daemon=True).start()
    if loopback:
        for target in [lambda: reader(peer_udp,peer_telemetry), lambda: log_reader(peer,b'[peer] ')]:
            threading.Thread(target=target,daemon=True).start()
    launch_ports()
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
        while not stop.wait(.2):
            if not test["busy"] and child.poll() is not None: break
            if not test["busy"] and peer is not None and peer.poll() is not None:
                record('log', b'Local loopback peer exited\n'); break
            if native is not None and native.poll() is not None:
                break
    finally:
        stop.set()
        if native is not None and native.poll() is None:
            native.terminate()
            native.wait(timeout=5)
        with lifecycle:
            port_stop.set()
            terminate_pair()
            for thread in port_threads: thread.join(timeout=1)
        if peer_udp is not None: peer_udp.close()
        server.shutdown()
        server.server_close()
        udp.close()
        for fd in fds.values():
            os.close(fd)
        temp.cleanup()


if __name__ == '__main__':
    main()
