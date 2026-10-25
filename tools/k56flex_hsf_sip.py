#!/usr/bin/env python3
"""The native legacy HSF K56flex client on a real SIP call.

sip_v90_modem places the call (ATD<ext>) with ME_AUDIO_RELAY set, so its own
DSP is bypassed and the call's G.711 is exchanged byte-exact with this
process; every 160 received codewords go through cruft's original Flex-only
HSF client (native.hsf.k56_client, as tools/k56flex_hsf_pair.py drives it)
and its 160 are sent back. RTP is the clock: a reply that is late is replaced
by silence in the engine and counted there ([RELAY] lines in server.log), so
the per-frame processing time is recorded here too.

This is a reference peer for interop work (a known-good K56flex client against
real hardware), not part of the modem. Run it from the wired rig (CT 117):

    python3 tools/k56flex_hsf_sip.py OUT --cruft /root/cruft --ext 7910 --seconds 90
"""
import argparse
import json
import os
import select
import socket
import subprocess
import sys
import termios
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('output', type=Path)
    ap.add_argument('--cruft', type=Path, default=ROOT.parent / 'cruft')
    ap.add_argument('--ext', default='7910')
    ap.add_argument('--account', default='2900')
    ap.add_argument('--sip-server', default='asterisk.net.cryan.nz',
                    help='registrar, or host:port of a peer to INVITE directly (local tests)')
    ap.add_argument('--seconds', type=int, default=90)
    ap.add_argument('--dac-taps', type=int, default=129)
    ap.add_argument('--max-upstream-rate', type=int, default=9600)
    ap.add_argument('--local-port', type=int, default=5079)
    ap.add_argument('--rtp-port', type=int, default=14610)
    args = ap.parse_args()

    sys.path.insert(0, str(args.cruft.resolve()))
    from native.hsf.k56_client import K56Client, PCMUStream

    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    sock_path = '/tmp/k56hsf-%d.sock' % os.getpid()
    pty = '/tmp/k56hsf-pty-%d' % os.getpid()
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(1)

    client = K56Client(modulation='flex', max_upstream_rate=args.max_upstream_rate,
                       dac_taps=args.dac_taps)
    stream = PCMUStream(client)
    line_rx, client_tx, dte = bytearray(), bytearray(), bytearray()
    states, frame_ms = [], []
    summary = dict(extension=args.ext, connected=False, end_reason='duration limit')

    env = dict(os.environ, ME_AUDIO_RELAY=sock_path, SIP_FORCE_PCMU='1', ME_MODE='v34')
    proc = fd = conn = None
    try:
        with (out / 'server.log').open('wb') as log:
            proc = subprocess.Popen(
                [str(ROOT / 'sip_v90_modem'), '--sip-server', args.sip_server,
                 '--username', args.account,
                 '--password', os.environ.get('EICON_SIP_PASSWORD', args.account),
                 '--local-port', str(args.local_port), '--rtp-port', str(args.rtp_port),
                 '--pty-link', pty, '--verbose'], cwd=ROOT, env=env, stdout=log, stderr=log)
            for _ in range(100):
                if os.path.exists(pty):
                    break
                if proc.poll() is not None:
                    raise RuntimeError('modem exited before its PTY opened')
                time.sleep(.1)
            fd = os.open(pty, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            a = termios.tcgetattr(fd)
            a[0] = a[1] = a[3] = 0
            a[2] |= termios.CLOCAL | termios.CREAD
            termios.tcsetattr(fd, termios.TCSANOW, a)
            time.sleep(3)
            os.write(fd, ('ATD%s\r' % args.ext).encode())
            srv.settimeout(30)
            conn, _ = srv.accept()
            summary['connected'] = True
            start = time.monotonic()
            pending = bytearray()
            last_state = None
            while time.monotonic() - start < args.seconds:
                r, _, _ = select.select([conn, fd], [], [], 1.0)
                if fd in r:
                    try:
                        dte.extend(os.read(fd, 4096))
                    except (BlockingIOError, OSError):
                        pass
                if conn not in r:
                    continue
                data = conn.recv(4096)
                if not data:
                    summary['end_reason'] = 'engine closed the relay (call ended)'
                    break
                pending.extend(data)
                while len(pending) >= 160:
                    block = bytes(pending[:160])
                    del pending[:160]
                    t0 = time.monotonic()
                    reply = stream.exchange(block)
                    frame_ms.append((time.monotonic() - t0) * 1000)
                    conn.sendall(reply)
                    line_rx.extend(block)
                    client_tx.extend(reply)
                    if client.pump and len(line_rx) % 8000 == 0:
                        st = client.pump.state()
                        brief = {k: st.get(k) for k in ('data', 'tx_label', 'rx_label') if k in st}
                        brief['carrier'] = client.pump.env.datapump_carrier
                        if brief != last_state:
                            states.append(dict(t=len(line_rx) / 8000, **brief))
                            last_state = brief
                if client.failure:
                    summary['end_reason'] = client.failure
                    break
    finally:
        if conn:
            conn.close()
        if fd is not None:
            try:
                os.write(fd, b'+++')
                time.sleep(1.2)
                os.write(fd, b'ATH\r')
                time.sleep(1)
                dte.extend(os.read(fd, 4096))
            except OSError:
                pass
            os.close(fd)
        if proc:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
        srv.close()
        if os.path.exists(sock_path):
            os.unlink(sock_path)
        (out / 'line-rx.g711').write_bytes(line_rx)
        (out / 'client-tx.g711').write_bytes(client_tx)
        (out / 'dte.bin').write_bytes(dte)
        (out / 'client-events.json').write_text(json.dumps(client.events, indent=2, default=str) + '\n')
        (out / 'client-states.json').write_text(json.dumps(states, indent=2, default=str) + '\n')
        frame_ms.sort()
        summary.update(
            audio_seconds=len(line_rx) / 8000,
            flex_agreed=getattr(client, 'flex_agreed', None),
            selected_modulation=getattr(client, 'selected_modulation', None),
            failure=client.failure,
            frame_ms_p50=frame_ms[len(frame_ms) // 2] if frame_ms else None,
            frame_ms_p99=frame_ms[int(len(frame_ms) * .99)] if frame_ms else None,
            frame_ms_max=frame_ms[-1] if frame_ms else None)
        (out / 'summary.json').write_text(json.dumps(summary, indent=2, default=str) + '\n')
        print(json.dumps(summary, indent=2, default=str))


if __name__ == '__main__':
    main()
