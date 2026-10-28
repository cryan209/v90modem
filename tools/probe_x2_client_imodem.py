#!/usr/bin/env python3
"""Our x2 analogue client dials the original Ie030002 I-modem as an x2 server.

The I-modem firmware runs in the sibling courier-emu (its I-modem pair
worker, answering); our engine is v90_engine_peer --call with ME_MODE=x2.
PCMU is byte-exact between them; neither side's response is recorded.

Both DTEs are captured: the I-modem's serial session in imodem-result.json
and ours in engine-dte.bin. --engine-send writes a line to our PTY every 2 s
of wall time after our CONNECT (ten times), --answer-send makes the I-modem
send one after its CONNECT. Run from a clean courier-emu export (its
Ie030002.nac is not tracked; link it in), for example:

  EMU=$(mktemp -d); git -C ../courier-emu archive f1ccd92 | tar -x -C $EMU
  ln -s $PWD/../courier-emu/Ie030002.nac $EMU/
  ../courier-emu/.venv/bin/python tools/probe_x2_client_imodem.py \\
      --emu $EMU --output artifacts/x2-client-imodem --fast \\
      --engine-at 'AT\\N0' --engine-send 'OUR-X2-CLIENT-0123456789!' \\
      --answer-send 'IMODEM-X2-SERVER-0123456789'

Until its echo canceller settles, this I-modem's ANSam carries an impairment
8-10 dB below the tone; V.8's ANSam detector accepts that (notch ratio 3, in
v8.c), so no ME_ANS_NOTCH_RATIO is needed (docs/x2_implementation.md).
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--emu', type=Path, default=Path(os.environ.get('X2_HOST_EMU_ROOT', ROOT.parent / 'courier-emu')))
    p.add_argument('--engine', type=Path, default=ROOT / 'v90_engine_peer')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--instructions', type=int, default=1_200_000_000)
    p.add_argument('--imodem-arg', action='append', default=[],
                   help="extra courier-emu probe_imodem_pair worker argument "
                        "(--post-call-diagnostics, --trace-digital-path, ...)")
    p.add_argument('--imodem-settings', default='S54=0S58=48&A3&B1Q0',
                   help='I-modem AT settings: S58=48 keeps x2, server and symmetric enabled, V.90 off')
    p.add_argument('--engine-at', default='', help='AT command sent to our PTY before the call')
    p.add_argument('--engine-send', default='', help='line our DTE sends after CONNECT')
    p.add_argument('--answer-send', default='', help='line the I-modem DTE sends after CONNECT')
    p.add_argument('--fast', action='store_true', help='do not pace the engine to 8000 samples/s')
    a = p.parse_args()
    a.output, a.emu, a.engine = a.output.resolve(), a.emu.resolve(), a.engine.resolve()
    sys.path.insert(0, str(a.emu))
    os.environ.setdefault('COURIER_LINE_FRAME_MS', '20')
    os.chdir(a.emu)
    from tools import probe_v90modem_closed_loop as cl
    from tools import probe_imodem_pair as pair

    class CallingPeer(cl.EnginePeer):
        def start(self):
            if self.process is not None:
                return
            self.log = (self.output / 'engine.log').open('wb')
            self.rx = (self.output / 'engine-rx.g711').open('wb')
            self.tx = (self.output / 'engine-tx.g711').open('wb')
            env = os.environ.copy()
            env['ME_MODE'] = 'x2'
            self.environment = {k: v for k, v in env.items() if k.startswith('ME_')}
            self.engine_sha256 = hashlib.sha256(self.engine.read_bytes()).hexdigest()
            # Unique per process: two probes (tests/slow.list runs them in
            # parallel) with the same output name crossed DTEs.
            pty = '/private/tmp/%s-%d-pty' % (self.output.name, os.getpid())
            self.process = subprocess.Popen([str(self.engine), pty, '--call'],
                                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=self.log, env=env)
            for _ in range(200):
                if os.path.exists(pty):
                    break
                time.sleep(0.02)
            fd = os.open(pty, os.O_RDWR | os.O_NOCTTY)
            if a.engine_at:
                os.write(fd, (a.engine_at + '\r').encode())
                reply, end = b'', time.monotonic() + 3
                while b'OK' not in reply and b'ERROR' not in reply and time.monotonic() < end:
                    if select.select([fd], [], [], 0.1)[0]:
                        reply += os.read(fd, 256)
                if b'OK' not in reply:
                    raise SystemExit(f'engine refused {a.engine_at!r}: {reply!r}')
            threading.Thread(target=self.dte, args=(fd,), daemon=True).start()
            self.origin = time.monotonic()

        def dte(self, fd):
            out = (self.output / 'engine-dte.bin').open('wb')
            seen, last, left = b'', None, 10
            while self.process and self.process.poll() is None:
                if select.select([fd], [], [], 0.2)[0]:
                    try:
                        data = os.read(fd, 4096)
                    except OSError:
                        break
                    out.write(data); out.flush(); seen += data
                if (a.engine_send and left and b'CONNECT' in seen
                        and (last is None or time.monotonic() - last > 2)):
                    os.write(fd, a.engine_send.encode()); last = time.monotonic(); left -= 1

    a.output.mkdir(parents=True, exist_ok=False)
    peer = CallingPeer(a.output, a.engine.resolve(), a.fast)
    pair.G711Peer = lambda path, listen: peer
    saved = sys.argv
    sys.argv = ['probe_x2_client_imodem', '--worker', 'answer', '--socket', 'unused',
                '--result', str(a.output / 'imodem-result.json'), '--protocol', 'x2',
                '--instructions', str(a.instructions), '--settings', a.imodem_settings,
                '--nvram', str(a.emu / 'artifacts/imodem-pair-x2-full-rate-routed-20261002/nvram-230400-switch2.sav'),
                '--trace-negotiation']
    if a.answer_send:
        sys.argv += ['--answer-send', a.answer_send]
    sys.argv += a.imodem_arg
    try:
        pair.run_side(pair.arguments())
    finally:
        sys.argv = saved
        peer.stop()
    result = json.loads((a.output / 'imodem-result.json').read_text())
    imodem = ''.join(s['text'] for s in result['serial_session'] if s['direction'].startswith('received'))
    ours = (a.output / 'engine-dte.bin').read_bytes()
    summary = {'imodem_dte': imodem, 'engine_dte': ours.decode('latin-1'),
               'imodem_connect': 'CONNECT' in imodem, 'engine_connect': b'CONNECT' in ours,
               'upstream_delivered': bool(a.engine_send) and a.engine_send in imodem,
               'downstream_delivered': bool(a.answer_send)
                   and a.answer_send.encode() in bytes(c & 0x7f for c in ours),
               'engine': peer.status()}
    (a.output / 'call.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(f"I-modem DTE: {imodem!r}")
    print(f"our DTE:     {ours[:200]!r}")
    print(f"upstream delivered: {summary['upstream_delivered']}, downstream delivered: {summary['downstream_delivered']}")
    return 0 if summary['imodem_connect'] and summary['engine_connect'] else 1


if __name__ == '__main__':
    sys.exit(main())
