#!/usr/bin/env python3
"""Multilink PPP (RFC 1990) over several v90modem calls to the Eicon rig.

Each --link starts its own sip_v90_modem (own SIP account, SIP and RTP port),
dials, and on CONNECT hands the PTY to its own pppd with `multilink`.  The
first link to finish IPCP creates the bundle; later links join it (pppd logs
"Link attached to bundle").  Once the requested number of links are in the
bundle, an incompressible HTTP download and/or upload runs over the bundle
interface against tools/eicon/http_test_server.py on the answerer, checked
by SHA-256 and length.

Link spec, space-separated key=value:
  ext=17950 user=2906 ms=AT+MS=CLEAR,0 framing=ppp clearmode=1 law=alaw
  ext=7920  user=2907 ms=AT+MS=V90,0 role=analogue law=alaw
Optional per link: sip_port, rtp_port, delay (seconds after the previous
link joined before dialling), env=K=V (repeatable).

Per-link byte counts come from the PTY relay (the async PPP stream each modem
carried, both directions); the bundle timeline from the bundle interface's
counters every 5 s.  Evidence: summary.json, timeline.csv, link-N/.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import pty as ptymod
import re
import select
import shlex
import subprocess
import termios
import threading
import time
import tty

ROOT = Path(__file__).resolve().parents[1]
PAYLOAD_SEED = b'eicon-ppp-http-v1'          # matches http_test_server.py


def parse_link(spec, index):
    link = dict(ms='AT+MS=V90,0', law='alaw', role='analogue', framing=None,
                clearmode='0', delay='0', env=[])
    for tok in shlex.split(spec):
        k, _, v = tok.partition('=')
        if k == 'env':
            link['env'].append(v)
        else:
            link[k] = v
    for need in ('ext', 'user'):
        if need not in link:
            raise SystemExit('link %d needs %s=' % (index, need))
    link.setdefault('sip_port', str(5091 + index))
    link.setdefault('rtp_port', str(14910 + 20 * index))
    return link


class Link:
    def __init__(self, index, cfg, outdir, binary, common_env):
        self.i, self.cfg = index, cfg
        self.d = outdir / ('link-%d-%s' % (index, cfg['ext']))
        self.d.mkdir(parents=True)
        self.binary = binary
        self.common_env = common_env
        self.proc = self.pppd = None
        self.fd = None
        self.stop = threading.Event()
        self.bytes_from_modem = 0          # downstream: card -> us
        self.bytes_to_modem = 0            # upstream: us -> card
        self.result = dict(index=index, ext=cfg['ext'], ms=cfg['ms'],
                           framing=cfg.get('framing'), user=cfg['user'])

    def start_modem(self):
        c = self.cfg
        env = os.environ.copy()
        for k in ('ME_V34_SPAN_FLOW_LOG', 'ME_V90_ROLE', 'SIP_FORCE_PCMA',
                  'SIP_FORCE_PCMU', 'ME_DATA_FRAMING', 'ME_CLEARMODE'):
            env.pop(k, None)
        env.update(self.common_env)
        if c.get('role') == 'analogue':
            env['ME_V90_ROLE'] = 'analogue'
        if c.get('framing'):
            env['ME_DATA_FRAMING'] = c['framing']
        if c.get('clearmode') == '1':
            env['ME_CLEARMODE'] = '1'
        env['SIP_FORCE_PCMA' if c['law'] == 'alaw' else 'SIP_FORCE_PCMU'] = '1'
        env.update(VPCM_ME_VERBOSE='1', VPCM_G711_TAP_DIR=str(self.d.resolve()),
                   ME_DUMP_DIR=str(self.d.resolve()))
        for kv in c['env']:
            k, _, v = kv.partition('=')
            env[k] = v
        self.ptyname = '/tmp/mlppp-%d-%d' % (os.getpid(), self.i)
        self.log = (self.d / 'server.log').open('wb')
        self.proc = subprocess.Popen(
            [str(self.binary), '--sip-server', 'asterisk.net.cryan.nz',
             '--username', c['user'], '--password', c.get('password', c['user']),
             '--local-port', c['sip_port'], '--rtp-port', c['rtp_port'],
             '--pty-link', self.ptyname, '--verbose'],
            cwd=ROOT, env=env, stdout=self.log, stderr=self.log)
        for _ in range(100):
            if self.proc.poll() is not None:
                raise RuntimeError('modem exited before PTY opened')
            if os.path.exists(self.ptyname):
                break
            time.sleep(.1)
        self.fd = os.open(self.ptyname, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0
        a[2] |= termios.CLOCAL | termios.CREAD
        termios.tcsetattr(self.fd, termios.TCSANOW, a)

    def dial(self, timeout=150):
        rx = bytearray()

        def read_until(token, t):
            end = time.monotonic() + t
            while time.monotonic() < end:
                if select.select([self.fd], [], [], .2)[0]:
                    try:
                        rx.extend(os.read(self.fd, 65536))
                    except BlockingIOError:
                        continue
                    if token in rx:
                        return True
                    if any(x in rx for x in (b'NO CARRIER', b'NO DIAL', b'BUSY', b'NO ANSWER')):
                        return False
            return False

        time.sleep(3)          # registration
        for cmd in ('ATE0', self.cfg['ms']):
            os.write(self.fd, cmd.encode() + b'\r')
            if not read_until(b'OK', 3):
                raise RuntimeError('AT rejected: ' + cmd)
            del rx[:]
        t0 = time.monotonic()
        os.write(self.fd, ('ATD' + self.cfg['ext'] + '\r').encode())
        if not read_until(b'CONNECT', timeout):
            raise RuntimeError('no CONNECT: ' + bytes(rx).decode(errors='replace').strip())
        self.result['connect_seconds'] = round(time.monotonic() - t0, 1)
        m = re.search(rb'CONNECT (\d+)', rx)
        self.result['connect_rate'] = int(m.group(1)) if m else None
        tail = bytes(rx).split(b'CONNECT', 1)[-1]
        return tail.split(b'\n', 1)[-1] if b'\n' in tail else b''

    def start_pppd(self, tail):
        master, slave = ptymod.openpty()
        tty.setraw(slave)
        if tail:
            os.write(master, tail)

        def relay():
            while not self.stop.is_set():
                r, _, _ = select.select([self.fd, master], [], [], .2)
                try:
                    if self.fd in r:
                        data = os.read(self.fd, 65536)
                        if data:
                            self.bytes_from_modem += len(data)
                            os.write(master, data)
                    if master in r:
                        data = os.read(master, 65536)
                        if data:
                            self.bytes_to_modem += len(data)
                            os.write(self.fd, data)
                except (BlockingIOError, OSError):
                    continue

        threading.Thread(target=relay, daemon=True).start()
        self.pppd_log = self.d / 'pppd.log'
        out = self.pppd_log.open('wb')
        self.pppd = subprocess.Popen(
            ['/usr/sbin/pppd', 'notty', 'nodetach', 'local', 'noauth', 'nodefaultroute',
             'noproxyarp', 'noipv6', 'noccp', 'novj', 'nopersist', 'logfd', '2',
             'ipcp-accept-local', 'ipcp-accept-remote', 'lcp-echo-interval', '10',
             'lcp-echo-failure', '6', 'debug', 'multilink', 'mrru', '1500',
             '0.0.0.0:'] + self.cfg.get('pppd_extra', []),
            stdin=slave, stdout=slave, stderr=out)

    def ppp_text(self):
        try:
            return self.pppd_log.read_text(errors='replace')
        except FileNotFoundError:
            return ''

    def close(self):
        self.stop.set()
        for p in (self.pppd, self.proc):
            if p is not None and p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    p.kill()
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        text = (self.d / 'server.log').read_text(errors='replace') if (self.d / 'server.log').exists() else ''
        self.result['retrains'] = len(re.findall(r'retrain', text, re.I))
        losses = re.findall(r'pkt loss=(\d+)', text)
        self.result['rtp_loss_reports'] = [int(x) for x in losses]
        cc = re.findall(r'clear channel: tx (\d+) frames.*?rx (\d+) frames.*?(\d+) bad', text)
        if cc:
            self.result['hdlc_tx_frames'], self.result['hdlc_rx_frames'], self.result['hdlc_bad'] = map(int, cc[-1])
        self.result['bytes_from_modem'] = self.bytes_from_modem
        self.result['bytes_to_modem'] = self.bytes_to_modem
        self.result['modem_exit'] = self.proc.poll() if self.proc else None


def payload(size, seed=PAYLOAD_SEED):
    return hashlib.shake_256(seed).digest(size)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('output', type=Path)
    ap.add_argument('--link', action='append', required=True)
    ap.add_argument('--bytes', type=int, default=12 * 1024 * 1024)
    ap.add_argument('--tests', default='download,upload', help='download,upload,duplex (comma list)')
    ap.add_argument('--port', type=int, default=19890)
    ap.add_argument('--join-timeout', type=float, default=90)
    ap.add_argument('--binary', type=Path, default=ROOT / 'sip_v90_modem')
    ap.add_argument('--jb-ms', default='40')
    ap.add_argument('--hold', type=float, default=0, help='extra seconds to keep the bundle up after the tests')
    args = ap.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    common_env = {'ME_JB_MS': args.jb_ms} if args.jb_ms else {}
    links = [Link(i, parse_link(s, i), out, args.binary.resolve(), common_env)
             for i, s in enumerate(args.link)]
    summary = dict(links=[], tests={}, ok=False, bytes=args.bytes)
    t_start = time.monotonic()
    timeline_stop = threading.Event()
    bundle = dict(ifname=None, peer=None)
    events = []

    def event(msg):
        stamp = round(time.monotonic() - t_start, 1)
        events.append((stamp, msg))
        print('[%7.1f] %s' % (stamp, msg), flush=True)

    def timeline():
        with (out / 'timeline.csv').open('w') as f:
            f.write('t,bundle_rx,bundle_tx,' + ','.join(
                'l%d_down,l%d_up' % (l.i, l.i) for l in links) + '\n')
            while not timeline_stop.wait(5):
                rx = tx = ''
                if bundle['ifname']:
                    base = Path('/sys/class/net') / bundle['ifname'] / 'statistics'
                    try:
                        rx = (base / 'rx_bytes').read_text().strip()
                        tx = (base / 'tx_bytes').read_text().strip()
                    except OSError:
                        pass
                f.write('%.1f,%s,%s,%s\n' % (time.monotonic() - t_start, rx, tx, ','.join(
                    '%d,%d' % (l.bytes_from_modem, l.bytes_to_modem) for l in links)))
                f.flush()

    threading.Thread(target=timeline, daemon=True).start()
    try:
        joined = 0
        for l in links:
            time.sleep(float(l.cfg['delay']))
            event('link %d: dialling %s (%s%s)' % (l.i, l.cfg['ext'], l.cfg['ms'],
                  ', ' + l.cfg['framing'] if l.cfg.get('framing') else ''))
            try:
                l.start_modem()
                tail = l.dial()
            except Exception as exc:
                l.result['error'] = str(exc)
                event('link %d: %s' % (l.i, exc))
                continue
            event('link %d: CONNECT %s after %.1f s' % (l.i, l.result['connect_rate'], l.result['connect_seconds']))
            l.start_pppd(tail)
            end = time.monotonic() + args.join_timeout
            while time.monotonic() < end:
                text = l.ppp_text()
                if l.pppd.poll() is not None:
                    l.result['error'] = 'pppd exited'
                    break
                # pppd multilink.c: "Link attached to <ifname>" on joining
                if re.search(r'Link attached to (\S+)', text):
                    l.result['joined'] = 'attached'
                    break
                m = re.search(r'remote IP address (\S+)', text)
                if m:
                    l.result['joined'] = 'created'
                    bundle['peer'] = m.group(1)
                    m2 = re.search(r'Using interface (\S+)', text)
                    bundle['ifname'] = m2.group(1) if m2 else None
                    break
                time.sleep(.5)
            if 'joined' in l.result:
                joined += 1
                l.result['join_seconds'] = round(time.monotonic() - t_start, 1)
                event('link %d: %s bundle (%d links)' % (l.i, l.result['joined'], joined))
            else:
                l.result.setdefault('error', 'did not join bundle')
                event('link %d: %s' % (l.i, l.result['error']))
        summary['links_joined'] = joined
        summary['bundle'] = dict(bundle)
        if not bundle['peer']:
            raise RuntimeError('no bundle formed')
        time.sleep(3)
        ping = subprocess.run(['ping', '-n', '-c', '5', '-i', '1', '-W', '5', bundle['peer']],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
        (out / 'ping.log').write_bytes(ping.stdout)
        m = re.search(rb'rtt min/avg/max/mdev = ([\d.]+)/([\d.]+)/([\d.]+)', ping.stdout)
        summary['ping'] = dict(status=ping.returncode,
                               avg_ms=float(m.group(2)) if m else None)
        url = 'http://%s:%d' % (bundle['peer'], args.port)
        up_file = out / 'upload.bin'
        up_file.write_bytes(payload(args.bytes, b'eicon-mlppp-upload-v1'))
        up_sha = hashlib.sha256(up_file.read_bytes()).hexdigest()

        def download(tag):
            dst = out / ('%s-download.bin' % tag)
            hdr = out / ('%s-download.headers' % tag)
            t0 = time.monotonic()
            r = subprocess.run(['curl', '-sS', '--max-time', '7200', '-D', str(hdr), '-o', str(dst),
                                '-w', '%{http_code} %{size_download} %{speed_download}',
                                '%s/download/%d' % (url, args.bytes)],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            dt = time.monotonic() - t0
            got = dst.read_bytes() if dst.exists() else b''
            want = payload(args.bytes)
            res = dict(curl=r.stdout.decode(errors='replace').strip(), seconds=round(dt, 1),
                       bytes=len(got), exact=got == want,
                       kBps=round(len(got) / dt / 1000, 2), kbps=round(len(got) * 8 / dt / 1000, 1))
            if dst.exists():
                dst.unlink()                       # 12 MB each; the hash is the evidence
            return res

        def upload(tag):
            t0 = time.monotonic()
            r = subprocess.run(['curl', '-sS', '--max-time', '7200', '-H', 'Expect:',
                                '--data-binary', '@' + str(up_file), '%s/upload' % url],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            dt = time.monotonic() - t0
            res = dict(seconds=round(dt, 1), response=r.stdout.decode(errors='replace').strip())
            try:
                j = json.loads(r.stdout)
                res.update(bytes=j['bytes'], exact=j['sha256'] == up_sha and j['bytes'] == args.bytes)
            except Exception:
                res.update(bytes=0, exact=False)
            res.update(kBps=round(res['bytes'] / dt / 1000, 2), kbps=round(res['bytes'] * 8 / dt / 1000, 1))
            return res

        def link_bytes():
            return [(l.bytes_from_modem, l.bytes_to_modem) for l in links]

        for test in [t for t in args.tests.split(',') if t]:
            before = link_bytes()
            event('test %s: start (%d bytes)' % (test, args.bytes))
            if test == 'download':
                res = download('dl')
            elif test == 'upload':
                res = upload('ul')
            elif test == 'duplex':
                box = {}
                th = threading.Thread(target=lambda: box.update(up=upload('dx')))
                th.start()
                box['down'] = download('dx')
                th.join()
                res = box
            else:
                raise SystemExit('unknown test ' + test)
            after = link_bytes()
            res['per_link'] = [dict(index=l.i, ext=l.cfg['ext'], down=a[0] - b[0], up=a[1] - b[1],
                                    alive=l.proc.poll() is None if l.proc else False)
                               for l, b, a in zip(links, before, after)]
            summary['tests'][test] = res
            event('test %s: %s' % (test, json.dumps(res)))
        if args.hold:
            time.sleep(args.hold)
        summary['ok'] = all(
            (r.get('exact') if 'exact' in r else (r['up'].get('exact') and r['down'].get('exact')))
            for r in summary['tests'].values()) and bool(summary['tests'])
    except Exception as exc:
        summary['error'] = str(exc)
        event('ERROR ' + str(exc))
    finally:
        timeline_stop.set()
        for l in reversed(links):
            l.close()
            summary['links'].append(l.result)
        summary['links'].sort(key=lambda r: r['index'])
        summary['events'] = events
        summary['elapsed_seconds'] = round(time.monotonic() - t_start, 1)
        try:
            (out / 'upload.bin').unlink()
        except FileNotFoundError:
            pass
        (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(dict(ok=summary['ok'], tests=summary['tests'], links=summary['links']), indent=1), flush=True)


if __name__ == '__main__':
    main()
