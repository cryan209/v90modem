#!/usr/bin/env python3
"""slmodemd as one fixed caller against several answerers, graded the same way.

Run ON the Proxmox host (needs `pct`).  slmodemd (CT 100, rig build, account
6003, the qc_slm_pve.sh settings) dials each target in turn with identical AT
settings, so a call to our server can be read against the same client's calls
to the hardware answerers:

  ours  6002  our sip_v90_modem in CT 117 (power-on offer, mu-law leg)
  7900  Eicon Diva digital answerer, echo menu  ("Select: " -> 1 -> ECHO:)
  7910  MultiTech ISI digital answerer, PPP on CONNECT (LCP echo test)
  6311, 6312  RasFinder analogue RAS behind the AudioCodes FXS ("login:")

Each call records time to CONNECT, the CONNECT line, drops during the hold,
and a target-specific data grade, and keeps slmodemd's -d9 log (and our
server's log and taps for `ours`) under <out>/<call>/.  Targets are visited
round-robin so redials to one controller are minutes apart.

  slm_targets.py /root/slmx/<batch> --rounds 3 [--targets ours,7900,...] [--ms 90]

--caller ours swaps the caller for our modem (CT 117, account 6002, mu-law leg,
V.90 analogue role towards 7900/7910) with the same targets and probes, so the
hardware answerers can be read against both clients.
"""
import argparse
import json
import os
import re
import select
import subprocess
import time
from pathlib import Path

CT_SLM, CT_OURS = '100', '117'
SLM_ACCT, OUR_ACCT = '6003', '6002'
OUR_PORT, OUR_RTP = 5074, 14100
PTY = '/tmp/v90slmx'


def sx(cmd, ct=CT_SLM, check=False):
    return subprocess.run(['pct', 'exec', ct, '--', 'bash', '-c', cmd],
                          capture_output=True, text=True, check=check).stdout


def ox(cmd):
    return sx(cmd, CT_OURS)


def kill_ours():
    ox("pkill -f '[s]ip_v90_modem.*--local-port %d'; pkill -f '[c]at %s'; "
       "sleep 1; true" % (OUR_PORT, PTY))


def start_ours(cdir, env, capture=True):
    kill_ours()
    ox('mkdir -p %s; rm -f %s' % (cdir, PTY))
    ox('cd /root/v90modem && ( env SIP_FORCE_PCMU=1 ME_LAPM_XID_OPTION_OCTETS=auto '
       'VPCM_G711_TAP_DIR=%s %s nohup timeout 400 ./sip_v90_modem --sip-server '
       'asterisk.net.cryan.nz --username %s --password %s --local-port %d --rtp-port %d '
       '--pty-link %s --verbose > %s/server.log 2>&1 < /dev/null & ); sleep 5'
       % (cdir, env, OUR_ACCT, OUR_ACCT, OUR_PORT, OUR_RTP, PTY, cdir))
    if capture:
        ox("stty -F %s raw -echo; ( cat %s > %s/pty-rx.bin 2>/dev/null < /dev/null & ); true"
           % (PTY, PTY, cdir))


def start_slm():
    sx("pkill '[s]lmodemd'; pkill '[d]-modem'; sleep 1; rm -f /dev/ttySL0")
    sx("cd /root/D-Modem-rig; rm -f /tmp/slm.log; ( env SLMODEMD_V8_OPEN_STUB=0 "
       "DM_RESAMPLER=sinc DM_UP_RESAMPLER=sinc DM_RS_HEADROOM=0.25 DM_SLM_FRAME_TRAILER=4 "
       "SLM_FREEZE_TX_TIMING=0 SIP_FORCE_PCMU=1 SIP_LOGIN=%s:%s@asterisk.net.cryan.nz "
       "nohup ./slmodemd/slmodemd -d9 -e /root/D-Modem-rig/d-modem > /tmp/slm.log 2>&1 < /dev/null & ); "
       "sleep 8; chmod 666 $(readlink -f /dev/ttySL0)" % (SLM_ACCT, SLM_ACCT))


PTY_RELAY = r'''
import os, select, sys, termios, tty
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY)
tty.setraw(fd)
a = termios.tcgetattr(fd); a[3] &= ~termios.ECHO; termios.tcsetattr(fd, termios.TCSANOW, a)
inp = sys.stdin.fileno()
while True:
    r = select.select([fd, inp], [], [])[0]
    if fd in r:
        try:
            d = os.read(fd, 65536)
        except OSError:
            break
        os.write(1, d)
    if inp in r:
        d = os.read(inp, 65536)
        if not d:
            break
        os.write(fd, d)
'''


class Dte:
    """The caller's DTE (slmodemd's ttySL0, or our PTY) through socat, with a
    growing receive buffer."""

    def __init__(self, log, ct=CT_SLM, dev='/dev/ttySL0'):
        # CT 117 has no socat; a raw-tty relay in python does the same job.
        argv = ['socat', dev + ',raw,echo=0,b115200', '-'] if ct == CT_SLM else \
            ['python3', '-u', '-c', PTY_RELAY, dev]
        self.p = subprocess.Popen(['pct', 'exec', ct, '--'] + argv,
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.rx = bytearray()
        self.log = log
        self.t0 = time.monotonic()
        self.dropped_at = None

    def pump(self, wait=0.1):
        if select.select([self.p.stdout], [], [], wait)[0]:
            chunk = os.read(self.p.stdout.fileno(), 65536)
            if chunk:
                self.rx.extend(chunk)
                self.log.write(chunk)
                self.log.flush()

    def send(self, data):
        if isinstance(data, str):
            data = data.encode()
        try:
            self.p.stdin.write(data)
            self.p.stdin.flush()
        except BrokenPipeError:
            pass

    def wait(self, tokens, timeout, start=None):
        """First of tokens to appear at/after start (default: now), or None."""
        start = len(self.rx) if start is None else start
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pump()
            hits = [(self.rx.find(t, start), t) for t in tokens]
            hits = [h for h in hits if h[0] >= 0]
            if hits:
                return min(hits)[1]
        return None

    def run_for(self, seconds, every=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if every:
                every()
            self.pump(0.05)

    def close(self):
        try:
            self.p.stdin.close()
        except Exception:
            pass
        self.p.terminate()
        try:
            self.p.wait(5)
        except subprocess.TimeoutExpired:
            self.p.kill()


# -- PPP (RFC 1662 async HDLC, RFC 1661 LCP) for the 7910 ---------------------

def fcs16(data):
    fcs = 0xffff
    for b in data:
        fcs ^= b
        for _ in range(8):
            fcs = (fcs >> 1) ^ 0x8408 if fcs & 1 else fcs >> 1
    return fcs


def ppp_frame(proto, payload):
    body = b'\xff\x03' + proto.to_bytes(2, 'big') + payload
    f = (fcs16(body) ^ 0xffff).to_bytes(2, 'little')
    out = bytearray(b'\x7e')
    for b in body + f:
        if b < 0x20 or b in (0x7d, 0x7e):
            out += bytes((0x7d, b ^ 0x20))
        else:
            out.append(b)
    return bytes(out + b'\x7e')


def ppp_parse(raw):
    """(good frames as (proto, payload), bad-FCS count) from a byte stream."""
    good, bad = [], 0
    for chunk in bytes(raw).split(b'\x7e'):
        if len(chunk) < 4:
            continue
        data, esc = bytearray(), False
        for b in chunk:
            if esc:
                data.append(b ^ 0x20)
                esc = False
            elif b == 0x7d:
                esc = True
            else:
                data.append(b)
        if len(data) < 4 or fcs16(data) != 0xf0b8:
            bad += 1
            continue
        d = bytes(data[:-2])
        if d[:2] == b'\xff\x03':
            d = d[2:]
        if d and d[0] & 1:
            proto, d = d[0], d[1:]
        elif len(d) >= 2:
            proto, d = int.from_bytes(d[:2], 'big'), d[2:]
        else:
            bad += 1
            continue
        good.append((proto, d))
    return good, bad


def probe_7910(dte, start, seconds):
    r = dict(lcp_seen=False, opened=False, echo_sent=0, echo_ok=0, terminated=False)
    if dte.wait([b'\x7e\xff\x7d\x23\xc0\x21', b'NO CARRIER'], 15, start) != b'\x7e\xff\x7d\x23\xc0\x21':
        return r
    r['lcp_seen'] = True
    magic = b'\x05\x06\x5a\x17\x2b\x91'
    acked, ours_acked, ident, seen = set(), False, 1, 0
    pad = bytes(range(32, 127)) * 2
    end = time.monotonic() + seconds
    next_tx = 0
    while time.monotonic() < end:
        dte.pump(0.05)
        frames, bad = ppp_parse(dte.rx[start:])
        r['fcs_bad'] = bad
        r['frames_good'] = len(frames)
        for proto, d in frames[seen:]:
            if proto != 0xc021 or len(d) < 4:
                continue
            code, pid = d[0], d[1]
            if code == 1 and pid not in acked:          # its Configure-Request
                dte.send(ppp_frame(0xc021, bytes((2,)) + d[1:]))
                acked.add(pid)
            elif code == 2 and pid == 1:                # Ack of ours
                ours_acked = True
            elif code == 5:                             # Terminate-Request
                r['terminated'] = round(time.monotonic() - dte.t0, 1)
                dte.send(ppp_frame(0xc021, bytes((6,)) + d[1:]))
            elif code == 10 and d[8:] == pad:              # Echo-Reply (its magic)
                r['echo_ok'] += 1
        seen = len(frames)
        r['opened'] = ours_acked and bool(acked)
        if time.monotonic() >= next_tx and not r['terminated']:
            next_tx = time.monotonic() + 0.5
            if not ours_acked:
                dte.send(ppp_frame(0xc021, bytes((1, 1, 0, 4 + len(magic))) + magic))
            elif acked:
                ident = ident % 255 + 1
                body = bytes((9, ident)) + (8 + len(pad)).to_bytes(2, 'big') \
                    + b'\x5a\x17\x2b\x91' + pad
                dte.send(ppp_frame(0xc021, body))
                r['echo_sent'] += 1
        if b'NO CARRIER' in dte.rx[start:]:
            break
    return r


# -- other probes --------------------------------------------------------------

def probe_7900(dte, start, seconds, lines=10):
    r = dict(menu=False, echo_selected=False, echo_ok=0, echo_sent=0)
    if dte.wait([b'Select: ', b'NO CARRIER'], 20, start) != b'Select: ':
        return r
    r['menu'] = True
    dte.send('1\r')
    if dte.wait([b'/menu exits.', b'NO CARRIER'], 12) != b'/menu exits.':
        return r
    r['echo_selected'] = True
    end = time.monotonic() + seconds
    for seq in range(lines):
        if time.monotonic() > end:
            break
        payload = ('SLM-%04d-' % seq + '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz' * 3).encode()
        r['echo_sent'] += 1
        dte.send(payload + b'\r')
        if dte.wait([b'ECHO: ' + payload + b'\r\n', b'NO CARRIER'], 20) != b'ECHO: ' + payload + b'\r\n':
            break
        r['echo_ok'] += 1
        dte.run_for(1)
    return r


def probe_rasfinder(dte, start, seconds, probes=10):
    """Its login field echoes what is typed; each probe is erased with
    backspaces so the 64-char field never truncates (mnp-live notes)."""
    r = dict(login=False, probe_sent=0, probe_ok=0)
    got = dte.wait([b'login:', b'NO CARRIER'], 10, start)
    if got is None:
        dte.send('\r')
        got = dte.wait([b'login:', b'NO CARRIER'], 10)
    if got != b'login:':
        return r
    r['login'] = True
    dte.run_for(1)
    end = time.monotonic() + seconds
    for seq in range(probes):
        if time.monotonic() > end:
            break
        text = ('p%02dQWERTYUIOPasdfghjkl%02d' % (seq, seq)).encode()
        r['probe_sent'] += 1
        mark = len(dte.rx)
        dte.send(text)
        if dte.wait([text, b'NO CARRIER'], 10, mark) != text:
            break
        r['probe_ok'] += 1
        dte.send(b'\x08' * len(text))
        dte.run_for(1.5)
    return r


def count_lines(data, tag):
    nums = [int(m) for m in re.findall(rb'^' + tag + rb'(\d{7})\r?$', bytes(data), re.M)]
    if not nums:
        return dict(lines=0)
    return dict(lines=len(nums), last=max(nums), missing=max(nums) + 1 - len(set(nums)))


def probe_ours(dte, start, seconds, cdir):
    """Numbered lines both ways at 200 lines/s (16 kbit/s) each."""
    ox("( sleep 0.5; i=0; end=$(( $(date +%%s) + %d )); while [ $(date +%%s) -lt $end ]; do "
       "j=0; while [ $j -lt 20 ]; do printf 'D%%07d\\r\\n' $i; i=$((i+1)); j=$((j+1)); done; "
       "sleep 0.1; done > %s ) > /dev/null 2>&1 < /dev/null &" % (seconds, PTY))
    state = dict(i=0, next=0)

    def tx():
        if time.monotonic() >= state['next']:
            state['next'] = time.monotonic() + 0.1
            dte.send(b''.join(b'U%07d\r\n' % (state['i'] + j) for j in range(20)))
            state['i'] += 20
    dte.run_for(seconds, tx)
    dte.run_for(5)
    r = dict(up_sent=state['i'], down=count_lines(dte.rx[start:], b'D'))
    up = ox('cat %s/pty-rx.bin' % cdir)
    r['up'] = count_lines(up.encode('latin-1', 'replace'), b'U')
    return r


SLM_PAT = re.compile(r'Modulations -|Phase2Info: (rtd|pcmType)|drop to|RETRAIN|CONNECT|'
                     r'ADI design|[Rr]ate (selected|chosen)|dataMode|DataMode|Hangup|hangup|'
                     r'Error Energy|Final|Line rate|upstream|downstream', re.I)


def one_call(out, name, target, args):
    cdir = out / name
    cdir.mkdir(parents=True)
    ext = OUR_ACCT if target == 'ours' else target
    r = dict(name=name, target=target, ext=ext,
             started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))
    ocdir = '/root/slmx/%s/%s' % (out.name, name)
    caller_ours = args.caller == 'ours'
    r['caller'] = args.caller
    if caller_ours:
        # Our modem dials as the V.90 analogue role towards the digital answerers.
        env = ('ME_V90_ROLE=analogue ' if target in ('7900', '7910') else '') + args.ours_env
        start_ours(ocdir, env, capture=False)
        init = ['AT', 'ATE0']
    else:
        if target == 'ours':
            start_ours(ocdir, args.ours_env)
        start_slm()
        init = ['AT', 'ATZ', 'ATX3', 'ATE0V1Q0', 'AT' + args.nparm]
    with (cdir / 'serial.bin').open('wb') as log:
        dte = Dte(log, CT_OURS, PTY) if caller_ours else Dte(log)
        try:
            ms = args.target_ms.get(target, args.ms)
            r['ms'] = ms
            for c in init + (['AT+MS=' + ms] if ms else []):
                dte.send(c + '\r')
                if dte.wait([b'OK', b'ERROR'], 5) != b'OK':
                    r['at_error'] = c
            mark = len(dte.rx)
            t_dial = time.monotonic()
            dte.send('ATD%s\r' % ext)
            got = dte.wait([b'CONNECT', b'NO CARRIER', b'BUSY', b'NO DIAL', b'NO ANSWER'], 100, mark)
            r['result'] = got.decode() if got else 'timeout'
            if got == b'CONNECT':
                dte.wait([b'\r\n'], 3, dte.rx.find(b'CONNECT', mark))
                m = re.search(rb'CONNECT[^\r\n]*', dte.rx[mark:])
                r['connect'] = m.group(0).decode()
                r['t_connect'] = round(time.monotonic() - t_dial, 1)
                start = mark + m.end()
                t_conn = time.monotonic()
                if target == 'ours':
                    r['data'] = probe_ours(dte, start, args.probe_seconds, ocdir)
                elif target == '7900':
                    r['data'] = probe_7900(dte, start, args.probe_seconds)
                elif target == '7910':
                    r['data'] = probe_7910(dte, start, args.probe_seconds)
                else:
                    r['data'] = probe_rasfinder(dte, start, args.probe_seconds)
                rest = args.hold - (time.monotonic() - t_conn)
                if rest > 0 and b'NO CARRIER' not in dte.rx[start:]:
                    dte.wait([b'NO CARRIER'], rest, start)
                nc = dte.rx.find(b'NO CARRIER', start)
                if nc >= 0:
                    r['dropped'] = True
                held = time.monotonic() - t_conn
                r['held_s'] = round(held, 1)
            if r['result'] == 'CONNECT' and not r.get('dropped'):
                time.sleep(1.2)
                dte.send('+++')
                time.sleep(1.5)
                dte.send('ATH\r')
                dte.run_for(3)
        finally:
            dte.close()
    if caller_ours:
        time.sleep(2)
        kill_ours()
        r['ours_log'] = ox("grep -aE 'CONNECT|data mode|DATA mode|retrain|Retrain|rate' %s/server.log "
                           "| grep -v TRACE | tail -25 | cut -c1-200" % ocdir).splitlines()
        (cdir / 'summary.json').write_text(json.dumps(r, indent=2) + '\n')
        return r
    for tap in ('dm_to_dsp.raw', 'dm_from_dsp.raw'):
        with (cdir / tap).open('wb') as f:
            subprocess.run(['pct', 'exec', CT_SLM, '--', 'cat', '/tmp/' + tap], stdout=f)
    slm_log = sx('cat /tmp/slm.log')
    (cdir / 'slm.log').write_text(slm_log)
    r['slm_lines'] = [l[:200] for l in slm_log.splitlines() if SLM_PAT.search(l)][-40:]
    sx("pkill '[s]lmodemd'; true")
    if target == 'ours':
        time.sleep(2)
        kill_ours()
        r['ours_pty_connect'] = ox("grep -ao 'CONNECT[^\\r]*' %s/pty-rx.bin | head -1" % ocdir).strip()
        r['ours_log'] = ox("grep -aE 'CONNECT|data mode|DATA mode|retrain|Retrain|rate' %s/server.log "
                           "| grep -v TRACE | tail -25 | cut -c1-200" % ocdir).splitlines()
    (cdir / 'summary.json').write_text(json.dumps(r, indent=2) + '\n')
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out', type=Path)
    ap.add_argument('--rounds', type=int, default=3)
    ap.add_argument('--targets', default='ours,7900,7910,6311,6312')
    ap.add_argument('--caller', choices=('slm', 'ours'), default='slm',
                    help='who dials: slmodemd (CT 100) or our modem (CT 117, account 6002)')
    ap.add_argument('--ms', default='', help="caller's AT+MS (default: its automode)")
    ap.add_argument('--target-ms', default='', help='per-target +MS, e.g. 7900=34,7910=34')
    ap.add_argument('--nparm', default='\\N3')
    ap.add_argument('--hold', type=float, default=60, help='seconds held after CONNECT')
    ap.add_argument('--probe-seconds', type=float, default=30)
    ap.add_argument('--ours-env', default='', help='extra env for our server')
    ap.add_argument('--gap', type=float, default=20, help='seconds between calls')
    args = ap.parse_args()
    args.target_ms = dict(kv.split('=', 1) for kv in args.target_ms.split(',') if kv)
    args.out.mkdir(parents=True, exist_ok=False)
    ox('mkdir -p /root/slmx/%s' % args.out.name)
    results = []
    targets = args.targets.split(',')
    if args.caller == 'ours' and 'ours' in targets:
        ap.error('--caller ours cannot dial target ours')
    for rnd in range(1, args.rounds + 1):
        for target in targets:
            name = '%02d-%s' % (len(results) + 1, target)
            print('CALL', name, flush=True)
            try:
                r = one_call(args.out, name, target, args)
            except Exception as exc:
                r = dict(name=name, target=target, error=repr(exc))
            results.append(r)
            (args.out / 'summary.json').write_text(json.dumps(results, indent=2) + '\n')
            print('RESULT', name, r.get('result'), r.get('connect'), 't=%s' % r.get('t_connect'),
                  'held=%s' % r.get('held_s'), 'drop' if r.get('dropped') else '',
                  json.dumps(r.get('data')), flush=True)
            time.sleep(args.gap)
    print('BATCH COMPLETE', args.out, flush=True)


if __name__ == '__main__':
    main()
