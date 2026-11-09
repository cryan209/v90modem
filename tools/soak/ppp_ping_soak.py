#!/usr/bin/env python3
"""Dial v90ras from an AT modem on this host and hold a PPP link with pings.

  ppp_ping_soak.py <out-dir> [--pty /tmp/mt9234zba] [--dial 2950]
                   [--hold 300] [--at AT+MS=V90 ...] [--size 512]

macOS pppd needs root, so this is a minimal user-space PPP peer (RFC 1661
LCP, RFC 1332 IPCP, RFC 1662 async HDLC framing): it rejects multilink
(RFC 1990 MRRU/endpoint), takes the address the server assigns, then sends
one ICMP echo a second to the server (10.254.200.1 on v90ras) with a
pseudorandom body and checks each reply byte for byte.  That grades both
directions end to end through V.42 and PPP, which a CONNECT alone does not.

Writes events.jsonl (every modem line, every PPP state change, every ping)
and summary.json.  Pass = CONNECT, IPCP up, the hold reached with the link
still up, and at least 95% of pings answered intact.
"""
import argparse
import json
import os
from pathlib import Path
import random
import select
import struct
import time
import tty

FCS_TABLE = []
for b in range(256):
    v = b
    for _ in range(8):
        v = (v >> 1) ^ 0x8408 if v & 1 else v >> 1
    FCS_TABLE.append(v)


def fcs16(data):
    f = 0xFFFF
    for b in data:
        f = (f >> 8) ^ FCS_TABLE[(f ^ b) & 0xFF]
    return f ^ 0xFFFF


def hdlc_encode(payload):
    f = fcs16(payload)
    raw = payload + bytes((f & 0xFF, f >> 8))
    out = bytearray(b'\x7e')
    for b in raw:
        if b < 0x20 or b in (0x7e, 0x7d):
            out += bytes((0x7d, b ^ 0x20))
        else:
            out.append(b)
    out.append(0x7e)
    return bytes(out)


def ip_checksum(data):
    if len(data) & 1:
        data += b'\0'
    s = sum(struct.unpack('!%dH' % (len(data) // 2), data))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return ~s & 0xFFFF


class Soak:
    def __init__(self, args):
        self.a = args
        self.out = args.out
        self.ev = open(self.out / 'events.jsonl', 'w')
        self.t0 = time.monotonic()
        self.fd = os.open(args.pty, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
        self.rx = bytearray()
        self.frame = bytearray()
        self.esc = False
        self.in_frame = False
        self.lcp_up_peer = self.lcp_up_us = False
        self.ipcp_up_peer = self.ipcp_up_us = False
        self.our_ip = b'\0\0\0\0'
        self.peer_ip = None
        self.magic = random.getrandbits(32) or 1
        self.ident = 1
        self.pings = {}
        self.ping_seq = 0
        self.stats = dict(sent=0, intact=0, corrupt=0, lost=0, rtt=[], bad_fcs=0, frames=0,
                          lcp_echo_rx=0, term_rx=0)
        self.down_reason = None

    def log(self, event, **kw):
        row = dict(t=round(time.monotonic() - self.t0, 3), event=event, **kw)
        self.ev.write(json.dumps(row) + '\n')
        self.ev.flush()
        if event not in ('ping', 'frame'):
            print(json.dumps(row), flush=True)

    def write(self, data):
        off = 0
        end = time.monotonic() + 10
        while off < len(data) and time.monotonic() < end:
            if select.select([], [self.fd], [], 0.1)[1]:
                try:
                    off += os.write(self.fd, data[off:])
                except BlockingIOError:
                    pass

    def read(self, timeout):
        if select.select([self.fd], [], [], timeout)[0]:
            try:
                return os.read(self.fd, 65536)
            except (BlockingIOError, OSError):
                return b''
        return b''

    # ---- AT phase -------------------------------------------------------
    def at(self, cmd, wait=('OK', 'ERROR'), timeout=5):
        self.write((cmd + '\r').encode())
        buf = b''
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            buf += self.read(0.2)
            text = buf.decode('latin1')
            for w in wait:
                if w in text:
                    self.log('at', cmd=cmd, reply=text.strip())
                    return text
        self.log('at', cmd=cmd, reply=buf.decode('latin1').strip(), timeout=True)
        return buf.decode('latin1')

    def dial(self):
        self.at('AT', timeout=3)
        self.at('ATZ', timeout=5)
        for c in self.a.at:
            self.at(c)
        self.write(('ATD' + self.a.dial + '\r').encode())
        self.log('dial', number=self.a.dial)
        buf = b''
        end = time.monotonic() + self.a.wait
        while time.monotonic() < end:
            chunk = self.read(0.2)
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                line = line.decode('latin1').strip()
                if not line:
                    continue
                self.log('modem', line=line)
                if line.startswith('CONNECT'):
                    self.connect_line = line
                    self.connect_t = time.monotonic() - self.t0
                    self.rx += buf
                    return True
                if line in ('NO CARRIER', 'BUSY', 'NO DIALTONE', 'NO ANSWER', 'ERROR'):
                    self.connect_line = line
                    return False
        self.connect_line = 'TIMEOUT'
        self.write(b'\r')
        return False

    # ---- PPP ------------------------------------------------------------
    def send_ppp(self, proto, info):
        self.write(hdlc_encode(b'\xff\x03' + struct.pack('!H', proto) + info))

    def send_cp(self, proto, code, ident, data):
        self.send_ppp(proto, struct.pack('!BBH', code, ident, 4 + len(data)) + data)

    def lcp_req(self):
        self.ident = (self.ident + 1) & 0xFF
        self.send_cp(0xC021, 1, self.ident, bytes((5, 6)) + struct.pack('!I', self.magic))

    def ipcp_req(self):
        self.ident = (self.ident + 1) & 0xFF
        self.send_cp(0x8021, 1, self.ident, bytes((3, 6)) + self.our_ip)

    def feed(self, data):
        for b in data:
            if b == 0x7e:
                if self.in_frame and len(self.frame) >= 4:
                    self.handle_frame(bytes(self.frame))
                self.frame.clear()
                self.esc = False
                self.in_frame = True
            elif b == 0x7d:
                self.esc = True
            else:
                if self.esc:
                    b ^= 0x20
                    self.esc = False
                self.frame.append(b)

    def handle_frame(self, f):
        if fcs16(f[:-2]) != (f[-2] | (f[-1] << 8)):
            self.stats['bad_fcs'] += 1
            self.log('bad_fcs', length=len(f))
            return
        self.stats['frames'] += 1
        f = f[:-2]
        if f[:2] == b'\xff\x03':
            f = f[2:]
        if f[0] & 1:
            proto, info = f[0], f[1:]
        else:
            proto, info = struct.unpack('!H', f[:2])[0], f[2:]
        if proto == 0xC021:
            self.handle_lcp(info)
        elif proto == 0x8021:
            self.handle_ipcp(info)
        elif proto == 0x0021:
            self.handle_ip(info)
        else:
            if self.lcp_up_us and self.lcp_up_peer:
                self.ident = (self.ident + 1) & 0xFF
                self.send_cp(0xC021, 8, self.ident, struct.pack('!H', proto) + info)
            self.log('proto_reject', proto=hex(proto))

    @staticmethod
    def options(data):
        i = 0
        while i + 2 <= len(data):
            t, l = data[i], data[i + 1]
            if l < 2:
                break
            yield t, data[i:i + l]
            i += l

    def handle_lcp(self, info):
        code, ident, length = struct.unpack('!BBH', info[:4])
        data = info[4:length]
        if code == 1:
            rej = b''.join(o for t, o in self.options(data) if t in (17, 19, 3))
            if rej:
                self.send_cp(0xC021, 4, ident, rej)
                self.log('lcp_confrej', options=rej.hex())
            else:
                self.send_cp(0xC021, 2, ident, data)
                if not self.lcp_up_peer:
                    self.log('lcp_peer_acked')
                self.lcp_up_peer = True
                if not self.lcp_up_us:
                    self.lcp_req()
        elif code == 2:
            if not self.lcp_up_us:
                self.log('lcp_us_acked')
            self.lcp_up_us = True
        elif code in (3, 4):
            self.log('lcp_nak_rej', code=code, data=data.hex())
            self.lcp_req()
        elif code == 5:
            self.send_cp(0xC021, 6, ident, b'')
            self.stats['term_rx'] += 1
            self.down_reason = 'LCP Terminate-Request: ' + data.decode('latin1', 'replace')
            self.log('lcp_term', reason=self.down_reason)
        elif code == 9:
            self.stats['lcp_echo_rx'] += 1
            self.send_cp(0xC021, 10, ident, struct.pack('!I', self.magic) + data[4:])
        elif code == 10:
            pass
        else:
            self.log('lcp_other', code=code)

    def handle_ipcp(self, info):
        code, ident, length = struct.unpack('!BBH', info[:4])
        data = info[4:length]
        if code == 1:
            rej = b''.join(o for t, o in self.options(data) if t != 3)
            if rej:
                self.send_cp(0x8021, 4, ident, rej)
            else:
                for t, o in self.options(data):
                    if t == 3:
                        self.peer_ip = o[2:6]
                self.send_cp(0x8021, 2, ident, data)
                if not self.ipcp_up_peer:
                    self.log('ipcp_peer_acked', peer='.'.join(map(str, self.peer_ip or b'')))
                self.ipcp_up_peer = True
                if not self.ipcp_up_us:
                    self.ipcp_req()
        elif code == 2:
            if not self.ipcp_up_us:
                self.log('ipcp_us_acked', ours='.'.join(map(str, self.our_ip)))
            self.ipcp_up_us = True
        elif code == 3:
            for t, o in self.options(data):
                if t == 3:
                    self.our_ip = o[2:6]
            self.ipcp_req()
        elif code == 4:
            self.log('ipcp_rej', data=data.hex())
        elif code == 5:
            self.send_cp(0x8021, 6, ident, b'')

    def send_ping(self):
        self.ping_seq = (self.ping_seq + 1) & 0xFFFF
        body = random.Random(self.ping_seq * 7919).randbytes(self.a.size)
        icmp = struct.pack('!BBHHH', 8, 0, 0, 0x5650, self.ping_seq) + body
        icmp = icmp[:2] + struct.pack('!H', ip_checksum(icmp)) + icmp[4:]
        hdr = struct.pack('!BBHHHBBH4s4s', 0x45, 0, 20 + len(icmp), self.ping_seq, 0, 64, 1, 0,
                          self.our_ip, self.peer_ip)
        hdr = hdr[:10] + struct.pack('!H', ip_checksum(hdr)) + hdr[12:]
        self.send_ppp(0x0021, hdr + icmp)
        self.pings[self.ping_seq] = (time.monotonic(), body)
        self.stats['sent'] += 1

    def handle_ip(self, pkt):
        if len(pkt) < 28 or pkt[9] != 1:
            return
        ihl = (pkt[0] & 15) * 4
        icmp = pkt[ihl:]
        if icmp[0] != 0:
            return
        ident, seq = struct.unpack('!HH', icmp[4:8])
        if ident != 0x5650 or seq not in self.pings:
            return
        sent, body = self.pings.pop(seq)
        rtt = time.monotonic() - sent
        if icmp[8:] == body:
            self.stats['intact'] += 1
            self.stats['rtt'].append(rtt)
            self.log('ping', seq=seq, rtt=round(rtt, 3))
        else:
            self.stats['corrupt'] += 1
            self.log('ping_corrupt', seq=seq)

    def run(self):
        ok = self.dial()
        res = dict(connect=self.connect_line, ipcp=False, held=0.0)
        if ok:
            res['connect_t'] = round(self.connect_t, 2)
            start = time.monotonic()
            last_req = 0
            last_ping = 0
            ipcp_t = None
            no_carrier = False
            buf_text = b''
            while time.monotonic() - start < self.a.hold:
                data = self.read(0.05)
                if data:
                    self.feed(data)
                    buf_text = (buf_text + data)[-64:]
                    if b'NO CARRIER' in buf_text:
                        no_carrier = True
                        self.log('modem', line='NO CARRIER')
                        break
                now = time.monotonic()
                if not self.lcp_up_us and now - last_req > 3:
                    self.lcp_req()
                    last_req = now
                elif self.lcp_up_us and self.lcp_up_peer and not self.ipcp_up_us and now - last_req > 3:
                    self.ipcp_req()
                    last_req = now
                if self.ipcp_up_us and self.ipcp_up_peer and self.peer_ip:
                    if ipcp_t is None:
                        ipcp_t = now - start
                        res['ipcp'] = True
                        res['ipcp_t'] = round(ipcp_t, 2)
                    if now - last_ping >= 1.0:
                        for seq, (sent, _) in list(self.pings.items()):
                            if now - sent > 10:
                                self.pings.pop(seq)
                                self.stats['lost'] += 1
                                self.log('ping_lost', seq=seq)
                        self.send_ping()
                        last_ping = now
                if self.down_reason:
                    break
            res['held'] = round(time.monotonic() - start, 1)
            res['no_carrier'] = no_carrier
            res['down_reason'] = self.down_reason
            self.stats['lost'] += len(self.pings)
            # Hang up: escape, ATH, and read the call report.
            time.sleep(1.2)
            self.write(b'+++')
            time.sleep(1.5)
            self.at('ATH', timeout=8)
        for q in self.a.report:
            self.at(q, timeout=8)
        rtt = self.stats.pop('rtt')
        res.update(self.stats)
        if rtt:
            rtt.sort()
            res['rtt_median'] = round(rtt[len(rtt) // 2], 3)
            res['rtt_max'] = round(rtt[-1], 3)
        answered = res['intact'] / res['sent'] if res['sent'] else 0
        res['pass'] = bool(ok and res['ipcp'] and res['held'] >= self.a.hold - 1
                           and not res.get('no_carrier') and answered >= 0.95)
        (self.out / 'summary.json').write_text(json.dumps(res, indent=1) + '\n')
        print(json.dumps(res), flush=True)
        return res['pass']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out', type=Path)
    ap.add_argument('--pty', default='/tmp/mt9234zba')
    ap.add_argument('--dial', default='2950')
    ap.add_argument('--hold', type=float, default=300)
    ap.add_argument('--wait', type=float, default=120, help='seconds to wait for CONNECT')
    ap.add_argument('--size', type=int, default=128,
                    help='ICMP body bytes per ping (512 a second flooded a 4800 fallback '
                         'and the MT9234 bridge died on a stalled USB write)')
    ap.add_argument('--at', action='append', default=[], help='AT command before dialling')
    ap.add_argument('--report', action='append', default=[], help='AT query after hangup')
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=False)
    raise SystemExit(0 if Soak(a).run() else 1)


if __name__ == '__main__':
    main()
