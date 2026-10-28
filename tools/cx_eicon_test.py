#!/usr/bin/env python3
"""Bounded serial-modem call to 7900/7910, with checked text/PPP LCP echoes.

LCP echoes prove bidirectional framed data, NOT PPP authentication/IP access.
Uses volatile AT settings only; preserves last-call diagnostics before ATZ.
Run with the project's .venv (pyserial); --selftest needs only Python stdlib.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import struct
import time


def fcs(data):
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value


def frame(packet):
    # RFC 1662 §§4.2/4.5: default ACCM, octet stuffing and PPP FCS-16.
    body = b'\xff\x03\xc0\x21' + packet
    body += struct.pack('<H', fcs(body) ^ 0xffff)
    return b'\x7e' + b''.join(bytes((0x7d, x ^ 0x20)) if x < 32 or x in (125, 126)
                               else bytes((x,)) for x in body) + b'\x7e'


class Decoder:
    def __init__(self):
        self.body = bytearray()
        self.escaped = False
        self.started = False
        self.bad_fcs = 0

    def feed(self, data):
        packets = []
        for byte in data:
            if byte == 126:
                if self.started and self.body and not self.escaped:
                    if len(self.body) >= 4 and fcs(self.body) == 0xf0b8:
                        packets.append(bytes(self.body[:-2]))
                    else:
                        self.bad_fcs += 1
                self.body.clear()
                self.escaped = False
                self.started = True
            elif self.started:
                if self.escaped:
                    self.body.append(byte ^ 32)
                    self.escaped = False
                elif byte == 125:
                    self.escaped = True
                elif byte >= 32:  # unescaped default-ACCM noise is discarded
                    self.body.append(byte)
                if len(self.body) > 65536:
                    self.body.clear()
                    self.started = False
        return packets


def lcp(code, ident, data=b''):
    return struct.pack('!BBH', code, ident, len(data) + 4) + data


class Probe:
    def __init__(self):
        self.decoder = Decoder()
        self.magic = b'CX01'
        self.options = b'\x05\x06' + self.magic
        self.configured = False
        self.peer_acked = False
        self.valid_frames = 0
        self.pending = {}
        self.echoes = set()
        self.packets = []

    def configure(self):
        return frame(lcp(1, 73, self.options))

    def echo(self, ident):
        payload = ('CX-EICON-%03d-' % ident).encode() + bytes(range(64))
        self.pending[ident] = payload
        return frame(lcp(9, ident, self.magic + payload))

    def feed(self, data):
        replies = []
        for body in self.decoder.feed(data):
            self.valid_frames += 1
            if body.startswith(b'\xff\x03'):
                body = body[2:]
            if not body.startswith(b'\xc0\x21') or len(body) < 6:
                continue
            packet = body[2:]
            code, ident, length = struct.unpack('!BBH', packet[:4])
            if length < 4 or length != len(packet):
                continue
            content = packet[4:]
            self.packets.append(dict(code=code, ident=ident, hex=packet.hex()))
            if code == 1:
                self.peer_acked = False
                # RFC 1661 §6.2: do not ACK authentication we don't implement.
                # A peer requiring PAP/CHAP may refuse; that is not a data pass.
                options, offset = [], 0
                while offset < len(content):
                    if offset+2 > len(content) or content[offset+1] < 2 or offset+content[offset+1] > len(content):
                        break
                    end = offset+content[offset+1]
                    options.append(content[offset:end])
                    offset = end
                if offset != len(content):
                    continue
                lengths = {1: 4, 2: 6, 5: 6, 7: 2, 8: 2}
                rejected = b''.join(option for option in options
                                    if option[0] not in lengths or len(option) != lengths[option[0]])
                if rejected:
                    replies.append(frame(lcp(4, ident, rejected)))
                else:
                    self.peer_acked = True
                    replies.append(frame(lcp(2, ident, content)))
            elif code == 2 and ident == 73 and content == self.options:
                self.configured = True
            elif code == 9 and len(content) >= 4:
                replies.append(frame(lcp(10, ident, self.magic + content[4:])))
            elif code == 10 and ident in self.pending and len(content) >= 4:
                if content[4:] == self.pending[ident]:
                    self.echoes.add(ident)
        return replies


def connect_details(text):
    details = {}
    rates = re.search(r'/(\d+):TX/(\d+):RX', text)
    if rates:
        details.update(tx_bps=int(rates[1]), rx_bps=int(rates[2]))
    else:
        rate = re.search(r'/(\d+)$', text)
        if rate:
            details['connect_carrier_bps'] = int(rate[1])
    carrier = re.search(r'/(V22B|V22|V32B|V32|V34|V90|V92)(?:/|$)', text)
    if carrier:
        details['negotiated_mode'] = carrier[1]
    return details


def selftest():
    assert connect_details('CONNECT 115200/LAPM/V42B/2400') == {'connect_carrier_bps': 2400}
    assert connect_details('CONNECT 115200/V34/LAPM/28800:TX/31200:RX') == dict(tx_bps=28800,rx_bps=31200,negotiated_mode='V34')
    packet = lcp(1, 73, b'\x05\x06CX01')
    wire = frame(packet)
    decoder = Decoder()
    decoded = []
    for byte in wire:
        decoded.extend(decoder.feed(bytes((byte,))))
    assert decoded == [b'\xff\x03\xc0\x21' + packet]
    assert decoder.feed(frame(packet[:-1] + b'2'))
    damaged = bytearray(wire)
    damaged[-3] ^= 1
    assert not decoder.feed(damaged) and decoder.bad_fcs == 1
    probe = Probe()
    probe.feed(frame(lcp(2, 72, probe.options)))
    assert not probe.configured
    probe.feed(frame(lcp(2, 73, probe.options)))
    assert probe.configured
    reject = probe.feed(frame(lcp(1, 9, b'\x03\x04\xc0\x23')))
    assert reject == [frame(lcp(4, 9, b'\x03\x04\xc0\x23'))] and not probe.peer_acked
    assert not probe.feed(frame(lcp(1, 10, b'\x05\x00')))
    assert probe.feed(frame(lcp(1, 11, b'\x05\x06PEER'))) and probe.peer_acked
    probe.echo(20)
    probe.feed(frame(lcp(10, 20, b'PEERwrong')))
    assert not probe.echoes
    probe.feed(frame(lcp(10, 20, b'PEER' + probe.pending[20])))
    assert probe.echoes == {20}
    print('PPP framing, fragmented input, FCS rejection, configuration and exact LCP echo checks passed')


def call(args):
    import serial
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(number=args.number, requested=args.mode, max_rate=args.max_rate,
                  device=args.device, utc=datetime.now(timezone.utc).isoformat(),
                  runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  connected=False, exact_lcp_echoes=0)
    started = time.monotonic()
    events = []
    rx = bytearray()
    dialed = False
    port = None
    raw = (args.output / 'rx.bin').open('wb')
    tx = (args.output / 'tx.bin').open('wb')

    def event(kind, **fields):
        item = dict(t=round(time.monotonic()-started, 3), event=kind, **fields)
        events.append(item)
        print(json.dumps(item), flush=True)

    def write(data):
        tx.write(data)
        tx.flush()
        # write_timeout bounds serial writes. tcdrain()/flush() can wait
        # indefinitely with CTS held low; failed escape uses verified &D2.
        port.write(data)

    def read():
        data = port.read(4096)
        if data:
            raw.write(data)
            raw.flush()
            rx.extend(data)
        return data

    def command(text, seconds=3):
        offset = len(rx)
        write(text.encode() + b'\r')
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            read()
            if re.search(rb'(?:\r|\n)(?:OK|ERROR)(?:\r|\n)', rx[offset:]):
                break
        answer = bytes(rx[offset:])
        event('command', command=text, response=answer.decode(errors='replace'))
        return answer

    try:
        port = serial.Serial(args.device, 115200, timeout=.1, write_timeout=2,
                             rtscts=True, exclusive=True)
        ms = 'AT+MS=' + args.mode + ',0'
        if args.max_rate:
            ms += ',300,%d,300,%d' % (args.max_rate,args.max_rate)
        for text in ('ATZ', 'ATE0V1Q0X4', 'ATW2', 'AT&C1&D2&K3', 'AT\\N3',
                     'ATS7=60', ms):
            if not re.search(rb'(?:\r|\n)OK(?:\r|\n)', command(text)):
                raise RuntimeError('AT setup rejected: '+text)
        offset = len(rx)
        dialed = True
        write(('ATDT'+args.number+'\r').encode())
        event('dial')
        deadline = time.monotonic()+args.startup
        match = None
        while time.monotonic() < deadline:
            read()
            incoming = bytes(rx[offset:])
            match = re.search(rb'(?:\r|\n)(CONNECT[^\r\n]*)[\r\n]+', incoming)
            if match:
                break
            failure = re.search(rb'(?:\r|\n)(NO CARRIER|NO DIALTONE|NO DIAL TONE|BUSY|ERROR)[\r\n]', incoming)
            if failure:
                result['failure'] = failure[1].decode()
                return result
        if not match:
            result['failure'] = 'startup timeout'
            return result
        result.update(connected=True, connect=match[1].decode(errors='replace'))
        event('connect', response=result['connect'])
        result.update(connect_details(result['connect']))
        probe = Probe()
        data = incoming[match.end():]
        sent_config = False
        next_config = next_echo = 0
        menu_rx = bytearray()
        menu_selected = echo_ready = False
        menu_echoes = 0
        expected_echo = None
        deadline = time.monotonic()+args.data_seconds
        while time.monotonic() < deadline:
            menu_rx.extend(data)
            if not menu_selected and b'Select: ' in menu_rx:
                result['service'] = 'text_echo_menu'
                menu_selected = True
                write(b'1\r')
            if menu_selected and b'/menu exits.' in menu_rx:
                echo_ready = True
            if expected_echo is not None and expected_echo in menu_rx:
                menu_echoes += 1
                event('text_echo', sequence=menu_echoes, exact=True)
                menu_rx.clear()
                expected_echo = None
            if echo_ready and expected_echo is None and menu_echoes < 3:
                payload = ('CX-%s-%s-%02d-' % (args.number,args.mode,menu_echoes)).encode() + b'0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz'
                expected_echo = b'ECHO: ' + payload + b'\r\n'
                write(payload+b'\r')
            for response in probe.feed(data):
                write(response)
            now = time.monotonic()
            if probe.valid_frames and not probe.configured and now >= next_config:
                write(probe.configure())
                sent_config = True
                next_config = now+2
            if sent_config and probe.configured and probe.peer_acked and now >= next_echo:
                for ident in range(20, 23):
                    if ident not in probe.echoes:
                        write(probe.echo(ident))
                next_echo = now+2
            if len(probe.echoes) == 3 or menu_echoes == 3:
                break
            data = read()
            if b'NO CARRIER' in data:
                result['failure'] = 'carrier lost during data test'
                break
        result.update(exact_text_echoes=menu_echoes, exact_lcp_echoes=len(probe.echoes), valid_ppp_frames=probe.valid_frames,
                      bad_ppp_fcs=probe.decoder.bad_fcs, lcp_config_acked=probe.configured,
                      peer_config_acked=probe.peer_acked, lcp_packets=probe.packets)
        event('data_result', echoes=len(probe.echoes), valid_frames=probe.valid_frames)
        return result
    except Exception as exc:
        result['error'] = str(exc)
        return result
    finally:
        if port is not None:
            try:
                if dialed:
                    # Guard time is required even on a startup/data timeout.
                    time.sleep(1.2)
                    write(b'+++')
                    end = time.monotonic()+1.5
                    while time.monotonic() < end:
                        read()
                    write(b'\r')
                ok = lambda response: bool(re.search(rb'(?:\r|\n)OK(?:\r|\n)', response))
                result['hangup_confirmed'] = ok(command('ATH0'))
                if not result['hangup_confirmed']:
                    # The modem may not accept the escape while its link is
                    # retraining. &D2 was verified during setup: DTR is the
                    # independent fallback, not more AT text sent as payload.
                    event('dtr_hangup_fallback')
                    port.dtr = False
                    time.sleep(1)
                    port.dtr = True
                    time.sleep(.3)
                    read()
                    result['hangup_confirmed'] = ok(command('AT')) and ok(command('ATH0'))
                if result['hangup_confirmed']:
                    for text in ('AT&V1', 'AT#UD', 'AT+MS?'):
                        answer = command(text, 4)
                        if text == 'AT&V1':
                            for label, key in ((b'LAST TX rate', 'last_tx_bps'),
                                               (b'LAST RX rate', 'last_rx_bps')):
                                match = re.search(re.escape(label) + rb'\.+\s*(\d+) BPS', answer)
                                if match:
                                    result[key] = int(match[1])
                    result['profile_restored'] = ok(command('ATZ'))
                    command('AT+MS?')
                else:
                    result['profile_restored'] = False
                    port.dtr = False
            except Exception as exc:
                result['cleanup_error'] = str(exc)
                port.dtr = False
            finally:
                port.close()
        raw.close()
        tx.close()
        result['elapsed_seconds'] = round(time.monotonic()-started, 3)
        (args.output / 'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        (args.output / 'events.json').write_text(json.dumps(events, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selftest', action='store_true')
    parser.add_argument('--device', default='/dev/cu.usbmodem246802461')
    parser.add_argument('--number', choices=('7900', '7910'))
    parser.add_argument('--mode', choices=('V22B', 'V32B', 'V34', 'V90', 'V92'), default='V34')
    parser.add_argument('--max-rate', type=int, help='optional TX/RX carrier ceiling (bit/s)')
    parser.add_argument('--startup', type=float, default=75)
    parser.add_argument('--data-seconds', type=float, default=35)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.selftest:
        selftest()
        return 0
    if not args.number or not args.output or min(args.startup, args.data_seconds) <= 0:
        parser.error('--number, --output and positive timeouts are required')
    if args.max_rate is not None and not 300 <= args.max_rate <= 56000:
        parser.error('--max-rate must be between 300 and 56000')
    result = call(args)
    print(json.dumps(result, indent=2))
    return 0 if ((result.get('exact_lcp_echoes') == 3 or result.get('exact_text_echoes') == 3) and result.get('hangup_confirmed')
                 and result.get('profile_restored') and not result.get('cleanup_error')) else 1


if __name__ == '__main__':
    raise SystemExit(main())
