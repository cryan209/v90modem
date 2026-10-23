#!/usr/bin/env python3
"""Check both legs of a G.711 RTP relay from one Ethernet/raw-IP pcap.

Payloads stay byte-exact. RTP identity (SSRC/sequence/timestamp) must be
preserved by the relay for this comparison. RTCP reception blocks follow
RFC 3550 6.4.1/6.4.2; layout also checked against vendored pjmedia/rtcp.h.
A receiver's loss report locates a concern beyond the captured relay egress;
it does not distinguish network loss from receiver behavior or prove causality.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import socket
import struct


def udp_packets(data):
    formats = {b'\xd4\xc3\xb2\xa1': ('<', 1e6), b'\xa1\xb2\xc3\xd4': ('>', 1e6),
               b'\x4d\x3c\xb2\xa1': ('<', 1e9), b'\xa1\xb2\x3c\x4d': ('>', 1e9)}
    if len(data) < 24 or data[:4] not in formats:
        raise ValueError('requires classic pcap')
    order, scale = formats[data[:4]]
    link = struct.unpack_from(order+'I', data, 20)[0]
    if link not in (1, 12, 101):
        raise ValueError('requires Ethernet or raw IPv4')
    pos = 24
    while pos < len(data):
        if pos+16 > len(data):
            raise ValueError('truncated pcap packet header')
        sec, frac, size, original = struct.unpack_from(order+'IIII', data, pos)
        pos += 16
        if size > original or pos+size > len(data):
            raise ValueError('truncated pcap packet')
        frame = data[pos:pos+size]
        pos += size
        if link == 1:
            if len(frame) < 14:
                continue
            kind = frame[12:14]
            at = 14
            while kind in (b'\x81\x00', b'\x88\xa8') and at+4 <= len(frame):
                kind, at = frame[at+2:at+4], at+4
            if kind != b'\x08\x00':
                continue
            frame = frame[at:]
        if len(frame) < 20 or frame[0] >> 4 != 4:
            continue
        ihl = (frame[0] & 15)*4
        total = struct.unpack_from('!H', frame, 2)[0]
        if frame[9] != 17 or struct.unpack_from('!H', frame, 6)[0] & 0x3fff:
            continue
        if ihl < 20 or total > len(frame) or total < ihl+8:
            raise ValueError('truncated/invalid IPv4 UDP packet')
        sport, dport, length = struct.unpack_from('!HHH', frame, ihl)
        if length < 8 or ihl+length > total:
            raise ValueError('truncated/invalid UDP packet')
        yield (sec+frac/scale, socket.inet_ntoa(frame[12:16]),
               socket.inet_ntoa(frame[16:20]), sport, dport, frame[ihl+8:ihl+length])


def rtp(data):
    if len(data) < 12 or data[0] >> 6 != 2 or data[1] & 127 not in (0, 8):
        return None
    head = 12+4*(data[0] & 15)
    if data[0] & 16:
        if head+4 > len(data):
            raise ValueError('truncated RTP extension')
        head += 4+4*struct.unpack_from('!H', data, head+2)[0]
    pad = data[-1] if data[0] & 32 else 0
    if head > len(data) or (data[0] & 32 and (pad == 0 or pad > len(data)-head)):
        raise ValueError('invalid RTP header/padding')
    seq, stamp, ssrc = struct.unpack_from('!HII', data, 2)
    return (ssrc, seq, stamp), data[head:len(data)-pad if pad else len(data)]


def reception_reports(data):
    """Yield (reported SSRC, cumulative loss, fraction loss, highest seq, jitter)."""
    pos = 0
    while pos+4 <= len(data) and data[pos] >> 6 == 2:
        count, kind = data[pos] & 31, data[pos+1]
        size = 4*(struct.unpack_from('!H', data, pos+2)[0]+1)
        if kind not in range(200, 208):
            return
        if pos+size > len(data):
            raise ValueError('truncated RTCP packet')
        if kind in (200, 201):
            at = pos+(28 if kind == 200 else 8)
            if at+24*count > pos+size:
                raise ValueError('truncated RTCP reception blocks')
            for _ in range(count):
                ssrc = struct.unpack_from('!I', data, at)[0]
                lost = int.from_bytes(data[at+5:at+8], 'big', signed=True)
                seq, jitter = struct.unpack_from('!II', data, at+8)
                yield ssrc, lost, data[at+4], seq, jitter
                at += 24
        pos += size


def compare(before, after):
    a, b = Counter(k for k, _ in before), Counter(k for k, _ in after)
    # Duplicate identities cannot provide a unique payload comparison.
    unique = all(n == 1 for n in a.values()) and all(n == 1 for n in b.values())
    source = dict(before)
    changed = sum(k in source and source[k] != v for k, v in after) if unique else None
    return dict(input_packets=len(before), output_packets=len(after),
                missing=sum((a-b).values()), added=sum((b-a).values()),
                unique_identities=unique, changed_payloads=changed,
                identical_order=before == after,
                byte_exact_relay=bool(before) and before == after)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('pcap', type=Path)
    p.add_argument('--source', required=True)
    p.add_argument('--relay', required=True)
    p.add_argument('--peer', required=True)
    p.add_argument('--tx-tap', type=Path)
    args = p.parse_args()
    if len({args.source, args.relay, args.peer}) != 3:
        p.error('source, relay and peer must be distinct addresses')
    legs = {(args.source, args.relay): [], (args.relay, args.peer): [],
            (args.peer, args.relay): [], (args.relay, args.source): []}
    reports = []
    first = None
    for time, src, dst, _, _, payload in udp_packets(args.pcap.read_bytes()):
        if (src, dst) not in legs:
            continue
        decoded = rtp(payload)
        if decoded:
            if first is None:
                first = time
            legs[src, dst].append(decoded)
        else:
            for ssrc, lost, fraction, seq, jitter in reception_reports(payload):
                reports.append(dict(time=time, src=src, dst=dst, ssrc=f'{ssrc:08x}',
                                    cumulative_lost=lost, fraction_lost=fraction,
                                    highest_sequence=seq, jitter_samples=jitter))
    for row in reports:
        row['seconds_from_first_rtp'] = round(row['time']-first, 6) if first is not None else None
    down = legs[args.relay, args.peer]
    result = dict(downstream=compare(legs[args.source, args.relay], down),
                  upstream=compare(legs[args.peer, args.relay], legs[args.relay, args.source]),
                  reception_reports=reports)
    if args.tx_tap:
        tap = args.tx_tap.read_bytes()
        wire = b''.join(v for _, v in down)
        result['tx_tap_to_peer_egress'] = dict(tap_bytes=len(tap), wire_bytes=len(wire),
             byte_exact=tap == wire, tap_sha256=hashlib.sha256(tap).hexdigest(),
             wire_sha256=hashlib.sha256(wire).hexdigest())
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
