#!/usr/bin/env python3
"""Report per-stream RTP pacing from a capture written by `--capture-prefix`.

Written to settle a specific question: run34's peer reported a sustained
`Timing Offset = +4800 ppm` against the card's downstream signal, and the
candidates were our sender pacing versus the peer's own media clock.  The
capture holds both directions with our host's send/receive wall-clock times, so
the RTP timestamp rate of each stream measured against that one clock separates
them.

`RtpCapture` writes **LINKTYPE_RAW** (101), i.e. bare IP with no Ethernet
header -- IP at offset 0, UDP at 20, RTP at 28 for a headerless IPv4 packet.
Parsing it as Ethernet yields plausible-looking garbage (constant 146-byte
payloads, nonsense SSRCs), so decode the link type rather than assuming.

    tools/rtp_pcap_timing.py artifacts/eicon-native-tower/run34.rtp.pcap

Per stream it prints the packet count, payload size, RTP timestamp span over
wall-clock span (the media clock rate in Hz), the implied offset from 8000 Hz
in ppm, and any sequence or timestamp discontinuity.  Wall-clock times come
from our host for both directions: for the stream we send, the rate measures
our own pacing; for the stream we receive, it measures the peer's media clock
relative to ours, with arrival jitter averaging out over a call-length span.
"""
from __future__ import annotations

import argparse
import collections
import struct
import sys
from pathlib import Path

NOMINAL_HZ = 8000.0


def read_pcap(path: Path):
    """Yield (wall_seconds, ip_src, ip_dst, udp_src, udp_dst, rtp) per packet."""
    blob = path.read_bytes()
    if len(blob) < 24:
        raise SystemExit(f'{path}: too short to hold a pcap header')
    magic = struct.unpack('<I', blob[:4])[0]
    if magic == 0xA1B2C3D4:
        endian, nanos = '<', False
    elif magic == 0xD4C3B2A1:
        endian, nanos = '>', False
    elif magic == 0xA1B23C4D:
        endian, nanos = '<', True
    elif magic == 0x4D3CB2A1:
        endian, nanos = '>', True
    else:
        raise SystemExit(f'{path}: not a pcap file (magic {magic:#010x})')
    _, _, _, _, _, _, link = struct.unpack(endian + 'IHHIIII', blob[:24])
    if link == 101:       # LINKTYPE_RAW
        ip_offset = 0
    elif link == 1:       # LINKTYPE_ETHERNET
        ip_offset = 14
    elif link == 113:     # LINKTYPE_LINUX_SLL
        ip_offset = 16
    else:
        raise SystemExit(f'{path}: unsupported link type {link}')

    offset = 24
    while offset + 16 <= len(blob):
        seconds, fraction, caplen, _ = struct.unpack(
            endian + 'IIII', blob[offset:offset + 16])
        offset += 16
        packet = blob[offset:offset + caplen]
        offset += caplen
        if len(packet) < ip_offset + 20:
            continue
        wall = seconds + fraction / (1e9 if nanos else 1e6)
        ip = packet[ip_offset:]
        if ip[0] >> 4 != 4:
            continue
        header_len = (ip[0] & 0x0F) * 4
        if ip[9] != 17 or len(ip) < header_len + 8:   # UDP only
            continue
        src = '.'.join(str(b) for b in ip[12:16])
        dst = '.'.join(str(b) for b in ip[16:20])
        udp = ip[header_len:]
        sport, dport, length = struct.unpack('>HHH', udp[:6])
        rtp = udp[8:length] if 8 < length <= len(udp) else udp[8:]
        if len(rtp) < 12 or rtp[0] >> 6 != 2:         # RTP version 2
            continue
        yield wall, src, dst, sport, dport, rtp


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pcap', type=Path)
    ap.add_argument('--from', dest='start', type=float, default=None,
                    help='ignore packets before this offset into the capture')
    ap.add_argument('--to', dest='end', type=float, default=None)
    args = ap.parse_args()

    streams: dict[tuple, list] = collections.defaultdict(list)
    base = None
    for wall, src, dst, sport, dport, rtp in read_pcap(args.pcap):
        payload_type = rtp[1] & 0x7F
        sequence, timestamp, ssrc = struct.unpack('>HII', rtp[2:12])
        if base is None:
            base = wall
        relative = wall - base
        if args.start is not None and relative < args.start:
            continue
        if args.end is not None and relative > args.end:
            continue
        key = (src, sport, dst, dport, ssrc, payload_type)
        streams[key].append((relative, sequence, timestamp, len(rtp) - 12))

    if not streams:
        print('no RTP packets found')
        return 1

    for key, rows in sorted(streams.items(), key=lambda kv: -len(kv[1])):
        src, sport, dst, dport, ssrc, payload_type = key
        rows.sort()
        sizes = collections.Counter(row[3] for row in rows)
        print(f'{src}:{sport} -> {dst}:{dport}  ssrc={ssrc:#010x} pt={payload_type}')
        print(f'    {len(rows)} packets, payload sizes '
              + ' '.join(f'{size}x{count}' for size, count in sizes.most_common(3)))
        if len(rows) < 2:
            continue

        # Unwrap the 32-bit timestamp and 16-bit sequence so a call-length span
        # is measured, not a wrap.
        span_ts = 0
        gaps = jumps = 0
        for older, newer in zip(rows, rows[1:]):
            delta_ts = (newer[2] - older[2]) & 0xFFFFFFFF
            if delta_ts > 0x80000000:
                delta_ts -= 0x100000000
            delta_seq = (newer[1] - older[1]) & 0xFFFF
            if delta_seq != 1:
                gaps += 1
            if delta_ts <= 0 or delta_ts > 8000:
                jumps += 1
                continue
            span_ts += delta_ts
        span_wall = rows[-1][0] - rows[0][0]
        rate = span_ts / span_wall if span_wall else float('nan')
        ppm = (rate / NOMINAL_HZ - 1.0) * 1e6
        print(f'    {span_ts} timestamp units over {span_wall:.4f} s wall '
              f'-> {rate:.2f} Hz ({ppm:+.0f} ppm from {NOMINAL_HZ:.0f})')
        spacing = collections.Counter(
            round((newer[0] - older[0]) * 1000) for older, newer in zip(rows, rows[1:]))
        print(f'    packet spacing ms: '
              + ' '.join(f'{ms}x{count}' for ms, count in spacing.most_common(4))
              + f'   sequence gaps {gaps}, timestamp jumps {jumps}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
