#!/usr/bin/env python3
"""Read Ethernet or raw IPv4 RTP from classic pcap; preserve payloads and packet gaps."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import socket
import struct


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('pcap', type=Path)
    ap.add_argument('output', type=Path)
    args = ap.parse_args()
    data = args.pcap.read_bytes()
    magic = data[:4]
    if magic not in (b'\xd4\xc3\xb2\xa1', b'\xa1\xb2\xc3\xd4'):
        ap.error('requires classic microsecond pcap')
    order = '<' if magic[0] == 0xd4 else '>'
    linktype = struct.unpack_from(order+'I', data, 20)[0]
    if linktype not in (1, 12, 101):
        ap.error('requires Ethernet or raw IP capture')
    streams = defaultdict(list)
    pos = 24
    while pos+16 <= len(data):
        sec, usec, size, _ = struct.unpack_from(order+'IIII', data, pos)
        pos += 16
        frame = data[pos:pos+size]
        pos += size
        if linktype == 1:
            if len(frame) < 42 or frame[12:14] != b'\x08\x00':
                continue
            ip = frame[14:]
        else:
            ip = frame
        if len(ip) < 28 or ip[0] >> 4 != 4:
            continue
        ihl = (ip[0] & 15)*4
        if ip[9] != 17 or struct.unpack_from('!H', ip, 6)[0] & 0x3fff:
            continue
        src, dst = socket.inet_ntoa(ip[12:16]), socket.inet_ntoa(ip[16:20])
        sport, dport, ulen = struct.unpack_from('!HHH', ip, ihl)
        rtp = ip[ihl+8:ihl+ulen]
        if len(rtp) < 12 or rtp[0] >> 6 != 2 or (rtp[1] & 127) not in (0, 8):
            continue
        seq, stamp, ssrc = struct.unpack_from('!HII', rtp, 2)
        head = 12+4*(rtp[0] & 15)
        if rtp[0] & 16:
            head += 4+4*struct.unpack_from('!H', rtp, head+2)[0]
        end = len(rtp)-(rtp[-1] if rtp[0] & 32 else 0)
        key = f'{src}_{sport}--{dst}_{dport}--{ssrc:08x}'
        streams[key].append((sec+usec/1e6, seq, stamp, rtp[head:end]))
    args.output.mkdir(exist_ok=True)
    summaries = {}
    for key, packets in streams.items():
        gaps = []
        for old, new in zip(packets, packets[1:]):
            sd, td = (new[1]-old[1]) & 65535, (new[2]-old[2]) & 0xffffffff
            if sd != 1 or td != len(old[3]):
                gaps.append(dict(after_sequence=old[1], next_sequence=new[1],
                    sequence_delta=sd, timestamp_delta=td, time=new[0]))
        (args.output/(key+'.g711')).write_bytes(b''.join(p[3] for p in packets))
        metadata = [dict(time=p[0], sequence=p[1], timestamp=p[2], bytes=len(p[3])) for p in packets]
        (args.output/(key+'.json')).write_text(json.dumps(metadata, indent=2)+'\n')
        summaries[key] = dict(packets=len(packets), payload_bytes=sum(len(p[3]) for p in packets),
            first_time=packets[0][0], last_time=packets[-1][0], gaps=gaps,
            max_interval_ms=max((b[0]-a[0])*1000 for a,b in zip(packets,packets[1:])) if len(packets)>1 else 0)
    (args.output/'summary.json').write_text(json.dumps(summaries, indent=2)+'\n')
    print(json.dumps(summaries, indent=2))


if __name__ == '__main__':
    main()
