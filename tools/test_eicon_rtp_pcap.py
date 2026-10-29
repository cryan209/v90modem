#!/usr/bin/env python3
"""Synthetic capture checks: diagnostics must not rewrite G.711 or hide gaps."""
import json
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import unittest

TOOL = Path(__file__).resolve().with_name('eicon_rtp_pcap_check.py')


def packet(linktype, sequence, timestamp, payload_type, payload, protocol=0x0800):
    rtp = struct.pack('!BBHII', 0x80, payload_type, sequence, timestamp, 0x12345678) + payload
    udp = struct.pack('!HHHH', 1234, 5678, len(rtp)+8, 0) + rtp
    ip = struct.pack('!BBHHHBBH4s4s', 0x45, 0, len(udp)+20, 1, 0, 64, 17, 0,
                     socket.inet_aton('192.0.2.1'), socket.inet_aton('192.0.2.2')) + udp
    if linktype == 1:
        return b'\0'*12 + struct.pack('!H', protocol) + ip
    if linktype == 276:
        return struct.pack('!HHIHBB8s', protocol, 0, 3, 1, 0, 6, b'\0'*8) + ip
    return ip


def run_capture(linktype, order, packets):
    with tempfile.TemporaryDirectory(prefix='eicon-pcap-') as directory:
        root = Path(directory)
        data = struct.pack(order+'IHHIIII', 0xa1b2c3d4, 2, 4, 0, 0, 65535, linktype)
        for i, p in enumerate(packets):
            data += struct.pack(order+'IIII', 100+i, 0, len(p), len(p)) + p
        capture = root/'capture.pcap'
        capture.write_bytes(data)
        output = root/'output'
        result = subprocess.run([sys.executable, str(TOOL), str(capture), str(output)],
                                capture_output=True, text=True)
        if result.returncode:
            raise AssertionError(result.stderr)
        summary = json.loads((output/'summary.json').read_text())
        streams = {p.stem: p.read_bytes() for p in output.glob('*.g711')}
        metadata = {p.stem: json.loads(p.read_text()) for p in output.glob('*.json')
                    if p.name != 'summary.json'}
        return summary, streams, metadata


class CaptureTests(unittest.TestCase):
    def test_linktypes_endianness_payloads_and_gaps(self):
        for linktype in (1, 12, 101, 276):
            for order in ('<', '>'):
                with self.subTest(linktype=linktype, order=order):
                    first = bytes(range(160))
                    second = bytes(reversed(range(160)))
                    packets = [packet(linktype, 65535, 0xffffff00, 8, first),
                               packet(linktype, 1, 0x40, 8, second)]
                    summary, streams, metadata = run_capture(linktype, order, packets)
                    self.assertEqual(len(summary), 1)
                    key, detail = next(iter(summary.items()))
                    self.assertEqual(streams[key], first+second)
                    self.assertEqual(detail['payload_types'], {'8': 2})
                    self.assertEqual(detail['payload_bytes'], 320)
                    self.assertEqual(detail['gaps'][0]['sequence_delta'], 2)
                    self.assertEqual(detail['gaps'][0]['timestamp_delta'], 320)
                    self.assertEqual([p['payload_type'] for p in metadata[key]], [8, 8])

    def test_codec_change_is_visible_without_transcoding(self):
        packets = [packet(276, 1, 0, 0, b'\xff'*160),
                   packet(276, 2, 160, 8, b'\xd5'*160)]
        summary, streams, metadata = run_capture(276, '<', packets)
        key, detail = next(iter(summary.items()))
        self.assertEqual(detail['payload_types'], {'0': 1, '8': 1})
        self.assertEqual(streams[key], b'\xff'*160+b'\xd5'*160)
        self.assertEqual(detail['gaps'], [])
        self.assertEqual([p['payload_type'] for p in metadata[key]], [0, 8])

    def test_sll2_protocol_is_checked(self):
        summary, streams, metadata = run_capture(276, '<',
            [packet(276, 1, 0, 8, b'\xd5'*160, protocol=0x86dd)])
        self.assertEqual((summary, streams, metadata), ({}, {}, {}))


if __name__ == '__main__':
    unittest.main()
