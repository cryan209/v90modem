#!/usr/bin/env python3
"""Negative controls for byte transparency and RTCP loss attribution."""
import socket
import struct
import unittest
from rtp_path_check import compare, reception_reports, rtp, udp_packets


def packet(seq, stamp, payload=b'\x7f\xff\x55\xd5'):
    return struct.pack('!BBHII', 0x80, 0, seq, stamp, 123)+payload


def capture(payload):
    ip = bytearray(20)
    ip[0], ip[9] = 0x45, 17
    struct.pack_into('!H', ip, 2, 28+len(payload))
    ip[12:16], ip[16:20] = socket.inet_aton('192.0.2.1'), socket.inet_aton('192.0.2.2')
    frame = bytes(12)+b'\x08\x00'+ip+struct.pack('!HHHH', 10000, 10002, len(payload)+8, 0)+payload
    return (struct.pack('<IHHIIII', 0xa1b2c3d4, 2, 4, 0, 0, 65535, 1)
            +struct.pack('<IIII', 100, 123000, len(frame), len(frame))+frame)


class PathCheckTest(unittest.TestCase):
    def test_transcoding_loss_reorder_duplicate_and_timestamp_controls(self):
        # Include sequence wrap and mu-law negative zero: a linear round trip
        # changing even one codeword must fail the transparency claim.
        a = [rtp(packet(65535, 0)), rtp(packet(0, 4)), rtp(packet(1, 8))]
        self.assertTrue(compare(a, a)['byte_exact_relay'])
        changed = a.copy()
        changed[1] = (a[1][0], b'\xff\xff\x55\xd5')
        self.assertEqual(compare(a, changed)['changed_payloads'], 1)
        self.assertFalse(compare(a, changed)['byte_exact_relay'])
        self.assertEqual(compare(a, a[1:])['missing'], 1)
        self.assertFalse(compare(a, a[::-1])['byte_exact_relay'])
        self.assertFalse(compare(a, a+[a[0]])['unique_identities'])
        wrong_clock = a.copy()
        wrong_clock[0] = rtp(packet(65535, 160))
        self.assertEqual(compare(a, wrong_clock)['added'], 1)
        self.assertFalse(compare([], [])['byte_exact_relay'])

    def test_capture_and_rtp_extension_padding(self):
        raw = packet(7, 160)
        rows = list(udp_packets(capture(raw)))
        self.assertEqual(rows[0][:5], (100.123, '192.0.2.1', '192.0.2.2', 10000, 10002))
        self.assertEqual(rows[0][5], raw)
        # One CSRC, one extension word and four padding bytes.
        extended = bytes([0xb1])+raw[1:12]+bytes(4)+b'\xab\xcd\x00\x01'+bytes(4)+raw[12:]+b'\x00\x00\x00\x04'
        self.assertEqual(rtp(extended), rtp(raw))
        with self.assertRaises(ValueError):
            list(udp_packets(capture(raw)[:-1]))
        with self.assertRaises(ValueError):
            rtp(bytes([0x90])+raw[1:12])

    def test_compound_reception_reports_and_signed_loss(self):
        sr = struct.pack('!BBHI', 0x80, 200, 6, 77)+bytes(20)
        block = struct.pack('!I', 123)+b'\x02\xff\xff\xff'+struct.pack('!IIII', 65537, 16, 0, 0)
        rr = struct.pack('!BBHI', 0x81, 201, 7, 88)+block
        self.assertEqual(list(reception_reports(sr+rr)), [(123, -1, 2, 65537, 16)])
        self.assertIsNone(rtp(rr))
        with self.assertRaises(ValueError):
            list(reception_reports(rr[:-1]))


if __name__ == '__main__':
    unittest.main()
