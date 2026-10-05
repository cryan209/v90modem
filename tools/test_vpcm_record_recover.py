"""Framing checks for the recording recovery tool, independent of DSP fits."""
import unittest

from vpcm_record_recover import crc16_octets, detection_octets, hdlc_frames


def serial_bits(data):
    return [byte >> bit & 1 for byte in data for bit in range(8)]


def stuffed_frame(octets):
    result, ones = [], 0
    for bit in serial_bits(octets):
        result.append(bit)
        ones = ones + 1 if bit else 0
        if ones == 5:
            result.append(0)
            ones = 0
    flag = serial_bits(b'\x7e')
    return flag + result + flag


class FramingTests(unittest.TestCase):
    def test_known_x25_crc_and_corruption(self):
        # CRC-16/X-25 check vector; FCS is complemented and LSB first.
        self.assertEqual(crc16_octets(b'123456789'), 0x6f91)
        frame = stuffed_frame(b'123456789\x6e\x90')
        self.assertEqual(hdlc_frames(frame)['fcs_valid_frames'], 1)
        frame[9] ^= 1
        self.assertEqual(hdlc_frames(frame)['fcs_valid_frames'], 0)

    def test_erasures_are_never_valid_frames(self):
        frame = stuffed_frame(b'123456789\x6e\x90')
        frame[9] = 2
        self.assertEqual(hdlc_frames(frame)['fcs_valid_frames'], 0)

    def test_adp_requires_complete_characters_and_mark_spacing(self):
        e = [0] + serial_bits(b'E') + [1]
        c = [0] + serial_bits(b'C') + [1]
        for gap in (8, 11, 16):
            octets, positions = detection_octets((e + [1]*gap + c + [1]*gap)*3 + [0])
            self.assertEqual(octets, b'EC'*3)
            self.assertEqual(len(positions), 3)
        for gap in (2, 17):
            self.assertEqual(detection_octets((e + [1]*gap + c + [1]*gap)*3 + [0])[0], b'')
        self.assertEqual(detection_octets(e + [1]*11 + c[:5])[0], b'')
        self.assertEqual(detection_octets([1]*100)[0], b'')


if __name__ == '__main__':
    unittest.main()
