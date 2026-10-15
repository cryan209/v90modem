#!/usr/bin/env python3
"""Independent on-hook FSK fixtures through the actual Apple CID receive path.

V.253 9.2.3.1.3: reject bad checksums; retain SDMF/MDMF caller fields.
V.23 section 2: 1300/2100 Hz, Bell 202: 1200/2200 Hz, 1200 bit/s.
Temporary recordings are private to this run; no hardware is opened.
"""
import json
import math
from pathlib import Path
import struct
import subprocess
import tempfile


def waveform(message, rate, v23=False, corrupt=False):
    packet = message + bytes([(-sum(message) + int(corrupt)) & 255])
    bits = [0, 1] * 150 + [1] * 180
    for value in packet:
        bits += [0] + [(value >> bit) & 1 for bit in range(8)] + [1]
    bits += [1] * 12
    samples = [650] * (rate // 5)
    phase = 0.0
    for index in range(math.ceil(len(bits) * rate / 1200)):
        bit = bits[min(len(bits)-1, index * 1200 // rate)]
        frequency = (1300 if bit else 2100) if v23 else (1200 if bit else 2200)
        samples.append(round(650 + 6000 * math.sin(phase)))
        phase = (phase + 2 * math.pi * frequency / rate) % (2 * math.pi)
    samples += [650] * (rate // 3)
    return struct.pack('<' + 'h' * len(samples), *samples)


def main():
    repo = Path(__file__).resolve().parents[1]
    def packet(kind, body):
        return bytes([kind, len(body)]) + body
    def field(kind, body):
        return bytes([kind, len(body)]) + body
    cases = [
        (packet(4, b'101609306421555123'), '6421555123', ''),
        (packet(128, field(1, b'10160930') + field(2, b'6004') + field(7, b'Apple Modem')), '6004', 'Apple Modem'),
        (packet(128, field(1, b'10160930') + field(4, b'P') + field(8, b'O')), 'P', 'O'),
    ]
    with tempfile.TemporaryDirectory(prefix='apple-cid-') as directory:
        fixture = Path(directory)/'rx.s16'
        for rate in (8000, 9600):
            for v23 in (False, True):
                for message, number, name in cases:
                    for corrupt in (False, True):
                        fixture.write_bytes(waveform(message, rate, v23, corrupt))
                        result = subprocess.run([str(repo/'apple_usb_modem_coupler'), '--cid-replay', str(fixture), '--rate', str(rate)], capture_output=True, text=True, check=True)
                        cid = json.loads(result.stdout)
                        assert cid['messages'] == (0 if corrupt else 1), (rate, v23, corrupt, cid, result.stderr)
                        if not corrupt:
                            assert (cid['number'], cid['name'], cid['date'], cid['time']) == (number, name, '1016', '0930'), cid
                            # Their passbands overlap: either decoder can accept the same valid frame.
                            assert cid['decoder'] in ('V.23', 'Bell 202'), cid
        # Valid checksum does not excuse truncated TLV structure.
        fixture.write_bytes(waveform(packet(128, b'\x02\x20short'), 9600))
        cid = json.loads(subprocess.check_output([str(repo/'apple_usb_modem_coupler'), '--cid-replay', str(fixture)], text=True))
        assert cid['messages'] == 0 and cid['malformed'] >= 1, cid
        # Constant ADC offset must not invent a caller.
        fixture.write_bytes(struct.pack('<h', 650)*19200)
        cid = json.loads(subprocess.check_output([str(repo/'apple_usb_modem_coupler'), '--cid-replay', str(fixture)], text=True))
        assert cid['messages'] == 0, cid
    print('Apple CID: Bell 202/V.23, 8/9.6 kHz, SDMF/MDMF, privacy, bad checksums, malformed fields and DC silence passed')


if __name__ == '__main__':
    main()
