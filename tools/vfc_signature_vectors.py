#!/usr/bin/env python3
"""Execute original V.FC signature TX/RX bit routines for independent fixtures.

No waveform/training claim: TX observes a719 before scrambling; RX enters
aa18 after acquisition, stopping at a40b before the overlay request.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--emulator', type=Path, required=True)
    p.add_argument('--rom', type=Path, required=True)
    p.add_argument('--library', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(args.emulator.resolve()))
    os.environ['COURIER_C5X_LIBRARY'] = str(args.library.resolve(strict=True))
    from courier_emu.rom import CourierRom
    from courier_emu.dsp import NativeC5x
    rom = CourierRom.load(args.rom)
    if hashlib.sha256(rom.data).hexdigest() != 'f1c91621fbf14aad34547056feb4425e64fcf0b38ec5d57f86c9f432ca999db6':
        raise ValueError('expected the captured Courier 403 firmware profile')
    args.output.mkdir(parents=True, exist_ok=False)
    tx = ['# code,32 native pre-scrambler dibits (a6e4/a6f0 -> a719)']
    with NativeC5x(rom) as c:
        c.set_mpmc_pin(0)
        c.set_pc_capture(0xa719, [0x3d0])
        for code in range(16):
            for a, v in ((0x3c8, 0xa6e4), (0x3d4, 0), (0x3e2, code), (0x3d1, 3),
                         (0x3d2, 2), (0x3d8, 0), (0x3d9, 0), (0x3da, 0)):
                c.set_data(a, v)
            c.clear_pc_captures()
            callback = 0xa6e4
            for _ in range(32):
                driver = (0xbc07, 0xbf01, 0x7a80, callback, 0x7980, 4)
                c.load_program(struct.pack('<6H', *driver), 0)
                c.set_pc(0)
                for _ in range(160):
                    c.step(1)
                    if c.state()['pc'] == 4:
                        break
                else:
                    raise RuntimeError('native signature TX did not return')
                callback = c.data(0x3c8)
            rows = c.pc_captures()
            if len(rows) != 32 or any(row[5] > 3 for row in rows):
                raise RuntimeError('missing raw TX dibits')
            tx.append(str(code) + ',' + ''.join(str(row[5]) for row in rows))
    (args.output / 'signature-tx.csv').write_text('\n'.join(tx) + '\n')
    rx = ['# header,body,mode_flags,rx_flags,accepted,response_count (aa18 -> a40b)']
    with NativeC5x(rom) as c:
        c.set_mpmc_pin(0)
        c.load_program(struct.pack('<6H', 0xbc06, 0xbf01, 0x7a80, 0xaa18, 0x7980, 4), 0)

        def receive(header, body, mode, flags):
            for a in range(0x330, 0x334): c.set_data(a, 0)
            c.set_data(0x3a6, mode)
            c.set_data(0x3af, flags)
            c.set_data(0x3ca, 0)
            accepted = False
            for word in (header, body):
                for bit in range(0, 32, 2):
                    c.set_data(0x320, (word >> bit) & 3)
                    c.set_pc(0)
                    for _ in range(160):
                        c.step(1)
                        pc = c.state()['pc']
                        if pc in (4, 0xa40b): break
                    else:
                        raise RuntimeError('native signature RX did not return')
                    if pc == 0xa40b: accepted = True
            rx.append(f'{header:08x},{body:08x},{mode:04x},{flags:04x},'
                      f'{int(accepted)},{c.data(0x3ca) if accepted else 0}')

        header = 0x4b434f52
        # Every repeated byte and every combination of the two priority bits.
        for byte in range(256):
            for mode in (0, 1):
                for flags in (0, 0x1000):
                    receive(header, byte * 0x01010101, mode, flags)
        # Every single header/body bit corruption, for each accepted code.
        for code in range(4):
            for bit in range(32):
                receive(header ^ (1 << bit), code * 0x01010101, 0, 0)
                receive(header, (code * 0x01010101) ^ (1 << bit), 0, 0)
    (args.output / 'signature-rx.csv').write_text('\n'.join(rx) + '\n')
    manifest = {'rom_sha256': hashlib.sha256(rom.data).hexdigest(),
                'library_sha256': hashlib.sha256(args.library.read_bytes()).hexdigest(),
                'tx_packets': len(tx)-1, 'rx_cases': len(rx)-1,
                'scope': 'isolated original firmware signature bit routines'}
    (args.output / 'signature-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest))


if __name__ == '__main__':
    main()
