#!/usr/bin/env python3
"""Execute Courier c1bf frame planning and the c88a schedule before bit packing."""
import argparse
import hashlib
import os
from pathlib import Path
import random
import struct
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for arg in ('emulator', 'rom', 'library', 'output'):
        p.add_argument('--' + arg, type=Path, required=True)
    p.add_argument('--map-output', type=Path)
    args = p.parse_args()
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(args.emulator.resolve()))
    os.environ['COURIER_C5X_LIBRARY'] = str(args.library.resolve(strict=True))
    from courier_emu.rom import CourierRom
    from courier_emu.dsp import NativeC5x
    r = CourierRom.load(args.rom)
    if hashlib.sha256(r.data).hexdigest() != 'f1c91621fbf14aad34547056feb4425e64fcf0b38ec5d57f86c9f432ca999db6':
        raise ValueError('expected captured Courier 403 firmware')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists(): raise ValueError('output already exists')
    if args.map_output and args.map_output.exists(): raise ValueError('map output already exists')
    rows = ['# baud,rate,extra,mask,phase,B,q,max_ring,k,16 native frame sizes']
    mapped = ['# baud,rate,extra,frame,cursor,next_cursor,bits,input128,direct8,ring8']
    rng = random.Random(0x564643)
    with NativeC5x(r) as c:
        o = next(o for o in r.dsp_overlays if o.index == 7)
        c.load_program(r.data[o.offset:o.offset + o.length], o.entry_word)
        c.set_mpmc_pin(0)

        def until(pc, limit=250):
            for _ in range(limit):
                c.step(1)
                if c.state()['pc'] == pc: return
            raise RuntimeError('native frame routine failed: ' + str(c.state()))

        for baud in range(6):
            for rate in range(9600, 28801, 2400):
                for extra in (0, 1):
                    c.set_data(0x3fb, 1)
                    c.set_data(0x3fd, baud)
                    c.set_data(0x3fe, (rate//2400 - 6) & 65535)
                    c.set_data(0x879, extra << 14)
                    driver = (0xbc07, 0xbf01, 0xbf09, 0x3a2, 0xbf0a, 0x879,
                              0x8b89, 0x7a80, 0xc1bf, 0x7980, 9)
                    c.load_program(struct.pack('<11H', *driver), 0)
                    c.set_pc(0)
                    until(9)
                    values = [c.data(a) for a in range(0x3a2, 0x3a8)]
                    mask, phase, b, q, m, k = values
                    if q & 0x8000: q -= 65536
                    c.load_program(struct.pack('<4H', 0xbc07, 0xbf01, 0x7980, 0xc88a), 0)
                    sizes = []
                    for _ in range(16):
                        c.set_pc(0)
                        until(0xc89c)
                        pattern = c.data(0x3fc)
                        sizes.append(8*(q+2) + sum((pattern >> (2*i)) & 3 for i in range(8)))
                    rows.append(','.join(map(str, (baud, rate, extra, mask, phase,
                                                  b, q, m, k, *sizes))))
                    if args.map_output and 0 <= q <= 7 and ((m+1) << q) <= 160:
                        # Native c81b generates g2, g4 and cumulative g8 counts.
                        setup = (0xbc07, 0xbf01, 0x6926, 0xbf09, 0xd848,
                                 0x7a80, 0xc81b, 0x7980, 7)
                        c.load_program(struct.pack('<9H', *setup), 0)
                        c.set_pc(0)
                        until(7, 10000)
                        c.set_data(0x3a3, phase)
                        driver = (0xbc07, 0xbf01, 0x7a80, 0xc88a, 0x7980, 4)
                        c.load_program(struct.pack('<6H', *driver), 0)
                        for frame, cursor in enumerate((0, 1, 63, 127)):
                            raw = (bytes(16) if frame == 0 else bytes([255])*16 if frame == 1
                                   else bytes(rng.randrange(256) for _ in range(16)))
                            for i, word in enumerate(struct.unpack('<8H', raw)):
                                c.set_data(0x22a+i, word)
                            c.set_data(0x3af, cursor)
                            c.set_data(0x3b5, 0)
                            c.set_pc(0)
                            until(4, 10000)
                            end = c.data(0x3af)
                            direct = bytes((c.data(0x246+i//2) >> (0 if i & 1 else 8))
                                           & ((1 << (q+2))-1) for i in range(8))
                            rings = bytes(c.data(0x23a+i) for i in range(8))
                            mapped.append(','.join(map(str, (baud, rate, extra, frame,
                                          cursor, end, (end-cursor) & 127))) + ','
                                          + raw.hex() + ',' + direct.hex() + ',' + rings.hex())
    args.output.write_text('\n'.join(rows) + '\n')
    if args.map_output:
        args.map_output.write_text('\n'.join(mapped) + '\n')
    print('Executed', len(rows)-1, 'native frame plans and their schedules')
    if args.map_output: print('Executed', len(mapped)-1, 'native complete frame mappings')


if __name__ == '__main__':
    main()
