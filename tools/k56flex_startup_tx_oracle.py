#!/usr/bin/env python3
"""Verify k56flex_startup_tx against the original module setup and symbol path.

Setup runs original 8F23(0), 94BA(1), 82B4 and 83A3 in the order overlay 8E
calls them (D698..D6A2); only the 2D3A overlay loader is stubbed, with
overlay 0x11 preloaded at the PM base 83A3 is given in DM 8EEF. Every DP118
word and the E504 pointer table must match the C initializer. Symbols then
run original 90FF, 97A2|97A8, 97C3, 43D1 (overlay 88, called directly rather
than through 2C27), 9367 and 8270 on the setup the firmware produced.
Width, producer, mapping, amplitude and gain are fixture inputs; 4A6C is not
run. No live gate timing or CONNECT claim.
"""
import argparse
import ctypes
import hashlib
import json
import random
import struct
import subprocess
import sys
import tempfile
from pathlib import Path


class Tx(ctypes.Structure):
    _fields_ = [('state', ctypes.c_uint16 * 128), ('history', ctypes.c_int16 * 64),
                ('output', ctypes.c_int16 * 128), ('history_write', ctypes.c_uint),
                ('history_read', ctypes.c_uint), ('output_write', ctypes.c_uint),
                ('producer', ctypes.c_uint), ('differential', ctypes.c_uint),
                ('samples', ctypes.c_uint32)]


def rev(value, width):
    return int(f'{value:0{width}b}'[::-1], 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mica', type=Path, default=Path.home() / 'MicaEmu')
    ap.add_argument('--symbols', type=int, default=384)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    firmware = args.mica / 'artifacts/k56flex-recovery-20261004'
    core = args.mica / '.build/k56flex-core'
    sys.path.insert(0, str(core / 'python'))
    import courier_emu.dsp as dsp
    dsp.build_library = lambda **kw: core / 'libcourier_c5x.dylib'

    repo = Path(__file__).resolve().parents[1]
    temp = tempfile.TemporaryDirectory(prefix='flex-startup-tx-')
    library = Path(temp.name) / 'flex.so'
    subprocess.run(['cc', '-shared', '-fPIC', '-I', str(repo), str(repo / 'k56flex.c'),
                    '-o', str(library)], check=True)
    native = ctypes.CDLL(str(library))
    native.k56flex_startup_tx_init.argtypes = [ctypes.POINTER(Tx), ctypes.c_uint, ctypes.c_uint,
                                               ctypes.c_uint, ctypes.c_uint16, ctypes.c_uint16]
    native.k56flex_startup_tx_symbol.argtypes = [ctypes.POINTER(Tx), ctypes.c_uint16,
                                                 ctypes.POINTER(ctypes.c_int16),
                                                 ctypes.POINTER(ctypes.c_int)]

    retained = struct.unpack('<65536H', (firmware / 'flex.data').read_bytes())
    banks = (firmware / 'overlay-11.bin').read_bytes()
    base, history_base = 0xc000, 0x7500

    def fresh():
        cpu = dsp.NativeC5x.from_program(0, (firmware / 'flex.prog').read_bytes(), model='c53')
        cpu.load_program((firmware / 'overlay-88.bin').read_bytes(), 0x4000)
        cpu.load_program(banks, base)
        cpu.load_program(struct.pack('<H', 0xef00), 0x2d3a)  # loader stub: ret
        for i in range(8):
            cpu.set_data(0x72a0 + i, retained[0x72a0 + i])
        return cpu

    def run(cpu, words, limit=20000):
        # SPM=0, OVM=0, external PM view, AR1 stack, as the original callers.
        prefix = [0xbc00, 0xbe47, 0xbe42, 0xbe4a, 0xae07, 2,
                  0xbf09, 0x6800, 0xbf00, 0xbd18, 0x8b89]
        program = prefix + words
        cpu.load_program(struct.pack(f'<{len(program)}H', *[x & 65535 for x in program]), 0x7000)
        stop = 0x7000 + len(program)
        cpu.set_pc(0x7000)
        for _ in range(limit):
            if cpu.state()['pc'] == stop:
                return
            cpu.step(1)
        raise AssertionError(('original code did not return', cpu.state()))

    setup = [0xb900, 0x7a89, 0x8f23, 0xb901, 0x7a89, 0x94ba,
             0x7a89, 0x82b4, 0x7a89, 0x83a3]
    rng = random.Random(0x82b483a3)
    cases, total = [], 0
    for width in [2, 4]:
        for producer in range(3):
            for differential in [0, 1]:
                for amplitude, gain in [(4096, 4096), (9159, 16384)]:
                    cpu = fresh()
                    for j in range(128):
                        cpu.set_data(0x8c00 + j, 0)
                    for j in range(64):
                        cpu.set_data(history_base + j, rng.randrange(65536))  # 82B4 must clear
                    cpu.set_data(0x8ed5, history_base)
                    cpu.set_data(0x8eef, base)
                    cpu.set_data(0x8c1f, width)
                    cpu.set_data(0x8c3e, amplitude)
                    cpu.set_data(0x8c11, gain)
                    cpu.set_data(0x8c21, [0x8f6f, 0x8f74, 0xbe68][producer])
                    run(cpu, setup)
                    tx = Tx()
                    assert native.k56flex_startup_tx_init(ctypes.byref(tx), width, producer,
                                                          differential, amplitude, gain) == 0
                    label = (width, producer, differential, amplitude)
                    for j in range(128):
                        if j in (0x19, 0x1a):
                            continue
                        assert cpu.data(0x8c00 + j) == tx.state[j], (label, 'setup', hex(j),
                                                                    hex(cpu.data(0x8c00 + j)), hex(tx.state[j]))
                    assert cpu.data(0x8c1a) == history_base + rev(tx.history_write, 6), label
                    assert cpu.data(0x8c19) == history_base + rev(tx.history_read, 6), label
                    assert [cpu.data(history_base + j) for j in range(64)] == [0] * 64, label
                    table = [cpu.data(0xe504 + j) for j in range(11)]
                    assert table == [base + 46 * j for j in range(11)], (label, table)

                    cpu.set_data(0x8c58, 0xd8b0)
                    cpu.set_data(0x8c5a, 100)
                    cpu.set_data(0xec6c, 0x7700)
                    cpu.set_data(0x8eea, 0x7e00)
                    cpu.set_data(0x7e00, 0)
                    for j in range(128):
                        cpu.set_data(0x7700 + j, 0)
                    symbol = [0x7a89, 0x90ff, 0x7a89, 0x97a8 if differential else 0x97a2,
                              0x7a89, 0x97c3, 0xbf0a, 0x8c0f, 0x8b8b,
                              0x7a8b, 0x43d1, 0x8b8a, 0x7a8a, 0x9367,
                              0xb901, 0x7a8a, 0x8270]
                    samples, consumed = [], 0
                    for tick in range(args.symbols):
                        word = rng.randrange(65536)
                        cpu.set_data(0x8c59, 0xd8a0)
                        cpu.set_data(0xd8a0, word)
                        pcm, used = (ctypes.c_int16 * 4)(), ctypes.c_int()
                        count = native.k56flex_startup_tx_symbol(ctypes.byref(tx), word, pcm,
                                                                 ctypes.byref(used))
                        assert count in (3, 4), (label, tick, count)
                        run(cpu, symbol)
                        where = (label, tick)
                        for j in range(128):
                            if j in (0x19, 0x1a, 0x58, 0x59, 0x5a):  # physical cursors, fixture queue
                                continue
                            assert cpu.data(0x8c00 + j) == tx.state[j], (where, hex(j),
                                                                        cpu.data(0x8c00 + j), tx.state[j])
                        assert cpu.data(0x8c19) == history_base + rev(tx.history_read, 6), where
                        assert cpu.data(0x8c1a) == history_base + rev(tx.history_write, 6), where
                        assert cpu.data(0xec6c) == 0x7700 + rev(tx.output_write, 7), where
                        assert [cpu.data(history_base + rev(j, 6)) for j in range(64)] == \
                            [x & 65535 for x in tx.history], (where, 'history')
                        assert [cpu.data(0x7700 + rev(j, 7)) for j in range(128)] == \
                            [x & 65535 for x in tx.output], (where, 'samples')
                        assert cpu.data(0x7e00) == tx.samples & 65535, where
                        samples.extend(pcm[:count])
                        consumed += used.value
                    assert len(samples) == args.symbols * 10 // 3, label
                    total += len(samples)
                    cases.append(dict(width=width, producer=producer, differential=differential,
                                      amplitude=amplitude, gain=gain, symbols=args.symbols,
                                      samples=len(samples), consumed_words=consumed,
                                      pcm_sha256=hashlib.sha256(struct.pack(f'<{len(samples)}h', *samples)).hexdigest()))
    report = dict(production_source_sha256=hashlib.sha256((repo / 'k56flex.c').read_bytes()).hexdigest(),
                  configurations=len(cases), samples=total, cases=cases, limitations=__doc__,
                  firmware={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                            [firmware / 'flex.prog', firmware / 'flex.data',
                             firmware / 'overlay-88.bin', firmware / 'overlay-11.bin']},
                  core_manifest=json.loads((core / 'manifest.json').read_text()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(f'{len(cases)} configurations, {total} samples: setup and symbols match original DSP')


if __name__ == '__main__':
    main()
