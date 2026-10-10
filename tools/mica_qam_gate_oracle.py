#!/usr/bin/env python3
"""Verify MICA K56flex first-gate control flow and B31D word selection.

Original collector/controller instructions run with supplied sliced decisions.
Sample processing, signal detector, yield and initial setup are isolated stubs;
this does not demonstrate PCM reception or a complete firmware scheduler.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mica', type=Path, default=Path.home() / 'MicaEmu')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    root = args.mica / 'artifacts/k56flex-recovery-20261004'
    core = args.mica / '.build/k56flex-core'
    sys.path.insert(0, str(core / 'python'))
    import courier_emu.dsp as dsp
    dsp.build_library = lambda **kw: core / 'libcourier_c5x.dylib'
    program = (root / 'flex.prog').read_bytes()
    cpu = dsp.NativeC5x.from_program(0, program, model='c53')
    overlay = (root / 'overlay-8e.asm').read_text()
    # Load retained overlay instruction words, preserving their PM addresses.
    for line in overlay.splitlines():
        parts = line.split()
        if not parts or not parts[0].endswith(':'):
            continue
        words = []
        for token in parts[1:]:
            if len(token) != 4 or any(c not in '0123456789abcdef' for c in token):
                break
            words.append(int(token, 16))
        if words:
            cpu.load_program(struct.pack('<' + 'H' * len(words), *words), int(parts[0][:-1], 16))

    def start(entry):
        words = [0xbc00, 0xbe47, 0xbe42, 0xbe4a, 0xbf09, 0x6800,
                 0xbf0a, 0x6400, 0x8b89, 0x7a89, entry, 0x8b00]
        cpu.load_program(struct.pack('<12H', *words), 0x7000)
        cpu.set_pc(0x7000)

    # B31D is resident and does not need waveform substitutions.
    selection = []
    table = [cpu.program(0xb317+i) for i in range(6)]
    for index, threshold in enumerate(table):
        for delta in [-1, 0, 1]:
            for forced in [0, 1]:
                cpu.set_data(0x8fa8, index)
                cpu.set_data(0x8fa9, (threshold + delta) & 0xffff)
                cpu.set_data(0xeeac, forced << 12)
                start(0xb31d)
                for _ in range(100):
                    if cpu.state()['pc'] == 0x700b:
                        break
                    cpu.step(1)
                else:
                    raise RuntimeError('B31D did not return')
                word = cpu.state()['acc'] & 0xffff
                expected = 0x8990 if delta < 0 or forced else 0x89b0
                assert word == expected, (index, threshold, delta, forced, word)
                selection.append(dict(index=index, threshold=threshold, delta=delta,
                                      forced=forced, word=word, stored=cpu.data(0x8fb8)))

    # Isolate E259's receive boundary. Its collector and DEB5..DEBF caller
    # branch remain original; no acceptance latch or gate bit is injected.
    for address in [0x3c71, 0x5a32, 0x1ef5]:
        cpu.load_program(struct.pack('<H', 0xef00), address)
    cpu.load_program(struct.pack('<2H', 0xb900, 0xef00), 0x3cbd)
    gates = []
    for valid in [False, True]:
        cpu.set_data(0x8f45, 0)  # original two-bit worker selects delay 5/23
        cpu.set_data(0x8f13, 0)
        cpu.set_data(0x8f31, 0)
        cpu.set_data(0x8e8f, 0)  # no abort from signal detector state
        word = 0x8990 | (0 if valid else 1)
        history = 0
        supplied = 0
        start(0xdeb3)
        for _ in range(1000000):
            pc = cpu.state()['pc']
            if pc in [0xdec1, 0xe07b, 0xe078, 0xde55]:
                break
            if pc == 0x5a32:
                bits = []
                for _ in range(4):
                    x = (word >> (supplied % 16)) & 1
                    y = x ^ ((history >> 4) & 1) ^ ((history >> 22) & 1)
                    history = ((history << 1) | y) & 0x7fffff
                    bits.append(y)
                    supplied += 1
                cpu.set_data(0x8d26, bits[0] | bits[1] << 1)
                cpu.set_data(0x8d27, bits[2] | bits[3] << 1)
            cpu.step(1)
        else:
            raise RuntimeError('gate fixture did not terminate')
        status = cpu.data(0x8f31)
        assert (status == 0x0800) == valid, (valid, pc, status)
        gates.append(dict(valid=valid, word=word, stop_pc=pc,
                          status=status, supplied_bits=supplied))
    report = dict(program_sha256=hashlib.sha256(program).hexdigest(),
                  overlay_sha256=hashlib.sha256(overlay.encode()).hexdigest(),
                  core_manifest=json.loads((core / 'manifest.json').read_text()),
                  training_table=table, selections=selection, first_gate=gates,
                  limitations=__doc__)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(f'{len(selection)} training-word selections and {len(gates)} first-gate paths verified')


if __name__ == '__main__':
    main()
