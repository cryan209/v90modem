#!/usr/bin/env python3
"""Recover hard-decision response acquisition using original MICA instructions.

Requires the read-only sibling MicaEmu recovery and isolated C53 library.
No waveform demodulator, scheduler, or live handshake is claimed.
"""
import ctypes
import subprocess
import tempfile
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
    recovery = args.mica / 'artifacts/k56flex-recovery-20261004'
    core = args.mica / '.build/k56flex-core'
    sys.path.insert(0, str(core / 'python'))
    import courier_emu.dsp as dsp
    dsp.build_library = lambda **kw: core / 'libcourier_c5x.dylib'
    program = (recovery / 'flex.prog').read_bytes()
    cpu = dsp.NativeC5x.from_program(0, program, model='c53')

    def invoke(entry):
        # Same isolated call ABI as the existing MICA recovery fixtures.
        words = [0xbc00, 0xbe47, 0xbe42, 0xbe4a, 0xbf09, 0x6800,
                 0xbf0a, 0x6400, 0x8b89, 0x7a89, entry, 0x8b00]
        cpu.load_program(struct.pack('<12H', *words), 0x7000)
        cpu.set_pc(0x7000)
        for _ in range(20000):
            if cpu.state()['pc'] == 0x700b:
                return cpu.state()['acc'] & 0xffff
            cpu.step(1)
        raise RuntimeError(cpu.state())

    # Compile the production collector and a small ABI wrapper in a temporary
    # directory; compare every original collector call, including its latency.
    repo = Path(__file__).resolve().parents[1]
    temp = tempfile.TemporaryDirectory(prefix='k56flex-response-')
    work = Path(temp.name)
    (work / 'wrapper.c').write_text('#include "mica_qam.h"\nstatic mica_qam_word_rx_t rx;\nint reset(unsigned tap) { return mica_qam_word_rx_init(&rx, tap); }\nint feed(unsigned a, unsigned b) {\n    mica_qam_word_rx_dibit(&rx, a);\n    return mica_qam_word_rx_dibit(&rx, b);\n}\nunsigned word(void) { return rx.word; }\n')
    library = work / 'response.so'
    subprocess.run(['cc', '-shared', '-fPIC', '-I', str(repo),
                    str(work / 'wrapper.c'), str(repo / 'mica_qam.c'),
                    '-o', str(library)], check=True)
    native = ctypes.CDLL(str(library))
    assert native.reset(0) == -1
    cases = []
    # 1D0E sets header mask 888F/value 8880. 1D27 recognises a repeated
    # sixteen-bit word, unlike the 24-bit extended-report collector 1D5A.
    # Exercise both original descrambler selections and every dibit alignment.
    for selector, tap in [(0, 5), (1, 18)]:
        for payload in range(32):
            for offset in range(8):
                for mode in ["invalid", "valid", "mismatch-recovery"]:
                    valid = mode != "invalid"
                    cpu.set_data(0x8f45, selector)
                    invoke(0x1d0e)
                    cpu.set_data(0x8f13, 0)
                    word = 0x8880 | (((payload * 0x1230) & 0x7770)) | (0 if valid else 1)
                    bits = [0] * (2 * offset) + [
                        ((word ^ (0x10 if mode == "mismatch-recovery" and repeat == 2 else 0)) >> i) & 1
                        for repeat in range(12) for i in range(16)]
                    history = 0
                    encoded = []
                    for bit in bits:
                        y = bit ^ ((history >> (tap - 1)) & 1) ^ ((history >> 22) & 1)
                        history = ((history << 1) | y) & 0x7fffff
                        encoded.append(y)
                    assert native.reset(tap) == 0
                    accepted = None
                    for n in range(0, len(encoded) - 3, 4):
                        cpu.set_data(0x8d26, encoded[n] | (encoded[n+1] << 1))
                        cpu.set_data(0x8d27, encoded[n+2] | (encoded[n+3] << 1))
                        invoke(0x1d27)
                        got = native.feed(encoded[n] | (encoded[n+1] << 1),
                                          encoded[n+2] | (encoded[n+3] << 1))
                        assert bool(got) == bool(cpu.data(0x8f08)), (selector, word, offset, n, got)
                        if got:
                            assert native.word() == cpu.data(0x8f27)
                        if cpu.data(0x8f08):
                            accepted = n + 4
                            break
                    assert (accepted is not None) == valid, (selector, word, offset, accepted)
                    if valid:
                        assert cpu.data(0x8f27) == word
                    cases.append(dict(selector=selector, tap=tap, word=word,
                                      offset=offset, valid=valid, mode=mode, accepted_bits=accepted))
    report = dict(program_sha256=hashlib.sha256(program).hexdigest(),
                  core_manifest=json.loads((core / 'manifest.json').read_text()),
                  cases=cases, comparisons=len(cases),
                  limitations='Original 1D0E/1D27/1D9E instructions; injected hard-decision dibits only. '
                  'The two tap selections are tested, not assigned to a live client role. '
                  'No receive filter, oscillator, scheduler, bank dispatch or gate setter exercised.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(f'{len(cases)} original-instruction response acquisition cases passed')


if __name__ == '__main__':
    main()
