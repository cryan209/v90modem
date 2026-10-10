#!/usr/bin/env python3
"""Compare production FIR coefficient sweeps with original MICA 6910/6B54.

Uses the retained bounded-history fixture. No adaptive convergence or PCM
startup is established, and zero-spacing history refresh remains unsupported.
"""
import argparse
import ctypes
import hashlib
from pathlib import Path
import subprocess
import tempfile


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mica', type=Path, default=Path.home() / 'MicaEmu')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    repo = Path(__file__).resolve().parents[1]
    class State(ctypes.Structure):
        _fields_ = [('tap', ctypes.c_uint), ('history_index', ctypes.c_uint), ('remaining', ctypes.c_uint)]
    with tempfile.TemporaryDirectory(prefix='flex-adapt-') as tmp:
        library = Path(tmp) / 'flex.so'
        subprocess.run(['cc', '-shared', '-fPIC', '-I', str(repo), str(repo / 'k56flex.c'), '-o', str(library)], check=True)
        native = ctypes.CDLL(str(library))
        pointer = ctypes.POINTER(ctypes.c_int16)
        native.k56flex_feedback_adapt.argtypes = [ctypes.POINTER(State), pointer, pointer, ctypes.c_uint, ctypes.c_uint,
                                                pointer, ctypes.c_uint, pointer, ctypes.c_uint]
        native.k56flex_feedback_adapt.restype = ctypes.c_int
        native.k56flex_feedback_fir.argtypes = [pointer, ctypes.c_uint, pointer, pointer]
        native.k56flex_feedback_fir.restype = None
        def c_adapt(tap, index, remaining, cf, hist, spacing, wrap, samples, phase, errors, error_phase):
            state = State(tap, index, remaining)
            co = (ctypes.c_int16*192)(*cf)
            h = (ctypes.c_int16*256)(*hist)
            rc = native.k56flex_feedback_adapt(ctypes.byref(state), co, h, spacing, wrap,
                                               (ctypes.c_int16*256)(*samples), phase,
                                               (ctypes.c_int16*128)(*errors), error_phase)
            assert rc == 0
            output = (ctypes.c_int16*4)()
            native.k56flex_feedback_fir((ctypes.c_int16*256)(*samples), phase, co, output)
            return ([0x0c40+state.tap, (0x7400+state.history_index)&65535, state.remaining], list(co), list(h), [v & 65535 for v in output])
        fixture = args.mica / 'artifacts/k56flex-recovery-20261004/verify_fir_adapt.py'
        source = fixture.read_text()
        source = source.replace('run(0x7210)\n h=', '''c_result=c_adapt(k,m1c-0x7400,m1d,cf,H,0,0,samples,ph,E,pe)
 run(0x7210)
 h=''')
        source = source.replace('run(0x7210)\n prog=', '''c_result=c_adapt(kin,m1c-0x7400,m1d,cf,H,m77,m78,samples,ph,E,pe)
 run(0x7210)
 prog=''')
        source = source.replace('assert got==new,', 'assert c_result[1]==got\n assert c_result[0]==[c.data(0xdae7+i) for i in range(3)]\n assert got==new,')
        source = source.replace('if expected is not None:\n', 'if expected is not None:\n  assert c_result[3]==expected\n')
        source = source.replace("assert [c.program(0x7400+i)", "assert c_result[2]==[signed(v&65535) for v in prog]\n assert [c.program(0x7400+i)")
        source = source.replace("(P/'fir-adapt-verification.json').write_text", 'output_path.write_text')
        source = source.replace('output_path.write_text', 'report.update(production_source_sha256=production_hash, production_sweeps=cases+cases2)\noutput_path.write_text')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        exec(compile(source, str(fixture), 'exec'), dict(__file__=str(fixture), c_adapt=c_adapt,
             output_path=args.output, production_hash=hashlib.sha256((repo/'k56flex.c').read_bytes()).hexdigest()))


if __name__ == '__main__':
    main()
