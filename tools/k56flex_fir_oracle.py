#!/usr/bin/env python3
"""Compare the production complex rotor with original MICA 6910 instructions.

Supplied input ring and seed coefficients; adaptation bypassed. Does not verify
coefficient acquisition, carrier lock, FIR filtering or raw PCM reception.
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
    with tempfile.TemporaryDirectory(prefix='flex-fir-') as tmp:
        library = Path(tmp) / 'flex.so'
        subprocess.run(['cc', '-shared', '-fPIC', '-I', str(repo),
                        str(repo / 'k56flex.c'), '-o', str(library)], check=True)
        native = ctypes.CDLL(str(library))
        pointer = ctypes.POINTER(ctypes.c_int16)
        native.k56flex_feedback_fir.argtypes = [pointer, ctypes.c_uint, pointer, pointer]
        native.k56flex_feedback_fir.restype = None
        def c_fir(samples, phase, coeff):
            output = (ctypes.c_int16 * 4)()
            native.k56flex_feedback_fir((ctypes.c_int16*256)(*samples), phase,
                                       (ctypes.c_int16*192)(*coeff), output)
            return [v & 65535 for v in output]
        fixture = args.mica / 'artifacts/k56flex-recovery-20261004/verify_receive_fir.py'
        source = fixture.read_text()
        source = source.replace('before=c.state()', 'assert c_fir(samples, phase, coeff) == expected\n  before=c.state()')
        source = source.replace("(P/'receive-fir-verification.json').write_text", 'output_path.write_text')
        source = source.replace('output_path.write_text', 'report.update(production_source_sha256=production_hash, production_complex_outputs=2*len(cases))\noutput_path.write_text')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        context = dict(__file__=str(fixture), c_fir=c_fir, output_path=args.output,
                       production_hash=hashlib.sha256((repo / 'k56flex.c').read_bytes()).hexdigest())
        exec(compile(source, str(fixture), 'exec'), context)


if __name__ == '__main__':
    main()
