#!/usr/bin/env python3
"""Compare the production complex rotor with original MICA 5A54 instructions.

Supplied coefficients and coordinates, saturation disabled. Does not verify
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
    with tempfile.TemporaryDirectory(prefix='flex-rotation-') as tmp:
        library = Path(tmp) / 'flex.so'
        subprocess.run(['cc', '-shared', '-fPIC', '-I', str(repo),
                        str(repo / 'mica_qam.c'), '-o', str(library)], check=True)
        native = ctypes.CDLL(str(library))
        native.mica_qam_rotate.argtypes = [ctypes.c_int16]*5 + [ctypes.POINTER(ctypes.c_int16)]*2
        native.mica_qam_rotate.restype = None
        def c_rotate(x, y, u, v, bias):
            a, b = ctypes.c_int16(), ctypes.c_int16()
            native.mica_qam_rotate(x, y, u, v, bias, ctypes.byref(a), ctypes.byref(b))
            return [a.value & 65535, b.value & 65535]
        fixture = args.mica / 'artifacts/k56flex-recovery-20261004/verify_receive_rotation.py'
        source = fixture.read_text()
        source = source.replace('run(0x7210)\n', '''assert c_rotate(*values[:2],*weights[:2],bias)+c_rotate(*values[2:],*weights[2:],bias) == wanted
  run(0x7210)
''')
        source = source.replace("(P/'receive-rotation-verification.json').write_text", 'output_path.write_text')
        source = source.replace('output_path.write_text', 'report.update(production_source_sha256=production_hash, production_complex_pairs=2*len(cases))\noutput_path.write_text')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        context = dict(__file__=str(fixture), c_rotate=c_rotate, output_path=args.output,
                       production_hash=hashlib.sha256((repo / 'mica_qam.c').read_bytes()).hexdigest())
        exec(compile(source, str(fixture), 'exec'), context)


if __name__ == '__main__':
    main()
