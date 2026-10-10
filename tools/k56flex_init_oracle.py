#!/usr/bin/env python3
"""Compare production timing initialization with original MICA 533F.

Supplied phase/correction/state; not clock acquisition.
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
    with tempfile.TemporaryDirectory(prefix='flex-init-') as tmp:
        library = Path(tmp)/'flex.so'
        subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'k56flex.c'),'-o',str(library)],check=True)
        native=ctypes.CDLL(str(library))
        native.k56flex_feedback_timing_init.argtypes=[ctypes.POINTER(ctypes.c_uint16),ctypes.c_int]
        native.k56flex_feedback_timing_init.restype=None
        def c_compare(initial,expected,table):
            state=(ctypes.c_uint16*128)(*[initial[i] for i in range(128)])
            native.k56flex_feedback_timing_init(state,table==0xbd05)
            assert list(state)==[expected[i] for i in range(128)]
        fixture=args.mica/'artifacts/k56flex-recovery-20261004/verify_timing_init.py'
        source=fixture.read_text().replace('assert got==expect,', 'c_compare(keep,expect,table)\n  assert got==expect,')
        source=source.replace("(P/'timing-init-verification.json').write_text",'output_path.write_text')
        source=source.replace('output_path.write_text','report.update(production_source_sha256=production_hash, production_cases=len(cases))\noutput_path.write_text')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        exec(compile(source,str(fixture),'exec'),dict(__file__=str(fixture),c_compare=c_compare,output_path=args.output,
             production_hash=hashlib.sha256((repo/'k56flex.c').read_bytes()).hexdigest()))


if __name__=='__main__':
    main()
