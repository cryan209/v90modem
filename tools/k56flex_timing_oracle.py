#!/usr/bin/env python3
"""Compare production timing controller with original MICA 45B8/460E/535B.

Bounded nonsaturating fixture, supplied ring/state; not timing acquisition.
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
    with tempfile.TemporaryDirectory(prefix='flex-timing-') as tmp:
        library = Path(tmp)/'flex.so'
        subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'k56flex.c'),'-o',str(library)],check=True)
        native=ctypes.CDLL(str(library))
        native.k56flex_feedback_timing.argtypes=[ctypes.POINTER(ctypes.c_uint16),ctypes.POINTER(ctypes.c_int16),ctypes.c_uint,ctypes.c_uint]
        native.k56flex_feedback_timing.restype=ctypes.c_int
        native.k56flex_feedback_phase.argtypes=[ctypes.POINTER(ctypes.c_uint16)]
        native.k56flex_feedback_phase.restype=None
        def join(state):
            actual=(ctypes.c_uint16*128)(*[state[i] for i in range(128)])
            native.k56flex_feedback_phase(actual)
            return list(actual)
        def compare(state,ring,p,tick,expected):
            actual=(ctypes.c_uint16*128)(*[state[i] for i in range(128)])
            rc=native.k56flex_feedback_timing(actual,(ctypes.c_int16*256)(*ring),p,tick)
            assert rc==0,(rc,p,tick)
            assert list(actual)==[expected[i] for i in range(128)], {i:(actual[i],expected[i]) for i in range(128) if actual[i]!=expected[i]}
        fixture=args.mica/'artifacts/k56flex-recovery-20261004/verify_timing_loop.py'
        source=fixture.read_text().replace('for n,v in S.items():', 'compare(S,ring,p,tick,exp)\n for n,v in S.items():')
        source=source.replace('key=(last[0]', 'joined=join(got)\n run(0x542d)\n assert joined==[c.data(B+n) for n in range(128)]\n key=(last[0]')
        source=source.replace("(P/'timing-loop-verification.json').write_text",'output_path.write_text')
        source=source.replace('output_path.write_text','report.update(production_source_sha256=production_hash, production_cases=len(cases), joined_timing_phase_cases=len(cases))\noutput_path.write_text')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        exec(compile(source,str(fixture),'exec'),dict(__file__=str(fixture),compare=compare,join=join,output_path=args.output,
             production_hash=hashlib.sha256((repo/'k56flex.c').read_bytes()).hexdigest()))


if __name__=='__main__':
    main()
