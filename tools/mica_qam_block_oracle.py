#!/usr/bin/env python3
"""Compare production block cadence with original MICA 6C50.

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
    with tempfile.TemporaryDirectory(prefix='flex-block-') as tmp:
        library = Path(tmp)/'flex.so'
        subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'mica_qam.c'),'-o',str(library)],check=True)
        native=ctypes.CDLL(str(library))
        native.mica_qam_block_count.argtypes=[ctypes.POINTER(ctypes.c_uint16)]
        native.mica_qam_block_count.restype=ctypes.c_uint
        def c_block(initial):
            state=(ctypes.c_uint16*128)(*[initial[i] for i in range(128)])
            native.mica_qam_block_count(state)
            return list(state)
        fixture=args.mica/'artifacts/k56flex-recovery-20261004/verify_block_count.py'
        source=fixture.read_text().replace('before46=c.data(D+0x46)', 'production=c_block({n:c.data(D+n) for n in range(128)})\n before46=c.data(D+0x46)')
        source=source.replace("cases.append(", "assert all(production[n]==c.data(D+n) for n in [0x42,0x43,0x2b])\n cases.append(")
        source=source.replace("(P/'block-count-verification.json').write_text",'output_path.write_text')
        source=source.replace('output_path.write_text','report.update(production_source_sha256=production_hash, production_cases=len(cases))\noutput_path.write_text')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        exec(compile(source,str(fixture),'exec'),dict(__file__=str(fixture),c_block=c_block,output_path=args.output,
             production_hash=hashlib.sha256((repo/'mica_qam.c').read_bytes()).hexdigest()))


if __name__=='__main__':
    main()
