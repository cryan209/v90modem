#!/usr/bin/env python3
"""Original 6D4E correlation angle, including 6831/6890, versus production C.

Supplied accumulated correlations; no full carrier acquisition claimed.
"""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import random
import struct
import subprocess
import tempfile


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mica',type=Path,default=Path.home()/'MicaEmu')
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();root=args.mica/'artifacts/k56flex-recovery-20261004'
    fixture=root/'verify_parameters.py'
    source=fixture.read_text().split('rng=random.Random')[0].replace("sys.path.insert(0,'/Users/scottcryan/courier-emu')",f"sys.path.insert(0,{str(args.mica/'.build/k56flex-core/python')!r})").replace("Path('/Users/scottcryan/courier-emu/.build/libcourier_c5x.dylib')",f"Path({str(args.mica/'.build/k56flex-core/libcourier_c5x.dylib')!r})")
    ctx=dict(__file__=str(fixture));exec(compile(source,str(fixture),'exec'),ctx)
    cpu,run,patch=ctx['c'],ctx['run'],ctx['patch']
    patch(0x7260,[0xbc00,0xae07,2,0xbd19,0xbf00,0xbf0a,0xd930,0x8b89,0x7a89,0x6d4e,0x9072,0x9873,0xef00])
    repo=Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='flex-predictor-phase-') as tmp:
        library=Path(tmp)/'flex.so'
        subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'k56flex.c'),'-o',str(library)],check=True)
        native=ctypes.CDLL(str(library))
        native.k56flex_feedback_predictor_angle.argtypes=[ctypes.POINTER(ctypes.c_uint16)]
        native.k56flex_feedback_predictor_angle.restype=ctypes.c_uint32
        cases=[];rng=random.Random(0x6d4e)
        values=[0,1,-1,7,-7,65535,-65535,65536,-65536,0x7fffffff,-0x80000000]
        pairs=[(x,y) for x in values for y in values]
        pairs += [(rng.randrange(-0x80000000,0x80000000),rng.randrange(-0x80000000,0x80000000)) for _ in range(4096)]
        for trial,(x,y) in enumerate(pairs):
            initial=[(x>>16)&65535,x&65535,(y>>16)&65535,y&65535]
            correlation=(ctypes.c_uint16*4)(*initial)
            for i,v in enumerate(initial):cpu.set_data(0xd930+i,v)
            expected=native.k56flex_feedback_predictor_angle(correlation)
            run(0x7260)
            observed=(cpu.data(0x8cf3)<<16)|cpu.data(0x8cf2)
            assert observed==expected,(trial,x,y,hex(observed),hex(expected))
            cases.append(dict(initial=initial,angle=observed))
    report=dict(core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()),cases=cases,comparisons=len(cases),limitations=__doc__,
        program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
        production_source_sha256=hashlib.sha256((repo/'k56flex.c').read_bytes()).hexdigest())
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'predictor angle cases matched')


if __name__=='__main__':main()
