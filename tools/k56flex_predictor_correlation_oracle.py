#!/usr/bin/env python3
"""Original mode-15 4B59..4B86 complex correlation versus production C.

Stops before timer/angle conversion; no full carrier acquisition claimed.
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
    patch(0x7260,[0xbc00,0xae07,2,0xbd19,0x7980,0x4b59])
    class Done(Exception): pass
    def stop(pc):
        if pc==0x4b87: raise Done()
    repo=Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='flex-predictor-phase-') as tmp:
        library=Path(tmp)/'flex.so'
        subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'k56flex.c'),'-o',str(library)],check=True)
        native=ctypes.CDLL(str(library))
        native.k56flex_feedback_predictor_correlate.argtypes=[ctypes.POINTER(ctypes.c_uint16),ctypes.POINTER(ctypes.c_int16),ctypes.POINTER(ctypes.c_int16)]
        native.k56flex_feedback_predictor_correlate.restype=None
        cases=[];rng=random.Random(0x4b59)
        for trial in range(2048):
            inputs=[rng.randrange(-32768,32768) for _ in range(6)]
            reference=[rng.randrange(-32768,32768) for _ in range(6)]
            initial=[rng.randrange(65536) for _ in range(4)]
            correlation=(ctypes.c_uint16*4)(*initial)
            for base,words in [(0x8ccf,inputs),(0xd920,reference),(0xd930,initial)]:
                for i,v in enumerate(words):cpu.set_data(base+i,v&65535)
            native.k56flex_feedback_predictor_correlate(correlation,(ctypes.c_int16*6)(*inputs),(ctypes.c_int16*6)(*reference))
            try: run(0x7260,hook=stop)
            except Done: pass
            observed=[cpu.data(0xd930+i) for i in range(4)]
            assert observed==list(correlation),(trial,inputs,reference,initial,observed,list(correlation))
            cases.append(dict(inputs=inputs,reference=reference,initial=initial,outputs=observed))
    report=dict(core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()),cases=cases,comparisons=len(cases),limitations=__doc__,
        program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
        production_source_sha256=hashlib.sha256((repo/'k56flex.c').read_bytes()).hexdigest())
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'mode-15 predictor correlation cases matched')


if __name__=='__main__':main()
