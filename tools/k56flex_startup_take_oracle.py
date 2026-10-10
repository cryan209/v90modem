#!/usr/bin/env python3
"""Verify bounded original 90FF symbol extraction with all three producers.

Supplied input word and history; no bit extraction, scheduler or CONNECT claim.
"""
import argparse
import ctypes
import subprocess
import tempfile
import hashlib
import json
from pathlib import Path
import random


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mica',type=Path,default=Path.home()/'MicaEmu')
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();root=args.mica/'artifacts/k56flex-recovery-20261004'
    fixture=root/'verify_parameters.py'
    source=fixture.read_text().split('rng=random.Random')[0].replace("sys.path.insert(0,'/Users/scottcryan/courier-emu')",f"sys.path.insert(0,{str(args.mica/'.build/k56flex-core/python')!r})").replace("Path('/Users/scottcryan/courier-emu/.build/libcourier_c5x.dylib')",f"Path({str(args.mica/'.build/k56flex-core/libcourier_c5x.dylib')!r})")
    ctx=dict(__file__=str(fixture));exec(compile(source,str(fixture),'exec'),ctx)
    cpu,run,patch=ctx['c'],ctx['run'],ctx['patch']
    def rev(v):return int(f'{v&65535:016b}'[::-1],2)
    def step(addr):return rev(rev(addr)+rev(0x1000))
    repo=Path(__file__).resolve().parents[1]
    temp=tempfile.TemporaryDirectory(prefix='flex-source-pair-');library=Path(temp.name)/'flex.so'
    subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'k56flex.c'),'-o',str(library)],check=True)
    native=ctypes.CDLL(str(library));pointer=ctypes.POINTER(ctypes.c_int16)
    native.k56flex_feedback_startup_take.argtypes=[ctypes.POINTER(ctypes.c_uint16),ctypes.c_uint16,ctypes.c_uint]
    native.k56flex_feedback_startup_take.restype=ctypes.c_int
    rng=random.Random(0x90ff);cases=[]
    for producer,entry in enumerate([0x8f6f,0x8f74,0xbe68]):
        patch(0x7240,[0xbc00,0xae07,2,0x7a89,0x90ff,0xef00])
        for width in range(1,16):
            for remaining in range(16):
                for repeat in range(4):
                    state=(ctypes.c_uint16*128)()
                    for offset in [1,2,0x12,0x79]:state[offset]=rng.randrange(65536)
                    state[4]=remaining;state[0x1f]=width;state[0x21]=entry
                    state[0x58]=0xd8b0;state[0x59]=0xd8a0;state[0x5a]=100
                    word=rng.randrange(65536)
                    for offset in range(128):cpu.set_data(0x8c00+offset,state[offset])
                    cpu.set_data(0xd8a0,word)
                    consumed=native.k56flex_feedback_startup_take(state,word,producer)
                    assert consumed==int(width>remaining)
                    run(0x7240)
                    for offset in [1,2,4,7,8,0x79]:
                        assert cpu.data(0x8c00+offset)==state[offset],(producer,width,remaining,repeat,hex(offset),cpu.data(0x8c00+offset),state[offset])
                    cases.append(dict(producer=producer,width=width,remaining=remaining,input=word,symbol=state[7],consumed=consumed))
    report=dict(production_source_sha256=hashlib.sha256((repo/'k56flex.c').read_bytes()).hexdigest(),cases=cases,comparisons=len(cases),limitations=__doc__,
        program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
        core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'symbol extractions matched')


if __name__=='__main__':main()
