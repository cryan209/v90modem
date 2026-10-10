#!/usr/bin/env python3
"""Original 4BB1 predictor phase/phasor update versus production C.

Original 6D93 loop filter composed with the phase tail and retained cosine
table. Does not verify 4AF3 error acquisition or a full receiver/carrier lock.
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
    data=(root/'flex.data').read_bytes();table=struct.unpack_from('<512h',data,0x7800*2)
    for i,v in enumerate(table):cpu.set_data(0x7800+i,v&65535)
    patch(0x7260,[0xbc00,0xae07,2,0xbd19,0x7a89,0x6d93,0x9072,0x9873,0xef00])
    patch(0x7240,[0x8aa0,0xbc00,0xae07,2,0xbd19,0x6a70,0x6271,0xbe1e,0x7980,0x4bb1])
    repo=Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='flex-predictor-phase-') as tmp:
        library=Path(tmp)/'flex.so'
        subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'k56flex.c'),'-o',str(library)],check=True)
        native=ctypes.CDLL(str(library))
        native.k56flex_feedback_predictor_phase.argtypes=[ctypes.POINTER(ctypes.c_uint16),ctypes.c_uint32,ctypes.POINTER(ctypes.c_int16)]
        native.k56flex_feedback_predictor_phase.restype=None
        native.k56flex_feedback_predictor_increment.argtypes=[ctypes.POINTER(ctypes.c_uint16)]
        native.k56flex_feedback_predictor_increment.restype=ctypes.c_uint32
        cases=[];rng=random.Random(0x4bb1)
        for trial in range(2048):
            phase=rng.randrange(1<<32);increment=rng.randrange(1<<32)
            state=(ctypes.c_uint16*128)(*[rng.randrange(65536) for _ in range(128)])
            state[0x3a]=phase>>16;state[0x3b]=phase&65535
            state[0x70]=increment>>16;state[0x71]=increment&65535
            for i,v in enumerate(state):cpu.set_data(0x8c80+i,v)
            increment=native.k56flex_feedback_predictor_increment(state)
            run(0x7260)
            observed=(cpu.data(0x8cf3)<<16)|cpu.data(0x8cf2)
            assert observed==increment,(trial,hex(observed),hex(increment))
            for offset in [0x38,0x39,0x55]:
                assert cpu.data(0x8c80+offset)==state[offset],(trial,offset)
            state[0x70]=increment>>16;state[0x71]=increment&65535
            cpu.set_data(0x8cf0,state[0x70]);cpu.set_data(0x8cf1,state[0x71])
            native.k56flex_feedback_predictor_phase(state,increment,(ctypes.c_int16*512)(*table))
            run(0x7240)
            for offset in [0x3a,0x3b,0x3c,0x3d]:
                assert cpu.data(0x8c80+offset)==state[offset],(trial,offset,cpu.data(0x8c80+offset),state[offset])
            cases.append(dict(phase=phase,increment=increment,outputs=[state[i] for i in [0x3a,0x3b,0x3c,0x3d]]))
    report=dict(core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()),cases=cases,comparisons=len(cases),limitations=__doc__,
        program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
        data_sha256=hashlib.sha256(data).hexdigest(),production_source_sha256=hashlib.sha256((repo/'k56flex.c').read_bytes()).hexdigest())
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'predictor phase and phasor cases matched')


if __name__=='__main__':main()
