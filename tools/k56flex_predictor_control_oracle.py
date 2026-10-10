#!/usr/bin/env python3
"""Complete original 4AF3 mode-15 predictor controller versus production C.

Supplied input/reference pairs and state; no live carrier acquisition claimed.
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
    cpu.set_data(0xee9b,15)
    patch(0x7260,[0xbc00,0xae07,2,0xbd19,0x8b89,0x7a89,0x4af3,0xef00])
    repo=Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='flex-predictor-phase-') as tmp:
        library=Path(tmp)/'flex.so'
        subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'k56flex.c'),'-o',str(library)],check=True)
        native=ctypes.CDLL(str(library))
        native.k56flex_feedback_predictor_control15.argtypes=[ctypes.POINTER(ctypes.c_uint16),ctypes.POINTER(ctypes.c_uint16),ctypes.POINTER(ctypes.c_int16),ctypes.POINTER(ctypes.c_int16),ctypes.POINTER(ctypes.c_int16)]
        native.k56flex_feedback_predictor_control15.restype=None
        cases=[];rng=random.Random(0x4af3)
        for trial in range(4096):
            inputs=[rng.randrange(-32768,32768) for _ in range(6)]
            reference=[rng.randrange(-32768,32768) for _ in range(6)]
            initial=[rng.randrange(65536) for _ in range(4)]
            words=[rng.randrange(65536) for _ in range(128)]
            words[0x57]=rng.choice([0,32767]);words[0x2c]=rng.choice([0,1,32767])
            words[0x4e]=rng.choice([0,1,2,40,65535])
            state=(ctypes.c_uint16*128)(*words)
            correlation=(ctypes.c_uint16*4)(*initial)
            for base,values in [(0x8c80,words),(0x8ccf,inputs),(0xd920,reference),(0xd930,initial)]:
                for i,v in enumerate(values):cpu.set_data(base+i,v&65535)
            # 8CCF overlaps DP119; these six words are unchanged by the controller.
            for i,v in enumerate(inputs):state[0x4f+i]=v&65535
            native.k56flex_feedback_predictor_control15(state,correlation,(ctypes.c_int16*6)(*inputs),(ctypes.c_int16*6)(*reference),(ctypes.c_int16*512)(*table))
            run(0x7260)
            observed=[cpu.data(0xd930+i) for i in range(4)]
            assert observed==list(correlation),(trial,'correlation',observed,list(correlation))
            for offset in [0x33,0x34,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x4e]:
                assert cpu.data(0x8c80+offset)==state[offset],(trial,hex(offset),cpu.data(0x8c80+offset),state[offset])
            cases.append(dict(enabled=words[0x57],loop=words[0x2c],timer=words[0x4e],outputs=[state[i] for i in [0x33,0x34,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x4e]],correlation=observed))
        sequences=[]
        for trial in range(32):
            state=(ctypes.c_uint16*128)()
            state[0x57]=32767;state[0x2c]=1;state[0x4e]=rng.choice([0,40])
            state[0x36]=rng.randrange(32768);state[0x37]=rng.randrange(32768)
            correlation=(ctypes.c_uint16*4)()
            for i,v in enumerate(state):cpu.set_data(0x8c80+i,v)
            for i in range(4):cpu.set_data(0xd930+i,0)
            for tick in range(128):
                inputs=[rng.randrange(-16384,16384) for _ in range(6)]
                reference=[rng.randrange(-16384,16384) for _ in range(6)]
                for base,values in [(0x8ccf,inputs),(0xd920,reference)]:
                    for i,v in enumerate(values):cpu.set_data(base+i,v&65535)
                native.k56flex_feedback_predictor_control15(state,correlation,(ctypes.c_int16*6)(*inputs),(ctypes.c_int16*6)(*reference),(ctypes.c_int16*512)(*table))
                run(0x7260)
                assert [cpu.data(0xd930+i) for i in range(4)]==list(correlation),(trial,tick,'correlation')
                for offset in [0x33,0x34,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x4e]:
                    assert cpu.data(0x8c80+offset)==state[offset],(trial,tick,hex(offset))
            sequences.append(dict(ticks=128,phase=[state[0x3a],state[0x3b]],integrator=[state[0x38],state[0x39]]))
    report=dict(core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()),cases=cases,comparisons=len(cases),sequences=sequences,sequence_ticks=128*len(sequences),limitations=__doc__,
        program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
        production_source_sha256=hashlib.sha256((repo/'k56flex.c').read_bytes()).hexdigest())
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'independent controller cases and',128*len(sequences),'continuous controller ticks matched')


if __name__=='__main__':main()
