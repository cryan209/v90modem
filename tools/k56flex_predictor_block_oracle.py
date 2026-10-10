#!/usr/bin/env python3
"""Verify joined three-pair 6C50 processing against the production block.

Workspace pointers and the stack profile selector are isolated fixture inputs.
Capture callback is a RET; supplied source/raw rings, no live CONNECT claim.
"""
import argparse
import ctypes
import subprocess
import tempfile
import hashlib
import json
import random
from pathlib import Path


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mica',type=Path,default=Path.home()/'MicaEmu')
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    root=args.mica/'artifacts/k56flex-recovery-20261004'
    fixture=root/'verify_parameters.py'
    source=fixture.read_text().split('rng=random.Random')[0]
    source=source.replace("sys.path.insert(0,'/Users/scottcryan/courier-emu')",f"sys.path.insert(0,{str(args.mica/'.build/k56flex-core/python')!r})")
    source=source.replace("Path('/Users/scottcryan/courier-emu/.build/libcourier_c5x.dylib')",f"Path({str(args.mica/'.build/k56flex-core/libcourier_c5x.dylib')!r})")
    ctx=dict(__file__=str(fixture));exec(compile(source,str(fixture),'exec'),ctx)
    cpu,run,patch=ctx['c'],ctx['run'],ctx['patch']
    # Original descriptor initializer uses SARAM program/data aliasing.
    patch(0x7220,[0xbc00,0x5d07,0x32,0xef00]);run(0x7220)
    for addr,value in {0x8ed9:0x0c40,0x8edc:0x6500,0x8eed:0x6200,0x67ff:0,0x6800:0}.items():
        cpu.set_data(addr,value)
    run(0x6bf4)
    descriptor=[cpu.data(0xdab7+i) for i in range(27)]
    report=dict(descriptor=descriptor,configuration=[cpu.data(0xdaa9+i) for i in range(14)],
                block_words={hex(a):cpu.data(a) for a in [0x8cc4,0x8cc5,0x8cc6,0x8cc7,0x8cc8,0x8cc9,0x8cca,0x8ccc,0x8ce9]},
                cleared_coefficient_words=[cpu.data(0x0c40+i) for i in range(288)],
                program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
                core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()),limitations=__doc__)
    assert descriptor[3]==0x8cc8 and descriptor[24]==0x8cc9
    assert descriptor[11]==2 and descriptor[13]==48
    assert all(v==0 for v in report['cleared_coefficient_words'])
    def rev(n,bits):return int(f'{n%(1<<bits):0{bits}b}'[::-1],2)
    def signed(n):return (n&32767)-(n&32768)
    patch(0x7250,[0xbc00,0xae07,0x32,0xbf00,0xaea0,0xdab7,0x7a89,0x6910,0x7c01,0xef00])
    cpu.set_data(0x6200,0)
    patch(0x7270,[0xbc00,0xae07,0x32,0xbf0a,0x6201,0x8b89,0x7a89,0x6cf8,0xef00]);run(0x7270)
    adapted_descriptor=[cpu.data(0xdab7+i) for i in range(27)]
    repo=Path(__file__).resolve().parents[1]
    class State(ctypes.Structure):
        _fields_=[('tap',ctypes.c_uint),('history_index',ctypes.c_uint),('remaining',ctypes.c_uint)]
    with tempfile.TemporaryDirectory(prefix='flex-predictor-adapt-') as tmp:
        library=Path(tmp)/'flex.so'
        subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'k56flex.c'),'-o',str(library)],check=True)
        native=ctypes.CDLL(str(library));pointer=ctypes.POINTER(ctypes.c_int16)
        native.k56flex_feedback_predictor_block.argtypes=[ctypes.POINTER(ctypes.c_uint16),ctypes.POINTER(State),pointer,pointer,pointer,ctypes.c_uint,pointer,pointer,ctypes.c_uint,ctypes.c_uint,ctypes.c_int,pointer,ctypes.POINTER(ctypes.c_uint16),pointer]
        native.k56flex_feedback_predictor_block.restype=ctypes.c_int
        import struct
        retained=(root/'flex.data').read_bytes()
        table=struct.unpack_from('<512h',retained,0x7800*2)
        for address in range(0x128f,0x13d2):cpu.set_data(address,struct.unpack_from('<H',retained,2*address)[0])
        for i,v in enumerate(table):cpu.set_data(0x7800+i,v&65535)
        patch(0x7280,[0xbc00,0xae07,0x32,0xbf00,0xbd19,0x8b89,0x7a89,0x6c50,0xef00])
        patch(0x7290,[0xef00])
        rng=random.Random(0x6cf8);cases=[]
        for trial in range(128):
            samples=[rng.randrange(-1500,1501) for _ in range(8192)]
            errors=[rng.randrange(-1000,1001) for _ in range(128)]
            coefficients=[rng.randrange(-20000,20001) for _ in range(288)]
            hist=[rng.randrange(-3000,3001) for _ in range(512)]
            phase=rng.randrange(8192);ep=rng.randrange(128)
            tap=rng.choice([0,48]);index=256;remaining=rng.randrange(2)
            for i,v in enumerate(samples):cpu.set_data(0xa000+rev(i,13),v&65535)
            for i,v in enumerate(errors):cpu.set_data(0xd400+rev(i,7),v&65535)
            for i,v in enumerate(coefficients):cpu.set_data(0x0c40+i,v&65535)
            patch(0x6400,[v&65535 for v in hist])
            cpu.set_data(0x8cc8,0xa000+rev(phase,13));cpu.set_data(0x8ccc,0xd400+rev(ep,7));cpu.set_data(0x8cc9,0xd928)
            for i,v in enumerate([0x0c40+tap,0x6400+index,remaining]):cpu.set_data(0xdab7+i,v)
            adaptation=State(tap,index,remaining);cf=(ctypes.c_int16*288)(*coefficients);h=(ctypes.c_int16*512)(*hist)
            words=[0]*128
            for offset,value in {9:3,0x44:2,0x45:3,0x48:0xa000+rev(phase,13),0x49:0xd928,0x4a:0xd928,0x4b:0x6200,0x4c:0xd400+rev(ep,7),0x69:0x7290,0x7f:0x6200,0x57:32767,0x2c:1,0x2d:30000,0x32:6,0x35:16384,0x36:64,0x37:256,0x3c:28000,0x3d:-6000,0x4e:0,0x55:4}.items():words[offset]=value&65535
            controller=(ctypes.c_uint16*128)(*words)
            correlation=(ctypes.c_uint16*4)()
            error_ring=(ctypes.c_int16*128)(*errors)
            mode=rng.choice([0,15]);bypass=rng.randrange(2)
            cpu.set_data(0xee9b,mode);cpu.set_data(0x8f4f,bypass<<4);cpu.set_data(0x8cd8,0)
            for i,v in enumerate(words):cpu.set_data(0x8c80+i,v)
            for i in range(4):cpu.set_data(0xd930+i,0)
            raw_values=[rng.randrange(-8000,8001) for _ in range(6)]
            raw=(ctypes.c_int16*6)(*raw_values);prediction=(ctypes.c_int16*6)()
            for i,v in enumerate(raw_values):cpu.set_data(0x6200+rev(i,7),v&65535)
            assert native.k56flex_feedback_predictor_block(controller,ctypes.byref(adaptation),cf,h,(ctypes.c_int16*8192)(*samples),(phase+4)&8191,raw,error_ring,ep,mode,bypass,(ctypes.c_int16*512)(*table),correlation,prediction)==0
            run(0x7280)
            got=[signed(cpu.data(0x0c40+i)) for i in range(288)]
            assert got==list(cf),(trial,'coefficients',[(i,got[i],cf[i]) for i in range(288) if got[i]!=cf[i]][:4])
            assert [signed(cpu.program(0x6400+i)) for i in range(512)]==list(h),(trial,'history')
            assert [cpu.data(0x6200+rev(i,7)) for i in range(6)]==[v&65535 for v in raw],(trial,'raw')
            assert [cpu.data(0xd400+rev(i,7)) for i in range(128)]==[v&65535 for v in error_ring],(trial,'errors',[(i,signed(cpu.data(0xd400+rev(i,7))),error_ring[i]) for i in range(128) if signed(cpu.data(0xd400+rev(i,7)))!=error_ring[i]][:12])
            assert [cpu.data(0x8ccf+i) for i in range(6)]==[v&65535 for v in prediction],(trial,'prediction')
            for offset in [0x2e,0x2f,0x30,0x31,0x33,0x34,0x35,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x4e,0x4f,0x50,0x51,0x52,0x53,0x54,0x55,0x56]:
                assert cpu.data(0x8c80+offset)==controller[offset],(trial,hex(offset),cpu.data(0x8c80+offset),controller[offset])
            assert [cpu.data(0xd930+i) for i in range(4)]==list(correlation),(trial,'correlation')
            assert [cpu.data(0xdab7+i) for i in range(3)]==[0x0c40+adaptation.tap,0x6400+adaptation.history_index,adaptation.remaining],(trial,'adaptation state')
            pointers={0x8cc8:0xa000+rev(phase+4,13),0x8cc9:0xd928+rev(6,3),0x8cca:0xd928+rev(6,3),0x8ccb:0x6200+rev(6,7),0x8ccc:0xd400+rev(ep+6,7)}
            for address,expected in pointers.items():assert cpu.data(address)==expected,(trial,hex(address),hex(cpu.data(address)),hex(expected))
            assert cpu.data(0x8cc6)==2 and cpu.data(0x8cab)==3,(trial,'sample counters')
            cases.append(dict(mode=mode,bypass=bypass,phase=phase,error_phase=ep,residual=list(raw),prediction=list(prediction)))
    report=dict(adapted_descriptor=adapted_descriptor,cases=cases,comparisons=len(cases),limitations=__doc__,data_sha256=hashlib.sha256((root/'flex.data').read_bytes()).hexdigest(),program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),production_source_sha256=hashlib.sha256((repo/'k56flex.c').read_bytes()).hexdigest(),core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'joined predictor processing blocks matched')

if __name__=='__main__':main()
