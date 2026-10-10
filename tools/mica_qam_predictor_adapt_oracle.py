#!/usr/bin/env python3
"""Verify original DAB7 6CF8 adaptation setup, sweeps and updated forward output.

Workspace pointers and the stack profile selector are isolated fixture inputs.
Bounded supplied source/error rings, no convergence or live CONNECT claim.
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
        subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'mica_qam.c'),'-o',str(library)],check=True)
        native=ctypes.CDLL(str(library));pointer=ctypes.POINTER(ctypes.c_int16)
        native.mica_qam_predictor_adapt.argtypes=[ctypes.POINTER(State),pointer,pointer,pointer,ctypes.c_uint,pointer,ctypes.c_uint]
        native.mica_qam_predictor_adapt.restype=ctypes.c_int
        native.mica_qam_predictor_fir.argtypes=[pointer,ctypes.c_uint,pointer,pointer]
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
            state=State(tap,index,remaining);cf=(ctypes.c_int16*288)(*coefficients);h=(ctypes.c_int16*512)(*hist)
            for tick in range(4):
                cpu.set_data(0x8cc9,0xd928)
                assert native.mica_qam_predictor_adapt(ctypes.byref(state),cf,h,(ctypes.c_int16*8192)(*samples),phase,(ctypes.c_int16*128)(*errors),ep)==0
                run(0x7250)
                got=[signed(cpu.data(0x0c40+i)) for i in range(288)]
                assert got==list(cf),(trial,'coefficients',[(i,got[i],cf[i]) for i in range(288) if got[i]!=cf[i]][:8])
                assert [cpu.data(0xdab7+i) for i in range(3)]==[0x0c40+state.tap,0x6400+state.history_index,state.remaining],(trial,'state',[cpu.data(0xdab7+i) for i in range(3)],list((state.tap,state.history_index,state.remaining)))
                assert [signed(cpu.program(0x6400+i)) for i in range(512)]==list(h),(trial,'history')
                out=(ctypes.c_int16*6)();native.mica_qam_predictor_fir((ctypes.c_int16*8192)(*samples),phase,cf,out)
                assert [cpu.data(0xd928+rev(i,3)) for i in range(6)]==[v&65535 for v in out],(trial,tick,'forward')
                cases.append(dict(trial=trial,tick=tick,tap=tap,remaining=remaining,phase=phase,error_phase=ep,outputs=list(out)))
    report=dict(adapted_descriptor=adapted_descriptor,cases=cases,comparisons=len(cases),sequence_count=128,ticks_per_sequence=4,limitations=__doc__,program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),production_source_sha256=hashlib.sha256((repo/'mica_qam.c').read_bytes()).hexdigest(),core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'predictor adaptation and forward cases matched')

if __name__=='__main__':main()
