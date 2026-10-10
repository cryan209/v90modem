#!/usr/bin/env python3
"""Verify original 43D1 rotation joined to 9367 output writer.

Supplied symbols, coefficients and gain; no FIR sample generator or CONNECT claim.
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
    native.k56flex_feedback_startup_rotate.argtypes=[ctypes.POINTER(ctypes.c_uint16),pointer,pointer]
    native.k56flex_feedback_startup_rotate.restype=ctypes.c_int
    native.k56flex_feedback_startup_output.argtypes=[ctypes.POINTER(ctypes.c_uint16),pointer,pointer,ctypes.POINTER(ctypes.c_uint)]
    native.k56flex_feedback_startup_output.restype=ctypes.c_int
    def rev6(v):return int(f'{v&63:06b}'[::-1],2)
    rng=random.Random(0x43d1);cases=[]
    patch(0x7240,[0xbc00,0xae07,2,0xbf0a,0x7300,0x8b8b,0x7a8b,0x43d1,0x8b8a,0x7a8a,0x9367,0xef00])
    for tick in range(4096):
        state=(ctypes.c_uint16*128)();state[0x2d]=0x7400;state[0x2e]=0x7420;state[0x13]=0x7400+2*(tick%16)
        state[0x11]=rng.randrange(65536)
        old=tick%64;cursor=ctypes.c_uint(old);ring=(ctypes.c_int16*64)()
        for j in range(64):
            ring[j]=rng.randrange(-32768,32768);cpu.set_data(0x7500+rev6(j),ring[j]&65535)
        cpu.set_data(0x8c11,state[0x11]);cpu.set_data(0x8c1a,0x7500+rev6(old))
        pair=(ctypes.c_int16*2)(*(rng.randrange(-32768,32768) for _ in range(2)))
        phasor=(ctypes.c_int16*2)(*(rng.randrange(-32768,32768) for _ in range(2)))
        for offset in [0x13,0x2d,0x2e]:cpu.set_data(0x8c00+offset,state[offset])
        patch(state[0x13],[x&65535 for x in phasor])
        for j,x in enumerate(pair):cpu.set_data(0x7300+j,x&65535)
        assert native.k56flex_feedback_startup_rotate(state,pair,phasor)==0
        assert native.k56flex_feedback_startup_output(state,pair,ring,ctypes.byref(cursor))==0
        run(0x7240)
        assert [cpu.data(0x7300+j) for j in range(2)]==[x&65535 for x in pair],(tick,'pair')
        assert cpu.data(0x8c13)==state[0x13],(tick,'phase')
        assert [cpu.data(0x7500+rev6(j)) for j in range(64)]==[x&65535 for x in ring],(tick,'ring')
        assert cpu.data(0x8c1a)==0x7500+rev6(cursor.value),(tick,'cursor')
        cases.append(dict(tick=tick,phasor=list(phasor),output=list(pair),phase=state[0x13]))
    report=dict(production_source_sha256=hashlib.sha256((repo/'k56flex.c').read_bytes()).hexdigest(),cases=cases,comparisons=len(cases),limitations=__doc__,
        program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
        core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'startup carrier rotations matched')


if __name__=='__main__':main()
