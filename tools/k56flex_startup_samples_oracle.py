#!/usr/bin/env python3
"""Verify whole original 8270 using supplied coefficient banks at SPM=0.

Supplied complex pair and gain; no pulse shaper, scheduler or CONNECT claim.
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
    native.k56flex_feedback_startup_samples.argtypes=[ctypes.POINTER(ctypes.c_uint16),ctypes.c_uint,pointer,ctypes.POINTER(ctypes.c_uint),pointer,ctypes.c_uint,pointer,ctypes.POINTER(ctypes.c_uint)]
    native.k56flex_feedback_startup_samples.restype=ctypes.c_int
    rng=random.Random(0x8270);cases=[];samples=0
    def rev6(v):return int(f'{v&63:06b}'[::-1],2)
    def rev7(v):return int(f'{v&127:07b}'[::-1],2)
    patch(0x7240,[0xbc00,0xae07,2,0xbf00,0xbd18,0x7a8a,0x8270,0xef00])
    for tick in range(512):
        state=(ctypes.c_uint16*128)();state[0x27]=10;state[0x28]=3;state[0x29]=43;state[0x2a]=0x7900;state[0x4c]=tick%3;state[0x4d]=tick%10
        symbols=1+tick%8;h=ctypes.c_uint(tick%64);o=ctypes.c_uint(tick%128)
        history=(ctypes.c_int16*64)(*(rng.randrange(-32768,32768) for _ in range(64)))
        banks=(ctypes.c_int16*440)(*(rng.randrange(-32768,32768) for _ in range(440)))
        output=(ctypes.c_int16*128)(*(rng.randrange(-32768,32768) for _ in range(128)))
        state[0x19]=0x7500+rev6(h.value)
        for offset in range(128):cpu.set_data(0x8c00+offset,state[offset])
        for j in range(64):cpu.set_data(0x7500+rev6(j),history[j]&65535)
        for j in range(128):cpu.set_data(0x7700+rev7(j),output[j]&65535)
        for j in range(10):cpu.set_data(0x7900+j,0x7a00+j*44)
        patch(0x7a00,[x&65535 for x in banks])
        cpu.set_data(0xec6c,0x7700+rev7(o.value));cpu.set_data(0x8eea,0x7770);cpu.set_data(0x7770,123)
        count=native.k56flex_feedback_startup_samples(state,symbols,history,ctypes.byref(h),banks,440,output,ctypes.byref(o))
        assert count>0;samples+=count
        run(0x7240,acc=symbols)
        for offset in [0x4c,0x4d,0x55,0x57]:assert cpu.data(0x8c00+offset)==state[offset],(tick,hex(offset))
        assert cpu.data(0x8c19)==0x7500+rev6(h.value),(tick,'history cursor')
        assert cpu.data(0xec6c)==0x7700+rev7(o.value),(tick,'output cursor')
        assert cpu.data(0x7770)==123+count,(tick,'counter')
        assert [cpu.data(0x7700+rev7(j)) for j in range(128)]==[x&65535 for x in output],(tick,'output')
        assert [cpu.data(0x7500+rev6(j)) for j in range(64)]==[x&65535 for x in history],(tick,'history')
        cases.append(dict(tick=tick,symbols=symbols,count=count,phase=state[0x4d],history_cursor=h.value,output_cursor=o.value))
    report=dict(production_source_sha256=hashlib.sha256((repo/'k56flex.c').read_bytes()).hexdigest(),cases=cases,comparisons=len(cases),limitations=__doc__,
        program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
        core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'startup sample blocks matched')


if __name__=='__main__':main()
