#!/usr/bin/env python3
"""Verify original 828B..8299 phase and coefficient-bank selection.

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
    native.k56flex_feedback_startup_phase.argtypes=[ctypes.POINTER(ctypes.c_uint16),ctypes.POINTER(ctypes.c_uint),ctypes.POINTER(ctypes.c_uint16)]
    native.k56flex_feedback_startup_phase.restype=ctypes.c_int
    rng=random.Random(0x828b);cases=[]
    def rev6(v):return int(f'{v&63:06b}'[::-1],2)
    patch(0x7240,[0xbc00,0xae07,2,0xbf00,0xbc00,0x1066,0xbe1e,0x7a8a,0x828b,0xef00])
    patch(0x829b,[0xef00])
    for tick in range(4096):
        state=(ctypes.c_uint16*128)()
        limit=10 if tick<2048 else rng.randrange(1,32768)
        increment=3 if tick<2048 else rng.randrange(limit+1)
        phase=(tick%17)-7 if tick<2048 else rng.randrange(-32768,limit)
        state[0x27]=limit;state[0x28]=increment;state[0x4d]=phase&65535;state[0x2a]=0x6000
        old=tick%64;cursor=ctypes.c_uint(old);bank=ctypes.c_uint16()
        cpu.set_data(0x66,phase&65535);cpu.set_data(0x67,0x7500+rev6(old));cpu.set_data(0x68,limit);cpu.set_data(0x69,increment);cpu.set_data(0x6b,state[0x2a])
        assert native.k56flex_feedback_startup_phase(state,ctypes.byref(cursor),ctypes.byref(bank))==0
        run(0x7240)
        got=cpu.state()
        assert cpu.data(0x67)==0x7500+rev6(cursor.value),(tick,'cursor')
        assert got['accb']&65535==state[0x4d],(tick,'phase',got)
        assert got['ar4']==bank.value,(tick,'bank',got)
        cases.append(dict(tick=tick,limit=limit,increment=increment,phase=phase,next_phase=state[0x4d],bank=bank.value,cursor=cursor.value))
    report=dict(production_source_sha256=hashlib.sha256((repo/'k56flex.c').read_bytes()).hexdigest(),cases=cases,comparisons=len(cases),limitations=__doc__,
        program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
        core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'startup phase selections matched')


if __name__=='__main__':main()
