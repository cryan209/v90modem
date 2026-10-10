#!/usr/bin/env python3
"""Verify original 9320 carrier presets and full 4A6C phase-cycle source writes.

Supplied symbols only; original phase coefficients, phase reset and cursor
advance are retained. Does not execute the transmit symbol mapper.
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
    subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'mica_qam.c'),'-o',str(library)],check=True)
    native=ctypes.CDLL(str(library));pointer=ctypes.POINTER(ctypes.c_int16)
    native.mica_qam_source_init.argtypes=[ctypes.POINTER(ctypes.c_uint16),ctypes.c_uint]
    native.mica_qam_source_emit.argtypes=[ctypes.POINTER(ctypes.c_uint16),pointer,ctypes.POINTER(ctypes.c_uint),pointer]
    native.mica_qam_source_emit.restype=ctypes.c_int
    def rev13(v):return int(f'{v&8191:013b}'[::-1],2)
    rng=random.Random(0x9320);cases=[];resets=0
    patch(0x7240,[0xbc00,0xae07,2,0xbf01,0xbd18,0xbf0a,0x8c0f,0x8b8c,0x7a8c,0x4a6c,0xef00])
    for profile in range(11):
        patch(0x7220,[0xbc00,0xae07,2,0xbd18,0xbf80,profile*3,0x8b89,0x7a89,0x9320,0xef00])
        run(0x7220)
        state=(ctypes.c_uint16*128)()
        assert native.mica_qam_source_init(state,profile)==0
        assert [cpu.data(0x8c00+i) for i in [0x1b,0x1c,0x1d,0x1e]]==[state[i] for i in [0x1b,0x1c,0x1d,0x1e]]
        cursor=ctypes.c_uint(8190);ring=(ctypes.c_int16*8192)()
        cpu.set_data(0x8c91,0xa000+rev13(cursor.value))
        for tick in range(128):
            pair=[rng.randint(-32768,32767) for _ in range(2)]
            for i,v in enumerate(pair):cpu.set_data(0x8c0f+i,v&65535)
            old=cursor.value
            neighbours=[(old-1)&8191,(old+2)&8191]
            before=[cpu.data(0xa000+rev13(i)) for i in neighbours]
            reset=native.mica_qam_source_emit(state,ring,ctypes.byref(cursor),(ctypes.c_int16*2)(*pair))
            assert reset in [0,1];resets+=reset
            run(0x7240)
            got=[cpu.data(0xa000+rev13(i)) for i in [old,(old+1)&8191]]
            assert got==[ring[i]&65535 for i in [old,(old+1)&8191]],(profile,tick,'source')
            assert cpu.data(0x8c91)==0xa000+rev13(cursor.value),(profile,tick,'cursor')
            assert cpu.data(0x8c1b)==state[0x1b],(profile,tick,'phase')
            assert [cpu.data(0xa000+rev13(i)) for i in neighbours]==before,(profile,tick,'neighbours')
            cases.append(dict(profile=profile,tick=tick,phase=state[0x1b],cursor=cursor.value,reset=reset,outputs=got))
    report=dict(production_source_sha256=hashlib.sha256((repo/'mica_qam.c').read_bytes()).hexdigest(),cases=cases,comparisons=len(cases),phase_resets=resets,limitations=__doc__,
        program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
        core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'source phase-cycle calls and',resets,'phase resets matched')


if __name__=='__main__':main()
