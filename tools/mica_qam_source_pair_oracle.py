#!/usr/bin/env python3
"""Verify original 4A6C writes complex source pairs into the DAB7 ring.

Supplied source pair, phase coefficients and cursor. Does not execute the
transmitter that produces source symbols or the full scheduler.
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
    native.mica_qam_source_pair.argtypes=[pointer,ctypes.POINTER(ctypes.c_uint),pointer,pointer]
    native.mica_qam_source_pair.restype=None
    def rev13(v):return int(f'{v&8191:013b}'[::-1],2)
    rng=random.Random(0x4a6c);cases=[]
    patch(0x7240,[0xbc00,0xae07,2,0xbf01,0xbd18,0xbf0a,0x8c0f,0x8b8c,0x7a8c,0x4a6c,0xef00])
    for trial in range(1024):
        initial=0xa000+rng.randrange(8192)
        pair=[rng.randint(-8192,8192) for _ in range(2)]
        phasor=[rng.randint(-16384,16384) for _ in range(2)]
        patch(0x7a00,[v&65535 for v in phasor])
        for a,v in {0x8c1b:0x7a00,0x8c1c:0x7a00,0x8c1d:0x7a01,0x8c1e:1,
                    0x8c91:initial,0x8c0f:pair[0],0x8c10:pair[1]}.items():cpu.set_data(a,v&65535)
        run(0x7240)
        second=step(initial);after=step(second)
        got=[cpu.data(initial),cpu.data(second)]
        x,y=pair;u,v=phasor
        expected=[((x*u-y*v+4096)>>13)&65535,((x*v+y*u+4096)>>13)&65535]
        ring=(ctypes.c_int16*8192)();cursor=ctypes.c_uint(rev13(initial-0xa000));old=cursor.value
        native.mica_qam_source_pair(ring,ctypes.byref(cursor),(ctypes.c_int16*2)(*pair),(ctypes.c_int16*2)(*phasor))
        assert [ring[old]&65535,ring[(old+1)&8191]&65535]==got
        assert 0xa000+rev13(cursor.value)==after
        assert got==expected,(trial,pair,phasor,got,expected)
        assert cpu.data(0x8c91)==after
        assert cpu.data(0x8c1b)==0x7a00
        cases.append(dict(initial=initial,final=after,pair=pair,phasor=phasor,outputs=got))
    report=dict(production_source_sha256=hashlib.sha256((repo/'mica_qam.c').read_bytes()).hexdigest(),cases=cases,comparisons=len(cases),limitations=__doc__,
        program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
        core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'source-pair writes and cursor updates verified')


if __name__=='__main__':main()
