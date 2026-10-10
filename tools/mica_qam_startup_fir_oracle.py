#!/usr/bin/env python3
"""Verify original 829E..82A3 FIR instruction sequence at SPM=0/OVM=0.

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
    subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'mica_qam.c'),'-o',str(library)],check=True)
    native=ctypes.CDLL(str(library));pointer=ctypes.POINTER(ctypes.c_int16)
    native.mica_qam_startup_fir.argtypes=[pointer,ctypes.c_uint,pointer,ctypes.c_uint,pointer]
    native.mica_qam_startup_fir.restype=ctypes.c_int
    rng=random.Random(0x82a0);cases=[]
    def rev6(v):return int(f'{v&63:06b}'[::-1],2)
    for tick in range(2048):
        taps=1+(tick%64);old=(tick//32)%64
        history=(ctypes.c_int16*64)(*(rng.randrange(-32768,32768) for _ in range(64)))
        coefficients=(ctypes.c_int16*taps)(*(rng.randrange(-32768,32768) for _ in range(taps)))
        if tick<1024:
            for j in range(64):history[j]=0
            history[(old-(tick//64))&63]=32767 if tick&1 else -32768
        for j in range(64):cpu.set_data(0x7500+rev6(j),history[j]&65535)
        patch(0x7800,[x&65535 for x in coefficients])
        patch(0x7240,[0xbc00,0xae07,2,0xbf00,0xb020,0xbf0a,0x7500+rev6(old),0xbf0b,0x7600,0xbf80,0x7800,0x881f,0x8b8a,0xbe59,0xbb00+(taps-1),0xaac0,0x708b,0xb040,0x9cfa,0xef00])
        sample=ctypes.c_int16()
        assert native.mica_qam_startup_fir(history,old,coefficients,taps,ctypes.byref(sample))==0
        run(0x7240)
        assert cpu.data(0x7600)==sample.value&65535,(tick,taps,old,cpu.data(0x7600),sample.value)
        assert [cpu.data(0x7500+rev6(j)) for j in range(64)]==[x&65535 for x in history]
        cases.append(dict(tick=tick,taps=taps,cursor=old,sample=sample.value))
    report=dict(production_source_sha256=hashlib.sha256((repo/'mica_qam.c').read_bytes()).hexdigest(),cases=cases,comparisons=len(cases),limitations=__doc__,
        program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
        core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'startup FIR samples matched')


if __name__=='__main__':main()
