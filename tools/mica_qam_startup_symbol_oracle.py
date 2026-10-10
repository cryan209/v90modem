#!/usr/bin/env python3
"""Verify original 97A2/97A8 -> 97C3 startup symbol mapping at SPM=0.

Supplied nibble, prior quadrant and amplitude; no transmit scheduler or CONNECT claim.
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
    native.mica_qam_startup_symbol.argtypes=[ctypes.POINTER(ctypes.c_uint16),ctypes.c_int]
    import struct
    data=(root/'flex.data').read_bytes()
    for i,v in enumerate(struct.unpack_from('<8H',data,0x72a0*2)):cpu.set_data(0x72a0+i,v)
    rng=random.Random(0x97c3);cases=[]
    for differential in [0,1]:
        entry=0x97a8 if differential else 0x97a2
        patch(0x7240,[0xbc00,0xae07,2,0xbf00,0xbd18,0x8b89,0x7a89,entry,0x7a89,0x97c3,0xef00])
        vectors=[(n,q,a) for n in range(16) for q in range(4) for a in [0,1,4096,9159,16384,32767,32768,65535]]
        vectors += [(rng.randrange(65536),rng.randrange(65536),rng.randrange(65536)) for _ in range(512)]
        for word,phase,amplitude in vectors:
            state=(ctypes.c_uint16*128)();state[7]=word;state[0x0c]=phase;state[0x3e]=amplitude
            for offset in [7,0x0c,0x3e]:cpu.set_data(0x8c00+offset,state[offset])
            native.mica_qam_startup_symbol(state,differential)
            run(0x7240)
            for offset in [7,0x0c,0x0f,0x10,0x3e]:
                assert cpu.data(0x8c00+offset)==state[offset],(differential,word,phase,amplitude,hex(offset),cpu.data(0x8c00+offset),state[offset])
            cases.append(dict(differential=differential,word=word,phase=phase,amplitude=amplitude,outputs=[state[0x0f],state[0x10]],quadrant=state[0x0c]))
    report=dict(production_source_sha256=hashlib.sha256((repo/'mica_qam.c').read_bytes()).hexdigest(),cases=cases,comparisons=len(cases),limitations=__doc__,
        program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
        core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'startup symbol mappings matched')


if __name__=='__main__':main()
