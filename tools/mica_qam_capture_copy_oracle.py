#!/usr/bin/env python3
"""Verify original 1A8C ring-to-capture copying and cursor direction.

Supplied source ring, destination-pointer cell and copy count; confirms a
consumer rather than a ring producer. Does not run full callback scheduling.
"""
import argparse
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
    rng=random.Random(0x1a8c);cases=[]
    for trial in range(128):
        count=[4,96][trial&1];initial=0xa000+rng.randrange(8192);cursor=initial
        expected=[];addresses=[]
        for j in range(count):
            value=rng.randrange(65536);cpu.set_data(cursor,value)
            expected.append(value);addresses.append(cursor);cursor=step(cursor)
        cpu.set_data(0x9088,0x6200);cpu.set_data(0x8ced,initial)
        # Wrapper selects AR1 before call, exactly as the real caller does.
        patch(0x7240,[0xbc00,0xbe47,0xbe42,0xbf80,count,0xbf0a,0x9088,
                      0xbf0b,0x8ced,0xbf0c,0,0xbf80,0x1000,0x8818,0xbf80,count,
                      0x8b89,0x7a89,0x1a8c,0xef00])
        run(0x7240)
        got=[cpu.data(0x6200+j) for j in range(count)]
        assert got==expected,(trial,initial,count,got[:4],expected[:4])
        assert cpu.data(0x8ced)==cursor
        assert [cpu.data(a) for a in addresses]==expected
        assert cpu.state()['acc']==0x6200+count
        cases.append(dict(count=count,initial_source=initial,final_source=cursor,
                          destination=0x6200,returned_end=cpu.state()['acc']))
    report=dict(cases=cases,comparisons=len(cases),limitations=__doc__,
        program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
        core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(len(cases),'ring-to-capture copies verified; source ring unchanged')


if __name__=='__main__':main()
