#!/usr/bin/env python3
"""Verify original DAB7 setup and forward FIR addressing/scaling.

Workspace pointers and the stack profile selector are isolated fixture inputs.
This is setup evidence, not a serial-to-report receiver or scheduler run.
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
    coefficients=[0]*288
    repo=Path(__file__).resolve().parents[1]
    temp=tempfile.TemporaryDirectory(prefix='flex-predictor-fir-')
    library=Path(temp.name)/'flex.so'
    subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'mica_qam.c'),'-o',str(library)],check=True)
    native=ctypes.CDLL(str(library))
    pointer=ctypes.POINTER(ctypes.c_int16)
    native.mica_qam_predictor_fir.argtypes=[pointer,ctypes.c_uint,pointer,pointer]
    native.mica_qam_predictor_fir.restype=None
    def compare(samples,phase,coefficients,expected):
        out=(ctypes.c_int16*6)()
        native.mica_qam_predictor_fir((ctypes.c_int16*8192)(*samples),phase,
                                              (ctypes.c_int16*288)(*coefficients),out)
        assert [v&65535 for v in out]==expected
    trials=[]
    # Each impulse selects one of the two component walks at each of 48 taps.
    for address in range(0xa000,0xc000):cpu.set_data(address,0)
    for tap in range(48):
        for component in range(2):
            phase=(tap*37+component*11)%8192
            source_pos=(phase-2+component-2*tap)%8192
            sample=4096
            coefficients=[0]*288
            for row in range(3):
                coefficients[96*row+tap]=256*(row+1)
                coefficients[96*row+48+tap]=-256*(row+1)
            for index,value in enumerate(coefficients):cpu.set_data(0x0c40+index,value&65535)
            cpu.set_data(0xa000+rev(source_pos,13),sample)
            cpu.set_data(0x8cc8,0xa000+rev(phase,13))
            cpu.set_data(0x8cc9,0xd928)
            cpu.set_data(0x8faa,0)
            run(0x7250)
            got=[cpu.data(0xd928+rev(j,3)) for j in range(6)]
            expected=[]
            for row in range(3):
                u=256*(row+1);v=-u
                real=sample*u if component==0 else -sample*v
                imag=sample*v if component==0 else sample*u
                expected += [((real+131072)>>18)&65535,((imag+131072)>>18)&65535]
            samples=[0]*8192;samples[source_pos]=sample
            compare(samples,phase,coefficients,got)
            assert got==expected,(tap,component,phase,got,expected)
            assert cpu.data(0x8cc9)==0xd928+rev(6,3)
            cpu.set_data(0xa000+rev(source_pos,13),0)
            trials.append(dict(tap=tap,component=component,phase=phase,outputs=got))
    report['forward_impulses']=trials
    random_cases=[];rng=random.Random(0xdab7)
    for trial in range(128):
        samples=[rng.randint(-1024,1024) for _ in range(8192)]
        coefficients=[rng.randint(-1024,1024) for _ in range(288)]
        phase=rng.randrange(8192)
        for n,val in enumerate(samples):cpu.set_data(0xa000+rev(n,13),val&65535)
        for n,val in enumerate(coefficients):cpu.set_data(0x0c40+n,val&65535)
        cpu.set_data(0x8cc8,0xa000+rev(phase,13));cpu.set_data(0x8cc9,0xd928)
        run(0x7250)
        got=[cpu.data(0xd928+rev(j,3)) for j in range(6)]
        compare(samples,phase,coefficients,got)
        random_cases.append(dict(phase=phase,outputs=got))
    report['random_cases']=random_cases
    report['production_source_sha256']=hashlib.sha256((repo/'mica_qam.c').read_bytes()).hexdigest()
    report['limitations'] += ' Original 6910 executed with supplied nonzero coefficients, zero adaptation, source ring A000-BFFF and scratch output D928-D92F.'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print('DAB7:',len(trials),'impulse cases,',len(random_cases),'random rings; 3 complex rows verified')


if __name__=='__main__':
    main()
