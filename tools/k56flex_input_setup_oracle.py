#!/usr/bin/env python3
"""Execute original 6BF4 input-FIR setup and inspect its expanded descriptor.

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
                block_words={hex(a):cpu.data(a) for a in [0x8cc4,0x8cc5,0x8cc6,0x8cc7,0x8cc8,0x8cc9,0x8cca,0x8ccc,0x8ce9,0x8cd7,0x8cb8,0x8cb9,0x8cba,0x8cbb,0x8cbc,0x8cbd]},
                cleared_coefficient_words=[cpu.data(0x0c40+i) for i in range(288)],
                program_sha256=hashlib.sha256((root/'flex.prog').read_bytes()).hexdigest(),
                core_manifest=json.loads((args.mica/'.build/k56flex-core/manifest.json').read_text()),limitations=__doc__)
    assert descriptor[3]==0x8cc8 and descriptor[24]==0x8cc9
    assert descriptor[11]==2 and descriptor[13]==48
    assert cpu.data(0x8cd7)==0x7fff and cpu.data(0x8cbc)==0x7fff and cpu.data(0x8cbd)==0
    assert all(v==0 for v in report['cleared_coefficient_words'])
    # Isolate the original two-word subtraction block (no instruction edits).
    # Preserve raw input debug stores; supplied pointer/register setup only.
    repo=Path(__file__).resolve().parents[1]
    temp=tempfile.TemporaryDirectory(prefix='flex-residual-')
    library=Path(temp.name)/'flex.so'
    subprocess.run(['cc','-shared','-fPIC','-I',str(repo),str(repo/'k56flex.c'),'-o',str(library)],check=True)
    native=ctypes.CDLL(str(library))
    native.k56flex_feedback_residual.argtypes=[ctypes.POINTER(ctypes.c_int16),ctypes.POINTER(ctypes.c_int16),ctypes.c_int]
    native.k56flex_feedback_residual.restype=None
    subtraction=[]
    rng=random.Random(0x4aaa)
    pairs=[([-20,50],[10,30]),([100,-100],[-10,10]),([0,0],[0,0]),
           ([-32768,32767],[32767,-32768])]
    pairs += [([rng.randint(-32768,32767) for _ in range(2)],
               [rng.randint(-32768,32767) for _ in range(2)]) for _ in range(512)]
    for bypass in [0,1]:
        for raw,predicted in pairs:
            cpu.set_data(0x8f4f,bypass<<4)
            for addr,val in zip([0x6200,0x6240],raw):cpu.set_data(addr,val&65535)
            for addr,val in zip([0x6600,0x6601],predicted):cpu.set_data(addr,val&65535)
            patch(0x7240,[0xbc00,0xae07,0x0002,0xbe47,0xbe42,0xbf0c,0x6200,0xbf0a,0x6400,
                          0xbf0f,0x6600,0xb940,0x8818,0x8b8c,0x7a8c,0x4aaa])
            cpu.set_pc(0x7240)
            for _ in range(100):
                if cpu.state()['pc']==0x4abe:break
                cpu.step(1)
            else:raise RuntimeError('subtraction fixture did not reach block end')
            got=[cpu.data(a) for a in [0x6200,0x6240]]
            expected=[(r-(0 if bypass else p))&65535 for r,p in zip(raw,predicted)]
            actual=(ctypes.c_int16*2)(*raw)
            native.k56flex_feedback_residual(actual,(ctypes.c_int16*2)(*predicted),bypass)
            assert [v&65535 for v in actual]==got
            assert got==expected,(bypass,got,expected)
            assert [cpu.data(a) for a in [0x6400,0x6401]]==[v&65535 for v in raw]
            subtraction.append(dict(bypass=bypass,raw=raw,predicted=predicted,residual=got))
    native.k56flex_feedback_rotate.argtypes=[ctypes.c_int16]*5+[ctypes.POINTER(ctypes.c_int16)]*2
    native.k56flex_feedback_rotate.restype=None
    native.k56flex_feedback_predictor.argtypes=[ctypes.POINTER(ctypes.c_int16)]*3+[ctypes.c_int]+[ctypes.POINTER(ctypes.c_int16)]*2
    native.k56flex_feedback_predictor.restype=None
    joined=[]
    for bypass in [0,1]:
        for trial in range(256):
            raw=[rng.randint(-16000,16000) for _ in range(2)]
            source_pair=[rng.randint(-16000,16000) for _ in range(2)]
            phasor=[rng.randint(-16000,16000) for _ in range(2)]
            for addr,val in {0x8f4f:bypass<<4,0x8cca:0x6400,0x8ccd:0x6700,0x8cd5:4,0x8cd8:0,
                             0x6200:raw[0],0x6240:raw[1],0x6400:source_pair[0],0x6404:source_pair[1],
                             0x6800:phasor[0],0x6801:phasor[1]}.items():cpu.set_data(addr,val&65535)
            patch(0x7240,[0xbc00,0xae07,2,0xbe47,0xbe42,0xbf01,0xbd19,
                          0xbf0c,0x6200,0xbf0d,0x6800,0xbf0e,0x6900,0xbf0f,0x6600,0x8b8d,0x7a8d,0x4a91])
            cpu.set_pc(0x7240)
            for _ in range(100):
                if cpu.state()['pc']==0x4af1:break
                cpu.step(1)
            else:raise RuntimeError('joined fixture did not reach subtraction end')
            a,b=ctypes.c_int16(),ctypes.c_int16()
            native.k56flex_feedback_rotate(*source_pair,*phasor,0,ctypes.byref(a),ctypes.byref(b))
            predicted=[a.value,b.value]
            got_prediction=[cpu.data(a) for a in [0x6600,0x6601]]
            assert got_prediction==[v&65535 for v in predicted],(trial,source_pair,phasor,got_prediction,predicted)
            actual=(ctypes.c_int16*2)(*raw)
            native.k56flex_feedback_residual(actual,(ctypes.c_int16*2)(*predicted),bypass)
            got=[cpu.data(a) for a in [0x6200,0x6240]]
            assert [v&65535 for v in actual]==got,(trial,bypass,got,list(actual))
            assert [cpu.data(a) for a in [0x6700,0x6701]]==[v&65535 for v in raw]
            lane=(ctypes.c_int16*2)(*raw)
            predout=(ctypes.c_int16*2)();errout=(ctypes.c_int16*2)()
            native.k56flex_feedback_predictor(lane,(ctypes.c_int16*2)(*source_pair),
                (ctypes.c_int16*2)(*phasor),bypass,predout,errout)
            assert [v&65535 for v in lane]==got
            got_error=[cpu.data(a) for a in [0x6900,0x6940]]
            assert [v&65535 for v in errout]==got_error,(trial,raw,phasor,got_error,list(errout))
            joined.append(dict(bypass=bypass,raw=raw,source_pair=source_pair,phasor=phasor,predicted=predicted,residual=got,error=got_error))
    report['joined_predictor_subtraction_cases']=joined
    report['subtraction_cases']=subtraction
    report['production_source_sha256']=hashlib.sha256((repo/'k56flex.c').read_bytes()).hexdigest()
    report['limitations'] += ' Original 4AAA..4ABD subtraction block uses supplied pointers only; routing from DAB7 output to this block is not executed.'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print('DAB7 descriptor:',descriptor)
    print('input setup: three 48-tap rows, zeroed coefficient workspace')


if __name__=='__main__':
    main()
