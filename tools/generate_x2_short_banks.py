"""Generate x2 short-record banks and frame vectors from original I-modem execution.
Usage (with courier-emu's Python):
  generate_x2_short_banks.py COURIER_ROOT CAPTURE_DIRECTORY OUTPUT_DIRECTORY
The capture supplies Ie030002 dsp-program.bin and dsp-data.bin. Outputs contain
constructed configurations and results, never firmware program words.
"""
from pathlib import Path
import hashlib
import json
import random
import struct
import sys

sys.path.insert(0,sys.argv[1])
from courier_emu.dsp import NativeC5x
capture=Path(sys.argv[2]);output=Path(sys.argv[3]);output.mkdir(parents=True,exist_ok=True)
program=(capture/'dsp-program.bin').read_bytes()
data=struct.unpack('<65536H',(capture/'dsp-data.bin').read_bytes())
rng=random.Random(0xcff4)
profiles=[];vectors=[]
with NativeC5x.from_program(0,program) as core:
    for address in range(32,65536):core.set_data(address,data[address])
    def invoke(entry):
        driver=[0xbc07,0x8b89,0xbe47,0xbe42,0xbe4a,0xbf01,0x7a80,entry,0x8b00]
        core.load_program(struct.pack('<9H',*driver),0x7000);core.set_pc(0x7000)
        for _ in range(100000):
            if core.state()['pc']==0x7008:return
            core.step(1)
        raise AssertionError(core.state())
    for index in range(1,16):
        for address,value in {0x340:(index<<2)|(13<<6),0xd9f1:0,0xd9f2:0x500,
                              0xf6d9:0x7fff,0x39f:0x1462,0xffd9:data[0xffd9]&~4}.items():
            core.set_data(address,value)
        invoke(0xcff4)
        assert core.data(0x3a2)==index-1
        sizes=[core.data(0x4bb-i) for i in range(6)]
        profiles.append((core.data(0x3c0),sizes,
            [[core.data(0xd9f4+128*i+j) for j in range(n)] for i,n in enumerate(sizes)],
            [core.data(0xde4f+j) for j in range(128)]))
        for md in range(7):
            bits=rng.getrandbits(core.data(0x3c0)+md)
            for address,value in {0x3ed:md,0x3af:0,0x3da:0,0x280:0,0x3fb:1,
                                  **{0x248+i:bits>>(16*i)&65535 for i in range(8)}}.items():
                core.set_data(address,value)
            invoke(0xcc95);invoke(0xce85)
            vectors.append((index,md,bits,[core.data(0x4c1-i)&255 for i in range(6)]))
def array(values):return '{'+','.join(map(str,values))+'}'
lines=['/* Original Ie030002 CFF4/D24A/D0AF, executed with mode=0, W3=0,',
       ' * W4=0500, mask=7FFF and the captured PCMU low-level call state.',
       ' * Constructed banks only; no firmware program words. */',
       'static const x2_pcm_config_t x2_short_profiles[15] = {']
for b,sizes,banks,levels in profiles:
    lines.append('{'+f'{b},5,0,'+array(sizes)+',{'+','.join(array(bank) for bank in banks)+'},'+array(levels)+'},')
lines+=['};'];(output/'x2_short_banks.h').write_text('\n'.join(lines)+'\n')
lines=['/* Original Ie030002 CC95/CE85, zero parity and monitor 0280.',
       ' * Constructed frame inputs; no firmware instructions. */',
       'struct x2_short_vector { unsigned index,md; uint64_t bits; uint8_t octets[6]; };',
       'static const struct x2_short_vector x2_short_vectors[] = {']
for index,md,bits,octets in vectors:
    lines.append('{'+f'{index},{md},UINT64_C({bits}),'+array(octets)+'},')
lines+=['};'];(output/'x2_short_vectors.h').write_text('\n'.join(lines)+'\n')
print(json.dumps({'profiles':len(profiles),'frames':len(vectors),'seed':hex(0xcff4),
                  'program_sha256':hashlib.sha256(program).hexdigest()}))
