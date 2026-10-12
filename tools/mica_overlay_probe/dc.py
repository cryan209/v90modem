import sys, math, pickle
sys.path.insert(0,'tools')
from pathlib import Path
from mica_at import Module, converse
exec(open(sys.argv[0].replace('dc.py','tone.py')).read().split('with Module')[0].split('FREQ=')[0])  # imports only
def lin2ul(s):
    BIAS=0x84; CLIP=32635
    sign=0x80 if s<0 else 0; s=min(abs(int(s)),CLIP)+BIAS
    e=0
    for ex in range(7,-1,-1):
        if s & (0x80<<ex): e=ex; break
    m=(s>>(e+3))&0xF
    return ~(sign|(e<<4)|m)&0xff
def run(level, iters=400):
    with Module(Path('mica-portware-map.bin').read_bytes(), courier='../courier-emu') as m:
        m.boot(); core=m.core
        converse(m,0,['ATS29=2'],per_line=3_000_000)
        m.host.configure_call(0, originate=False); m.run(300_000); m.host.start_answer(0)
        oct_=lin2ul(level) if level is not None else 0xff
        for it in range(iters):
            need=max(0,300-core.g711_rx_pending())
            core.queue_g711_rx(bytes([oct_])*need) if level is not None else None
            m.run(100_000)
        return [core.data(a) for a in range(0x10000)]
A=run(4000); B=run(-4000); Z=run(None)
pickle.dump((A,B,Z),open(sys.argv[0].replace('dc.py','dc.pkl'),'wb'))
d=[a for a in range(0x10000) if A[a]!=B[a]]
print('differ A vs B:',len(d)); 
def s16(v): return v-65536 if v>=32768 else v
for a in d[:60]: print(f'{a:04x} A={s16(A[a]):6d} B={s16(B[a]):6d} Z={s16(Z[a]):6d}')
