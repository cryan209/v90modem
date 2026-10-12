import sys, math
sys.path.insert(0,'tools')
from pathlib import Path
from mica_at import Module, converse
FREQ=float(sys.argv[1]); AMP=float(sys.argv[2]); ITER=int(sys.argv[3])
def lin2ul(s):
    BIAS=0x84; CLIP=32635
    sign=0x80 if s<0 else 0; s=min(abs(int(s)),CLIP)+BIAS
    e=7
    for ex in range(7,-1,-1):
        if s & (0x80<<ex) : e=ex; break
    else: e=0
    m=(s>>(e+3))&0xF
    return ~(sign|(e<<4)|m)&0xff
with Module(Path('mica-portware-map.bin').read_bytes(), courier='../courier-emu') as m:
    m.boot(); core=m.core
    converse(m,0,['ATS29=2'],per_line=3_000_000)
    m.host.configure_call(0, originate=False); m.run(300_000)
    base=len(core.g711_tx()); m.host.start_answer(0)
    n=0; last=None; pos=base
    for it in range(ITER):
        need=max(0,300-core.g711_rx_pending())
        blk=bytes(lin2ul(AMP*math.sin(2*math.pi*FREQ*(n+i)/8000)) for i in range(need)) if AMP>0 else b''
        n+=need
        if blk: core.queue_g711_rx(blk)
        m.run(100_000)
        h=m.host.channel_state(0)['handler']
        if it%300==0 or h!=last:
            print(f'it{it} t={n/8000:.1f}s tx={len(core.g711_tx())} handler={h:#x} rx_pending={core.g711_rx_pending()} underruns={core.g711_rx_underruns()}',flush=True)
        last=h
