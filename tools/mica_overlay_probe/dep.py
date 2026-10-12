import sys, math, hashlib
sys.path.insert(0,'tools')
from pathlib import Path
from mica_at import Module, converse
def lin2ul(s):
    sign=0x80 if s<0 else 0; s=min(abs(int(s)),32635)+0x84
    e=0
    for ex in range(7,-1,-1):
        if s & (0x80<<ex): e=ex; break
    return ~(sign|(e<<4)|((s>>(e+3))&0xF))&0xff
F=float(sys.argv[1]); S29=int(sys.argv[2])
with Module(Path('mica-portware-map.bin').read_bytes(), courier='../courier-emu') as m:
    m.boot(); core=m.core
    converse(m,0,[f'ATS29={S29}'],per_line=3_000_000)
    m.host.configure_call(0, originate=False); m.run(300_000)
    base=len(core.g711_tx()); m.host.start_answer(0); n=0
    for it in range(1500):
        if F>0:
            need=max(0,300-core.g711_rx_pending())
            core.queue_g711_rx(bytes(lin2ul(8000*math.sin(2*math.pi*F*(n+i)/8000)) for i in range(need))); n+=need
        m.run(100_000)
    tx=core.g711_tx(base)
    print(f'F={F:6.0f} S29={S29} tx={len(tx)} sha={hashlib.sha1(tx).hexdigest()[:12]} handler={m.host.channel_state(0)["handler"]:#x}')
