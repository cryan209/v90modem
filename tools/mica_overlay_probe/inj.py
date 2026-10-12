import sys, ctypes, time
sys.path.insert(0,'tools')
from pathlib import Path
from mica_at import Module, converse
S29=int(sys.argv[1]); TOTAL=int(sys.argv[2]); LAW=int(sys.argv[3]) if len(sys.argv)>3 else 0
lib=ctypes.CDLL(''+str(__import__('pathlib').Path(__file__).resolve().parent)+'/libv32caller.dylib')
STUBS={0x2bba:'7C/7D',0x2bc9:'90/91',0x2bd8:'8A/8B',0x2be7:'7E/7F',0x2bf6:'8C/8D',0x2c11:'80/81',0x2c18:'8E/8F'}
with Module(Path('mica-portware-map.bin').read_bytes(), courier='../courier-emu') as m:
    m.boot(); core=m.core; core.set_coverage(True)
    converse(m,0,[f'ATS29={S29}'],per_line=3_000_000)
    assert m.host.configure_call(0, originate=False); m.run(300_000)
    base=len(core.g711_tx())
    assert m.host.start_answer(0)
    assert lib.caller_init(0x1000|0x0400|0x0040|0x0200|0x0020)==0
    pos=base; done=0; seen={}; last=-1; t0=time.time(); cum=0
    CH=100_000
    while done<TOTAL:
        m.run(CH); done+=CH
        tx=core.g711_tx(pos); n=len(tx); pos+=n
        # feed in <=1024-sample blocks
        out=bytearray()
        for i in range(0,n,1024):
            blk=tx[i:i+1024]; ib=(ctypes.c_uint8*len(blk)).from_buffer_copy(blk)
            ob=(ctypes.c_uint8*len(blk))(); lib.caller_step(ib,ob,len(blk),LAW); out+=bytes(ob)
        if out: core.queue_g711_rx(bytes(out))
        for pc,nm in STUBS.items():
            f=core.first_exec(pc)
            if f and pc not in seen: seen[pc]=f; print(f'[{done/1e6:.0f}M] loader {nm} first_exec={f}',flush=True)
        h=m.host.channel_state(0)['handler']
        if h!=last: print(f'[{done/1e6:.0f}M] handler {h:#x} caller_rate={lib.caller_rate()} done={lib.caller_done()}',flush=True); last=h
    print('final caller_rate',lib.caller_rate(),'done',lib.caller_done(),'tx octets',pos-base,'secs',time.time()-t0)
import numpy as np
