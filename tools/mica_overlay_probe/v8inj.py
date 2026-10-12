import sys, ctypes
sys.path.insert(0,'tools')
from pathlib import Path
from mica_at import Module, converse
SP=''+str(__import__('pathlib').Path(__file__).resolve().parent)+''
S29=int(sys.argv[1]); MODS=int(sys.argv[2],0); TOTAL=int(sys.argv[3])
v8=ctypes.CDLL(SP+'/libv8caller.dylib'); v32=ctypes.CDLL(SP+'/libv32caller.dylib')
STUBS={0x2bba:'7C/7D',0x2bc9:'90/91',0x2bd8:'8A/8B',0x2be7:'7E/7F',0x2bf6:'8C/8D',0x2c11:'80/81',0x2c18:'8E/8F'}
with Module(Path('mica-portware-map.bin').read_bytes(), courier='../courier-emu') as m:
    m.boot(); core=m.core; core.set_coverage(True)
    converse(m,0,[f'ATS29={S29}'],per_line=3_000_000)
    assert m.host.configure_call(0, originate=False); m.run(300_000)
    base=len(core.g711_tx()); assert m.host.start_answer(0)
    assert v8.v8c_init(MODS)==0
    pos=base; done=0; seen={}; last=-1; phase='v8'; v8st=None
    while done<TOTAL:
        m.run(100_000); done+=100_000
        tx=core.g711_tx(pos); pos+=len(tx); out=bytearray()
        for i in range(0,len(tx),1024):
            blk=tx[i:i+1024]; ib=(ctypes.c_uint8*len(blk)).from_buffer_copy(blk); o=(ctypes.c_uint8*len(blk))()
            if phase=='v8': v8.v8c_step(ib,o,len(blk))
            else: v32.caller_step(ib,o,len(blk),0)
            out+=bytes(o)
        core.queue_g711_rx(bytes(out))
        st=v8.v8c_status()
        if phase=='v8' and st!=v8st: v8st=st; print(f'[{done/1e6:.0f}M] V8 status {st} mods={v8.v8c_mod():#x} peer={v8.v8c_peer():#x}',flush=True)
        if phase=='v8' and st in (2,):
            phase='v32'; print(f'[{done/1e6:.0f}M] V.8 done -> start V.32bis caller',flush=True); v32.caller_init(0x1260)
        for pc,nm in STUBS.items():
            f=core.first_exec(pc)
            if f and pc not in seen: seen[pc]=f; print(f'[{done/1e6:.0f}M] loader {nm} first_exec={f}',flush=True)
        h=m.host.channel_state(0)['handler']
        if h!=last: last=h; print(f'[{done/1e6:.0f}M] handler {h:#x} v32rate={v32.caller_rate() if phase=='v32' else '-'} done={v32.caller_done() if phase=='v32' else '-'}',flush=True)
    print('end phase',phase, (v32.caller_rate(), v32.caller_done()) if phase=='v32' else '')
