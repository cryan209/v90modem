"""Answerer vs injected caller, framed like mica_peer_probe.py.
usage: call.py S29 MODS(hex V.8 bits) SECONDS [v8bis(0/1)]"""
import sys, ctypes
sys.path.insert(0,'tools')
from pathlib import Path
from mica_at import Module, converse
SP=''+str(__import__('pathlib').Path(__file__).resolve().parent)+''
S29=int(sys.argv[1]); MODS=int(sys.argv[2],0); SECS=float(sys.argv[3]); V8BIS=int(sys.argv[4]) if len(sys.argv)>4 else 0
v8=ctypes.CDLL(SP+'/libv8caller.dylib'); v32=ctypes.CDLL(SP+'/libv32caller.dylib')
STUBS={0x2bba:'7C/7D',0x2bc9:'90/91',0x2bd8:'8A/8B',0x2be7:'7E/7F',0x2bf6:'8C/8D',0x2c11:'80/81',0x2c18:'8E/8F'}
BLK=160
with Module(Path('mica-portware-map.bin').read_bytes(), courier='../courier-emu') as m:
    m.boot(); core=m.core; core.set_coverage(True)
    r=converse(m,0,[f'ATS29={S29}',f'ATS53={V8BIS}'],per_line=3_000_000); assert r.count(b'OK')>=2,r
    assert m.host.configure_call(0, originate=False); m.run(300_000)
    assert m.host.dsp_command(0,0xA0,[0])
    if MODS: assert v8.v8c_init(MODS)==0
    START=float(sys.argv[5]) if len(sys.argv)>5 else 0
    cursor=len(core.g711_tx()); mono=bytearray(); phase='v8' if MODS else 'wait'; seen={}; last=-1; v8st=None
    def feed(rx_mono):
        pk=bytearray(len(rx_mono)*2); pk[0::2]=rx_mono; pk[1::2]=rx_mono; core.queue_g711_rx(bytes(pk))
    def caller(blk):
        ib=(ctypes.c_uint8*BLK).from_buffer_copy(blk); o=(ctypes.c_uint8*BLK)()
        if phase=='wait': return bytes([0xFF])*BLK
        (v8.v8c_step if phase=='v8' else v32.caller_step)(*( (ib,o,BLK) if phase=='v8' else (ib,o,BLK,0)))
        return bytes(o)
    for _ in range(2): feed(caller(bytes([0xFF])*BLK))
    consumed=0
    while len(mono)<SECS*8000:
        m.run(100_000)
        tx=core.g711_tx(); new=tx[cursor:]; cursor=len(tx); mono+=new[0::2]
        while len(mono)-consumed>=BLK:
            feed(caller(bytes(mono[consumed:consumed+BLK]))); consumed+=BLK
        t=len(mono)/8000
        if phase=='wait' and t>=START: phase='v32'; v32.caller_init(0x1260); print(f'[{t:5.2f}s] V.32bis caller starts',flush=True)
        st=v8.v8c_status()
        if phase=='v8' and st!=v8st: v8st=st; print(f'[{t:5.2f}s] V.8 status {st} mods={v8.v8c_mod():#x} peer={v8.v8c_peer():#x}',flush=True)
        if phase=='v8' and st==2: phase='v32'; v32.caller_init(0x1260); print(f'[{t:5.2f}s] -> V.32bis caller',flush=True)
        for pc,nm in STUBS.items():
            f=core.first_exec(pc)
            if f and pc not in seen: seen[pc]=f; print(f'[{t:5.2f}s] loader {nm}',flush=True)
        h=m.host.channel_state(0)['handler']
        if h!=last: last=h; print(f'[{t:5.2f}s] handler {h:#x}'+(f' v32 rate={v32.caller_rate()} done={v32.caller_done()}' if phase=='v32' else ''),flush=True)
    print('end',phase,(v32.caller_rate(),v32.caller_done(),'remote_rates=%#x'%v32.caller_remote_rates()) if phase=='v32' else '')
