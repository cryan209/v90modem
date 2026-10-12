import sys
sys.path.insert(0,'tools')
from pathlib import Path
from mica_at import Module, converse
S29=int(sys.argv[1])
STUBS={0x2bba:'7C/7D',0x2bc9:'90/91',0x2bd8:'8A/8B',0x2be7:'7E/7F',0x2bf6:'8C/8D K56',0x2c05:'80|8E sel',0x2c11:'-> 80/81',0x2c18:'-> 8E/8F'}
with Module(Path('mica-portware-map.bin').read_bytes(), courier='../courier-emu') as m:
    m.boot()
    core=m.core
    core.set_coverage(True)
    print(converse(m,0,[f'ATS29={S29}','ATS29?'],per_line=3_000_000).decode(errors='replace').replace('\r\n',' | '))
    assert m.host.configure_call(0, originate=False)
    m.run(300_000)
    assert m.host.start_answer(0)
    m.run(int(sys.argv[2]) if len(sys.argv)>2 else 30_000_000)
    for pc,n in STUBS.items():
        print(f'S29={S29} {pc:04x} {n:12s} first_exec={core.first_exec(pc)}')
    print('handler', hex(m.host.channel_state(0)['handler']))
