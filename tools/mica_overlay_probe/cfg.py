import sys, pickle
sys.path.insert(0,'tools')
from pathlib import Path
from mica_at import Module, converse
S29=int(sys.argv[1]); N=int(sys.argv[2])
SP=''+str(__import__('pathlib').Path(__file__).resolve().parent)+''
with Module(Path('mica-portware-map.bin').read_bytes(), courier='../courier-emu') as m:
    m.boot(); core=m.core
    converse(m,0,[f'ATS29={S29}','ATS53=0'],per_line=3_000_000)
    m.host.configure_call(0, originate=False); m.run(300_000)
    m.host.dsp_command(0,0xA0,[0]); m.run(N)
    pickle.dump([core.data(a) for a in range(0x10000)],open(f'{SP}/cfg_{S29}.pkl','wb'))
