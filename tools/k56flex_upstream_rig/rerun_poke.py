"""Rerun the saved 28800 connect command with a poke-capable mica_trace.

Build: patch a COPY of MicaEmu native/mica_trace.cpp with mica_trace_poke.patch,
compile it against .build/k56flex-board/obj/{i960_core,mica_board,c5x_core}.o
and put the binary at S/bld/mica_trace_poke. usage: rerun_poke.py S TAG [--poke T ADDR VAL]...
Run from the MicaEmu directory. Extra args go before the peer `--`; outputs in S/k56/TAG.
"""
import json,sys,os,subprocess
# usage: pk.py S tag [--poke T A V ...]  : full command with the poke binary and extra args before '--'
S,tag,*extra=sys.argv[1:]
f='artifacts/k56flex-client-20261009/connect-xc-input-command.json'
c=json.load(open(f)); old=os.path.abspath(os.path.dirname(f)); D=f'{S}/k56/{tag}'; os.makedirs(D+'/dte',exist_ok=True)
a=[x.replace(old,D) for x in c['command']]
a[0]=S+'/bld/mica_trace_poke'
i=a.index('--'); a[i:i]=extra
if os.environ.get('TRN'):  # client --trn-symbols (default 512)
    j=a.index('--report'); a[j:j]=['--trn-symbols',os.environ['TRN']]
env={**os.environ,**c['environment'],'V34_DATA_TX_DUMP':f'{D}/tx','K56_REPORT_GRID_SCALE':os.environ.get('K56_REPORT_GRID_SCALE','1.3417')}
r=subprocess.run(a,env=env,cwd=c['cwd'],capture_output=True,text=True)
print(tag,r.returncode,[l for l in r.stderr.splitlines() if 'poked' in l])
