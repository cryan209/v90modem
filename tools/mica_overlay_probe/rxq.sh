#!/bin/zsh
# usage: rxq.sh TAG GAIN [S29]  -> ones fraction of MICA's received words 13-16.5 s
SC=/private/tmp/claude-501/-Users-scottcryan-v90modem/b442a30f-0d70-488d-a0ac-3b8d09113582/scratchpad/rig
cd /Users/scottcryan/MicaEmu
GAIN=$2 $SC/.build/mica_line --map mica-portware-map.bin --seconds 17 --init-at ATS29=${3:-3} --capture-peer $SC/q_$1.g711 --dte-tcp $((7600+RANDOM%300)) -- $SC/terbo_peer 4.5 0 0 > /dev/null 2>&1
$SC/.build/mica_trace $SC/q_$1.g711 --map mica-portware-map.bin --option 19 ${3:-3} --host --events --from 13 --to 16.5 --seconds 16.6 2>&1 | grep -E "dsp0  feffe9[0-9a-f]{2} <-" | awk '$4=="<-"{print $3, $5}' > $SC/q_$1.txt
python3 - <<PY
vals=[]
for l in open('$SC/q_$1.txt'):
    a,v=l.split(); a=int(a,16)
    if 0xfeffe98c<=a<0xfeffe9ff: vals.append(int(v,16))
bits=[(v>>k)&1 for v in vals for k in range(16)]
def desc(a,b): d=[bits[n]^(bits[n-a] if n>=a else 0)^(bits[n-b] if n>=b else 0) for n in range(len(bits))]; return sum(d[100:])/len(d[100:])
print('gain $2 S29 ${3:-3}: words',len(vals),'ones raw %.3f  GPC %.3f  GPA %.3f'%(sum(bits)/len(bits),desc(18,23),desc(5,23)))
PY
