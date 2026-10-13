#!/bin/zsh
# usage: tq.sh iso k m port -> MICA's received-word ones fraction in terbo mode with the caller transmitting that variant
SC=/private/tmp/claude-501/-Users-scottcryan-v90modem/b442a30f-0d70-488d-a0ac-3b8d09113582/scratchpad/rig
cd /Users/scottcryan/MicaEmu
tag=$1_$2_$3
TERBO_TX=$1,$2,$3 $SC/.build/mica_line --map mica-portware-map.bin --seconds 17 --init-at ATS29=2 --capture-peer $SC/tq_$tag.g711 --dte-tcp $4 -- $SC/terbo_peer 4.5 0x4000 1 > /dev/null 2>&1
$SC/.build/mica_trace $SC/tq_$tag.g711 --map mica-portware-map.bin --option 19 2 --host --events --from 13 --to 16.5 --seconds 16.6 2>&1 | awk '$4=="<-"&&$2=="dsp0"{print $3,$5}' > $SC/tq_$tag.txt
python3 - <<PY
vals=[]
for l in open('$SC/tq_$tag.txt'):
    a,v=l.split(); a=int(a,16)
    if 0xfeffe98c<=a<0xfeffe9ff: vals.append(int(v,16))
bits=[(v>>k)&1 for v in vals for k in range(16)]
print('terbo tx %s: words %d  ones %.3f'%('$tag',len(vals),sum(bits)/max(1,len(bits))))
PY
