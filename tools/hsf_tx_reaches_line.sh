#!/bin/bash
# Does our DTMF reach the LINE?  off-hook return must exceed the on-hook
# control, which is pure internal codec loopback.
run() { env $1 HSF_CALL_INIT=1 ./hsf_fxo_probe --call-seq --dial 9898 --dial-amp 20000 \
        --stream 8 --rx-out /tmp/tt.raw >/dev/null 2>&1
  python3 - <<'PY'
import struct, cmath, math
FS=16000.0
rx=[x[0] for x in struct.iter_unpack('<h',open('/tmp/tt.raw','rb').read())]
W=1600; best=0
def amp(s,f):
    N=len(s); k=f*N/FS
    return 2*abs(sum(s[j]*cmath.exp(-2j*math.pi*k*j/N) for j in range(N)))/N
for i in range(int(FS*0.9),int(FS*2.2),W):
    s=rx[i:i+W]; m=sum(s)/W; s=[x-m for x in s]
    best=max(best,(amp(s,852)+amp(s,1477))/2)
print("%.0f"%best)
PY
}
for cfg in "" "HSF_TRAIL2_REG=0x35b7"; do
  off=$(run "$cfg"); on=$(run "$cfg HSF_NO_HOOK=1")
  echo "cfg='${cfg:-default}'  off-hook=$off  on-hook(loopback)=$on"
done
