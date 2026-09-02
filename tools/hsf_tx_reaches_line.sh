#!/bin/bash
# Does our DTMF reach the LINE?  off-hook return must exceed the on-hook
# control, which is pure internal codec loopback.
set -euo pipefail
tmpdir=$(mktemp -d "${TMPDIR:-/tmp}/hsf-tx-line.XXXXXX")
trap 'rm -rf "$tmpdir"' EXIT
run() {
  local cfg=$1 name=$2 raw="$tmpdir/$2.raw" log="$tmpdir/$2.log"
  if ! env $cfg HSF_CALL_INIT=1 ./hsf_fxo_probe --call-seq --dial 9898 --dial-amp 20000 \
       --stream 8 --rx-out "$raw" >"$log" 2>&1; then
    cat "$log" >&2
    return 1
  fi
  if [[ ! -s "$raw" ]]; then
    echo "hsf_fxo_probe produced no receive samples ($name)" >&2
    cat "$log" >&2
    return 1
  fi
  python3 - "$raw" <<'PY'
import struct, cmath, math
import sys
FS=16000.0
rx=[x[0] for x in struct.iter_unpack('<h',open(sys.argv[1],'rb').read())]
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
  label=${cfg:-default}
  off=$(run "$cfg" "${label//[^A-Za-z0-9]/_}-off")
  on=$(run "$cfg HSF_NO_HOOK=1" "${label//[^A-Za-z0-9]/_}-on")
  echo "cfg='${cfg:-default}'  off-hook=$off  on-hook(loopback)=$on"
done
