#!/bin/bash
# Does our DTMF reach the LINE?  Compare the line tone during a dial against
# an off-hook silence control.  On-hook is not the control: this codec returns
# TX through a strong internal loopback on-hook and suppresses it off-hook.
set -euo pipefail
tmpdir=$(mktemp -d "${TMPDIR:-/tmp}/hsf-tx-line.XXXXXX")
trap 'rm -rf "$tmpdir"' EXIT
run() {
  local cfg=$1 mode=$2 name=$3 raw="$tmpdir/$3.raw" log="$tmpdir/$3.log"
  local tx_args=(--feed)
  if [[ $mode == dial ]]; then
    tx_args=(--dial 9898 --dial-amp 20000)
  fi
  if ! env $cfg HSF_CALL_INIT=1 ./hsf_fxo_probe --call-seq "${tx_args[@]}" \
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
def amp(s,f):
    N=len(s); k=f*N/FS
    return 2*abs(sum(s[j]*cmath.exp(-2j*math.pi*k*j/N) for j in range(N)))/N
def band(t0,t1):
    s=rx[int(FS*t0):int(FS*t1)]
    m=sum(s)/len(s); s=[x-m for x in s]
    return amp(s,400)
print("pre=%.0f during=%.0f"%(band(.2,.9),band(1.2,2.5)))
PY
}
for cfg in "" "HSF_TRAIL2_REG=0x35b7"; do
  label=${cfg:-default}
  dial=$(run "$cfg" dial "${label//[^A-Za-z0-9]/_}-dial")
  silence=$(run "$cfg" silence "${label//[^A-Za-z0-9]/_}-silence")
  echo "cfg='${cfg:-default}'  dial[$dial]  silence[$silence]"
done
