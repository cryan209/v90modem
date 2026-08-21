#!/bin/bash
# One plain-V.34 (no V.90) call from the d-modem rig, held long enough for
# V.42 LAPM to establish.  The peer is forced to V.34 only with automode 0,
# so nothing here depends on our V.8 offer suppressing V.90.
#
#   v34_lapm_call.sh <server.log> [hold-seconds] [serial-out]
set -u
TOWER=${TOWER:-root@tower.net.cryan.nz}
SERVERLOG=${1:?usage: v34_lapm_call.sh server.log [hold-sec] [serial-out]}
HOLD=${2:-120}
SERIAL_OUT=${3:-/tmp/v34-lapm-serial.out}
SLMODEMD=${SLMODEMD:-/src/slmodemd/slmodemd}
MS=${MS:-34,0,2400,33600}

echo "CONTROL: bouncing rig ($SLMODEMD, AT+MS=$MS)"
ssh -o BatchMode=yes "$TOWER" "docker restart d-modem" >/dev/null 2>&1
sleep 8
ssh -o BatchMode=yes "$TOWER" "docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz -e DM_RESAMPLER=sinc -e DM_RS_HEADROOM=0.25 d-modem sh -c '$SLMODEMD -d9 -e /src/d-modem > /tmp/slm.log 2>&1'" 2>/dev/null
sleep 12

off=$(stat -f %z "$SERVERLOG" 2>/dev/null); off=${off:-0}
(
  printf 'AT\r';   sleep 2
  printf 'ATZ\r';  sleep 2
  printf 'ATX3\r'; sleep 1
  printf 'ATE1V1Q0\r'; sleep 1
  printf 'AT\\N0\r'; sleep 1
  printf 'AT+MS=%s\r' "$MS"; sleep 2
  printf 'AT+MS?\r'; sleep 2
  printf 'ATD6001\r'
  sleep "$HOLD"
) | ssh -o BatchMode=yes "$TOWER" docker exec -i d-modem socat /dev/ttySL0,raw,echo=0,b115200 - >"$SERIAL_OUT" 2>&1 &
dialpid=$!

outcome=none
for tick in $(seq 1 $(( HOLD / 5 )) ); do
    sleep 5
    slice=$(tail -c "+$((off+1))" "$SERVERLOG" 2>/dev/null)
    if echo "$slice" | grep -aq "V.42 LAPM connected"; then outcome=LAPM; break; fi
    if echo "$slice" | grep -aqE "V.42 (detection reported unsupported|LAPM link error|LAPM disconnected)"; then
        outcome=LAPM_FAILED; break
    fi
done
kill "$dialpid" 2>/dev/null || true
wait "$dialpid" 2>/dev/null || true
ssh -o BatchMode=yes "$TOWER" "docker exec d-modem sh -c 'cat /tmp/slm.log'" >"${SERIAL_OUT%.out}.peer.log" 2>/dev/null
echo "CONTROL: outcome=$outcome serial=$SERIAL_OUT peer=${SERIAL_OUT%.out}.peer.log"
[ "$outcome" = LAPM ]
