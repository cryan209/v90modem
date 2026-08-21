#!/bin/bash
# Plain-V.34 call in the other direction: we originate, the d-modem peer
# answers.  The peer's shipped DSP runs a different Phase 2 state family in
# each role (TX_PHASE1_ANS/RX_PHASE1_ANS vs TX_PHASE1_CALL/RX_PHASE1_CALL),
# and only the answer-role one has ever completed on this rig -- its
# call-role path enters the 11.2.2.1.1 recovery on entering TX_PHASE2_CALL in
# every call and never leaves it.
#
#   v34_originate_call.sh <server.log> [hold-seconds] [serial-out]
set -u
TOWER=${TOWER:-root@tower.net.cryan.nz}
SERVERLOG=${1:?usage: v34_originate_call.sh server.log [hold-sec] [serial-out]}
HOLD=${2:-120}
SERIAL_OUT=${3:-/tmp/v34-answer-serial.out}
SLMODEMD=${SLMODEMD:-/src/slmodemd/slmodemd}
MS=${MS:-34,0,2400,33600}
PTY=${PTY:-/tmp/v90modem}

echo "CONTROL: bouncing rig ($SLMODEMD, AT+MS=$MS, auto-answer)"
ssh -o BatchMode=yes "$TOWER" "docker restart d-modem" >/dev/null 2>&1
sleep 8
ssh -o BatchMode=yes "$TOWER" "docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz -e DM_RESAMPLER=sinc -e DM_RS_HEADROOM=0.25 d-modem sh -c '$SLMODEMD -d9 -e /src/d-modem > /tmp/slm.log 2>&1'" 2>/dev/null
sleep 12

# Arm the peer to answer: V.34 only, auto-answer on the first ring.
(
  printf 'AT\r';   sleep 2
  printf 'ATZ\r';  sleep 2
  printf 'ATX3\r'; sleep 1
  printf 'ATE1V1Q0\r'; sleep 1
  printf 'AT\\N0\r'; sleep 1
  printf 'AT+MS=%s\r' "$MS"; sleep 2
  printf 'ATS0=1\r'; sleep 2
  sleep "$HOLD"
) | ssh -o BatchMode=yes "$TOWER" docker exec -i d-modem socat /dev/ttySL0,raw,echo=0,b115200 - >"$SERIAL_OUT" 2>&1 &
armpid=$!
sleep 12

off=$(stat -f %z "$SERVERLOG" 2>/dev/null); off=${off:-0}
python3 "$(dirname "$0")/../pty_dial.py" 6000 --pty "$PTY" --wait "$HOLD" \
    >"${SERIAL_OUT%.out}.local.out" 2>&1 &
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
kill "$dialpid" "$armpid" 2>/dev/null || true
wait "$dialpid" "$armpid" 2>/dev/null || true
ssh -o BatchMode=yes "$TOWER" "docker exec d-modem sh -c 'cat /tmp/slm.log'" >"${SERIAL_OUT%.out}.peer.log" 2>/dev/null
echo "CONTROL: outcome=$outcome peer=${SERIAL_OUT%.out}.peer.log local=${SERIAL_OUT%.out}.local.out"
[ "$outcome" = LAPM ]
