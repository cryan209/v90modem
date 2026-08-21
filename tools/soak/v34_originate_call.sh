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
ssh -o BatchMode=yes "$TOWER" "docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz -e DM_RESAMPLER=sinc -e DM_RS_HEADROOM=${DM_RS_HEADROOM:-0.25} -e DM_TX_GAIN=${DM_TX_GAIN:-4.0} -e DM_RX_GAIN=${DM_RX_GAIN:-1.0} d-modem sh -c '$SLMODEMD -d9 -e /src/d-modem > /tmp/slm.log 2>&1'" 2>/dev/null
sleep 12

# Arm the peer to answer: V.34 only, then ATA.
#
# ATA is what puts slmodemd off-hook in answer mode, and socket_start() forks
# d-modem with m->dial_string as argv[1] -- empty after ATA, which the patched
# d-modem reads as "register and answer the next INVITE".  Sample flow does
# not start until the media is up, so the answer datapump stays stalled in its
# read until we dial.  slmodemd's own signal-detect timeout is 12 s, so the
# dial has to follow within a few seconds.
(
  printf 'AT\r';   sleep 2
  printf 'ATZ\r';  sleep 2
  printf 'ATX3\r'; sleep 1
  printf 'ATE1V1Q0\r'; sleep 1
  printf 'AT\\N0\r'; sleep 1
  printf 'AT+MS=%s\r' "$MS"; sleep 2
  printf 'ATA\r'
  sleep "$HOLD"
) | ssh -o BatchMode=yes "$TOWER" docker exec -i d-modem socat /dev/ttySL0,raw,echo=0,b115200 - >"$SERIAL_OUT" 2>&1 &
armpid=$!
# Wait for the peer to actually be registered before dialling: ATA has to
# fork d-modem, start pjsua and complete a REGISTER first, and dialling ahead
# of that gets the call routed somewhere else entirely (V.8 then fails against
# whatever answered).
for i in $(seq 1 20); do
    sleep 1
    if ssh -o BatchMode=yes "$TOWER" \
        "docker exec d-modem sh -c 'grep -ac \"registration success\" /tmp/slm.log'" \
        2>/dev/null | grep -qv '^0$'; then
        echo "CONTROL: peer registered after ${i}s"
        break
    fi
done

off=$(stat -f %z "$SERVERLOG" 2>/dev/null); off=${off:-0}

# The PBX does not always route to a freshly-registered 6000 -- some attempts
# are answered before they reach the peer at all (its log records no INVITE).
# Retry until the peer actually sees the call.
dialpid=
for attempt in 1 2 3; do
    python3 "$(dirname "$0")/../pty_dial.py" 6000 --pty "$PTY" --wait "$HOLD" \
        >"${SERIAL_OUT%.out}.local.out" 2>&1 &
    dialpid=$!
    got=no
    for i in $(seq 1 8); do
        sleep 1
        if ssh -o BatchMode=yes "$TOWER" \
            "docker exec d-modem sh -c 'grep -ac \"Incoming call\" /tmp/slm.log'" \
            2>/dev/null | grep -qv '^0$'; then
            got=yes; break
        fi
    done
    if [ "$got" = yes ]; then
        echo "CONTROL: peer took the call on dial attempt $attempt"
        break
    fi
    echo "CONTROL: dial attempt $attempt never reached the peer; retrying"
    kill "$dialpid" 2>/dev/null || true
    wait "$dialpid" 2>/dev/null || true
    sleep 3
done

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
