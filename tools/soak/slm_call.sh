#!/bin/bash
# One call between us (v90modem-sip container) and the slmodemd/d-modem rig,
# in either direction, run ON TOWER (needs docker).
#
#   ORIG=slm|ours slm_call.sh <artifact-dir-name> [hold-seconds]
#
# ORIG=slm  (default) the rig dials our account, we answer.
# ORIG=ours we dial 6000; the rig is armed with ATA (d-modem's listen mode:
#           an empty dial string after ATA means "register, answer the next
#           INVITE"), and slmodemd's 12 s signal-detect timeout means the dial
#           has to follow its registration within a few seconds.
#
# MS=<slmodemd +MS>  e.g. 132 (V.32bis), 34, 90, 92; unset = its automode.
# SLM_*/DM_* in the environment go to slmodemd/d-modem (SLM_V92_PCM_UPSTREAM=1 lifts its
# V.92 PCM-upstream ignore), ME_*/V34_*/V90_*/... to our server.
# PAYLOAD=1 sends numbered lines both ways once the call is up and grades them.
set -u
NAME=${1:?usage: ORIG=slm|ours slm_call.sh <artifact-dir-name> [hold-seconds]}
HOLD=${2:-120}
ORIG=${ORIG:-slm}
ACCT=${SLM_ACCT:-2901}           # 2900-2904 u-law, 2905-2909 A-law
PEER=${SLM_PEER:-6000}
MS=${MS:-}
NPARM=${NPARM:-'\N3'}
SLMODEMD=${SLMODEMD:-/src/slmodemd/slmodemd}
HEADROOM=${HEADROOM:-0.25}
PAYLOAD=${PAYLOAD:-}
PAY_DELAY=${PAY_DELAY:-50}       # seconds from the dial to the first payload line
DIR=/root/v90modem/artifacts/$NAME
case "$ACCT" in 290[5-9]) LAW=SIP_FORCE_PCMA=1 ;; *) LAW=SIP_FORCE_PCMU=1 ;; esac

envs=$(env | grep -E '^(ME|V34|V90|V92|VPCM|V42|V44|DS)_' | sed 's/^/-e /' | tr '\n' ' ')
slmenvs=$(env | grep -E '^(SLM|DM)_[A-Z0-9_]*=' | grep -vE '^(SLM_(ACCT|PEER)|DM_RS_HEADROOM)=' | sed 's/^/-e /' | tr '\n' ' ')

docker exec v90modem-sip sh -c "pkill -x sip_v90_modem; pkill -f '[c]at /tmp/v90slm'; mkdir -p $DIR; true"
docker exec -d $envs -e $LAW -e ME_LAPM_XID_OPTION_OCTETS=auto -e VPCM_G711_TAP_DIR=$DIR \
    v90modem-sip sh -c "cd /root/v90modem && timeout $((HOLD + 60)) ./sip_v90_modem \
        --sip-server asterisk.net.cryan.nz --username $ACCT --password $ACCT \
        --local-port 5074 --rtp-port 14100 --pty-link /tmp/v90slm --verbose \
        > $DIR/server.log 2>&1"
sleep 6
# raw and -echo first: in echo mode the tty hands every byte we receive back
# to us as DTE input, which we then transmit to the peer.
docker exec v90modem-sip sh -c "stty -F /tmp/v90slm raw -echo; (cat /tmp/v90slm > $DIR/pty-rx.bin &) ; true"

docker restart d-modem >/dev/null 2>&1
sleep 8
docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz -e DM_RESAMPLER=sinc \
    -e DM_RS_HEADROOM=$HEADROOM $slmenvs d-modem sh -c "$SLMODEMD -d9 -e /src/d-modem > /tmp/slm.log 2>&1"
sleep 12

if [ -n "$PAYLOAD" ]; then
    docker exec -d v90modem-sip sh -c "sleep $PAY_DELAY; i=0; end=\$(( \$(date +%s) + $HOLD - $PAY_DELAY - 10 )); \
        while [ \$(date +%s) -lt \$end ]; do j=0; while [ \$j -lt 100 ]; do printf 'D%07d\r\n' \$i; i=\$((i+1)); j=\$((j+1)); done; sleep 0.05; done > /tmp/v90slm"
fi

if [ "$ORIG" = slm ]; then DIALCMD="ATD$ACCT"; else DIALCMD=ATA; fi
(
  printf 'AT\r';   sleep 2
  printf 'ATZ\r';  sleep 2
  printf 'ATX3\r'; sleep 1
  printf 'ATE1V1Q0\r'; sleep 1
  printf 'AT%s\r' "$NPARM"; sleep 1
  [ -n "$MS" ] && { printf 'AT+MS=%s\r' "$MS"; sleep 2; }
  printf '%s\r' "$DIALCMD"
  if [ -n "$PAYLOAD" ]; then
    sleep "$PAY_DELAY"
    i=0; end=$(( $(date +%s) + HOLD - PAY_DELAY - 10 ))
    while [ "$(date +%s)" -lt "$end" ]; do
      j=0; while [ $j -lt 100 ]; do printf 'U%07d\r\n' $i; i=$((i+1)); j=$((j+1)); done
      sleep 0.05
    done
    sleep 10
  else
    sleep "$HOLD"
  fi
) | docker exec -i d-modem timeout $((HOLD + 30)) socat /dev/ttySL0,raw,echo=0,b115200 - \
    > /tmp/slm-serial-$NAME.out 2>&1 &
peerpid=$!

if [ "$ORIG" = ours ]; then
    # ATA forks d-modem, which must REGISTER before the INVITE can reach it.
    for i in $(seq 1 25); do
        sleep 1
        if docker exec d-modem sh -c 'grep -ac "registration success" /tmp/slm.log' 2>/dev/null | grep -qv '^0$'; then
            echo "peer registered after ${i}s (after config)"; break
        fi
    done
    docker exec v90modem-sip sh -c "printf 'AT\r' > /tmp/v90slm; sleep 1; printf 'ATD$PEER\r' > /tmp/v90slm"
fi
wait $peerpid

docker exec d-modem sh -c 'cat /tmp/slm.log' > /tmp/slm-peer-$NAME.log 2>/dev/null
docker cp /tmp/slm-serial-$NAME.out v90modem-sip:$DIR/peer-serial.out
docker cp /tmp/slm-peer-$NAME.log v90modem-sip:$DIR/peer.log
sleep 5
docker exec v90modem-sip sh -c "pkill -x sip_v90_modem; pkill -f '[c]at /tmp/v90slm'; true"
docker exec v90modem-sip sh -c "cd $DIR; \
  echo ours: \$(grep -ao 'CONNECT[^\r]*' pty-rx.bin | head -1); \
  echo peer: \$(grep -ao 'CONNECT[^\r]*' peer-serial.out | head -1); \
  echo upstream U-lines at our DTE: \$(tr -d '\r' < pty-rx.bin | grep -ac '^U[0-9]\{7\}\$') last \$(tr -d '\r' < pty-rx.bin | grep -ao '^U[0-9]\{7\}\$' | tail -1); \
  echo downstream D-lines at peer DTE: \$(tr -d '\r' < peer-serial.out | grep -ac '^D[0-9]\{7\}\$') last \$(tr -d '\r' < peer-serial.out | grep -ao '^D[0-9]\{7\}\$' | tail -1)"
echo "done: $DIR"
