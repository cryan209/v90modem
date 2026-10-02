#!/bin/bash
# One call from the slmodemd/d-modem rig to us, run ON TOWER (needs docker).
# We answer in the v90modem-sip container as one of our own accounts (6001
# refuses registration now); the rig dials that account.
#
#   slm_answer_call.sh <artifact-dir-name> [hold-seconds]
#
# MS=34,0,2400,33600 forces the peer to plain V.34; unset leaves it in its
# default automode (V.90).  ME_* in the environment pass through to our side.
set -u
NAME=${1:?usage: slm_answer_call.sh <artifact-dir-name> [hold-seconds]}
HOLD=${2:-120}
ACCT=${SLM_ACCT:-2901}           # 2900-2904 u-law, 2905-2909 A-law
MS=${MS:-}
NPARM=${NPARM:-'\\N3'}           # literal, as v34_lapm_call.sh sends it
SLMODEMD=${SLMODEMD:-/src/slmodemd/slmodemd}
HEADROOM=${HEADROOM:-0.25}
PAYLOAD=${PAYLOAD:-}             # set: send numbered lines both ways after connect
PAY_DELAY=${PAY_DELAY:-50}       # seconds from ATD to the first payload line
DIR=/root/v90modem/artifacts/$NAME
case "$ACCT" in 290[5-9]) LAW=SIP_FORCE_PCMA=1 ;; *) LAW=SIP_FORCE_PCMU=1 ;; esac

envs=$(env | grep -E '^(ME|V34|V90|VPCM|V42|DS)_' | sed 's/^/-e /' | tr '\n' ' ')
docker exec v90modem-sip sh -c "mkdir -p $DIR"
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
    -e DM_RS_HEADROOM=$HEADROOM d-modem sh -c "$SLMODEMD -d9 -e /src/d-modem > /tmp/slm.log 2>&1"
sleep 12
if [ -n "$PAYLOAD" ]; then
    # Downstream payload: numbered lines into our PTY on the same schedule.
    docker exec -d v90modem-sip sh -c "sleep $PAY_DELAY; i=0; end=\$(( \$(date +%s) + $HOLD - $PAY_DELAY - 10 )); \
        while [ \$(date +%s) -lt \$end ]; do j=0; while [ \$j -lt 100 ]; do printf 'D%07d\r\n' \$i; i=\$((i+1)); j=\$((j+1)); done; sleep 0.05; done > /tmp/v90slm"
fi
(
  printf 'AT\r';   sleep 2
  printf 'ATZ\r';  sleep 2
  printf 'ATX3\r'; sleep 1
  printf 'ATE1V1Q0\r'; sleep 1
  printf 'AT%s\r' "$NPARM"; sleep 1
  [ -n "$MS" ] && { printf 'AT+MS=%s\r' "$MS"; sleep 2; }
  printf 'ATD%s\r' "$ACCT"
  if [ -n "$PAYLOAD" ]; then
    # Upstream payload: numbered lines from the peer's DTE once the call is
    # up, paced at about 2000 lines (20 kB) a second so the rig never has to
    # buffer more than a moment of it.
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
    > /tmp/slm-serial-$NAME.out 2>&1
docker exec d-modem sh -c 'cat /tmp/slm.log' > /tmp/slm-peer-$NAME.log 2>/dev/null
docker cp /tmp/slm-serial-$NAME.out v90modem-sip:$DIR/peer-serial.out
docker cp /tmp/slm-peer-$NAME.log v90modem-sip:$DIR/peer.log
sleep 5
docker exec v90modem-sip sh -c "pkill -x sip_v90_modem; pkill -f 'cat /tmp/v90slm'; true"
if [ -n "$PAYLOAD" ]; then
    docker exec v90modem-sip sh -c "cd $DIR; \
      echo upstream U-lines at our DTE: \$(tr -d '\r' < pty-rx.bin | grep -c '^U[0-9]\{7\}\$') of last sent \$(tr -d '\r' < pty-rx.bin | grep -o '^U[0-9]\{7\}\$' | tail -1); \
      echo downstream D-lines at peer DTE: \$(tr -d '\r' < peer-serial.out | grep -c '^D[0-9]\{7\}\$') of last \$(tr -d '\r' < peer-serial.out | grep -o '^D[0-9]\{7\}\$' | tail -1)"
fi
echo "done: $DIR"
