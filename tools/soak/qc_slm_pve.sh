#!/bin/bash
# One call between our server and slmodemd/d-modem on the Proxmox rig, run ON the
# Proxmox host (needs `pct`).  Containers: CT_SLM (100) runs /root/D-Modem
# (slmodemd + the repo's rig/d-modem/d-modem.c, sinc resampler), CT_OURS (117)
# runs /root/v90modem/sip_v90_modem.  Accounts on asterisk.net.cryan.nz:
# slmodemd 6003, ours 6002 (6000, 6001 do not authenticate with pw=ext).
# CT_SLM has two builds: SLM_BUILD=rig (default) is /root/D-Modem-rig = D-Modem at cf5ea9e6
# (before its internal RcFixed resampler) + the repo's rig/d-modem/d-modem.c with the sinc
# resampler, like the tower; SLM_BUILD=stock is /root/D-Modem (RcFixed, --sip-* flags), which
# cannot hold a V.90 downstream (rig/README.md).
# SLM_BIN=slmodemd_map (rig build only) is the same slmodemd linked with the V92EchoCanceller
# bypass hook (rig/slmodemd/README.md); both carry the env-gated PCM-upstream patch, so a V.92
# call is SLM_BIN=slmodemd_map SLM_ENV='SLM_V92_PCM_UPSTREAM=1 DM_V92EC_BYPASS=1'.
#
#   ORIG=slm|ours PQC=0|1|2|3 MS=<slm +MS> HOLD=<s> OURS_ENV="ME_V90_ROLE=analogue" \
#   OURS_INIT="AT+PIG=0" qc_slm_pve.sh <name>
#
# ORIG=slm  slmodemd dials our account.   ORIG=ours  we dial 6003 (slmodemd ATA).
# Results land in /root/qc/<name> in CT_OURS (server.log, pty-rx.bin) and
# /tmp/qc-<name>.* on this host (slmodemd serial and log).  PAYLOAD=1 sends
# numbered lines both ways and counts them.
set -u
NAME=${1:?usage: qc_slm_pve.sh <name>}
CT_SLM=${CT_SLM:-100}
CT_OURS=${CT_OURS:-117}
ORIG=${ORIG:-slm}
PQC=${PQC:-3}
MS=${MS:-92}
HOLD=${HOLD:-90}
PAYLOAD=${PAYLOAD-1}
OURS_ENV=${OURS_ENV:-}
OURS_INIT=${OURS_INIT:-AT+PIG=0}
OUR_ACCT=${OUR_ACCT:-6002}
SLM_ENV=${SLM_ENV:-}
SLM_ACCT=${SLM_ACCT:-6003}
DIR=/root/qc/$NAME
ox() { pct exec $CT_OURS -- bash -c "$1"; }
sx() { pct exec $CT_SLM -- bash -c "$1"; }

ox "pkill -x sip_v90_modem; sleep 1; mkdir -p $DIR; rm -f /tmp/v90slm; pkill -f '[c]at /tmp/v90slm'; true"
ox "cd ${SRV_DIR:-/root/v90modem} && ( env SIP_FORCE_PCMU=1 ME_MODE=v92 ME_LAPM_XID_OPTION_OCTETS=auto VPCM_G711_TAP_DIR=$DIR $OURS_ENV \
    nohup timeout $((HOLD + 90)) ./sip_v90_modem --sip-server asterisk.net.cryan.nz --username $OUR_ACCT --password $OUR_ACCT \
    --local-port 5074 --rtp-port 14100 --pty-link /tmp/v90slm --verbose > $DIR/server.log 2>&1 & ); sleep 5"
ox "stty -F /tmp/v90slm raw -echo; ( cat /tmp/v90slm > $DIR/pty-rx.bin & ) ; true"
for c in "AT" "$OURS_INIT" "AT+PQC=$PQC"; do
    echo "$c" | tr ';' '\n' | while read -r l; do
        [ -n "$l" ] && ox "printf '%s\r' '$l' > /tmp/v90slm"; sleep 1
    done
done

sx "pkill '[s]lmodemd'; pkill '[d]-modem'; sleep 1; rm -f /dev/ttySL0"
if [ "${SLM_BUILD:-rig}" = rig ]; then
    # The tower rig's build (/root/D-Modem-rig, D-Modem before its internal resampler, built
    # with the repo's rig/d-modem/d-modem.c): sinc resampler in d-modem, SIP_LOGIN env.
    sx "cd /root/D-Modem-rig; rm -f /tmp/slm.log; \
        ( env SLMODEMD_V8_OPEN_STUB=0 DM_RESAMPLER=sinc DM_UP_RESAMPLER=${DM_UP_RESAMPLER:-sinc} DM_RS_HEADROOM=0.25 DM_SLM_FRAME_TRAILER=4 SIP_FORCE_PCMU=1 $SLM_ENV \
          SIP_LOGIN=$SLM_ACCT:$SLM_ACCT@asterisk.net.cryan.nz nohup ./slmodemd/${SLM_BIN:-slmodemd} -d9 -e /root/D-Modem-rig/d-modem \
          > /tmp/slm.log 2>&1 & ); sleep 8; chmod 666 \$(readlink -f /dev/ttySL0)"
else
    sx "cd /root/D-Modem; rm -f /tmp/slm.log; \
        ( env SLMODEMD_V8_OPEN_STUB=0 SIP_FORCE_PCMU=1 $SLM_ENV \
          nohup ./slmodemd/slmodemd -d9 -e /root/D-Modem/d-modem --sip-server asterisk.net.cryan.nz --sip-user $SLM_ACCT --sip-password $SLM_ACCT \
          > /tmp/slm.log 2>&1 & ); sleep 8; chmod 666 \$(readlink -f /dev/ttySL0)"
fi

if [ "$ORIG" = slm ]; then DIALCMD="ATD$OUR_ACCT"; else DIALCMD=ATA; fi
PAY_DELAY=${PAY_DELAY:-45}
if [ -n "$PAYLOAD" ]; then
    ox "( sleep $PAY_DELAY; i=0; end=\$(( \$(date +%s) + $HOLD - $PAY_DELAY - 10 )); \
        while [ \$(date +%s) -lt \$end ]; do j=0; while [ \$j -lt 50 ]; do printf 'U%07d\r\n' \$i > /tmp/v90slm; i=\$((i+1)); j=\$((j+1)); done; sleep 0.1; done ) > /dev/null 2>&1 &"
fi
(
  printf 'AT\r';   sleep 2
  printf 'ATZ\r';  sleep 2
  printf 'ATX3\r'; sleep 1
  printf 'ATE1V1Q0\r'; sleep 1
  printf 'AT%s\r' "${SLM_NPARM:-\\N3}"; sleep 1
  [ -n "$MS" ] && { printf 'AT+MS=%s\r' "$MS"; sleep 2; }
  printf '%s\r' "$DIALCMD"
  if [ -n "$PAYLOAD" ]; then
    sleep "$PAY_DELAY"
    i=0; end=$(( $(date +%s) + HOLD - PAY_DELAY - 10 ))
    while [ "$(date +%s)" -lt "$end" ]; do
      j=0; while [ $j -lt 50 ]; do printf 'D%07d\r\n' $i; i=$((i+1)); j=$((j+1)); done
      sleep 0.1
    done
    sleep 10
  else
    sleep "$HOLD"
  fi
) | pct exec $CT_SLM -- timeout $((HOLD + 30)) socat /dev/ttySL0,raw,echo=0,b115200 - > /tmp/qc-$NAME.serial 2>&1 &
peer=$!

if [ "$ORIG" = ours ]; then
    # slmodemd's ATA gives up after 12 s of no signal: dial soon after it registers.
    sleep ${DIAL_DELAY:-13}
    ox "printf 'ATD$SLM_ACCT\r' > /tmp/v90slm"
fi
wait $peer
sx "cat /tmp/slm.log" > /tmp/qc-$NAME.slm.log 2>/dev/null
sleep 2
ox "pkill -x sip_v90_modem; pkill -f '[c]at /tmp/v90slm'; true"
sx "pkill '[s]lmodemd'; true"
echo "== $NAME (ORIG=$ORIG PQC=$PQC MS=$MS ${OURS_ENV})"
echo "ours : $(ox "grep -ao 'CONNECT[^\r]*' $DIR/pty-rx.bin | head -1")"
echo "peer : $(grep -ao 'CONNECT[^\r]*' /tmp/qc-$NAME.serial | head -1)"
echo "ours->slm U-lines at the peer DTE: $(tr -d '\r' < /tmp/qc-$NAME.serial | grep -ac '^U[0-9]\{7\}$')  slm->ours D-lines at our DTE: $(ox "tr -d '\r' < $DIR/pty-rx.bin | grep -ac '^D[0-9]\{7\}\$'")"
echo "ours QC log:"; ox "grep -a 'V.92 9\.\|Quick Connect\|short Phase\|ODP' $DIR/server.log | cut -c1-170 | head -12"
