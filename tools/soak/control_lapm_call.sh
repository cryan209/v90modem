#!/bin/bash
# Hold a SmartLink call past physical DATA and require V.42 LAPM establishment.
set -u
TOWER=${TOWER:-root@tower.net.cryan.nz}
SERVERLOG=${1:?usage: control_lapm_call.sh server.log [serial-output]}
SERIAL_OUT=${2:-/tmp/v90-lapm-serial.out}

ssh -o BatchMode=yes "$TOWER" "docker exec d-modem sh -c 'for d in /proc/[0-9]*; do cmd=\$(tr \"\\0\" \" \" < \$d/cmdline 2>/dev/null); case \"\$cmd\" in *slmodemd*|*socat*) kill \${d#/proc/} 2>/dev/null;; esac; done; sleep 1; rm -f /tmp/slm.log; true'; docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz -e DM_RESAMPLER=sinc -e DM_RS_HEADROOM=0.25 d-modem sh -c '/src/slmodemd/slmodemd_trnref -d9 -e /src/d-modem > /tmp/slm.log 2>&1'; sleep 3; docker exec -d d-modem sh -c 'socat TCP-LISTEN:5556,reuseaddr,fork FILE:/dev/ttySL0,raw,echo=0'" 2>/dev/null
sleep 2
at=$( (printf 'AT\r'; sleep 2) | nc -4 -w 5 tower.net.cryan.nz 5556 2>/dev/null )
case "$at" in *OK*) echo "CONTROL: AT bridge OK";; *) echo "CONTROL: AT bridge dead"; exit 1;; esac

off=$(stat -f %z "$SERVERLOG" 2>/dev/null); off=${off:-0}
(printf 'ATZ\r'; sleep 2; printf 'ATX3\r'; sleep 2; printf 'AT+MS=90,1,300,56000\r'; sleep 2; printf 'ATD6001\r'; sleep 280) \
    | nc -4 -w 300 tower.net.cryan.nz 5556 >"$SERIAL_OUT" 2>&1 &
ncpid=$!
outcome=none
physical=0
for tick in $(seq 1 45); do
    sleep 6
    slice=$(tail -c "+$((off+1))" "$SERVERLOG" 2>/dev/null)
    if echo "$slice" | grep -aq "V.42 LAPM connected"; then
        outcome=LAPM
        break
    fi
    if echo "$slice" | grep -aq "V.90 startup complete"; then
        physical=1
        outcome=DATA_NO_LAPM
    fi
    if echo "$slice" | grep -aqE "V.42 (detection reported unsupported|LAPM link error|LAPM disconnected)"; then
        outcome=LAPM_FAILED
        break
    fi
    noc=$(ssh -o BatchMode=yes "$TOWER" "docker exec d-modem sh -c 'grep -ac \"NO CARRIER\" /tmp/slm.log 2>/dev/null'" 2>/dev/null)
    if [ "${noc:-0}" -gt 0 ] 2>/dev/null; then
        if [ "$physical" -eq 0 ]; then outcome=NOCARRIER; fi
        break
    fi
done
kill "$ncpid" 2>/dev/null || true
wait "$ncpid" 2>/dev/null || true
lastee=$(ssh -o BatchMode=yes "$TOWER" "docker exec d-modem sh -c 'grep -a \"Error Energy\" /tmp/slm.log | tail -1'" 2>/dev/null)
echo "CONTROL: outcome=$outcome last_peer_error_energy=$lastee serial=$SERIAL_OUT"
[ "$outcome" = LAPM ]
