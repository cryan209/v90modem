#!/bin/bash
# Single control call, morning-batch style: no PTY pump, plain nc dial.
# Reports whether the handshake completes ("V.90 startup complete").
TOWER=root@tower.net.cryan.nz
SERVERLOG="$1"

ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'for d in /proc/[0-9]*; do cmd=\$(tr \"\\0\" \" \" < \$d/cmdline 2>/dev/null); case \"\$cmd\" in *slmodemd*|*socat*) kill \${d#/proc/} 2>/dev/null;; esac; done; sleep 1; rm -f /tmp/slm.log; true'; docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz -e DM_RESAMPLER=sinc -e DM_RS_HEADROOM=0.25 d-modem sh -c '/src/slmodemd/slmodemd_trnref -d9 -e /src/d-modem > /tmp/slm.log 2>&1'; sleep 3; docker exec -d d-modem sh -c 'socat TCP-LISTEN:5556,reuseaddr,fork FILE:/dev/ttySL0,raw,echo=0'" 2>/dev/null
sleep 2
at=$( (printf 'AT\r'; sleep 2) | nc -4 -w 5 tower.net.cryan.nz 5556 2>/dev/null )
case "$at" in *OK*) echo "CONTROL: AT bridge OK";; *) echo "CONTROL: AT bridge dead"; exit 1;; esac

off=$(stat -f %z "$SERVERLOG" 2>/dev/null); off=${off:-0}
(printf 'ATZ\r'; sleep 2; printf 'ATX3\r'; sleep 2; printf 'AT+MS=90,1,300,56000\r'; sleep 2; printf 'ATD6001\r'; sleep 280) | nc -4 -w 300 tower.net.cryan.nz 5556 > /dev/null 2>&1 &
ncpid=$!
outcome=none
for tick in $(seq 1 30); do
  sleep 6
  slice=$(tail -c "+$((off+1))" "$SERVERLOG" 2>/dev/null)
  if echo "$slice" | grep -aq "V.90 startup complete"; then outcome=DATA; break; fi
  noc=$(ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'grep -ac \"NO CARRIER\" /tmp/slm.log 2>/dev/null'" 2>/dev/null)
  if [ "${noc:-0}" -gt 0 ] 2>/dev/null; then outcome=NOCARRIER; break; fi
done
kill $ncpid 2>/dev/null
lastee=$(ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'grep -a \"Error Energy\" /tmp/slm.log | tail -1'" 2>/dev/null)
echo "CONTROL: outcome=$outcome  last peer Error Energy: $lastee"
