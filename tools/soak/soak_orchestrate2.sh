#!/bin/bash
# Soak orchestrator v2: early-aborts a call as soon as the first-attempt
# Phase 4 MP coin flip is lost (server logs "restarting Phase 2 (2,"),
# since retrained attempts never reach data mode on this rig. Redials
# until an initial attempt survives to DATA, then lets the pumps run.
SP="$(cd "$(dirname "$0")" && pwd)"
SOAKDIR="$1"
SERVERLOG="$2"
TOWER=root@tower.net.cryan.nz
MAXATTEMPTS="${3:-8}"

bounce_rig() {
  # Kill hard and *verify*: a surviving slmodemd from the previous attempt
  # re-creates /dev/ttySL0, and the two instances then fight over the dial
  # ("Dialer was aborted" -> instant NO CARRIER on every redial).
  ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c '
      for pass in 1 2 3; do
        for d in /proc/[0-9]*; do
          cmd=\$(tr \"\\0\" \" \" < \$d/cmdline 2>/dev/null)
          case \"\$cmd\" in *slmodemd*|*socat*|*d-modem*) kill -9 \${d#/proc/} 2>/dev/null;; esac
        done
        sleep 1
      done
      rm -f /tmp/slm.log; true'" 2>/dev/null
  left=$(ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'ps ax -o comm 2>/dev/null | grep -c \"slmodemd_trnref\"'" 2>/dev/null)
  if [ "${left:-0}" -gt 0 ] 2>/dev/null; then
    echo "  bounce: $left slmodemd still alive after kill; restarting container"
    ssh -o BatchMode=yes $TOWER "docker restart d-modem" >/dev/null 2>&1
    sleep 8
  fi
  ssh -o BatchMode=yes $TOWER "docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz -e DM_RESAMPLER=sinc -e DM_RS_HEADROOM=0.25 d-modem sh -c '/src/slmodemd/slmodemd_trnref -d9 -e /src/d-modem > /tmp/slm.log 2>&1'; sleep 12; docker exec -d d-modem sh -c 'socat TCP-LISTEN:5556,reuseaddr,fork FILE:/dev/ttySL0,raw,echo=0'" 2>/dev/null
  sleep 2
  at=$( (printf 'AT\r'; sleep 2) | nc -4 -w 5 tower.net.cryan.nz 5556 2>/dev/null )
  case "$at" in *OK*) return 0;; *) return 1;; esac
}

for attempt in $(seq 1 "$MAXATTEMPTS"); do
  echo "ATTEMPT $attempt: $(date -u +%H:%M:%SZ) bouncing rig"
  if ! bounce_rig; then
    echo "ATTEMPT $attempt: AT bridge dead; docker restart"
    ssh -o BatchMode=yes $TOWER "docker restart d-modem" >/dev/null 2>&1
    sleep 6
    bounce_rig || { echo "ATTEMPT $attempt: rig unrecoverable"; continue; }
  fi
  AD="$SOAKDIR/v2attempt$attempt"; mkdir -p "$AD"
  off=$(stat -f %z "$SERVERLOG" 2>/dev/null); off=${off:-0}
  python3 "$SP/soak_pty.py" "$AD" > "$AD/pty.log" 2>&1 &
  ptypid=$!
  sleep 2
  python3 "$SP/soak_sock.py" "$AD" > "$AD/sock.log" 2>&1 &
  sockpid=$!
  verdict=none
  for tick in $(seq 1 60); do
    sleep 5
    slice=$(tail -c "+$((off+1))" "$SERVERLOG" 2>/dev/null)
    if echo "$slice" | grep -aq "V.90 startup complete"; then verdict=DATA; break; fi
    if echo "$slice" | grep -aq "restarting Phase 2 (2,"; then verdict=FLIP_LOST; break; fi
    kill -0 $sockpid 2>/dev/null || { verdict=SOCK_EXIT; break; }
  done
  if [ "$verdict" = DATA ]; then
    echo "ATTEMPT $attempt: DATA MODE — letting pumps run the soak schedule"
    for i in $(seq 1 60); do kill -0 $sockpid 2>/dev/null || break; sleep 5; done
    for i in $(seq 1 12); do kill -0 $ptypid 2>/dev/null || break; sleep 5; done
    kill $ptypid $sockpid 2>/dev/null
    ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'cat /tmp/slm.log'" > "$AD/slm.log" 2>/dev/null
    sed 's/^/  sock: /' "$AD/sock.log" | tail -6
    sed 's/^/  pty:  /' "$AD/pty.log" | tail -6
    if grep -q "SOCK DONE" "$AD/sock.log" && grep -q "PTY DONE" "$AD/pty.log"; then
      echo "SOAK SUCCESS on attempt $attempt (data in $AD)"
      exit 0
    fi
    echo "ATTEMPT $attempt: reached data but soak incomplete; retrying"
  else
    echo "ATTEMPT $attempt: $verdict — aborting call early"
    kill $ptypid $sockpid 2>/dev/null
    ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'cat /tmp/slm.log'" > "$AD/slm.log" 2>/dev/null
  fi
  wait $ptypid $sockpid 2>/dev/null
done
echo "SOAK FAILED after $MAXATTEMPTS attempts"
exit 1
