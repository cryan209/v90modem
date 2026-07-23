#!/bin/bash
# Batch V.90 calls against the tower rig to validate:
#  (a) the Phase 2 tone-sequencing rework (8b06612) — completion rate,
#      no "aborting after N INFO1a" families
#  (b) the MP upstream rate cap (8ae49e1) — "[V90] MP upstream rate cap"
#      + peer Tx bit rate == cap + data-mode hold time
# Runs ALL calls (no early exit) so we get reliability stats.
TOWER=root@tower.net.cryan.nz
TAPDIR="$1"
CALLS="${2:-6}"
SERVERLOG="$3"

restart_rig() {
  ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'for d in /proc/[0-9]*; do cmd=\$(tr \"\\0\" \" \" < \$d/cmdline 2>/dev/null); case \"\$cmd\" in *slmodemd*|*socat*) kill \${d#/proc/} 2>/dev/null;; esac; done; sleep 1; rm -f /tmp/smartlink-trn2d-pairs.s16 /tmp/slm.log; true'; docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz -e DM_RESAMPLER=sinc -e DM_RS_HEADROOM=0.25 d-modem sh -c '/src/slmodemd/slmodemd_trnref -d9 -e /src/d-modem > /tmp/slm.log 2>&1'; sleep 3; docker exec -d d-modem sh -c 'socat TCP-LISTEN:5556,reuseaddr,fork FILE:/dev/ttySL0,raw,echo=0'" 2>/dev/null
  sleep 2
  at=$( (printf 'AT\r'; sleep 2) | nc -4 -w 5 tower.net.cryan.nz 5556 2>/dev/null )
  case "$at" in *OK*) return 0;; *) return 1;; esac
}

peer_grep() { ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'grep -ac \"$1\" /tmp/slm.log 2>/dev/null'" 2>/dev/null; }

collect_call() {
  local n="$1" off="$2"
  ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'cat /tmp/slm.log'" > "$TAPDIR/slm-call$n.log" 2>/dev/null
  tail -c "+$((off+1))" "$SERVERLOG" > "$TAPDIR/server-call$n.log" 2>/dev/null
  if ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'test -s /tmp/smartlink-trn2d-pairs.s16'" 2>/dev/null; then
    ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'cat /tmp/smartlink-trn2d-pairs.s16'" > "$TAPDIR/pairs-call$n.s16" 2>/dev/null
  fi
}

for call in $(seq 1 "$CALLS"); do
  echo "=== call $call: $(date -u +%H:%M:%SZ) restarting rig"
  if ! restart_rig; then echo "call $call: AT bridge dead after restart; aborting"; exit 1; fi
  off=$(stat -f %z "$SERVERLOG" 2>/dev/null); off=${off:-0}
  (printf 'ATZ\r'; sleep 2; printf 'ATX3\r'; sleep 2; printf 'AT+MS=90,1,300,56000\r'; sleep 2; printf 'ATD6001\r'; sleep 500) | nc -4 -w 520 tower.net.cryan.nz 5556 > /dev/null 2>&1 &
  ncpid=$!
  outcome=none data_at=0
  for tick in $(seq 1 50); do
    sleep 6
    slice=$(tail -c "+$((off+1))" "$SERVERLOG" 2>/dev/null)
    if echo "$slice" | grep -aq "V.90 startup complete"; then outcome=DATA; data_at=$((tick*6)); break; fi
    noc=$(peer_grep "NO CARRIER"); if [ "${noc:-0}" -gt 0 ] 2>/dev/null; then outcome=NOCARRIER; break; fi
  done
  hold=n/a
  if [ "$outcome" = DATA ]; then
    echo "call $call: DATA MODE at ~${data_at}s — measuring hold"
    for htick in $(seq 1 24); do
      sleep 10
      le=$(peer_grep "Link Error"); noc=$(peer_grep "NO CARRIER")
      if [ "${le:-0}" -gt 0 ] 2>/dev/null || [ "${noc:-0}" -gt 0 ] 2>/dev/null; then hold="~$((htick*10))s"; break; fi
    done
    [ "$hold" = n/a ] && hold=">240s (still up)"
  fi
  kill $ncpid 2>/dev/null; wait $ncpid 2>/dev/null
  sleep 2
  slice=$(tail -c "+$((off+1))" "$SERVERLOG" 2>/dev/null)
  cap=$(echo "$slice" | grep -ao "MP upstream rate cap: [0-9]*" | tail -1)
  p2abort=$(echo "$slice" | grep -ac "aborting after"); info1a=$(echo "$slice" | grep -ac "INFO1a")
  startup=$(echo "$slice" | grep -a "V.90 startup complete" | tail -1 | sed 's/.*\[ME\] //')
  peertx=$(ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'grep -a \"Tx bit rate\" /tmp/slm.log 2>/dev/null | tail -1'" 2>/dev/null)
  summary=$(ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'grep -ac \"drop to V34\" /tmp/slm.log; grep -ac \"retrain requested\" /tmp/slm.log; grep -ac \"Building CPt\" /tmp/slm.log; grep -ac \"enter Data Phase\" /tmp/slm.log; grep -ac delayedRetrainRequest /tmp/slm.log' 2>/dev/null" 2>/dev/null | tr '\n' ' ')
  echo "call $call: outcome=$outcome hold=$hold p2aborts=$p2abort info1a_lines=$info1a drops/retrains/cpt/dataphase/delayed = $summary"
  [ -n "$cap" ] && echo "call $call: RATE CAP: $cap"
  [ -n "$startup" ] && echo "call $call: $startup"
  [ -n "$peertx" ] && echo "call $call: peer: $peertx"
  collect_call "$call" "$off"
done
echo "BATCH DONE ($CALLS calls, artifacts in $TAPDIR)"
