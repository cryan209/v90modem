#!/bin/bash
# Batch V.90 capture calls against the tower rig; stop on first TRN2REF.
TOWER=root@tower.net.cryan.nz

restart_rig() {
  ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'for d in /proc/[0-9]*; do cmd=\$(tr \"\\0\" \" \" < \$d/cmdline 2>/dev/null); case \"\$cmd\" in *slmodemd*|*socat*) kill \${d#/proc/} 2>/dev/null;; esac; done; sleep 1; true'; docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz -e DM_RESAMPLER=sinc -e DM_RS_HEADROOM=0.25 d-modem sh -c '/src/slmodemd/slmodemd_trnref -d9 -e /src/d-modem > /tmp/slm.log 2>&1'; sleep 3; docker exec -d d-modem sh -c 'socat TCP-LISTEN:5556,reuseaddr,fork FILE:/dev/ttySL0,raw,echo=0'" 2>/dev/null
  sleep 2
  at=$( (printf 'AT\r'; sleep 2) | nc -w 5 tower.net.cryan.nz 5556 2>/dev/null )
  case "$at" in *OK*) return 0;; *) return 1;; esac
}

for call in 1 2 3 4; do
  if ! restart_rig; then echo "call $call: AT bridge dead after restart; aborting"; exit 1; fi
  (printf 'ATZ\r'; sleep 2; printf 'ATX3\r'; sleep 2; printf 'AT+MS=90,1,300,56000\r'; sleep 2; printf 'ATD6001\r'; sleep 200) | nc -w 220 tower.net.cryan.nz 5556 > /dev/null 2>&1 &
  ncpid=$!
  # poll for outcome: TRN2REF (success) or NO CARRIER (attempt over)
  outcome=none
  for tick in $(seq 1 40); do
    sleep 6
    st=$(ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'grep -ac TRN2REF /tmp/slm.log 2>/dev/null; grep -ac \"NO CARRIER\" /tmp/slm.log 2>/dev/null'" 2>/dev/null)
    trn=$(echo "$st" | sed -n 1p); noc=$(echo "$st" | sed -n 2p)
    if [ "${trn:-0}" -gt 0 ] 2>/dev/null; then outcome=TRN2REF; break; fi
    if [ "${noc:-0}" -gt 0 ] 2>/dev/null; then outcome=NOCARRIER; break; fi
  done
  kill $ncpid 2>/dev/null
  drops=$(ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'grep -ac \"drop to V34\" /tmp/slm.log 2>/dev/null; grep -ac \"retrain requested\" /tmp/slm.log 2>/dev/null'" 2>/dev/null | tr '\n' '/')
  echo "call $call: outcome=$outcome drops/retrains=$drops"
  if [ "$outcome" = TRN2REF ]; then
    echo "SUCCESS: hook firing - leave rig up, collect /tmp/smartlink-trn2d-pairs.s16"
    # let the call keep running to accumulate pairs
    sleep 60
    ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'grep -a TRN2REF /tmp/slm.log | tail -3; ls -la /tmp/smartlink-trn2d-pairs.s16'" 2>/dev/null
    exit 0
  fi
done
echo "BATCH DONE: no Phase 4 in 4 calls"
