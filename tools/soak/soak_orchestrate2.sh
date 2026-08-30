#!/bin/bash
# Soak orchestrator v2: early-aborts a call as soon as the first-attempt
# Phase 4 MP coin flip is lost (server logs "restarting Phase 2 (2,"),
# since retrained attempts never reach data mode on this rig. Redials
# until an initial attempt survives to DATA, then lets the pumps run.
# Set SLMODEM_DIAG=1 to run the peer at -d9 -l11 and preserve its framed
# sample/bit log, TRN2 interposer pairs, and bridge PCM taps with the attempt.
SP="$(cd "$(dirname "$0")" && pwd)"
SOAKDIR="$1"
SERVERLOG="$2"
TOWER=root@tower.net.cryan.nz
MAXATTEMPTS="${3:-8}"
SLMODEM_DIAG="${SLMODEM_DIAG:-0}"

collect_peer_diag() {
  ad="$1"
  ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'cat /tmp/slm.log'" > "$ad/slm.log" 2>/dev/null
  [ "$SLMODEM_DIAG" = 1 ] || return 0

  # These are diagnostic-only files with explicit, bounded names.  The
  # slmodemd_trnref interposer supplies the received/reference symbol pairs;
  # -l11 supplies SmartLink's framed RX/TX samples, bits, data and characters.
  # The bridge taps bracket the 8 kHz <-> 9.6 kHz conversion independently.
  for spec in \
      smartlink-trn2d-pairs.s16:pairs.s16 \
      slmodem.log.slamr0:slmodem.log.slamr0 \
      dm_to_dsp.raw:dm_to_dsp.raw \
      dm_to_dsp_9600.raw:dm_to_dsp_9600.raw \
      dm_from_dsp.raw:dm_from_dsp.raw \
      dm_from_dsp_9600.raw:dm_from_dsp_9600.raw; do
    remote=${spec%%:*}; local_name=${spec#*:}
    if ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'test -s /tmp/$remote'" 2>/dev/null; then
      ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'cat /tmp/$remote'" \
        > "$ad/$local_name" 2>/dev/null
    fi
  done
}

bounce_rig() {
  # Restart the container outright between attempts.  Killing slmodemd alone
  # left per-call state behind (and a survivor would re-create /dev/ttySL0,
  # after which two instances fight over the dial: "Dialer was aborted" ->
  # instant NO CARRIER on every redial).  A restart is a few seconds and
  # removes the whole class.
  ssh -o BatchMode=yes $TOWER "docker restart d-modem" >/dev/null 2>&1
  sleep 8
  if [ "$SLMODEM_DIAG" = 1 ]; then
    ssh -o BatchMode=yes $TOWER "docker exec d-modem sh -c 'rm -f /tmp/slm.log /tmp/smartlink-trn2d-pairs.s16 /tmp/slmodem.log.slamr0 /tmp/dm_to_dsp.raw /tmp/dm_to_dsp_9600.raw /tmp/dm_from_dsp.raw /tmp/dm_from_dsp_9600.raw'; docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz -e DM_RESAMPLER=sinc -e DM_RS_HEADROOM=0.25 d-modem sh -c 'cd /tmp && /src/slmodemd/slmodemd_trnref -d9 -l11 -e /src/d-modem > /tmp/slm.log 2>&1'; sleep 12; docker exec -d d-modem sh -c 'socat TCP-LISTEN:5556,reuseaddr,fork FILE:/dev/ttySL0,raw,echo=0'" 2>/dev/null
  else
    ssh -o BatchMode=yes $TOWER "docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz -e DM_RESAMPLER=sinc -e DM_RS_HEADROOM=0.25 d-modem sh -c '/src/slmodemd/slmodemd_trnref -d9 -e /src/d-modem > /tmp/slm.log 2>&1'; sleep 12; docker exec -d d-modem sh -c 'socat TCP-LISTEN:5556,reuseaddr,fork FILE:/dev/ttySL0,raw,echo=0'" 2>/dev/null
  fi
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
  # How long to wait for a call to announce data mode before giving up on it.
  # A winning call gets there in about a minute, so the old flat 300 s spent
  # four of every five minutes waiting on calls that had already lost -- which
  # matters when reaching data mode is a lottery and the batch is the ticket
  # supply.  SOAK_WATCH_SEC overrides; the default is unchanged.
  for tick in $(seq 1 $(( ${SOAK_WATCH_SEC:-300} / 5 )) ); do
    sleep 5
    slice=$(tail -c "+$((off+1))" "$SERVERLOG" 2>/dev/null)
    if echo "$slice" | grep -aq "V.90 startup complete"; then verdict=DATA; break; fi
    # Give a call several retrains before abandoning it.  Aborting at the
    # first one dates from July, when a retrained attempt never reached data
    # mode; that is worth re-testing rather than assuming, and the peer's own
    # retrain counter reaches at least 4 on this rig.
    if echo "$slice" | grep -aq "restarting Phase 2 (${MAXRETRAIN:-5},"; then verdict=FLIP_LOST; break; fi
    kill -0 $sockpid 2>/dev/null || { verdict=SOCK_EXIT; break; }
  done
  if [ "$verdict" = DATA ]; then
    echo "ATTEMPT $attempt: DATA MODE — letting pumps run the soak schedule"
    for i in $(seq 1 $(( ${SOAK_SECONDS:-105} / 5 + 30 )) ); do kill -0 $sockpid 2>/dev/null || break; sleep 5; done
    for i in $(seq 1 12); do kill -0 $ptypid 2>/dev/null || break; sleep 5; done
    kill $ptypid $sockpid 2>/dev/null
    collect_peer_diag "$AD"
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
    collect_peer_diag "$AD"
  fi
  wait $ptypid $sockpid 2>/dev/null
done
echo "SOAK FAILED after $MAXATTEMPTS attempts"
exit 1
