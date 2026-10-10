#!/bin/bash
# Place one call to the RasFinder and HOLD it with the PTY left free, so a
# person can attach and log in, and transfers (ZMODEM etc.) can then be run
# against the same PTY.  Run inside the v90modem-sip container.
#
#   rasfinder_session.sh <artifact-dir> [max-seconds]
#
# The session ends when the call disconnects, <artifact-dir>/stop exists, the
# server exits, or max-seconds (default 1800) passes.  Attach, from a host with ssh to tower:
#
#   ssh -t root@tower.net.cryan.nz docker exec -it v90modem-sip \
#       socat -,raw,echo=0,escape=0x1d /tmp/v90rf,raw,echo=0
#
# (Ctrl-] detaches and leaves the call up.)  Environment as rasfinder_call.sh:
# RF_EXT (default 3999, the RasFinder hunt group), RF_USER, RF_PTY; ME_/V34_/V90_/V42_ pass through.
set -u
. "$(cd "$(dirname "$0")" && pwd)/rig_server.sh" || exit 2
DIR=${1:?usage: rasfinder_session.sh <artifact-dir> [max-seconds]}
MAX=${2:-1800}
EXT=${RF_EXT:-3999}
USER_EXT=${RF_USER:-2900}
PASS=${RF_PASS:-$USER_EXT}
PTY=${RF_PTY:-/tmp/v90rf}
PORT=${RF_PORT:-5072}
RTP=${RF_RTP:-14000}

mkdir -p "$DIR"
rm -f "$DIR/stop"
LOG="$DIR/server.log"
if [ -z "${SIP_FORCE_PCMU:-}" ] && [ -z "${SIP_FORCE_PCMA:-}" ]; then
    case "$USER_EXT" in
        290[5-9]) export SIP_FORCE_PCMA=1 ;;
        *)        export SIP_FORCE_PCMU=1 ;;
    esac
fi
if [ -n "$(rig_server_pids "$PORT")" ]; then
    echo "rasfinder_session: a server already holds port $PORT" >&2
    exit 1
fi

ME_LAPM_XID_OPTION_OCTETS=${ME_LAPM_XID_OPTION_OCTETS:-auto} \
    VPCM_G711_TAP_DIR="$DIR" \
    ./sip_v90_modem --sip-server asterisk.net.cryan.nz \
        --username "$USER_EXT" --password "$PASS" \
        --local-port "$PORT" --rtp-port "$RTP" \
        --pty-link "$PTY" --verbose >"$LOG" 2>&1 &
pid=$!
for _ in $(seq 1 30); do
    sleep 1
    [ -e "$PTY" ] && break
done
if [ ! -e "$PTY" ]; then
    echo "rasfinder_session: $PTY never appeared; see $LOG" >&2
    kill "$pid" 2>/dev/null; wait "$pid" 2>/dev/null
    exit 1
fi
sleep 3
stty -F "$PTY" raw -echo 2>/dev/null
printf 'ATD%s\r' "$EXT" > "$PTY"
echo "rasfinder_session: dialled $EXT, pid $pid; touch $DIR/stop to end"

end=$(( $(date +%s) + MAX ))
while kill -0 "$pid" 2>/dev/null && [ ! -e "$DIR/stop" ] && [ "$(date +%s)" -lt "$end" ] \
      && ! grep -q "SIP disconnected" "$LOG" 2>/dev/null; do
    sleep 1
done
kill "$pid" 2>/dev/null
wait "$pid" 2>/dev/null
echo "rasfinder_session: ended"
