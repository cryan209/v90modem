#!/bin/bash
# One outbound call to the RasFinder analogue RAS hunt group 3999, placed by
# a dedicated server instance so the long-lived 6001 server on the default
# port is left alone.
#
#   rasfinder_call.sh <artifact-dir> [hold-seconds]
#
# Environment starting with ME_/V34_/V90_/VPCM_ is passed through to the
# server, so an A/B is "the same command twice with one variable moved".
#
# The RasFinder rejects close redials: leave 60-90 s between calls.
set -u
DIR=${1:?usage: rasfinder_call.sh <artifact-dir> [hold-seconds]}
HOLD=${2:-45}
EXT=${RF_EXT:-3999}
PTY=${RF_PTY:-/tmp/v90rf}
PORT=${RF_PORT:-5072}
RTP=${RF_RTP:-14000}

mkdir -p "$DIR"
LOG="$DIR/server.log"

# A stray server holding the SIP port makes every call die with a short log and
# no call at all, which reads exactly like the rig refusing to connect.  Note
# pgrep -f would match this script's own command line.
if pgrep -x sip_v90_modem >/dev/null; then
    if pgrep -fl "local-port $PORT" >/dev/null 2>&1; then
        echo "rasfinder_call: a server already holds port $PORT" >&2
        exit 1
    fi
fi

# XID compatibility is detected from the peer; 3/4 remain diagnostic overrides.
ME_LAPM_XID_OPTION_OCTETS=${ME_LAPM_XID_OPTION_OCTETS:-auto} \
    VPCM_G711_TAP_DIR="$DIR" SIP_FORCE_PCMU=${SIP_FORCE_PCMU:-1} \
    ./sip_v90_modem --sip-server asterisk.net.cryan.nz \
        --username 6001 --password 6001 \
        --local-port "$PORT" --rtp-port "$RTP" \
        --pty-link "$PTY" --verbose >"$LOG" 2>&1 &
pid=$!

for _ in $(seq 1 30); do
    sleep 1
    [ -e "$PTY" ] && break
done
if [ ! -e "$PTY" ]; then
    echo "rasfinder_call: $PTY never appeared; see $LOG" >&2
    kill "$pid" 2>/dev/null; wait "$pid" 2>/dev/null
    exit 1
fi
sleep 3

# Whatever the RasFinder sends once connected (a login prompt or PPP frames)
# arrives on the PTY; keep it, since a data mode that decodes is proved by the
# bytes, not by our own receiver's opinion of itself.
stty -F "$PTY" raw -echo 2>/dev/null
cat "$PTY" > "$DIR/pty-rx.bin" &
catpid=$!
printf 'ATD%s\r' "$EXT" > "$PTY"
sleep "$HOLD"

kill "$pid" 2>/dev/null
wait "$pid" 2>/dev/null
kill "$catpid" 2>/dev/null
wait "$catpid" 2>/dev/null

echo "=== $DIR ==="
grep -aE "V\.8 |negotiated|Phase 3|Phase 4|data mode|CONNECT|NO CARRIER|retrain|fall" "$LOG" | tail -40
if [ -s "$DIR/pty-rx.bin" ]; then
    echo "--- PTY: $(wc -c < "$DIR/pty-rx.bin") bytes; printable excerpt:"
    tr -c '[:print:]\n' '.' < "$DIR/pty-rx.bin" | head -c 600; echo
fi
