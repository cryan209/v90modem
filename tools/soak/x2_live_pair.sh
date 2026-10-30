#!/bin/bash
# One live x2 call between two of our modems through Asterisk: our analogue
# client (AT+MS=X2 on a call we place) dials our digital server, both on the
# live SIP/RTP path, both PCMU.  Run from the repository root on a rig host
# (vmhost CT 117 or tower), never this Mac's Wi-Fi.
#
#   x2_live_pair.sh <dir> [hold-seconds]
#
# After CONNECT each side sends LINES (default 40) numbered lines, one every
# 0.5 s, and keeps what arrives in <dir>/{server,client}/pty-rx.bin.  Taps go
# to <dir>/{server,client}/ (VPCM_G711_TAP_DIR), so the server's transmitted
# codewords can be compared with the client's received ones: x2's downstream
# only survives a byte-exact bearer.
#
# Accounts 2902 (server) and 2903 (client), u-law test accounts; ports
# 5098/5100, RTP 15400/15500.  SRV_ENV / CLI_ENV: extra "K=V K=V" for each
# side.  CLI_INIT / SRV_INIT: ;-separated AT commands before the call.
# MODE=x2 (default; asymmetric, PCMU only) or x2-symm (symmetric 64000,
# either law).  LAW=alaw forces PCMA and defaults to the A-law accounts
# 2906/2907.  INTERVAL: seconds between lines (default 0.5).
set -u
. "$(cd "$(dirname "$0")" && pwd)/rig_server.sh" || exit 2
DIR=${1:?usage: x2_live_pair.sh <dir> [hold-seconds]}
HOLD=${2:-90}
LINES=${LINES:-40}
MODE=${MODE:-x2}; LAW=${LAW:-ulaw}; INTERVAL=${INTERVAL:-0.5}
case "$MODE" in x2) MS=X2 ;; x2-symm) MS=X2S ;; *) echo "MODE: x2 or x2-symm" >&2; exit 2 ;; esac
case "$LAW" in
    ulaw) FORCE=SIP_FORCE_PCMU=1; DEF_SRV=2902; DEF_CLI=2903 ;;
    alaw) FORCE=SIP_FORCE_PCMA=1; DEF_SRV=2906; DEF_CLI=2907 ;;
    *) echo "LAW: ulaw or alaw" >&2; exit 2 ;;
esac
SRV_USER=${SRV_USER:-$DEF_SRV}; SRV_PORT=${SRV_PORT:-5098}; SRV_RTP=${SRV_RTP:-15400}
CLI_USER=${CLI_USER:-$DEF_CLI}; CLI_PORT=${CLI_PORT:-5100}; CLI_RTP=${CLI_RTP:-15500}
SRV_INIT=${SRV_INIT:-ATE0;AT&A3;AT+MS=$MS;ATS0=1}
CLI_INIT=${CLI_INIT:-ATE0;AT&A3;AT+MS=$MS}
for p in "$SRV_PORT" "$CLI_PORT"; do
    if [ -n "$(rig_server_pids "$p")" ]; then
        echo "x2_live_pair: a server already holds port $p" >&2; exit 1
    fi
done
mkdir -p "$DIR/server" "$DIR/client"

start() {   # side user port rtp extra-env
    local side=$1 user=$2 port=$3 rtp=$4 extra=$5
    env $FORCE ME_MODE=$MODE VPCM_G711_TAP_DIR="$DIR/$side" $extra \
      ./sip_v90_modem --sip-server asterisk.net.cryan.nz \
        --username "$user" --password "$user" \
        --local-port "$port" --rtp-port "$rtp" \
        --pty-link "/tmp/x2live-$side" --verbose >"$DIR/$side/server.log" 2>&1 &
    echo $!
}
pty_up() {  # side init
    local side=$1 init=$2 pty=/tmp/x2live-$1
    for _ in $(seq 1 30); do sleep 1; [ -e "$pty" ] && break; done
    [ -e "$pty" ] || { echo "no pty for $side" >&2; return 1; }
    sleep 2
    stty -F "$pty" raw -echo 2>/dev/null
    cat "$pty" > "$DIR/$side/pty-rx.bin" &
    echo $! > "$DIR/$side/cat.pid"
    IFS=';' read -ra cmds <<< "$init"
    for c in "${cmds[@]}"; do printf '%s\r' "$c" > "$pty"; sleep 0.5; done
}
srv=$(start server "$SRV_USER" "$SRV_PORT" "$SRV_RTP" "${SRV_ENV:-}")
cli=$(start client "$CLI_USER" "$CLI_PORT" "$CLI_RTP" "${CLI_ENV:-}")
pty_up server "$SRV_INIT" && pty_up client "$CLI_INIT" || { kill $srv $cli; exit 1; }
sleep 3
printf 'ATD%s\r' "$SRV_USER" > /tmp/x2live-client
echo "dialled $SRV_USER from $CLI_USER"

sent_s=0; sent_c=0
end=$(( $(date +%s) + HOLD ))
while [ "$(date +%s)" -lt "$end" ]; do
    sleep "$INTERVAL"
    for side in server client; do
        grep -aq CONNECT "$DIR/$side/pty-rx.bin" 2>/dev/null || continue
        if [ $side = server ] && [ $sent_s -lt "$LINES" ]; then
            printf 'srv %05d the quick brown fox jumps over the lazy dog 0123456789\r\n' $sent_s > /tmp/x2live-server
            sent_s=$((sent_s + 1))
        elif [ $side = client ] && [ $sent_c -lt "$LINES" ]; then
            printf 'cli %05d the quick brown fox jumps over the lazy dog 0123456789\r\n' $sent_c > /tmp/x2live-client
            sent_c=$((sent_c + 1))
        fi
    done
done
kill $srv $cli 2>/dev/null; wait $srv $cli 2>/dev/null
for side in server client; do kill "$(cat "$DIR/$side/cat.pid")" 2>/dev/null; done

for side in server client; do
    peer=$([ $side = server ] && echo cli || echo srv)
    got=$(tr -c '[:print:]\n' '.' < "$DIR/$side/pty-rx.bin" | grep -a "^$peer [0-9]\{5\} the quick brown fox jumps over the lazy dog 0123456789" | sort -u | wc -l)
    echo "$side: $(grep -ao 'CONNECT[^\r]*' "$DIR/$side/pty-rx.bin" | head -1); $got of $LINES '$peer' lines exact; $(grep -ac 'NO CARRIER' "$DIR/$side/pty-rx.bin") NO CARRIER"
done
