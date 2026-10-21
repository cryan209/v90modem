#!/bin/bash
# One outbound MNP (V.42 Annex A) call to the RasFinder, MNP forced on our
# side (+ES=4,4: initiate the alternative procedure, LAPM not acceptable),
# with both error-control bit streams dumped for tools/mnp_frames.py.
#
#   rasfinder_mnp_call.sh <artifact-dir> [ext] [mode] [hold-seconds]
#
# ext defaults to 6311 (6311-6313 are the RasFinder lines), mode (ME_MODE) to
# v22, which V.8 makes V.22bis 2400.  After CONNECT the RasFinder prints a
# login banner and echoes what is typed at "login:", so PAYLOAD (printf %b
# escapes) goes up twice and a CR follows each time.  Its login field keeps
# 64 characters and drops the rest of that write, so a payload over 64
# characters with no backspaces does NOT come back whole; that is the
# application, not MNP (docs/mnp.md, "Live calls").  The default payload
# backspaces over itself so a 72-octet LT stays inside the field.
#
# The dumps contain whatever crosses the link: use test accounts only.  Leave
# 60-90 s between calls on one line.  Run from the repository root.
set -u
. "$(cd "$(dirname "$0")" && pwd)/rig_server.sh" || exit 2
DIR=${1:?usage: rasfinder_mnp_call.sh <artifact-dir> [ext] [mode] [hold-seconds]}
EXT=${2:-6311}
MODE=${3:-v22}
HOLD=${4:-50}
USER_EXT=${RF_USER:-2904}       # u-law account
PORT=${RF_PORT:-5096}
RTP=${RF_RTP:-15300}
PTY=${RF_PTY:-/tmp/v90mnp}
PAYLOAD=${PAYLOAD:-$(printf 'a%.0s' $(seq 40))$(printf '\\b%.0s' $(seq 20))$(printf 'b%.0s' $(seq 12))}

mkdir -p "$DIR"
LOG="$DIR/server.log"
if [ -n "$(rig_server_pids "$PORT")" ]; then
    echo "rasfinder_mnp_call: a server already holds port $PORT" >&2
    exit 1
fi

SIP_FORCE_PCMU=1 ME_MODE=$MODE \
DS_RX_BIT_DUMP="$DIR/rx-bits.txt" DS_TX_BIT_DUMP="$DIR/tx-bits.txt" \
VPCM_G711_TAP_DIR="$DIR" \
    ./sip_v90_modem --sip-server asterisk.net.cryan.nz \
        --username "$USER_EXT" --password "$USER_EXT" \
        --local-port "$PORT" --rtp-port "$RTP" \
        --pty-link "$PTY" --verbose >"$LOG" 2>&1 &
pid=$!
for _ in $(seq 1 30); do sleep 1; [ -e "$PTY" ] && break; done
if [ ! -e "$PTY" ]; then
    echo "rasfinder_mnp_call: $PTY never appeared; see $LOG" >&2
    kill "$pid" 2>/dev/null; wait "$pid" 2>/dev/null
    exit 1
fi
sleep 3
stty -F "$PTY" raw -echo 2>/dev/null
cat "$PTY" > "$DIR/pty-rx.bin" &
catpid=$!
for c in 'ATE0' 'AT+ES=4,4' 'AT&A3' 'AT+ER=1' 'ATX4'; do
    printf '%s\r' "$c" > "$PTY"
    sleep 1
done
printf 'ATD%s\r' "$EXT" > "$PTY"

n=0
for _ in $(seq 1 "$HOLD"); do
    sleep 1
    grep -aq CONNECT "$DIR/pty-rx.bin" 2>/dev/null || continue
    n=$((n + 1))
    case $n in
        5|25)  printf '%b' "$PAYLOAD" > "$PTY" ;;
        12|32) printf '\r' > "$PTY" ;;
    esac
done
kill "$pid" 2>/dev/null; wait "$pid" 2>/dev/null
kill "$catpid" 2>/dev/null; wait "$catpid" 2>/dev/null

echo "=== $DIR ==="
grep -aE "MNP|\+ER|CONNECT|NO CARRIER" "$LOG" | head -20
echo "--- PTY: $(wc -c < "$DIR/pty-rx.bin") bytes:"
tr -c '[:print:]\n' '.' < "$DIR/pty-rx.bin" | head -c 800; echo
echo "--- frames: tools/mnp_frames.py $DIR/tx-bits.txt 2400 (and rx-bits.txt)"
