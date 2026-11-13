#!/bin/bash
# Rig-host half of courier_serial_call.py: our server on one test account for
# one call from a hardware modem, with taps.  Run from a worktree on a rig
# host (vmhost CT 117: /root/x2live), never with pkill -x: the server is
# stopped by its SIP port (rig_server.sh).
#
#   courier_serial_server.sh <name> <hold-seconds> [K=V ...]
#
# MODE (default x2) is ME_MODE; SRV_INIT the ;-separated AT commands before
# the call (default ATE0;AT&A3;AT+MS=X2;ATS0=1); LINES numbered lines sent
# after CONNECT (default 40), one every INTERVAL s (default 0.5).  Prints
# READY when registered, SRVCONNECT <line> at CONNECT, and the count of the
# caller's lines that arrived exactly.  Taps, RTP CSVs, server.log and
# pty-rx.bin land in artifacts/<name>/.  Touch /tmp/x2hw-stop to end early.
set -u
. "$(cd "$(dirname "$0")" && pwd)/rig_server.sh" || exit 2
NAME=${1:?usage: courier_serial_server.sh <name> <hold-seconds> [K=V ...]}
HOLD=${2:?hold seconds}; shift 2
USER_=${SRV_USER:-2902}; PORT=${SRV_PORT:-5098}; RTP=${SRV_RTP:-15400}
DIR=$PWD/artifacts/$NAME; mkdir -p "$DIR"
PTY=/tmp/x2hw-server
[ -n "$(rig_server_pids "$PORT")" ] && rig_server_kill "$PORT" && sleep 1
rm -f "$PTY" /tmp/x2hw-stop
env SIP_FORCE_PCMU=1 ME_MODE="${MODE:-x2}" VPCM_G711_TAP_DIR="$DIR" VPCM_ME_VERBOSE=1 "$@" \
  timeout $((HOLD + 150)) ./sip_v90_modem --sip-server asterisk.net.cryan.nz \
    --username "$USER_" --password "$USER_" --local-port "$PORT" --rtp-port "$RTP" \
    --pty-link "$PTY" --verbose > "$DIR/server.log" 2>&1 &
srv=$!
for _ in $(seq 1 30); do sleep 1; [ -e "$PTY" ] && break; done
[ -e "$PTY" ] || { echo "no pty"; kill $srv; exit 1; }
sleep 2
stty -F "$PTY" raw -echo
cat "$PTY" > "$DIR/pty-rx.bin" & catp=$!
IFS=';' read -ra cmds <<< "${SRV_INIT:-ATE0;AT&A3;AT+MS=X2;ATS0=1}"
for c in "${cmds[@]}"; do printf '%s\r' "$c" > "$PTY"; sleep 0.5; done
echo READY
i=0; t0=
while kill -0 $srv 2>/dev/null; do
    sleep "${INTERVAL:-0.5}"
    [ -e /tmp/x2hw-stop ] && break
    grep -aq CONNECT "$DIR/pty-rx.bin" || continue
    if [ -z "$t0" ]; then
        t0=$(date +%s)
        echo "SRVCONNECT $(grep -ao 'CONNECT[^\r]*' "$DIR/pty-rx.bin" | head -1)"
    fi
    if [ $i -lt "${LINES:-40}" ]; then
        printf 'srv %05d the quick brown fox jumps over the lazy dog 0123456789\r\n' $i > "$PTY"
        i=$((i + 1))
    fi
    [ $(( $(date +%s) - t0 )) -ge "$HOLD" ] && break
    grep -aq 'NO CARRIER' "$DIR/pty-rx.bin" && break
done
sleep 4
kill $srv 2>/dev/null; wait $srv 2>/dev/null; kill $catp 2>/dev/null
echo "server: $(tr -c '[:print:]\n' '.' < "$DIR/pty-rx.bin" | grep -ac '^cli [0-9]\{5\} the quick brown fox jumps over the lazy dog 0123456789') cli lines exact; $(grep -ac 'NO CARRIER' "$DIR/pty-rx.bin") NO CARRIER"
