#!/bin/bash
# wecall_live.sh <dir> <AT+MS=...> [number]: one live call from our modem to
# a modem that answers on SIP (default 2902: tools/soak/half_sip_call.py
# --answer with D-Modem's emulated Courier, from vmhost CT 128).  Run on a
# rig host (vmhost CT 117), never this Mac's Wi-Fi.
#
# Our modem is account 2903 (u-law test account), SIP port 5100, RTP 15500,
# PCMU, PTY /tmp/wecall-<pid>.  BIN: the sip_v90_modem to run (default
# ./sip_v90_modem).  After CONNECT it sends LINES (default 40) 'cli' lines,
# one every INTERVAL (0.5 s), and counts the far end's 'srv' lines, as
# x2_live_pair.sh does.  Taps go to <dir> (VPCM_G711_TAP_DIR).
set -u
DIR=${1:?usage: wecall_live.sh <dir> <AT+MS=...> [number]}
MS=${2:?AT+MS command}
NUM=${3:-2902}
BIN=${BIN:-./sip_v90_modem}
LINES=${LINES:-40}; INTERVAL=${INTERVAL:-0.5}; HOLD=${HOLD:-15}
PORT=5100; RTP=15500; PTY=/tmp/wecall-$$
if ss -lun | grep -q ":$PORT "; then echo "wecall_live: port $PORT in use" >&2; exit 1; fi
mkdir -p "$DIR"
env SIP_FORCE_PCMU=1 VPCM_G711_TAP_DIR="$DIR" ${CLI_ENV:-} \
  "$BIN" --sip-server asterisk.net.cryan.nz --username 2903 --password 2903 \
    --local-port $PORT --rtp-port $RTP --pty-link "$PTY" --verbose > "$DIR/server.log" 2>&1 &
pid=$!
trap 'kill $pid 2>/dev/null; kill $cat 2>/dev/null; rm -f "$PTY"' EXIT
for _ in $(seq 1 30); do sleep 1; [ -e "$PTY" ] && break; done
[ -e "$PTY" ] || { echo "no pty" >&2; exit 1; }
sleep 2
stty -F "$PTY" raw -echo 2>/dev/null
cat "$PTY" > "$DIR/pty-rx.bin" & cat=$!
for c in ATE0 "$MS"; do printf '%s\r' "$c" > "$PTY"; sleep 0.5; done
sleep 1
printf 'ATD%s\r' "$NUM" > "$PTY"
t0=$(date +%s)
until grep -aqE 'CONNECT|NO CARRIER|BUSY|ERROR' "$DIR/pty-rx.bin" 2>/dev/null; do
    sleep 0.2; [ $(( $(date +%s) - t0 )) -gt 90 ] && break
done
connect=$(grep -aoE 'CONNECT[^\r]*' "$DIR/pty-rx.bin" | head -1)
if [ -n "$connect" ]; then
    for i in $(seq 0 $((LINES - 1))); do
        printf 'cli %05d the quick brown fox jumps over the lazy dog 0123456789\r\n' "$i" > "$PTY"
        sleep "$INTERVAL"
    done
    end=$(( $(date +%s) + HOLD ))
    while [ "$(date +%s)" -lt "$end" ]; do
        n=$(tr -c '[:print:]\n' '.' < "$DIR/pty-rx.bin" | grep -a "^srv [0-9]\{5\} the quick brown fox jumps over the lazy dog 0123456789" | sort -u | wc -l)
        [ "$n" -ge "$LINES" ] && break
        sleep 1
    done
fi
sleep 3
got=$(tr -c '[:print:]\n' '.' < "$DIR/pty-rx.bin" | grep -a "^srv [0-9]\{5\} the quick brown fox jumps over the lazy dog 0123456789" | sort -u | wc -l)
nc=$(grep -ac 'NO CARRIER' "$DIR/pty-rx.bin")
rate=$(grep -aoE 'V\.(32bis|22bis) (start-up complete|training complete)[^)]*\)|retrain[^,]*complete[^\n]*|reception lost[^\n]*' "$DIR/server.log" | sort | uniq -c | tr '\n' ';')
echo "ours: ${connect:-no CONNECT}; $got of $LINES 'srv' lines exact; $nc NO CARRIER; $rate"
