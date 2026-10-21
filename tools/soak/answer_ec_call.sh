#!/bin/bash
# Answer one incoming call with error-control bit dumps and grade it: after
# CONNECT, send OURS_LINES (default 20) numbered lines to the caller, one a
# second, and keep what arrives in <dir>/pty-rx.bin.  The partner of
# tools/cx_hold_dial.py (the CX93001 dialling in, docs/mnp.md).
#
#   answer_ec_call.sh <dir> [mode] ["<AT init, ;-separated>"] [hold-seconds]
#
# Defaults: ME_MODE v22 (V.22bis), init "ATE0;AT&A3;AT+ER=1;ATS0=1" (factory
# +ES: V.42 detection, then MNP, then V.14), 100 s.  Registers as RF_USER
# (2904) on RF_PORT (5096); run from the repository root.  Test accounts only.
set -u
. "$(cd "$(dirname "$0")" && pwd)/rig_server.sh" || exit 2
DIR=${1:?usage: answer_ec_call.sh <dir> [mode] [init] [hold]}
MODE=${2:-v22}; INIT=${3:-ATE0;AT&A3;AT+ER=1;ATS0=1}; HOLD=${4:-100}
PORT=${RF_PORT:-5096}; RTP=${RF_RTP:-15300}; USER_EXT=${RF_USER:-2904}
PTY=${RF_PTY:-/tmp/v90mnp}
mkdir -p "$DIR"
if [ -n "$(rig_server_pids "$PORT")" ]; then
    echo "answer_ec_call: a server already holds port $PORT" >&2
    exit 1
fi
SIP_FORCE_PCMU=1 ME_MODE=$MODE \
DS_RX_BIT_DUMP="$DIR/rx-bits.txt" DS_TX_BIT_DUMP="$DIR/tx-bits.txt" \
VPCM_G711_TAP_DIR="$DIR" \
  ./sip_v90_modem --sip-server asterisk.net.cryan.nz \
    --username "$USER_EXT" --password "$USER_EXT" \
    --local-port "$PORT" --rtp-port "$RTP" \
    --pty-link "$PTY" --verbose >"$DIR/server.log" 2>&1 &
pid=$!
for _ in $(seq 1 30); do sleep 1; [ -e "$PTY" ] && break; done
[ -e "$PTY" ] || { echo "no pty"; kill $pid; exit 1; }
sleep 2
stty -F "$PTY" raw -echo 2>/dev/null
cat "$PTY" > "$DIR/pty-rx.bin" &
catpid=$!
IFS=';' read -ra CMDS <<< "$INIT"
for c in "${CMDS[@]}"; do printf '%s\r' "$c" > "$PTY"; sleep 0.5; done
echo READY
c=0
for i in $(seq 1 "$HOLD"); do
  sleep 1
  if grep -aq CONNECT "$DIR/pty-rx.bin" 2>/dev/null; then
    c=$((c+1))
    if [ $c -ge 4 ] && [ $c -le $((3 + ${OURS_LINES:-20})) ]; then
      printf 'ours %02d the quick brown fox jumps over the lazy dog 0123456789\r\n' $((c-3)) > "$PTY"
    fi
  fi
done
kill $pid 2>/dev/null; wait $pid 2>/dev/null
kill $catpid 2>/dev/null; wait $catpid 2>/dev/null
grep -aE "MNP|V\.42|LAPM|CONNECT|NO CARRIER|V8 selected|training complete|detection" "$DIR/server.log" | grep -v "RX dump" | head -30
echo "--- PTY $(wc -c < "$DIR/pty-rx.bin") bytes:"
tr -c '[:print:]\n' '.' < "$DIR/pty-rx.bin" | head -c 2500; echo
