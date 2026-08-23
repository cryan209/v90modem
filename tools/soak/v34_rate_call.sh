#!/bin/bash
# One plain-V.34 call at a chosen symbol rate and bit rate, held long enough
# for V.42 LAPM and a numbered payload run, with the G.711 taps and the
# data-mode symbol dump kept.
#
# The point of the knobs is the open question in docs/v34_data_mode_rates.md:
# nothing before data mode measures what this direction will carry, so the
# only way to find out what a line carries is to ask for it and look.
#
#   v34_rate_call.sh <outdir> [baud] [bps] [hold-seconds]
set -u
OUT=${1:?usage: v34_rate_call.sh outdir [baud] [bps] [hold]}
BAUD=${2:-3000}
BPS=${3:-14400}
HOLD=${4:-90}
SP="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SP/../.." && pwd)"

mkdir -p "$OUT"
LOG="$OUT/server.log"
: > "$LOG"

cd "$ROOT"
VPCM_G711_TAP_DIR="$OUT" VPCM_ME_VERBOSE=1 ME_V34_SPAN_FLOW_LOG=1 \
ME_MODE=v34 ME_DATA_FRAMING=lapm \
ME_V34_BAUD="$BAUD" ME_V34_BPS="$BPS" \
V34_DATA_FRAME_DUMP="$OUT/frames" \
env ${EXTRA_ENV:-} ./sip_v90_modem --sip-server asterisk.net.cryan.nz --username 6001 \
    --password 6001 --pty-link /tmp/v90modem >>"$LOG" 2>&1 &
srv=$!
trap 'kill $srv 2>/dev/null' EXIT

for i in $(seq 1 40); do
    grep -aq "modem ready" "$LOG" && break
    sleep 1
done
grep -aq "modem ready" "$LOG" || { echo "CONTROL: server did not start"; exit 1; }
echo "CONTROL: server up, $BAUD baud / $BPS bps"

python3 "$SP/v34_rate_pty.py" "$OUT" &
pty=$!

KEEP=1 NPARM='\\N3' bash "$SP/v34_lapm_call.sh" "$LOG" "$HOLD" "$OUT/call.out"
rc=$?
kill $pty 2>/dev/null; wait $pty 2>/dev/null
sleep 1
kill $srv 2>/dev/null; wait $srv 2>/dev/null
trap - EXIT

echo "CONTROL: rate-request log"
grep -a "measured probe\|measured selection\|Phase-4 TRN SNR\|Tx MP receive-rate\|DATA mode: parms" "$LOG" | tail -12
echo "CONTROL: peer rate choice"
grep -aE "finally txbitrate|Final choice data rate" "$OUT/call.peer.log" 2>/dev/null | tail -3
echo "CONTROL: lines the peer's DTE received: $(grep -ac '^S[0-9]' "$OUT/call.out" 2>/dev/null)"
exit $rc
