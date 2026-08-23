#!/bin/bash
# One V.90 soak run with our server under test, reporting the upstream the
# call actually delivered.
#
# The upstream is the direction that matters here: it is V.34-modulated, so it
# goes through the same receiver as a plain V.34 call, and it is where the
# Phase 2 CC notch at 1200 Hz sits if nothing retires it.
#
#   v90_notch_ab.sh <outdir> [attempts]
set -u
OUT=${1:?usage: v90_notch_ab.sh outdir [attempts]}
ATTEMPTS=${2:-4}
SP="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SP/../.." && pwd)"

mkdir -p "$OUT"
LOG="$OUT/server.log"
: > "$LOG"
cd "$ROOT"

VPCM_G711_TAP_DIR="$OUT" VPCM_ME_VERBOSE=1 V34_DATA_FRAME_DUMP="$OUT/frames" \
env ${EXTRA_ENV:-} ./sip_v90_modem --sip-server asterisk.net.cryan.nz --username 6001 \
    --password 6001 --pty-link /tmp/modem0 >>"$LOG" 2>&1 &
srv=$!
trap 'kill $srv 2>/dev/null' EXIT
for i in $(seq 1 40); do grep -aq "modem ready" "$LOG" && break; sleep 1; done
grep -aq "modem ready" "$LOG" || { echo "CONTROL: server did not start"; exit 1; }
echo "CONTROL: server up"

SOAK_SOCK_ALWAYS=1 bash "$SP/soak_orchestrate2.sh" "$OUT" "$LOG" "$ATTEMPTS" 2>&1 | tail -14
rc=$?
kill $srv 2>/dev/null; wait $srv 2>/dev/null
trap - EXIT

echo "CONTROL: notch decisions"
grep -aE "Notch filter|notch" "$LOG" | sed 's/^/  /' | head -5
echo "CONTROL: upstream delivered to our PTY"
for d in "$OUT"/v2attempt*; do
    [ -f "$d/rx_pty.bin" ] || continue
    n=$(grep -aoE '^U[0-9]{7}$' "$d/rx_pty.bin" 2>/dev/null | wc -l | tr -d ' ')
    tot=$(wc -c < "$d/rx_pty.bin" | tr -d ' ')
    echo "  $(basename "$d"): $n intact U-lines out of $tot bytes"
done
exit $rc
