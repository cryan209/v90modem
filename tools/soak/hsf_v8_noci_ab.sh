#!/bin/bash
# Live A/B on ME_V8_NO_CI over the HSF analogue coupler.
#
# The knob is on the CALLING (analogue) side: with CI suppressed the V.8 caller
# enters V8_AWAIT_ANSAM directly instead of running V8_WAIT_1S + the CI phase
# before it will send CM, which is what puts our CM 5.4 s into a 5.2 s
# answerer CM-wait.  Arms ALTERNATE so a drifting rig cannot masquerade as an
# effect, and the server is restarted per call so each gets its own G.711 taps.
set -u
OUT=${OUT:-artifacts/noci-ab-$(date -u +%H%M%SZ)}
REPEATS=${REPEATS:-3}
SECS=${SECS:-60}
SETTLE=${SETTLE:-20}
mkdir -p "$OUT"
for r in $(seq 1 "$REPEATS"); do
  for arm in noci ci; do
    v=0; [ "$arm" = noci ] && v=1
    RUN="$OUT/$arm-r$r"
    mkdir -p "$RUN"
    pkill -x sip_v90_modem 2>/dev/null; sleep 4
    VPCM_G711_TAP_DIR="$RUN" nohup ./sip_v90_modem --sip-server asterisk.net.cryan.nz \
        --username 6001 --password 6001 --pty-link /tmp/v90server > "$RUN/server.log" 2>&1 &
    sleep 6
    RUN="$RUN" SECS="$SECS" SETTLE="$SETTLE" EXTRA="ME_V8_NO_CI=$v" \
        bash tools/hsf_v34_call.sh > "$RUN/call.log" 2>&1
    echo "== $arm r$r done"
  done
done
pkill -x sip_v90_modem 2>/dev/null; sleep 3
nohup ./sip_v90_modem --sip-server asterisk.net.cryan.nz --username 6001 \
    --password 6001 --pty-link /tmp/v90server > /tmp/v90server.log 2>&1 &
echo "OUT=$OUT"
