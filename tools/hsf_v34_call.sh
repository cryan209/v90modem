#!/bin/bash
# One HSF-analogue call into the digital side, with a DTE traffic grader on
# both PTYs.  The receive sampling phase is the coupler's HSF_RX_DELAY; live it
# is effectively a lottery per Phase 3 attempt, so this is meant to be repeated.
set -u
RUN=${RUN:-artifacts/hsf-v90/call-$(date -u +%H%M%SZ)}
SECS=${SECS:-90}
mkdir -p "$RUN"
rm -f /tmp/v90hsf
# The ATA/PBX needs the port idle for a while or the next dial gets
# number-unobtainable instead of ringing.
sleep "${SETTLE:-25}"
env HSF_CALL_INIT=1 HSF_DIAL_DELAY_MS=0 HSF_DIAL_TONE_MS=200 HSF_DIAL_GAP_MS=150 \
    ${MODE:+ME_MODE=$MODE} ${PHASE:+HSF_RX_DELAY=$PHASE} \
    ${STEP:+HSF_RX_DELAY_STEP=$STEP} ${PERIOD:+HSF_RX_DELAY_PERIOD_MS=$PERIOD} \
    ${STIMEOUT:+ME_V34_PHASE3_S_TIMEOUT_MS=$STIMEOUT} ${TXGAIN:+HSF_TX_GAIN=$TXGAIN} ${EXTRA:-} \
    ./hsf_v90_coupler ${WAIT:+--wait $WAIT} --call-seq --v90-couple --pty-link /tmp/v90hsf \
    --dial "${DIAL:-6001}" --dial-amp 20000 --stream "$SECS" \
    --rx-out "$RUN/hsf-rx.raw" --tx-out "$RUN/hsf-tx.raw" > "$RUN/coupler.log" 2>&1 &
COUPLER=$!
for i in $(seq 1 100); do [ -e /tmp/v90hsf ] && break; sleep 0.1; done
if [ ! -e /tmp/v90hsf ]; then echo "no PTY"; wait $COUPLER; exit 1; fi
/tmp/hsfvenv/bin/python tools/hsf_pty_traffic.py /tmp/v90hsf hsf /tmp/v90server srv \
    $((SECS - 5)) "${GAP:-0.12}" > "$RUN/traffic.log" 2>&1
wait $COUPLER
echo "== $RUN"
grep -E "V8 selected|CONNECT|V34 restart|TRAINING timeout|training complete" "$RUN/coupler.log" | tail -6
cat "$RUN/traffic.log"
