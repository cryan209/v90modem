#!/bin/bash
# Dial until data mode, restarting the modem server between rounds.
#
# sip_v90_modem leaks a pjsua media transport per call and stops being able
# to create them after roughly nine (PJ_ETOOMANY).  Every call after that
# fails to get a media channel, the far end reports NO CARRIER, and the whole
# thing reads as the peer refusing to train -- it cost this session two wrong
# diagnoses.  Until the leak itself is fixed, keep each server process well
# inside its budget and give every round a fresh one.
#
#   soak_rounds.sh <rundir> <rounds> <attempts-per-round>
SP="$(cd "$(dirname "$0")" && pwd)"
RD="$1"; ROUNDS="${2:-6}"; PER="${3:-6}"
[ -n "$RD" ] || { echo "usage: soak_rounds.sh <rundir> [rounds] [attempts]" >&2; exit 2; }

for round in $(seq 1 "$ROUNDS"); do
    echo "=== ROUND $round/$ROUNDS $(date -u +%H:%M:%SZ): fresh server"
    pkill -f sip_v90_modem; sleep 3
    RDIR="$RD/round$round"; mkdir -p "$RDIR/tap"
    ME_V34_DATA_CARRIER_TRACK=0 SIP_FORCE_PCMU=1 VPCM_ME_VERBOSE=1 \
        ME_V90_UPSTREAM_MAX_BPS=9600 VPCM_G711_TAP_DIR="$RDIR/tap" \
        ./sip_v90_modem --sip-server asterisk.net.cryan.nz \
            --username 6001 --password 6001 --pty-link /tmp/modem0 \
            > "$RDIR/server.log" 2>&1 &
    sleep 12
    if ! grep -aq "registration success" "$RDIR/server.log"; then
        echo "=== ROUND $round: server did not register; skipping"
        continue
    fi
    if bash "$SP/run_until.sh" "$RDIR" "$RDIR/server.log" \
            "V.90 startup complete" "$PER"; then
        echo "=== ROUND $round: reached data mode"
        exit 0
    fi
    if grep -aq "PJ_ETOOMANY" "$RDIR/server.log"; then
        echo "=== ROUND $round: hit the media-transport leak; next round"
    fi
done
echo "=== no round reached data mode"
exit 1
