#!/bin/bash
# §9.6 V.90 rate renegotiation against a live peer, with BOTH G.711 taps kept.
#
# The question: does this rig's analogue modem answer 384T of Rd, or not?
#
# The standing claim is that it does not -- docs/retrain_and_resync.md and the
# comment on me_v90_reneg_enabled() both say so, and ME_V90_RENEG is default
# off because of it.  The evidence behind that is two calls in which the peer
# retrained and its log said SILENCERETRAIN.  But SILENCERETRAIN is the state
# where the peer TRANSMITS silence before its own Tone A -- see the
# peer-initiated retrain trace, DATAXMIT=>SILENCERETRAIN=>TONE_AB -- so it is
# not the peer reporting that it heard nothing from us.  It is consistent with
# the peer detecting Rd fine and failing later in §9.6.2, i.e. with us sending
# something wrong.
#
# And the same peer DOES implement §11.6: verified live, it answered our S,
# renegotiated 9600 -> 12000 and kept LAPM up across the change.
#
# So this captures the evidence that settles it, rather than either modem's
# opinion of what happened:
#   live-tx.g711  -- what WE actually put on the DS0, to demodulate against
#                    §9.6.1.1 (Rd 384T at U_INFO, R-bar-d 24T, then TRN2d/MP)
#   live-rx.g711  -- what the peer sent back during and after our Rd
#   server.log    -- our side's view
#
# Read the taps against the clause.  Do NOT diagnose this from the two modems'
# logs: their clocks differ by a per-call offset, which has cost this project
# a round before (docs/v34_plain_phase2_call_role.md).
#
#   v90_reneg_probe.sh <outdir> [calls] [reneg-after-ms] [seconds-per-call]
set -u
OUT=${1:?usage: v90_reneg_probe.sh outdir [calls] [after_ms] [seconds]}
CALLS=${2:-3}
AFTER=${3:-20000}
SECS=${4:-105}
SP="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$OUT"

# A stray server from an earlier batch holds the SIP port and every call in
# this one then dies with "bind() error: Address already in use" -- a 1879
# byte server.log and no call at all, which reads exactly like a rig that is
# refusing to connect.  That silently burned five of six calls once; fail
# fast instead.
if pgrep -x sip_v90_modem > /dev/null 2>&1; then
  echo "CONTROL: a sip_v90_modem is already running -- kill it first:"
  pgrep -lx sip_v90_modem | sed 's/^/  /'
  exit 1
fi

for r in $(seq 1 "$CALLS"); do
  d="$OUT/reneg-r$r"
  [ -d "$d" ] && { echo "CONTROL: r$r present, skipping"; continue; }
  echo "CONTROL: $(date -u +%H:%M:%SZ) call $r/$CALLS, reneg at ${AFTER}ms"
  # ME_V90_RENEG_AFTER_MS is the probe.  ME_V90_RENEG=1 so that if the peer
  # DOES answer, the engine's own carrier-loss path can use it later in the
  # same call -- the probe itself is not gated on it.
  # DIL_PROFILE bypasses the Ja-parse lottery (§34) -- on a rig sitting in
  # that blocker, "no CRC-valid Ja descriptor" concedes to plain V.34 on
  # every attempt and there is no V.90 data mode to renegotiate from at
  # all.  It preloads this peer's known descriptor so Sd can start; it is
  # a bypass, not a fix, and it is here only so §9.6 can be reached.
  EXTRA_ENV="ME_V90_RENEG_AFTER_MS=$AFTER ME_V90_RENEG=1 ${DIL_PROFILE:+ME_V90_DIL_PROFILE=$DIL_PROFILE} ${MORE_ENV:-}" \
    SOAK_SECONDS="$SECS" bash "$SP/v90_notch_ab.sh" "$d" 3 > "$d.out" 2>&1
  echo "  --- our side ---"
  grep -aE "RENEG_AFTER_MS probe|Rate renegotiation|renegotiation (timeout|timed out)|rate renegotiation" \
       "$d/server.log" 2>/dev/null | head -8 | sed 's/^/  /'
  ls -l "$d"/live-*.g711 2>/dev/null | sed 's/^/  /'
done

echo
echo "CONTROL: to settle it, demodulate OUR OWN Rd out of live-tx.g711 and"
echo "         check it against §9.6.1.1 before reading anything else."
