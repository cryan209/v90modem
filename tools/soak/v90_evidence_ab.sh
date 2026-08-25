#!/bin/bash
# Live A/B of the frame-phase confirmation stage.
#
# Arm "fixed" is the default build: me_rx_g711_slip_permitted() returns false
# and nothing is spliced into the received stream.  Arm "slip" sets
# ME_V90_SWEEP_NEEDS_EVIDENCE=0 and restores the old behaviour.  Everything else -- one
# binary, one rig, interleaved calls -- is held constant, and the arms are
# alternated rather than run in blocks so a drifting rig cannot masquerade as
# an effect.
#
# Scored on clean TIME and longest unbroken hold off the server's own
# DATA-bits lines, plus intact U-lines delivered to our PTY, which is the
# honest measure: byte percentages count the garbage emitted while unlocked.
#
#   v90_phase_ab.sh <outdir> [rate] [repeats] [attempts]
set -u
OUT=${1:?usage: v90_phase_ab.sh outdir [rate] [repeats] [attempts]}
RATE=${2:-19200}
REPEATS=${3:-3}
ATTEMPTS=${4:-3}
SP="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SP/../.." && pwd)"
mkdir -p "$OUT"

for r in $(seq 1 "$REPEATS"); do
  for arm in evidence noevidence; do
    d="$OUT/$arm-r$r"
    [ -d "$d" ] && { echo "CONTROL: $arm r$r present, skipping"; continue; }
    if [ "$arm" = noevidence ]; then extra="ME_V90_UPSTREAM_MAX_BPS=$RATE ME_V90_PHASE_NO_MARKS=1 ME_V90_SWEEP_NEEDS_EVIDENCE=0"
    else                      extra="ME_V90_UPSTREAM_MAX_BPS=$RATE ME_V90_PHASE_NO_MARKS=1"; fi
    echo "CONTROL: $(date -u +%H:%M:%SZ) arm=$arm repeat=$r rate=$RATE"
    EXTRA_ENV="$extra" bash "$SP/v90_notch_ab.sh" "$d" "$ATTEMPTS" > "$d.out" 2>&1
    # Confirm the knob actually took effect in this run.
    grep -a "candidates survived\|locked on shell evidence\|frame phase locked" "$d/server.log" | head -1 | sed 's/^/  /'
  done
done

echo
echo "CONTROL: sweep-evidence A/B, rate $RATE"
python3 "$ROOT/tools/slip_ab_summary.py" "$OUT"
