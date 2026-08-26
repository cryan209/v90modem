#!/bin/bash
# Live A/B of the V.90 Ja concession (docs/v90_phase3_s_and_rbs_false_positive.md §35k).
#
# Arm "concede" is the default build: after N consecutive Phase 3 attempts with
# no CRC-valid Ja descriptor the engine stops asking for V.90 and continues as
# plain V.34.  Arm "persist" sets ME_V90_JA_CONCEDE_ATTEMPTS=0 and restores the
# old behaviour, in which the peer retrains three times, gives up itself, and
# offers a V.34 INFO1a we reject.
#
# The two questions this has to answer, in order:
#
#   1. Does conceding COST anything?  It fires only when V.90 has failed twice,
#      so V.90 data-mode reachability must be unchanged.  That is the risk:
#      a call that would have parsed on attempt 3 is one we now give up on.
#   2. Does it BUY anything?  On calls where V.90 was never going to work, how
#      much sooner does the call reach a working V.34 link.
#
# Arms are alternated rather than blocked so a drifting rig cannot masquerade
# as an effect, one call per run.
#
# The threshold is an argument so the SHIPPING default can be tested rather
# than whatever it happened to be when the script was written.
#
#   v90_concede_ab.sh <outdir> [repeats] [threshold]
set -u
OUT=${1:?usage: v90_concede_ab.sh outdir [repeats] [threshold]}
REPEATS=${2:-4}
THRESH=${3:-3}
SP="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SP/../.." && pwd)"
mkdir -p "$OUT"

for r in $(seq 1 "$REPEATS"); do
  for arm in concede persist; do
    d="$OUT/$arm-r$r"
    [ -d "$d" ] && { echo "CONTROL: $arm r$r present, skipping"; continue; }
    if [ "$arm" = persist ]; then extra="ME_V90_JA_CONCEDE_ATTEMPTS=0"
    else                         extra="ME_V90_JA_CONCEDE_ATTEMPTS=$THRESH"; fi
    echo "CONTROL: $(date -u +%H:%M:%SZ) arm=$arm repeat=$r"
    EXTRA_ENV="$extra" bash "$SP/v90_notch_ab.sh" "$d" 1 > "$d.out" 2>&1
    grep -a "conceding V.90\|declined by peer INFO1a" "$d/server.log" | head -1 | sed 's/^/  /'
  done
done

echo
echo "CONTROL: concede A/B (threshold $THRESH)"
python3 "$ROOT/tools/concede_ab_summary.py" "$OUT"
