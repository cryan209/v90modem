#!/bin/bash
# Live A/B of the V.90 Ja DEADLINE trigger (docs §35m).
#
# The attempt-count trigger cannot win a race whose clock the peer owns, so it
# was replaced by a deadline: concede if no CRC-valid Ja descriptor has arrived
# T seconds of Phase 3 into the call.  This measures it live.
#
# The attempt count is disabled in BOTH arms so the deadline is the only
# variable -- otherwise a concession could come from either trigger and the
# arms would differ in two ways at once.
#
#   arm "deadline": ME_V90_JA_DEADLINE_SEC=T  ME_V90_JA_CONCEDE_ATTEMPTS=0
#   arm "persist" : ME_V90_JA_DEADLINE_SEC=0  ME_V90_JA_CONCEDE_ATTEMPTS=0
#
# Two questions, as in §35l, and they are not the same measurement:
#   COST -- V.90 data-mode reachability per call, which must not drop.  The
#           corpus says 20 s loses no call that reached data mode; this is the
#           live check of that.
#   BUY  -- on calls that never get a descriptor, how much sooner the call
#           stops flogging V.90, and whether the give-up is OURS or the peer's.
#
# Arms alternated, one call per run.
#
#   v90_deadline_ab.sh <outdir> [repeats] [seconds]
set -u
OUT=${1:?usage: v90_deadline_ab.sh outdir [repeats] [seconds]}
REPEATS=${2:-4}
T=${3:-20}
SP="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SP/../.." && pwd)"
mkdir -p "$OUT"

for r in $(seq 1 "$REPEATS"); do
  for arm in deadline persist; do
    d="$OUT/$arm-r$r"
    [ -d "$d" ] && { echo "CONTROL: $arm r$r present, skipping"; continue; }
    if [ "$arm" = persist ]; then
      extra="ME_V90_JA_DEADLINE_SEC=0 ME_V90_JA_CONCEDE_ATTEMPTS=0"
    else
      extra="ME_V90_JA_DEADLINE_SEC=$T ME_V90_JA_CONCEDE_ATTEMPTS=0"
    fi
    echo "CONTROL: $(date -u +%H:%M:%SZ) arm=$arm repeat=$r"
    EXTRA_ENV="$extra" bash "$SP/v90_notch_ab.sh" "$d" 1 > "$d.out" 2>&1
    grep -a "conceding V.90\|declined by peer INFO1a" "$d/server.log" | head -1 | sed 's/^/  /'
  done
done

echo
echo "CONTROL: deadline A/B (T=${T}s)"
python3 "$ROOT/tools/concede_ab_summary.py" "$OUT"
