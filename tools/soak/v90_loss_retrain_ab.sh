#!/bin/bash
# Live A/B of §9.5.1.1 retrain-on-upstream-loss over a LONG call.
#
# The 600 s soak of the shipping defaults (artifacts/v90-longsoak-*) connected
# cleanly every time -- first Phase 4 attempt 5/5 after §38 -- but only one of
# five calls HELD.  In three of the four that did not, the first disruption
# after data mode is OUR OWN detector:
#
#   Rx - V.90 upstream carrier lost: 0.70 from the constellation for 3200
#   symbols (settled at 0.155)
#   V.90 upstream carrier lost and §9.6 is not available; initiating a
#   §9.5.1.1 retrain (1 of 4)
#
# The downstream at that moment is perfect -- the one call that took no retrain
# delivered 222377 lines with 4 missing -- so we tear down a working direction
# to chase the known-broken one (the upstream frame-phase/eye problem, which a
# retrain does not fix).  §9.5.1.1 says the digital modem MAY retrain at any
# time, so both policies are conformant and the question is which pays.
#
# Arms alternated, one call per run, no other overrides.
#
#   v90_loss_retrain_ab.sh <outdir> [repeats] [seconds-per-call]
set -u
OUT=${1:?usage: v90_loss_retrain_ab.sh outdir [repeats] [seconds]}
REPEATS=${2:-3}
SECS=${3:-600}
SP="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SP/../.." && pwd)"
mkdir -p "$OUT"

for r in $(seq 1 "$REPEATS"); do
  for arm in control noloss; do
    d="$OUT/$arm-r$r"
    [ -d "$d" ] && { echo "CONTROL: $arm r$r present, skipping"; continue; }
    if [ "$arm" = control ]; then extra=""
    else                         extra="ME_V90_RETRAIN_ON_LOSS=0"; fi
    echo "CONTROL: $(date -u +%H:%M:%SZ) arm=$arm repeat=$r (${SECS}s)"
    EXTRA_ENV="$extra" SOAK_SECONDS="$SECS" \
        bash "$SP/v90_notch_ab.sh" "$d" 3 > "$d.out" 2>&1
    grep -a "startup complete" "$d/server.log" | head -1 | sed 's/^/  /'
  done
done

echo
echo "CONTROL: §9.5.1.1 retrain-on-loss A/B over ${SECS}s calls"
for arm in control noloss; do
  echo "=== $arm"
  t="$OUT/.$arm"; rm -rf "$t"; mkdir -p "$t"
  for d in "$OUT/$arm"-r*/; do
    [ -d "$d" ] || continue
    ln -s "$(cd "$d" && pwd)" "$t/soak-$(basename "$d" | sed 's/.*-r/r/')"
  done
  SOAK_SECONDS="$SECS" python3 "$ROOT/tools/v90_long_soak_summary.py" "$t"
done
