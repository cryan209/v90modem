#!/bin/bash
# Sweep the V.90 upstream rate and measure the eye at each one.
#
# The two axes the name promises are rate and TIME: the eye's quality is one
# number per call (CINR, distance from the lattice), but how long it lasts is
# a different number and the two do not move together.  A rate can settle at
# 0.045 and hold for eight seconds, or settle at 0.045 and hold for the whole
# call; tools/eye_summary.py reports both, and `hold_s` is the axis that
# separates them.
#
# Each row is a live call, so the whole sweep is slow and the handshake is not
# guaranteed -- a row that never reached data mode reports "-" rather than
# being retried for ever.
#
#   v90_rate_matrix.sh <outdir> [rates...] [--repeats N] [--attempts N]
set -u
OUT=${1:?usage: v90_rate_matrix.sh outdir [rates...]}
shift
SP="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SP/../.." && pwd)"
REPEATS=${REPEATS:-2}
ATTEMPTS=${ATTEMPTS:-3}
RATES=${*:-"19200 21600 24000 26400 28800 31200"}

mkdir -p "$OUT"
cd "$ROOT"
echo "CONTROL: rate matrix into $OUT — rates: $RATES, $REPEATS repeat(s) each"

for rate in $RATES; do
    for r in $(seq 1 "$REPEATS"); do
        d="$OUT/rate$rate-r$r"
        [ -d "$d" ] && { echo "CONTROL: $rate r$r already present, skipping"; continue; }
        echo "CONTROL: $(date -u +%H:%M:%SZ) rate $rate repeat $r"
        EXTRA_ENV="ME_V90_UPSTREAM_MAX_BPS=$rate" \
            bash "$SP/v90_notch_ab.sh" "$d" "$ATTEMPTS" > "$d.out" 2>&1
        python3 "$ROOT/tools/eye_summary.py" "$d" 2>/dev/null | tail -1
    done
done

echo
echo "CONTROL: matrix"
python3 "$ROOT/tools/eye_summary.py" "$OUT"/rate*-r* 2>/dev/null
