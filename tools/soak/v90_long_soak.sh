#!/bin/bash
# Long soak of the SHIPPING V.90 defaults -- no ME_* overrides at all.
#
# The A/B that set TRN1d to 20004T (docs/v90_phase3_s_and_rbs_false_positive.md
# §38) ran the standard 105 s schedule, which proves the path exists.  This
# proves it STAYS up: SOAK_SECONDS stretches soak_pty.py/soak_sock.py's
# three-phase schedule (downstream only / upstream only / bidirectional), so a
# 600 s call spends ~200 s in each.
#
# One call per run and a fresh server process per call, because the unit of
# this measurement is a call: soak_orchestrate2.sh exits at the first call that
# completes a schedule, so N calls need N invocations, not attempts=N.
#
#   v90_long_soak.sh <outdir> [calls] [seconds-per-call]
set -u
OUT=${1:?usage: v90_long_soak.sh outdir [calls] [seconds]}
CALLS=${2:-6}
SECS=${3:-600}
SP="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SP/../.." && pwd)"
mkdir -p "$OUT"

for r in $(seq 1 "$CALLS"); do
  d="$OUT/soak-r$r"
  [ -d "$d" ] && { echo "CONTROL: r$r present, skipping"; continue; }
  echo "CONTROL: $(date -u +%H:%M:%SZ) call $r/$CALLS, ${SECS}s schedule"
  # Deliberately no EXTRA_ENV: the point is the defaults as shipped.
  SOAK_SECONDS="$SECS" bash "$SP/v90_notch_ab.sh" "$d" 3 > "$d.out" 2>&1
  grep -a "TRN1d complete\|startup complete" "$d/server.log" | head -2 | sed 's/^/  /'
done

echo
echo "CONTROL: long soak of shipping defaults"
SOAK_SECONDS="$SECS" python3 "$ROOT/tools/v90_long_soak_summary.py" "$OUT"
