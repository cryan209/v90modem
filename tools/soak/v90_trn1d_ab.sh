#!/bin/bash
# Live A/B of TRN1d length against the first Phase 4 attempt.
#
# docs/v90_phase3_s_and_rbs_false_positive.md §32 left this as a lead, not a
# result: ONE call at ME_V90_TRN1D_SYMBOLS=16008 (2001 ms, against the 312 ms
# default) got the FIRST Phase 4 attempt to data mode, where it otherwise
# always retrains -- but at 40000 bps instead of 52000, and the call later
# destabilised.  One call each way proves nothing, so this runs both arms
# alternated (never blocked, so a drifting rig cannot masquerade as an effect)
# with one call per run, and scores:
#
#   - did the FIRST Phase 4 attempt reach data mode (the claim), and
#   - at what downstream rate, and did the call then hold (the cost).
#
# The peer grades Phase 4 in a "linear mapping study in TRN2" and prints both
# ends of it, so its own log is the instrument for the first question; ours is
# the instrument for the second.
#
#   v90_trn1d_ab.sh <outdir> [repeats] [trn1d-symbols]
set -u
OUT=${1:?usage: v90_trn1d_ab.sh outdir [repeats] [trn1d-symbols]}
REPEATS=${2:-4}
TRN=${3:-16008}
SP="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SP/../.." && pwd)"
mkdir -p "$OUT"

for r in $(seq 1 "$REPEATS"); do
  for arm in control trn$TRN; do
    d="$OUT/$arm-r$r"
    [ -d "$d" ] && { echo "CONTROL: $arm r$r present, skipping"; continue; }
    if [ "$arm" = control ]; then extra=""
    else                         extra="ME_V90_TRN1D_SYMBOLS=$TRN"; fi
    echo "CONTROL: $(date -u +%H:%M:%SZ) arm=$arm repeat=$r"
    # One call per run: the unit of this experiment is a call, and letting the
    # orchestrator redial would mix arms' worth of calls into one directory.
    EXTRA_ENV="$extra" bash "$SP/v90_notch_ab.sh" "$d" 1 > "$d.out" 2>&1
    grep -a "TRN1d complete" "$d/server.log" | head -1 | sed 's/^/  /'
  done
done

echo
echo "CONTROL: TRN1d A/B, $TRN vs default"
python3 "$ROOT/tools/trn1d_ab_summary.py" "$OUT"
