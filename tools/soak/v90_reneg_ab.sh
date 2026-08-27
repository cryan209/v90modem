#!/bin/bash
# Live A/B of §9.6 rate-renegotiation-on-upstream-loss over a LONG call.
#
# The question this exists to answer.  §9.6 now completes: the peer's CP
# decodes, we send MP'/Ed, and it returns to DATAXMIT (verified live 3/3,
# docs/retrain_and_resync.md).  That makes initiating one a real option again
# -- but the engine's trigger is `v34_v90_upstream_carrier_lost()`, and on this
# rig the upstream is the chronically broken direction, so a renegotiation
# fires against a receiver whose eye has shut for reasons a fresh Phase 4 may
# not fix (docs/v90_upstream_data_path.md).
#
# That is exactly the shape §39 measured for the RETRAIN on the same trigger,
# where the answer was that recovering the upstream cost more downstream than
# it was worth: ME_V90_RETRAIN_ON_LOSS went to 0.  With that cap at 0, the
# control arm here does NOTHING on upstream loss, so this A/B is a clean
# "cheap resync vs leave it alone" -- not "resync vs retrain".
#
# Method is §39's, deliberately, so the two are comparable:
#   * 600 s calls -- 105 s cannot see this.  §39's own finding was that the
#     short schedule proves the path exists and says nothing about holding it.
#   * arms ALTERNATED, one call per run, so a drifting rig cannot masquerade
#     as an effect.
#   * one binary, ME_V90_RENEG the only variable, no other overrides.
#   * scored on carried time and LINE counts.  Read `carried`, not the
#     schedule length: every call completes its 600 s schedule whatever
#     happens, and `carried` is the last moment the link delivered a byte.
#     And read lines, not byte percentages -- an unlocked receiver emits
#     garbage that inflates a byte total.
#
#   v90_reneg_ab.sh <outdir> [repeats] [seconds-per-call]
set -u
OUT=${1:?usage: v90_reneg_ab.sh outdir [repeats] [seconds]}
REPEATS=${2:-3}
SECS=${3:-600}
SP="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SP/../.." && pwd)"
mkdir -p "$OUT"

if pgrep -x sip_v90_modem > /dev/null 2>&1; then
  echo "CONTROL: a sip_v90_modem is already running -- kill it first:"
  pgrep -lx sip_v90_modem | sed 's/^/  /'
  exit 1
fi

for r in $(seq 1 "$REPEATS"); do
  for arm in control reneg; do
    d="$OUT/$arm-r$r"
    [ -d "$d" ] && { echo "CONTROL: $arm r$r present, skipping"; continue; }
    if [ "$arm" = control ]; then extra=""
    else                         extra="ME_V90_RENEG=1"; fi
    echo "CONTROL: $(date -u +%H:%M:%SZ) arm=$arm repeat=$r (${SECS}s)"
    EXTRA_ENV="$extra" SOAK_SECONDS="$SECS" \
        bash "$SP/v90_notch_ab.sh" "$d" 3 > "$d.out" 2>&1
    grep -a "startup complete" "$d/server.log" 2>/dev/null | head -1 | sed 's/^/  /'
    grep -acE "requesting a §9.6 rate renegotiation|Rate renegotiation [0-9]+ complete" \
         "$d/server.log" 2>/dev/null | sed 's/^/  reneg lines: /'
  done
done

echo
echo "CONTROL: §9.6 renegotiation-on-upstream-loss A/B over ${SECS}s calls"
for arm in control reneg; do
  echo "=== $arm"
  t="$OUT/.$arm"; rm -rf "$t"; mkdir -p "$t"
  for d in "$OUT/$arm"-r*/; do
    [ -d "$d" ] || continue
    ln -s "$(cd "$d" && pwd)" "$t/soak-$(basename "$d" | sed 's/.*-r/r/')"
  done
  SOAK_SECONDS="$SECS" python3 "$ROOT/tools/v90_long_soak_summary.py" "$t"
done

echo
echo "CONTROL: per-call §9.6 activity"
for d in "$OUT"/*-r*/; do
  [ -d "$d" ] || continue
  printf '  %-14s requested=%s completed=%s timeout=%s\n' "$(basename "$d")" \
    "$(grep -ac 'requesting a §9.6 rate renegotiation' "$d/server.log" 2>/dev/null)" \
    "$(grep -ac 'Rate renegotiation [0-9]* complete' "$d/server.log" 2>/dev/null)" \
    "$(grep -ac 'no E within' "$d/server.log" 2>/dev/null)"
done
