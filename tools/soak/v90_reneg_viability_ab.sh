#!/bin/bash
# Live A/B of the §9.6 viability gate: does an attempt that never decoded the
# peer's CP' correctly stop this call from opening another?
#
# BOTH arms run with ME_V90_RENEG=1 -- this is not about whether to initiate
# (measured separately, artifacts/reneg-ab-*: 6 requested, 0 completed, and
# the knob stays default off).  It is about what an ENABLED renegotiation
# costs.  The variable is ME_V90_RENEG_VIABILITY: 0 is the old behaviour, up
# to ME_V90_MAX_RENEGOTIATIONS attempts a call, and the default gates further
# attempts once one has taken §9.6.1's timeout without a CP'.
#
# The primary metric is ATTEMPTS PER CALL, which is what the gate acts on and
# is a count rather than a throughput figure -- the previous A/B's line counts
# had one dead call per arm and are not the instrument to hang this on.  The
# prediction from that A/B's own logs is 3, 1, 2 attempts ungated against at
# most 1 apiece gated.
#
# Method is §39's, as before: 600 s calls, arms ALTERNATED one call at a time,
# one binary, no other overrides.
#
#   v90_reneg_viability_ab.sh <outdir> [repeats] [seconds-per-call]
set -u
OUT=${1:?usage: v90_reneg_viability_ab.sh outdir [repeats] [seconds]}
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
  for arm in ungated gated; do
    d="$OUT/$arm-r$r"
    [ -d "$d" ] && { echo "CONTROL: $arm r$r present, skipping"; continue; }
    if [ "$arm" = ungated ]; then extra="ME_V90_RENEG=1 ME_V90_RENEG_VIABILITY=0"
    else                          extra="ME_V90_RENEG=1"; fi
    echo "CONTROL: $(date -u +%H:%M:%SZ) arm=$arm repeat=$r (${SECS}s)"
    EXTRA_ENV="$extra" SOAK_SECONDS="$SECS" \
        bash "$SP/v90_notch_ab.sh" "$d" 3 > "$d.out" 2>&1
    printf '  attempts=%s completed=%s gate_fired=%s\n' \
      "$(grep -ac 'requesting a §9.6 rate renegotiation' "$d/server.log" 2>/dev/null)" \
      "$(grep -ac 'Rate renegotiation [0-9]* complete' "$d/server.log" 2>/dev/null)" \
      "$(grep -ac 'not opening another on this call' "$d/server.log" 2>/dev/null)"
  done
done

echo
echo "CONTROL: §9.6 viability gate A/B over ${SECS}s calls"
printf '  %-12s %8s %10s %10s %9s\n' call attempts completed gate_fired carried
for d in "$OUT"/*-r*/; do
  [ -d "$d" ] || continue
  printf '  %-12s %8s %10s %10s\n' "$(basename "$d")" \
    "$(grep -ac 'requesting a §9.6 rate renegotiation' "$d/server.log" 2>/dev/null)" \
    "$(grep -ac 'Rate renegotiation [0-9]* complete' "$d/server.log" 2>/dev/null)" \
    "$(grep -ac 'not opening another on this call' "$d/server.log" 2>/dev/null)"
done

echo
for arm in ungated gated; do
  echo "=== $arm"
  t="$OUT/.$arm"; rm -rf "$t"; mkdir -p "$t"
  for d in "$OUT/$arm"-r*/; do
    [ -d "$d" ] || continue
    ln -s "$(cd "$d" && pwd)" "$t/soak-$(basename "$d" | sed 's/.*-r/r/')"
  done
  SOAK_SECONDS="$SECS" python3 "$ROOT/tools/v90_long_soak_summary.py" "$t"
done
