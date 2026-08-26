#!/bin/bash
# Live A/B of TRN2d length against the first Phase 4 attempt.
#
# docs/v90_phase3_s_and_rbs_false_positive.md §33/§36/§36a established that the
# first Phase 4 attempt against SmartLink deterministically fails
# (the peer's own "linear mapping study in TRN2" runs 1.82 s and asks for a
# retrain, with ph4MeanErrorEnergyBeforeToAfterUpdateRatio = 2.047 EXACTLY, on
# every default-length call in the corpus) and that lengthening TRN1d to
# 16008T fixes it at a 23% downstream rate cost.
#
# This tests the other training signal, which nothing has swept.  Over 853
# Phase 4 study windows in artifacts/ the peer's study -> "Null MP @ end of
# TRN2d" interval is 1.300 s in every one, i.e. its TRN2d window is 1.3 s --
# while V90_TRN2D_DEFAULT_SYMBOLS is 4000T = 500 ms.  So the peer spends 62%
# of the window it uses to grade our TRN2d looking at our MP instead.
# §9.4.1.2 asks for a minimum of 2040T and §9.4.1.3 allows up to 2000 ms
# before MP, so 10398T (1300 ms) and 12000T (1500 ms) are both conformant.
#
# TRN2d is transmitted AFTER the peer's constellation is designed from CPt, so
# unlike TRN1d it cannot move the negotiated rate -- which is the whole reason
# it is worth testing.
#
#   v90_trn2d_ab.sh <outdir> [repeats] [trn2d-symbols,...]
set -u
OUT=${1:?usage: v90_trn2d_ab.sh outdir [repeats] [trn2d-symbols,...]}
REPEATS=${2:-4}
TRNS=${3:-10398,12000}
SP="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SP/../.." && pwd)"
mkdir -p "$OUT"

for r in $(seq 1 "$REPEATS"); do
  for arm in control $(echo "$TRNS" | tr ',' '\n' | sed 's/^/trn/'); do
    d="$OUT/$arm-r$r"
    [ -d "$d" ] && { echo "CONTROL: $arm r$r present, skipping"; continue; }
    # Same pinning as v90_trn1d_ab.sh: both give-up triggers off in every arm,
    # so a call that concedes V.90 part way through cannot be scored as a
    # TRN2d datum, and the run stays comparable to the TRN1d sweeps.
    common="ME_V90_JA_DEADLINE_SEC=0 ME_V90_JA_CONCEDE_ATTEMPTS=0"
    if [ "$arm" = control ]; then extra="$common"
    else                         extra="$common ME_V90_TRN2D_SYMBOLS=${arm#trn}"; fi
    echo "CONTROL: $(date -u +%H:%M:%SZ) arm=$arm repeat=$r"
    EXTRA_ENV="$extra" bash "$SP/v90_notch_ab.sh" "$d" 1 > "$d.out" 2>&1
    grep -a "TRN2d (" "$d/server.log" | head -1 | sed 's/^/  /'
  done
done

echo
echo "CONTROL: TRN2d A/B, $TRNS vs default"
python3 "$ROOT/tools/trn1d_ab_summary.py" "$OUT"
