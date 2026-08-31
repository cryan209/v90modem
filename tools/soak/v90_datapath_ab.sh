#!/bin/bash
# Live A/B of the fixed-point V.34 datapath against the floating-point one.
#
# The two arms differ at COMPILE time (-DV34_FIXED_POINT changes both v34rx.c
# and the private header, so the modes cannot share objects), which is why
# this drives two separately built binaries through SERVER_BIN rather than an
# ME_* knob.  Build them first:
#
#   make fixed && cp sip_v90_modem /tmp/sip_v90_modem.fixedpt
#   make float && cp sip_v90_modem /tmp/sip_v90_modem.floatpt
#
# and pass the directory holding them.  Each binary reports its own datapath
# at startup ("[ME] V.34 datapath: ..."), read back from libspandsp rather
# than from the flag the application saw, and this script asserts the arm it
# meant to run is the arm that ran -- a -D in CFLAGS can otherwise leave the
# library and the application disagreeing with no other symptom.
#
# Arms are alternated rather than run in blocks, so a drifting rig cannot
# masquerade as an effect.  Scored on clean TIME, longest unbroken hold and
# intact U-lines: byte percentages count the garbage emitted while unlocked.
#
#   v90_datapath_ab.sh <outdir> [bindir] [repeats] [attempts]
set -u
OUT=${1:?usage: v90_datapath_ab.sh outdir [bindir] [repeats] [attempts]}
BIN=${2:-/tmp/v90-datapath-bin}
REPEATS=${3:-3}
ATTEMPTS=${4:-3}
SP="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SP/../.." && pwd)"
mkdir -p "$OUT"

for arm in fixedpt floatpt; do
  [ -x "$BIN/sip_v90_modem.$arm" ] || {
    echo "CONTROL: missing $BIN/sip_v90_modem.$arm -- build both arms first"; exit 2; }
done

for r in $(seq 1 "$REPEATS"); do
  for arm in fixedpt floatpt; do
    d="$OUT/$arm-r$r"
    [ -d "$d" ] && { echo "CONTROL: $arm r$r present, skipping"; continue; }
    echo "CONTROL: $(date -u +%H:%M:%SZ) arm=$arm repeat=$r"
    SERVER_BIN="$BIN/sip_v90_modem.$arm" \
      bash "$SP/v90_notch_ab.sh" "$d" "$ATTEMPTS" > "$d.out" 2>&1
    # Confirm the binary that ran is the arm we asked for.
    got=$(grep -a "V.34 datapath:" "$d/server.log" | head -1 | sed 's/.*datapath: //')
    case "$arm:$got" in
      "fixedpt:fixed point"|"floatpt:floating point") echo "  datapath: $got" ;;
      *) echo "  CONTROL: WRONG ARM -- wanted $arm, log says '${got:-nothing}'" ;;
    esac
  done
done

echo
echo "CONTROL: fixed-point vs floating-point datapath"
python3 "$ROOT/tools/slip_ab_summary.py" "$OUT"
