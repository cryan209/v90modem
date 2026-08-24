#!/bin/bash
# Replay every recorded call twice -- as taped, and with its own clock-recovery
# splices undone -- and report clean TIME and the longest unbroken hold.
#
# Window counts are a trap here: a white stretch emits short windows and a
# clean one long windows, so a window-weighted percentage flatters a call that
# spent most of its seconds white.  Score seconds.
set -u
MATRIX=${1:?usage: desplice_matrix.sh <matrix-dir> [jobs]}
JOBS=${2:-4}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT=$(mktemp -d)

score() {  # stdin: replay log -> "clean_s total_s longest_s"
  awk '/DATA bits: t=/{
        match($0,/t=[0-9.]+s/); t=substr($0,RSTART+2,RLENGTH-3)+0
        match($0,/sym err [0-9.]+/); e=substr($0,RSTART+8,RLENGTH-8)+0
        d=t-pt; if(d<0)d=0; pt=t; tot+=d
        if(e<0.30){cl+=d; run+=d; if(run>best)best=run} else run=0
      } END{printf "%.1f %.1f %.1f", cl+0, tot+0, best+0}'
}

one() {
  d="$1"; name=$(basename "$d"); rate=$(echo "$name" | sed 's/^rate//; s/-r[0-9]*$//')
  [ -f "$d/live-rx.g711" ] || return
  "$ROOT/v90_upstream_replay" "$d/live-rx.g711" ulaw 3200 "$rate" > "$OUT/$name.base.out" 2> "$OUT/$name.base.log"
  ho=$(grep -o 'handover at [0-9.]*' "$OUT/$name.base.out" | head -1 | awk '{print $3}')
  [ -n "$ho" ] || { echo "$name no-data-mode" > "$OUT/$name.row"; return; }
  b=$(score < "$OUT/$name.base.log")
  python3 "$ROOT/tools/desplice_call.py" "$d" "$OUT/$name.g711" --handover "$ho" > "$OUT/$name.desp" 2>&1
  n=$(grep -o 'undid [0-9]*' "$OUT/$name.desp" | awk '{print $2}')
  "$ROOT/v90_upstream_replay" "$OUT/$name.g711" ulaw 3200 "$rate" > /dev/null 2> "$OUT/$name.fix.log"
  f=$(score < "$OUT/$name.fix.log")
  echo "$name ${n:-0} $b $f" > "$OUT/$name.row"
  rm -f "$OUT/$name.g711"
}

for d in "$MATRIX"/*/; do
  [ -d "$d" ] || continue
  # bash 3.2 (macOS) has no `wait -n`; poll instead.
  while [ "$(jobs -rp | wc -l)" -ge "$JOBS" ]; do sleep 1; done
  one "$d" &
done
wait

printf "%-14s %5s | %8s %8s | %8s %8s | %s\n" call slips as-taped hold de-spliced hold gain
printf -- "----------------------------------------------------------------------------------\n"
for r in "$OUT"/*.row; do
  read -r name n bc bt bl fc ft fl < "$r" 2>/dev/null || continue
  [ "$n" = "no-data-mode" ] && { printf "%-14s %5s | %s\n" "$name" "-" "no data mode"; continue; }
  awk -v n="$name" -v s="$n" -v bc="$bc" -v bl="$bl" -v fc="$fc" -v fl="$fl" \
    'BEGIN{printf "%-14s %5d | %7.1fs %7.1fs | %7.1fs %7.1fs | %+.1fs clean, %+.1fs hold\n", n, s, bc, bl, fc, fl, fc-bc, fl-bl}'
done | sort
rm -rf "$OUT"
