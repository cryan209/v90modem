#!/bin/sh
# Score the V.90 upstream receiver over a directory of recorded calls.
#
# Every figure in docs/v90_upstream_data_path.md that describes how long the
# upstream holds its constellation came from reading one replay's log by eye.
# This runs the whole matrix, in parallel, and reports clean TIME -- never
# window counts, which are the trap the doc warns about: a white stretch
# emits short windows and a clean one long windows, so a window-weighted
# percentage read 78% on a call that was 21% clean.
#
#   tools/v90_upstream_bench.sh <dir-of-call-dirs> [outdir]
#
# The rate of each call is read from its own server.log, so a directory of
# mixed rates scores correctly.  Environment reaches the replay unchanged,
# which is the point: run it twice with one knob moved and diff the tables.
set -e
dir=${1:?usage: v90_upstream_bench.sh <dir> [outdir]}
out=${2:-$(mktemp -d)}
mkdir -p "$out"
here=$(cd "$(dirname "$0")/.." && pwd)
for call in "$dir"/*/; do
    name=$(basename "$call")
    tap="$call/live-rx.g711"
    [ -f "$tap" ] || continue
    log="$call/server.log"
    rate=$(sed -n 's/.*upstream selection: [0-9]* baud, rate cap \([0-9]*\) bps.*/\1/p' "$log" 2>/dev/null | head -1)
    baud=$(sed -n 's/.*upstream selection: \([0-9]*\) baud.*/\1/p' "$log" 2>/dev/null | head -1)
    [ -n "$rate" ] || continue
    # Skip a capture whose own call never reached upstream data mode.  There
    # is no B1 in it to find, so the replay scans the entire file at every
    # candidate instant -- minutes of CPU to conclude what the server log
    # already says -- and it tells us nothing about the receiver.
    if ! grep -q "upstream DATA bits" "$log"; then
        echo "$name: the call itself never reached data mode; skipped" \
            > "$out/$name.skip"
        continue
    fi
    # A capture the replay cannot find B1 in scans the whole file at every
    # candidate instant and takes tens of minutes to conclude nothing, which
    # on a twelve-call matrix is the entire runtime.  Bound it.
    (
        "$here/v90_upstream_replay" "$tap" ulaw "${baud:-3200}" "$rate" \
            > "$out/$name.out" 2>&1 &
        pid=$!
        ( sleep "${BENCH_TIMEOUT:-420}"; kill "$pid" 2>/dev/null ) &
        guard=$!
        wait "$pid" 2>/dev/null
        kill "$guard" 2>/dev/null
    ) &
done
wait
for f in "$out"/*.out; do
    printf '%-16s ' "$(basename "$f" .out)"
    python3 "$here/tools/v90_upstream_score.py" < "$f"
done
echo "logs in $out"
