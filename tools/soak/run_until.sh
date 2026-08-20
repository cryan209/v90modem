#!/bin/bash
# Keep the rig dialling until something appears in the server log.
#
# Both the Phase-3 retrain lottery and the upstream acquisition lottery mean
# an interesting call can take many attempts, and soak_orchestrate2.sh stops
# as soon as one reaches data mode -- which is not the same thing as reaching
# the state you are waiting for.  This runs it one attempt at a time and
# checks after each, so a batch can be left alone.
#
#   run_until.sh <rundir> <server.log> <egrep pattern> [max attempts]
#
# Exits 0 the moment the pattern appears, 1 if the attempts run out.
SP="$(cd "$(dirname "$0")" && pwd)"
RUNDIR="$1"
SERVERLOG="$2"
PATTERN="$3"
MAX="${4:-40}"

if [ -z "$RUNDIR" ] || [ -z "$SERVERLOG" ] || [ -z "$PATTERN" ]; then
    echo "usage: run_until.sh <rundir> <server.log> <pattern> [max]" >&2
    exit 2
fi

start_size=$(stat -f %z "$SERVERLOG" 2>/dev/null || echo 0)

for attempt in $(seq 1 "$MAX"); do
    echo "=== RUN_UNTIL attempt $attempt/$MAX $(date -u +%H:%M:%SZ)"
    bash "$SP/soak_orchestrate2.sh" "$RUNDIR" "$SERVERLOG" 1 2>&1 \
        | sed 's/^/    /'
    if tail -c "+$((start_size + 1))" "$SERVERLOG" 2>/dev/null \
           | grep -aqE "$PATTERN"; then
        echo "=== RUN_UNTIL matched after $attempt attempts:"
        tail -c "+$((start_size + 1))" "$SERVERLOG" | grep -aE "$PATTERN" \
            | tail -3
        exit 0
    fi
done

echo "=== RUN_UNTIL gave up after $MAX attempts"
exit 1
