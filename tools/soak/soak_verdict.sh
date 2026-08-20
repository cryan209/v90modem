#!/bin/bash
# One-line-per-attempt verdict for a soak run directory.
#
# The interesting question is never "did it connect" but where in the chain
# it stopped: CP accepted -> upstream armed -> B1 acquired and validated ->
# slips corrected -> bytes at each DTE.  Reading that out of two pump logs
# and a server log by hand each time is how attempts get misread; this
# prints the whole chain at once.
#
#   soak_verdict.sh <rundir>
RD="$1"
[ -d "$RD" ] || { echo "usage: soak_verdict.sh <rundir>" >&2; exit 2; }
LOG="$RD/server.log"
printf "%-6s %8s %8s %7s %7s  %s\n" try down-B up-B lines dur note
for d in "$RD"/try*/; do
    [ -d "$d" ] || continue
    n=$(basename "$d")
    down=$(grep -a "SOCK DONE\|SOCK t=" "$d/sock.log" 2>/dev/null | tail -1 | sed -E 's/.*rx=([0-9]+).*/\1/')
    up=$(grep -a "PTY DONE\|PTY t=" "$d/pty.log" 2>/dev/null | tail -1 | sed -E 's/.*rx=([0-9]+).*/\1/')
    dur=$(grep -a "SOCK t=" "$d/sock.log" 2>/dev/null | tail -1 | sed -E 's/.*t= *([0-9.]+).*/\1/')
    note=$(grep -a "NO CARRIER at\|dial failed" "$d/sock.log" 2>/dev/null | tail -1 | cut -c1-40)
    printf "%-6s %8s %8s %7s %7s  %s\n" "$n" "${down:-0}" "${up:-0}" "-" "${dur:-0}" "$note"
done
echo
echo "chain (whole run):"
for pat in "strict batch recovered" "CP_VALID.*accepted=1" "upstream RX data prepared" \
           "B1 out-of-sample check" "B1 rejected" "upstream B1 acquired" \
           "slip of" "startup complete"; do
    printf "  %-32s %s\n" "$pat" "$(grep -acE "$pat" "$LOG" 2>/dev/null)"
done
