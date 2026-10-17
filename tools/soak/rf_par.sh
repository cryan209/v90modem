#!/bin/bash
# Three RasFinder calls in parallel (6311-6313), N rounds. usage: rf_par.sh <tag> [rounds]
cd "$(dirname "$0")/../.." || exit 1
TAG=${1:?tag}; ROUNDS=${2:-3}
for r in $(seq 1 "$ROUNDS"); do
  for i in 0 1 2; do
    RF_EXT=$((6311+i)) RF_USER=$((2900+i)) RF_PORT=$((5072+i)) RF_RTP=$((14000+i*100)) RF_PTY=/tmp/v90rf$i \
      ./tools/soak/rasfinder_call.sh artifacts/$TAG-r$r-$((6311+i)) 50 >/dev/null 2>&1 &
  done
  wait
  for i in 0 1 2; do d=artifacts/$TAG-r$r-$((6311+i))
    echo "== r$r $((6311+i)): $(grep -aE 'declined by peer|^CONNECT' $d/server.log | cut -c1-90 | tr '\n' '|')"
    grep -aE 'CONNECT [0-9]|V.34 B1:|V.90 data' $d/server.log | cut -c1-140 | head -3
  done
  [ "$r" -lt "$ROUNDS" ] && sleep 100
done
echo ALLDONE
