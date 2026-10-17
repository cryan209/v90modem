#!/bin/bash
# N solo RasFinder calls, 100 s apart, rotating 6311-6313.  usage: rf_serial.sh <tag> [n]
cd "$(dirname "$0")/../.." || exit 1
TAG=$1; N=${2:-6}
for i in $(seq 1 "$N"); do
  e=$((6311+(i-1)%3))
  RF_EXT=$e ./tools/soak/rasfinder_call.sh artifacts/$TAG-$i-$e 50 >/dev/null 2>&1
  [ "$i" -lt "$N" ] && sleep 100
done
echo SERIALDONE
