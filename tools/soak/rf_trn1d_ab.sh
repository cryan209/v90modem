#!/bin/bash
# 3 lanes in parallel, one TRN1d length per lane, arms rotated over extensions each round.
cd /root/v90modem || exit 1
ARMS=(12000 16008 20004)
for r in $(seq 1 8); do
  for i in 0 1 2; do
    arm=${ARMS[$(( (i+r) % 3 ))]}
    ME_V90_TRN1D_SYMBOLS=$arm RF_EXT=$((6311+i)) RF_USER=$((2900+i)) RF_PORT=$((5072+i)) RF_RTP=$((14000+i*100)) RF_PTY=/tmp/v90rf$i \
      ./tools/soak/rasfinder_call.sh /root/trn/t$arm-r$r-$((6311+i)) 50 >/dev/null 2>&1 &
  done
  wait
  [ "$r" -lt 8 ] && sleep 100
done
echo TRNDONE
