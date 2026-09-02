#!/bin/sh
# Sweep the hsfusbcd2261_ path codes (and the script-8/12 ordering) looking for
# the one that makes bulk OUT drain instead of filling once and stopping.
#
# The failure signature is a fixed TX byte count -- 2432 with 128-byte blocks --
# repeated whatever else changes.  Anything OTHER than a constant here is the
# result; a run that merely completes proves nothing, since script 12 completes
# (data=0c01) in the arm that does not work.
#
# Codes come from hsfusbcd2261_: ordinary part 0->0x02, 1->0x01; the ctx+0x68
# variant 0->0x14, 1->0x13, 2->0x12, 3->0x17.  0x17 post-prime is already known
# to leave TX at 2432 and is included as the control.
set -u
P=./hsf_fxo_probe
SECS=${SECS:-5}
run() {
	printf '%-44s ' "$*"
	"$@" 2>&1 | awk '/^rx [0-9]/ {print "tx="$6" rx="$2; found=1} END {if(!found) print "NO STREAM LINE"}'
	sleep 1
}
for code in 0x02 0x01 0x14 0x13 0x12 0x17; do
	# --post-script: after the 9/5 session bring-up, which is the order
	# hsfusbcd2167_ implies (it is a stream OPEN on a live session).
	run $P --start-codec --feed --dtmf 5 --stream $SECS \
		--post-script 8,12 --post-patch $code,$code
	# --script: before the session, after the ring is primed.  Included
	# because "stream open" and "session up" are our reading, not the
	# device's, and the two orders are one flag apart.
	run $P --start-codec --feed --dtmf 5 --stream $SECS \
		--script 8,12 --patch $code,$code
done
# Script 12 alone, in case script 8 (the tone/cadence script) is disturbing it.
for code in 0x02 0x01 0x17; do
	run $P --start-codec --feed --dtmf 5 --stream $SECS \
		--post-script 12 --post-patch $code,$code
done
