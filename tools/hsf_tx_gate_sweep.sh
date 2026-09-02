#!/bin/sh
# Sweep the hsfusbcd2261_ path codes (and the script-8/12 ordering) looking for
# the one that makes bulk OUT drain instead of filling once and stopping.
#
# The failure signature is a fixed TX byte count -- 2432 with 128-byte blocks --
# repeated whatever else changes.  Anything OTHER than a constant is the result;
# a run that merely completes proves nothing, since script 12 completes
# (data=0c01) in the arm already refuted live.
#
# Codes come from hsfusbcd2261_: ordinary part 0->0x02, 1->0x01; the ctx+0x68
# variant 0->0x14, 1->0x13, 2->0x12, 3->0x17.  0x17 post-session is the control
# and is known to leave TX at 2432.
#
# THE HEALTH CHECK IS NOT OPTIONAL.  This part can degrade into a state where
# the codec produces no audio at all while EP0 still answers, scripts 9 and 5
# still complete and report, and TX still fills its FIFO -- so every per-run
# signal except RX bytes stays healthy-looking.  It does not recover without a
# replug and a firmware reload, and every arm after that point measures run
# order rather than the setting, so this aborts there instead of printing a
# table.  The known cause was the probe abandoning each session; script 6 is
# now sent by default and four consecutive runs hold ~127400 RX bytes.  The
# check stays because it is what caught that, and a silent RX is invisible in
# every other per-run signal.
set -u
P=./hsf_fxo_probe
SECS=${SECS:-3}
rc=0

run() {
	printf '%-52s ' "$*"
	out=$("$@" 2>&1)
	line=$(printf '%s\n' "$out" | grep '^rx [0-9]')
	if [ -z "$line" ]; then
		echo "NO STREAM LINE -- device not answering"
		rc=1
		return 1
	fi
	rx=$(printf '%s\n' "$line" | awk '{print $2}')
	tx=$(printf '%s\n' "$line" | awk '{gsub(",","",$8); print $8}')
	printf 'rx=%-8s tx=%s\n' "$rx" "$tx"
	if [ "$rx" -eq 0 ]; then
		echo
		echo "ABORTING: rx=0, so the codec has stopped.  Everything from here"
		echo "would measure the degradation, not the setting.  Replug, reload"
		echo "firmware, and resume from this arm."
		rc=1
		return 1
	fi
	sleep 1
	return 0
}

for code in 0x02 0x01 0x14 0x13 0x12 0x17; do
	# --post-script: after the 9/5 session bring-up, which is the order
	# hsfusbcd2167_ implies (it is a stream OPEN on a live session).
	run $P --start-codec --feed --dtmf 5 --stream $SECS \
		--post-script 8,12 --post-patch $code,$code || exit $rc
	# --script: before the session, after the ring is primed.  Included
	# because "stream open" and "session up" are our reading, not the
	# device's, and the two orders are one flag apart.
	run $P --start-codec --feed --dtmf 5 --stream $SECS \
		--script 8,12 --patch $code,$code || exit $rc
done
# Script 12 alone, in case script 8 (the tone/cadence script) is disturbing it.
for code in 0x02 0x01 0x17; do
	run $P --start-codec --feed --dtmf 5 --stream $SECS \
		--post-script 12 --post-patch $code,$code || exit $rc
done
exit $rc
