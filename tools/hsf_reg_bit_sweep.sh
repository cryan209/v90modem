#!/bin/sh
# Sweep the streaming control registers one bit at a time, during a live stream,
# watching whether transmit starts draining.
#
# The registers that differ between an idle and a streaming dump are 0x06/0x07
# (hardware-driven), 0x18-0x1b (the two moving pairs) and 0x1c-0x1f.  Only the
# last four are static and host-writable, so they are what is swept; their
# streaming values are 0xa0, 0xa0, 0x44, 0xb8 and their idle values 0x20, 0x20,
# 0x00, 0x3c.
#
# One bit at a time, ORed onto the streaming baseline, because writing 0xff to
# 0x1e during a live stream wedged the part.  Each arm is restored afterwards,
# since register writes PERSIST across runs and would otherwise pollute every
# later arm.
#
# The failure signature is tx == 2432 (or thereabouts): the FIFO fills once and
# stops.  Anything above ~10000 is transmit actually draining.  rx == 0 means
# the part has wedged and every later arm is void, so this aborts there.
set -u
P=./hsf_fxo_probe
R=tools/hsf_regdump.py
rc=0

live_write() {   # reg val -> prints rx/tx
	python3 "$R" --live --write "$1" "$2" 2>&1 | awk '/^   rx [0-9]/{gsub(",","",$8); printf "rx=%-8d tx=%-8d", $2+0, $8+0} /DEVICE WEDGED/{printf " WEDGED"}'
}
restore() { python3 "$R" --write "$1" "$2" >/dev/null 2>&1; }

for spec in "0x1c a0 20" "0x1d a0 20" "0x1e 44 00" "0x1f b8 3c"; do
	reg=$(echo $spec | cut -d' ' -f1)
	live=$(echo $spec | cut -d' ' -f2)
	idle=$(echo $spec | cut -d' ' -f3)
	for b in 01 02 04 08 10 20 40 80; do
		val=$(printf '%02x' $(( 0x$live | 0x$b )))
		[ "$val" = "$live" ] && continue          # bit already set
		printf '%s |= 0x%s -> 0x%s   ' "$reg" "$b" "$val"
		out=$(live_write "$reg" "0x$val")
		echo "$out"
		case "$out" in
			*WEDGED*|"") echo "ABORT: part wedged; replug, reload, resume here"; exit 1 ;;
		esac
		case "$out" in
			rx=0\ *) echo "ABORT: rx=0, everything after this is void"; exit 1 ;;
		esac
		restore "$reg" "0x$idle"
		sleep 1
	done
done
exit $rc
