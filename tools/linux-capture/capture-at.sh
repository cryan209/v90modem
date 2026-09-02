#!/bin/sh
# Capture usbmon (bus 1, all endpoints) while driving the modem through AT
# commands on ttySHSF0.  The point is the BULK-OUT sequence that makes the part
# transmit, so this drives it as far as dialling.
OUT=${1:-/root/hsf/cap-at}
CMDS=${2:-"ATZ|ATI3|ATX3DT123|"}
mkdir -p "$OUT"
modprobe usbmon 2>/dev/null
mount -t debugfs none /sys/kernel/debug 2>/dev/null
cat /sys/kernel/debug/usb/usbmon/1u > "$OUT/usbmon.txt" &
MONPID=$!
sleep 1
stty -F /dev/ttySHSF0 115200 raw -echo
(cat /dev/ttySHSF0 > "$OUT/at.log" &) ; CATPID=$!
echo "$CMDS" | tr '|' '\n' | while read c; do
    [ -n "$c" ] && printf '%s\r' "$c" > /dev/ttySHSF0
    sleep 3
done
sleep 5
printf '+++' > /dev/ttySHSF0; sleep 2
printf 'ATH\r' > /dev/ttySHSF0; sleep 2
kill $MONPID 2>/dev/null
pkill -f "cat /dev/ttySHSF0" 2>/dev/null
sleep 1
echo "--- AT responses:"; cat "$OUT/at.log"
echo "--- usbmon lines: $(wc -l < "$OUT/usbmon.txt")"
echo "--- bulk lines:   $(grep -c ':1:00[0-9]:[0-9]' "$OUT/usbmon.txt")"
awk '{print $4}' "$OUT/usbmon.txt" | sort | uniq -c | sort -rn | head
