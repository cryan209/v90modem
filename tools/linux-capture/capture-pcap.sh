#!/bin/sh
# Full-payload usbmon capture (the text interface truncates data at 32 bytes,
# which loses both the tail of a script body and all of the bulk audio).
# Writes a pcap; decode with tools/hsf_pcap.py.
OUT=${1:-/root/hsf/cap-pcap}
CMDS=${2:-"ATZ|ATI3|ATX3DT123|"}
mkdir -p "$OUT"
modprobe usbmon 2>/dev/null
tcpdump -i usbmon1 -s 0 -w "$OUT/usb.pcap" >/dev/null 2>&1 &
TPID=$!
sleep 2
stty -F /dev/ttySHSF0 115200 raw -echo
cat /dev/ttySHSF0 > "$OUT/at.log" &
echo "$CMDS" | tr '|' '\n' | while read c; do
    [ -n "$c" ] && printf '%s\r' "$c" > /dev/ttySHSF0
    sleep 3
done
sleep 5
printf '+++' > /dev/ttySHSF0; sleep 2
printf 'ATH\r' > /dev/ttySHSF0; sleep 2
kill $TPID 2>/dev/null; pkill -f "cat /dev/ttySHSF0" 2>/dev/null
sleep 1
echo "--- AT responses:"; cat "$OUT/at.log"
ls -la "$OUT/usb.pcap"
