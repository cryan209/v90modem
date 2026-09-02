#!/bin/sh
# Definitive capture: starts BEFORE the driver loads, so it carries the whole
# lifecycle -- enumeration, any firmware download, interface claiming, the
# CD2_CONTROL_SCRIPT bring-up, and the bulk datapump on a live dial.
OUT=${1:-/root/hsf/cap-full}
mkdir -p "$OUT"
modprobe usbmon 2>/dev/null
for m in hsfusbcd2 hsfsoar hsfserial hsfengine hsfosspec; do rmmod $m 2>/dev/null; done
sleep 2
tcpdump -i usbmon1 -s 0 -w "$OUT/usb.pcap" >/dev/null 2>&1 &
TPID=$!
sleep 2
cd /root/hsf/hsfmodem-linux/modules
for m in hsfosspec hsfengine hsfserial hsfsoar hsfusbcd2; do insmod ./$m.ko; done
sleep 5
/root/hsf/usbreset /dev/bus/usb/$(lsusb | grep 0572:1300 | sed 's/Bus \([0-9]*\) Device \([0-9]*\).*/\1\/\2/') || true
sleep 6
stty -F /dev/ttySHSF0 115200 raw -echo
cat /dev/ttySHSF0 > "$OUT/at.log" &
for c in ATZ ATI3 ATX3DT123; do printf '%s\r' "$c" > /dev/ttySHSF0; sleep 4; done
sleep 6
printf '+++' > /dev/ttySHSF0; sleep 2; printf 'ATH\r' > /dev/ttySHSF0; sleep 2
kill $TPID 2>/dev/null; pkill -f "cat /dev/ttySHSF0" 2>/dev/null
sleep 1
echo "--- AT:"; cat "$OUT/at.log"; ls -la "$OUT/usb.pcap"
