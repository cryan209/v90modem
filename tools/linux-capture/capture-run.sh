#!/bin/sh
# Run inside the wheezy capture guest.  Starts usbmon FIRST, then loads the
# vendor stack, then resets the device -- so the driver is already waiting when
# the part re-enumerates and its ~3 s bootloader window opens.
set -e
OUT=${1:-/root/hsf/cap}
mkdir -p "$OUT"
modprobe usbmon 2>/dev/null || true
mount -t debugfs none /sys/kernel/debug 2>/dev/null || true
BUS=$(lsusb | grep 0572:1300 | sed 's/Bus \([0-9]*\).*/\1/' | sed 's/^0*//')
echo "modem on bus $BUS"
cat /sys/kernel/debug/usb/usbmon/${BUS}u > "$OUT/usbmon.txt" &
MONPID=$!
sleep 1
cd /root/hsf/hsfmodem-linux/modules
for m in hsfosspec hsfengine hsfserial hsfsoar hsfusbcd2; do
    lsmod | grep -q "^$m " || insmod ./$m.ko
done
sleep 2
DEV=/dev/bus/usb/$(lsusb | grep 0572:1300 | sed 's/Bus \([0-9]*\) Device \([0-9]*\).*/\1\/\2/')
echo "resetting $DEV"
/root/hsf/usbreset "$DEV" || true
sleep 8
kill $MONPID 2>/dev/null || true
sleep 1
echo "captured $(wc -l < "$OUT/usbmon.txt") usbmon lines"
dmesg | tail -25
