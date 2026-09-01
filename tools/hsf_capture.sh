#!/bin/sh
# Capture the Conexant HSF USB modem's wire traffic, one action per file.
#
# Run this INSIDE an x86 Linux guest with the modem passed through and the
# vendor hsfmodem driver installed.  The point of one-file-per-action is
# attribution: a single long capture makes you guess which bytes belong to
# which operation, and guessing is what this whole exercise is trying to stop.
#
# The capture only needs the CONTROL PLANE, so the guest does not need to be
# fast -- emulated x86 on Apple Silicon is fine.  The soft modem's datapump
# will underrun and it does not matter; going off-hook still emits the script.
#
#   ./hsf_capture.sh /dev/ttySHSF0 outdir
#
# Then copy outdir back to the Mac and run tools/hsf_usbmon.py over it.
set -e

TTY=${1:-/dev/ttySHSF0}
OUT=${2:-hsf-capture}
VID=0572
PID=1300

mkdir -p "$OUT"

modprobe usbmon 2>/dev/null || true
MON=/sys/kernel/debug/usb/usbmon
[ -d "$MON" ] || { echo "usbmon not mounted; try: mount -t debugfs none /sys/kernel/debug"; exit 1; }

BUS=$(lsusb | sed -n "s/^Bus \([0-9]*\) Device [0-9]*: ID $VID:$PID.*/\1/p" | head -1)
[ -n "$BUS" ] || { echo "modem $VID:$PID not found on the bus"; exit 1; }
BUS=$(printf %d "$BUS")
echo "modem on bus $BUS, capturing $MON/${BUS}u"

cap() {                       # cap <name> <seconds> <shell to run>
    name=$1; secs=$2; shift 2
    echo
    echo "=== $name (${secs}s) ==="
    cat "$MON/${BUS}u" > "$OUT/$name.mon" &
    catpid=$!
    sleep 1
    sh -c "$*" || true
    sleep "$secs"
    kill $catpid 2>/dev/null || true
    wait $catpid 2>/dev/null || true
    echo "  $(wc -l < "$OUT/$name.mon") lines -> $OUT/$name.mon"
}

at() { printf '%s\r' "$1" > "$TTY"; sleep 2; }

echo
echo "Unplug the modem now, then press enter -- the plug event carries the"
echo "firmware upload and the initial script load, which anchors everything."
read _
cat "$MON/${BUS}u" > "$OUT/00-plugin.mon" &
catpid=$!
echo "Plug the modem back in, wait for the driver to bind, then press enter."
read _
kill $catpid 2>/dev/null || true
echo "  $(wc -l < "$OUT/00-plugin.mon") lines -> $OUT/00-plugin.mon"

# One action per capture, with dead air either side so the boundaries are clean.
cap 01-idle      5  "true"
cap 02-offhook   5  "at ATH1"
cap 03-onhook    5  "at ATH0"
cap 04-offhook2  5  "at ATH1"
cap 05-onhook2   5  "at ATH0"
cap 06-dial      8  "at 'ATDT12345'"

echo
echo "=== 07-ring (20s) ==="
echo "CALL THE LINE NOW -- a ring while on-hook is the notification we most need."
cat "$MON/${BUS}u" > "$OUT/07-ring.mon" &
catpid=$!
sleep 20
kill $catpid 2>/dev/null || true
echo "  $(wc -l < "$OUT/07-ring.mon") lines -> $OUT/07-ring.mon"

echo
echo "done.  Copy $OUT back to the Mac:"
echo "  python3 tools/hsf_usbmon.py $OUT/*.mon"
