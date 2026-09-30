#!/bin/zsh
#
# Two Apple USB Modems (05ac:1401) into a conference bridge, and a one-way
# tone test across it.  This measures the ANALOGUE path -- each modem's own
# 2-wire hybrid, the FXS port, the PBX and the bridge's mixer -- with no modem
# protocol involved at all, so a result here is about the bearer and nothing
# else.
#
#   usage: tools/apple_modem_pair_test.sh <out-dir> [number] [rate]
#
# Two selections have to be made and are NOT interchangeable: the USB device
# (APPLE_MODEM_ADDR=bus:addr, what the probe uses for the hook) and the
# CoreAudio device (APPLE_MODEM_AUDIO_UID, a substring of the UID, what the
# audio tool uses for the codec).  Map one to the other empirically -- go
# off-hook on one USB address and capture on each UID; the railed stream is
# the other modem.  `./apple_usb_modem_probe --descriptors` lists the USB
# addresses and `./apple_usb_modem_audio list` the UIDs.
#
# Run ./apple_usb_modem_probe --configure against EACH address first, or
# CoreAudio never sees that modem at all.
#
# DTMF level is not a free choice: one of the two FXS ports here rejects the
# 0.15 default and needs 0.35, so join() retries louder rather than reporting
# a dial that was never accepted.  The check is the 400 Hz dial tone being
# GONE, which is the only evidence available that a leg actually joined.
set -e
cd "$(dirname "$0")/.."
OUT=${1:?usage: $0 <out-dir> [number] [rate]}; NUM=${2:-2280}; RATE=${3:-9600}
mkdir -p "$OUT"

: ${A_USB:=1:6} ${A_UID:=1143000}     # modem A
: ${B_USB:=1:7} ${B_UID:=000000}      # modem B
: ${DIALTONE_HZ:=400}                 # continuous dial tone here is 400 Hz

hook()  { APPLE_MODEM_ADDR=$1 ./apple_usb_modem_probe --hook $2 2>&1 | grep "^hook"; }
level() { # level <file> <hz> -> dBFS of that line
  python3 - "$1" "$2" "$RATE" <<'PY'
import sys, struct, cmath, math
d = open(sys.argv[1],'rb').read(); f = float(sys.argv[2]); r = float(sys.argv[3])
x = [v/32768.0 for v in struct.unpack('<%dh' % (len(d)//2), d[:len(d)//2*2])]
m = sum(x)/len(x)
a = abs(sum((v-m)*cmath.exp(-2j*math.pi*f*n/r) for n, v in enumerate(x)))*2/len(x)
print("%.1f" % (20*math.log10(max(a, 1e-12))))
PY
}

join() {  # join <usb> <uid> <tag>
  local usb=$1 uid=$2 tag=$3 amp lvl
  hook $usb off >/dev/null 2>&1 || true; sleep 2
  hook $usb on; sleep 2
  for amp in 0.15 0.25 0.35; do
    APPLE_MODEM_TX_AMP=$amp APPLE_MODEM_AUDIO_UID=$uid \
      ./apple_usb_modem_audio dial $RATE $NUM "$OUT/${tag}_dial.s16" >/dev/null 2>&1
    sleep 3
    APPLE_MODEM_AUDIO_UID=$uid ./apple_usb_modem_audio capture $RATE 1 \
      "$OUT/${tag}_chk.s16" >/dev/null 2>&1
    lvl=$(level "$OUT/${tag}_chk.s16" $DIALTONE_HZ)
    echo "$tag: dial at amp $amp -> ${DIALTONE_HZ} Hz now $lvl dBFS"
    [[ ${lvl%%.*} -lt -30 ]] && { echo "$tag joined"; return 0; }
    hook $usb off >/dev/null 2>&1; sleep 3; hook $usb on >/dev/null 2>&1; sleep 2
  done
  echo "$tag FAILED to join $NUM"; return 1
}

trap 'hook $A_USB off >/dev/null 2>&1; hook $B_USB off >/dev/null 2>&1' EXIT

join $A_USB $A_UID A
join $B_USB $B_UID B

echo "== baseline: both silent =="
APPLE_MODEM_AUDIO_UID=$A_UID ./apple_usb_modem_audio capture $RATE 2 "$OUT/base_A.s16" >/dev/null 2>&1 &
APPLE_MODEM_AUDIO_UID=$B_UID ./apple_usb_modem_audio capture $RATE 2 "$OUT/base_B.s16" >/dev/null 2>&1 &
wait

for f in 300 1000 2000 3000; do
  echo "== A -> B at $f Hz =="
  ( APPLE_MODEM_AUDIO_UID=$A_UID ./apple_usb_modem_audio tone $RATE $f 2 "$OUT/txA_$f.s16" >/dev/null 2>&1 ) &
  ( APPLE_MODEM_AUDIO_UID=$B_UID ./apple_usb_modem_audio capture $RATE 3 "$OUT/rxB_$f.s16" >/dev/null 2>&1 ) &
  wait; sleep 1
  echo "== B -> A at $f Hz =="
  ( APPLE_MODEM_AUDIO_UID=$B_UID ./apple_usb_modem_audio tone $RATE $f 2 "$OUT/txB_$f.s16" >/dev/null 2>&1 ) &
  ( APPLE_MODEM_AUDIO_UID=$A_UID ./apple_usb_modem_audio capture $RATE 3 "$OUT/rxA_$f.s16" >/dev/null 2>&1 ) &
  wait; sleep 1
done

echo "== level linearity, A -> B at 1000 Hz =="
for amp in 0.02 0.05 0.15 0.35; do
  ( APPLE_MODEM_TX_AMP=$amp APPLE_MODEM_AUDIO_UID=$A_UID ./apple_usb_modem_audio tone $RATE 1000 2 "$OUT/lin_tx_$amp.s16" >/dev/null 2>&1 ) &
  ( APPLE_MODEM_AUDIO_UID=$B_UID ./apple_usb_modem_audio capture $RATE 3 "$OUT/lin_rx_$amp.s16" >/dev/null 2>&1 ) &
  wait; sleep 1
done

echo "== double talk: A 1000 Hz and B 1400 Hz together =="
( APPLE_MODEM_AUDIO_UID=$A_UID ./apple_usb_modem_audio tone $RATE 1000 2 "$OUT/dt_A.s16" >/dev/null 2>&1 ) &
( APPLE_MODEM_AUDIO_UID=$B_UID ./apple_usb_modem_audio tone $RATE 1400 2 "$OUT/dt_B.s16" >/dev/null 2>&1 ) &
wait
echo "== done; analyse with tools/apple_modem_tone_report.py =="
