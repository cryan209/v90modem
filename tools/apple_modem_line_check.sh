#!/bin/zsh
#
# One-command health check for an Apple USB Modem's analogue line: go
# off-hook, transmit a tone into the dial tone, and measure how much of it
# comes straight back through the 2-wire hybrid.
#
#   usage: tools/apple_modem_line_check.sh <usb bus:addr> <audio uid substring>
#
# A healthy port here returns the tone about 24 dB down.  A port whose pair
# has gone open, high-impedance or badly terminated returns nearly all of it
# -- 1 to 7 dB down -- and the symptom further up is that the exchange stops
# detecting that modem's DTMF, so dialling fails intermittently and then not
# at all.  Nothing else in this tree sees that: the line still has dial tone,
# `--read 1d` still says "pair present", and the modem still transmits.
set -e
cd "$(dirname "$0")/.."
USB=${1:?usage: $0 <bus:addr> <uid-substring>}; U=${2:?}
RATE=9600; AMP=0.35          # -9.1 dBFS per tone
TMP=$(mktemp -t applemodem).s16
trap 'APPLE_MODEM_ADDR=$USB ./apple_usb_modem_probe --hook off >/dev/null 2>&1; rm -f "$TMP"' EXIT

APPLE_MODEM_ADDR=$USB ./apple_usb_modem_probe --hook on 2>&1 | grep "^hook"
sleep 2
APPLE_MODEM_TX_AMP=$AMP APPLE_MODEM_AUDIO_UID=$U \
    ./apple_usb_modem_audio tone $RATE 1000 2 "$TMP" >/dev/null 2>&1
python3 - "$TMP" "$AMP" <<'PY'
import sys, struct, math, cmath
rate, f = 9600.0, 1000.0
d = open(sys.argv[1], 'rb').read()
x = [v/32768.0 for v in struct.unpack('<%dh' % (len(d)//2), d[:len(d)//2*2])]
x = x[int(1.0*rate):int(2.0*rate)]
m = sum(x)/len(x)
a = abs(sum((v-m)*cmath.exp(-2j*math.pi*f*n/rate) for n, v in enumerate(x)))*2/len(x)
tx = 20*math.log10(float(sys.argv[2])); rx = 20*math.log10(max(a, 1e-12))
erl = tx - rx
print("  transmitted %+.1f dBFS, returned %+.1f dBFS -> echo return loss %.1f dB" % (tx, rx, erl))
print("  %s" % ("healthy" if erl > 15 else
                "BAD -- the pair looks open or badly terminated; expect DTMF to fail"))
PY
