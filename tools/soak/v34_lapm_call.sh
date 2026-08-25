#!/bin/bash
# One plain-V.34 (no V.90) call from the d-modem rig, held long enough for
# V.42 LAPM to establish.  The peer is forced to V.34 only with automode 0,
# so nothing here depends on our V.8 offer suppressing V.90.
#
#   v34_lapm_call.sh <server.log> [hold-seconds] [serial-out]
set -u
TOWER=${TOWER:-root@tower.net.cryan.nz}
SERVERLOG=${1:?usage: v34_lapm_call.sh server.log [hold-sec] [serial-out]}
HOLD=${2:-120}
SERIAL_OUT=${3:-/tmp/v34-lapm-serial.out}
SLMODEMD=${SLMODEMD:-/src/slmodemd/slmodemd}
# The rig's own loop model pads its transmit by this factor before the ZOH
# staircase and 4400 Hz lowpass.  0.25 is -12 dB and is what the V.90 work
# settled on; it is also most of why this rig's channel measures 16.8 dB and
# will not carry a dense constellation.
HEADROOM=${HEADROOM:-0.25}
# The rig's loop lowpass cutoff, 3500..4000 Hz.  It matters above 2400 baud:
# 3000 baud at the low carrier occupies up to 3480 Hz and 3200 baud up to
# 3621, so the default cutoff removes the top of the band and leaves a notch
# no equalizer can invert.
DM_RS_FC=${DM_RS_FC:-}
# The rig's upstream (DSP 9600 -> net 8000) polyphase resampler.  This is the
# one in our receive path, and its kernel length is the ceiling on what we can
# be asked to decode: a 6:5 polyphase resampler whose kernel is too short has
# five branches with different responses, which is a periodically time-varying
# filter and so exactly the part no LTI equalizer at our end can remove.
# Defaults in the rig are 32 taps at 3700 Hz.
DM_UP_TAPS=${DM_UP_TAPS:-}
DM_UP_FC=${DM_UP_FC:-}
MS=${MS:-34,0,2400,33600}
# Error-control mode for the peer's DTE interface.  \N0 is normal (no error
# correction) -- it is what the V.90 V.14 soak needs, and with it the peer
# never runs V.42's detection phase, so a V.42 call against it correctly
# reports an unsupported peer.  \N3 is auto-reliable: LAPM if the far end
# offers it, buffered if not.
# printf's %s does not interpret escapes, so this must hold ONE backslash:
# '\\N3' sends the literal AT\\N3, which the peer answers with ERROR (seen on
# every call until 2026-08-26).
NPARM=${NPARM:-'\N3'}

echo "CONTROL: bouncing rig ($SLMODEMD, AT+MS=$MS)"
ssh -o BatchMode=yes "$TOWER" "docker restart d-modem" >/dev/null 2>&1
sleep 8
ssh -o BatchMode=yes "$TOWER" "docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz -e DM_RESAMPLER=sinc -e DM_RS_HEADROOM=$HEADROOM ${DM_RS_FC:+-e DM_RS_FC=$DM_RS_FC} ${DM_UP_TAPS:+-e DM_UP_TAPS=$DM_UP_TAPS} ${DM_UP_FC:+-e DM_UP_FC=$DM_UP_FC} d-modem sh -c '$SLMODEMD -d9 -e /src/d-modem > /tmp/slm.log 2>&1'" 2>/dev/null
sleep 12

off=$(stat -f %z "$SERVERLOG" 2>/dev/null); off=${off:-0}
(
  printf 'AT\r';   sleep 2
  printf 'ATZ\r';  sleep 2
  printf 'ATX3\r'; sleep 1
  printf 'ATE1V1Q0\r'; sleep 1
  printf 'AT%s\r' "$NPARM"; sleep 1
  printf 'AT+MS=%s\r' "$MS"; sleep 2
  printf 'AT+MS?\r'; sleep 2
  printf 'ATD6001\r'
  sleep "$HOLD"
) | ssh -o BatchMode=yes "$TOWER" docker exec -i d-modem socat /dev/ttySL0,raw,echo=0,b115200 - >"$SERIAL_OUT" 2>&1 &
dialpid=$!

outcome=none
for tick in $(seq 1 $(( HOLD / 5 )) ); do
    sleep 5
    slice=$(tail -c "+$((off+1))" "$SERVERLOG" 2>/dev/null)
    # KEEP=1 records the LAPM connection but holds the call for the full hold
    # time, so a payload test has a link to run over: killing the dial closes
    # the peer's serial, which drops its DTR and clears the call.
    if echo "$slice" | grep -aq "V.42 LAPM connected"; then
        outcome=LAPM
        [ -n "${KEEP:-}" ] || break
    fi
    if echo "$slice" | grep -aqE "V.42 (detection reported unsupported|LAPM link error|LAPM disconnected)"; then
        outcome=LAPM_FAILED; break
    fi
done
kill "$dialpid" 2>/dev/null || true
wait "$dialpid" 2>/dev/null || true
ssh -o BatchMode=yes "$TOWER" "docker exec d-modem sh -c 'cat /tmp/slm.log'" >"${SERIAL_OUT%.out}.peer.log" 2>/dev/null
echo "CONTROL: outcome=$outcome serial=$SERIAL_OUT peer=${SERIAL_OUT%.out}.peer.log"
[ "$outcome" = LAPM ]
