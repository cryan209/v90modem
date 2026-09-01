#!/bin/sh
# V.34 fax interop, stage 1: what does a real fax machine offer in its V.8 CM?
#
# For a V.34 fax the CALLING machine sends CNG, hears our ANSam, and then sends
# a V.8 CM naming its call function ("T.30 Tx FAX") and the modulations it
# supports.  That single frame settles, before any code is written to act on
# it, whether the machine offers V.34 fax over this SIP/ATA path at all --
# Super G3 is routinely lost to an ATA's own fax handling or to transcoding.
#
# Nothing here needs the fax service classes or T.30 Annex F.  The server
# answers as an ordinary data modem; the point is only to present ANSam so the
# CM is transmitted, and to record it.  Our V.8 already parses and logs both
# fields (v8_call_function_to_str(), v8_log_supported_modulations()), so no
# offline decode is needed for the first look.
#
#   tools/fax_v8_probe.sh <label> [seconds] [extension]
#
# then dial that extension from the fax machine.  The G.711 taps land beside
# the log, so the call is reproducible offline whatever the log says.

set -e
LABEL="${1:-faxprobe}"
SECS="${2:-120}"
EXT="${3:-6000}"
SIP_SERVER="${SIP_SERVER:-asterisk.net.cryan.nz}"
SIP_USER="${SIP_USER:-$EXT}"
SIP_PASS="${SIP_PASS:-$EXT}"
LOCAL_PORT="${LOCAL_PORT:-5064}"
RTP_PORT="${RTP_PORT:-41000}"

cd "$(dirname "$0")/.."

# Guard the PORT, not the process.  There is deliberately more than one server
# on this machine -- a second extension is how a fax probe runs beside a data
# soak -- so "is a sip_v90_modem running" is the wrong question.  A stray
# server holding OUR SIP port is the failure that matters: every call then dies
# with a tiny log and no call at all, which reads exactly like the far end
# refusing to connect.
if lsof -nP -iUDP:"$LOCAL_PORT" >/dev/null 2>&1; then
    echo "fax_v8_probe: UDP $LOCAL_PORT is already in use; pick another with LOCAL_PORT=" >&2
    lsof -nP -iUDP:"$LOCAL_PORT" >&2
    exit 1
fi

# ME_V34_SPAN_FLOW_LOG=1 is what turns the SpanDSP V.8 FLOW messages on; at the
# default level the CM contents are never printed.  SIP_FORCE_PCMU=1 keeps the
# bearer a byte-exact G.711 stream -- a transcoded path cannot carry V.34 fax
# and would make a negative result meaningless.
echo "fax_v8_probe: registering as $SIP_USER on $SIP_SERVER (SIP $LOCAL_PORT, RTP $RTP_PORT)"
echo "fax_v8_probe: dial $EXT from the fax machine"
ME_V34_SPAN_FLOW_LOG=1 \
SIP_FORCE_PCMU=1 \
exec python3 tools/v90_hardware_interop.py \
    --label "$LABEL" \
    --duration "$SECS" \
    -- \
    --sip-server "$SIP_SERVER" \
    --username "$SIP_USER" \
    --password "$SIP_PASS" \
    --local-port "$LOCAL_PORT" \
    --rtp-port "$RTP_PORT" \
    --pty-link /tmp/v90faxprobe
