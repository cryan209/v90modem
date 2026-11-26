#!/bin/bash
# dmodem_matrix.sh <outdir> [row regex]: D-Modem's interop peers (the callers)
# against half_v90modem (the answerer; the caller for the "we" rows), three calls at a time, through
# D-Modem's interop_call and netsim (docs/dmodem_interop.md).  Each row's line
# and AT commands are those of D-Modem's own row for that peer
# (dsp/test/interop_test.cpp), with our AT commands as the answerer.
#   DMODEM_TEST=<D-Modem build>/dsp/test   (default /root/D-Modem/build/dsp/test)
#   HALF_V90MODEM=<path>                   (default: this checkout's)
#   RT=1                                   pace the calls in real time
# Prints one line per row: interop_call's status, the caller's CONNECT, the
# data check, and whether our PCM-upstream receiver sent a precoded CPd and
# locked B1u.
OUT=${1:?usage: $0 <outdir> [row regex]}; FILT=${2:-.}
T=${DMODEM_TEST:-/root/D-Modem/build/dsp/test}
US=${HALF_V90MODEM:-$(cd "$(dirname "$0")/.." && pwd)/half_v90modem}
V92Q="ATX3;AT+MS=V92;AT+PIG=0;AT+PQC=0"
LOOP="NET_LOOP=0.45"
# The emulated Courier's V.34/V.90 does not work even against D-Modem's own
# modem, so its rows are V.32bis and V.22bis, picked by S registers.
CV32="ATS34=128S56=224S58=33"; CV22="ATS27=4S56=224S58=33&N3"
rows=(
"sl_v92_ulaw|smartlink|ATX3;AT+MS=92|$V92Q|NET=9600:8000:ulaw SMARTLINK_V92FULL=1"
"sl_v92_alaw|smartlink|ATX3;AT+MS=92|$V92Q|NET=9600:8000:alaw SMARTLINK_V92FULL=1"
"sl_v90_ulaw|smartlink|ATX3;AT+MS=90|ATX3|NET=9600:8000:ulaw NET_FLOOR=3"
"hsf_v92_ulaw|hsf|ATX3;AT+MS=V92|$V92Q|$LOOP NET=8000:8000:ulaw"
"hsf_v92_alaw|hsf|ATX3;AT+MS=V92|$V92Q|$LOOP NET=8000:8000:alaw"
"hsf_v90_ulaw|hsf|ATX3;AT+MS=V90|ATX3|$LOOP NET=8000:8000:ulaw"
"ess_v92_ulaw|ess|ATX3;AT+PIG=0|$V92Q|$LOOP NET=8000:8000:ulaw"
"ess_v92_alaw|ess|ATX3;AT+PIG=0|$V92Q|$LOOP NET=8000:8000:alaw"
"ess_v90_ulaw|ess|ATX3;AT+MS=V90|ATX3|$LOOP NET=8000:8000:ulaw"
"agere_v92_ulaw|agere|ATX3;AT+MS=V92;AT+PQC=0|$V92Q|$LOOP NET=8000:8000:ulaw"
"agere_v90_ulaw|agere|ATX3;AT+MS=V90|ATX3|$LOOP NET=8000:8000:ulaw"
"sm56_v90_ulaw|sm56|ATX3;AT+MS=V90|ATX3|$LOOP NET=8000:8000:ulaw"
"pctel_v90_ulaw|pctel|ATX3;AT+MS=V90,1,300,56000|ATX3|$LOOP NET=8000:8000:ulaw"
"broadcom_v92_ulaw|broadcom|ATX3;AT+PIG=0;AT+MS=V92|$V92Q|$LOOP NET=8000:8000:ulaw"
"broadcom_v90_ulaw|broadcom|ATX3;AT+MS=V90|ATX3|$LOOP NET=8000:8000:ulaw"
"intel_v92_ulaw|intel|ATX3;AT+PIG=0|$V92Q|INTEL_RATE=9600 $LOOP NET=9600:8000:ulaw"
"intel_v90_ulaw|intel|ATX3;AT+MS=V90|ATX3|INTEL_RATE=9600 $LOOP NET=9600:8000:ulaw"
"courier_v32b_ulaw|courier|$CV32|ATX3;AT+MS=V32B|$LOOP NET=8000:8000:ulaw"
"courier_v22b_ulaw|courier|$CV22|ATX3;AT+MS=V22B|$LOOP NET=8000:8000:ulaw"
"we_courier_v32b_ulaw|courier|ATX3;AT+MS=V32B|$CV32|$LOOP NET=8000:8000:ulaw|we"
"we_courier_v22b_ulaw|courier|ATX3;AT+MS=V22B|$CV22|$LOOP NET=8000:8000:ulaw|we"
)
run() {
  IFS='|' read -r name peer pa pb envs dir <<< "$1"
  D=$OUT/$name; rm -rf $D; mkdir -p $D
  # dir "we": we are the caller (A) and dial with DIAL=1, since we answer a
  # bare ATD with ERROR; pa is then our AT commands and pb the peer's.
  if [ "$dir" = we ]; then A=$US; B=./half_$peer; envs="$envs DIAL=1"; else A=./half_$peer; B=$US; fi
  ( cd $T && env $envs STREAM=10 V90MODEM_VERBOSE=1 ${RT:+MODEM_REALTIME=1} PROTO_B="$pb" LOG_DIR=$D NETSIM=$T/netsim \
      timeout 420 nice -n10 ./interop_call $A $B "$pa" > $D/driver.log 2>&1; echo $? > $D/rc )
  rc=$(cat $D/rc)
  ca=$(grep -a -o -E '^A: .*CONNECT[^\\]*' $D/driver.log | grep -o -E 'CONNECT[^\\]*' | head -1)
  data=$(grep -a -E '^[AB] received' $D/driver.log | tr '\n' ' ')
  mode=$(grep -a -o -E 'V\.92 PCM-upstream B1u receiver armed[^,]*, drn=[0-9]+ rate=[0-9]+|B1u locked by decoding|CPd precoder' $D/modemB.log | sort -u | tr '\n' ';')
  echo "$name rc=$rc A:[$ca] $data | $mode"
}
export -f run; export OUT T US
printf '%s\n' "${rows[@]}" | grep -E "$FILT" | xargs -P3 -I{} bash -c 'run "$@"' _ {}
echo MATRIX-DONE
