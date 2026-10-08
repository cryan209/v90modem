# Eicon upstream corruption: packet-boundary isolation

In `artifacts/eicon-rtp-boundary-20261008/`, simultaneous Asterisk Ethernet
RTP and Eicon Audio1 capture accompanies a V.90 PCMU call. One 201-byte echo
passes; the second fails and the Eicon initiates retraining.

Our RTP transport records 1959 outgoing packets with no sequence/timestamp
gaps; maximum send interval is 26.021 ms. Asterisk receives 1948, with 11
missing packets in three runs: local TX file seconds 6.66–6.74, 18.96–19.02
and 29.40–29.48. It also receives one reordered pair near startup. All 1948
received payloads match the corresponding local TX tap bytes exactly.
Asterisk forwards all those payloads unchanged to the Cisco, preserving
the same missing sequence numbers. The tcpdump capture reports zero kernel
capture drops. Downstream has no sequence gaps at Asterisk.

The local route to Asterisk uses utun8, source 10.69.220.8, the connected
cryanfw WireGuard tunnel. The trace currently narrows missing packets to the
interval between our transport send instrumentation and Asterisk ingress;
it does not prove UDP send success or which tunnel/network component drops
them. G.711 modem audio cannot tolerate this missing waveform.

Card audio at the established initial offset of 987 samples first differs
from our waveform at local TX 18.957250, immediately before the three-packet
loss. Later offset changes and card CRC errors are also observed. Error
indications can lag acquisition damage; their exact timing alone does not
prove which loss burst caused a particular frame error.

The independent raw-symbol check on the preceding call matched all 58640
symbols, and its outgoing 402 complete HDLC frames were valid. This is useful
transmitter evidence, but no complete conformance or hardware-success claim.

A separate current-source Linux build in the existing LAN test container is
being prepared to remove the WireGuard media path without changing the
running modem, gateway, Eicon or Asterisk configuration.

## LAN comparison, current source

The isolated Linux build is `/tmp/eicon-current-20261008` inside the existing
`v90modem-sip` test container on tower. It reuses Linux pjproject dependencies,
rebuilds SpanDSP cleanly, and builds the modem from copied current sources.
SHA-256 matches were checked for modem_engine.c, sip_modem.c, data_stack.c,
v90_analogue_tx.c and v34tx.c. Existing modem processes were left running.

`artifacts/eicon-lan-20261008/` passes all ten exact 201-byte echoes, a total
2010 payload bytes, at 56000 downstream / 31200 upstream. The RTP route is
192.168.88.3 -> Asterisk 192.168.88.122 -> Cisco 192.168.88.62. All 4257
upstream packets arrive at Asterisk, with no sequence/timestamp gaps and no
payload discrepancies from the local tap. The forwarded upstream also has
4257 packets with no gaps. Eicon records zero wrong-CRC indications.

Card input matches all 536875 decoded linear samples from local TX 18.000
through 85.109375 seconds, at a constant 585-sample offset. The only 182
codeword changes are the two representations of zero. No waveform damage
or timing shift is present in that interval. This is hardware interop
confirmation for the tested V.90 PCMU call, rather than a loopback result.
It does not establish native V.34 interop or every rate/law combination.

## Two-ended tunnel capture confirms post-send loss

`artifacts/eicon-tunnel-boundary-20261008/` contains a raw-IP capture from
Mac utun8 and simultaneous Asterisk Ethernet capture for another failed
PCMU V.90 call. All 2071 upstream packets appear on utun8 with no gaps;
maximum send interval is 25.837 ms. Only 2063 arrive at Asterisk. Missing
sequences are 11609–11612 and 12172–12175: two 80 ms holes. Asterisk ingress
also stalls for 623.867 ms and receives reordered bursts. Relative matched
packet delay varies by 544.579 ms (this subtracts clock-offset differences;
absolute one-way delays are not asserted). Both captures report zero kernel
drops. Thus these packets were lost after entering the tunnel, rather than
merely recorded before an unsuccessful UDP send. The precise dropping
component within the tunnel/network path remains unmeasured.

The PCMA LAN follow-up `artifacts/eicon-lan-alaw-20261008/` completes 30 exact
201-byte echoes (6030 payload bytes), at 56000 downstream / 31200 upstream.
Final RTP statistics report zero packet loss both ways, and no retrain is
logged. Together, the two LAN calls carry 8040 checked payload bytes in both
G.711 laws. Plain V.34 is being tested separately.

## Native V.34: a distinct implementation fault, now hardware-confirmed

LAN native V.34 initially still fails Phase 4 despite exact delivery of
six seconds of MP audio to the card. Its J(16) request was systematically
classified as J(4). Removing that override alone still fails because the
16-point transmitter lacks its 10 dB energy compensation. With both fixes,
native PCMA and PCMU each pass ten exact echoes at 31200 upstream / 21600
downstream, and the Eicon reports aborted/CRC 0/0 on both calls. See
`v34_tx_eicon_audit.md` for the ITU clauses, staged trials and regressions.

The successful LAN comparisons establish a usable testing path. No change
has repaired the Mac-to-LAN WireGuard loss itself, and no running modem,
VPN, gateway or Eicon persistent configuration was changed.
