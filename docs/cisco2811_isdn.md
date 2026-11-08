# Cisco 2811 PCMCIA ISDN gateway

Configured 2026-11-08 over `/dev/cu.usbserial-FT4TQOFT` at 9600 8N1.
This is a separate device from TopRouter (192.168.88.62).

Hardware: Cisco 2811, IOS 12.4(22)T IPVOICEK9, two VIC2-2BRI-NT/TE
cards (four S/T BRI ports), VWIC-1MFT-E1 and PVDM2-64.

FastEthernet0/0 uses DHCP: observed 10.69.70.182/24, gateway 10.69.70.1,
MAC 001e.7a78.7868. Reserve this lease on the DHCP server: Asterisk's
IP identify and static contact depend on this address. Both directions of
ICMP to Asterisk (192.168.88.122) passed, two packets each.

BRI ports emulate the Euro-ISDN (basic-net3 / DSS1) network, layers 1–3,
with line power enabled. Connect the PCMCIA card's S/T interface using a
BRI crossover cable, not an Ethernet crossover cable. See Cisco document
15415, "Configuring Network Side ISDN BRI Voice Interface Cards":
https://www.cisco.com/c/en/us/support/docs/voice/digital-ccs/15415-bri-voice-port-cfg.html

| Asterisk number | BRI port |
|---|---|
| 2600–2609 | 0/1/0 |
| 2610–2619 | 0/1/1 |
| 2620–2629 | 0/2/0 |
| 2630–2689, 2693–2699 | 0/2/1 |
| 2690 | Dedicated digital v90modem PPP pilot |
| 2691, 2692 | Dedicated digital v90modem PPP lines |

Numbers are MSNs/aliases on the listed bus, not extra B channels. Each
BRI has two B channels. Set the card to Euro-ISDN/DSS1 and an MSN from
its port's range. The card can dial four-digit Asterisk numbers directly.
2600–2699 was unused; 2700–2800 overlaps the existing `_2[7-9]XX`
Cisco UCM route and was deliberately left alone.

Asterisk endpoint `isdn2811` identifies 10.69.70.182/32, uses
`from-cisco`, and permits CLEARMODE and A-law. `_26XX` in `default`
routes to this endpoint. Backups before changes:
`/etc/asterisk/{pjsip,extensions}.conf.before-isdn2811-20261108`.

Digital data is intended to use RFC 4040 CLEARMODE (opaque DS0), with
Q.931 UDI bearer mapping. However IOS 12.4(22)T rejects both `encap
clear-channel` and `encap clear-channel standard` under `voice service
voip` / `sip`. The clear-channel codec preference and outgoing UDI mapping
were accepted, but RFC 4040 interoperability is **not established**. Cisco
documents legacy X-CCD/8000 encapsulation without the standard command:
https://www.cisco.com/c/en/us/td/docs/ios/voice/command/reference/vr_book/vr_v1.html
An IOS upgrade supporting standard encapsulation may be necessary; do not
claim a digital data pass on this configuration. G.711 A-law is offered for speech; neither offering nor
SIP reachability proves successful ISDN data transfer. Fixed 40 ms
playout, echo cancellation disabled and VAD disabled avoid audio processing
on modem/data bearers. No DSP source or constants changed.

Verification: Asterisk SIP OPTIONS returned Available (9.568 ms); the
2600 route resolves to isdn2811 and 2700 still resolves to Cisco UCM. All
four BRI ports report network side / basic-net3, but Layer 1 DEACTIVATED
and Layer 2 NOT Activated. No end-to-end call or data transfer was tested.
Port line power reported ON. Nominal fixed playout is 40 ms; IOS requires
a maximum greater than nominal, so maximum is 60 ms. The unsupported
voice-port `station-id number` setting was not installed; caller identity
comes from the terminal's Q.931 setup.

## Legacy X-CCD compatibility, 2026-11-08

The PCMCIA card brought BRI0/2/0 Layer 1 ACTIVE and Layer 2 to TEI 64,
SAPI 0, MULTIPLE_FRAME_ESTABLISHED. Initial 7920 attempts failed with Q.850
cause 63. A subsequent 7940 capture showed the 2811 offering X-CCD/8000
(PT125) and PCMA/8000 (PT8), and Asterisk answering **PCMA**. Eicon
controller 3 logged the resulting incoming calls as **CIP 1 (speech)** in
`diva-sip-gateway`, while 7940's PPP process never received CONNECT. This
was a speech fallback, not a successful digital PPP connection.

Added `tools/asterisk/codec_xccd.c`: registers X-CCD and maps to/from
CLEARMODE by copying opaque DS0 bytes unchanged (RFC 4040 section 3).
The target PBX builds it in `/opt/asterisk/codecs` and loads the installed
`/usr/lib/asterisk/modules/codec_xccd.so`; no live PBX restart was required.
`xccd test` passed **512 frames / 206848 bytes**, every codeword and unchanged
sample counts, in both an isolated instance and live Asterisk 22.5.1.

`isdn2811` now permits x-ccd/clearmode/alaw and uses context
`from-isdn2811`. Its `_79[0-5]0` route checks `CHANNEL(audionativeformat)`: X-CCD or
CLEARMODE calls use endpoint `isdn2811-digital-out`, allowing **CLEARMODE
only**, to TopRouter 192.168.88.62. Speech calls and other numbers include
the existing `from-cisco` context.
Original shared 7940 routing for other callers is unchanged. Backups:
`/etc/asterisk/{pjsip,extensions}.conf.before-xccd-7940-20261108`.

The PPP answerer `eicon-7940-7` uses ttyds7, binds Diva controller 3
(`AT+iQ=a3`), accepts MSN 7940 (`AT+iA7940`), and uses `AT&F14` protocol
autodetection. Its expected interface is **e79p8**, local 10.254.118.1,
peer 10.254.118.2. The peer-file comment claiming controller 7 / 7930 is
stale; the active chat script specifies controller 3 / 7940.

A real digital probe from wired CT117 to 17940 passed at **64000 bit/s**,
LCP/PAP/IPCP and **3/3 pings (0% loss)** in 15.5 seconds. Eicon assigned
10.254.118.2 (local server 10.254.118.1); `eicon-7940-7` recorded the
negotiation. This supersedes the old 6 November destination timeout for
that digital path. It does **not** yet qualify the PCMCIA/X-CCD leg.
Provenance: CT117 `/var/tmp/7940-digital-probe-20261108/result.json`.
The harness summary labels mode `v34` from its default argument, but the
explicit override was `AT+MS=CLEAR,0`, with `ME_CLEARMODE=1` and
`ME_DATA_FRAMING=ppp`; the trace explicitly negotiated CLEARMODE.

### Final route and voice sharing checks

A PCMCIA retry at 19:00:37 reached Asterisk on 7940 with X-CCD/PCMA
offered. The digital-only outgoing offer (CLEARMODE with no telephone-event)
was rejected by TopRouter with **SIP 488 / Q.850 cause 65**, producing the
Windows 770 failure before Eicon PPP received a call. The final dedicated
endpoint still allows **CLEARMODE only** but uses `dtmf_mode=rfc4733` (a
separate telephone-event SDP entry). A real wired probe through that exact
endpoint passed **64000 bit/s, LCP, PAP, IPCP and 3/3 pings, 0% loss**, in
15.5 seconds. The earlier first pass used shared endpoint 6501.
Provenance: CT117 `/var/tmp/7940-digital-probe-20261108-r5/result.json`.
The temporary validation extension 17941 was removed afterward.

A CAPI listener on Eicon controller 3 also claimed speech calls to MSN
7940 and forwarded them to SIP. Added repeatable `--ignore-isdn-number`
to `/usr/local/sbin/diva_sip_gateway.py` and configured only 7940 in
`/etc/systemd/system/diva-sip-gateway.service.d/local-answerer.conf`. CAPI
2.0 Part I section 5.6 specifies CONNECT_RESP Reject=1 as **ignore**, leaving
the call available to the tty answerer, rather than rejecting it on the
network. The patch is `tools/eicon/diva_sip_gateway-local-answerer.patch`.
The service had no active calls when restarted. Backup:
`/usr/local/sbin/diva_sip_gateway.py.before-ignore7940-20261108`.
Spec: https://capi.org/wp-content/uploads/capi20-5-I.pdf

A voice test reached the local modem, connected at **31200 bit/s**, and
received LCP. It then retrained/stalled, never authenticated, and ended
after 50.1 seconds. Thus speech routing and modem answer are established;
a successful voice PPP session is **not** claimed.
Provenance: CT117 `/var/tmp/7940-voice-probe-20261108/result.json`.

The PCMCIA/X-CCD path still requires its own successful PPP verification;
controlled wired CLEARMODE tests alone do not establish it.

## Direct v90modem digital PPP comparison

Added the separate [2690 digital server](../rig/v90ras/isdn/README.md) on
wired CT117 to bypass TopRouter/Eicon while testing the PCMCIA input.
Two CLEAR/HDLC workers and noauth multilink PPP answer on 2691/2692;
2690 hunts both. The first server-side test passed at **64000 bit/s**,
LCP/IPCP and **3/3 pings (0% loss)**. The client/server shared a host,
so the ping result does not establish payload transfer over the DS0.
Two test calls also joined one multilink bundle in **10.3 s**.
No actual PCMCIA pass is inferred
from this controlled wired test. See the linked deployment note.

## Rejection isolated to missing caller number

A controlled repeat reproduced the 7940 rejection with a blank caller
number **even with telephone-event present**. Changing only caller identity
to **2620** restored CONNECT 64000, PAP/IPCP and **3/3 pings, 0% loss**.
Thus the earlier telephone-event explanation did not isolate the root
cause. Added a fallback caller number 2620 only for empty calling numbers
on the new gateway's digital 7900–7950 route. The exact TopRouter internal
dial-peer selection remains uninspected.

Also added a second 7940 PPP answerer (idle ttyds16 alongside ttyds7).
Two anonymous synthetic callers through the repaired production route
formed one bundle and transferred **65536 bytes each way, SHA-256 exact**,
at **104.5 kbit/s download / 106.6 kbit/s upload**. The existing two-channel
2690 card session stayed Up. Details and reproducible configuration:
[7940 repair](../rig/eicon-7940/README.md). The card's own 7940 retry is
still required for final hardware confirmation.

## Cisco per-port caller identity defaults

Updated 2026-11-08: incoming voice-port translation profiles now fill only
an empty caller number on the 2811, at the source:

| BRI port | Translation profile | Default caller number |
|---|---|---|
| 0/1/0 | CID2600 | 2600 |
| 0/1/1 | CID2610 | 2610 |
| 0/2/0 | CID2620 | 2620 |
| 0/2/1 | CID2630 | 2630 |

Each rule is `rule 1 /^$/ /NUMBER/`. IOS `test voice translation-rule`
verified empty input maps to the four assigned numbers and supplied
**2625 does not match** any rule, so a supplied number is preserved.
The profiles are attached with `translation-profile incoming CIDNUMBER`.
Configuration fragment: [callerid-defaults.cfg](../rig/cisco2811/callerid-defaults.cfg).
Cisco reference (empty-string match and profile direction):
https://www.cisco.com/c/en/us/support/docs/voice/call-routing-dial-plans/64020-number-voice-translation-profiles.html

These changes apply to new calls. Existing two-channel calls to 7930
remained Up during the update. The Asterisk missing-number fallback is
retained as a harmless last resort; it only runs if the Cisco still sends
no caller number. A fresh call is needed to verify the caller number at
the PBX before that fallback runs. No media processing was changed.

`write memory` completed; `show startup-config` verified all four rules,
profiles, and incoming voice-port attachments persisted.
