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
| 2630–2699 | 0/2/1 |

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
