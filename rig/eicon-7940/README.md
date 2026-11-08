# Eicon 7940 caller-number fix and second PPP answerer

Configured 2026-11-08. The card's anonymous digital call reached Asterisk,
but TopRouter rejected the outgoing CLEARMODE offer with SIP 488 / Q.850
cause 65, before Eicon received it. A controlled test through the same
endpoint, with CLEARMODE and telephone-event unchanged, reproduced this:

| Caller identity | Measured result |
|---|---|
| Blank | SIP 488 / cause 65; no CONNECT in 4.5 s |
| 2620 | CONNECT 64000, PAP/IPCP, 3/3 pings, 0% loss, 15.5 s |
| Blank, through production route with fallback | Same successful PPP/ping result, 15.5 s |

This establishes caller identity as the deciding difference in these tests.
The exact internal TopRouter dial-peer selection was not inspected; do not
claim a particular dial-peer/default codec as proven. Earlier attribution
to the absence of telephone-event alone was incomplete: the anonymous call
also failed **with** telephone-event enabled.

Asterisk `from-isdn2811` now uses this in its digital branch for 7900–7950:

```
same => n(digital),ExecIf($["${CALLERID(num)}" = ""]?Set(CALLERID(num)=2620))
same => n,Dial(PJSIP/isdn2811-digital-out/sip:${EXTEN}@192.168.88.62:5060,60)
```

2620 is the internal identity of the card on the new Cisco BRI0/2/0.
Existing nonempty caller numbers are preserved. The prior dialplan is
`/etc/asterisk/extensions.conf.before-isdn-callerid-20261108`.
This is limited to the new gateway's digital access-number route; the
working 2690 calls and other source endpoints were not changed.

7940 previously had only ttyds7 / `eicon-7940-7.service`. Added the idle
`/dev/ttyds16` with `eicon-7940-16.service`, using the same existing chat
script `/etc/chatscripts/eicon-7940-answer` (`AT&F14`, controller 3,
MSN 7940, auto-answer). The included peer file uses the same server name,
IP pair (10.254.118.1 / .2), ifname e79p8, and RFC 1990 bundle identity
as ttyds7, but a distinct linkname and report file. Both units are enabled.
No PAP secrets or shared PPP policies were changed.

## Validation

Two anonymous synthetic callers from wired CT117 traversed the **actual
production fallback route**, Asterisk, TopRouter and Eicon. Both connected
at **64000 bit/s**, authenticated with PAP, and joined one bundle (first
created at **7.2 s**, second attached at **13.9 s**). Download and upload each
transferred **65536 bytes with matching SHA-256**, at **104.5 kbit/s down**
and **106.6 kbit/s up**. Both members carried data in each direction and
reported zero RTP loss. The harness reported one bad HDLC frame on each
member; the verified payloads were exact, and no long-soak claim is made.
Provenance: CT117 `/var/tmp/7940-anonymous-multilink-20261108/summary.json`.
Unlike the same-host 2690 test, this crosses hosts and actual B channels.

Temporary exact test extension 17941 and temporary HTTP fixture service
were removed/stopped afterward. Both 7940 TTYs reported DCD OFF and the
answerer units active. The user's two live 2690 calls remained Up throughout
(final check: 1716 seconds). The actual PCMCIA retry to 7940 remains the
final hardware confirmation, separate from these controlled tests.
