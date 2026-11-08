# Digital ISDN PPP directly into v90modem

Deployed 2026-11-08 on wired CT117 (192.168.88.230), alongside the existing
analogue `v90ras` server. The PCMCIA card calls through the Cisco 2811 and
Asterisk, bypassing TopRouter and Eicon. This isolates the X-CCD input path.

| Number | Target |
|---|---|
| 2690 | Pilot; line 0, then line 1 if busy/unavailable |
| 2691 | Direct line 0, SIP port 5400 / RTP 33000 |
| 2692 | Direct line 1, SIP port 5401 / RTP 33100 |

The three exact Asterisk routes override the generic `_26XX` BRI aliases.
Endpoint names are `isdn2691` and `isdn2692`, static contacts on CT117,
CLEARMODE only. Asterisk `codec_xccd.so` bridges the 2811's legacy X-CCD
format to CLEARMODE byte-exactly (RFC 4040 section 3). Existing 2950–2960
analogue service is unchanged.

The binary is the existing `/opt/v90ras/sip_v90_modem`; systemd units are
`v90isdn.service` and `v90isdn-ppp@0` / `@1`. Profiles are installed as
`/etc/v90isdn/profile.0` and `.1`. No SIP registration is needed; incoming
calls use static contacts and `--no-register`. `AT+MS=CLEAR,0` bypasses modem
training; `ME_DATA_FRAMING=ppp` uses synchronous HDLC on the DS0 and async
HDLC on the PTY (RFC 1662). Two pppd instances share RFC 1990 multilink
identity and can join one bundle.

PPP is `noauth`, matching the internal test rig's v90ras policy. Server IP
10.254.201.1, client 10.254.201.2, interface `v90isdn0`, DNS 192.168.88.1.
IP forwarding is already enabled on CT117. `/etc/nftables.d/v90isdn.nft`
adds a separate masquerade table and is included by `/etc/nftables.conf`;
the prior file is `nftables.conf.before-v90isdn-20261108`. The live rule was
added without flushing existing tables.

Asterisk backups: `/etc/asterisk/{pjsip,extensions}.conf.before-v90isdn-20261108`.

First controlled wired test: **64000 bit/s**, LCP/IPCP, client 10.254.201.2,
**3/3 pings, 0% loss**, 11.8 seconds total. No PAP was requested (noauth).
Provenance: CT117 `/var/tmp/2690-digital-probe-20261108/result.json`. This
confirms PPP negotiation on the new server. The synthetic client and server
ran on the same CT, so those pings can be locally routed; they do not prove
IP payload transfer through the DS0. The card/2811 X-CCD path still needs
its own validation.

Operate without restarting the analogue server:

```
systemctl status v90isdn v90isdn-ppp@0 v90isdn-ppp@1
journalctl -u v90isdn -u v90isdn-ppp@0 -u v90isdn-ppp@1 -f
```

Two-link negotiation probe: both calls CONNECT **64000**, first bundle
created after **6.6 s**, second link attached after **10.3 s**. Client used
`endpoint local:26:90:00:01` so its identity differed from the same-host
server's MAC discriminator. No download/upload tests were requested, so
the harness's overall `ok` is false by design (`bool(tests)`); this result
is a bundle-negotiation check, not a payload/throughput pass. Both calls
ended and the lines were verified idle before handoff. Provenance: CT117
`/var/tmp/2690-multilink-probe-20261108-r2/summary.json`.
