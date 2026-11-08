# Asterisk CLEARMODE module

`codec_clearmode.c` registers RFC 4040 CLEARMODE as an opaque 8000-octet/s
format. It provides no translator. SIP endpoints at both ends must permit
`allow=clearmode`; audio-processing dialplan applications are inappropriate
for its opaque payload. Installing this module alone does not change endpoint
codec preferences or prove the Cisco/Eicon path preserves bytes.

Installed on `asterisk.net.cryan.nz`, Asterisk 22.5.1, 2026-10-08:

- Source: `/opt/asterisk/codecs/codec_clearmode.c`
- Module: `/usr/lib/asterisk/modules/codec_clearmode.so`
- Loaded live with `module load codec_clearmode.so`, no PBX restart.
- Existing `autoload=yes` and module load priority 60 register the format
  before channel drivers (priority 70) on subsequent starts.
- Verified Running status and successful temporary loopback UnicastRTP
  channel creation using `c(clearmode)`.

Build against the target PBX's matching configured source tree:

```sh
make -C /opt/asterisk/codecs ASTTOPDIR=/opt/asterisk \
  ASTCFLAGS='-I/opt/asterisk/include -D_GNU_SOURCE' \
  SOLINK='-shared' codec_clearmode.so
```

Asterisk's public codec registration pins modules until process shutdown.
Consequently removal or replacement requires a planned PBX restart; ordinary
live unload is intentionally refused. To remove, move the installed `.so`
out of the module directory (or add `noload=codec_clearmode.so`) and restart
at a suitable time. Rebuild against matching headers after Asterisk upgrades.

The `core show codecs` FORMAT column says `no cached format` because public
`ast_codec_register()` leaves its internal `format_name` label unset. The
module separately registers the actual named format in the cache and RTP
engine; the local RTP channel test checks its lookup works.

PJSIP enabled on 2026-10-08 and reloaded successfully:

- `[endpoint-modem](!)`: `allow=!all,clearmode,ulaw`
- `[endpoint-modem-alaw](!)`: `allow=!all,clearmode,alaw`
- `[6501]` endpoint: CLEARMODE first, then the existing ulaw/alaw entries.
  Dialplan `_7XXX` uses this endpoint for 7900/7910.

Verified effective `allow` lists on 2900, 2905, 6001, 6501 and 8421.
Remote configuration backup:
`/etc/asterisk/pjsip.conf.before-clearmode-20261008T115944`.
The existing active call remained present after the PJSIP reload. This check
establishes configuration activation, not end-to-end byte integrity.

# Legacy Cisco X-CCD compatibility

These modules build inside the **target Asterisk source tree**, not the
softmodem's `SRCS` or test object lists. Do not add them to the modem binary.

`codec_clearmode.c` registers RFC 4040 CLEARMODE/8000. `codec_xccd.c` adds
Cisco IOS legacy X-CCD/8000 and byte-preserving bridges to/from CLEARMODE.
The latter is needed for the Cisco 2811 IOS 12.4(22)T, which does not have
`encap clear-channel standard`. Cisco's command reference documents the
legacy X-CCD default:
https://www.cisco.com/c/en/us/td/docs/ios/voice/command/reference/vr_book/vr_v1.html

Both formats carry one opaque DS0 octet per 8 kHz sample (RFC 4040 section
3). The bridge copies bytes and relabels the format. It has **no audio codec
translation path**: no G.711 decoding/re-encoding, PLC, gain, resampling or
sample insertion. Asterisk nevertheless labels the format bridge a
"translator" in its channel diagnostics.

Install with the target PBX's configured source/build options, for example:

```
# In the configured Asterisk source tree, after copying both .c files to codecs/
make codecs
install -m 755 codecs/codec_xccd.so /usr/lib/asterisk/modules/
asterisk -rx 'module load codec_xccd.so'
asterisk -rx 'xccd test'
```

`codec_xccd` requires `codec_clearmode`. With `autoload=yes`, its module
metadata expresses this dependency. Codec registration pins these modules
until process shutdown; they cannot be unloaded/replaced live. Build and
test new versions in an isolated Asterisk process first.

`xccd test` exercises the real Asterisk translation paths in both directions:
512 frames / 206848 bytes, mixed 80/160/256/1120-byte packets, every codeword,
and exact byte and sample-count equality. It sends no network traffic.
It passed both in an isolated Asterisk 22.5.1 process and on the live PBX on
2026-11-08. This checks the module, not end-to-end ISDN or RTP delivery.

For digital calls, allow `x-ccd` on the legacy Cisco endpoint and force
`clearmode` on the destination gateway endpoint. Do not allow a digital-only
route to fall back to G.711: speech bearer selection can hand the call to
a different answerer. See `docs/cisco2811_isdn.md` for deployed routing.

For this TopRouter, the final digital-only endpoint uses `dtmf_mode=rfc4733`:
the 7940 PCMCIA call's offer without telephone-event was rejected with SIP
488 / cause 65. The final endpoint (CLEARMODE plus telephone-event) passed
a separate real digital PPP probe at 64000 bit/s and 3/3 pings with 0% loss.
The separate telephone-event payload does not convert or inspect DS0 bytes.

Subsequent controlled testing isolated **missing caller number**, rather
than telephone-event alone, as the remaining TopRouter rejection trigger:
an anonymous call with telephone-event still failed, whereas the identical
call with caller 2620 passed. The deployed digital route now supplies 2620
only when the incoming caller number is empty. See
[the measured A/B and two-channel payload pass](../../rig/eicon-7940/README.md).
