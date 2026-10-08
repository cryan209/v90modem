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
