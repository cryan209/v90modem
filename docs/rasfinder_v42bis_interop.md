# RasFinder V.42bis login data and XID compatibility

## What the bytes are

The Tower LAN call `rf-xid-three-v42bis-r2` on 1 October 2026 reaches
V.34/LAPM at 19200 bit/s and receives this terminal login banner:

```text
MULTITECH SOFTWARE SYSTEMS
USA
login:
```

A blank response subsequently produces `Invalid User Name!!!` and another
`login:` prompt. The initial banner starts in V.42bis transparent mode. An
independent HDLC flag/destuff/FCS reader verifies its information frames, and
the independent Python V.42bis decoder produces exactly the live C codec's
PTY bytes, including the line endings and NULs. This application exchange is
a terminal login, not a PPP negotiation. No authenticated login was attempted.

The actual banner also explains the earlier nine-bit compressed stream:
seeding the independent dictionary with the newly observed transparent banner
maps its initial codewords to `MULTITECH SOFTWARE SYSTEMS`, followed by `USA`.
Reconstruction stops at code 427, which needs additional history not present
in that seed. The old dictionary is not fully reconstructed, and this result
must not be presented as a complete decode of every old capture.

## Wire difference and compatibility setting

The default XID uses a four-octet HDLC optional-functions field and a general
parameter group length of 20, as required by V.42 (03/2002), Table 11a Note 1.
RasFinder version 4.12 emits three octets and group length 19. With our
four-octet request, repeated calls returned P0=3 even when our actual XID
requested P0=0, and their first I frames referenced an already populated
V.42bis dictionary without the captured ECM/STEPUP/bootstrap sequence.

A diagnostic caller using the peer's three-octet format changed the result:

- P0=0: peer returned a general-only XID (compression defaults off), then
  acknowledged three carriage returns but supplied no information frames.
- P0=3: peer returned matching P0=3/P1=1024/P2=32 and the readable initial
  transparent banner above, followed by a readable invalid-user response.
- One intervening P0=3 call failed V.34 training before XID; it is not a
  compression outcome.

A second successful call, `rf-compat-regular`, uses the regular server in an
isolated Tower build. It reproduces the initial banner and login prompt; an
independent HDLC extraction and fresh Python decoder again match PTY exactly.

The regular engine exposes `ME_LAPM_XID_OPTION_OCTETS=3` for this compatibility
encoding; the default remains four. The public library setter accepts only
three or four and should be used before `v42_restart()`. The engine also
restarts the freshly created link when detection is disabled so that an XID
queued during initialization uses the requested encoding. No DSP or G.711
processing changes are involved. `tools/soak/rasfinder_call.sh` defaults to
this compatibility setting and hunt group 3999; its environment overrides
permit the four-octet comparison.

V.42bis (01/1990), 5.2, 5.6 and 6.2 require C-INIT and the initial root-only
dictionary at establishment; 7.2 requires initial transparent mode. A new
connection does not download a dictionary from an ISP. We have demonstrated
a working negotiation encoding, but have not proved the RasFinder firmware's
internal reason for the old missing-context streams or performed a randomized
per-port comparison. No RasFinder reset was needed for the readable capture.

## Validation and artifacts

The default and three-octet outgoing XID regression cases independently
inspect the transmitted HDLC frames and verify the TLV boundaries, optional
functions and explicit P0=0. `v42_link_test` and `data_stack_test` pass on macOS
and in the isolated Linux build in Tower's `v90modem-sip` container.

Ignored artifacts are under `artifacts/tower-interop-20261001/`:
`rf-readable-analysis.json`, `rf-readable-decoded.bin`,
`actual-banner-reconstruction.json`, `rf-xid-three-analysis.json`, and the
corresponding raw bit streams, G.711 taps, callback schedules and PTY captures.
