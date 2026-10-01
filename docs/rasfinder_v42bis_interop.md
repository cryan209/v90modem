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

The engine and library now detect this compatibility case from the wire.
Automatic mode starts with Table 11a's four-octet format. If a fully parsed
initial XID reply advertises three octets, the caller sends one new XID command
using three, before SABME or information transfer. The first reply does not
publish compression parameters or initialize a dictionary. The final reply
settles those parameters, then normal establishment initializes the codec.
An answering endpoint mirrors a three-octet initial command in its response.
There is no peer-name, address or banner lookup.

V.42 8.10.1/8.10.2 define the negotiation/indication exchange and permit another
XID command before SABME. The shortened encoding remains a compatibility
exception to Table 11a, not a normative V.42 requirement. T401/N400 bound the
new exchange under 8.10.3; exhaustion reports link error and cannot proceed
with unconfirmed parameters, consistent with Annex III.3. Adaptation is
limited to initial negotiation; it does not reset a live data dictionary.
Each new link starts with four again.

`ME_LAPM_XID_OPTION_OCTETS=auto` is the default, including in the RasFinder
harness. Values 3 and 4 remain explicit diagnostic overrides. The public
setter accepts 0 (auto), 3 or 4 before `v42_restart()`. Selected encoding is
available in the negotiated parameters and engine log. The harness dials
hunt group 3999. No DSP or G.711 processing changes are involved.

Undefined dictionary entries or reserved compression commands already take
the data stack's compression error path. Arbitrary binary bytes in a valid
transparent stream cannot be identified as corrupt merely because they are
unreadable; no payload-guessing heuristic was added.

V.42bis (01/1990), 5.2, 5.6 and 6.2 require C-INIT and the initial root-only
dictionary at establishment; 7.2 requires initial transparent mode. A new
connection does not download a dictionary from an ISP. We have demonstrated
a working negotiation encoding, but have not proved the RasFinder firmware's
internal reason for the old missing-context streams or performed a randomized
per-port comparison. No RasFinder reset was needed for the readable capture.

## Validation and artifacts

Automatic dialogue tests cover legacy and modern replies, forced modes,
no intermediate parameter event, initial four-octet encoding on a new link,
malformed option lengths, bounded failure without SABME, and information
transfer after the completed exchange. The default and three-octet outgoing
XID regression cases independently
inspect the transmitted HDLC frames and verify the TLV boundaries, optional
functions and explicit P0=0. `v42_link_test` and `data_stack_test` pass on macOS
and in the isolated Linux build in Tower's `v90modem-sip` container.

The automatic hardware call `rf-auto-r2` has no compatibility override. Its
independently CRC-verified RX stream contains the initial three-octet reply,
a second reply to our compatibility exchange, UA, and the fresh login banner.
The log reports `options=3 octets`; a fresh reference decoder matches its PTY
bytes exactly. `rf-auto-analysis.json` contains the frames and comparison.
An earlier automatic call was terminated during training, before XID.

Two SmartLink hardware retests failed V.42 detection before XID; the previous
nonautomatic-server control also failed before XID, during modem training.
None grades the new XID exchange or application roundtrip against SmartLink.
Modern and legacy peer dialogue/transfer regressions pass offline, and the
previous successful SmartLink captures remain in the earlier interop report.

Ignored artifacts are under `artifacts/tower-interop-20261001/`:
`rf-readable-analysis.json`, `rf-readable-decoded.bin`,
`actual-banner-reconstruction.json`, `rf-xid-three-analysis.json`, and the
corresponding raw bit streams, G.711 taps, callback schedules and PTY captures.
