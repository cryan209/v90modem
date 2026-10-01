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

## 2026-10-01: stale agreements and unsolicited final acknowledgements

The opaque captures repeat optional-functions `8e 89 00`, whereas readable
captures finish with `8a 89 00`. The former advertises single-frame SREJ,
which our offer does not request and our implementation does not support.
Previously the receiver checked the field length but ignored its values.
V.42 clause 10 and Table 11a Note 1 require initiator request and responder
agreement for an optional procedure. Responses agreeing to unrequested
s-SREJ, TEST, extended FCS or m-SREJ now leave T401 running rather than
publishing parameters or sending SABME (8.10.2–8.10.3). A delayed reply to the
previous compatibility probe is a plausible explanation for the repeated
response, not a proven statement about the firmware.

Separately, `tx_information_rr_rnr_response()` always sent F=1, including
when answering an I frame with P=0 and no pending outgoing data. This
violated 8.4.2.2; 8.4.2.1 requires F=1 only for P=1. It now echoes P as F.
Public-API HDLC dialogue tests observe ordinary and polled I-frame
acknowledgements and their N(R), as well as stale XID retries and bounded
failure for each unsupported optional procedure. Both link and data-stack
regressions pass on macOS and Tower Linux.

The verified corrected live call is
`xid-validation/pf-fixed/rf-pf-live-20261001/12000`. Its independent RX decoder
finds **19 CRC-valid frames, zero bad FCS and zero malformed frames**. The
peer sends `8e` twice, then `8a`, then UA and a readable MULTITECH banner.
Thus a real repeated non-agreement no longer commits the link prematurely.
This single call does not establish a reliability percentage or prove that
all opaque streams have the same cause. The acknowledgement fix has offline
wire validation; this capture does not contain our transmitted HDLC bits.

Opt-in `DS_TX_FRAME_DUMP` records DTE input and the payload handed to LAPM;
it contains plaintext, so use only test credentials. In this call the two
credential writes are exactly `6262730d` (`bbs` plus CR) at both boundaries.
The peer independently acknowledges N(R)=1 and N(R)=2, then N(R)=3 for the
extra CR sent eight seconds later. It responds `You are logged off` and
never reaches ENiGMA. The prior verified XID-only call similarly acknowledged
both writes but returned `Invalid Password!!!`. This narrows local byte
corruption without proving the far-end application's password handling.
RasFinder-to-BBS authentication is still unresolved.

Transfer verification matters: an earlier elevated archive operation read
old workspace sources, so its purported fixed live test did not run these
changes and must not count as validation. Subsequent archives were created
inside the normal workspace context, transferred separately, and all three
source SHA-256 hashes matched on Tower before compiling and running tests.

### Both-direction wire capture and line termination

`DS_TX_BIT_DUMP` now provides opt-in ASCII transmit bits alongside the
existing RX tap, allowing independent HDLC/FCS verification beyond the
DTE/compressor boundary. Like the payload diagnostic, it includes test
credentials and is disabled unless a path is provided.

The paced CRLF call under `xid-validation/paced` contains 21 valid frames
in each direction, with zero bad FCS and zero malformed frames. Outgoing
I-frame payloads spell `bbs\r\n` twice, with consecutive sequence numbers.
The peer acknowledges them and sends `Invalid Password!!!`. Outgoing
ordinary I-frame acknowledgements have F=0; the acknowledgement to the
peer's P=1 poll has F=1. This verifies the corrected P/F behavior on an
actual call, beyond the offline regression.

A separate LF-only username was echoed without advancing the prompt.
Completing it with CR produced `Invalid User Name!!!`; LF is therefore not
an interchangeable delimiter in this terminal session. CRLF introduces
an additional input byte, so its password rejection is not a clean
CR-only authentication result. The next control is paced CR only.

A fresh independent SmartLink-to-RasFinder call with a 12000 bit/s V.34
cap, DM_TX_GAIN=4 and the previously documented resampler/headroom passed
initial negotiation but retrained and returned NO CARRIER before data.
It provides no independent authentication outcome.

The paced CR-only capture is `xid-validation/paced-cr`. Its transmitted
wire has 30 CRC-valid frames and no bad FCS or malformed frames; independent
extraction spells `bbs\r`, `bbs\r`, then an extra `\r` after 30 seconds.
The peer acknowledges all nine per-byte I frames and returns a CRC-valid
`Invalid Password!!!` after the extra CR. It still does not respond
immediately to the first password CR. RX has 28 valid frames, one bad-FCS
candidate and 43 malformed candidates over the full capture, including its
later disconnect; do not describe this entire receive stream as error free.
No LF was present in this test, so the CRLF result is not the sole reason
for the rejection. The user confirms the RasFinder account also uses bbs/bbs.

An attempted read-only opening of Remote User Database was rejected before
execution by automatic approval review because administrative access had not
been authorized. No management authentication or settings changes occurred.
Further account/terminal-server inspection requires explicit administrative
authorization and any configured management credentials.

### Authorized read-only Telnet management inspection

The user subsequently authorized Telnet inspection at 10.69.70.32. The
management interface required no password to read these menus. The bbs
record is entry 1, with a masked three-character password; its actual bytes
cannot be established from this display. Auto Protocol is Telnet and Host
IP is 192.168.88.56. Inbound and Telnet permissions are enabled; callback,
callback security, outbound, framed protocol and Rlogin permissions are
disabled. All 24 hours of all seven days are marked allowed. Daily and
monthly limits display 24:00 and 744 hours. Connection limit displays 00:00;
its semantics have not been verified. Concurrent logins displays 36826,
which is unusual but not proof of corruption or a cause of rejection.

RADIUS and accounting are disabled, with no configured server address.
All three WAN ports are enabled, Async, Modem Connect, Answering and
Terminal Server enabled, with scripts disabled. Each WAN's global Telnet
address and the terminal-server global address are 0.0.0.0; the user-specific
Telnet address is configured separately as above. No port-specific service
configuration difference was observed in these screens. No password, field
value or configuration was changed or saved. Raw menu captures are under
`xid-validation/management`.

The visible destination and permissions match the intended setup. A masked
password cannot prove that the saved credential matches the supplied bbs;
re-saving that one value is a separate configuration action, not part of
this read-only inspection.

### Authorized password re-save attempt

The user authorized re-saving only the bbs password as bbs and retesting.
Selecting the existing account's password field and entering bbs returned
`User name can't be modified`, followed by `ESC to PREV menu`. The next
record-exit save prompt was confirmed with y; that session displayed BBS,
but a fresh session again displayed bbs. Returning through the management
menus did not offer another save confirmation. Do not claim the password
update succeeded: the explicit error and masked value prevent verification.

A read-only comparison shows DOWNLOAD's concurrent-login limit is 5,
whereas bbs displays 36826. Thus the unusual value is specific to that
record, not universally displayed by the menu. This and the rejected edit
suggest an account/editor problem, without proving its cause or whether
that value is related to password rejection. Other accounts were not edited
or used for authentication.

The retest reached CONNECT 7200 (12000 cap), readable banner, username echo
and password prompt. The same paced bbs/CR password had no immediate
response, and an extra CR after 30 seconds elicited Invalid Password.
No successful password reset or RasFinder-to-BBS login is established.
The captured edit and modem session are in `xid-validation/password-save`.

### Encoder mode and delivery control

After the user confirmed explicitly changing the RasFinder password to
bbsbbs, `DS_V42BIS_FORCE_COMPRESSED` was added as an opt-in diagnostic.
It selects the existing codec's ALWAYS policy; V.42bis 7.8.1's ECM and
7.9's C-FLUSH behavior remain codec-generated. Default policy is unchanged.
Data-stack regressions pass in default and forced-compressed modes locally,
and the default suite passes on the verified Tower build. Source hashes
were matched before its build.

The live compressed control reached 12000 bit/s, a readable banner and the
password prompt. TX contains 39 CRC-valid HDLC frames with no bad FCS or
malformed frames. An independent fresh Python V.42bis decoder reads the
I-frame sequence as `bbs\r`, `bbsbbs\r`, and the extra CR after 30 seconds.
The password CR is an actual compressed literal followed by FLUSH, rather
than an assumed transparent-mode completion. The call subsequently returned
NO CARRIER without a password rejection or BBS banner. This does not prove
that the RasFinder decoder consumed the stream; it verifies the encoder's
actual output and fails to produce a successful delivery/authentication
control. Artifacts: `xid-validation/compressed`.

A plain-LAPM control explicitly disabled compression and selected three XID
option octets. It trained V.34 at 12000 and passed V.42 detection, but failed
LAPM establishment and tore down before application input. It is not an
uncompressed password test. Artifacts: `xid-validation/plain`.

The distinction remains important: the credential payload and our own
independent decode are verified, but HDLC acknowledgements alone do not
verify the peer's decompressor-to-terminal or terminal-password-parser
handoff. The first-password-CR delay is unresolved, not grounds to assume
the user supplied a different password.
