# Eicon BRI live tests, 8 October 2026

Targets: 7900, PCMA, Diva controller 2; 7910, PCMU, controller 1.
Modem role: V.90 analogue caller, SIP accounts 2905/2900 respectively.
Gateway: Cisco 192.168.88.62, BRI voice ports 0/0/0 and 0/0/1.
Eicon diagnostics: eicon420.net.cryan.nz, `bri-test-answerer` journal,
`mantool`, and `mlog`.

## Established outcomes

Both laws reached V.90 data mode, 56000 downstream / 31200 upstream,
V.42 LAPM and V.42bis, and received the BRI test menu. These are actual
hardware connections, but neither establishes a stable data connection.
Checked 201-byte echo lines stall, and the Eicon subsequently retrains.

Reducing the upstream cap to 28800, 21600 or 12000 does not fix it.
At 12000, one call carried three complete checked lines; a later forced
V.90 call carried one. Disabling the nonlinear x warp made LAPM
establishment worse and was not retained. A V.32bis fallback also stalled
and the Eicon reported receive CRC errors, so the failure is not exclusive
to V.90 PCM mapping.

The first failed PCMU Phase 4 call transmitted a independently decoded,
CRC-valid CP-prime on its local TX tap. The peer continued MP-prime and
never returned E. This does not establish that the peer received that
frame intact. No protocol change is justified by this capture alone.

## Native receive evidence

`artifacts/eicon-native-v90-7910.txt`, paired with
`artifacts/eicon-fix-7910-native-v90-r2/`, records a forced V.90 connection
at 56000 TX / 12000 RX on the Eicon. Its first bad HDLC CRC indication is
12:1318:359 in the card trace; errors continue until its supervisor starts
a retrain at 12:1326:331. Thus upstream data corruption is observed by the
foreign receiver, rather than inferred from our own receiver state.

`artifacts/eicon-native-audio-7910.txt`, paired with
`artifacts/eicon-fix-7910-native-audio-r3/`, contains 657 Audio1 records,
each 520 bytes: 260 paired receive/transmit G.711 samples. The first byte
of each printed SAMPLE word matches the local TX tap byte-exact over long
sections. Unique 32-byte matches show card-minus-local sample offsets:

| Card stream seconds | Local TX seconds | Offset in samples |
|---:|---:|---:|
| 3.455 | 3.32225 | 1062 |
| 5.395 | 5.25225 | 1142 |
| 8.180 | 8.04725 | 1062 |
| 14.560 | 14.43725 | 982 |
| 16.200 | 16.04725 | 1222 |

These are independent stream origins, not common wall-clock timestamps.
Changes by 80 samples suggest playout insertion/deletion. Trace accounting
and the location of those changes still need confirmation before assigning
the fault to the Cisco or Asterisk. The card reports zero DSP sample
overruns and receive overflow in this capture.

## Fixed playout experiment and remaining isolation

The BRI ports originally had default playout settings. On 7910 only,
`playout-delay nominal 200` and `playout-delay mode fixed` were tested.
`artifacts/eicon-fix-7910-fixed200-r1/` carried four checked echo lines,
then stalled on line 4. Native trace
`artifacts/eicon-native-fixed200-7910.txt` records RX aborted/CRC 22/0
at teardown, with no supervisor retrain observed before the harness stopped
the call. The change is therefore not a validated fix and was restored to
defaults. No startup configuration was saved.

A direct SIP comparison requires allowing 10.69.220.8 in the Cisco SIP
trusted list. The user approved that temporary entry, but the HTTP CLI
cannot enter its nested submode; the attempted `ipv4` command was rejected
by IOS and no entry was added. VTY `transport input none` disables SSH.
The subsequent direct call received no SIP answer. A console-applied entry
is needed before that comparison can establish anything about the bearer.
Audio tracing was verified disabled after the capture.

No source or protocol fix has been established in this session.

## Direct-path retry after console access change

The subsequent user-requested retry was accepted directly by the Cisco.
`artifacts/eicon-direct-retry-20261008-r1/` reached V.90 56000/12000,
LAPM and the menu, but its first checked echo line failed. The paired
`artifacts/eicon-direct-retry-native-7910.txt` records RX aborted/CRC
104/31 for that call. Therefore Asterisk is not necessary for this fault.

With fixed 200 ms Cisco playout, direct call
`artifacts/eicon-direct-fixed-retry-20261008-r2/` passed three checked
lines, then failed. Cisco's active-call report showed g711ulaw,
Transcoded: No, lost/early/late 0/1/0, and playout delay 205 ms.

The repeat `artifacts/eicon-direct-fixed-audio-20261008-r3/` failed its
first echo line. Paired card audio is
`artifacts/eicon-direct-fixed-audio-20261008.txt`. Decoded receive samples
match local transmit windows at local TX seconds 18, 24, 27 and 30 with a
constant card-minus-TX offset of 2343 samples. Equality is 99.9–99.95%;
decoded linear waveform correlation rounds to 1.0000. Thus the earlier
adaptive-playout sample discontinuities are not a sufficient explanation
for the remaining fixed-buffer data fault. Upstream modulation/encoding
interoperability needs investigation as well. The repeat Eicon teardown
reports RX aborted/CRC 14/3 and zero DSP sample overruns.

The playout experiment was again restored to defaults. The temporary SIP
trusted-list entry was applied externally through the console, and must be
removed there when direct testing ends because the HTTP interface cannot
enter that submode. No running configuration was saved by these tests.


## V.90 retry after enabling Asterisk CLEARMODE

Calls remain G.711: live Asterisk channel inspection confirms PCMU on both
legs for 7910, PCMA on both legs for 7900, and no translation in either
direction. Enabling the separate CLEARMODE format does not itself move
V.90 onto an unrestricted digital bearer.

`artifacts/eicon-v90-asterisk-after-clearmode-7910-r1/` completes V.90 at
56000 downstream / 31200 upstream, establishes V.42 LAPM with V.42bis,
and returns the menu and echo-mode prompt. The first 201-byte test line
is only partly echoed, then the Eicon sends Tone B and our engine enters
9.5.2 retrain at +27431 ms. No exact echo line passes. Final RTP stats:
zero received packets lost, four transmit packets reported lost; these
aggregate statistics do not establish when or where the loss occurred.

`artifacts/eicon-v90-asterisk-after-clearmode-7900-r1/` trains at 54666
downstream / 31200 upstream, but V.42 detection fails, falls back to V.14,
and the peer sends Tone B, causing 9.5.2 retrain at +24837 ms. A second
training completes at 56000/31200 (+37896 ms), followed by another retrain
at +44596 ms. The next training completes at 56000 downstream / 19200
upstream (+56936 ms); the menu and echo-mode prompt then arrive. Three
201-byte lines echo exactly (603 payload bytes), but line 3 fails and the
peer causes a third retrain at +88996 ms. The recovered call therefore
carries real data briefly but does not sustain it. These trials provide
no controlled evidence of improved V.90 reliability after the Asterisk
CLEARMODE installation.
No source, Cisco, or Eicon configuration was changed for the V.90 retry.
