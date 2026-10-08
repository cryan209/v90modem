# Eicon digital rate-adaptation tests, 8 October 2026

Live targets: 7900 PCMA / controller 2 and 7910 PCMU / controller 1.
SIP accounts 2905 and 2900 through Asterisk. The original
`bri-test-answerer` service was stopped during testing, replaced by a
temporary `/tmp/bri_isdn_answerer.py` service, and restored afterward.
Its installed source and persistent configuration were not changed.

## Results

| Mode | 7900 A-law | 7910 µ-law |
|---|---|---|
| V.120 acknowledged, 64000 | 3/3 exact 201-byte echo lines | 3/3 exact 201-byte echo lines |
| V.110 async, 9600 | 10/10 exact 201-byte echo lines | S/X startup succeeds; received menu characters corrupted |
| CLEAR with diagnostic raw framing, 64000 | Menu received; echo selection fails amid idle octets | Menu received; echo selection fails amid idle octets |

Each successful line includes a sequence number and three repetitions of
digits, uppercase and lowercase letters, checked against the complete
`ECHO: ...` response. These are bounded hardware tests, not a throughput
or long-term stability claim.

Final captures:

- `artifacts/eicon-isdn-v120-7900-longwait-20261008-r2/`
- `artifacts/eicon-isdn-v120-7910-longwait-20261008-r6/`
- `artifacts/eicon-isdn-v110-7900-data-20261008-r2/`
- `artifacts/eicon-isdn-v110-7910-data-20261008-r2/`
- `artifacts/eicon-isdn-clear-7900-20261008-r1/`
- `artifacts/eicon-isdn-clear-7910-20261008-r1/`

## Setup and traps found

Eicon's AT.txt documents factory profiles 6 (V.120 64k), 3 (V.110 async)
and 8 (bit-transparent). The temporary answerer used the appropriate
profile, controller binding `+iQ=aN`, accepted number `+iA7900/7910`,
`+iS1`, `+iC0`, `&K0`, and `E0V1S0=1\V1#CID=14`.
V.110 additionally used `+iB5` for 9600 bps. The TTY was raw, hardware
flow control disabled, and DTR/RTS explicitly asserted.

Two setup errors must not be counted as protocol failures:

1. Incoming Q.931 BC was `80 90 a2` (speech). In the Eicon driver's
   `tty_module/atp.c:atAnswer()`, a speech call received under a digital
   profile with configured Service > 2 is switched to `ISDN_PROT_MDM_a`.
   Setting `+iS1` avoids that override. Management then confirms
   HDLC 64k / V.120, rather than Modem ASYNC / V.42.
2. AT.txt says `+iC1` switches to data mode. The actual `atp.c` parser
   implements `At->f.Data = Val ? 0 : 1`: **+iC0 enters data mode**.
   Using +iC1 established links but left the service's menu writes in
   command mode, so no data bytes were sent.

Local caller commands were `AT+MS=V120,0` with `ME_V120_ACK=1`, or
`AT+MS=V110,0,0,9600`. V.120 compression was off in the successful tests.
An independent HDLC decode of the captures verifies CRC-valid SABME,
UA, RR and I frames. The Eicon's per-character V.120 echo is slow enough
that an eight-second whole-line timeout falsely reports failure; the
successful checks allowed thirty seconds per line.

## µ-law evidence and bearer qualification

V.110 on 7910 enters DATA after S/X ON both ways and reports 3957
received frames, 125 bytes, no bad or unsupported frames. Yet the menu
starts `CSM0MODEM0TEST0LAB` instead of `BRI MODEM TEST LAB`.

The V.110 receive stream uses DS0 codewords 3F, 7F, BF, FF for its
16-kbit/s intermediate-rate packing. The A-law capture contains 2139
occurrences of 7F. The µ-law capture contains **zero** occurrences of 7F,
while the other three codewords are present. This is consistent with
7F-to-FF normalization of the two µ-law zero representations along the
voice path, which preserves audio but corrupts digital bits. The exact
device doing it was not isolated by these tests. V.120 succeeding with
acknowledgement does not prove the µ-law bearer is byte-exact.

These tests tunnel rate-adaptation protocols over G.711 on a speech B
channel; they do not establish a Q.931 unrestricted-digital/CLEARMODE
call. Cisco's voice-port bearer-cap command offers speech and 3100hz,
and the observed incoming BC remains speech. No persistent gateway
configuration was changed for these ISDN tests.

CLEAR/raw has no async start/stop framing or HDLC idle suppression, so
idle DS0 octets reach the application as data. The menu is consequently
not a valid transparent-stream checker. Its failed selection is not
evidence that the CLEAR bit pipe itself is defective; a continuous
pattern checker is needed to grade it. No CLEAR pass is claimed.

These tests do not resolve the separate V.34 upstream encoder investigation.


## Direct CLEARMODE / unrestricted digital retry

After the user enabled Cisco clear-channel codec preference and bidirectional
UDI bearer mapping, direct calls negotiate RFC 4040 CLEARMODE. The initial
SIP 500 / cause 127 disappeared after removing the temporary Eicon answerer's
`AT+iS1` speech-service filter. Eicon `mantool` and the D-channel trace
confirm BC `88 90` (unrestricted digital, 64 kbit/s), rather than speech.
The Asterisk-routed comparison still negotiated PCMU.

V.110 at 9600 was tested directly on both extensions:

- `artifacts/eicon-v110-clearmode-udi-20261008-r3/` (7910)
- `artifacts/eicon-v110-clearmode-udi-7900-20261008-r4/` (7900)

Both receive valid framing, with zero reported bad or unsupported frames,
but fail the clause 7.1 startup guard after ten seconds. Independent decode
of the tap's two significant bits per DS0 octet shows our transmitter
switching S/X ON (`000000` / `00`) while the received Eicon frames remain
S/X OFF (`111111` / `11`). Reasserting RTS/DTR after the Eicon profile reset
also fails (`eicon-v110-clearmode-rts-20261008-r7`).

A direct acknowledged V.120 trial on 7910 likewise negotiates CLEARMODE but
never produces the menu (`eicon-v120-clearmode-udi-20261008-r8`). No successful
data transfer over true UDI/CLEARMODE is claimed. The remaining fault has not
been localized to the Cisco, Eicon configuration, or our implementation.
`artifacts/eicon-clearmode-bchannel-20261008.txt` records the UDI SETUP and
Eicon's queued menu bytes; these are application-side B-channel records,
not a byte-exact wire comparison of the DS0.

The temporary digital answerer was stopped and the normal
`bri-test-answerer.service` restored after these tests. User-applied Cisco
CLEARMODE configuration was left in place. No protocol source was changed.


## Asterisk CLEARMODE bridge now carries V.110 data

After installing `tools/asterisk/codec_clearmode.c` on Asterisk 22.5.1 and
adding CLEARMODE to both modem templates and endpoint 6501, calls via
Asterisk negotiate CLEARMODE on both bridge legs. Live `core show channel`
reports native/read/write formats CLEARMODE and no read/write transcoding.
Both Eicon controllers confirm UDI BC `88 90` and V.110 async 9600.

Both initial V.110 calls passed three exact 201-byte echo lines. Longer
7910 run r2 and standalone 7900 run r3 each pass ten exact lines (2010
payload bytes sent and echoed per call). The concurrent 7900 r2 run passed
line 0, then returned a truncated line 1; its final RTP report records four
transmit packets lost (80 ms), with zero received packets lost. This is
consistent with digital data damage from packet loss; it is not proof of
where that loss occurred. The standalone repeat was clean.

Evidence: `artifacts/eicon-v110-clearmode-asterisk-7910-r2/` and
`artifacts/eicon-v110-clearmode-asterisk-7900-r3/`; failed concurrent run
`artifacts/eicon-v110-clearmode-asterisk-7900-r2/`.
7910's successful RX tap now contains 2153 instances of DS0 codeword 7F,
which was absent from the earlier corrupted PCMU comparison. No V.90
improvement is established by these ISDN rate-adaptation tests.

Acknowledged V.120 at 64 kbit/s also passes three exact 201-byte echo lines
on each extension through Asterisk CLEARMODE:
`artifacts/eicon-v120-clearmode-asterisk-7900-r1/` and
`artifacts/eicon-v120-clearmode-asterisk-7910-r1/`.
The temporary ISDN service was stopped and the normal Eicon answerer
restored (confirmed active) after validation. CLEARMODE module and endpoint
configuration remain enabled on Asterisk.
