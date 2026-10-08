# Eicon other-mode hardware calls, 8 October 2026

Fifteen sequential calls used the existing isolated modem build in
`/tmp/eicon-current-20261008` inside Tower's `v90modem-sip` container.
The caller ran on the wired LAN, with fixed 40 ms RTP prefetch, PCMU endpoint
7910 and PCMA endpoint 7900. No modem DSP, answerer or gateway configuration
was changed. The initial matrix waited at least 65 seconds between calls to
the same controller. Follow-up calls also left at least that interval.

Every successful row reached the menu, selected text echo, and returned three
complete, byte-exact `ECHO:` lines. These are short connection/payload tests,
not sustained binary-transfer or PPP results.

| Requested mode | PCMU | PCMA |
|---|---|---|
| V.34 | Initial call failed before CONNECT; repeat passed at 21600 bit/s | Passed at 24000 bit/s |
| V.32bis | Passed at 14400 bit/s | Passed at 14400 bit/s |
| V.32 | Passed at 9600 bit/s | Passed at 9600 bit/s |
| V.22bis | Passed at 2400 bit/s | Passed at 2400 bit/s |
| V.22 | Passed at 1200 bit/s with longer echo deadline | Passed at 1200 bit/s with longer echo deadline |
| V.92 | Passed via V.90 fallback, 56000 downstream / 31200 upstream | Passed via V.90 fallback, 56000 downstream / 31200 upstream |

The V.32 path shares the V.32bis V.8 label and datapump, but both CONNECT
responses confirm its 9600 bit/s ceiling. The V.92 calls explicitly log
V.90 negotiation; they do not establish V.92 interoperability.

## Follow-up findings

Both initial V.22 calls reached CONNECT 1200, LAPM and the menu, but the
12-second echo deadline expired during the first 206-byte payload. Repeats
using a 60-second deadline passed all three lines in both laws. Complete
responses took 13.80–14.58 seconds. The original failures were harness
timeouts, not evidence of corrupt payload. The harness now defaults to
20 seconds and exposes `--echo-timeout`.

The first V.34 PCMU call disconnected after about 46 seconds with no CONNECT.
Its repeat reached data, but only after the local B1 SNR policy requested
21600 bit/s and retrained; it then passed all three 201-byte echoes. Thus
V.34 connected successfully in both laws, but this batch does not establish
reliable first-attempt startup or sustained V.34 operation.

## Evidence and reproduction

`artifacts/eicon-other-modes-20261008-r1/` contains the twelve-call matrix.
`artifacts/eicon-other-modes-20261008-recheck/` contains the V.22 repeats and
V.34 PCMU repeat. Each call preserves the summary, PTY transcript, modem log,
G.711 TX/RX taps and engine I/O schedule. Rate claims above use CONNECT and
engine negotiation records, not the requested mode alone.

The harness maps engine mode names to the correct AT+MS carrier names
(`v22` to V22B, `v22-1200` to V22, `v32bis` to V32B), always with automode 0.
Its legacy default matrix remains V.34/V.90. Run individual other modes from
the Tower build, with the isolated SIP/RTP ports idle:

```sh
ME_JB_MS=40 python3 tools/eicon_call_batch.py artifacts/v22-recheck \
  --mode v22-1200 --law ulaw --repeats 1 --lines 3 --echo-timeout 60
```

The updated harness passed Python syntax compilation and was exercised by
these live calls. No DSP protocol changes were made.

## Explicit 33.6 kbit/s trials

Four further Tower LAN calls tested the maximum rather than relying on the
3200-baud default (whose initial ceiling is 31200 bit/s). All used
`ME_V34_BAUD=3429 ME_V34_BPS=33600 ME_JB_MS=40` with the existing binary.

Two strict calls used `AT+MS=V34,0,33600,33600,33600,33600`. Neither law
reached CONNECT. Our logged INFO1c included `3429=14`, the 33600 bit/s row,
but both peer INFO1a messages selected 3200 baud in both directions and
projected N=13 (31200 bit/s) for our transmit direction. Both calls ended
in Phase 4. This establishes a failed strict-rate trial, not that the card
or our implementation can never support 33600 bit/s.

Two further calls retained the full 33600 offer but allowed lower rates with
`AT+MS=V34,0`. PCMU initially selected the same 3200-baud profile, retried
at 2400 baud, and reached data with MP rates of 16800 bit/s toward us and
21600 bit/s toward the Eicon. It passed three exact echoes. PCMA did not
reach CONNECT. The higher starting profile therefore did not produce a
higher working rate on these calls.

Evidence: `artifacts/eicon-v34-33600-20261008-r1/` and
`artifacts/eicon-v34-maxoffer-20261008-r1/`. The temporary strict-rate runner
is preserved with the local evidence as `strict_runner.py`. No persistent
DSP defaults or peer settings changed. Live Eicon configuration could not be
read because SSH authentication failed; card settings and the reason for
its 3200-baud selection remain unverified. These four trials demonstrate no
33.6-kbit/s connection on this setup.

## Does 3429 baud itself work?

Fresh local Mac duplex tests on 8 October distinguish the symbol rate from
the bit rate. `make v34_duplex_test` relinked against the current SpanDSP
library. At 3429 baud / 21600 bit/s, both G.711 laws trained both endpoints
and recovered over 16000 bits per direction with zero errors. At 3429 /
33600 both endpoints trained, but PCMU counted 282/166 errors and PCMA
714/485 errors. A linear PCM control at 33600 counted 564/0 errors, so the
high-rate failure is not explained by G.711 quantization alone. These are
single zero-delay loopback runs on the Mac, not Tower hardware calls or an
acquisition-delay sweep. Logs are in `artifacts/v34-3429-check-20261008/`.

Thus 3429 baud demonstrably works at 21600 in the current duplex harness;
33600 payload is not clean in these runs. None of the Eicon calls above
actually selected 3429 baud despite our offering it.

## 33600 loopback error investigation

The errors are a failing modem-path test, not an acceptable successful
connection. A fresh `v34_data_test` passed all 420 mapper/decoder cases.
The duplex harness only starts counting after matching the far end's 32-bit
PRBS prefix, and reports a failing exit status when payload errors occur.
Captured INFO1a confirms actual 3429-baud selection in the local tests.

At 33600, an eight-delay sweep (0, 3, 7, 11, 17, 23, 31, 40 samples) passed
only delay 23 over linear PCM; neither G.711 law passed any delay. Extending
the delay-23 linear run to 100000 received bits per direction then counted
2066/17240 errors. The zero-delay linear run at that length counted 801/287;
the same 3429-baud linear control at 21600 counted zero errors in both
directions over 100000 bits. A short pass therefore does not establish
reliable 33600 operation even in the unimpeded local bearer.

The transmitted and received pre-decoder symbol dumps align at an offset
of 120 symbols with mean gain approximately one and negligible mean phase
error. Residual RMS is about 0.5–0.6 lattice units, with approximately
33–35 dB SNR measured against the transmitter's actual symbols. This
independently confirms distortion upstream of bit grading. Ordinary
21-tap symbol-domain least-squares fitting removes little of it; a
seven-phase widely-linear fit reduces held-out RMS from 0.532 to 0.379
(caller RX) and 0.520 to 0.323 (answer RX). This is consistent with the
previously documented periodic image problem at the 4/7 carrier ratio,
but does not by itself locate the remaining distortion in TX or RX.

Turning off data equalizer adaptation substantially worsened the long
linear test (28909/22357 errors). Disabling carrier tracking or freezing
timing did not make both directions clean. No DSP constants or defaults
were changed to obtain a pass; a corrective DSP change remains open.
Diagnostic logs, symbol dumps, analysis scripts and the delay sweep summary
are preserved in `artifacts/v34-3429-diagnosis-20261008/`.

## Completed 3429/33600 receiver repair

The investigation above is superseded by the B1 conditioning and data-clock
repair described in [v34_3429_dense_data.md](v34_3429_dense_data.md). With
actual defaults, 23 completed delay/bearer rows carry over 46 million bits
with zero payload errors. One PCMA delay still fails earlier MP acquisition.
The Tower build also passes long 33600 loopbacks in both laws. Fresh strict
33600 Eicon calls still select 3200/N=13 in INFO1a and fail the forced-rate
call; live 33600 interoperability remains unestablished.
