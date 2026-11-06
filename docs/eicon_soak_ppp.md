# Eicon sustained transfers and PPP

The answerer source is now preserved in `tools/eicon/bri_test_answerer.py`,
with its shared binary test in `tools/eicon/stream_test.py`. Install both
beside each other in `/usr/local/sbin` on eicon420. The existing service and
four-channel configuration continue to apply. Menu options 1–5 and Q retain
their existing meanings; option 6 selects timed binary traffic and option 7
starts an explicit PPP session.

Run from the LAN on Tower. The Mac WireGuard path has independently measured
RTP loss and is unsuitable for grading hardware performance. The current
isolated modem build is `/tmp/eicon-current-20261008` in `v90modem-sip`.
Only one caller may use SIP port 5078 / RTP port 14600 at a time.

```sh
python3 tools/eicon_soak_test.py artifacts/soak-v90-ulaw --mode v90 --law ulaw
```

Defaults are 610 seconds download, 610 seconds upload, then 310 seconds
simultaneous download/upload, on one call. Success requires both endpoints
agree on byte counts, zero sequence/CRC/exact-payload errors, and at least
600 seconds of received traffic for the one-direction tests or 300 seconds
in **both** directions for the simultaneous test. Startup and end-marker waiting
are excluded from the receive span; queued payload drain counts as received
traffic. Every frame carries 1024 bytes
from SHAKE256 of the direction and sequence, preventing V.42bis from turning
a repetitive text test into a misleading throughput result. Frames and
end markers are handled independently in each direction with bounded
application buffering. A timeout or dropped carrier fails the run.

Per-direction progress is saved every 30 seconds; `summary.json` contains
both endpoints' results. The modem log, DS0 taps and I/O schedule are kept
in the same output directory. Short runs can exercise the protocol with
`--download-seconds 3 --upload-seconds 3 --duplex-seconds 3`, but deliberately
fail the long-run duration requirement. Local framing checks:

```sh
python3 tools/eicon/stream_test_test.py
```

## PPP

PPP belongs above the modem byte stream. The engine must transport its
async HDLC bytes unchanged; it does not need to implement IP or LCP itself.
The menu explicitly hands its connected descriptor to host `pppd`, rather
than guessing from incoming bytes. Both hosts need the Debian `ppp` package,
Linux PPP support and `/dev/ppp`. A Docker caller needs `--cap-add NET_ADMIN`
and `--device /dev/ppp`. The ordinary Tower modem container has neither;
a temporary `eicon-ppp-test` runner supplies them.

```sh
python3 tools/eicon_soak_test.py artifacts/ppp-v90-ulaw --test ppp
```

Option 7 supplies a distinct pair per tty: ttyds1 uses 10.254.91.1/.2,
ttyds2 10.254.92.1/.2, ttyds3 10.254.93.1/.2, ttyds4 10.254.94.1/.2.
The server owns .1; the caller owns .2. The test waits for IPCP then checks
30 ICMP replies with zero loss. `pppd.log` and `ping.log` preserve evidence.
Sessions have a 30-minute limit and LCP echo monitoring. This is an
unauthenticated lab IP link; no default route, proxy ARP or Internet sharing
is configured. PPP compression is disabled to simplify measurement.
The caller hangs up after PPP teardown; the answerer does not inject menu
text into the PPP stream.

Options follow the [PPP project's pppd manual](https://ppp.samba.org/pppd.html).
This PPP session check does not substitute for sustained TCP/IP testing.

## Deployment

Back up the live answerer, install both Python files, syntax-check them and
restart `bri-test-answerer` only when its calls are idle. Install `ppp` with
`apt-get install ppp` on the answerer and PPP runner. The service configuration
remains `/etc/bri-test/config.json`; channel numbering determines the PPP
address pair.

For repeatable test runs, copy `tools/eicon/` and `tools/eicon_soak_test.py`
into the modem build. A second concurrent call must use the other SIP account
(`--law alaw`) and separate `--local-port 5080 --rtp-port 14800`. Snapshotting
a running test container with Docker's default `commit` **pauses it** and
invalidates modem measurements. Use an existing build image, or explicitly
`docker commit --pause=false` while idle; never pause an active media test.

## Live validation, 8 October 2026

PPP completed IPCP and 30/30 ICMP replies with zero loss in both laws,
with normal LCP teardown. Mean RTT was 398.277 ms for PCMA and 389.972 ms
for PCMU. Evidence is in `reference-data/artifacts/eicon-ppp-v90-alaw-20261008/` and
`reference-data/artifacts/eicon-ppp-v90-ulaw-20261008/`.

The first transfer attempt crossed the answerer deployment restart. A second
attempt was invalidated by the Docker snapshot pause and deliberately stopped.
Neither is counted as a sustained-transfer result. The fresh runs are
`reference-data/artifacts/eicon-soak-v90-ulaw-20261008-r3/` and
`reference-data/artifacts/eicon-soak-v90-alaw-20261008/`; both passed all three phases
on one uninterrupted call per law.
Each call lasted about 26 minutes 37 seconds. Every payload frame was checked
exactly, and both ends agreed on all byte counts. Neither modem log records
a retrain; final RTP statistics report zero lost packets in both directions.

| Law | Test | Download bytes | Upload bytes | Download receive span (s) | Upload receive span (s) |
|---|---|---:|---:|---:|---:|
| PCMU | download | 3,315,712 | 0 | 623.06 | 0.00 |
| PCMU | upload | 0 | 2,236,416 | 0.00 | 621.24 |
| PCMU | duplex | 1,505,280 | 1,157,120 | 324.62 | 321.34 |
| PCMA | download | 3,141,632 | 0 | 623.60 | 0.00 |
| PCMA | upload | 0 | 2,236,416 | 0.00 | 621.24 |
| PCMA | duplex | 1,427,456 | 1,157,120 | 325.28 | 321.34 |

Total checked payload: **16,177,152 bytes**, with zero sequence, CRC or
exact-payload errors. Download alone averaged 5.32 kB/s (PCMU) / 5.04 kB/s
(PCMA); upload alone 3.60 kB/s in both. During simultaneous traffic, measured
download was 4.63 / 4.39 kB/s, with upload 3.59 kB/s. Rates use receiver
elapsed time and decimal kB; these are observed payload rates, not raw
negotiated bit rates. The soak and PPP calls negotiated V.90 56000
downstream / 31200 upstream with LAPM and V.42bis.

The local artifacts include the full modem logs, DS0 taps, schedule,
progress and summaries. `reference-data/artifacts/eicon-answerer-soak-20261008.log`
preserves the peer journal; `reference-data/artifacts/eicon-soak-build-20261008.sha256`
identifies the tested modem binary and core sources.

The final source was loaded at an idle interval after all tests. The temporary
PPP runner and snapshot image were removed after copying evidence locally.
The existing Tower modem container remains running. These results establish
the tested V.90 LAN calls and basic PPP sessions; native V.34 long runs and
ten-minute TCP/IP soaks are separate tests.

## HTTP over PPP

The same caller can download and POST an incompressible binary payload:

```sh
python3 tools/eicon_soak_test.py artifacts/http-ppp-ulaw \
  --test ppp --http-bytes 1048576 --law ulaw
```

Start the temporary endpoint on Eicon before dialling:

```sh
python3 tools/eicon/http_test_server.py --bind 10.254.93.1 --port 19890
```

It waits for IPCP to assign the specified address and binds only that PPP
address. For PCMA on ttyds1, use 10.254.91.1. If another channel answers,
use its assigned address from the menu's PPP banner. End the temporary HTTP
server after the test. No LAN or Internet HTTP listener is configured.

The caller disables HTTP proxies, requests `/download/<size>`, checks the
received length and SHA-256, then POSTs the same payload to `/upload` and
checks the server's received length and SHA-256. HTTP status, byte counts,
hashes, elapsed time and payload throughput are saved under `http` in
`summary.json`. The downloaded binary and upload's JSON receipt are also
preserved. The endpoint accepts up to 16 MiB; the CLI's `--http-bytes 0`
default retains the original ping-only PPP test.

This validates HTTP/TCP/IP through the modem PPP link to its peer. External
Internet routing, DNS and HTTPS are separate from this local endpoint test.

### HTTP hardware results, 8 October 2026

Both laws passed one 1 MiB HTTP GET and one 1 MiB POST over the same PPP
session, with status 200, exact byte counts and matching SHA-256 at both
receivers. Payload was incompressible.

| Law | Download seconds | Download kB/s | Upload seconds | Upload kB/s |
|---|---:|---:|---:|---:|
| PCMU | 200.572 | 5.228 | 301.418 | 3.479 |
| PCMA | 200.209 | 5.237 | 301.438 | 3.479 |

Tower route evidence records `ppp1` for 10.254.93.1 and `ppp0` for
10.254.91.1. The HTTP server logs requests from the corresponding PPP client
addresses. Neither HTTP call logs a modem retrain, and final RTP reports
zero packet loss both ways.

Evidence is preserved in `reference-data/artifacts/eicon-http-ppp-v90-ulaw-20261008/` and
`reference-data/artifacts/eicon-http-ppp-v90-alaw-20261008/`, including downloaded binaries,
upload receipts, route records, modem/PPP logs and DS0 taps. The temporary
HTTP listeners and PPP runner were removed after the test.

## Tower LAN throughput profile, 8 October 2026

A controlled PCMU comparison kept the same binary, V.90 56000/31200 call,
LAPM parameters and 90/90/30-second binary probe. The only media setting
changed was fixed RTP prefetch: 200 ms versus 40 ms. The caller can now
select it explicitly with `--jitter-buffer-ms 40`; leaving this flag absent
retains the engine/environment default. The requested setting is saved in
new summaries. `--test probe` grades the selected short receive spans
(with a two-second tolerance), rather than claiming a ten-minute soak.

```sh
V42_STATS=5 python3 tools/eicon_soak_test.py artifacts/lan-perf \
  --test probe --download-seconds 90 --upload-seconds 90 --duplex-seconds 30 \
  --jitter-buffer-ms 40
```

| PCMU binary payload | 200 ms | 40 ms |
|---|---:|---:|
| Download alone | 5,320.5 B/s | 6,444.6 B/s |
| Upload alone | 3,583.8 B/s | 3,590.2 B/s |
| Simultaneous download | 4,673.2 B/s | 6,447.0 B/s |
| Simultaneous upload | 3,566.4 B/s | 3,576.7 B/s |

Both probes passed all exact payload/count checks, with no retrains or
reported RTP loss. The improved download is **92.1% of 56,000/8 B/s**;
simultaneous download also reaches 92.1%. Steady downstream LAPM information
rate rose from 5,425 to 6,572 octets/s. Steady saturated upstream I-frame
acknowledgement RTT fell from 345 to 194 ms, and both arms had zero LAPM
retransmissions. The negotiated window was k=15 and N401=128 in both arms.

The window permits 1,920 information octets outstanding. At roughly 350 ms
round trip it cannot sustain the full 7,000 B/s raw downstream rate. The
latency reduction and matching throughput increase are consistent with that
window limit. The caller's `idle-window` statistic describes its **upstream**
transmitter; it does not directly measure the Eicon's downstream window stalls.
Statistics must be separated by test phase; averages across download,
upload and duplex periods are not continuous-direction throughput.

Evidence: `reference-data/artifacts/eicon-throughput-baseline-20261008/` and
`reference-data/artifacts/eicon-throughput-jb40-20261008/`. This is one call per arm and
shorter than the earlier 26-minute soaks. The 40 ms setting is verified on
the wired Tower LAN path; the engine's general 200 ms default is unchanged.

HTTP confirmation at 40 ms used a 512 KiB GET followed by a 512 KiB POST
on one PCMU PPP call. Both passed exact length and SHA-256 checks. Download
was **6,212.5 B/s in 84.392 s**, versus the earlier 200 ms HTTP result of
5,227.9 B/s (1 MiB). Upload was **3,474.9 B/s in 150.879 s**, essentially
unchanged. HTTP download therefore reached **88.8%** of the raw downstream
rate, close to 90%; it must not be reported as exceeding 90%. The comparison
uses different payload lengths and includes HTTP connection/setup time.
Evidence: `reference-data/artifacts/eicon-http-ppp-jb40-20261008/`.

```sh
python3 tools/eicon_soak_test.py artifacts/lan-http \
  --test ppp --http-bytes 1048576 --jitter-buffer-ms 40
```

## 4 November 2026: vmhost to new BRI spans 7920/7930

Deployed CT117 build `d6859491`, analogue V.90 role, PCMA, reaches
56000 downstream / 31200 upstream on both targets. The answerer is
`bri-new-spans.service` on eicon420 (SSH user `scottcryan`), not
`bri-test-answerer.service`: 7920 uses controller 8 / ttyds14–15 and
7930 controller 7 / ttyds12–13.

Three-minute simultaneous binary attempts receiver-check **891904 downstream
bytes on 7920** and **890880 on 7930**, zero payload errors, with last received
blocks around 197/195 seconds. Answerer progress independently checks
**647168/646144 upstream bytes** through the 180-second checkpoints, zero
errors. Neither duplex call completes its final exchange; these are not
complete duplex passes.

Separate upstream checks do complete with matching sender/receiver totals:

| Target | Receiver-verified upstream bytes | Receive span | Result |
|---|---:|---:|---|
| 7920, unrestricted application writes | 737280 | 204.553 s | 720 blocks, zero sequence/CRC/exact-payload errors; final report received |
| 7930, application paced at 2800 bytes/s | 503808 | 179.594 s | 492 blocks, zero errors; final report received |

An unrestricted 7930 upload checks **726016 bytes**, zero errors, before
carrier loss leaves **11264 queued payload bytes** unconfirmed. Pacing is an
application diagnostic, not a modem/DS0 change or a proven fix. A paced 7920
attempt is invalidated by overlapping scratch-driver cleanup; exclude it.
Do not infer an error-free unrestricted duplex qualification from these
separate upstream successes. End-marker/completion failure remains unresolved.

Provenance on vmhost: `/root/bri-long-20261104.log`,
`/root/bri-upload-20261104.log`, `/root/bri-paced-20261104.log`; modem logs/taps
under `/root/slmx/bri-{long,upload,paced}-20261104/`. These scratch drivers use
the deployed build, not the Mac's pending DSP changes. Receiver observations
come from eicon420's `bri-new-spans` journal. No DSP constants, G.711 processing,
thresholds or defaults changed.

## 6 November 2026: 7920/7930 modulation matrix

Build `1662ce8a` (clean build in a CT117 tmpfs tree), PCMA leg, account 6002,
our modem originating. The 7920/7930 answerers are now pppd bundle members
(`eicon-79x0-*.service`, card init `AT+MS=V90,1`, `require-pap`), not the
`bri-new-spans` echo menu. Grade per call: CONNECT, then an LCP probe
(Configure exchange, then 190-byte Echo-Requests every 0.5 s with the echoed
payload compared byte-exact). Two rounds × five arms × two targets = 20 calls,
alternating targets.

| Arm (`AT+MS`, role) | Card's V.8 offer | Trained | 7920 | 7930 |
|---|---|---|---|---|
| V90,0 analogue | V.34, V.90 | V.90 56000 down / 31200 up | 2/2 | 2/2 |
| V90,0 digital | V.34 only | V.34 24000 | 2/2 | 2/2 |
| V34,0 | V.34 | V.34 | 21600 2/2 | 24000 2/2 |
| V92,0 analogue | V.34, V.90 (no V.92) | V.90 56000/31200 | 2/2 | 2/2 |
| V32B,0 | V.32bis | V.32bis 14400 | 2/2 | 2/2 |

All 20 calls connect, LCP opens, **zero FCS errors**, and 45–47 of 46–48
echoes per call return byte-exact (the missing one is in flight at
hang-up). Every call ends at about 37.5 s after CONNECT. The answerer's pppd
sends `TermReq "Authentication failed"` 30 s after LCP opens, because the probe
does not do PAP. This is the rig's grade limit, not a modem drop.

- **V.90 digital role** cannot run PCM here. The Diva is itself the digital
  end, so its V.8 menu omits V.90 and the call is plain V.34 (V.90 needs one
  analogue end). Our digital role needs an analogue peer.
- **V.92** is not offered by the card; the analogue role falls back to V.90.
- **V.34 on 7920 retrains itself downward, 2/2.** The B1 estimate reads
  30.9 dB as "about 21600", while the same span reads 31.3–32.1 dB as 24000 on
  the V90-digital arm. We cap our TX, ask for 21600 and retrain per §11.5. The
  retrained B1 then reads **20.4 / 22.2 dB** (16800/19200 class), about 10 dB
  worse than before the retrain, and the call stays at 21600. 7930 stayed at
  24000 (31.2–33.3 dB) and did not retrain. Data-mode SNR on these calls reads
  32.7–33.5 dB while the rate estimate still says 24000, so 24000 at ~33 dB
  is itself conservative.

**The "10 dB lower" was a B1 misreading, not the line (same day).** An offline
`v90_engine_replay --dial --sync --echo-ref` of calls 05 and 15 reproduces both
readings exactly. After the retrain, B1 carries a residual carrier of
0.1366 and 0.1106 degrees/symbol (1.21 and 0.98 Hz); every first training reads
under 0.005. Across B1's 128 symbols that ramp alone caps the normalised
correlation at 0.996–0.997, which reads as 20–22 dB. DATA is entered
de-rotated by that same rate and measures 30.3–33.7 dB. The far end is not
off frequency: a z⁴ estimate on the raw tap over 25 s of post-retrain data gives
+0.04 and +0.13 Hz from nominal. The error is our NCO, which sits at 1827.4 Hz
against 1828.6. Fixed: `v34rx_data.c` now reads B1's correlation with the
measured ramp removed, as DATA will see it. On replay the post-retrain readings
become 29.4 / 31.4 dB, and the six first trainings are unchanged. This is a
measurement only.

Three defects remain open, all ours:
1. **Phase 4 S detection misses on half of the entries.** FIXED the same
   day, see below. The eye explanation first recorded here was wrong.
2. **The carrier loop drags while stuck in S.** It tracks TRN symbols against
   S targets for the whole timeout. On first trainings TRN then pulls it back
   to 1828.6. On both retrains (the card's J asks us for 4-point TRN there)
   it stays 1.2 Hz low into MP, where tracking freezes.
3. **The rate rule at 3200 baud** (FIXED the same day: calibrated table, live 26400
   on 12 of 12 with no retrain; see `docs/v34_data_mode_rates.md`) was the uncalibrated (SNR+13)/6. With
   the 1.0 dB margin, 24000 needs B1 >= 31.0 dB, and 7920 reads 30.9 against
   Phase 4 TRN's 32.9. So its first B1 asks for a retrain to 21600 on a line
   DATA decodes at 33 dB (distance to grid 0.036).

**Phase 4 S: we enter after the S has gone (same day).** The receiver's S
was not misread. As caller we enter Phase 4 only after Phase 3 has
confirmed the answerer's S (11.4.1.1: S is answered with J'), and the Phase
4 conditioning reset then costs about 37 bauds before |z| settles. Against
the 7920, Phase 3 confirmed S about 20 ms later than against the 7930 (J
lasted 855 against 835 ms), so the Phase 4 receiver saw only the last
~20 alternating bauds of S-bar before a 16-point TRN, and on a retrain no
alternation at all. The 24-of-32 window can't be met then, and the stage
sat out its 2048-baud timeout (640 ms) while TRN went by. HEAD timed out
the same way on the RasFinder 3429 fixture. `v34rx.c` now also takes TRN
as begun when S/S-bar's +/-90 alternation (10.1.3.7) stops, junction or
not. Two triggers:
- a run of 8 strict alternations has been seen and the last 8 bauds hold at
  most 2 (TRN alternates with probability 1/4);
- 28 of the last 32 bauds carry signal and at most 12 alternated.

Replays (`--dial --sync --echo-ref`) of the eight V.34/V.90-digital calls
went from 4 S timeouts (two first trainings, both retrains) to none, with
hand-over at baud 55–96. Two more first trainings had reported a
"junction" at baud 124 and 154, which falls inside TRN; they now hand over
at 56 and 66. The 7930's junction entries are unchanged.
Loopback `v34_duplex_test` over 26 rows x 8 delays: 206/208 on both builds,
the same two rows failing. The fallback never fires there, because loopback
always finds the junction. Live A/B, 24 calls, HEAD and fix alternated per
round, v34 and v90d against both cards: all 24 connect and pass the LCP
probe. Retrains: HEAD 3 of 12, fix 4 of 12. First B1 means: HEAD 31.43,
fix 31.57 dB. Every retrain is a first B1 of 30.7–31.0 dB against the 31.0
cut-off, i.e. defect 3, and the extra 640 ms is not visible in time to
CONNECT. So the fix is neutral live and removes a timeout. New fast row:
`reference-data/artifacts/eicon7920-p4-late-s-20261106` (14 s of call 03);
it fails on HEAD and passes with the fix.

**B1 on the RasFinder 3429 fixture is luck.** With `--sync`, B1's
normalised correlation reads 1.000 (36 dB) only when Phase 4 S exits
between baud 2016 and 2056. Any other exit, by timeout or by the new rule,
reads 0.999 (27.1–28.2 dB), with TRN SNR flat at 33.3–34.1. HEAD's
2048-baud timeout landed in that window, and the fast row had pinned
"1.000". The row now asserts what it exists for: B1 8 symbols after E,
trellis state 13, correlation >= 0.99. Without `--sync` the same replay
flips between 28 and 36 dB with CPU load, so don't grade B1 on it unsynced.
Defect 2 (carrier drag while stuck in S) now arises only when the fallback
doesn't fire.

A/B provenance: `/root/e79_ab.py`, batch log `/root/e79/ab1.log`, per-call
`/root/e79/ab1/`, server logs in CT 117 `/tmp/e79runs/ab1/`.

Provenance on vmhost: driver `/root/e79_matrix.py` (uses `/root/slm_targets.py`),
batch log `/root/e79/m1.log`, per-call summaries `/root/e79/m1/`, server logs
`/root/e79/m1-ct117-logs.tgz`.

## 6 November 2026: six SIP destinations and PAP/IP boundary

A wired CT117 probe using `/tmp/e79/sip_v90_modem` dialled 7900, 7910,
7920, 7930, 7940 and 7950 with `AT+MS=V90,0`, analogue role. The first
four trained at **31200 bit/s in V.34**, not V.90: the captured 7900 V.8
peer menu offered V.34 only. LCP opened on all four. Exact LCP echoes
returned **21/22, 21/22, 22/22 and 22/23** respectively; PPP FCS errors
were **1, 0, 0 and 0**. These probes omit PAP and do not qualify IP service.
7950 trained at **24000 bit/s**, but supplied no decoded PPP frames during
45 seconds after CONNECT. 7940 returned **NO DIAL** after 8 seconds.

An authenticated PCMA call to 7900 using PAP user `v90modem` assigned peer
**10.254.104.13**, completed five pings with **0% packet loss**, and finished
in **34.7 seconds**. Separate 7910 (PCMU and PCMA), 7920 (PCMA), and 7930
(PCMA, V.34 ceiling 26400, trained 24000) calls did not complete IP setup.
Server diagnostics confirm PAP success followed by **`Couldn't create ppp
interface mtppp0: File exists`** on ttyM4, ttyds14 and ttyds13. Thus at least
some 7910 calls are answered by the MultiTech ttyM services, rather than the
Diva ttyds services. Both sets of peers force the same interface name.
Do not label these failures bad credentials or a demonstrated codec defect.

The PBX has routes for all six numbers via 192.168.88.62. A missing 7940
answerer was added as `eicon-7940-7.service`, ttyds7, bound explicitly to Diva
controller 3, copying the existing 7930 PPP policy. It remains ready but a
repeat call still failed before SIP connection; card-to-gateway span routing
is unverified. The proposed unique PPP interface-name fix was blocked by
automatic approval review pending explicit approval; it was **not applied**.
Gateway SSH access as scottcryan was denied. A separate unrestricted-digital
17950 call (`AT+MS=CLEAR,0`, `ME_DATA_FRAMING=ppp`) reported CONNECT 64000 but
received no LCP and is not a working PPP connection.

`tools/eicon_ppp_direct.py` now accepts `--ppp-user` with password in
`EICON_PPP_PASSWORD`, hides passwords in pppd diagnostics, supports an explicit
binary/SIP account, and requires zero ping loss for a pass. It clears both
inherited G.711 force flags before selecting the requested law. No DSP or
sample-path changes were made. PAP is RFC 1334 §2; async PPP framing remains
RFC 1662 §4, with the existing clear-channel conversion in §5 on the DS0.

Provenance on vmhost: `/root/e79/sip-six-20261106/`; CT117 modem evidence:
`/tmp/e79runs/sip-six-20261106/` and `/tmp/e79-auth-*-20261106/`.
