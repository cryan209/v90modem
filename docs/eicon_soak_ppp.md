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
for PCMU. Evidence is in `artifacts/eicon-ppp-v90-alaw-20261008/` and
`artifacts/eicon-ppp-v90-ulaw-20261008/`.

The first transfer attempt crossed the answerer deployment restart. A second
attempt was invalidated by the Docker snapshot pause and deliberately stopped.
Neither is counted as a sustained-transfer result. The fresh runs are
`artifacts/eicon-soak-v90-ulaw-20261008-r3/` and
`artifacts/eicon-soak-v90-alaw-20261008/`; both passed all three phases
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
progress and summaries. `artifacts/eicon-answerer-soak-20261008.log`
preserves the peer journal; `artifacts/eicon-soak-build-20261008.sha256`
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

Evidence is preserved in `artifacts/eicon-http-ppp-v90-ulaw-20261008/` and
`artifacts/eicon-http-ppp-v90-alaw-20261008/`, including downloaded binaries,
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

Evidence: `artifacts/eicon-throughput-baseline-20261008/` and
`artifacts/eicon-throughput-jb40-20261008/`. This is one call per arm and
shorter than the earlier 26-minute soaks. The 40 ms setting is verified on
the wired Tower LAN path; the engine's general 200 ms default is unchanged.

HTTP confirmation at 40 ms used a 512 KiB GET followed by a 512 KiB POST
on one PCMU PPP call. Both passed exact length and SHA-256 checks. Download
was **6,212.5 B/s in 84.392 s**, versus the earlier 200 ms HTTP result of
5,227.9 B/s (1 MiB). Upload was **3,474.9 B/s in 150.879 s**, essentially
unchanged. HTTP download therefore reached **88.8%** of the raw downstream
rate, close to 90%; it must not be reported as exceeding 90%. The comparison
uses different payload lengths and includes HTTP connection/setup time.
Evidence: `artifacts/eicon-http-ppp-jb40-20261008/`.

```sh
python3 tools/eicon_soak_test.py artifacts/lan-http \
  --test ppp --http-bytes 1048576 --jitter-buffer-ms 40
```
