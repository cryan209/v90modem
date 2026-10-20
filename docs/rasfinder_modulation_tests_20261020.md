# RasFinder modulation checks, 20 October 2026

Twelve bounded outbound calls test the current working tree on extensions
6311/6312/6313. V.34 reaches LAPM and the readable MULTITECH login banner
on all three lines at `CONNECT 24000`. V.90 remains inconsistent: one
48,000/21,600 call delivers the banner and holds about 61 seconds, one
45,333/21,600 call decodes white and retrains, and one reaches login only
after V.34 fallback. V.32bis times out training on every line.

This is a current-state hardware check, not a controlled improvement A/B.
The 16 October results in `v90_rasfinder_phase4.md` already reached login
on 15 of 19 V.90 data-mode calls; the 17 October interleaved batch reached
V.90 data on 4/6 current-main calls, with login on three. Today's three
V.90 offers do not demonstrate a higher success rate or establish a
regression. Direct V.34's three 24,000 login results are encouraging
relative to the earlier 16,800 fallback observations, but differ in host,
date and call path.

## Method and provenance

Tower SSH timed out. Calls instead ran on the wired vmhost CT 117, an
existing rig host, from the isolated `/root/rf-current-20261020` tree.
No other modem server was stopped. The existing `/root/rf-head` tree was
left intact. Four available CPUs supported three concurrent calls, one
per extension, with separate u-law accounts 2901/2902/2903, SIP ports
5090/5092/5094, RTP ports 15000/15100/15200 and PTYs. Each call was allowed
90 seconds after dialling; each round was followed by 90 seconds without
calls before redialling those lines.

The source snapshot is HEAD `3d21a8ff` plus the existing uncommitted tracked
changes. `source-manifest.json` hashes every archived input;
`working-tree.patch` records those existing changes. The isolated build
used vendored pjproject and SpanDSP, `make -j4 CC=clang sip_v90_modem`,
with a clean SpanDSP source build to account for private-header changes.
The clang-only selective V.90 acquisition optimization was enabled.
Binary SHA-256:
`11d323aa7ad230236c94719e13195161e3fb900b8cbd013d2782a3a3810a9613`.

The existing `tools/soak/rasfinder_call.sh` drove the calls. `ME_MODE`
selected each power-on offer, PCMU was forced, and V.90 offers used the
previous comparison ceiling `ME_V90_UPSTREAM_MAX_BPS=21600`. Other DSP
switches were left at defaults. The harness uses automatic LAPM XID
compatibility. After a readable login appeared, a separate observer sent
one empty CR and retained any response; it did not submit credentials.
This does not establish an error-free payload transfer or throughput.

## Per-call results

`CONNECT` is the DTE-reported rate; V.90 upstream/downstream rates come
from the startup trace. V.34 MP rates are retained in `results.json`.
Durations use each call's own `[TRACE +Nms]` clock, measured from DATA
entry to the first logged loss/retrain or harness termination, not from
log interleaving across processes.

| Offer | Extension | Result |
| --- | --- | --- |
| V.90 | 6311 | V.34 fallback: 31,200 then 19,200; LAPM + login at 19,200. Peer rate renegotiations at +66.0 s and +88.7 s. Not a V.90 data success. |
| V.90 | 6312 | V.90 DATA at +25.545 s, downstream 45,333 / upstream 21,600. No LAPM/login; receiver reports white output (decision error around 0.61–0.67, bit statistics in the log), peer retrain at +40.104 s. Later V.34 21,600 also fails to sustain data. |
| V.90 | 6313 | V.90 DATA at +27.685 s, downstream 48,000 / upstream 21,600; LAPM + login. No logged resync before harness termination at +88.675 s: about 61 s in DATA. |
| V.34 | 6311 | `CONNECT 24000`, LAPM + login. DATA +14.491 s through termination +88.649 s, about 74 s, no logged decoding loss. |
| V.34 | 6312 | `CONNECT 24000`, LAPM + login, then receiver stops decoding and initiates V.34 §11.6 renegotiation twice. |
| V.34 | 6313 | `CONNECT 24000`, LAPM + login. DATA +17.398 s through termination +88.623 s, about 71 s, no logged decoding loss. |
| V.32bis | 6311 | V.8 selects V.32bis; 60 s TRAINING timeout, NO CARRIER. |
| V.32bis | 6312 | V.8 selects V.32bis; 60 s TRAINING timeout, NO CARRIER. |
| V.32bis | 6313 | Annex A USB1 → Tc → AC path enters V.32bis training; 60 s TRAINING timeout, NO CARRIER. |
| V.22bis | 6311 | `CONNECT 2400`, LAPM + login. Training complete +10.099 s, termination +88.606 s: about 79 s with no logged carrier loss after training. |
| V.22bis | 6312 | `CONNECT 2400`, LAPM + login. Training complete +8.179 s, termination +88.710 s: about 81 s with no logged carrier loss after training. |
| V.22bis | 6313 | Physical training complete at 2400, V.42 detection succeeds, but LAPM never connects; disconnect at +39.128 s with NO CARRIER and no banner. |

## Bearer checks and limits

All 12 calls use RTP payload type 0 (PCMU). Across 46,668 received and
47,579 transmitted packets, the local traces contain zero sequence gaps
and zero timestamp discontinuities. There are no logged slow media
callbacks at the default logging threshold. Maximum observed RX packet
spacing is 30.20 ms; maximum TX spacing is 49.98 ms on the failed V.90/6312
call. That spacing is an observation, not proof of causation. These local
traces do not show packets lost downstream of our transmit tap. No bulk
payload, BER or throughput test was performed.

The failed V.90/6312 call advertised LAPM in V.8, started V.42 detection,
then timed out to buffered V.14 and delivered unreadable bytes. This is
not merely a binary login banner: the receiver's own symbol measurements
also report white output. Its subsequent restart follows the existing
V.90 §9.5 / V.34 §11.5 recovery path. These measurements do not isolate
which component caused the initial bad decode. Check our own transmit tap
before attributing it to the RasFinder or gateway.

Evidence is retained locally under
`reference-data/artifacts/rf-modulation-20261020/`: per-call `run.json`,
server log, PTY bytes, raw G.711 taps, RTP traces and interaction metadata;
aggregate `results.json`; source manifest and patch; source archive;
tested binary and build log; runner, observer and analysis scripts.
The remote originals are under
`/root/rf-current-20261020/artifacts/rf-20261020/` in CT 117.
These are retained hardware evidence, not new regression-test fixtures.
