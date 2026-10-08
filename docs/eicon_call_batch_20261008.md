# Eicon hardware call batch, 8 October 2026

Eight sequential calls alternated forced V.34 and V.90 across PCMU endpoint
7910 and PCMA endpoint 7900. The modem's rebuilt native-transmit corrections
were present throughout. No peer or gateway configuration changed. The harness
waited at least 65 seconds between calls to the same controller and preserved
byte-exact TX/RX G.711, RTP arrival records, engine I/O schedules, PTY bytes,
and logs in `artifacts/eicon-batch-20261008-r1/`.

| Call | Mode/law | Observed result |
|---|---|---|
| 01 | V.34 / PCMU | INFO1a, Phase 4 MP, three training-failure retries; no menu |
| 02 | V.90 / PCMA | 56000/31200, LAPM and menu; first 201-byte echo fails |
| 03 | V.90 / PCMU | 56000/31200, three exact 201-byte echoes; fourth fails |
| 04 | V.34 / PCMA | Phase 4 failures; local PTY EAGAIN invalidates harness outcome |
| 05 | V.34 / PCMU | INFO1a, Phase 4 MP failure; retry offers all-zero INFO1c |
| 06 | V.90 / PCMA | 56000/31200, three exact 201-byte echoes; fourth fails |
| 07 | V.90 / PCMU | 56000/31200, LAPM and menu; first echo fails |
| 08 | V.34 / PCMA | INFO1c, no INFO1a before disconnect |

A replacement for call 04, `artifacts/eicon-batch-20261008-recheck/`,
reaches INFO1a/Phase 4, fails training three times, then sends an all-zero
INFO1c. It delivers no menu. The harness now tolerates a nonblocking read
race and scopes raw PCM dump files per call; earlier shared `/tmp` raw dumps
were not used for this analysis.

## Native V.34: peer never acknowledges our MP

Recorded-schedule replay of call 01 reproduces its first failure within
14 ms of the live trace. Per-symbol logging was enabled only offline:
`/tmp/eicon-replay-01.log`. The failure is specifically **no E within 3250 ms
of the first decoded MP**, rather than inability to decode MP. The card sends
CRC-valid Type-1 MP with acknowledgement zero, requests 4800 bit/s in our
transmit direction, and continues unacknowledged MP after we switch to MP'.

An independent waveform decoder (`tools/v34_mp_tap_check.py`) applied to
call 01's actual TX DS0 at file seconds 12–18 recovers **247 CRC-valid
Type-0 MP frames: 13 MP and 234 MP'**. It uses blind equalization,
V.34 10.1.3.3 differential decoding, caller GPC (7.1), and the
10.1.2.3.2/Table-13 CRC excluding sync/start/fill bits. Its result is saved as
`01-v34-ulaw/independent-tx-mp.json`.

This rules out malformed locally emitted MP bits on this captured attempt.
It does not show what the card receives after the bearer, nor establish that
it accepts every parameter. V.34 11.4.1.1.2/.3 requires the MP -> MP' -> E
dialogue; our observed MP' transmission alone does not complete it.

Retries introduce a second failure: calls 01, 04, 05 and the replacement
eventually advertise zero projected rate at every INFO1c row. Table 15
defines zero as an unusable symbol rate. This removes every usable offer and
helps explain a subsequent INFO1a stall, but is not the original MP failure.
The underlying probe/retrain state remains to be diagnosed.

## V.90: startup succeeds, payload fails in both laws

All four baseline V.90 calls complete B1d with 48/48 correct frames, enter
56000 downstream / 31200 upstream, establish LAPM and receive the menu.
Two calls carry three exact echo lines each (1206 checked payload bytes
total); two fail their first line. All subsequently receive sustained peer
Tone B and enter the V.90 9.5.2 retrain path. Thus startup/B1d passing does
not establish sustained payload correctness, and the failure is not isolated
to one G.711 law. Correct initial echoes also refute an always-broken mapper.

The four V.90 calls have no local RX or TX RTP sequence/timestamp gaps;
RX arrival intervals peak at 95–105 ms, TX at 22–24 ms. RX clock-recovery
sample slips are disabled. Aggregate remote RTCP loss reports cannot locate
loss in time or establish the reason for the stall. These records do not
exclude jitter-buffer insertion or downstream bearer distortion.

Call 01 V.34 has two RX packet gaps and a 151 ms arrival interval, but
call 05 reaches the same MP failure with no local RX gaps. Local packet
loss is therefore not a sufficient explanation for the repeated MP failure.

Prior card-side evidence in `docs/eicon_live_20261008.md` strengthens the
upstream suspicion: direct fixed-playout captures reported Eicon RX CRC
errors while card receive samples matched our local TX at 99.9–99.95% and
a constant offset. Those are separate earlier trials, not card observations
from this batch. They prevent treating Asterisk or adaptive playout as a
complete explanation; sustained upstream encoding/interoperability remains
open even after the native B1/nonlinear corrections.

## Lower-rate diagnostic

Two further calls used the existing
`ME_V90_ANALOGUE_UPSTREAM_MAX_N=9` diagnostic ceiling (21600 bit/s).
PCMA trained at 56000/21600, PCMU at 50666/21600. **Each carried one exact
201-byte echo, failed its second, and received peer Tone B retrain.** The
PCMA call had no local RTP sequence/timestamp gaps in either direction;
the PCMU call had one RX gap and no TX gaps. Evidence is in
`artifacts/eicon-batch-20261008-cap21600-a/` and `...-cap21600-u/`.
Reducing the upstream ceiling does not remove the failure on these trials.
The total is eleven placed calls, including the invalid PTY trial and its
replacement. No sustained ten-line echo test passed.

## Reproduce

```
python3 tools/eicon_call_batch.py artifacts/eicon-new-batch --repeats 2 --lines 10
.venv/bin/python tools/v34_mp_tap_check.py \
  artifacts/eicon-batch-20261008-r1/01-v34-ulaw/live-tx.g711 12 18
```

The MP checker deliberately supports caller/PCMU/4-point MP only. Every
window is indexed against its own file origin; TX and RX origins are not
interchangeable. Logs, not cross-file sample offsets, establish the live
stage timing quoted above.
