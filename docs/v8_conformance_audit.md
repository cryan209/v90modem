# V.8 conformance audit: our engine, the recordings, and the RasFinder (2026-09-29)

This audit checks our V.8 (`spandsp-master/src/v8.c` plus the engine glue)
clause by clause against ITU-T V.8 (11/2000), `ITU Docs/T-REC-V.8-200011-I!!PDF-E-1.pdf`.
It uses three sources of evidence: our own caller against our own answerer in
a closed loop, the recorded RasFinder taps, and the RasFinder findings already
logged in `docs/v90_rasfinder_phase4.md`. Every call below is measured, and
the tools are given with each result.

## Summary

1. **Te is not implemented. CM follows CI with zero silence on 110 of 112 live
   calls.** This is a clear violation of 8.1.1. It is not what separates
   passing calls from failing ones (see the table). Its stated purpose is
   echo-control disabling, and the path to the RasFinder has an echo
   canceller in the ATA. That makes it a live candidate for the "peer never
   hears our CM" failure. There is now a fix behind `ME_V8_TE_MS` (default
   off). It has not been tested live.
2. **Our CI's own echo, 12 dB below ANSam, stops our ANSam detector from ever
   firing.** On the closed loop the call fails outright: the caller never
   detects ANSam, and the answerer times out after 5.94 s. At 14 dB or more
   the call passes. With CI off, even a 6 dB echo passes. The RasFinder path
   has ~20 dB of margin, so this is not its failure. It does apply to any
   poorer hybrid, such as the HSF analogue line.
3. **Our own engine completes V.8 against itself** at every one-way delay
   tried (0 to 250 ms) and with near-end echo up to -14 dB, in both Te arms.
   Nothing in our V.8 fails against our own V.8 except case 2.
4. The RasFinder failure modes already on record are unchanged. The mode (i)
   ~2250 Hz tone is the peer's own V.22 fallback. 8.2.2 requires a peer that
   hears no CM within 5 +/- 1 s of ANSam to do exactly that. So in mode (i)
   the peer did not detect our CM. Our CM itself is conformant
   (independently demodulated) and bit-identical on passing and failing
   calls.

## Clause-by-clause

| Clause | Requirement | Ours | Verdict |
|---|---|---|---|
| 8.1.1 | 1 s of silence, then CI/CT/CNG | CI starts at 0.98 s on the TX tap | follows |
| 7.1 | CI ON >= 3 sequences and <= 2 s; OFF 0.4-2 s | ON 0.40 s (4 sequences), OFF 0.52 s | follows |
| 7.1 | CI = 10 ones, 10 sync bits, call function | `c1`, verified independently | follows |
| 8.1.1 | CI repeats until ANS/ANSam (no count limit) | 60 repeats (46544d05); was 10 | follows now |
| 8.1.1 | Stop the call signal once ANSam is detected (may finish 3 sequences) | the burst in progress finishes. Detection itself needs three >= 425 ms reversal cycles (~1.4 s), so up to one extra burst goes into ANSam | follows (the detector is slow on purpose: the one-cycle variant falsely fired on ringback decay) |
| 7.2 | "A call DCE shall not transmit a signal CM unless ANSam has been detected" | `handle_calling_modem_connect_tone()` also accepts **ANS and ANS/** (the comment says: packet networks strip the 15 Hz AM) | **deviates**, on purpose |
| 8.1.1 | ANS (not ANSam), so go to V.32bis Annex A / T.30, not V.8 | as above: treated as ANSam | **deviates** |
| **8.1.1** | **after ANSam, "transmit no signal for a period Te prior to transmitting signal CM"; Te begins when the call signal ends; >= 0.5 s, >= 1 s for V.25 echo-canceller disabling** | **Te = 0 whenever detection lands inside a CI burst: 110/112 live calls** | **violation, fixed behind `ME_V8_TE_MS`** |
| 7.3 | CM content and category rules (PCM means PSTN access present and modulation b5 set) | `c1 65 12 10 2a 47 8d`, demodulated independently | follows |
| 7.2/V.2 | level | CM at -13.8 dBm0 on the tap (`ME_V8_TX_POWER_DBM0` adjusts) | follows |
| 8.1.2 | CJ after >= 2 identical JM | yes. **Also** after a single saved JM candidate at the 5 s timeout (f73deaa1) | **deviates** (a fallback that only fires when V.8 would otherwise fail) |
| 8.1.2 | 75 +/- 5 ms of silence after CJ, then sigC | fixed earlier (see the `V8_CJ_ON` comment) | follows |
| 8.2 | answerer: >= 0.2 s of silence before answer tone | `ansam_start_delay_timer` | follows |
| 7.2 | ANSam 2100 Hz, 15 Hz AM, reversals every 450 ms | spandsp generator, `ANSAM_PR` | follows |
| 7.4/8.2.2 | JM only after >= 2 identical CM | yes. **Also** a single saved CM at timeout | **deviates** (same fallback) |
| 8.2.2 | ANSam ends once CM is received | ANSam is **held** after CM is recognised, to fit SmartLink's 4.5 s echo-gate window (comment at the `V8_CM_WAIT` got_cm_jm branch) | **deviates** (peer workaround) |
| 8.2.3 | JM continues until all 3 octets of CJ are received | yes, plus "carrier drop after CM/JM = CJ" | permitted ("other criteria may be used") |

## Finding 1: Te was never implemented

`V8_HEARD_ANSAM` sets a 1 s timer and says it waits "to comply with the spec".
Until the timer expires, though, the case **falls through into `V8_CM_ON`**.
There, `queue_contents(s->tx_queue) < 10` → `send_cm_jm()` refills the FSK
queue while the current CI burst is still draining. So the modulator never
goes idle, and CM's 90-bit period starts in the same 100 ms block in which CI
ends. This was measured on the TX tap of `rf-v8fix-200101Z/trn12000-r1`
(passed) and `control-r1` (failed), byte-identical from 0 to 9 s: CI
5.5-6.0 s, silence, CI again at 6.5 s, then CM runs straight on from 6.9 s.

Across the corpus (`tools/v8_timeline_audit.py`, 185 RasFinder call
directories, 112 with both an ANSam and a CM on the taps):

| V.8 status | Te = 0 | Te >= 1 s |
|---|---|---|
| 2 (passed) | 53 | 1 |
| 4 (failed) | 57 | 1 |

(The audit prints "te 0.52" for these calls. That figure is the CI gap
*before* the CI burst that runs into CM; the true Te is zero.)

So Te = 0 **does not separate** pass from fail, because nearly every call has
it. Two things still make it the first thing to test live:

- Figure 1/V.8 defines Te as "the silent period allowed for disabling of
  network echo-control equipment". The failing mode is exactly the one where
  the peer never hears our CM. On record for that mode: CM bit-identical,
  same level, louder CM no help, and ANSam running its full 5.4 s. The path
  toward the peer runs through a VG224 ATA whose voice-mode EC/NLP state was
  already the leading suspect.
- A continuous V.21 stream with no silence gives the peer's CM receiver no
  fresh carrier onset. The 10-ones preamble is still in every CM, but a
  receiver that arms on energy rising after ANSam never sees that rise.

Fix: `ME_V8_TE_MS=<ms>` holds the `V8_HEARD_ANSAM` state until CI has
finished (`fsk_tx_on` false), then waits Te of silence before the first CM.
While it waits it keeps the V.21 receiver running and does not queue CM.

Offline check, with `v90_engine_replay --dial` on
`rf-v8fix-200101Z/trn12000-r1` and the start shifted with `--from` so that
detection lands inside a CI burst:

| `--from` | legacy Te | `ME_V8_TE_MS=1000` | V.8 |
|---|---|---|---|
| 2.1-2.4 s (detection in a CI gap) | 1.1-1.4 s | same | 2 / 2 |
| 2.6-2.8 s (detection in a CI burst) | **0** | **1.00 s** | 2 / 2 |

Our V.8 still completes with the conformant Te against the recorded peer.
That replay is open-loop, though: a recording's JM does not wait for our CM.
**Default stays off** until a live A/B on the RasFinder, with arms alternated,
settles it. That follows the project rule not to move a default that no live
call has exercised.

## Finding 2: our own CI echo blinds our ANSam detector

`tools/v8_loop_test.c` is a closed-loop harness: our `v8_init()` caller against
our answerer, each through a one-way delay, plus near-end echo of each side's
own transmit. With
100 ms of delay and a 30 ms echo:

| near-end echo | caller side | answerer side |
|---|---|---|
| -18 / -16 / -14 dB | OK 6.40 s | OK 6.40 s |
| **-12 dB, at the caller** | **caller never detects ANSam; answerer times out at 5.94 s** | |
| -12 dB, at the answerer | | OK |
| -12 or -6 dB at the caller, **CI disabled** | OK | OK |

`modem_connect_tones_rx()` needs three clean 450 ms ANSam cycles. CI bursts
take 0.4 s of every 0.92 s, and their echo lands on each cycle. The RasFinder
path measures ~20 dB return loss, with ANSam arriving at -18.1 to -18.7 dBFS
and our CI leaving at about -17 dBFS. That puts the CI echo ~18 dB under
ANSam, 4-6 dB clear of the threshold, so this finding does not explain the
RasFinder failures. It is still a real fragility of our caller on any 2-wire
analogue path (HSF), and it is one more reason the detector is slow (CI lands
inside ANSam).

The obvious remedies all conflict with evidence already on record:
- stop CI earlier: `ME_V8_NO_CI` measured 0/4 against 0/4 on the RasFinder
- detect after one cycle: this falsely fired on ringback decay

So this finding is recorded, not changed.

## Finding 3: our engine against itself

At 0, 20, 100 and 250 ms one-way delay, with no echo or -20 dB echo, both
roles complete in 6.2-6.7 s and both Te arms give identical results. The
~6.3 s is mostly the answerer's deliberate ANSam hold (the SmartLink
workaround in the table). The only self-failure found is Finding 2.

An open-loop trap to avoid: feeding our caller's *replayed* TX tap into our
answerer (`v90_engine_replay <tx tap>`) shows the answerer decoding five CMs
and never accepting them in the legacy arm. That is not a defect. The legacy
CM burst lands before the replayed answerer is listening, and in a replay
the answerer's ANSam never drives the caller. Use the closed loop, not a tap
fed back in.

## What could be the cause on the RasFinder, ranked

1. **The path toward the peer loses our CM.** This is still the strongest
   reading. ANSam runs its full 5 +/- 1 s and then the peer falls back to
   V.22 per 8.2.2, while our TX tap is byte-identical on pass and fail
   calls. It is all-or-nothing per call. That points at per-call state
   downstream of us: ATA passthrough/VBD switching, EC/NLP, adaptive
   playout.
2. **Te = 0 interacting with that echo control.** Te is the one timing
   requirement we break on every call, and it exists for exactly this
   equipment. It is cheap to test: `ME_V8_TE_MS=1000`, arms alternated,
   scored with `tools/rf_ansam_latency.py` (ANSam > 5.0 s = CM unheard) and
   the V.8 status.
3. The ANS-as-ANSam, single-candidate and held-ANSam deviations. None of
   them acts on a call where the peer never hears CM, so none can cause
   failure mode (i). The single-candidate JM fallback is what rescues
   mode (ii).
4. Our own receive path. The jitter-buffer insertion defect (74563a6d) was
   ours and is fixed. Finding 2 is ours and has margin on this rig.

## Tools and how to reproduce

- `tools/v8_timeline_audit.py <call dir>...` needs numpy
  (`python3 -m venv v; v/bin/pip install numpy`). It reports ANSam onset,
  CI transmitted inside ANSam, Te, CM start and CM level per call.
- Replay with Te forced: `ME_V8_TE_MS=1000 VPCM_G711_TAP_DIR=<dir> ./v90_engine_replay <rx tap> ulaw --fast --dial --from 2.7`
- Closed loop: `tools/v8_loop_test.c` (the build line is in its header). It
  reads the environment, so `ME_V8_TE_MS` applies. spandsp's own
  `tests/v8_tests` needs libsndfile, which this machine does not have.

## 2026-09-30: the failures track network jitter on this Mac's Wi-Fi, and Te does not help

**Live Te A/B, arms alternated, 90 s between calls:** `ME_V8_TE_MS=1000` 0/6,
control 0/6 (`artifacts/rf-te-ab-{te,ctl}-{1..6}`); a default probe call the
same morning also failed (`rf-0930-probe-1`). Every failure is the known shape:
the peer's ANSam runs its full ~5.4 s and 2250 Hz (V.22 USB1) follows, so the
peer never recognised a CM.

**What does separate the good and bad periods is RTP jitter.** Over every
RasFinder call in `artifacts/` (first ~12 s, i.e. the V.8 window, from
`rtp-rx.csv`, transit-time spread):

| period | V.8 pass | RX transit spread p95 |
|---|---|---|
| 09-28 all day | 29/34 | 0.1-12 ms (one outlier 68) |
| 09-29 from ~11:00 | falling to 0/22 by 22:00 | 60-155 ms on most calls |
| 09-30 morning | 0/13 | still 60-120 ms |

The ATA's own RTCP receiver reports (pjsua's second `jitter` line, i.e. what
the far end measured on OUR stream) agree: 3.5 ms avg / 13.8 ms max on the
healthy 09-28 `rf-v34-s3`, 14-16 ms avg on the 09-30 calls. Our send pacing
is unchanged throughout (`rtp-tx.csv` wall deltas: sd 0.5-1.1 ms on every
short call), so the jitter is added after our process.

**It is the first Wi-Fi hop.** This machine reaches Asterisk over `en0`
(Wi-Fi; the Ethernet adapters en3/en4 have no link) and AWDL is up. Pinging
the LAN gateway 10.69.70.1 at 10 Hz: min 2.4-2.7 ms, avg 12-37 ms,
**max 92-173 ms** -- periodic stalls of the kind AWDL channel hopping
produces. A ~100 ms stall in our stream underruns the VG224's voice-mode
playout buffer, which conceals it, and a 300 bit/s V.21 CM with 100 ms of
concealment in it is lost; V.8 needs two identical consecutive CMs. That also
explains why the peer's CM-detection latency grew on the calls that did pass
on 09-29 (ANSam 3-5 s instead of the healthy 2.2 s): it was missing CMs
intermittently.

**Two things this rules in and out.**
- Every V.8 knob tried since 09-29 (louder CM, no CI, one-cycle ANSam
  detection, CM restart, Te, the ATA EC/NLP/CN/attenuation changes) was
  measured on this impaired path, so those NEGATIVE results are not evidence
  about V.8. Re-run the ones worth having on a clean path.
- The ANSam-onset-vs-CI-phase correlation (failures skew to ANSam arriving
  >= 0.77 s after a CI burst start) is confounded by date and is most likely
  the 200 ms fixed RX jitter buffer landing (74563a6d); within 09-29 17-18h
  the two phases pass alike (6/11 vs 9/17).

**Next:** put this host on wired Ethernet (or `sudo ifconfig awdl0 down`,
which is a user action -- it is a system network setting), confirm gateway
ping max < ~15 ms and `rtp-rx.csv` p95 back under ~12 ms, then place calls.
Measurement: transit `(arrival_ms - t0) - (rtp_ts/8 - ts0)` over the first
600 packets of the MOST COMMON SSRC in `rtp-rx.csv`, less its 5th
percentile. **Not the first SSRC** -- that is ~8 packets of early media before
the answered stream starts a new one, and a first pass at this measured
those. Shape on a 09-30 call: flat, then a 60-150 ms jump draining in 40 ms
steps (`137 97 57 17`) several times a second -- packets held and released in
a burst, i.e. a stalling link; the 09-28 calls sit flat at 1-12 ms.

## 2026-09-30 (later): confirmed -- off Wi-Fi, V.8 passes 6/6

The same commit, unchanged defaults, run from **tower** (wired, 0.08-0.25 ms to
Asterisk, which it hosts) instead of this Mac's Wi-Fi: **V.8 status=2 on 6 of
6 valid calls** (`artifacts/rf-tower-{1,3,4,5,6,7}`), against 0/13 from the Mac
that morning. RX transit p95 0.1 ms. The peer ends ANSam **2.1-2.2 s** after
it starts on every call -- the healthy signature of catching our first CMs --
and every call negotiates V.90 (peer offers V.22bis, V.34, V.90, LAPM).
(`rf-tower-2` is not a V.8 attempt: it dialled seconds after call 1 and was
rejected outright, the known close-redial behaviour.) Past V.8 the calls meet
the separately tracked V.90 Phase 3/4 retrain and fall back to V.34; that is
not V.8.

So no V.8 code change was needed; the fix is the path. **Run RasFinder calls
from a wired host.** Recipe on tower (Unraid, no compiler on the host):

    docker run -d --name v90modem-sip --network host debian:bookworm sleep infinity
    docker exec v90modem-sip bash -c 'apt-get update && apt-get install -y build-essential autoconf automake libtool pkg-config libtiff-dev libjpeg-dev libssl-dev libasound2-dev libavformat-dev libavcodec-dev libswscale-dev libavutil-dev libv4l-dev libopus-dev uuid-dev procps python3'
    git archive --prefix=v90modem/ HEAD | ssh tower.net.cryan.nz 'docker exec -i v90modem-sip tar -x -C /root'
    # the tiff-fx Makefile.in stub is untracked -- copy it too
    tar -c spandsp-master/test-data/itu/tiff-fx/Makefile.in | ssh tower.net.cryan.nz 'docker exec -i v90modem-sip tar -x -C /root/v90modem'
    # build the vendored libraries SERIALLY first (spandsp.h is generated), then the server;
    # the Linux local-pjproject link needs -luuid, which the makefile does not add:
    make spandsp && make pjproject && make -j16 sip_v90_modem   # link fails on uuid_*
    eval "$(make -n sip_v90_modem | grep -E '^(gcc|cc) .* -o sip_v90_modem') -luuid"
    tools/soak/rasfinder_call.sh artifacts/rf-tower-N 60      # inside the container

Leave 90 s between calls, including before the first call of a batch.
