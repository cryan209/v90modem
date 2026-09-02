# The HSF USB modem as v90modem's analogue side

`hsf_v90_coupler` is `hsf_fxo_probe.c` compiled with `HSF_V90_COUPLER`, which
adds the modem engine.  The HSF DAA seizes a real analogue line, dials the
digital side's extension through the PBX, and the engine then runs as the
ANALOGUE modem over that line while `sip_v90_modem` answers on SIP as the
digital modem.  It is the first path in this project where the two roles face
each other across a genuine 2-wire analogue bearer rather than a byte-exact
G.711 one, and most of what follows is a consequence of that single difference.

    ./sip_v90_modem --sip-server asterisk.net.cryan.nz --username 6001 \
        --password 6001 --pty-link /tmp/v90server --verbose

    MODE=v22 SECS=90 SETTLE=40 RUN=artifacts/hsf-v90/callN \
        ./tools/hsf_v34_call.sh

`tools/hsf_v34_call.sh` places one call and grades DTE traffic across both
PTYs with `tools/hsf_pty_traffic.py`.  `MODE` is `ME_MODE`; `SETTLE` is the
idle time before dialling, which the ATA needs -- dial too soon after the
previous call and the exchange answers with number-unobtainable instead of
ringing, which reads exactly like a rig that has stopped working.

## What works

* **V.8 completes over the analogue line, both ways.**
* **V.90 is negotiated with the HSF as the analogue modem** (`ME_V90_ROLE=
  analogue`), Phase 2 completes bidirectionally -- INFO0a/INFO0d, the tone
  choreography, INFO1d and INFO1a -- and both sides enter Phase 3.
* **V.22bis reaches data mode and carries DTE traffic.**  Three consecutive
  calls connected at 2400 bit/s; the best delivered **756 intact numbered
  lines, 750 of them in sequence**, from the digital side to the HSF's DTE.

## What does not, and how far each gets

* **V.90 Phase 3**: the digital modem transmits Sd (its `raw_v90_tx` counter
  moves, and the recording carries the 6-symbol Sd signature -- 45-50% of the
  band energy at 1333 Hz, which is 8000/6).  The analogue-role receiver never
  finds it.  That receiver is a *codeword* state machine validated only
  against `artifacts/eicon-digital-downstream/`, a byte-exact G.711 capture;
  over a line with unknown gain, band-limiting and delay there are no exact
  codewords to slice.
* **Plain V.34**: Phase 2 completes, and Phase 3/Phase 4 reach as far as
  PHASE4_MP on the best live call, but usually the receiver sits in
  PHASE3_WAIT_S for the whole call.  See the sampling-phase finding below.

## Three defects found here, all ours

**1. The HSF receive stream carries a DC offset, and it blocks V.8 outright.**
Measured at 908 counts against a 5000-count ANSam.  SpanDSP's ANS/ANSam
detector rejects the tone completely on that: the recorded audio as-is is
never recognised in 70 s, and the same audio with the offset removed is
recognised as `ANSam/` in 1.4 s.  Isolated outside the engine with a
`modem_connect_tones_rx` harness, which is what separated "the line is bad"
from "our front end is".  The coupler now high-passes at 40 Hz (-0.08 dB at
300 Hz) before the samples enter the modem.  Before this fix no call got past
V.8 at all; after it, V.90 Phase 2 completed on the first attempt.

**2. Starting the engine on a post-dial timer ran V.8 into ringback.**  The
first version waited a fixed 6500 ms and then began V.8 CI; the PBX was still
ringing, and V.8's 10 s timeout expired within a second of the far end
answering.  Ringback here is 400/450 Hz and the answering modem's ANSam is
2100 Hz, so a Goertzel separates them: the engine now starts on 200 ms of
detected 2100 Hz, with the old timer demoted to a backstop.

**3. V.22bis reported CONNECT on carrier-up and hung up on carrier-down.**
`v22bis_put_bit_cb()` treated `SIG_STATUS_CARRIER_UP` as a completed
connection and `SIG_STATUS_CARRIER_DOWN` as a failure.  Over a digital bearer
that is survivable; over this line both ends printed "training complete" 8 ms
after V.8 and dropped 60 ms later on the carrier-down that follows the V.8
tail -- a CONNECT to the DTE with a modem that had never trained.  Only
`SIG_STATUS_TRAINING_SUCCEEDED` is a connection now, and a carrier drop before
training is waited through (the training timeout still bounds it).  The
successful calls' logs show exactly the sequence the old code died on:
`carrier up` -> `carrier down before training` -> `carrier up` -> trained.

## The V.34 finding: receive sampling phase, not clock rate

Plain V.34 sits in `PHASE3_WAIT_S` and never sees the far end's S.  Swept as a
pure fractional delay over one 16 kHz sample on a recorded call, with a no-op
resample as the control and a low-pass-only arm to separate filtering from
phase:

| arm | result |
| --- | --- |
| no-op resample (bit-identical) | PHASE3_WAIT_S for the whole call |
| low-pass only, no shift | PHASE3_WAIT_S |
| fractional delay 0.625-0.875 | PHASE4_MP |

So it is the sampling instant.  Replayed across three recorded calls the
fraction of phases that reach Phase 4 is 1/10, 2/10 and 9/10 -- the
sensitivity is real, its severity is per call, and live it is a lottery.  The
coupler's `HSF_RX_DELAY` sets that phase; 0.8 is the default because it is the
only value that reaches Phase 4 on more than one recording, and 0.0 -- the
obvious choice, and what a plain 2:1 average gives -- fails on all three.

**A 297 ppm clock error was measured first and is WRONG.**  The fit straddled
one of ANSam's 450 ms phase reversals.  Fitted on a reversal-free stretch the
HSF codec reads **16000.4 Hz, +26 ppm**, on three independent recordings, and
correcting a real 26 ppm and a bogus 297 ppm in the opposite direction both
"fixed" the call -- which is what exposed the resampling, not the rate, as the
active ingredient.  Take the lowest-residual window, and keep the control arm.

**10.1.3.7's S is detectable at any phase; only this detector is not.**  At a
failing phase the three-bin measurement (fc, fc +/- baud/2) that §9.6's
renegotiation watch already uses reads 0.933 with a 50 ms run -- S plainly
present.  `V34_PHASE3_S_SPECTRAL=1` wires that into `PHASE3_WAIT_S` and it
fires, publishing `V34_EVENT_S` three times on a call that acquires nothing.
**It does not rescue the call**: PP conditioning after S is still
constellation-domain and still needs the eye.  Kept, default off, because it
is the measurement that says where the remaining work is -- the eye-phase
chooser runs during PP, i.e. after a gate that cannot be passed without it.

`ME_V34_PHASE3_S_TIMEOUT_MS` bounds the wait so a stuck attempt restarts Phase
2 rather than burning the call; default 0 (off).  Measured live it restarts
cleanly once and then the peer, already in its own data mode, does not follow,
so it is not on by default either.

## Instruments added

* `--rx-replay FILE [--replay-from S] [--replay-secs N]` runs the whole
  analogue side offline against a recorded receive tap, at the same block size
  and through the same decimation as the live path.  Live failures reproduce
  in it exactly, which is how the sampling-phase sweep was run at all -- four
  hypotheses in an afternoon instead of four calls each.  The transmit is
  discarded, so it cannot prove anything the far end must respond to.
* `--pty-link PATH` gives the coupler a DTE.
* `HSF_RX_DELAY`, `HSF_RX_DELAY_STEP`/`_PERIOD_MS`, `HSF_TX_GAIN`.

## Open

* The V.90 analogue Phase 3 receiver needs to recover codewords from an
  analogue waveform rather than slice exact ones.  That is the whole of the
  remaining V.90 work on this path.
* V.34 Phase 3 acquisition needs to work at an arbitrary sampling phase.
* Both V.22bis directions are lossy and the HSF-to-digital direction delivers
  no intact lines at all (bytes arrive, so bits flow and are corrupted).  A
  2.5x transmit gain made the downstream *worse* (21 lines against 176 at the
  same offered load), so level is not the lever; the 2-wire hybrid's echo of
  each end's own transmit is the untested suspect.

## Build note

`hsf_v90_coupler.c` is a two-line wrapper that `#include`s
`hsf_fxo_probe.c`, and this makefile has no header dependencies, so an edit to
the probe used to leave the coupler linked against the previous build --
which cost this session one live call whose log showed code that was not in
the binary.  `makefile` now names the dependency explicitly.
