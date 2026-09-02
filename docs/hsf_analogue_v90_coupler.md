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

## Symbol timing: non-data-aided recovery is impossible on this bearer

The obvious next step after the level slicer (`v90_analogue_linear.c`) was a
symbol timing loop, so the caller's sampling phase would stop mattering and
`HSF_RX_DELAY` could go.  It was built -- a cubic/windowed-sinc interpolator,
a maximum-mean-square phase acquisition, an early-late energy tracker and a
Mueller and Muller tracker -- and then **withdrawn, because the signal carries
no timing information to recover**.

The reason is that the V.90 downstream has **zero excess bandwidth**.  Its
symbol rate is 8 kHz and the line passes at most ~3.6 kHz, so there is no pair
of frequencies `f` and `f - 8000` both inside the channel, and therefore no
symbol-rate spectral line for any nonlinearity to lock to.  Equivalently: the
received process is stationary, not cyclostationary, so its statistics do not
depend on where in the symbol it is sampled -- and every non-data-aided timing
detector is some estimate of that dependence.

Measured, on the Eicon fixture through a zero-order hold and a 3600 Hz
low-pass, mean square against sampling instant swept across a whole symbol with
a 64-tap windowed-sinc interpolator:

| instant | 0.000 | 0.250 | 0.500 | 0.750 |
|---|---|---|---|---|
| mean square | 5.026e5 | 5.026e5 | 5.024e5 | 5.026e5 |

**0.04% across the entire symbol** -- flat, and far below the interpolator's own
passband loss, which is what every phase estimate built on it actually locked
to (all of them settled at the caller's own instant whatever phase the channel
was given).  Widening the model channel to 4400 Hz brings the variation up to
1.7%, which is the control: the metric works when there is excess bandwidth and
this bearer has none.

That is not an obscure case -- it is the case Mueller and Muller was written
for.  Baud-rate timing recovery with no excess bandwidth has to be
**decision-directed**, and a decision here means a G.711 level recovered
through the channel, which needs the equaliser.  Hard-limited M&M was tried to
avoid that (the full ladder is so dense that a correct decision is within a per
cent of the sample it came from, and the detector cancels itself); it has a
correct S-curve but its self-noise wandered 2% of a symbol on a byte-exact
fixture, where the right answer is to sit still.

So the ordering is settled, and it is the opposite of the one attempted:

1. A **fractionally-spaced equaliser at T/2**, fed the HSF's own 16 kHz stream
   before the decimation rather than the engine's 8 kHz.  An FSE absorbs the
   sampling phase by construction -- that is why the V.34 receiver in this tree
   uses one -- so "works at any sampling phase" falls out of it rather than
   needing a loop of its own.  §8.4.5's TRN1d is 30000T of constant-modulus
   scrambled ones, which is 3.75 s of ideal CMA training material.
2. Then decision-directed M&M on the equalised symbols, for the sample-rate
   drift only.

That also settles the level question the slicer left open, since an equaliser
is required anyway: at 3600 Hz the pulse's first neighbours carry **±0.14** of
the main tap's 0.80, and the G.711 ladder's steps near the top are finer than
that, so no slicer can recover the levels however well the instant is chosen.

## The channel, measured

The far end transmits V.34's line probing signal during Phase 2 -- 21 cosines
of **equal amplitude** from 150 Hz to 3750 Hz (V.34 11.2.3; the table is in
`spandsp-master/src/make_v34_probe_signals.c`) -- so every recorded call that
reached Phase 2 contains a complete channel sounding, and equal transmit
amplitudes are what make it one.  `tools/hsf_probe_response.py` finds it
without needing the call's log or its clock (nothing else on this line has
substantial energy at both 150 Hz and 3750 Hz) and reports magnitude and phase.

Measured on `artifacts/hsf-v90/call-c1`, and **identical to 0.1 dB in all nine
recorded calls that got far enough to carry a probe**:

| Hz | dB | phase | Hz | dB | phase |
|---|---|---|---|---|---|
| 150 | -0.4 | -1° | 2250 | -0.7 | +24° |
| 300 | -0.1 | -19° | 2550 | -0.9 | +27° |
| 450 | -0.1 | -20° | 2700 | -1.2 | +27° |
| 600 | 0.0 | -18° | 2850 | -1.3 | +25° |
| 750 | -0.0 | -15° | 3000 | -1.4 | +21° |
| 1050 | -0.2 | -6° | 3150 | -1.7 | +15° |
| 1350 | -0.4 | +3° | 3300 | -1.8 | +4° |
| 1500 | -0.4 | +7° | 3450 | -2.5 | -14° |
| 1650 | -0.4 | +12° | 3600 | -4.4 | -40° |
| 1950 | -0.5 | +19° | 3750 | -8.4 | -72° |

Empty-bin noise floor -50 dB; bulk delay 0.90 ms removed from the phases, so
what is left is group-delay distortion.

**This is much kinder than the brickwall the equaliser was tuned against, and
it changes the conclusion.**  There is usable energy right up against the 4 kHz
Nyquist of the 8 kHz symbol rate -- -8.4 dB at 3750 -- where the guessed
3600 Hz low-pass had none at all.  So the "zero excess bandwidth" reading that
ruled out non-data-aided timing recovery is still true in kind (the last
250 Hz is unmeasured, since the probe stops there) but nothing like as
absolute; and the intersymbol interference is far milder than assumed.  The
measurement is now the test's channel: `hsf_measured_response[]` in
`v90_analogue_rx_test.c`, with TRN1d's converged dispersion falling from 0.033
on the guess to 0.0013-0.020 on the real thing.

## A sounder of our own, for above 3750 Hz

V.34's probe stops at 3750, and the last 250 Hz below the 4 kHz Nyquist of the
8 kHz DS0 symbol rate is what decides whether a symbol-rate timing tone exists
at all -- and how much intersymbol interference the analogue receiver has to
undo.  Our own transmit is not constrained to V.34's 150 Hz grid, so
`v90_sounder.c` puts 25 tones on a 50 Hz grid: V.34's own set to 3750 (so a
sounding overlaps the probe and the two compare directly) plus 3800, 3850,
3900 and 3950.  The block is 160 samples at 8 kHz, so every tone is an exact
multiple of the block rate and nothing leaks between bins; V.34's four
deliberate gaps (900, 1200, 1800, 2400) stay empty as the noise reference.

**Running it needs one call.**  On the end that should transmit -- the digital
side for the downstream, the HSF side for the upstream:

```bash
ME_SOUNDER=1 ME_G711_CAPTURE=/tmp/sound ./sip_v90_modem --sip-server ... 
```

`ME_SOUNDER` replaces that side's transmit with the sounder for the whole call
(no handshake happens), and `ME_G711_CAPTURE` records the exact codewords it
sent.  Then, against the coupler's `hsf-rx.raw`:

```bash
python3 tools/hsf_probe_response.py hsf-rx.raw --sounder --reference /tmp/sound.tx.ulaw
```

With `--reference` the result is a true two-port response, RX/TX: the transmit
path's own µ-law quantisation and whatever level the generator chose are in the
reference rather than in the answer.

The generator is checked in `make test` before it costs a call, because a
transmit-only signal cannot be verified after the fact -- a rig session that
comes back with a strange response could not tell whether the line did that or
the generator did.  The check runs it through the µ-law quantiser it will go
out through and asserts every tone is there (the weakest sits 37.7 dB over the
empty bins) and that the peak keeps 10 dB of headroom (Schroeder phases give a
crest factor of 2.74).  Clipping is the failure that matters: it would put
energy straight into the empty bins, the floor would read high, and every weak
tone above 3600 Hz would be dismissed as noise -- which looks exactly like the
answer the sounder exists to find.

## The equaliser, and what it now recovers

`v90_analogue_fse.c` is that equaliser: 32 T/2 taps, NLMS, CMA on §8.4.5's
TRN1d, handed over to a decision-directed loop as soon as the ladder is
calibrated.  Fed from the coupler's 16 kHz stream through `me_rx_v90a_16k()`,
before the decimation.  Measured over a zero-order-hold channel band-limited to
3840 Hz, with the sampling phase swept across a whole symbol:

| what | result |
|---|---|
| §8.4.5 TRN1d signs | 100% at every phase |
| §8.6 constellation sized for this line | 89-92% of codewords exact (20-78% with frozen taps) |
| §9.3.2.9 DIL verdict | 90-93 of a clean bearer's 121 usable Ucodes, **0 wrongly claimed** |

The DIL row is the one that matters and it took the right question to see it.
Scored on exact codeword recovery the DIL reads 25%, which looks like a
failure and is not: it probes the whole ladder deliberately, most of a real
ladder is not separable over a real line, and finding that out is its purpose.
Scored on its VERDICT -- which Ucodes arrived distinguishable from their
neighbours -- it recovers three quarters of them and, in the direction that
matters, claims none it should not.  Missing a usable Ucode costs a little
rate; calling an unusable one usable puts it in CP and the digital modem then
transmits a constellation the line cannot carry for the rest of the call.

## Open

* None of this has a live call behind it.
* What the path does above 3750 Hz is still unmeasured.  The sounder to
  settle it is built and self-checked (below); it needs one rig call.
* Nothing tracks a sample-rate offset between the two ends.  At T/2 that shows
  up as the tap set walking off its span, not as a phase error.
* §8.4.4's Sd arrives before TRN1d has trained anything, so the first 48 ms of
  Phase 3 go through an unconverged equaliser and §9.3.2.4's Sd-to-S̄d
  transition is the ordering problem left.
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
