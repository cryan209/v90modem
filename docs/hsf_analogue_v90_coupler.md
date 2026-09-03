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
ME_SOUNDER=1 VPCM_G711_TAP_DIR=<run> ./sip_v90_modem --sip-server ...
```

`ME_SOUNDER` replaces that side's transmit with the sounder for the whole call
(no handshake happens, so the engine's own V.8 timeout ends the call after
about twelve seconds -- which is ample).  The transmit tap is
`VPCM_G711_TAP_DIR`, which writes `<run>/live-tx.g711`; `ME_G711_CAPTURE`
records the RECEIVE side only and is not the reference.  Then, against the
coupler's `hsf-rx.raw`:

```bash
python3 tools/hsf_probe_response.py hsf-rx.raw --sounder --reference <run>/live-tx.g711
```

With `--reference` the result is a true two-port response, RX/TX: the transmit
path's own µ-law quantisation and whatever level the generator chose are in the
reference rather than in the answer.

**The PBX could not reach the ATA, and that is fixed (2026-09-03).**  Asterisk
IS 192.168.88.122 -- the "ATA L1" peer in the SIP dialogue is Asterisk relaying
extension 6004, not the ATA itself, which is a separate device.  Its registered
contact was `sip:6004@172.16.0.126:32700`, the ATA's own LAN address, and
Asterisk has no route there: `pjsip show contacts` read **Unavail, RTT nan**,
and a ping from Asterisk to 172.16.0.126 loses 100% where this Mac, on that
LAN, reaches it in 7 ms.  So Asterisk received our RTP (its own RTCP reported
0.7% loss on our 600 packets) and forwarded it to an address that does not
exist from where it stands, while the ATA -> Asterisk direction worked because
that one is outbound.  Textbook one-way audio, and nothing to do with the modem.

`/etc/asterisk/pjsip.conf`, the `[endpoint-modem]` template, now carries
`rewrite_contact=yes` and `rtp_symmetric=yes`, so Asterisk uses the address the
packets actually arrive from rather than the one in the Contact header.  The
ATA re-registered as `sip:6004@10.69.111.2:32700` -- its OpenVPN address, which
Asterisk reaches in 48 ms -- and went **Avail**.

**Verified by the line itself**: after the fix a dialled call carries ringback
at a clean AC-RMS of 762 with no clipping, where before it carried an AC-RMS of
3 from the moment the digits went out.

**THE SOUNDING IS DONE, and the band above 3750 Hz is measured (2026-09-03).**
The answer was already in the recordings: the window between the digits and the
far end's answer carries our transmit at close to unity gain and does NOT clip,
and the sounder is in it.  Two-port, against a recording of the codewords we
sent:

| Hz | dB | Hz | dB | Hz | dB |
|---|---|---|---|---|---|
| 150-3450 | flat within 2.4 | 3600 | -4.3 | 3800 | -10.3 |
| 3450 | -2.4 | 3750 | **-8.4** | 3850 | -12.7 |
|  |  |  |  | 3900 | -16.3 |
|  |  |  |  | 3950 | **-22.8** |

Noise floor -39 dB, so the 3950 Hz point sits 16 dB clear of it and is a
measurement rather than a floor reading.  **It repeats within 0.2 dB over a
37 dB range of transmit level** (RMS 3510, 702 and 50), and its 3750 Hz value
agrees with V.34's own probe to 0.1 dB -- a different signal, in a different
session.

So the path does not stop at 3750: it rolls off smoothly to -22.8 dB at 3950.
The zero-excess-bandwidth reading that ruled out non-data-aided timing recovery
holds only in the sense that what is left up there is 20 dB down -- there is
energy against the 4 kHz Nyquist, not a dead band.  Folding this into the
equaliser's test channel is NOT done: the magnitudes are solid but reconstructing
an impulse response from them produced a two-lobed pulse, so the phase handling
needs checking before the model is changed.

**Still open, and it is the analogue front end.**  The moment the call
CONNECTS the received signal jumps to AC-RMS 18640, clipping 23% of samples, a
hard-limited waveform whose zero crossings put it at about 210 Hz.  It is not
what we transmit: dropping the sounder from RMS 3500 to 702 -- 14 dB, confirmed
on the transmit tap -- changed the received level not at all and the clipped
sample count by 26 in 16000.  Ringback, one-way and before answer, is clean;
whatever this is arrives with the answer.  Until it is understood the sounding
cannot be made, because a clipped receive destroys exactly the weak top-of-band
tones the sounding exists to measure.

**The rig's receive path is saturating (2026-09-03).**  Off-hook and not
dialling, the HSF hears dial tone -- 400 Hz, steady for 35 s, so the line and
the DAA are alive -- but the samples are pinned at ±32767 for 69% of a window,
a hard-clipped waveform, where the same dial tone in the previous session's
`call-c1` peaks at 10383 and never clips.  And once digits go out the receive
stream drops to an AC-RMS of **3 counts** and stays there, through the whole of
a connected call, in both directions (the far end receives 95280 octets at
RMS 1.2 from the ATA).  No measurement is possible in that state, and neither
end of it -- 80 dB apart -- looks like a linear path.  Sorting out the receive
level is the prerequisite for the sounding, and it is a rig question, not a
code one.

Two things that looked like faults and were not, recorded so they are not
chased again.  `rx 0 bytes in 0 packets` from the coupler means **no transmit
feed**, not a stopped codec: the data pump refills RX on TX completion, so a
run without `--feed` (or without the engine driving transmit) delivers nothing
and says so in its own log -- the streaming runs print "feeding signed-linear
silence out" and the dead ones do not.  And `--wait --load --script 9` was
blamed for it; it is innocent.  Note also that `--wait` alone returns the
instant EP0 answers, so on a device whose firmware is already running it exits
immediately and never sees a replug: **`--bootloader` is what makes it wait for
the window**, which is exactly what its comment says.

**Verified on the wire, 2026-09-03**: with `ME_SOUNDER=1` the digital side's
transmit tap carries all 25 tones at equal amplitude (~990 each), the empty
bins 38 dB down, RMS 3510 and peak 9852 -- the self-check's numbers, through
the real G.711 passthrough.  The measurement itself did not complete: on two
calls the analogue leg carried digital silence in BOTH directions while SIP
signalling was healthy (call CONFIRMED, media wired, and the far end received
95280 octets of silence at RMS 1.2 from the ATA), so there was nothing to
measure.  The HSF was off-hook throughout -- its own dial digits are in the
recording at RMS 25000 as hybrid echo, and the standing DC offset never changed
-- and a third call returned `rx 0 bytes in 0 packets`, the codec transport
having stopped altogether.  That is a rig state to sort out with the device in
hand, not a property of the sounder.

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

## Why the receive clips against the HT802: a headroom figure, measured

The receive path saturates on every tone the ATA generates, and the level at
which it does so is now a number rather than an impression.  All of it comes
off the recordings already in `artifacts/`; no rig time was needed.

Estimating amplitude from the clipped duty ratio (for a sine limited at `C`,
the clipped fraction is `(2/pi) arccos(C/A)`), on 300 ms windows:

| signal | capture | clipped | amplitude | dBFS |
|---|---|---|---|---|
| ringback burst | `hsf-silence-234045Z` t=2.7 | 48.0% | 43596 | **+2.5** |
| dial tone | `hsf-dialtone-225613Z` t=5.0 | 45.9% | 42345 | **+2.2** |
| post-answer tone | `hsf-silence-234045Z` t=25 | 23.0% | 33986 | **+0.3** |
| far-end V.34 probe | `hsf-v90/call-c1` t=12 | 0% | 8645 | -11.6 |

So the ATA's call-progress tones land **2 dB past full scale** and the far
end's modem signal lands 12 dB below it.  A Grandstream call-progress tone is
nominally **-13 dBm**, which puts 0 dBFS on this receive path at about
**-15 dBm** -- where a telephony front end should saturate near +3 dBm0.  The
receive path is therefore carrying roughly **18 dB more gain than the line
wants**, and that single figure accounts for everything observed: tones clip,
modem audio does not, and a data call runs on ~12 dB of headroom.

Three things this rules out.

**It is not our transmit.**  Already measured in the sounding: a 37 dB change
of transmit level left the saturating signal unchanged to the sample.

**It is not a register we chose.**  The comment in `hsf_fxo_probe.c` that "one
of these is 20-plus dB hot against the vendor's setting" is not supported.
The call sequence writes `35b7 2004 a208 f200 aae8 6040 35b4 aac8`, byte for
byte what the vendor driver writes on this hardware (`hsf_usb_daa.md`), and
`call-c1` -- which does not clip -- wrote exactly the same eight words.  The
register set cannot explain a difference it does not have.

**It is not the frequency it was recorded as.**  The post-answer tone is
**399 Hz**, not the 210 Hz that commit eb3ef515 read off its zero crossings; a
Goertzel gives 399.0 with the 3rd harmonic at 1197 and, after t=28 s, sidebands
at 389 and 409 -- a 10 Hz amplitude modulation, which hard clipping alone
cannot manufacture.

### What the tones are, and a second problem behind them

The cadence identifies the generator.  The bursts before the call is answered
run 0.4 s on / 0.2 off / 0.4 on / 2.0 off -- the NZ ringback cadence exactly --
at the same 399 Hz as the off-hook dial tone.  These are the HT802's own tones,
inserted toward the phone, which is why the far end never hears them.

But the far end never hears anything else either.  Over the whole of a
connected call the server's own receive tap reads **RMS 4.8, peak 16** in the
silence run and **RMS 57** in the sounder run -- dither, not audio -- while the
line at the HSF carries a continuous clipping 399 Hz from the answer instant
on.  So after answer the ATA is neither passing our transmit to the network nor
passing the network to us; it is playing a tone at the phone.  A hookswitch
flash (the ATA holding the call and returning dial tone) fits, and so does a
polarity reversal on answer that the DAA re-seizes through -- but the onset
carries **no DC transient at all** (the samples go from the 900-count baseline
straight into a full-level sine), which is not what a loop-current interruption
looks like, so neither is established.

### The experiment that splits the remaining two

The 18 dB is either the ATA driving hot or the HSF hearing hot, and one run
each way separates them, because the ATA's tone level and the far end's modem
level are set by different things:

* Lower the HT802's call-progress tone level by a known N dB and re-capture
  off-hook dial tone.  If our amplitude falls by N and the far-end modem level
  does not move, the tones are simply loud and the headroom figure above is
  what it is.
* Sweep the HSF's own gain field.  The register words are the vendor's, but
  four of the fields inside them (`0x8d6`, `0x8da`, `0x8dc`, `0x8e4`) are
  derived from a config struct this probe assumes is all-zero, and one is
  already known to be wrong: the last word had to be hardcoded to `0x35b4` to
  match the vendor, which means `(cfg[0] >> 5) & 7 == 2`, not 0.  `0x8d6` is
  the untested 2-bit select next to it, and it lands in the word logged as
  `0x6040`: the four values are `0x6040 / 0x6240 / 0x6440 / 0x6640`.
  `--regs d6,da,dc,e4` and `HSF_CALL_REGS` sweep them without a rebuild.

Off-hook dial tone is the stimulus to sweep against: it is continuous, it is
the ATA's own generator so it does not depend on a call being placed, and it
measures 42345 +/- 200 across every capture that contains it.

### The gain field, swept: it is real, and it is already near its floor

Run against off-hook dial tone as planned (`--call-seq --feed --stream 8
--rx-out`, no `--dial`; the ATA's own generator, so no call is needed).
`artifacts/hsf-gainsweep-000922Z/`.  Amplitude is read from the **slope at the
zero crossing** -- `A = median|v[i+1]-v[i]| / (2*pi*399/16000)` -- which is
unaffected by clipping, where the duty-ratio estimate goes to pieces above 85%
clipped.  The two agree at the baseline (42422 vs 42406) and the baseline
repeats across the session (42406, 42097, and 42132-42288 on every run that
changed nothing).

The word logged as `0x6040` **is** the receive gain, and only that word is:

| word | change | dBFS |
|---|---|---|
| `0x7050` / `0x7010` | +0x1000, +0x10 | **-1.3** |
| `0x7000` | +0x1000, -0x40 | -0.9 |
| `0x6050` | +0x10 | +0.7 |
| `0x6040` | **default** | **+2.2** |
| `0x6140` | +0x100 | +4.6 |
| `0x7c50` | | +6.1 |
| `0x6240` | d6 = 0x200 | +12.2 |
| `0x6440` | d6 = 0x400 | +17.2 |
| `0x6640` | d6 = 0x600 | +19.5 |

Everything else is inert to within the baseline's own spread (+/-0.1 dB):
`reg_dc` at 0 / 0x800 / 0x1800 (words `0x25b4`, `0x2db4`, `0x3db4`), `reg_e4`
at 0 / 0x80 and `reg_da` at 0x1000 (`0xaa28`, `0xaaa8`, `0xbae8`), `d4 = 0`
(`0xf000`), `d0 = 0` (`0x6000`), the trailing word's own e4 field (`0xaa08`),
and the vendor's **first-form** set `2004 a208 f000 a228 6000 25b4` taken
whole.  Bits `0x8/0x4/0x2/0x1/0x20/0x80/0x800` of the gain word do nothing.

**So the sweep answers the question, and the answer is no.**  The reachable
range is -1.3 to +19.5 dBFS and the default sits 3.5 dB above the floor.  The
~18 dB has to come off somewhere else: the HT802's own output level, or a
resistive pad in the line.  Every value of this field that is not the default
makes it worse or barely better, so the default stays.

Two things fell out of the sweep.

**The trailing post-off-hook write is what connects the receive path.**  With
`HSF_TRAIL_REG=0` (skip it) or `=0xaae8` (bit `0x20` set) the stream goes
completely dead -- RMS 0.5 and 3.4, no dial tone, and the DC baseline collapses
from **+900 counts to -0.4**.  The shipped `0xaac8` is `0xaae8` with `0x20`
cleared, so that bit gates receive.  A baseline re-run immediately afterwards
came back at +2.2 dBFS, which is the control that says the rig had not wedged.

**The +900-count DC offset is a health marker, not a defect.**  It is present
on every run whose receive path is connected and absent on both dead ones, so
`dc ~= 900` is the cheapest check that the codec is actually listening -- worth
knowing given the DC offset was previously treated only as something to remove
before ANSam detection.

### The decode is not the problem, and the ATA's -2 dB does not reach the tones

Two checks, both cheap, prompted by the reasonable suspicion that the clipping
is an artefact of how we read the codec's stream.

**The received waveform is a hard-clipped sine, harmonic for harmonic.**
`--rx-out` writes the USB bulk payload verbatim -- no scaling, no clamping --
and at a flat top the bytes read `ff7f ff7f ff7f`, so the rail is the device's.
Against a synthetic 399 Hz sine of amplitude 42084 limited at the int16 rails:

| | H1 | H3/H1 | H5/H1 | H7/H1 | H9/H1 |
|---|---|---|---|---|---|
| measured | 18442 | 0.0907 | 0.0318 | 0.0035 | 0.0088 |
| clipped sine, A = 42084 | 18500 | 0.0929 | 0.0344 | 0.0040 | 0.0115 |
| clipped sine, A = 38000 | 17859 | 0.0506 | 0.0298 | 0.0101 | 0.0022 |
| clipped sine, A = 46000 | 18929 | 0.1270 | 0.0269 | 0.0176 | 0.0092 |

The match is to the third decimal and the neighbouring amplitudes do not fit,
so the amplitude is pinned to within about 1 dB by the harmonics alone -- a
completely independent route to the same 42084 the zero-crossing slope gives.
**A misread format cannot produce a textbook clipped-sine harmonic series**, so
the int16-LE reading is right, the codec is linear right up to the rail, and
the line signal really is ~2 dB over the codec's full scale.

**The ATA's port gain does not apply to its own call-progress tones.**  With
the HT802 changed from 0 dB to -2 dB, the same capture reads **42084 against
the previous 42406 -- 0.07 dB, i.e. nothing**.  On this ATA the port gain is on
the RTP-to-line voice path; the tones come from the call-progress tone
generator and carry their own level in the tone string (`f1=400@-13`), which is
what has to change to stop dial tone and ringback clipping.

Which leaves the measurement that has not been made: **the voice path's
absolute level**.  It is the one that matters -- the tones are a nuisance, the
far end's modem signal is the signal -- and no capture here traverses it,
because after answer this ATA passes neither direction (server receive tap RMS
4.8, peak 16, for a whole connected call).  The way to get it without solving
that first is to make the ATA the *called* party, so the HSF answers a ring
instead of placing a call, and to put a known absolute level on the far end:
Asterisk's `Milliwatt` is a 0 dBm test tone, so

    channel originate PJSIP/6004 application Milliwatt

with the probe off-hook on ring gives dBFS per dBm0 for the whole path in one
capture -- and the -2 dB port change should then show up as exactly 2 dB.

### Could the DAA be compressing?  No -- swept today, and it expands slightly

The natural objection to "the -2 dB did not move it" is that something between
the line and the samples is levelling: an AGC or a compressing DAA would
swallow a 2 dB change, and would equally explain a level that sits pinned at
the rail whatever arrives.  It would also mean neither of the two amplitude
estimates means what it says.

`HSF_ECHO_AMP` now sets the echo tone's amplitude, so the linearity is a sweep
rather than an argument.  Off-hook, dial `1` to silence the dial tone, then
200 ms bursts of 1000 Hz every second into the ATA's digit-collect window, and
measure the returning hybrid sidetone with a Goertzel over 100 ms windows
(`artifacts/hsf-linearity-005746Z`):

| TX amplitude | RX 1 kHz | step |
|---|---|---|
| 2000 | 47.6 | |
| 4000 | 95.5 | x2.007 |
| 8000 | 192.6 | x2.017 |
| 16000 | 418.3 | x2.171 |
| 24000 | 704.4 | x1.684 (x1.5 expected) |

**21.6 dB of transmit produces 23.4 dB of receive.**  Linear to within a
fraction of a dB up to 16000 and then bending very slightly *upward* -- the
opposite of compression, and small enough to be the transmit side.  It agrees
with the earlier sounding, where the returning level tracked ours across a
**37 dB** range (3500 -> 3800, 702 -> 762, 50 -> 55, ratio 1.086/1.085/1.100).
(At amplitude 1000 the echo is under the line's own noise: the "peak" windows
land at 0.8-1.1 s, on the dialled digit, not on the 6-11 s bursts.  Excluded.)

So all three ways the 2 dB could have hidden are now closed:

* **not the estimator** -- the zero-crossing slope is read where the waveform
  is nowhere near the rail, so it reports the true amplitude of a clipped sine;
  a 2 dB change of the line would appear as 2 dB of slope;
* **not the decode** -- the harmonic series is a hard-clipped sine's, to the
  third decimal;
* **not a compressing DAA or codec AGC** -- the path is linear over 21.6 dB
  today and 37 dB in the sounding, and the gain word moves the level by up to
  19.5 dB, so the chain plainly does respond to a real gain change.

The remaining reading is the simple one: the HT802's port gain does not act on
the tone its own call-progress generator produces.  Worth confirming the -2 dB
was applied at all (it should show on the *voice* path), and the Milliwatt
originate above is the measurement that would show it -- but note that even if
it applies, 2 dB is not the 18 dB the headroom figure asks for.

### CORRECTION: the receive path is correctly scaled.  9099 settles it

Dialling out works and always did.  Extension **9099 is an echo test**, and
one call to it answers the level question completely -- with a known transmit
amplitude and a return path, no assumption about anybody's nominal level is
needed.  `--dial 9099 --echo-tone 1000 --echo-on-ms 60 --tx-out`, transmit
amplitude 8000 (`artifacts/hsf-echo9099-010721Z`).  Four bursts arrive per
second and `--tx-out` pins which is which:

| burst | delay after our own | peak |
|---|---|---|
| local hybrid sidetone | +40 ms | 610 |
| network echo | +280 ms | **8088** |
| echo round two | +540 ms | 594 |
| echo round three | +800 ms | 55 |

**We transmit 8000 and get 8088 back through the ATA, Asterisk and the echo
application: the voice path is unity to within 1 dB, round trip.**  The whole
25 s call peaks at 8258-8546 with **zero clipped samples**.  The trans-hybrid
loss is ~25 dB (610/8000), which is healthy, and the round-trip delay is
280 ms with each further lap 22 dB down.

So **the earlier "0 dBFS is about -15 dBm, roughly 18 dB too much receive
gain" is withdrawn.**  It rested on assuming the HT802's call-progress tones
sit at their nominal -13 dBm; the echo test shows they do not.  With the voice
path at unity, 0 dBFS is 0 dBm0, and a far end transmitting at -13 dBm0 lands
at -13 dBFS -- which is exactly where `call-c1`'s V.34 probe landed (-11.6),
and it is 13 dB of headroom, not 2.  Nothing is wrong with the receive gain,
the DAA, the codec or the decode.

**What is hot is the HT802's tone generator, by about 15 dB.**  Its dial tone
and ringback arrive at +2.2 dBFS, i.e. it is generating them at roughly
+2 dBm0 where a call-progress tone should be near -13.  That is the tone-level
field in the call-progress tone string, which is also why the port gain change
from 0 to -2 dB moved them not at all: the port gain is on the voice path, and
the voice path was never the problem.

**And the 400 Hz at answer is confined to the 6001 leg.**  The 9099 call is
silent between bursts (RMS 0.1-0.3 at 1 kHz) for its whole length -- no tone
appears when it connects, at either 1000 Hz or 2100 Hz transmit.  So the
continuous clipping 399 Hz that arrives the moment our own `sip_v90_modem`
answers, together with that leg carrying no audio in either direction, is a
property of that call and not of the ATA answering calls in general.  A plain
2100 Hz from our side does not provoke it on 9099, so a naive
fax/modem-tone detector in the ATA is not enough to explain it on its own.

### CORRECTION: the 6001 leg carries audio both ways, and the 400 Hz is the HANGUP

Both halves of "after answer this ATA passes neither direction" were mine.

**The line-to-network silence was us feeding silence.**  All three runs the
claim rested on say so in their own logs -- `streaming for 35s (feeding
signed-linear silence out)` -- because they ran `hsf_fxo_probe`, not the
coupler, so there was no modem and no audio on the analogue side at all.  The
server's `RMS 4.8, peak 16` was a faithful recording of nothing being sent.

Run the same call with a tone instead (`--dial 6001 --echo-tone 1000
--echo-on-ms 60`, transmit amplitude 8000, server restarted with
`VPCM_G711_TAP_DIR`, `artifacts/hsf-6001-audio-011539Z`):

* **HSF to server**: our 1 kHz bursts arrive in `live-rx.g711` at **peak 5884-6140**,
  `dom=1000.0Hz`, once per second, for the whole call.
* **server to HSF**: the server's ANSam sits on the line at **2100 Hz,
  magnitude 3500-4100, with zero clipped samples**, continuously from 12.60 s
  to 23.50 s of the capture.

So the leg is healthy in both directions, at a sane level, with headroom.

**And the 400 Hz starts at the DISCONNECT, not at the answer.**  In the same
capture the 2100 Hz stops and the 399 Hz starts in the *same* 100 ms window --
`23.40  2100=3820  400=4.6  clip=0.00` then `23.60  2100=907  400=9199
clip=0.21` -- and the ANSam ran 10.9 s against the media tap's 11.9 s call.
Our own modem fails V.8 twice against a line with no modem on it, hangs up
about twelve seconds in, and the ATA then does what an ATA does for an
off-hook line with no call: **dial tone**.  Every earlier capture ran 35 s over
a 12 s call, so the tone filled two thirds of the recording and looked like it
began at the answer; the alignment that said so was a cross-clock guess of
exactly the kind this file warns about elsewhere.

`eb3ef515`'s "whatever this is arrives with the answer" is therefore wrong too,
and with it the idea that something breaks when the call connects.  Nothing
breaks.  What is left of the whole thread is the one measured fact: the HT802
generates its call-progress tones about 15 dB hot, so **dial tone and ringback
clip the ADC** -- which matters for a modem only in the windows where they are
present, i.e. before dialling, during ringback, and after a hangup that leaves
the line off-hook.

### A real coupler call to 6001, with the level question settled

`RUN=... SECS=90 SETTLE=20 tools/hsf_v34_call.sh`, server restarted with
`VPCM_G711_TAP_DIR` so both taps are kept (`artifacts/hsf-v90/call-012705Z`).

**The clipping is not in the call.**  Over the whole 90 s the only clipped
samples are the ringback bursts at t=3 and t=6 (464 and 465 per 4000) and the
startup transient.  From the answer on, the line runs at RMS 5000-5600 with
peaks of 9500-10800 and **zero clipped samples** -- 10 dB of headroom, exactly
as the echo-test calibration says it should be.  So the ATA's hot tone
generator does not touch the modem's own window, and the level thread is closed
for the purposes of this call.

**What blocks it now is V.8, and the two ends are out of step.**  The analogue
side detects ANSam, starts, and transmits CM throughout -- but reads the
answerer's JM as `call function=V series modem data, modulations=none`, takes
its 10 s timeout (`status=Call negotiation failed (4)` at `+10080ms`) and stops.
The digital side reads our CM the same way twice (`modulations=none`), declares
`V.8 failed before CM ... retrying answer tone with ANSam`, and on that retry
decodes it **completely** -- `V.22/V.22bis duplex, V.34 duplex, V.90 duplex`,
`protocol=LAPM`, `PCM=V.90/V.92 digital available` -- selects V.90 and enters
`state=TRAINING mod=V90` about a second *after* the analogue side has given up.
It then trains alone for 80 s against a modem that is no longer there.

So both ends decode the other's V.8 frame with an **empty modulation list** for
the first ten seconds and then, on the answerer's second ANSam, the digital
side gets a perfect one.  That symmetry is the thing to chase: an empty list
inside a frame whose call function decoded is a V.21 frame that framed and then
lost its later octets, not a failure to hear the peer at all.  Both taps for
the call are on disk, so it can be worked offline.

## The "empty modulation list" is the CI, and the real fault is a CM/JM race

Worked entirely off the two G.711 taps of `artifacts/hsf-v90/call-012705Z`, with
a 60-line V.21 demodulator and 10-bit framer that prints **every** octet
sequence a receiver could have framed, so the wire is read independently of
either modem's own state.

**The empty list is correct reporting of a CI.**  In the answerer's receive tap:

| tap time | octets |
|---|---|
| 2.633 - 4.463 s | `00 C1` repeated -- **CI**, sync octet `0x00`, call function only |
| 5.375 s | first complete **CM**, `E0 C1 65 12 10 2A 47 8D`, then a corrupt repeat |
| 6.435 / 7.037 s | CM then a corrupt second copy (`... 65 16 10`, `... D0 82`) |
| 7.637 s | **five clean identical CM repeats** |

A CI carries no modulation octets, so `call function=V series modem data,
modulations=none` is exactly right for it -- and the engine's own log confirms
the status: `V8 result: status=Call function (CI) received (5)`.  It is
`me_log_v8_peer_summary()` printing an in-progress event, not a decode failure.
`vpcm_decode --v8` reads the same CM out of **our own transmit tap** (`C1 65 12
10`), so the CM we send is well formed and the CM the answerer receives is the
same bytes.

**What actually fails is a race, and it reproduces offline to the millisecond.**
`v90_engine_replay <tap> ulaw --fast --from 0` prints SpanDSP's own
`Timeout waiting for CM`; `--from 2.0` prints `CM recognised` and no timeout.
(`v90_engine_replay` normally skips to `find_call_start()`, which is why an
unqualified replay of this call succeeds where the call failed -- **pass
`--from 0` to reproduce a live V.8 failure**.)  The budget is in `v8_restart()`:
the answerer's CM-wait is `200 + 5000 ms` from media connect, and

* our first complete CM lands at **5.375 s**, ~200 ms inside it,
* but `got_cm_jm` wants two identical repeats and the early ones carry bit
  errors, so the first clean pair is at **7.637 s**,
* the answerer times out, retries ANSam, decodes the CM immediately, and sends
  **JM at 11.453 s** (measured in its own transmit tap),
* by which time the analogue side has taken its own 10 s V.8 timeout at
  **+10080 ms** and hung up -- it misses the JM by about 1.4 s.

**Why our CM is late: two ANSam detections in series.**  The coupler starts the
engine only once it has itself detected ANSam (~0.3 s, and it exists because
starting on a post-dial timer ran V.8 into ringback), and SpanDSP's V.8 caller
then runs `V8_WAIT_1S` -> CI -> its own `ansam_rx` detection -> `Te` before it
will send CM.  From `enter V8` to the CM on the wire is **3.7 s**, against an
answerer waiting 5.2 s from a point 0.3 s earlier.

`ME_V8_NO_CI=1` skips CI so the caller enters `V8_AWAIT_ANSAM` directly.  **It
is default off and NOT yet demonstrated**: in `--rx-replay` the arms differ
enormously in the right direction -- with CI the caller emits **no CM at all**
in 10 s of V.8, without it the CM goes out and repeats -- but that replay does
not reproduce the live caller timing either (live, with CI, the CM did go out
at 3.7 s), so the measurement grades the knob's direction and not its worth.
It needs a live A/B.  CI is in any case addressed to the network before the
answer, and a caller that starts V.8 *because* it has already heard ANSam has
nothing to announce with it.

### ME_V8_NO_CI live A/B: no benefit, and my model of the delay was wrong

`tools/soak/hsf_v8_noci_ab.sh`, three repeats a side, arms **alternated**, one
binary, the server restarted per call so each gets its own G.711 taps
(`artifacts/noci-ab-135307Z`).  Scored with `tools/noci_ab_summary.py`, which
skips a run whose `coupler.log` has not reached its terminal summary -- a run's
directory exists from the moment it starts, and reading it early reports a call
that has not happened yet as a failure.

| run | arm | V.8 | at | answerer CM-wait timeouts |
|---|---|---|---|---|
| noci-r1 | no CI | successful | 6316 ms | 0 |
| noci-r2 | no CI | **failed** | 9970 ms | 1 |
| noci-r3 | no CI | successful | 5408 ms | 0 |
| ci-r1 | CI | *no call placed* | - | - |
| ci-r2 | CI | successful | 5416 ms | 0 |
| ci-r3 | CI | successful | 5432 ms | 0 |

**no CI 2/3, CI 2/2.**  The knob does not help, and the arm without it did
marginally better.  (`ci-r1`'s dial never reached the PBX -- the server logs no
incoming call -- so it is a rig miss, not an arm result, and the tool says so
rather than counting it as a failure.)

**The control says the knob did what it claims**: counted on the answerer's own
receive tap with the V.21 framer, the no-CI arm carries **0** bursts containing
`00 C1` and the CI arm carries 1 and 3.

**And that measurement refutes the model behind it.**  First CM on the wire:
no-CI 4.263 / 6.574 / 4.222 s, CI 4.159 / 4.388 s.  **The CI phase costs
nothing** -- SpanDSP evidently runs it while waiting for ANSam and reaches CM at
the same instant either way, so "V8_WAIT_1S plus the CI phase is what puts our
CM past the answerer's budget" is wrong.  What actually varies is ANSam
detection: the one failing call had its CM at **6.574 s**, 2.3 s later than
every other call in either arm, and that is what missed the ~5.2 s window.

So the race is real and per-call, the determinant is when the CM arrives, and
CI is not what delays it.  `ME_V8_NO_CI` stays default off with this table
beside it.  The levers left are the ones that address the arrival time itself:
the answerer's `200 + 5000 ms` CM-wait budget, `got_cm_jm`'s requirement for
two *identical* repeats when the early ones carry bit errors, and the caller's
own 10 s V.8 timeout, which in the failing call expired 1.4 s before the JM.

Worth noting for the next thread: all four successful calls entered TRAINING as
**mod=V34**, not V90, on a `v90` profile whose V.8 result carries
`pcm=0x2` (V.90 available).

### "Training V.34 not V.90" was two different things, and one was a real bug

**The trace label is a red herring.**  `start_v34_training()` sets
`g_mod = ME_MOD_V34` and traces `enter TRAINING: mod=V34` unconditionally,
because **V.90's Phases 2-4 ARE V.34's** -- the same log says
`V8 selected V90` and `V.8 negotiated V.90 PCM downstream + V.34 upstream` two
lines earlier, and the `[ME] V.90 bridge:` lines follow.  The call was on the
V.90 path throughout.  The trace now reports `mod=V90 (via V.34 phases)` where
the digital role is selected, so a log cannot say that again.

**The real bug: the coupler was running the DIGITAL role on an analogue line.**
`me_v90_analogue_role()` is gated on `ME_V90_ROLE=analogue`, and neither
`hsf_v90_coupler` nor `tools/hsf_v34_call.sh` set it -- so the V.8 result took
the digital-role branch and *both ends of the call believed they were the
digital modem*.  The giveaway is in the log: `V.90 notch filter at 1200 Hz (our
CC TX), RX CC at 2400 Hz`, which is §8.2.3.1's assignment for the **digital**
side.  `--v90-couple` now defaults `ME_V90_ROLE=analogue` (`setenv(..., 0)`, so
an explicit setting still wins); this binary is wired to a 2-wire line through
a DAA and V.90 puts the analogue modem on the calling side, which is the side
that dials, so there is no other correct value.

Live, one call with no environment overrides at all:

    [ME] V.90 role: ANALOGUE (opt-in; Phase 4 B1/B1d ... enabled)
    [ME] V.90 analogue DIL: measurement-120x66 -- 120 segments, 990.0 ms
    [ME] V.8 negotiated V.90 with this end as the ANALOGUE modem (U_INFO=78)
    [ME] V.90 analogue: notch at 2400 Hz (our CC TX), RX CC at 1200 Hz
    [ME] V.90 analogue Phase 3 started: symbol-rate code 4, high carrier,
         U_INFO=76, Ja descriptor N=120 (1260 bits)

-- the mirror-image notch, a real DIL descriptor, and the analogue Phase 3
transmitter running, none of which happened before.

**The blocker has moved to the digital side.**  It stays at
`tx=V90_PHASE2_B_INFO0_SEEN(24)` with `rx=TONE_A(5)` and `phase3_started=0`,
never transmits Sd, and the analogue side takes its §9.3.2 deadline in Ja at
+16222 ms and initiates the §9.5.2.1 retrain.  That is the coupled Phase 2 seam
`vpcm_loopback_test` covers between our own two roles, now facing a real line.

## It DOES send Sd.  The analogue side cannot hear it, and Ja then reads as a retrain

The digital side's own log settles the premise:

    [V90] Phase 3: analogue Ja detected, starting Sd
    [V90] Phase 3: Sd complete (64 reps), starting S-bar-d
    [V90] Phase 3: S-bar-d complete, starting TRN1d

and it is on the wire at the analogue end: scanning `hsf-rx.raw` for **1333 Hz**
-- 8000/6, the line Sd's six-symbol pattern puts in the band -- the window at
25.80 s reads magnitude 1384 on an RMS of 3189, about 38% of the power, with
2400 Hz at 6.  Sd reached the analogue modem's ADC.

**What fails is the analogue receiver, and it is the documented open item:**
`[ME] V.90 analogue RX: hunting Sd (Sd 0 reps, S-bar-d 0 reps, TRN1d 0T, Jd 0
frames)` for the whole attempt.  `v90_analogue_rx.c` is a **codeword** state
machine -- correct when the downstream is the DS0 stream, and there are no exact
G.711 codewords left after a 2-wire analogue round trip.

The rest follows mechanically, and all of it is correct behaviour given that:

1. §9.3.2.3 has the analogue modem transmit Ja **until it detects Sd**, so with
   Sd undetected it keeps Ja up.
2. Ja rides the analogue modem's control channel at **2400 Hz** -- which is
   exactly the tone the digital side's §9.5.1.2 retrain watch listens for (the
   tone is chosen by role: 2400 Hz where `calling_party` and `v90_mode`
   differ).  §9.3.2.4 makes the analogue modem silent through `PHASE3_WAIT_S`,
   so sustained 2400 Hz there is read as a retrain request:
   `Rx - Tone A detected in stage PHASE3_WAIT_S (80 ms); peer initiated a
   retrain` -> `Peer retrained during tx_phase=5; dropping to WAIT_JA`.
3. The analogue side then takes its own §9.3.2 deadline in Ja at +16222 ms and
   initiates the §9.5.2.1 retrain.  Both ends restart.  Loop.

Reproduces offline from the digital side's tap, which is where to work it:

    ME_V34_SPAN_FLOW_LOG=1 ./v90_engine_replay <live-rx.g711> ulaw --fast --from 0

**The primary fix is the analogue Sd detector** -- it needs the equaliser, not a
slicer, which is exactly the ordering the symbol-timing section above already
settled (an FSE first, then decisions).  **The secondary defect is real but
would not rescue this call:** 80 ms of 2400 Hz cannot distinguish "the peer is
still in Ja because it has not seen my Sd" from "the peer wants a retrain",
because both are 2400 Hz and only the *modulation* differs -- Ja is a data
signal and §9.5.2.1's Tone A is pure.  A purity test (energy in the carrier bin
against the ±300 Hz control-channel skirts) separates them; nothing here
measures it yet.

## The equaliser for the Sd hunt: fitted to §8.4.4, not blind

The blocker above -- Sd arrives, the analogue receiver never finds it -- is a
chicken-and-egg, and naming it that way is what solves it.  The codeword slicer
has to be told what level a Ucode is before it can produce codewords; the only
thing on the line that says so is Sd; and the Sd hunt reads codewords.

**The blind equaliser cannot break it, and this is measured rather than
argued.**  CMA drives every output symbol to one modulus, and §8.4.4's Sd is
four slots at W and *two at zero*.  Fed real Sd through a dispersive channel the
existing FSE produces **±1 on all six slots** -- the sign pattern survives and
every trace of the structure the hunt looks for is gone.  CMA is right for
§8.4.5's TRN1d, which is scrambled ones on a single Ucode and genuinely constant
modulus; it is simply the wrong algorithm for the signal that comes first.

Sd needs no blind algorithm, because it is a **known sequence**: §8.4.4 makes it
+W, 0, +W, -W, 0, -W, repeating.  So `v90_analogue_sd.c` fits the T/2 taps to it
directly, by least squares over a window, and what falls out is not just an
acquisition but the equaliser itself.

Four things the fit absorbs rather than searches, which is what keeps it to one
solve instead of twelve: the **channel**, the **sampling phase**, the **slot
phase** and the **whole-symbol delay** (a fractionally-spaced filter of this
length can supply any delay, so every slot hypothesis fits equally well and
differs only in which tap carries the energy), and the **sign** (fitting -r
returns -h with an identical residual).  It therefore cannot tell Sd from
§9.3.2.4's S-bar-d -- and does not need to, because what marks that boundary is
the *change*, which is unambiguous.  One thing it cannot absorb and is told
instead: the **T/2 parity**, the two eyes, since a filter fitted for even
samples evaluated on odd ones is shifted by half a symbol.  Both are tried.

**The score is held out** -- fitted on the first half of the window, residual
measured on the second.  That is the whole of what stops it accepting anything:
with 32 free taps an in-sample fit explains noise perfectly.  Measured:

| input | held-out score |
|---|---|
| Sd through the channel, any sampling phase | **1.000** |
| Sd at 25 dB SNR | 0.998 |
| Sd at 18 dB SNR | 0.994 |
| TRN1d-like (one level, random signs) | **-0.138** |
| noise | **-0.034** |
| silence | rejected (R not positive definite) |

The gate is 0.80, in the empty middle.  `v90_analogue_sd_test` runs all of it
plus the streaming structural scorer -- the ratio test on an already-equalised
stream, which rejects TRN1d, silence, noise, and a six-periodic signal with the
right zeros and the wrong signs (the control that says the sign term does work).
End to end, fit then structure, it acquires at sampling phases 0.00/0.25/0.50/
0.75, at a 10x gain change, and at 18 dB SNR.  In `make test`.

Live wiring: `me_rx_v90a_16k()` now starts the equaliser **FROZEN**, buffers a
128 ms window, fits, installs the taps and only then lets a symbol reach the
slicer; CMA starts at TRN1d where it belongs.  A window that does not contain Sd
slides by half and is logged with its score, so a live log says whether the
detector saw nothing or scored something it did not believe.

**Not verified live.**  Three call attempts after the change: two lost the
intermittent V.8 CM/JM race documented above and never reached Phase 3, and one
did not reach the PBX at all.  A stray coupler process from a timed-out run also
held the USB interface for one of them -- `hsf_fxo: claim interface 0 failed:
LIBUSB_ERROR_ACCESS` is that, not a wedged device, and killing the process
cleared it.
