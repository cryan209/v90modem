# V.34 data mode above 12000 bit/s

Status 2026-08-22.  Before this, **nothing above 12000 bit/s decoded at all** --
not against a foreign modem, and not over a bit-exact loopback against our own
transmitter.  14400, 16800, 19200 and 21600 each trained, entered data mode and
then produced white output.  The rate ladder had only ever been exercised at
9600, which is why the matrix work of 2026-08-21 could sweep every symbol rate
and still miss this entirely.

21600 now recovers payload with **zero errors at 2800, 3000, 3200 and 3429
baud**, in both companding laws, and 14400 and 19200 are clean at 2400 baud.
`make test` asserts six 21600 rows beside the ten 9600 ones.

## How to see anything here at all

Every metric available inside the receiver is confounded on a dense
constellation, because they all rest on its own decisions and its own decisions
are what is wrong.  The instrument that broke this open is
`V34_DATA_TX_DUMP`: the transmitter's data-mode symbols, in the same Q9.7 units
and the same order as the receiver's `V34_DATA_FRAME_DUMP`, so the two files
subtract.  Correlate them per window and the window's complex ratio is the
channel's gain and phase; what is left is the honest residual.

Two cheap in-receiver reads came out of it and are worth keeping:

* `Rx - DATA: distance to grid` -- mean squared distance to 9.x's odd-integer
  lattice.  **0.667 is the value for symbols with no relation to the lattice at
  all** (uniform over a cell), so a reading at two thirds means white, not
  noisy.  Its absolute value otherwise means nothing without knowing the
  constellation.
* `Rx - DATA: shell index over k bits` -- 9.6.3.3's r0 bound, which owes nothing
  to the content, so it separates wrong grouping from wrong symbols.

Read together: grid small + shell 0% is a working data mode, grid small + shell
high is correct symbols grouped wrongly, grid large is a fault before the
mapper.

## Defect 1: a residual carrier of 0.7 Hz that data mode never removed

Against ground truth the phase advanced **0.1054 degrees per symbol**, 12748
degrees over one call.  It is there at every rate -- the receiver's own log
reads 1799.3 and 1800.4 Hz where the truth is 1800 -- but at 9600 the
decision-directed loop pulls it out and the measured drift is exactly zero,
while above 12000 the first decisions are already wrong and it never can.  The
loop holds a lock it cannot acquire, and the split falls where the uncoded Q
bits appear: 12000 (q=0) is clean, 14400 (q=1) is not.

10.1.3.1's B1 is a known 96-symbol sequence, so it can supply the *frequency*
and not just the phase.  Correlate its two halves separately; the angle between
them is the advance over half the sequence.  Averaging coherently inside each
half before taking an angle is what makes it work at this length -- the obvious
one-lag autocorrelation over all 96 symbols reads 0.91 degrees per symbol where
the truth is 0.105.  The derotator advances by the result every symbol, and the
data-mode loop integrates the remainder into it as a second-order term.

Both loops steer only on decisions close enough to the lattice to be worth
trusting.  Ungated, the frequency integrator accumulates the noise from wrong
decisions and drives the phase *away*: 0.139 degrees per symbol and a 17434
degree excursion, worse than having no loop at all.

## Defect 2: nothing adapted the equalizer in data mode

CMA stops in Phase 4, and the taps it leaves are wrong by about 5% on the
symbols either side of the main one.  Measured against ground truth, a linear
filter on the *true* symbols removes 79% of the residual power on a linear
bearer, so it is ISI and not noise.  That costs 9600 nothing and is fatal to a
dense constellation: the same taps put 21600's symbols -- RMS ~20 on a grid of
spacing 2 -- a whole decision region from where they belong.

Decision-directed LMS now runs in data mode, with the decision mapped back
through the derotator into the equalizer's own domain.  Without it, 2800, 3000,
3200 and 3429 at 21600 carry 40358, 1075, 589 and 427 errors instead of none.

### The loop must freeze when the receiver stops being healthy

This is the part the loopback could not show.  Live at 3000 baud/9600 against
the SmartLink rig, the LMS made things **worse**: a call sitting at 0.10 from
the lattice took one disturbance about twelve thousand symbols in, went to 0.66
for the rest of the call, and delivered 73 of 300 pattern lines -- where the
same call with `ME_V34_DATA_EQ=0` held 0.10 throughout and delivered all 300.
The loop is a ratchet: after a disturbance the decisions are wrong and it trains
the taps on them, so there is no way back.

A smaller step is the wrong lever and was tried: at 0.02 and 0.05 of the
training step the live behaviour is no better and the loopback regresses (2800
goes from 0 errors to 3062).  What was missing is a health test.  The receiver
takes the baseline distance that settles just after B1 -- the absolute value
means nothing on its own -- and adapts only while it stays near it.  Freezing on
the way down leaves the frozen-tap receiver as the worst case rather than
something worse than it.  Live that is parity: 34 windows at 0.11-0.15 and all
300 lines, with the collapse that remains at the very end of the call happening
with adaptation off too.

## What the rates cost, in SNR

`V34_DUPLEX_NOISE_DB` puts calibrated white noise on the harness bearer, and
`V34_DUPLEX_G711_SNR` reports what the codec itself costs on the actual
waveform (37.3 dB at this transmit level -- so G.711 is not the constraint at
any rate the ladder reaches).

21600 at 3200 baud, injected SNR against bit errors over ~16000 bits:

| injected SNR | errors |
|---|---|
| 40 dB | 0 |
| 34 dB | 27 |
| 30 dB | 59 / 103 |
| 27 dB | 1060 / 947 |

6.75 bits/symbol wants about 34 dB in theory once V.34's coding and shaping
gain are counted, so the receiver is within a few dB of where it should be.

**21600 at 2400 baud is deliberately not asserted.**  It is 9 bits/symbol, an
896-point constellation, the densest configuration V.34 allows, and the bearer's
own noise floor is marginal for it; it reaches about 2% bit errors and the
result is dominated by an acquisition coin flip rather than by steady-state
SNR.  Every other symbol rate reaches 21600 without an error, which is how a
real modem gets there.

## Live: the rig's channel is the binding constraint, and it is measured

Against the d-modem/slmodem rig our receiver measures **16.8 dB** on the wire
(mean symbol power 7.22, distance 0.1517, at 3000 baud/9600 where the call is
healthy and V.42 LAPM carries all 300 pattern lines).  21600 needs about 37 dB.
That gap is not something the receiver can close, and two independent things
agree with the number: the peer's own receiver measures its side and selects
**9600** for its receive direction, and raising the rig's deliberate -12 dB pad
(`HEADROOM=1.0`, now a parameter of `tools/soak/v34_lapm_call.sh`) changes the
measured distance not at all -- 0.11-0.14 either way.  So the rig carries
~9600, live 21600 is out of reach on it, and the receiver work above is
demonstrated on the loopback and on calibrated noise instead.

The open item this leaves is the one the peer already implements: **choose the
rate we ask for from a measured receive SNR** rather than from a configured
start profile.  The distance-to-grid figure above is exactly the measurement
that would drive it.

## Live: 9600 was never the ceiling -- 14400 runs on this rig (2026-08-22)

The section above concluded that the rig carries about 9600 and that live
21600 is out of reach.  The first half of that was wrong, and the reason is
worth recording because it was hiding in plain sight in the call that
measured it.

**Every live plain-V.34 call had `ME_V34_BPS=9600` set.**  The rate this modem
asks for in MP is `min(configured start profile, L1/L2 probe projection)`, and
the profile was the binding term, so 9600 is what we asked for and 9600 is
what the peer sent.  Its own log says so: in `artifacts/v34-21600-20260822T-c65`
the peer measured **its** receive direction at `equerr 1593`, chose data rate 6
(14400) for what it would accept from us, and then transmitted 9600 -- our
number, not its own.  The 16.8 dB figure was a correct measurement of a
channel nobody had asked to work harder.

Asking for more, at 3000 baud, three calls kept under `artifacts/v34-rate-*`
and driven by the new `tools/soak/v34_rate_call.sh`:

| asked | peer transmitted | distance to grid, our receive | outcome |
|---|---|---|---|
| 14400 | 14400 | 0.32-0.39 for 13 s, then 0.65 | decodes, 245 payload lines |
| 16800 | 16800 | 0.63-0.68 from the first window | white for the whole call |
| 21600 | 21600 | -- | `NO CARRIER`, 3944 symbols |

The line measures **18 dB** at every one of those rates -- it is the
constellation that changes, not the channel -- and 18 dB is enough for 4.8
bits/symbol and not for 5.6.  16800 failing from its *first* window rather
than degrading is what an SNR cliff looks like; it is not a defect, and
16800 is clean on the loopback at the same symbol rate.

So `>9600` on this rig is achievable, and 14400 is where it stops.

### Nothing before data mode can measure this line

Both of the places a rate decision could be taken were checked, and neither
can see the impairment:

* **The Phase-2 L1/L2 probe.**  Extracted from the recording at t=9.02 s of
  the c65 call, its unoccupied bins read 1.7e-24 -- the rig adds *no* noise,
  and the code's `if (noise < 1.0f) noise = 1.0f` floor is all that keeps the
  projection finite.  It reported 37.9 dB for a data mode that then ran at
  17.6.  The impairment is signal-proportional, and a 21-tone probe on a
  150 Hz grid puts its own distortion products back on that grid.
* **The Phase-4 TRN segment**, which is what the peer uses (`V34EQU equerr`
  feeding `V34DATARATE` thresholds one log line before `txmp bits`).
  `v34_phase4_trn_measured_rate_n()` implements the measurement; swept over
  `V34_DUPLEX_NOISE_DB` at 40, 34, 30, 27, 24 and 20 dB it reads 9.8, 8.7,
  8.4, 8.4, 11.4 and 9.8 dB while the same calls' data mode moves from 0.0018
  to 0.0285.  It is measuring this receiver's own Phase-4 convergence, not
  the line.  **Our Phase-4 TRN symbols sit at ~10 dB on a bearer where our
  own data mode reaches 31 dB** -- 20 dB worse, and the same story as Phase 3
  ("TRN mean distance to nearest 4-point 0.620").  Until that is closed, the
  measurement is logged and never applied; `ME_V34_TRN_RATE=1` applies it,
  which on a clean loopback caps 9600 down to 4800, which is the point.

The one instrument that does see it is the data mode itself, so
`Rx - DATA: distance to grid` now reports the receive SNR and the rate the
line will carry alongside the distance, and says plainly when the output is
white.  The rate mapping is `bits = (snr_db + 13)/6`, calibrated so that this
one 18 dB channel accepts 14400 and refuses 16800 -- one channel, not a
derivation.

**Open, and now the whole of it:** closing the loop.  The measurement exists
and the line's answer is known only *after* the rate has been committed, so
V.34 12.2's rate renegotiation (the peer's `renegDownthresh` /
`renegUpthresh`, which it computes on every call) is what turns it into a
rate this modem picks by itself.  Until then the rate is `ME_V34_BPS` and the
default -- the maximum for the symbol rate -- is wrong on any real line.

## Beyond 14400 on this rig: no, and the ceiling is the peer's transmitter

> **Superseded — this section's conclusion is wrong.**  Every bound in it was
> measured with the carrier taken as exact and on calls whose decisions were
> not clean, and both errors push the number down.  The wire is ~38 dB, not
> 19.5, and the ceiling was a filter inside our own engine.  The section is
> kept because its method is sound and its ruled-out causes stay ruled out;
> read the section after it for what is actually true.


14400 is where this rig stops, and the reason is now measured rather than
inferred.  Every number below is receiver-independent.

`tools/v34_channel_bound.py` answers the question that cannot be answered from
inside the receiver: given a recorded tap and the symbols the receiver
decided, it fits the **best possible** fractionally-spaced least-squares
equalizer -- non-causal, as long as you like, solved exactly rather than
adapted -- and reports the residual.  That is an upper bound on every linear
receiver, ours included.

On the 14400 call (`artifacts/v34-rate-20260822T081433Z-b3000-14400`):

| measured on | best possible linear receiver |
|---|---|
| what we received, whole path | **19.5 dB** (41 taps 18.7, 81 taps 19.2) |
| our actual receiver, same call | 18.5 dB |
| **slmodem's own 9600 Hz output**, before any resampler, codec or network | **21.5 dB** |

So our receiver is within **1 dB** of the best any linear receiver could do on
that recording, and the whole rig path -- 6:5 polyphase resampler, mu-law,
Asterisk -- costs about 2 dB on top of a source that is already only 21.5 dB.
**The ceiling is the far modem's own transmitter.**  Its pre-interpolation tap
is written unconditionally to `/tmp/dm_from_dsp_9600.raw` in the d-modem
container, which is what makes that row possible; pull it with
`docker exec d-modem cat` and pass `--rate 9600 --raw16`.

The channel is static, not time-varying: the same fit over 1024, 512, 256 and
128-symbol windows gives 20.0, 20.2, 20.7 and 21.7 dB, which is one number
plus each window's overfitting bias.  So there is no time-varying component
for a smarter receiver to chase.

Four candidate causes were tested on the rig and all four are ruled out:

* **level** -- `HEADROOM=1.0` against the default 0.25 (a 12 dB change in the
  peer's transmit level): 18.4 dB against 18.5.
* **timing slips** -- `tools/measure_timing_slips.py` finds one slip in the
  whole call, where the V.90 soak rounds saw 28 in 290 s.
* **band clipping** -- the rig's ZOH loop model and its 4400 Hz lowpass are
  built but *not* in the path; `ACTIVE downstream path: windowed-sinc
  polyphase interpolator` is, at 257 taps.
* **the upstream resampler kernel** -- `DM_UP_TAPS=64` against the default 32
  (both at fc=3700): no improvement, 17.6 dB.  `DM_UP_TAPS` and `DM_UP_FC` are
  now plumbed through `tools/soak/v34_lapm_call.sh` for anyone re-testing.

### Why 16800 cannot fit inside 19.5 dB, arithmetically

The lattice spacing is always 2 in these units, so at a fixed SNR the absolute
distance to the grid scales with the constellation's power.  14400 at 3000
baud runs at mean symbol power 21.7 and distance 0.35.  16800 is 1.74 times
the power, so the same 18.5 dB puts it at 0.61 -- and 0.667 is white.  That is
exactly what the live calls show (0.63-0.68 from the first window).  Getting
16800 to a working 0.35 needs about +2.4 dB, i.e. ~21 dB received, which is
above what the wire delivers and at the far transmitter's own source limit.

### The one route that is not SNR-limited, and it is ours

16800 at **3429 baud** is 4.9 bits/symbol -- the same density as the 14400 that
works -- and would fit inside 19.5 dB with room to spare.  It does not train.
Live at 3429 the peer sets up its modulator and demodulator at 3429/1959,
reaches `XMITMP`, sits there and then `going into retrain in Handshake`: our
Phase 4 never completes the MP exchange, at 16800
(`artifacts/v34-rate-20260822T083436Z-b3429-16800`) and at 9600
(`...-b3429-9600`) alike, so there is not even an SNR reading for that symbol
rate.  3429 is the same row that is weakest on the loopback -- 3429/19200 does
not train there either, and 3429 has been the one row outside `make test`'s
asserts since the symbol-rate matrix work.

**Fixing 3429 is the whole of the remaining route past 14400 on this rig**, and
unlike everything else in this section it is a defect at our end.  Note the
peer's `equerr` at 3429 is ~5000 against ~1600 at 3000, so 3429 may well
measure several dB worse once it does train; that is the first thing to
measure if it starts working.


## 21600 live: the ceiling was a notch filter of our own (2026-08-22)

**Live against the SmartLink rig, at 3000 baud, our receive direction now runs
at 21600 bit/s** -- 166760 symbols at 0.037 from the lattice, 34.9 dB -- where
the previous section concluded 14400 was the rig's hard ceiling.  16800 runs at
0.008-0.013 and 34.4-34.8 dB, at 3000 and at 3429 baud alike.  The peer will
not transmit above 21600 whatever we ask for (24000 and 26400 both come back as
`finally txbitrate 21600`), so that is now the far end's limit rather than ours.

### What was wrong, and how it hid

`v34_update_echo_policy()` puts a **30 Hz notch at our own transmit carrier**
into the receive path, to stop our transmitter's carrier leaking into our
receiver.  Whether it does so was decided by the *separation* of the two
carriers: at 3000 baud, TX high (2000 Hz) against RX low (1800 Hz) is 200 Hz
apart, which passed the `>= 150 Hz` test.  But the received signal is 3000 baud
at 12% excess bandwidth around 1800 Hz -- it occupies **120 to 3480 Hz**, and
2000 Hz is 200 Hz from the middle of it.  Carrier separation is the wrong test;
what matters is whether the notch lands inside the band being received, and at
every symbol rate this pairing reaches, it does.

A 30 Hz notch has an impulse response about 266 samples long -- 114 symbols at
3000 baud, against the 63 symbols the receiver's equalizer spans.  It cannot be
equalized out.  Measured, on otherwise identical back-to-back calls:

| 3000 baud / 9600 | distance to grid | receive SNR |
|---|---|---|
| `ME_V34_ECHO=notch` (the old behaviour) | 0.139 | **17.2 dB** |
| no echo filter (the new default) | 0.002 | **34.8 dB** |

**Two things kept this hidden for a long time.**  The notch is applied *after*
the RX G.711 tap, so every recording of a live call shows a clean signal and
every offline analysis of one exonerates the wire.  And `v34_duplex_test` does
not run the engine at all -- it drives `v34_rx()` directly -- so no loopback
row could ever show it.  The gap between 30 dB on the loopback and 17 dB live
was the whole symptom, and it was attributed to the line.

### The instrument that found it

`tools/v34_channel_bound.py` fits the best possible fractionally-spaced
least-squares equalizer to a recorded tap, given the symbols the receiver
decided, and reports the residual on **held-out** data.  That bounds every
linear receiver, so it separates "the wire cannot carry this" from "our
receiver is not getting it".  On three independent clean calls the wire reads
**38-39 dB held out** -- transparent, at G.711's own ceiling -- against a
receiver delivering 17.  A 20 dB gap on a transparent wire is not a channel
problem, and that is what turned the search inward.

Two traps in using it, both of which produced wrong answers first:

* **The carrier must be right to a fraction of a hertz.**  A fixed equalizer
  cannot undo a rotation that moves: 0.18 Hz -- the difference between 1959 and
  3429 baud's exact 1959.18 -- is 19 degrees across a 1024-symbol window, and
  that unremovable ramp caps the fit at about 20 dB however good the line is.
  It reported 20.7 dB for a line that is really 38.  The tool now sweeps the
  offset out and prints what it found.
* **The decided symbols are the truth only where the call is decoding.**  Run
  on a marginal call (14400 at 0.35 from the lattice) the fit is being asked to
  reproduce our own errors, and it reads 19.5 dB on the same wire that a clean
  9600 call reads 39 on.  Check `distance to grid` first and measure inside a
  healthy stretch.

Those two together are the whole of why the previous section concluded the
opposite, including its "the peer's own 9600 Hz transmit tap measures 21.5 dB":
that measurement used a 14400 call's decisions.  The rig's own transmitter is
fine.

### What else was tested and is still ruled out

Level (`HEADROOM` 0.25 against 1.0: 18.4 against 18.5 dB), the peer's clock
(one slip in a whole call), the rig's loop lowpass (built but not in the path),
and the rig's upstream resampler kernel (`DM_UP_TAPS` 32 against 64: no
change).  All were measured against a notched receiver, so they were measuring
the notch's floor -- but they were also all *negative* results, and none of
them becomes positive now.  `DM_UP_TAPS` and `DM_UP_FC` are plumbed through
`tools/soak/v34_lapm_call.sh` if anyone wants to re-run them honestly.

### Choosing the echo policy

The default is now: if the transmit carrier falls inside the receive band, use
**no echo filter**; otherwise the notch as before.  `ME_V34_ECHO` forces
`none`, `notch` or `canceller` for an A/B.  This rig has no measurable echo at
all -- if it had any, our transmit (which is uncorrelated with the peer's data)
would show up as noise in the RX tap and cap the least-squares bound, and the
bound is 38 dB.  On a real two-wire hybrid there would be echo, and the right
tool there is the adaptive canceller, not a notch inside the receive band; that
pairing is untested here, and the one call that forced `canceller` did not
reach data mode.

### The 3429 row, which needed a fix of its own

3429 baud never trained live and alternated pass/fail by rate parity on the
loopback -- 9600 fail, 12000 pass, 14400 fail, 16800 pass, 19200 fail, 21600
pass, identically in both companding laws.  That is not SNR; it is
deterministic.  The cause is the T/2 eye-phase chooser firing **inside**
10.1.3.6's supervised PP conditioning.  That stage is trained against a known
232-baud sequence with the AGC frozen, so moving the symbol instant part way
through invalidates every sample after the move: measured at 3429, a flip
inside the window takes the PP mean residual from 0.192 to 0.805, after which
TRN never locks (55% ones against 80%), the far end's J never decodes exactly
(d4=3 where a healthy call reads d4=0) and Phase 3 deadlocks with each side
waiting for the other.

The chooser now measures during PP but does not act until conditioning is done.
Scoped to the move rather than the measurement, deliberately: suppressing the
accumulation instead shifts every later window boundary and changed decisions
long after PP, which cost 3000/21600 u-law 37 bit errors.  And scoped to the
window rather than to a vote count: 2743 baud flips just *before* PP starts and
must keep doing so, so requiring two agreeing windows costs 2743/9600 in both
laws.  `ME_V34_EYE_PP_GUARD=0` restores the old behaviour;
`ME_V34_EYE_SELECT` now also accepts `0`, having previously only recognised the
literal `off`.

With that and the echo fix, 3429/16800 runs live at 0.008 from the lattice and
34.5 dB.
