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
