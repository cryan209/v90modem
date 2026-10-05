# V.32bis Compliance Plan

This project needs a standards-compliant ITU-T V.32bis implementation, not a
teaching modem or a "V.32bis-like" approximation. The local normative source is:

- [T-REC-V.32bis-199102-I!!PDF-E.pdf](/Users/scottcryan/v90modem/ITU%20Docs/T-REC-V.32bis-199102-I!!PDF-E.pdf)

## Clause 8 implementation and regression status (2026-09-30)

The reactive SpanDSP dialogue now supports in-band rate renegotiation under
V.32bis 8.1/8.2 and Figure 5. `v32bis_start_rate_renegotiation()` requests an
enabled desired rate; R4 offers that rate and enabled lower rates. R5 offers
the responder's current desired rate and enabled lower rates independently of
R4. Both sides select the highest common rate, transmit whole R words for at
least 64T, send E, and transmit 24T of B1 with zeroed convolutional state.
The scrambler/differential handoff follows 5.3 and 5.3.2. No receiver restart,
pulse-shaper restart, G.711 path change, coefficient change or sample-rate
conversion is involved.

The existing implementation had missed the constellation handoff from E to
B1: the known B1 targets were interpreted as four-point rate-signal targets,
corrupting carrier/equalizer tracking at 12000 and 14400. The receiver now
uses the selected data constellation from the first through the last B1
symbol, while retaining the trained loops. Preamble detection splits a receive
callback at the clamp instant instead of applying the new sink to preceding
samples. Incoming E must name the highest common R4/R5 rate. Completion clears
the initiating role so the next exchange can reverse roles; restart clears the
connection's renegotiation state and count. Tone watchers are initialized even
when a harness starts directly at the conditioning dialogue.

Clause 8 Note 2's no-common-rate case repeats E for at least 64T before
clearing. The cleared modem reports a current rate of zero, incomplete start-up,
and no more transmit samples. Table 5 Note 3's incoming GSTN-cleardown R word
is also recognized in the renegotiation path and exercised with an explicit
on-wire cleardown-word fixture. Foreign implementation interop is untested.

`make v32bis-reneg-test` runs 48 focused duplex rows: all five target rates,
both laws, either initiating role, simultaneous requests, a second exchange
with the role reversed, two simultaneous 7200-to-14400 upgrades, two
no-common-rate cleardowns, two explicit remote-cleardown fixtures, and two
exchanges with clause 6 tones skipped.
Repeated exchanges split RX into 13/67/80 and 80/80-sample callbacks. Invalid,
disabled, pre-data, null-context and overlapping requests are rejected. The
cleardown rows also assert at least 64 transmitted symbol intervals of E.

The PRBS check requires clean data before the procedure and thousands of clean
bits after it. Detection lag lets preamble symbols reach circuit 104 as data;
the checker therefore searches a bounded displacement from its last verified
pre-procedure PRBS state. It requires 64 consecutive matching bits within a
128-bit receive window and then grades every remaining bit. This does not
claim an error-free payload across the clamp interval. All successful focused
rows carry zero errors before and after that bounded resynchronization.

Validation: the focused rows, full C duplex suite, and C infrastructure smoke
test pass. The Python reference's 106 tests and datapump's 17 tests pass. The broader `v32bis-test`
target encounters three existing scrambler-source-parser errors, reproduced
against HEAD: its regex selects an earlier `if (s->calling_party)` block rather
than the initialization assignments. The repository-wide `make test` stops at
plain V.34's 3000/9600/u-law duplex row (no training within 60 seconds).
These failures are outside the changed V.32bis paths.

This is offline coverage, not hardware interoperability. V.8/modem-engine,
V.42 and PTY integration now exist (see "Engine integration" below); full
retrain recovery and foreign-modem V.32 compatibility remain separate work. Historical milestone lists below describe
the original plan rather than overriding this measured status.

## Scope

The implementation target is a duplex modem for GSTN and leased 2-wire circuits
with:

- Echo-cancelled full duplex operation
- 1800 Hz carrier
- 2400 symbols/s
- Supported synchronous rates: 4800, 7200, 9600, 12000, 14400 bit/s
- V.32 compatibility at 4800 and 9600 bit/s
- Startup rate-sequence exchange
- In-band rate change without retrain

## Compliance Strategy

We will build the modem in two layers:

1. A Python reference implementation driven directly from the recommendation.
2. An integration path into the main modem code only after the reference layer
   has stable clause-by-clause tests.

The Python layer is not a shortcut. It is the executable specification for the
standard.

## Current Local References

- `ITU Docs/T-REC-V.32bis-199102-I!!PDF-E.pdf`
- `spandsp-master/src/v32bis.c`
- `spandsp-master/src/v17tx.c`
- `spandsp-master/src/v17rx.c`
- `spandsp-master/src/v17_v32bis_tx_constellation_maps.h`
- `spandsp-master/tests/v32bis_tests.c`

Important note: the bundled SpanDSP tree is useful as an implementation
reference, but `spandsp-master/tests/v32bis_tests.c` explicitly marks its
V.32bis support as work in progress. It is not sufficient evidence of
compliance.

## Native SpanDSP Baseline

The project build now configures the bundled SpanDSP with both
`--enable-v34` and `--enable-v32bis`. Enabling the dormant code exposed and
fixed its stale Godard-header dependency, obsolete echo-canceller API calls,
and incomplete public lifecycle API. `v32bis_spandsp_test` pins honest
initialisation and restart behaviour at 4800, 7200, 9600, 12000 and 14400
bit/s, supported-rate validation, signal-cutoff forwarding, and waveform
output from the inherited V.17 modulation core.

The clause-level startup logic is now native too. SpanDSP builds and validates
Table 5 R1/R2/R3 words, builds and validates the one-rate Table 6 E word,
selects the highest common rate, generates §5.2's 256T S / 16T S-bar / TRN
conditioning stream with the direction-specific scrambler, and encodes and
decodes startup words through Table 1 while carrying the TRN handoff state.
The native regression is pinned bit-for-bit to the Python reference for both
caller and answerer, including final scrambler and differential states.

The symbol-domain seam is connected. `v32bis_prepare_startup_tx()` queues one
§5/§6 burst (conditioning, two identical R words, and E); `v32bis_tx()` feeds
those exact complex states through the existing V.17 1800 Hz/2400-baud pulse
shaper, while `v32bis_rx()` delivers symbols from the shared carrier, timing
and fractionally-spaced-equalizer front end to the V.32bis state machine rather
than running the V.17 fax-training switch. The native regression exercises
that path through both PCMU and PCMA at 8 kHz and requires the complete 1576
symbol burst to cross the seam.

The receive seam is now live rather than a symbol counter. It acquires S
without an oracle by fitting both alternating phases over a 64-symbol window,
detects the S-to-S-bar transition structurally, regenerates the
role-directional 1280T TRN sequence, and uses its known symbols to train the
shared carrier loop and LMS FSE. It then decodes and validates both repeated
Table 5 R words and Table 6 E from the recovered TRN scrambler/differential
state. PCMU and PCMA tests require that blind acquisition complete and that the
received offer and selected rate agree with the transmitted words.

E now hands the recovered scrambler and differential state to the existing
V.17 data decoder, while the transmitter hands its matching state to the V.17
data encoder and switches constellations without a waveform reset.

## The B1 segment, and the seam it exposed

A real 6 B1 marks segment now sits between E and data on both sides: the
transmitter forces mark bits through the ordinary V.17 encoder
(`v32bis_b1_bits_remaining`), and the receiver regenerates the same symbols and
uses them as **supervised** training targets on the data constellation.

The regeneration is exact.  Dumping the transmitter's symbol index against the
receiver's reconstruction gives **0 mismatches in 128 symbols at every one of
the five rates, in both laws** -- so B1 is not a suspect for anything below,
and the training targets are right.

Two defects sat at the handoff, both invisible at 4800 and fatal above it:

- **The trellis emits the symbol from `t - 15`**, so its first 15 symbols of
  output after the handoff are traceback fill.  Those bits must be neither
  delivered to circuit 104 nor **shifted into the descrambler**: the V.32bis
  descrambler is a 23 bit self-synchronizing register, and letting the fill in
  destroys the seed taken from the end of B1.  The symptom was 11 bit errors
  confined to the first 24 bits at 7200.
  Suppressing *more* bits is the wrong fix and was measured: it deletes real
  data and desynchronizes the stream (11 errors becomes 976).
- **`s->diff` is updated from the trellis output on every symbol**, fill
  included, so the differential state seeded from the end of B1 was overwritten
  before the first real symbol used it.  The residue was a single wrong line
  bit, which the descrambler tripled into errors at bits 1, 19 and 24 -- `n`,
  `n+18` and `n+23`, the answerer's two descrambler taps.  Read that signature
  directly: it says one bit, not three.

With both fixed, **4800 and 7200 recover the PRBS with zero errors in both
G.711 laws**, and those two rows are asserted in `make test`.

## All five rates now recover the PRBS without error

9600, 12000 and 14400 used to fail, and the explanation recorded here -- that
the shared V.17 LMS step is unnormalized and its gradient noise (0.42 of a
unit against a half-spacing of 1.0) is survivable by a 4 or 8 point decision
and not by a dense one -- was **wrong**.  All five rates now recover the PRBS
with zero bit errors in both G.711 laws, and all ten rows are asserted in
`make test`.

**The cause was the receive AGC, and it was never latched.**  `v17rx.c` only
updates `agc_scaling` "until we have locked down the setting", the latch being
`agc_scaling_save`, which V.17 sets as it leaves its own training stages.  The
V.32bis path takes the symbol stream over through `symbol_sink` *before* those
stages run, so nothing ever set it and the AGC re-derived its scaling from the
instantaneous power meter on **every T/2 sample for the whole call**.  It is
now latched from inside the V.32bis TRN handler, where S has already run for
256 symbols and the level is settled.  Measured as the rms distance from the
equalizer output to the **transmitter's own** symbol:

    rate     before   after
    7200      0.731   0.326
    9600      0.883   0.260
    12000     1.030   0.812
    14400     1.022   0.232

**How it was found, because three metrics said the opposite first.**  The
receiver's own eye -- distance to the point it *chose* -- read 0.60 at 14400
and 0.73 at 7200, i.e. the failing rate looked better than the error-free one,
because a wrong decision is self-consistent.  Dumping the transmitter's symbol
index against the receiver's (`V32BIS_SYM_DUMP_TX` / `_RX`) showed 53% of
14400's symbols simply wrong, so the decode was not marginal.  Measured against
those true symbols the residual is ~10% of the symbol amplitude at **every**
rate and in every phase, because the constellations are all normalized to the
same mean power -- one impairment, not a constellation effect.

Three candidate mechanisms were then eliminated by measurement, and each is
worth not re-testing: it is **not gradient noise** (annealing the step changes
the floor by a few percent, and below `V32BIS_EQ_SLOW=0.03` the equalizer does
not converge at all -- 8.0, i.e. no equalization -- so the loop was
convergence-limited, not noise-limited); **not timing jitter** (freezing the
Godard loop leaves 7200 at 0.732); and **not linear distortion** (a
least-squares fit of the transmitted symbols to the equalizer output with
taps from -40 to +40 still leaves 0.63, and the neighbouring taps are 0.01).

What settled it was two bounds.  A least-squares fit of the best possible
33/65/129-tap T/2 equalizer to the receiver's **own baseband stream**
(`V32BIS_T2_DUMP`) reaches only 0.665 held out, against the 0.731 the receiver
achieves -- so the equalizer was already near optimal and the damage was
upstream of it.  The same fit against the **transmit audio**
(`V32BIS_AUDIO_DUMP`, mixed to baseband at 1800 Hz, one tap set per
3-symbol phase class) reaches **0.0007** -- so the waveform is clean and any
linear receiver can recover it exactly.  The loss is entirely between the
audio and the equalizer input, which is where the AGC sits.  Read those two
numbers together: either alone says nothing.

Consequences for the tuning recorded above.  The **energy-normalized LMS**
(`V32BIS_NLMS=1`) existed for a residual that was the AGC; with the AGC
latched it is clearly harmful (7952 bit errors against 0) and no
`V32BIS_EQ_FAST` setting betters the plain step, so its "needs its own retune"
status is withdrawn rather than outstanding.  The **anneal** is no longer
load-bearing: every combination of `V32BIS_TRN_FAST` in {0, 80, 160, 320, 640,
1280} with `V32BIS_EQ_SLOW` in {0.1, 0.3, 1.0} is clean on all ten rows.  It is
kept at 160/0.1 because the smallest step (0.05) still costs up to 743 errors.

**Decision-directed equalizer adaption now runs in the data phase.**  V.17
leaves the equalizer frozen after training, which is right for a fax burst on a
static channel and wrong for a V.32bis connection; `tune_equalizer()` was
commented out at its data-mode call site.  It is enabled for the V.32bis path
only (`V32BIS_DATA_EQ=0` disables) and is worth 192 bit errors and one clean
row on its own after the AGC fix.

Diagnostics, all env-gated and all caching their getenv: `V32BIS_B1_SYMBOLS`,
`V32BIS_TRN_FAST`, `V32BIS_EQ_FAST`, `V32BIS_EQ_SLOW`, `V32BIS_NLMS`,
`V32BIS_DATA_EQ`, `V32BIS_DATA_EYE` (the self-consistent eye -- read it only
beside a true-symbol measurement), `V32BIS_SYM_DUMP_TX`/`_RX`,
`V32BIS_T2_DUMP`, `V32BIS_AUDIO_DUMP`, `V32BIS_TIMING_HOLD`,
`V32BIS_CARRIER_HOLD`, `V32BIS_LINEAR` (bypass G.711, to attribute a defect to
the bearer or exonerate it) and `V32BIS_PEAK`.

`v17rx.c` and `v17tx.c` are shared with the V.17 fax modems, so all of the
above is scoped to the V.32bis path and the full suite is green.

Diagnostics, all env-gated: `V32BIS_B1_SYMBOLS` (B1 length; longer does not
help, 256 measured slightly worse), `V32BIS_TRN_FAST`, `V32BIS_EQ_SLOW`,
`V32BIS_NLMS`.

## Clause 6 start-up now runs as a dialogue, and two modems train each other

`v32bis_prepare_startup_tx()` still queues one self-contained burst, which is
what the offline harnesses grade.  Beside it, `v32bis_start_startup()` runs
Figure 3 as the half-duplex dialogue clause 6 actually describes, with each
segment generated only once the event that releases it has arrived:

    call   SILENT -> (R1) S(NT) COND R2... -> (R3) E -> data
    answer COND R1... -> (S) SILENT -> (R2) COND R3... -> (E) E -> data

`v32bis_duplex_test` points a calling and an answering instance at each other
through G.711 in both directions and tells neither what the other supports.
**Eleven rows -- five rate pairings in both laws, plus one with NT and MT set
-- negotiate the right rate and then carry the PRBS in both directions with
zero bit errors**, 37806 to 113610 bits a side.  The rate rows are chosen so
the negotiation has to do real work: 6.1's "R2 shall exclude rates not
appearing in the previously received rate signal R1" is what stops the call
modem's longer list winning, and 6.2's "the data rate selected by R3 shall be
within those indicated by R2" is what picks the single rate.

Three defects came out of it, and none of them is reachable from the
single-burst harness.

- **`v17_tx_restart()` must not be called mid-stream.**  It zeroes the pulse
  shaper history and resets the carrier and baud phases, which is harmless
  before a burst starts and a hole in the middle of one.  The burst path calls
  it before transmitting anything; the dialogue reaches the same code at the E
  handoff, where E sits between the rate signals and B1 with no gap.  The far
  end lost carrier and stopped producing symbols at all.  `v32bis_tx_set_rate()`
  now changes only the constellation.
- **The one-tap channel estimate went stale across the rate signals.**
  `startup_enter_data_rx()` hands it to the FSE at the E handoff, and it was
  last updated during TRN.  In the dialogue a rate signal repeats for well over
  a thousand symbols while the far end works through its own script, so the
  answer modem entered data on an estimate measured about 1800 symbols
  earlier -- **its whole data phase, 56486 and 47131 bit errors at 14400 and
  12000 in u-law, with the other direction clean**.  It is now tracked through
  the rate-signal stages as well.
- **The 4800 bit/s decode tracked nothing at all.**  `decode_baud()`'s
  uncoded branch sliced the symbol and returned without `track_carrier()` or
  `tune_equalizer()`, so at 4800 the whole data phase ran open loop.  That
  survives a fax burst -- and 4800 does not exist in V.17, so this is the
  V.32bis path only -- but over thousands of symbols it collected about 1.5%
  bit errors in bursts, in three of the four 4800 directions and in both laws.

Two of those three present as "the receiver fails" and are transmit-side or
handoff-side, so **read the direction that works as well as the one that does
not**: at 14400 and 12000 the call modem was clean on the identical code path
throughout, and the asymmetry is what pointed at the long wait between TRN
and data rather than at the receiver.

`V32BIS_TRACE=1` prints both scripts into one stream -- phase changes, the S
event, each decoded 16-bit word and the two E words -- which is what makes the
interleaving of the two roles readable.

## The tone phases, and NT and MT measured rather than assumed

`v32bis_start_tones()` runs clause 6 from its beginning, so the round-trip
estimates are produced rather than supplied.  Figure 2-5's carrier states are
four points 90 degrees apart, which is what makes the whole choreography work:
a modem repeating state A puts a pure 1800 Hz tone on the line, one
alternating A and C puts a suppressed-carrier pair at 1800 -/+ 1200 Hz -- the
600 Hz and 3000 Hz 6.1 tells the call modem to look for -- and state C is
state A turned through 180 degrees, so **every transition clause 6 calls a
"phase reversal" is a sign change of the whole waveform**: AA to CC, AC to CA,
and CA back to AC alike.

Each side runs one coherent sliding-window detector per tone it has to watch.
Two disjoint 20-sample windows of the mixed signal are compared, so a reversal
shows as their dot product going negative -- and, because the leading window's
magnitude dips to a minimum exactly when the reversal sits in the middle of
it, **the instant of the reversal is recovered, not just its occurrence**.
That matters: 6.1 and 6.2 do not ask for a reaction, they ask for one 64 +/- 2
symbol intervals later, measured at the line terminals.

`v32bis_duplex_test` grades that directly.  Both ends' pulse shaper delays are
equal, so the gap between their two scheduled transitions, read off their
transmit symbol indices, is the delay the Recommendation measures.  Over three
one-way channel delays:

    one-way   NT           MT           scheduled reversal delay
    0T        128 (128)    65 (64)      64 (64)
    24T       176 (176)    113 (112)    88 (88)
    72T       272 (272)    209 (208)    136 (136)

with the geometry's own expectations in brackets: NT spans two 64T hops plus
the round trip, MT one hop plus the round trip, and the scheduled delay is
64T plus the one way.  **NT and the reversal delay are exact at all three
delays; MT is consistently one symbol high.**  All three tone rows then go on
to negotiate a rate and carry the PRBS in both directions without error.

**The transmit pulse shaper's group delay had to be measured, not derived.**
It is what converts "at the line terminals" into a transmit symbol index, and
it does not cancel in the figures above.  The 9 symbol-spaced taps suggest 4
symbols; the interpolating structure, which indexes the coefficient sets as
`TX_PULSESHAPER_COEFF_SETS - 1 - baud_phase`, makes it 3.  At 4 every row
above is one symbol low and at 5 two symbols low -- inside the +/- 2 the
Recommendation allows, and wrong.  A test that only asserted the tolerance
would have accepted all three.

The V.17 receiver is not fed while the tones are running: there is no
conditioning signal to train on, and 6.1 and 6.2 both have the modem condition
its receiver only once the tones are done.

Still missing: 6.2's V.25 answer sequence (the engine's job, not the modem's)
and V.8/modem-engine/V.42/PTY integration.  Both now exist; see the next
section.

## Engine integration: V.8, V.25 and Annex A automode (2026-10-05)

`modem_engine.c` has `ME_MOD_V32BIS`.  `start_v32bis_training()` creates the
datapump, offers every rate up to `ME_V32BIS_MAX_BPS` (default 14400), and runs
clause 6 from its tone phases with `v32bis_start_tones()`, so NT and MT are
measured on the call.  Completion is polled (`v32bis_startup_complete()`, set
when B1 has been received), and then `on_training_complete()` starts the data
stack exactly as for V.22bis and V.34: V.14, or V.42 LAPM when V.8 negotiated
it, with CONNECT at the negotiated rate.  Circuit 104 stays clamped while the
start-up or a clause 8 renegotiation is running.  `ME_V32BIS=0` removes all of
it.

Three ways in:

- **V.8.**  CM/JM now carry V8_MOD_V32 (Table 4's "V.32/V.32bis duplex", the
  same octet as V.22, so the frame does not change shape; `ME_V8_ADVERTISE_V32=0`
  withdraws it).  `v8_result_handler()` takes V.32 after V.90 and V.34, so
  where those are common nothing changes; `ME_MODE=v32bis` offers only V.32
  and V.22.  Per V.8 8.1.2/8.2.3 both sides are silent 75 ms after CJ and then
  send sigC/sigA, i.e. AA and AC.
- **Answer automode (A.2.2, V.8 8.2.2).**  While V.8 runs, the answer modem
  watches for AA and, on it, stops its answer tone and starts 6.2 at the second
  paragraph.  When V.8 ends without a CM (after the alternate-answer-tone
  retry, so V.8 callers are not affected) it sends USB1 as a V.22bis answer
  modem for Ta = 3000 ms; S1/SB1 inside Ta keeps V.22bis, otherwise it goes to
  6.2's AC.  `ME_V8=0` replaces V.8 with V.25's ANS (2100 Hz, 450 ms
  reversals, 3.3 s, then 75 ms silence) and runs the same automode.
- **Call automode (A.2.1, V.8 8.1.1).**  The call modem watches for AC
  (600 and 3000 Hz) throughout V.8 and takes it as sigA.  If its V.8 fails it
  stays silent and keeps listening until the V.8 phase timeout, because a
  V.32bis automode answer modem only sends AC after Ta.  Answering 1 s of
  plain ANS with AA (A.2.1.3) is ON with `ME_V8=0` and opt-in otherwise
  (`ME_V25_ANS_AA=1`): SpanDSP's V.8 deliberately accepts ANS as ANSam,
  because some networks strip the 15 Hz AM, and turning those calls into AA
  would move them from V.34/V.90 to V.32bis.

The detectors are `v25_automode.c`: 10 ms Goertzel blocks with 600, 1800,
2100 and 3000 Hz on exact 100 Hz bins (so our own ANSam, the loudest thing in
the receive path, cannot leak into the AA bin, and is excluded from the
denominator), V.21 channel 1 at 980/1180 Hz, and SpanDSP's connect-tone
detector for ANS against ANSam.

**Two hazards found by the engine-level test, both fixed.**  (a) A V.8 call
modem that hears our answer tone keeps sending CI and CM, and SpanDSP's V.22bis
answer modem takes that V.21 channel 1 FSK for a low-band carrier -- it
reported CARRIER_UP and then TRAINING_SUCCEEDED, and the call CONNECTed at 2400
bit/s V.22bis to a modem that was trying to do V.8.  During Ta, a V.22bis
status while V.21 channel 1 is on the line is now not S1/SB1.  (b) AA is a pure
1800 Hz line, and some call modems send a continuous 1800 Hz guard tone (the
NZ-market USR this project tests against), which is pure 1800 Hz through V.8's
silent Te before CM.  While V.8 runs, AA must therefore hold for 1 s and the
call must not have shown any V.21 channel 1; a V.32bis caller holds AA from 1 s
into the answer tone until it hears AC (A.2.1.3, 6.1), so a real one loses
nothing.

**Test.**  `v32bis_engine_pair_test <ulaw|alaw> <v8|automode|aa>` runs two whole
engines in two processes (the engine is a process singleton), clocked in
lockstep over a socketpair carrying G.711, each with its own DTE PTY.  It
requires CONNECT 14400 on both PTYs and 150 numbered lines typed at each DTE to
arrive intact and in order at the other.  `v8`: both in `ME_MODE=v32bis`, V.8
selects V.32bis, LAPM, CONNECT at 10.6 s.  `automode`: an ordinary V.8 caller
against a `ME_V8=0` answerer -- ANS, USB1 for Ta (with the caller's CM ignored
as above), AC, then V.32bis, CONNECT at 10.4 s.  `aa`: neither side runs V.8;
AA during ANS, CONNECT at 5.7 s.  All six rows, and `automode` again with the
caller's V.32 bit withdrawn, are in `make test`.  NT/MT come
out 128/65 on this one-frame loop, the same as `v32bis_duplex_test` at zero
delay, which is the check that the engine starts the datapump's transmit and
receive sample clocks on the same tick (they must: clause 6 schedules transmit
symbols off received sample instants, so the engine discards receive and holds
transmit silent until both can start on one block).

**Two defects the engine pair found, neither of them in acquisition
(2026-10-05).**  `automode` used to pass only while the caller's CM carried the
V.32 bit, and `aa` failed about one run in eight under concurrent load.

- *Ta leaked into the DTE.*  During A.2.2's Ta the answerer runs the V.22bis
  answerer, whose receiver demodulates a V.8 caller's CI/CM as low-band data;
  `v22bis_put_bit_cb()` fed those bits to the data stack, which parked the
  bytes in the DTE receive ring until DATA flushed them after CONNECT -- on
  every automode call.  V.22bis 6.3.1.1.2 e) makes the modem "ready to receive
  data" only once trained, so bits now reach circuit 104 only after
  `SIG_STATUS_TRAINING_SUCCEEDED`.  The V.32bis receiver had decoded the
  caller's data correctly all along: with the bit withdrawn the caller's
  transmit audio is byte-identical to the passing run from 6.4 s to 14 s, and
  so is the answerer's data-mode bit stream.  What the bit changed was whether
  the leaked junk contained a 0x00, and the harness counted lines with
  `strchr`/`strlen`.  The harness now parses bounded by the byte count and
  fails on any byte after CONNECT that is not an intact line;
  `ME_V8_ADVERTISE_V32=0` rows for both laws are in `make test`.
- *Clause 8 found its preamble in data.*  8.2's watch compared a 20-sample
  coherent tone sum against a flat 6x the received rms, on the belief that
  data gives 3x per line.  A Rayleigh magnitude averages sqrt(pi*20/4) = 4x,
  and the call modem sums AC's two lines, so data averaged 7.3 and could hold
  above 6 for the 133 samples detection needs.  Whether it did depended on
  the bits, and so on PTY timing.  The threshold is now 0.7 of a clean
  preamble's 20*sqrt(N/2) (14 with two lines, 9.9 with one); over 56 s of the
  engine pair's data the longest run above it is 40 samples.  Detection
  timing then became honest -- ~46T into the 56T head instead of early on a
  run that data had started -- and that exposed the data decoder's carrier
  frequency walking on the preamble's wrong decisions: the call modem
  responding to a second, answer-initiated renegotiation at 12000 came back
  white.  The watch restores `carrier_phase_rate` to its value at the start
  of the run; restoring the equalizer instead did not help.  160 concurrent
  engine-pair runs, 0 failures (9 of 48 before).

**Checked unchanged:** engine replays of a V.90 answer call
(`goal-matrix-115515Z/rate24000-r1`), a plain V.34 answer call
(`v34-21600-20260822T-c65`) and a V.90-to-V.34 fallback dial
(`rf-tower-fb-6`), old binary against new, differ only in the V.32 bit of our
CM/JM and the log line naming it.

**Not done:** USB1 heard by a call modem AFTER it has started AA (A.2.1.3's
V.22bis branch, with its >800 ms ANS rule) -- A.2.1.2 itself is done, see
below; V.32bis clause 7 retrains from the engine; and any hardware
interop.  Clause 8 renegotiations by the far end are followed (the V.14 rate
is updated) but the engine never initiates one.

## USB1 on the calling side, and a real pre-V.8 V.22bis modem (2026-10-05)

A.2.1.2 is implemented.  `v25_automode.c` gained a USB1 detector: USB1 is
unscrambled binary 1 at 1200 bit/s in V.22's high channel, dibit 11 is a 270
degree step per 600 baud symbol (V.22 Table 2), so it is a pure line at
2400 - 150 = **2250 Hz** -- the same "exactly 2250 Hz tone" the RasFinder notes
record when that peer abandons V.8.  100 ms of it holding 70% of the band,
with the 1800/550 Hz guard tones left out of the denominator.  The call
modem, in V.8 or listening after V.8 failed, then: with V.32bis on, goes
silent (V.8's CM is in the low band the V.22bis answerer watches for S1/SB1)
and waits A.2.1.2's Tc (3200 ms, > 3100) for AC, since an A.2.2 automode
answerer sends USB1 for Ta = 3000 ms before AC; with V.32bis off, starts the
V.22bis call modem at once, whose own 155 ms detection and 456 ms wait
(V.22bis 6.3.1.1.1) follow.  `ME_V22_LEGACY=0` withdraws both directions.

**`ME_V8=0` did nothing in V.22 mode.**  The answer side's V.25 ANS was
started only when V.32bis was enabled, so `ME_MODE=v22 ME_V8=0` ran V.8, and
the two `make test` rows billed as "legacy V.22bis" negotiated V.22bis
through V.8 at both ends.  With V.22 alone the answerer now sends V.25 ANS,
75 ms of silence and USB1 with no Ta (V.22bis 6.3.1.2.1); the V.32bis AA/AC
watches are gated on V.32bis being enabled.  Rows: our V.8 caller (default
and V.22-only) against that answerer, both laws; the legacy caller against
our default V.8 answerer (which reaches USB1 only after retrying V.8 with its
second answer tone, ~12 s -- inside any real caller's S7, but slow); legacy
against legacy.

## Against slmodemd: the first foreign V.32bis peer (2026-10-05)

Everything above was measured between two copies of this modem, so a
convention both ends got wrong the same way could not show.  The SmartLink
soft modem (`slmodemd` with its `dsplibs.o` DSP, as packaged in AonCyberLabs'
D-Modem) was put on a line to this engine with no SIP:
`audio_sock_modem` runs the engine on a raw G.711 socket, and
`rig/slm_bridge/slm_bridge.c` is the `slmodemd -e` program that converts
slmodemd's 9600 Hz linear socket to it (windowed-sinc 6/5 in both directions,
real time, taps).  `tools/slm_local_pair.py` places one call and grades numbered
lines both ways.  Every finding below was settled by demodulating the line
taps independently of this receiver (`v32bis` TRN, R, E and B1 decoded from the
spec's own definitions; slmodemd's TRN matches 5.2.3's GPA and GPC vectors in
1200 of 1200 symbols, which is what calibrates the demodulator).

Seven defects, all ours and all invisible to the self-tests:

- **Table 5's sync bits.**  B7, B11 and B15 are all 1 and 5.3.1 detects on
  B0-B3, B7, B11, B15; the code sent and required B11 = B15 = 0.  slmodemd's
  R1 is 0x9ff0 and ours was 0x17f0.  `V32BIS_RATE_FIXED_BITS` 0x8990,
  `V32BIS_RATE_SYNC_VALUE` 0x8880 (the K56flex firmware's report header check in
  `k56flex.c` already used the same 0x888f/0x8880).
- **9600 and 7200 swapped.**  B6 = 9600 and B9 = 7200; the rate masks double as
  the word's bits and had them the other way round, so a V.32bis peer read our
  9600 as 7200.  `V32BIS_RATE_9600` is now 0x0040, `V32BIS_RATE_7200` 0x0200.
- **E's sync test included B4, B8, B13 and B14.**  At 4800 slmodemd sends
  B8 = 0 (Note 1's V.32 interworking, E = 0x88bf) and Note 2 says B13/B14 are
  ignored on reception.  E now matches B0-B3 = 1 and B7, B11, B15 = 1 only.
- **The rate signal was reseeded every word.**  5.3 scrambles and
  differentially encodes it as one stream (the "ITU-oriented policy" of
  reseeding from the end of TRN, so repeated words came out as identical
  symbols, was an inference).  slmodemd's R1 descrambled continuously reads as
  1573 identical valid words; a continuous descrambler fed ours reads a stable
  non-Table-5 pattern, and slmodemd never answered it.  The transmitter, the
  burst path and the receiver all carry the state through every word and E.
- **TRN was assumed to be exactly 1280.**  5.2.3 allows 1280 to 8192 and
  slmodemd sends 5500-8200.  The receiver framed the 8 symbols after its own
  1280 as a rate word and went back to hunting for S.  After 1280 it now decodes
  continuously and slides until two identical valid sequences end on a symbol.
  It must NOT train the equalizer decision-directed through that stretch:
  measured, that left 9600 data at 0.68 from the constellation (white) against
  0.13 and every line intact without it.
- **B and D were swapped.**  Figure 2-5 puts D = 10 at (-2,6) and B = 01 at
  (2,-6); the enum had them the other way, so S went out as A/D and S-bar as
  C/B -- the same tones, so nothing listening for S alone noticed.  S
  acquisition fitted the A/D model too, and against slmodemd's (correct) ABAB it
  locked 90 degrees off and spent the first ~128 TRN symbols walking the one-tap
  gain back.  slmodemd itself sends ABAB / CDCD / CCCCCCCCCAAACCC.
- **The call modem's 600/3000 Hz detectors took V.25 ANS for AC.**  The 20
  sample windows leak 2100 Hz at ~1.4 x rms, the presence test was only relative
  to its own peak, and ANS's phase reversals read as reversals in "the tone"
  (NT = 12 symbols).  A line of AC must now stand at half its ideal W/2 x rms.

And one convention the Recommendation leaves open: where the trellis rates'
differential encoder starts at B1.  Only the convolutional encoder's delay
elements are zeroed (6.1, 6.2; V.32 Figure 2 draws the two encoders apart).
This modem continues from E's final symbol; slmodemd starts from 00 (its B1
matches that 128/128 and ours 0/128).  Our receiver trains on B1 as known data,
so it now generates both readings, votes over the first 16 symbols and keeps the
winner.  slmodemd's receiver does not care which we send (measured both ways),
so the transmitter is unchanged.

**Results** (`tools/slm_local_pair.py`, V.42 off in slmodemd with `AT\N0`, our
`ME_V8=0`, 150 numbered lines each way, 6 dB of loss into slmodemd -- see below):

- slmodemd calling, we answer: 4800 / 7200 / 9600 / 12000 / 14400 all CONNECT
  at the requested rate; slmodemd -> our DTE carries 150/150 at every rate in
  u-law (A-law 7200 failed once, both ways); our DTE -> slmodemd is 150/150 at
  4800 and 9600 and intermittent at 12000 and 14400, where slmodemd's receiver
  reports "SNR drop" a second or two into data and retrains.
- We call, slmodemd answers: **fails at every rate with the defaults**, for two
  reasons, both open.  Our clause 6 Note 3 echo canceller training sequence
  (after S for NT) is not tolerated by slmodemd as answer modem at any length
  (256 to 2048 symbols); with `V32BIS_EC_TRAIN=0`, 4800 passes 150/150 both
  ways but 7200 and above still fail in data with slmodemd's SNR monitor at
  6-8 dB.  That second failure needs BOTH our S and S-bar to be correct: with
  either one mirrored back (A/D S, or C/B S-bar) 9600 passes 150/150.
  slmodemd answer-mode receiver behaviour we cannot see into; TRN length (to
  8000), a near-end echo of -20/-30 dB and the B1 convention were all tried and
  change nothing.
- slmodemd overloads at 0 dB of loss: at our -11 dBm0 (inside V.2's -9 dBm0) its
  receiver reports SNR 14-16 dB and retrains at 12000 and 14400; 6 dB of loss
  fixes it, so the rig defaults to that (`--slm-rx-gain-db`).

Also from this session's offline harness: one informational row of the
`v32bis_duplex_test` hybrid sweep (31.0 dB, 80 samples, canceller OFF) went from
pass to fail; every canceller-on row still passes and the suite still exits 0.
The Python reference (`tools/v32bis_ref`) has since been brought to the same
conventions -- Table 5's sync bits, the B/D labels, the 9600/7200 masks, E's
sync test and the continuous rate signal -- and its golden vectors regenerated
and cross-checked against `v32bis_spandsp_test.c` (see "Startup Handoff
Status").

## The near end echo canceller

V.32bis is full duplex on one pair, so each modem's own transmit returns
through its own hybrid into its own receiver.  The canceller is now in the
sample path: `v32bis_tx()` queues every sample it puts on the line into a
transmit reference FIFO, and `v32bis_rx()` consumes one reference sample per
received sample through `modem_echo_can_update()` before anything else sees
the block -- the clause 6 tone detectors and the V.17 receiver alike.
`V32BIS_ECHO_CAN=0` takes it out.

It does not adapt during the tone phases.  `modem_echo.c`'s own documentation
is explicit that LMS adaption "can go seriously wrong" on a highly
correlative transmit signal, and clause 6's tones are the worst case there
is: state A repeated is a pure 1800 Hz tone and alternating A and C is a pair
of pure tones.  The estimate is still subtracted throughout; only the
adaption stands down.

**The vendored canceller's adaption step was broken, and that is worth more
than the wiring.**  `modem_echo_can_update()` computed a transmit power
estimate, never used it, and applied a hardcoded `shift = 1` -- an
unnormalised LMS whose loop gain scales with the signal.  At modem levels
that is a step of about mu = 1, so it diverges rather than converging.  It is
upstream SpanDSP's own long-standing defect, not a local edit, and it is the
mechanism behind the V.90 note that the same module "was not cancelling an
echo -- it was adding one", reading `pre_rms=0 post_rms=159` on digital
silence.  The step is now normalised by the transmit power the code already
maintained:

    shift = log2(N) + log2(P) - 30 - log2(mu)

for N taps at mean square transmit power P.  **Count both `>>15` steps.**  A
tap is applied as `fir_taps32[i] >> 15` and is itself Q15, so one sample moves
tap i by `x[i]*e/2^(shift + 30)` in tap units; dropping one of the two shifts
puts the step 32768x out, which is exactly the intermediate value this work
went through -- it looked like an improvement only because being far too
small left the filter effectively frozen.

**The step has to be very small, because a V.32bis modem is in double talk for
the whole call.**  The adaption's error term is dominated by the far end
signal, which is uncorrelated with the reference and is typically well above
the echo, and an LMS loop injects misadjustment noise of about mu/2 of that
interference.  At mu = 1/16 the canceller removed 11.6 dB of the received
power on a bearer whose echo was 37 dB down -- adding far more than it took
away -- and every hybrid row failed with it in and passed with it out.  The
default is mu = 1/65536 (`V32BIS_ECHO_MU` is -log2 of it).

`v32bis_duplex_test` now puts a three-tap near end hybrid on each side --
three taps rather than one, so a canceller cannot pass by being a pure delay
and gain -- and sweeps its return loss against the channel delay, with the
canceller in and out.  Rows passing, in against out:

    mu_shift   31 dB return loss   37 dB
    12         0/3 vs 1/3          0/3 vs 3/3
    16         3/3 vs 1/3          3/3 vs 3/3
    20         2/3 vs 1/3          3/3 vs 3/3

A peak rather than a plateau, which is what says the adaption is doing the
work rather than the 31 dB result being an acquisition coin flip.  The 31 dB
rows are asserted; the ones below are printed and not graded.

## Clause 6 Note 3: the echo canceller training sequence

Note 3 is what makes the canceller converge, and without it the canceller was
useless below about 30 dB of hybrid return loss.  The reason is structural: a
V.32bis modem is in double talk for essentially the whole call, so the
adaption's error term is dominated by the far end signal, which is
uncorrelated with the reference and is typically above the echo.  There is
no step size that fixes that.  What fixes it is an interval in which the far
end is silent, and clause 6 provides exactly two.

**Read where the Recommendation puts its three "(see Note 3 below)"
references, because they are the whole design.**  6.1: "After this period
[NT] has expired (see Note 3 below), the modem shall transmit the receiver
conditioning signal" -- and the answer modem ceased transmitting when it
detected that NT-long S, so it is silent.  6.2: "cease transmitting for a
period of 16 symbol intervals and then (see Note 3 below) transmit the
receiver conditioning signal" -- and the call modem has been silent since its
own second phase reversal.  6.2 again, on the receive side: "if an incoming S
sequence persists, or when an S sequence reappears (see Note 3 below)", which
is the answer modem being told to tolerate the call modem's optional sequence
arriving before its S.  So both training windows are points at which the far
end is quiet **because of what the tone phases and the rate signals already
made it do**, and the third reference is the interop obligation.

`V32BIS_TX_PHASE_EC_TRAIN` sits at those two points.  `V32BIS_EC_TRAIN` sets
its length in symbol intervals, 0 for none -- Note 3 makes the sequence
optional, so 0 is a conformant modem and the far end must cope either way,
which `v32bis_duplex_test` covers with an asymmetric row each way (one end
sending none against the other sending 2048, and one sending the full 8192
against the other sending none).  The default is 2048, which at 2400 baud is
853 ms: Note 4 warns that a G.165 network echo canceller needs 650 ms.

**The signal.**  Note 3 says it "need not be defined in detail", then
constrains it three ways, and all three are met and checked rather than
asserted.  It must keep energy on the line, to hold network echo control
devices disabled.  It must not exceed 8192 symbol intervals.  And its power
in the three 200 Hz bands centred at 600, 1800 and 3000 Hz, summed, must be
at least 1 dB below the power in the rest of the bandwidth, averaged over any
6 ms interval.  Those three frequencies are the carrier and the two lines S
and S-bar put on the line at 1800 +/- 1200 Hz, so what is being asked for is
a signal with no spectral lines -- one the far end cannot mistake for Segment
1 or 2 of the 5.2 conditioning signal.  Scrambled data on the 4 point
training constellation has none; it is the same construction as TRN, which
Note 3's own first sentence says is suitable.  Measured on the transmit
audio with a 48 sample (6 ms) Goertzel at each of the three frequencies, the
worst window has **4.8 dB** of margin against the 1 dB required, and the
Goertzel's main lobe is wider than 200 Hz so that reading is conservative.

**Windowing that measurement is the trap.**  The transmit phase changes part
way through a block, and the block it changes in carries either silence or
the conditioning signal's S -- whose entire content is lines at exactly the
three frequencies in question.  Measuring a boundary block reported 0.0 dB on
a sequence that is actually 4.8 dB clear, which reads as a conformance
failure and is an instrument failure.  Only blocks that both began and ended
inside the sequence are kept.

**The step is switched by a tag that travels with the reference sample, not
by the wall clock.**  What matters is whether the far end was silent when the
sample that is echoing *now* went out, so each entry in the transmit
reference FIFO carries a bit saying so, and the whole of the echo's delay
spread is covered without guessing at the round trip.  The step is 1/8 while
that bit is set and 1/65536 otherwise.

**The tag is confirmed against the line, and taking it at face value is what
made this dangerous.**  It says "we believe clause 6 has the far end silent
here"; if the belief is wrong the fast step is applied to an error term that
is mostly far end signal, which diverges.  It was wrong immediately: with the
tone phases skipped, NT is zero, so 6.1's S is zero symbols long, the answer
modem never ceases, and the call modem transmits its training sequence into a
full strength R1.  Every hybrid row then failed with the canceller in and
passed with it out.  A hybrid cannot return more than a few dB below what was
sent, so received power more than 6 dB above the reference power now vetoes
the fast step whatever the script believes.

**Result.**  The canceller's estimate now tracks the injected hybrid to a
tenth of a dB -- -13.0 dB at 12.6 dB return loss, -18.8 at 18.6, -24.8 at
24.6, -31.2 at 31.0 -- and over the sweep of return loss against channel
delay the canceller carries the call in **15 of 15** rows against **6 of 15**
without it, with the 12.6 to 24.6 dB rows failing 9 of 9 with the canceller
out.  Those are the rows the sweep asserts.

**One defect fell out of the hybrid rows and it is not the canceller's.**
6.1 has the call modem "conditioned to detect ... one of two incoming tones
at frequencies 600 +/- 7 Hz and 3000 +/- 7 Hz", and only "subsequently to
detect a phase reversal in that tone".  The dwell that phrasing implies was
not there -- 6.2 spells one out for the answer modem's 1800 Hz tone and the
call side had none -- so over a hybrid this modem's own state A leaked enough
into the 600 and 3000 Hz detectors to be taken for the far end's tone, and
then for a reversal in it, **before the far end's tone had arrived**: NT came
out at 151 and MT at -38.  It failed with the canceller out as well as in,
which is what said it was not the canceller's.  The call side now requires
the same 64 symbol periods of presence, which cannot miss a real reversal
because 6.2 has the answer modem send at least 128 symbol intervals of
alternating A and C before its first one.  The side effect is that NT is now
one symbol high rather than exact, as MT already was; the differences between
delays stay exact, so it is the origin that moved, and both remain inside
6.1/6.2's +/- 2.

`make v32bis-test` runs the native smoke test plus all Python reference,
SpanDSP-comparison, waveform, and datapump tests.

## Reference vs SpanDSP

The Python reference layer follows the ITU-T Recommendation by default.

- Default startup modelling uses the ITU-oriented handoff policy:
  differential state is derived from the final transmitted `TRN` symbol,
  scrambler continuity is carried from the end of `TRN`, and the
  convolutional state is explicitly zeroed at `B1` entry.
- The normal-startup scrambler carry-forward is still an interoperability
  assumption, because the Recommendation is less explicit here than it is for
  `TRN` initialization and renegotiation.
- The local SpanDSP comparison harness is still valuable, but it should be read
  as an implementation cross-check, not as the normative source.
- When the comparison harness reports a better match with a SpanDSP-specific
  startup seed, that does not override the Recommendation. It means SpanDSP is
  making additional startup-state choices beyond the explicitly modelled ITU
  handoff path.

## Startup Handoff Status

The startup path now has a sharper split between what the Recommendation says
explicitly and what the reference model still has to infer.

Explicitly anchored in the Recommendation:

- `TRN` starts with the scrambler register at zero.
- Startup differential state is derived from the final transmitted `TRN`
  symbol.
- The rate signal is one continuously scrambled, differentially encoded
  stream (5.3): repeated `R` words and `E` run on from one another, and
  nothing resets between `TRN`, `R`, `E` and `B1`.
- The convolutional/trellis state is explicitly zero at `B1` entry.
- Renegotiation startup is a separate case with an explicit scrambler reset.

Reference policy (C datapump and `tools/v32bis_ref` agree):

- Normal startup carries scrambler and differential state forward from the
  end of `TRN` into the first `R` word, then from each word into the next and
  into `E` -- the earlier policy of reseeding every word (and `E`) from the
  end of `TRN` was an inference, and slmodemd never answered it (see "Against
  slmodemd" above).
- `B1` begins with the scrambler/differential state `E` left plus zero
  convolution state.  Where the differential encoder starts at `B1` is the one
  convention left open (slmodemd starts it from 00; see above).
- Labels follow Figure 2-5: `A` = 00 at (-6,-2), `D` = 10 at (-2,6), `B` = 01
  at (2,-6), `C` = 11 at (6,2), i.e. 4800 table indices 0, 1, 2, 3.  `S` =
  ABAB and `S-bar` = CDCD each alternate between points 180 degrees apart.
- Table 5's sync bits are B0-B3 = 0 and B7, B11, B15 = 1 (every rate: 0x9ff0);
  Table 6's `E` is recognised on B0-B3, B7, B11, B15 = 1 only (Notes 1, 2).
  The reference's golden vectors (`test_rate_signal_vectors_match_c_datapump`)
  are the C datapump's `v32bis_spandsp_test.c` ones.

What remains inferred rather than fully proven from the Recommendation text:

- Whether every implementation should preserve exactly the same effective seed
  that SpanDSP uses at the first real post-training data symbol.

Current implementation evidence:

- The Python reference path is internally consistent with the ITU-oriented
  startup model and the local receiver/frontend tests.
- The SpanDSP comparison harness shows that the aligned datapump core matches
  SpanDSP once the startup seed is forced to SpanDSP's effective state.
- The current harness therefore distinguishes two questions:
  normative startup modelling and implementation-parity startup seeding.

Practical reading of the local comparison report:

- `seed_summary` tells us whether the ITU-oriented Python startup state and the
  SpanDSP post-training state agree on scrambler, differential, and
  convolutional state.
- `python_spec_exact_match_prefix_symbols` tells us how many initial startup
  symbols match under the ITU-oriented seed.
- `python_spandsp_seed_exact_match_prefix_symbols` tells us how many initial
  startup symbols match when the Python path is forced to SpanDSP's effective
  seed.
- If the SpanDSP-seeded path matches while the ITU-oriented path diverges at
  `scrambled_bits`, the remaining difference is startup-state policy, not the
  datapump core.

Current project stance:

- The Python default remains the ITU-oriented reference path.
- SpanDSP-seeded startup is a diagnostic mode, not the normative default.
- Any future integration into the main modem path should preserve this
  distinction explicitly rather than silently adopting SpanDSP startup seeding
  as the standard.

Blind (oracle-free) startup word decoding:

- The first word of a rate signal is encoded from the TRN-derived state
  (differential state from the final TRN symbol, scrambler register carried
  from the end of TRN); every later word and E continue that stream.
- The blind datapump receiver therefore seeds the first word's decode with
  the recovered constellation state at the end of the conditioning segment
  (the TRN state labels map to the same points as `Q0..Q3`, so the recovered
  state index is the differential seed directly) and the nominal 1280-symbol
  TRN-end scrambler register, and decodes E through the 16 symbols of R3
  before it.  The logical receivers (`receiver.py`, `v32_receiver.py`) decode
  each Q run as one stream and slide a 16-bit window over the bits.
- Decoding isolated 8-symbol windows from a zero (or TRN-end) scrambler
  register is wrong: the 23-bit self-synchronizing descrambler never sees
  enough history inside a single 16-bit word to recover.  The flip side is
  that a slip or erasure costs 23 bits, so recovering a corrupted R needs a
  further two clean repetitions after it.
- An E detection is only accepted when its rate field decodes to exactly one
  rate, mirroring the logical receiver's guard.

## Work Breakdown

### Phase 1: Spec-Locked Tables and Bit-Level Logic

Status: complete in the Python oracle; native startup/data mappings covered

- Encode the supported bit rates and bits/symbol relationships.
- Implement the differential quadrant encoder from Table 1/V.32bis.
- Implement the trellis/convolutional encoder used by the coded rates.
- Add exact constellation tables for the V.32bis data modes.
- Add unit tests tied to the recommendation tables and figures.

Exit criteria:

- The Python reference encoder emits the correct coded symbol indices for all
  supported rates.
- Unit tests validate the differential encoder truth table and known
  constellation points.

### Phase 2: Scrambling, Framing, and Rate Sequences

Status: complete, including the reactive clause 6 ordering and the tone
phases that produce NT and MT

- Implement transmit and receive scramblers with caller/answerer directionality.
- Implement startup rate-sequence exchange.
- Add explicit tests for rate-sequence generation and parsing.

Exit criteria:

- Both sides negotiate a common rate in an offline harness. Met: eleven
  duplex rows negotiate and then carry data without error.
- Bit-level traces match the intended startup flow. Met, via `V32BIS_TRACE`.

### Phase 3: Passband Modulation and Receiver Front End

Status: zero-BER at all five rates over the G.711 loopback

- Pulse shaping
- Carrier generation and recovery
- Symbol timing recovery
- Adaptive equalization

Exit criteria:

- Offline loopback passes at all supported rates over a simulated clean
  channel. Met for the clean G.711 loopback; no impaired-channel model yet.

### Phase 4: Full-Duplex Echo-Cancelled Operation

Status: the whole of clause 6, tone phases included, completes and exchanges
data over a clean bearer with a real channel delay (`v32bis_duplex_test`),
and over a 2-wire hybrid down to 12.6 dB return loss with the echo canceller
and clause 6 Note 3's training sequence, where without the canceller the call
fails from about 30 dB down

- Echo canceller. Done, with Note 3's training sequence.
- Duplex startup sequencing
- Robustness under realistic line models

Exit criteria:

- Back-to-back duplex simulation completes training and exchanges data. Met
  for a clean bearer with a real one-way channel delay, and for a 2-wire
  hybrid down to 12.6 dB return loss, which is the worst modelled.

### Phase 5: Rate Renegotiation and V.32 Interop Boundaries

Status: pending

- In-band rate changes without retrain
- V.32-compatible operation at 4800 and 9600
- Compliance regression suite

Exit criteria:

- Automated regressions cover rate changes and V.32 compatibility paths.

## Immediate Deliverables

The first Python reference package should include:

- Exact supported-rate definitions
- Differential encoder
- Trellis encoder state machine
- Symbol-index generator for 4800/7200/9600/12000/14400
- Exact constellation coordinates for the lower-rate modes first
- Unit tests

## Known Gaps After This First Step

The initial reference package will not yet include:

- Startup training waveforms
- Passband audio generation
- Echo cancellation
- Full receive-side trellis decoding
- Rate-sequence exchange
- Rate renegotiation

Those are deliberate next milestones, not omissions in the final target.
