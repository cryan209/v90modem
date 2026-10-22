# RasFinder Phase 4: CRC-anchored measurements, 2026-09-28

## 2026-10-16 (late): what fires the "upstream discontinuity resync" -- four causes, one fixed

The 13 data-mode calls in `opt-*`/`opt2-*` that ended in "V.90 upstream
abrupt discontinuity: 3 local timing searches failed" (V.90 9.6 / V.34 11.6
resync request) do not share a cause.  Measured off the taps, receiver-independent
first (`tools/measure_timing_slips.py`, `tools/echo_lag_track.py`, a
least-squares equalizer refitted outside the receiver on the same
post-echo-canceller samples), then with the receiver's own symbols:

| cause | calls | ours? |
| --- | --- | --- |
| FSE's newest tap read an unreceived sample across a timing step | opt-12, opt-6, opt-9, opt2-1, opt2-4, opt2-7 | ours, **fixed (5eec76d5)** |
| slow equalizer walk-off from ~18 s into data, 6311 only | opt-1, opt-4, opt-7 | ours, **open** |
| our echo moved 5 ms (the AudioCodes re-centred its playout) | opt-3, opt2-5, opt2-6 | trigger is probably our send timing; open |
| the peer started a retrain itself | opt-8 | no |

**No call had a slip or level change on the wire.**  The peer's symbol-rate
line holds its phase to +/-10 degrees through every resync instant, and the
received level and spectrum are flat (+/-0.6 dB per 250 Hz band) between early
data mode and the collapse.  Our transmit tap is unchanged until our own Rd.

**First, the replay had to be made to agree with the call.**  `--sync` replays
diverged from live at B1: on opt2-5 the 4-point training failed (0.0148)
where the live call measured 0.00024.  Two reasons: the live RX arrives as two
80-sample calls per 20 ms tick (`--split`, which also puts the echo
canceller's bulk delay on the live 1923 rather than 1843), and from TRN2d on
the replay's own transmit parts from the live one (`v34_tx.raw` differs from
sample 27999), so the canceller was subtracting an echo that was never on the
wire.  **`v90_engine_replay --echo-ref`** (new) feeds the canceller from the
recorded `live-tx.g711` instead.  With `--dial --sync --split --echo-ref` all
six calls in question fire their resync within 20 ms of the live instant, and
the unfixed build reproduces 11 of the 13 live resyncs across the 19 taps.

**The fixed defect.**  The RasFinder's clock is ~6 ppm off ours (Gardner
`freq` -2e-5 T/3 per symbol; the independent fit sees the same -0.058 T/3/s
delay drift, and its -3.9 deg/s phase drift is exactly that delay at the
1829 Hz carrier), so the timing loop takes a whole T/3 step every 17.5 s
(56000 symbols).  The step itself is seamless by construction -- `acc` -0.503
at index n reads the same samples as +0.497 at n-1 (`V90_T3_SYMBOL_PROBE`
either side of opt-1's step) -- but the receiver emits each symbol the moment
its 21st tap has arrived (`next_symbol - pre + 21 == raw_count` in the probe),
and with a positive fraction `v90_t3_raw_get_frac()` blends that tap with the
NEXT sample, which `v90_t3_raw_get()` returns as zero.  So on 6311 (drift
negative) each step flipped the fraction to +0.5 and halved the newest tap at
a stroke: lattice error 0.027 -> 0.089 at the first step, 0.035 -> 0.100 at
the second.  On 6313 (opt-12, drift positive) the fraction crept up to +0.5
over 17 s, the equalizer adapted around the growing corruption, and the step
that removed it took the eye from 0.032 to 0.569 in one symbol.  The signal
was clean across all of it: the outside equalizer refitted per 1200 symbols
holds 0.016-0.05.  Fix: the emitter waits for the one sample interpolation
needs; the slip scorer skips candidates that reach past the ring.  After it,
every step is invisible (0.027/0.027, 0.034/0.035, 0.019/0.021 either side)
and opt-12 holds 0.020 for the rest of its valid recording.

Across the 13 calls with a live resync (replay valid only up to the live
resync, after which the peer is answering our Rd):

| build | holds past its live resync | same instant as live | fires earlier |
| --- | --- | --- | --- |
| before | 1 | 11 | 1 |
| 5eec76d5 | 6 | 5 | 2 (opt-1 by 0.2 s, opt-7 by 1.6 s: the walk-off below) |

**Open: the 6311 walk-off.**  On every 6311 call the receiver's eye rises
from 0.026 to ~0.04 from about 18 s into data mode, then walks off over
1.5-2 s (0.05 -> 0.6) around 28-30 s.  The wire does not change (the outside
refit holds 0.041-0.050 from 3.8 s to 27.5 s), and nothing in the receiver
visibly moves: FSE centroid 10.46-10.54 throughout, Gardner accumulator
tracking cleanly, the output is not rotated or scaled (best rotation 0, gain
0.99-1.00 until 0.25), no restore or blind recovery before it.  The DD-LMS
step governs it: 0.02 (default) collapses there, 0.05-0.1 at ~22 s, 0.01 and
0.005 hold to the end of every valid window (8 and 9 of the 13 instead of 6,
none earlier) but their eye still climbs to 0.08-0.1, so a smaller step only
postpones it.  Left at 0.02 here: changing it needs the upstream rate matrix
(`tools/v90_upstream_bench.sh`), not 6311 alone.

**Open: echo moves.**  On opt2-5, opt2-6 and opt-3 the eye goes white in one
symbol (0.026 -> 0.6) with the wire's level, spectrum and timing unchanged,
and `echo_lag_track.py` shows our echo stepping 40 samples (5 ms) at that
instant and back 0.25-0.8 s later (opt2-5: 2008 -> 1968 at 26.9 s, back at
27.7 s; opt2-6: 1967 -> 1926 at 29.8 s; opt-3: 1928 -> 1968 at 57.85 s).  No
shift, rotation or refit of an outside equalizer recovers the symbols, as
expected for ~-21 dB of uncancelled echo against a 35 dB eye.  The canceller's
tracker logs the move only after the resync has already fired (60 ms after).
Each move follows late RTP sends from us (opt-3: one 22 ms late send 170 ms
before; opt2-5/-6: a burst 10-16 ms late 0.3-0.8 s before), but a similar
burst ~0.5 s into data mode appears on 17 of the 19 calls and only these
three moved, so send timing is a likely trigger and not shown to be the
cause.  Receive-side compute is not it: data-mode callbacks are 2.1 ms at most
in the replay (`ME_SLOW_CALLBACK_US=2000`).

**opt-8** is the peer's: the symbol power jumps (lattice error 1.3) and the
peer's retrain is detected 75 ms after our resync request.

Live batch, mini, 5eec76d5: LIVE_RESULTS_PENDING

## 2026-10-16 (night): the post-B1 callbacks were unoptimised code; LAPM 4/20 -> 15/19

**SpanDSP has been built at -O0 all along.**  The makefile configures it
with `CFLAGS="$(TIFF_CFLAGS) $(JPEG_CFLAGS)"`, which replaces autoconf's
default `-g -O2`, so every DSP loop in `libspandsp.a` is unoptimised.  The
post-B1 acquisition steps (one template per packet since 857af503) each cost
4.5-6.5 ms on the dev Mac free-running, and the mini is not slower at all
(same M4, same 2.4 s for a `--fast` replay): what costs 3-4x live is
real-time pacing, a thread that wakes every 20 ms for a few ms runs on a
clocked-down core.  The real-time replay here showed it too (9-17 ms for the
same steps).  A user-interactive QoS class and a Mach time-constraint
policy on the media thread were both tried and changed nothing, so the work
itself had to shrink.

**What changed (1e3a4e0f, 254a3402):**

- `v34rx.c` is built with `-O2` (clang only; `SPANDSP_V34RX_OPT_CFLAGS` in
  the makefile, built before the main SpanDSP make so no automake is
  needed) and makes every function optnone except the T/3 B1 acquisition:
  template build, coarse scan and fit, 4-point training and locate, the
  least-squares solve.  Optimising more is not bit-exact: all of SpanDSP at
  -O2 changes 3 of 24 RasFinder replays (our Phase 2 transmit moves about 1%
  and a Phase 3 S detection a frame), and `v34tx.c` at -O2, **even with every
  function optnone**, stops `v56_loopback_test --ad 5 --edd 3` syncing.
  That last one means its output depends on more than arithmetic
  (uninitialised stack reads are the usual suspect); not chased here.
- The MP template's constellation power (a 32000-symbol run of the V.34
  mapper, which stays -O0 in `v34tx.c`) is split over the three packets
  before its only consumer, `v90_t3_blind_acquire()`, in the same order into
  the same sums; reloading that template after a failed search no longer
  repeats it, and a search window that is going to fail (best fit under
  0.95) no longer runs it at all.  That last case was opt-5 below: 21-22 ms
  per retry window after its 4-point training failed.
- Longest data-mode callback in the replays: 6.5 ms -> 2.4 ms on the dev
  Mac; a real-time replay on the mini 3.4-7.9 ms (live it was 20-26 ms).

**Proof of "no decoded bit changed":** `v90_engine_replay --sync` (new) runs
the engine on the media clock and wakes and drains the strict CP worker
between frames.  Before it, two runs of the same build differed: the worker
was woken mid-callback and read state the callback was still changing, which
moved batch CPt recoveries by a frame.  With `ME_RX_BIT_DUMP` (new: every bit
the receiver hands up), all 30 taps (`lec-*`, `lec2-*`, `opt-*`) give
identical bits, logs, and TX and RX PCM dumps against the -O0 build.  make
test-fast 550/550.  `ME_SLOW_CALLBACK_US` lowers the slow-callback log
threshold, to see the steps on a faster host.

**Live, from the mini, 2901 -> 6311/6312/6313, upstream cap 21600, 90 s apart:**

| build | calls | V.90 data mode | LAPM + login | echo moved at B1 | data-mode callback > 20 ms | held before first resync |
| --- | --- | --- | --- | --- | --- | --- |
| lec + lec2 (before) | 24 | 20 | 4 | 16 | every call | 12 s on 12 of 20 |
| 1e3a4e0f, `opt-*` | 12 | 11 | **9** | none (opt-5 in its retry windows) | opt-5 only (fixed in 254a3402) | 15-45 s, median 29 |
| 254a3402, `opt2-*` | 9 | 8 | **6** | none | none in V.90 (opt2-9 was a V.34 fallback) | 30-45 s; opt2-5, -6 1 s |

Both batches together: 19 of 21 calls in V.90 data mode, **15 of 19 to LAPM
and the login prompt** (was 4 of 20).

The calls that missed: opt-5 (4-point training failed, retry windows of
21-22 ms moved the echo, peer retrained; fixed since), opt-6 (B1 acquired
cleanly, decision error 0.025, yet the data decodes white: 50% ones and
3-12% bad shell frames, so the 4-point-end + 9 symbols B1 position is
probably a symbol out on this call; open), opt-10 (the RasFinder answered
with V.25 automode and we trained V.32bis; no V.8 at all).  In `opt2-*`,
opt2-5 and opt2-6 decode cleanly for 1.3-1.6 s (decision error 0.025, V.42
detection succeeds) and then the discontinuity detector fires with no echo
move and no slow callback before it; opt2-9 fell back to plain V.34 19200,
whose data-mode callbacks (22-24 ms) are the V.34 path, not this one.

**What limits it now** is no longer the echo.  Calls hold LAPM 15-45 s and
then the discontinuity detector asks for a 9.6 resync with no echo move
before it (opt-1, -4, -7, -12, and only 1.5 s in on opt2-5, -6); the echo
moves the logs do show (opt-3, opt-9, opt2-4, opt2-7) all come after a
resync, from the retrain's own callbacks.  Next: what the upstream does
in the seconds before those resyncs, on the `opt-*` taps.

## 2026-10-16 (evening): the Phase 3/4 "lottery" was our echo-canceller fit stalling DIL

**The calls that never reached V.90 data mode were ours.**  Of the 11 V.90
attempts on the fix builds (`rf-mini-20261016/fix2-*`, `fix3-*`, `fix4-*`),
four first attempts failed (fix2-6313, fix3-2-6312, fix4-1-6311,
fix4-4-6311), and every one of them died the same way: the peer sent a
CRC-valid **1788-bit CPt** that we rejected (`accepted=0`, 6-7 copies) until
it retrained about 4.7 s after our Ri.  Every first attempt that reached data
mode got a 972-bit CPt instead.  1788 = 156 + 136 x 12 is Table 14 with six
constellations and bit 128 set (codec-output masks follow the transmit
masks).  Decoded outside the receiver (`V90_CP_BIT_DUMP` under
`v90_engine_replay --dial`, framed with `tools/v90_cp_frame.py`), fix4-1's CPt
is drn=9, Sr=1, ld=1, `trn1d_gain` 16384 (exactly 2.0, i.e. 6 dB of claimed
loss), six identical transmit sets {53,94,112,119}, and six codec sets that
are not a pad of them or of each other: {48,75,85}, {50,52,80}, {49,79},
{67,75,85}, {37,44,80}, {68,75}.  `v90_configure_phase4_mapper()` refuses a
codec set whose population differs from its transmit set, so the frame never
configures TRN2d.  The peer is not merely mis-sizing a pad: its DIL analysis
came out as garbage.

**What goes wrong first is our transmission at DIL start.**  The V.90 line
echo canceller is fitted when we detect the analogue modem's S, i.e. in the
media callback that switches our transmitter from J'd to DIL (8.3.4,
9.3.1.x).  That fit's bulk-delay search correlated 5953 lags one at a time
through the bounds-checked `tx_at()`, 10-16 ms on the mini (measured with a
bench on the host itself), on top of the ~10 ms of normal receive work, and
it is logged as `slow media callback: rx took 23-28 ms (v34_rx=10)` straight
after `line echo canceller: fit 1`.  Our RTP is sent from that same callback,
so `rtp-tx.csv` shows a send gap at DIL start on **every** call, and its
length decides the call:

| attempt | send gap at DIL start | CPt | outcome |
| --- | --- | --- | --- |
| fix2-6311 #1, fix2-6312 #1, fix3-1, fix3-3 #1, fix3-4 #1, fix4-3 | 30.8-32.9 ms | 972 bits | V.90 data mode (all 6) |
| fix3-4 #2 (a later retrain) | 33.3 ms | 972 bits | data mode |
| fix4-2, fix3-3 #2 (a later retrain) | 30.8 ms | 972 bits | TRN2d/MP sent, peer retrained without CP (below) |
| fix2-6313, fix3-2, fix4-1 #1, fix4-1 #2, fix4-4 #1, fix4-4 #2 | 36.0-41.7 ms | 1788 bits, rejected | peer retrain at Ri + ~4.7 s |
| fix2-6311 #2 | 37.5 ms | none decoded | peer retrain after Ri |
| fix2-6312 #2 (a later retrain) | 42.3 ms | 972 bits | data mode |

So 7 of the 8 attempts with a gap over 35 ms failed at CPt, and none of the
10 under 34 ms did.  This is the mechanism already measured after B1 (top
section below): a hole in our RTP longer than the AudioCodes' playout margin
makes it re-centre, and here the hole lands in the first few DIL segments, so
the analogue modem measures a DIL that is shifted part way through.  The
replay reproduces the rejected CPt exactly but cannot test the fix: the peer's
answer is in the recording.

**Fix 1 (57f34b5b, `v34_line_ec.c`):** the bulk-delay search runs over a linear
copy of the transmit history with 16 lags accumulated per pass, each in its
original summation order, and the least-squares loops index the same copy.
The fit takes 1.2-4.5 ms on the mini (was 10-16) and 2.3 ms on the dev Mac
(was 13.6); its output (lag, ERLE, every fit line) is identical on all 12
taps.  make test-fast 550/550.

**The other two failure shapes, not this one:**

- fix4-2 accepted a normal CPt and sent R-bar-i, TRN2d and MP, but the peer
  kept sending CPt for 4 s (about 25 frames) and retrained: it never
  detected R-bar-i (9.4.2.2).  No send gap or slow callback there.
  fix3-3's second attempt is similar (TRN2d/MP, retrain 6.6 s later).  Left
  open.
- fix-6313 (the build before fix2) died in Phase 3: the peer sent Tone A
  (band2400 0.43) where its S should have been, about 1.8 s into our Jd, so
  its TRN1d training failed.  The only anomaly is a 35.2 ms send gap 1.9 s
  into TRN1d with no slow callback logged.  The INFO1a that followed is not
  nonsense: baud 0 (2400) both ways, pre-emphasis 10, is a valid V.34
  INFO1a; this peer falls back to V.34 after a failed V.90 Phase 3, and the
  call reached plain V.34 data at 21600 (the 2400-baud maximum).

**Live, from the mini (`rf-mini-20261016/lec-*`, `lec2-*`), accounts 2901 ->
6311/6312/6313 in rotation, upstream capped at 21600, 90 s apart:**

| build | calls | V.90 data mode | send gap at DIL start | bad CPt | slow callback in DIL |
| --- | --- | --- | --- | --- | --- |
| fix2-fix4 (before) | 11 | 6 | 30.8-42.3 ms | 4 calls | 23-28 ms (the fit) |
| 57f34b5b (fit only), `lec-*` | 12 | **10** | 23.2-32.4 ms | 0 | 31-40 ms on lec-1 #2, lec-3, lec-9 (structural check, below) |
| 3c9645b0 (both), `lec2-*` | 12 | **10** | 25.8-32.4 ms | 0 | none |

No unusable CPt in 24 calls and 40 DIL starts.  Two of the 1788-bit CPts in
these batches (lec-2, lec-9) carry consistent codec sets, one chord below the
transmit sets with `trn1d_gain` 2.0, and are accepted and used: bit 128 is
not the defect, a DIL measured across a playout jump is.

Batch 1 showed a second synchronous job inside DIL.  Every S candidate after
the one that ends Jd went through p3_demod's 200 ms structural check before
`v90.c`, which ignores S during DIL's first descriptor cycle (all of DIL on
this peer) whatever the check says.  That made 31-40 ms callbacks, and both
batch-1 calls that missed data mode (lec-3, lec-9) had one, then got as far
as TRN2d/MP without the peer ever sending CP.  **3c9645b0** runs the check
only when `v90_dil_s_can_terminate()`; the replays' outcomes are unchanged,
and batch 2 had no slow callback in Phase 3 at all.

What still misses V.90 data mode is not a hole of ours: the peer retraining
in our Jd (fix-6313 and lec2-11 #1: no S, its TRN1d training failed),
the peer never answering our R-bar-i/TRN2d/MP with CP (fix4-2, fix3-3 #2,
lec2-11 #2), and lec2-12, where the peer's first INFO1a already chose V.34
(baud 4 both ways).

**What now limits LAPM is the same mechanism after B1, not this one.**  Of
the 20 calls that reached V.90 data mode, the 4 that reached the RasFinder
login prompt (lec-12, lec2-3, lec2-6, lec2-9) are the 4 whose echo never
moved.  On almost every call the receive callback just after B1 is still
20-26 ms on the mini (d99f144d had none over 20 ms in its one data-mode
call), the echo moves +5 to +10 ms, the tracker follows, and about 12 s
later the discontinuity detector asks for a resync and the peer retrains.
The callbacks need to come down further on this host, or the tracker's
correction needs to keep the upstream alive.

## 2026-10-16 (later): the eye closes because our echo moved and our canceller did not

**What changes on the wire 0.8 s after B1 is the delay of our own echo.**
The far end's line carries our downstream back to us at about -23 dB, and the
V.90 line echo canceller (`v34_line_ec.c`) is fitted to it once, in Phase 3.
Measured from the taps with `tools/echo_lag_track.py`, the echo delay is
constant through training and then steps later in the middle of data mode:

| call | data-mode stall | echo delay (tap samples) before -> after |
| --- | --- | --- |
| v90-6313-cap9600 | ~85 ms | 1928 -> 2288 -> 2328 |
| diag-6313-21600 | 107 ms | 2006 -> 2686 |
| final-6311-21600 | 110 ms | 1928 -> 2648 |
| v14-6313-21600 | 107 ms | 1848 -> 2528 |
| blind-6312-21600 | 101 ms | 1848 -> 2529 |
| bf-6312-21600 | 43.5 ms | 1848 -> 2089 |

The cause is ours.  Our transmit is sent from the same pjmedia callback that
runs the receiver, so a long receive callback is a hole in our RTP.  At
acquisition the T/3 receiver ran its six B1 templates (6 ms each), the
4-point training (8 ms) and then caught up on every symbol since B1 (about
1750 of them, 75 ms) in one callback: 107-120 ms, about 0.55 s after B1.  The
AudioCodes' playout buffer underran on it and re-centred, so everything we
sent afterwards plays out (and echoes) 30-85 ms later; one echo round trip
(~240 ms) after the hole, the canceller is subtracting the echo at the old
delay, i.e. adding a second copy.  The peer also hears a gap in our
downstream.

**The proof is offline on the calls' own taps:** with the Phase-3 echo filter
simply moved to the measured delays, the 9600 call's post-event error goes
from 0.065 back to 0.0016 (0.0012 before the event), and the 21600 call's
lattice distance over data symbols 2700-5000 from 0.62 to 0.046.  The
alignment has to be exact: one sample either side of 2683 and the 21600 eye
stays shut, which is why nothing earlier (gain, rotation, timing, precoding)
fitted.  The added error was white, data-independent and uncorrelated with the
peer's symbols, as an echo of our transmit is; regressing it on our TX tap
found nothing only because the first attempt used the wrong tap alignment.
The line residual barely moves across the event (23.0 -> 22.5 dB, the ~180 rms
of echo every call has); it is the receiver's view of it that changes.

Why the earlier tests missed it: a 43 ms hole "did not move the event"
because the event is fixed by when the hole happens, not by its length; the
replay reproduces it because the recording already contains the moved echo;
and V.14 framing coincides because the peer's DTE data starts at about the
same time.

**Fix, two parts:**

1. No long callbacks at acquisition (`v34rx.c`).  The B1 template search runs
   one template per 20 ms packet (`V34_V90_T3_ACQ_STEP_GAP`), the 4-point
   training and commit get a packet of their own, and the emitter produces at
   most one symbol per T/3 sample (`V34_V90_T3_EMIT_BUDGET`), so the post-B1
   backlog drains at three times real time over about a quarter of a second.
   Replays: the data-mode callback goes from 115-120 ms to 20 ms; the decoded
   bits and errors are unchanged (6311-1, 6312-1, 6313-1, the cap9600 fixture).
2. The canceller follows a moved echo (`v34_line_ec.c`, data mode only).  It
   correlates what it receives against its own prediction of the echo at
   +/-150 ms of shift, both whitened by a 16th-order predictor of the received
   signal (the echo is -25 dB in band but -5 to -9 dB at 3.6-3.9 kHz), over a
   sliding second, and moves the bulk delay when a shifted peak is 6 sigma
   above the floor, 3 times the present alignment and 1.5 times the next peak
   (a periodic training transmission has an alias at every period; that is
   why it is off outside `ME_DATA`).  Offline on the 9600 call it finds the
   +400 about 1.3 s after it happens; `v34_line_ec_test` checks the move, no
   move on a 60 s stationary echo, and no alias move with a periodic
   transmission.  `ME_LEC_NO_TRACK=1` holds the fitted delay.

**Verified live** below (wired LAN host); vmhost and tower were down.  The replay
cannot test part 2: once in data mode our transmit depends on what we decode,
so the replay's transmit no longer matches the one the recorded echo came
from.  Next call: check `slow media callback` stays under ~20 ms in data
mode, `tools/echo_lag_track.py` shows no step (or the log shows `echo moved`
and the eye recovering), and whether LAPM comes up.

**Live, from a wired LAN host (Mac mini 10.69.70.168, `rf-mini-20261016`;
vmhost was down).** Accounts 2901 → 6311/6312/6313, upstream capped at 21600:

| build | V.90 data mode | echo moved | outcome |
| --- | --- | --- | --- |
| f654ad3e (search spread, tracker) | 2 of 3 | +15 ms on both (29 ms callbacks) | 6312: tracker followed after ~1 s, then **LAPM + RasFinder login prompt**, upstream 0.025 for 14 s; 6311: discontinuity detector asked for a 9.6 resync before the tracker moved |
| 857af503 (one step per packet) | 3 of 4 | +10 ms on 2 (4-point training 21-25 ms on this host) | 6311 no move: **LAPM + login**, 31 s; the two that moved resynced ~12 s in |
| d99f144d (training split, cheap builds) | 1 of 4 | none (no callback over 20 ms) | 6313: **LAPM + login**, upstream 0.024-0.029 for the whole 47 s until our hangup |

The calls that never reached data mode retrained in Phase 3/4 or got an
INFO1a that declined V.90.  That was not the rig's lottery: most of it was
our echo-canceller fit stalling the start of DIL (section above, fixed in
57f34b5b and 3c9645b0).  The mini is about 2.5x slower than the dev Mac, and
the playout moved by about (callback - 13 ms), so callbacks need to stay well
under one packet on the slowest host that runs calls.  Plain V.34 fallbacks on
the same rig reached LAPM and the login prompt too (16800).

Two calls from this Mac over the VPN (vmhost down, `reference-data/artifacts/rf-mac-20261016/`)
could not test it.  The far end reports 0.3-0.4% of OUR packets lost, in
80 ms bursts, and nothing lost in our receive direction.  Call 1 lost V.8:
ANSam, then the pure 2250 Hz tone at ~8 s, then V.32 AC (600 + 3000 Hz), so
the peer never heard our CM.  Call 2 missed INFO1a three times (once invalid),
declined V.90 and fell back to plain V.34 16800, with no LAPM.  The V.90 T/3
path never ran.  Live checks still need the wired rig.

## 2026-10-16: B1 is acquired without a template; the upstream now dies ~0.8 s later

**Why B1 never matched: the MT5634 does not reset its scrambler before B1.**
10.1.3.1/V.34 zeroes the scrambler, trellis encoder, differential encoder and
precoder before B1, and every template we build assumes it.  On
`v90-6313-cap9600` (upstream 9600, 12-point constellation, decisions clean at
0.04 from the lattice) the symbols after E are V.34 9600 data on our own
mapper's constellation, and every later mapping frame of B1 is reproduced
*exactly* by our mapper from scrambler state `0x18e679` -- which neither a
zero reset nor ones run on from E's line bits reaches.  The B1 symbols'
amplitude sequence is identical across calls on different lines, so the state
is something fixed in the modem, but nothing on the wire says what.  With the
scrambler wrong, every template is uncorrelated whatever the trellis state, so
the search sat at the 42-parameters-on-128-symbols noise floor (17-27%).  It is
the same thing v34rx_data.c already records for this peer's B1 after a V.34
renegotiation (0.19-0.25 at every state).

Two more facts, from the same calls: the clock offset to our bearer is about
10 ppm, and **B1 starts nine symbols after the last 4-point symbol** of CP'/E
on every call measured (6311-1, 6313-1, diag-6313-21600, and cap9600 to the
mapping frame) -- one transition symbol and eight of near-silence.  The
silence is not always quiet enough to find by energy (diag-6313-21600: powers
0.02-0.2 of the signal), and a B1 start one symbol out decodes clean symbols
(0.02 from the lattice) to white bits, which the frame-phase sweep cannot
repair because it only moves whole mapping frames.

**Fix (v34rx.c, `v90_t3_blind_acquire()`):** when no template fits, train the
T/3 filter on the 4-point CP'/E before E (CMA, decision-directed LMS, then
least squares on the decisions: 35-37 dB on every call), find its last clean
symbol, take B1 nine symbols later, scale to the constellation using the
data's own measured power, and start the Viterbi decoder with no state
preference.  B1 is only needed to train; the scrambler self-synchronises and
the quadrant is differential.  Only tried after the template search fails, so
peers that do reset (slmodemd) are unaffected; `ME_V90_T3_NO_BLIND=1` turns it
off.  `tests/fast.list` replays `reference-data/artifacts/rf-mt5634-v90-b1` (the cap9600
call) and requires B1 to decode as 100% ones.

Replays: 6311-1/6312-1/6313-1 at 31200 decode B1 at 91-99% ones; cap9600
decodes at 0.002 from the lattice, 100% ones, and V.42 detection succeeds.
Live from vmhost CT 117 (`blind-6311-1` at 31200, `blind-6312-21600`,
`final-6311-21600`, `bf-6312-21600`, `v14-6313-21600`): every call acquires,
B1 decodes as ones, and the V.42 detection phase succeeds on our side.

**The new blocker: about 0.8 s after B1 the upstream eye closes for good, on
every call, at 31200 and at 21600** (0.02 from the lattice to 0.66 within
about 20 symbols at 21600; the peer retrains ~12 s later; LAPM never comes
up).  It is in the signal, not the receiver: a fully frozen receiver, and an
offline least-squares filter on the receiver's raw T/3 samples
(`diag-6313-21600/raw.bin`), both see it.  Ruled out on those samples: a
timing slip or drift (the 3200 Hz cyclostationary line holds phase across
it), gain (power 92.2 -> 92.4, gain scan 0.3-3 x), rotation, the V.34
non-linear encoder (9.7, both Theta), precoding (3- and 6-tap DD fits find
h ~ 0), a 4-point retrain signal, RTP loss (0 lost packets), and our V.42:
with `ME_DATA_FRAMING=v14` (we send only ones) it happens the same, at the
moment the peer's own DTE data starts (V.14 framing appears in the same
window).  Our transmit leaves a 60-130 ms hole in the RTP stream when the B1
search runs in the media thread (`rtp-tx.csv`), but a 43 ms one makes no
difference to when the eye closes.  Next: decide whether it is the peer or
the AudioCodes, e.g. by comparing what the gateway passes against the
peer's line with the analogue-side capture, and look at the error vector
around the event on a 9600 call (which survives it: 0.008 -> 0.07).

## 2026-10-15: Phase 4 passes; the upstream B1 is now the blocker

The RasFinder is back on three AudioCodes MP-114 lines, ext 6311/6312/6313
(calls placed from vmhost CT 117, wired; tower was down; recipe in
`reference-data/artifacts/rf-pve-20261015/run.sh`). On HEAD (c2262d89), one call per line:
V.8, Phase 2, 3 and 4 complete on the first attempt every time, and the
peer sends data-mode CP and E. **V.90 data mode at 45333 / 45333 / 49333
downstream, 31200 upstream** is the first V.90 data mode against this peer.
The 2.04 s Phase 4 timer described below no longer fires.

What fails now is our upstream receive. The V.90 T/3 B1 search never
matches. Its best fit over 7 windows is 17-27%, which is the noise floor,
and the bar is 95%. V.42 then sees nothing, and the peer retrains about 12 s
into data. The second V.90 attempt does the same thing. The call then falls
back to plain V.34 21600, where the symbol-domain B1 finder locks (correlation
1.000, 8 symbols after E, far-end trellis state 13, SNR 35 dB). The peer
hangs up at about 90 s, before LAPM comes up.

Ruled out (each on the offline replay, which reproduces the failure in
3.5 s: `v90_engine_replay reference-data/artifacts/rf-pve-20261015/v90-6313-1/live-rx.g711
ulaw --fast --dial`):

- **Carrier.** The CPt batch log's `carrier=high` only means
  `v90_cp_live.c` tries the high carrier first; differential CPt decoding
  tolerates the ~91 Hz difference. Phase 3's S detector reads the low
  carrier at 1.00/0.01, and the receive tap after E is a continuous in-band
  3200-baud signal (centroid about 1780 Hz).
- **Trellis start state** (the MT5634 trait that broke plain V.34). The V.90
  template searched over every initial state of all three trellis sizes,
  in a temporary hack that has been reverted, still peaks at 25-27%.
- **Rate.** `ME_V90_UPSTREAM_MAX_BPS=9600` live gives an upstream of 9600 and
  the same 20% fits.

So B1 is not being found where the T/3 search looks, or the template is
wrong for this peer in some way that does not depend on rate (precoder,
shaping, timing, or the anchor).

## Where this stands (updated 2026-09-29)

V.90 data mode against the RasFinder is **not** established. What is now settled,
newest first; the sections below are in discovery order and carry the evidence.

**The Phase 4 blocker is a timer, not a verdict.** From our TRN2d to the peer's
Tone A is 2.042 s ± 28 ms across four calls that differ in what we transmit *and*
how we receive; §9.4.1.3's 2000 ms MP deadline is the only comparable interval in
Phase 4. So the peer never *recognises* our MP rather than grading and rejecting
it — which retires "it is rejecting our TRN2d, so the fault is in TRN2d itself".

**Our side is conformant, verified on the failing call's own transmit tap.**
TRN2d descrambles to ones, MP is Type 0 CRC-valid with zero demap failures,
Phase 3 gives 177 CRC-valid Jd frames; the K/S split, modulus algorithm, label
order, Table 16 layout and Ri codeword are audited against the clauses, and the
§5.4.5 shaper is exonerated by SmartLink's CPt carrying the same Sr = 1 while
that peer reaches data mode.

**The remaining lever is TRN2d length, and it has never been moved on this
peer.** Both directions need sweeping. The arms, the scorer and the baseline are
in place; it wants a session where the rig reaches Phase 4 at all.

**Two things are ours and are the next blockers behind it:** our CP-window
receive is echo-limited by our own transmit and improvable from 0.572 to 0.787
SCR ones with two existing knobs, and `ME_V34_ECHO=canceller` is condemned on
this path. **One thing is probably not ours at all:** 8 of 21 calls that day
died in V.8, eleven of sixteen such failures carrying a pure 2250 Hz tone that
is too spectrally clean to have crossed the analogue loop.

**Do not** make §9.3.1.3's 500 ms Sd bound absolute — this peer's Ja descriptor
arrives 2.12–2.83 s after the first Ja bits, 11 of 11, and it would break every
working call.


V.90 data mode is not established. The RasFinder accepts our Phase 3, sends
CPt, terminates that group after our barred Ri, then retrains while we repeat
MP. V.90 9.4.2.2 permits SCR after CPt; 9.4.2.3 then requires CP. We have not
reliably recovered a data-mode CP from this peer.

## Correcting the receive diagnosis

The previous claim that an independent demodulator reads the failing region
at 5.75 degrees was measured at RX-file 28–30.4 seconds in
`reference-data/artifacts/rf-maxpow-c1/live-rx.g711`. That is **after** this call's Phase 4
retrain. It cannot exonerate the wire or condemn the live equalizer.

`tools/v90_phase4_capture_check.py` now locates Table-14 frames using the CRC,
with the prescribed clockwise differential decode and GPA descrambler. It
reports offsets in the RX file, not elapsed engine time or TX-file time:

| RX-file offset | observation |
| --- | --- |
| 23.000312 s | first 1788-bit CPt, CRC valid |
| 23.279688 s | second CPt, CRC bad |
| 23.250–23.375 s | differential phase residual about 5.5 degrees RMS |
| 23.375–23.500 s | residual rises to about 20.5 degrees |
| following SCR window | about 21–22 degrees, imperfect GPA ones |

This independent frontend reproduces the degradation in the live receiver.
The live equalizer experiments remain valid negative results, but neither
"the wire is clean" nor "every equalizer explanation is ruled out" follows.
The new columns in `V34_MP_RX_DUMP` append the receiver sample counter,
cumulative timing correction, and half-symbol phase to the original ten
columns, so future comparisons need not infer sample positions from a stage
counter that resets.

Reproduction (Python with NumPy, PCMU taps):

```sh
python3 tools/v90_phase4_capture_check.py \
  reference-data/artifacts/rf-maxpow-c1/live-rx.g711 21 25.4 \
  --tx reference-data/artifacts/rf-maxpow-c1/live-tx.g711 --echo-window 23.5 25.2
```

## Echo is measurable, but cancellation is not yet a connection fix

Cross-correlating the first half of RX-file 23.5–25.2 s against the TX tap
finds a 936-sample RX-minus-TX **file offset**, with correlation 0.2099. It
is not a physical round-trip-delay measurement: tap origins can differ.
A 129-tap FIR fitted on that first half removes 13.028% of its power and
9.577% in the separate, held-out second half. Unlike a fit scored on its own
training data, the held-out result demonstrates predictive TX correlation.

Subtracting this fit offline improves the following SCR phase residual from
about 21 to 13 degrees and its decoded ones fraction to roughly 95%, but the
second CPt still fails CRC. This is evidence of interference, not proof that
echo alone explains the whole failure. The fit is diagnostic; it changes no
wire codewords and is not installed as a live filter.

The echo checker was also exercised with an independent synthetic signal:
a known 936-sample delay and gain 0.2 were recovered, with about 80% power
removed on both train and held-out portions. An unrelated-noise control
removed 0.284% on train and **minus 0.306%** on held-out data.

## Live experiments in this session

Captures are under `reference-data/artifacts/rf-cp-investigation/` and remain local evidence,
not cloneable test fixtures. Calls were spaced at least 60 seconds apart.

* `live-baseline`: the first Phase 3 attempt failed; the retrain reached DIL
  and Ri near the end of the 40-second hold. No Phase 4 verdict.
* `live-codec`: `ME_V90_SHAPER_METRIC=codec` reached CPt/TRN2d/MP. CPt was
  accepted at engine +28.422 s and the peer retrained at +30.541 s, **2.119 s
  later**. Thus the previously unmeasured codec-metric arm does not rescue
  Phase 4 on this call. The default transmit-level metric remains unchanged.
* `live-echo`: enabling a recent-TX-reference NLMS filter at the beginning
  of CP reception made acquisition worse: no accepted CPt, and reported
  output power slightly exceeded input power. This is not a successful fix.
* `live-echo-late`: the refined arm enables the filter only after CPt, during
  TRN2d/MP. The call fell back before reaching that point, so it supplies no
  result for the refined arm.

The existing optional canceller advances a queue read cursor only while it
runs. That cursor need not correspond to current RX time after cancellation
has been disabled. The experimental `ME_V90_CP_ECHO=1` instead references
recent TX history and permits a longer echo tail/callback skew. It defaults
off; its acquisition failure above must not be reported as an improvement.

The full `make test` suite passed during this investigation. Hardware data
mode and payload remain unproven. Receiver improvements alone cannot establish
why the peer rejects our downstream training; a peer-side diagnostic or a
successful controlled Phase 4 trial is still needed.

## The peer's Phase 4 abandonment is a timer, not a verdict (2026-09-29)

The interval from our barred-Ri acknowledgement of the peer's CPt to its
retrain, over the four calls on record that reached Phase 4:

| capture | CPt accepted | peer retrain | window | CP frames |
| --- | --- | --- | --- | --- |
| `rf-maxpow-c1` | +23283 ms | +25400 ms | **2117 ms** | 0 |
| `rf-padrep-a1` | +24555 ms | +26675 ms | **2120 ms** | 0 |
| `rf-cp-investigation/live-echo-late2` | +23364 ms | +25483 ms | **2119 ms** | 0 |
| `rf-cp-investigation/live-codec` | +28422 ms | +30541 ms | **2119 ms** | 0 |

Those four calls differ in `ME_V90_CP_PAD_REPAIR`, in `ME_V90_SHAPER_METRIC`
(transmit level vs codec output) and in whether the experimental CP-era echo
filter ran — that is, in what we transmitted and in how we received. A 3 ms
spread across them is a timer expiring. A study reaching a verdict would move
when the thing it studies moves, the way SmartLink's `linear mapping study`
moves deterministically with TRN1d length (§37/§38).

**§9.4.1.3 is the only 2000 ms in Phase 4**: "Within 2000 ms of beginning to
transmit TRN2d, the digital modem shall send MP sequences." The fit is within
the measurement's own uncertainty once the round trip (99–161 ms, from pjsua's
own RTT report on these calls) is allowed for, and no other Phase 4 interval is
close — §9.4.2.2 caps SCR at 4000 ms and §9.4.1/§9.4.2's retrain bounds are
15 s plus five round trips. So the reading is that the peer never recognises
our MP and gives up on the clause's deadline.

`tools/rf_phase4_summary.py` prints that table from `server.log`, and the
engine now stamps the Phase 4 transmit stage changes with `[TRACE +Nms]` so
they can be read against the retrain on one clock. (`[V90]` is unbuffered
stderr and `[ME]` is buffered; they interleave out of order in `server.log`,
so only the `[TRACE]` stamps are safe to read a sequence from.)

## The window measured on the wire, not from our own detector (2026-09-29)

The 2117-2120 ms table is measured between two engine events, so it carries our
Tone-A detector's confirmation latency. Measuring the peer's Tone A directly on
the receive tap instead -- 10 ms blocks, 2400 Hz fraction over 0.30 with the
block RMS over 500 -- and taking our TRN2d start from the CPt-accept stamp
(`--phase4-trace` puts TRN2d at 134T by RX-file 23.300 s on `rf-maxpow-c1`, so
it began 23.283 s, which is the stamp):

| capture | our TRN2d | peer Tone A on the tap | delta |
| --- | --- | --- | --- |
| `rf-maxpow-c1` | 23.283 s | 25.320 s | **2.037 s** |
| `rf-padrep-a1` | 24.555 s | 26.600 s | **2.045 s** |
| `live-echo-late2` | 23.364 s | 25.420 s | **2.056 s** |
| `live-codec` | 28.422 s | 30.450 s | **2.028 s** |

Mean **2.042 s, spread 28 ms**. Both endpoints bracket one round trip -- we
sent the TRN2d, the peer answered -- so the peer's own window is that minus the
round trip: **1.92 s at a 120 ms round trip, 2.00 s at 40 ms**. The 936-sample
TX-to-RX offset the echo fit reports is an upper bound on that delay and
includes an unknown difference in tap origins, so the true figure sits inside
that range and §9.4.1.3's 2000 ms is its top end. Nothing else in Phase 4 is
within half a second of it.

## Our Phase 4 transmit is conformant on the failing call itself

`v90_analogue_rx_test --phase4-trace reference-data/artifacts/rf-maxpow-c1/live-tx.g711
<accepted-cp.bits> 186000` — our own transmit tap from the failing call, fed
to the analogue-role receiver configured by the CPt the peer actually sent —
reads Ri → barred Ri → **4074T of TRN2d descrambling to 11356 ones** → **MP
type 0, `max_drn=13`, `rate_mask=0x0FFF`, zero demap failures** (accepted CPt
recovered with `V90_CP_ACCEPT_DUMP` on `v90_engine_replay --dial`).

Read against the clauses rather than against our own receiver, which shares
the implementation and so cannot falsify a convention:

* **§5.4.2** — `d0..d(S-1)` are the sign bits and `dS..d(D-1)` the modulus
  bits, `b0` least significant. `v90_map_shaped_scrambled_frame()` builds
  `r` from `scrambled[sign_bits + i] << i`. Correct.
* **§5.4.3** — `Ki = Ri mod Mi`, `R(i+1) = (Ri - Ki)/Mi`, interval 0 first.
  Correct, and `r` is required to reach zero.
* **§5.4.4** — members of Ci labelled in **descending** order, label 0 the
  largest PCM code. `v90_cp_constellation_ucode()` walks the mask from the
  top. Correct.
* **§8.6.3 / Table 16** — Type 0 is 86 bits through the fill bit, extended to
  the next multiple of 6 symbols; at this peer's `D = K + S = 12 + 5 = 17`
  that is 102 bits = 6 mapping frames = 36 symbols, which is what we send.
  The CRC covers the 16-bit groups after each start bit, excluding frame sync.
* **§8.6.4** — "Ri denotes signal R using the single PCM codeword whose Ucode
  is U_INFO for all data frame intervals", so the 6.8 dB step from Ri to the
  CPt constellation at the R̄i→TRN2d seam is spec-mandated and is present on
  every V.90 call including the ones that reach data mode elsewhere.

So the MP reaches the wire, on time (500 ms into a 2000 ms budget), in a form
an independent decode of our own transmission accepts.

## The §5.4.5 spectral shaper is exonerated, by the other peer's own CPt

The shaper is the one part of the Phase 4 transmit chain that cannot be audited
by reading alone: §5.4.5.5 leaves the *choice* of sign inversion rule free
("select the rule that minimizes the spectral metric"), so only the rule
definitions and the trellis constraint are normative, and Figure 2's state
transitions do not survive text extraction. Rules A/B/C/D and the per-state
allowable sets do check out against the clause — `v90_shaper_rule_inverts()` is
B=all, C=even, D=odd, and `v90_select_shaper_rule()` offers {A,B} from state 0
and {C,D} from state 1, which is §5.4.5.5's own example — but the next-state
mapping (A,C to 0; B,D to 1) is unverified against the figure.

It does not need to be verified from the figure, because it has been verified
against a foreign peer. The CPt this peer sends is `D=17, K=12`, so
`S = D - K = 5` and `Sr = 6 - S = 1`: spectral shaping **enabled**, one sign bit
of redundancy. **SmartLink's CPt is `D=23, K=18`, so S = 5 and Sr = 1 as well**
— and that peer's analogue receiver decodes our shaped TRN2d and MP, completes
the MP/CP exchange and reaches V.90 data mode at 52000 bps. Same role pairing,
same shaping redundancy, same code.

So the shaper, the §5.4.3 modulus mapper, the §5.4.4 label order, the K/S split
and the Table 16 MP framing are all exercised by a foreign analogue modem that
gets to data mode, and none of them can be what the RasFinder fails to
recognise. What differs between the two peers' Phase 4 is the constellation
size (K=12 and four Ucodes per interval here against K=18 and eight there) and
that this one sits behind a real analogue loop with a measured ~-20 dB echo.
That leaves TRN2d length — never moved on this peer — as the lever, which is
where the unanswered sweep points.

## The post-CPt region is SCR, and there is no CP — now with a CRC anchor

An earlier reading of this region was withdrawn because its alignment was
matched onto the peer's Tone A, which is a near-constant dibit and therefore
matches any low-entropy reference. Anchored instead on the CRC-valid CPt
(`tools/v90_phase4_capture_check.py ... 22.9 25.2`):

```
RX 23.000312s CPt bits=1788 CRC=OK
RX 23.279687s CPt bits=1788 CRC=BAD
RX 23.000312s diff_rms_deg= 8.05 GPA_ones=0.089 max_dibit_fraction=0.273
RX 23.250312s diff_rms_deg= 5.54 GPA_ones=0.095 max_dibit_fraction=0.270
RX 23.375312s diff_rms_deg=20.52 GPA_ones=0.204 max_dibit_fraction=0.278
RX 23.625312s diff_rms_deg=21.67 GPA_ones=0.773 max_dibit_fraction=0.260
   ... through 25.0 s: diff_rms_deg 20.4-22.4, GPA_ones 0.77-0.89
```

A CP frame is ~94% zeros, so `GPA_ones` near 0.1 is a CP-family frame and
near 0.8 is SCR decoding imperfectly; `max_dibit_fraction` stays at 0.26-0.31
throughout, which is uniform, so none of this is the constant-dibit artefact.
**Two CPt frames and then SCR to the retrain, with no CP anywhere** — §9.4.2.3
makes CP a "shall", and this peer does not send it.

## Our own CP-window receive is echo-limited, by our own transmit

The residual steps once, at RX 23.375 s, from 5.5° to 20.5°, and holds there.
Measured on the taps over the same instants:

| RX-file t | TX RMS | RX RMS |
| --- | --- | --- |
| 23.20 s | 943 | 630 |
| 23.30 s | **2082** | 643 |
| 23.40 s | 2100 | 657 |

Our transmit steps **6.8 dB** at the R̄i→TRN2d seam (Ri is at U_INFO = 78,
magnitude 943; the CPt constellation is at ~2080), and the receive residual
steps **117 ms later** — which is the 936-sample TX-to-RX offset the echo fit
already reports, and is inside the 99–161 ms SIP round trip pjsua measured on
these calls. The received *level* barely moves, because a 9.6% power
contribution is 0.4 dB; what it destroys is the phase.

So the previously reported echo fit (a 129-tap FIR removing 9.577% of power on
**held-out** data, against −0.306% for an unrelated-noise control) is not an
incidental correlation: it is our own Phase 4 transmit arriving back at our
receiver one round trip later, and it is what takes the differential residual
from 5.5° to 21° for the whole window. An echo at 9.6% of received power is a
~9.8 dB SNR, and `atan(1/√9.4)` is 18° — close to the 21° observed.

This is a receive-side limit and **not** the reason the peer retrains, but it
is the next blocker behind it: at 21° and 82% ones we would be unlikely to
decode the peer's CP even if it sent one.

## Corrections to the standing notes

* "Extending startup TRN2d from 3996T to 12000T also changes nothing" is
  **not supported by any RasFinder evidence**. Every capture in `artifacts/`
  with a non-default TRN2d (`trn2d12000-025350Z`, `trn2d-ab-212417Z`, the
  July `v90-hardware` runs) is a SmartLink or Eicon call; no RasFinder capture
  has ever run one. `ME_V90_TRN2D_SYMBOLS` accepts 2040–16000, and §9.4.1.2's
  "minimum of 2040T" together with §9.4.1.3's 2000 ms budget makes anything up
  to ~14000T conformant. The default 3996T = 499 ms leaves three quarters of
  that budget unused, and against a peer behind a real analogue hybrid that is
  the obvious thing to have swept.
* The region after CPt was described as "not SCR ... 50.4% ones". With a CRC
  anchor it reads 0.77–0.89 and it is SCR. The 50.4% figure came from the
  withdrawn Tone-A alignment.

## Rig yield, and what it costs

`ME_V90_TRN1D_SYMBOLS` defaults to 20004T, set in §38 against SmartLink.
§9.3.2.7 lets the analogue modem retrain if Jd does not arrive within 4500 ms
of the end of Ja, and Jd is the last thing we send after Sd (384T), S̄d (48T)
and TRN1d, so 20004T puts Jd's first symbol 2554 ms into that budget and
leaves **1945 ms** of Jd airtime — before §9.3.1.3's Sd delay of up to 500 ms
is spent. The one RasFinder call on record that reached Phase 4 needed
**1586 ms** of Jd before the peer answered with S, i.e. 82% of the budget.
§21 already recorded a USR Courier retraining *sooner* at long TRN1d for
exactly this reason, and no RasFinder call has ever run a different one.

Over every call in `artifacts/` that completed TRN1d, `TRN1d + (Jd symbols to
S)` is **exactly 23888** for every length from 2496T to 20004T — so on
SmartLink the peer's S is a fixed timer measured from the start of TRN1d and
Jd airtime never binds there, which is why 20004T is safe on that rig. The one
RasFinder success gives 32688, a different constant, and one point cannot say
whether that peer's S is keyed on the same origin.

On 2026-09-29 the rig reached Phase 4 in **0 of 4** calls at the default,
failing variously in V.8 (`modulations=none`), and in Phase 3 with "no S after
24796 Jd symbols".

## Our CP-window receive, measured against SCR (2026-09-29)

SCR is binary ones (§8.3.5, through §10.1.3.9/V.34's modulation with GPA), so
the descrambled ones fraction over the SCR era is a known-content reference
for our own receiver, needing no CP frame from the peer. Scored on the startup
Phase 4 bit stream (`V90_CP_BIT_DUMP` under `v90_engine_replay --dial --fast`
on `reference-data/artifacts/rf-maxpow-c1/live-rx.g711`), SCR windows being those reading
over 0.45 ones:

| configuration | bits emitted | SCR ones (mean) | best window |
| --- | --- | --- | --- |
| default | 6040 | 0.572 | 0.700 |
| `ME_V90_CP_STREAM_STARTUP=1` | 20864 | **0.760** | 0.915 |
| that plus `ME_V90_CP_ECHO=1` | 20864 | **0.787** | **0.970** |
| that plus `ME_V34_ECHO=canceller` | — | **never reaches the CP window** | — |

The independent T/2 CMA frontend reads 0.77–0.89 on the same audio, so 0.787
is close to what this recording supports, and the default's 0.572 is not a
property of the wire. The gap is the hypothesis-lock gating: the emit is
conditioned on `mp_hypothesis >= 0`, so by default the framer sees only the
fragments a lock covers — 6040 bits of a window that is 20864 bits long.

**Both knobs stay default off, for measured reasons.**
`ME_V90_CP_STREAM_STARTUP=1` costs the §11.6 row `V34_DUPLEX_RENEG=4000
./v34_duplex_test 2400 9600 ulaw` **24 post-renegotiation bit errors against 0,
with caller resync restarts 7 → 12** — reproduced here, so that note is not
stale. And none of this can be validated end to end on this peer, because it
never sends a CP; SCR ones is a proxy, sound because SCR's content is known,
but a proxy.

**`ME_V34_ECHO=canceller` must not be enabled on this path.** Its own log is
the verdict: over a whole call it **never once removes power**, reading
`pre_rms=380 post_rms=28962 (-37.6 dB)` early and still `-5.1 dB` two hundred
thousand samples later — a net injector throughout, converging only in the
sense that it eventually adds less. With it on, the call does not reach the CP
window at all (no `V90_CP_BIT_DUMP` is produced), which is the same outcome the
previous session saw live and attributed to `ME_V90_CP_ECHO`. That knob is the
short recent-TX-reference filter and, with the window unfragmented, it is
mildly **helpful** rather than harmful.

A method note, because it cost a wrong conclusion here first: when sweeping
configurations that can abort before writing their output, `rm -f` the output
path each iteration. Reusing one filename made a run that produced nothing
score as byte-identical to the previous arm, and "the echo knobs make no
difference" was read off that stale file.

## The §9.3.1.3 Sd bound must NOT be made absolute on this peer (2026-09-29)

With the Phase 3 stages now stamped, today's commonest failure reads:

```
[TRACE +18848ms] V90 strict RX event=INFO1A_VALID u_info=78 -> Phase3
[TRACE +24846ms] V90 tx stage -> Sd          <- 6000 ms later
[TRACE +24907ms] V90 tx stage -> TRN1d
[TRACE +25028ms] V90 peer retrain detected
```

Ja bits were being captured from t=19.13 s with `parsed=0` throughout, so we
sat until v90.c's 6000 ms `ME_V90_WAIT_JA_FALLBACK_MS` and the peer retrained
182 ms after our Sd finally appeared. §9.3.1.3 says "After receiving Ja, the
digital modem may wait for up to 500 ms and shall then transmit signal Sd", and
the condition is receiving the **signal**, so the obvious change is to make
that 500 ms an absolute deadline from the first Ja bits rather than — as now —
merely the point at which a heuristic stops being suppressed, a heuristic that
still needs an energy gap and on these calls never fires at all.

**Do not make that change.** Over every RasFinder call in `artifacts/` whose Ja
descriptor parsed, the gap from the first captured Ja bits to the parse is:

```
2.25 2.15 2.25 2.29 2.23 2.25 2.29 2.51 2.31 2.83 2.12   (n=11, seconds)
```

— never under 2.1 s. This peer puts its DIL descriptor late in Ja, exactly as
`v90_ja_heuristic_allowed()`'s comment describes for the SmartLink class, so a
500 ms Sd deadline would stop the peer's Ja (§9.3.2.4) before the descriptor
arrived **on every currently-working call**. The current behaviour — wait for
the descriptor, bounded by the interop timer — is right for this peer, and the
failures are calls where the descriptor never parses at all, which starting Sd
earlier does not rescue either: without a descriptor there is no DIL plan.
The escape that remains untried is a preloaded descriptor
(`ME_V90_DIL_PROFILE`, §34's lead) combined with the clause's bound, and note
the descriptor is identical on every call that parses it
(`N=192 LSP=120 LTP=120`), which is what would make a preset for this peer
credible. It needs a Ja-transition detector that works when no energy gap
appears, which is the part that does not exist.

## A latent bug found while reading that gate

`v90_ja_heuristic_allowed()` measured its §9.3.1.3 escape from a
**function-scope `static`**, i.e. process lifetime, and this server runs many
calls per process. On every call after the first the origin still held call
one's timestamp, the elapsed time was already far past the bound, and the
heuristic was allowed immediately — so the descriptor protection the function
exists to provide was silently absent from every call but the first, and with a
2.2 s descriptor that is not a small window. Same shape as the
`v90_retire_phase2_cc_notch()` latch. Now file-scope and reset in
`v90_dil_capture_reset()`, whose four call sites are all per-call or
per-retrain scope.

## Rig yield, 2026-09-29

**0 of 21 calls reached Phase 4**, against 4 on 2026-09-28 with the same
defaults. Breakdown: 8 ended in V.8 with `status=4` / no JM received at all;
the rest reached Phase 2 or Phase 3 and ended in `INFO1A_INVALID` or
"no S after N Jd symbols". None of it is attributable to the TRN2d knob, which
acts strictly after all of it, and the knob is verified to reach the
transmitter at every swept value (2040 / 3996 / 12000 mapped symbols).

So **the TRN2d sweep is set up and unanswered**: the arms, the scorer
(`tools/rf_phase4_summary.py`), the stage stamps and the baseline (four calls
at 3996T, window 2117–2120 ms, zero CP frames) are all in place, and it needs
a session where the rig reaches Phase 4. One Phase-4 call per arm suffices,
because the peer's verdict is deterministic to 3 ms.

## The V.8 failure has a signature, and it is not a JM we failed to read

`V.8 result: status=4 ... modulations=none` ended 8 of 21 calls on 2026-09-29.
The first reading — that spandsp discards a single unrepeated JM — is wrong
(no candidate is ever stored; see the commit history). The taps say what
actually happens, and it is deterministic:

| capture | 2250 Hz-dominant blocks | first | last |
| --- | --- | --- | --- |
| `rf-trn1d-ab-194155Z/trn8004-r1` (fail) | 19 | **10.9 s** | 12.7 s |
| `rf-trn1d-ab-194155Z/control-r2` (fail) | 30 | **10.9 s** | 13.8 s |
| `rf-v8fix-200101Z/control-r1` (fail) | 18 | **10.9 s** | 12.6 s |
| `rf-v8fix-200101Z/trn12000-r2` (fail) | 23 | **10.9 s** | 13.1 s |
| `rf-v8fix-200101Z/trn12000-r1` (ok) | **0** | — | — |
| `rf-maxpow-c1` (ok) | **0** | — | — |

On a failing call the peer's ANSam runs 5.4 → 10.8 s (2100 Hz fraction 0.98,
RMS ~1000, alternating 950/1025 — the 15 Hz AM), and at **10.9 s it stops and
transmits a tone instead of JM**: 92.8% of the band energy at **2250 Hz** with
6.7% at 2850 Hz, and *nothing* at 980, 1180, 1650, 1850 or 2100 Hz. So there is
no V.21 JM on the wire to decode. Our CM is going out throughout at RMS 808,
and the peer plainly heard it — stopping ANSam is what V.8 has the answering
modem do on detecting CM.

Four for four at 10.9 s, zero on the calls that work, so it is a timer in the
peer or the network rather than a channel effect. What emits 2250 Hz here is
not identified: it is no V-series signal this path uses, and our taps end at
12.6–13.8 s because our own V.8 gives up at ~12.9 s and the engine hangs up
(`[ME] V.8 failed (status=4), hanging up`). **The cheap next experiment is to
hold the call through that instead of hanging up**, and see whether the tone
ends and the peer retries.

An offline `vpcm_decode --v8` of these receive taps reports a weak "CM" with
plausible-looking fields; that is not the peer. Its bytes overlap our own CM
and two such reads of two calls gave contradictory contents. **A receive tap on
a path with echo is not a record of what the peer sent** — read the tones.

## The best-motivated yield experiment, and why it is not yet a result

`ME_V90_JA_HEURISTIC_FALLBACK_MS` defaults to **500**, and what it does is allow
the early-Sd heuristics once the descriptor has not arrived within that time.
Its justification (§35k) is that "a healthy descriptor arrives a median 0.3 s
into Phase 3" — **measured on SmartLink.** On this peer the descriptor arrives
**2.12–2.83 s after the first Ja bits, in 11 of 11 calls**, so a 500 ms fallback
allows Sd four to five times sooner than the descriptor can appear, and our Sd
terminates the peer's Ja per §9.3.2.4. By construction that is a mechanism for
losing both the descriptor and the peer's Jd answer at once.

**It is a hypothesis, not a finding.** Over every RasFinder call in
`artifacts/` that transmitted Phase 3: the fallback fired on 3 calls and none
reached Phase 4; it did not fire on 15 and 4 reached Phase 4. With a 27% base
rate that split is what chance produces (Fisher exact, two-sided, p ≈ 1.0), so
the correlation carries nothing.

The experiment it prices is cheap and low-risk:
`ME_V90_JA_HEURISTIC_FALLBACK_MS=3000`, past this peer's 2.83 s worst case,
alternated against the 500 ms default and scored on whether Phase 3 completes.
It cannot disturb a healthy call, because `v90_ja_heuristic_allowed()` returns
true immediately once `g_v90_dil_parse_logged` is set, so the only calls it
changes are those whose descriptor is late or absent — which are already lost
today — and its worst case is the 6000 ms `ME_V90_WAIT_JA_FALLBACK_MS` those
calls already reach.

`ME_V90_V8_FAIL_HOLD=1` keeps the bearer up after a failed V.8 instead of
hanging up, so a tap records what follows the 2250 Hz tone. Default off.

### That tone is not the RasFinder's

Measured in the same call, with the peer's own ANSam as the control — it
provably traversed the analogue loop, since it carries the 15 Hz AM and the
§8.1.1 phase reversals:

| signal | frequency | envelope sd | phase-step sd | max jump |
| --- | --- | --- | --- | --- |
| peer ANSam, 6.5–8.1 s | **2101.1 Hz** | 15.7% | 23.3° | 167.8° |
| the tone, 11.0–12.6 s | **2250.0 Hz** | **0.1%** | **0.09°** | **0.3°** |

The peer's answer tone sits **1.1 Hz off** nominal — that is the RasFinder's own
oscillator, seen through the loop — while the 2250 Hz tone is exactly on an
integer frequency (2850.0 Hz for its second component, 11.4 dB down) with no
measurable phase noise and a 0.1%-stable envelope over 1.6 s.

**So it was not generated by anything on the far side of the analogue loop.** It
is digitally generated at or after the point where the loop's impairments would
have been added — the VG224 ATA or the PBX — not by the RasFinder. The RTP trace
narrows it further: over the call's 491 packets the SSRC changes exactly once, at
packet 8 (~0.16 s, the early-media to answered transition), and **not at the tone
onset**, so the pure tone arrives in the *same* stream that carried the peer's
ANSam. It is a substitution inside the media path rather than a second source
mixed in — which is what a gateway's modem/fax-tone handling does, and is
consistent with 5.4 s of continuous 2100 Hz (indistinguishable from a fax CED to
a naive detector) being what triggers it. Which
reframes the failure: these calls are most likely being pre-empted by the
gateway, and the standing advice that "its V.8 intermittently yields
`modulations=none`" attributes to the peer something that is probably not the
peer at all. `ME_V90_V8_FAIL_HOLD=1` is how to see what follows the tone.

### The peer's CM-detection latency, and the level asymmetry

V.8 has the answering modem stop ANSam once it has detected CM, so the end of
the 2100 Hz burst in the receive tap is when the peer heard us.
`tools/rf_ansam_latency.py` reports it. On the nine calls of one batch it looked
perfectly bimodal (2.1–3.3 s and success, 5.3–5.5 s and failure); **scored over
all 63 RasFinder calls in `artifacts/` that contain an ANSam burst, that is a
small-sample artefact and the ranges overlap** — status=2 runs 2.1–4.9 s
(median 2.2, n=47) and status=4 runs 2.2–5.5 s (median 5.4, n=16). What survives
is one-way, and sharply:

| indicator | catches | false alarms |
| --- | --- | --- |
| ANSam longer than 5.0 s | **11 of 16** failures | **0 of 47** successes |
| the 2250 Hz tone present | **11 of 16** failures | **0 of 47** successes |

They are the same eleven calls. So a long ANSam and the tone each imply failure
with no false positive in 47 successes, but **five of the sixteen failures do
neither** — they fail with a normal 2.2 s ANSam and no tone, which is a second
failure mode this has not characterised. All four calls that reached Phase 4 had
an ANSam of exactly 2.2 s.

Read the **end** as the primitive: ANSam starts at 5.3–5.5 s, and on the eleven
tone calls it runs to a fixed 10.7 s and is then cut off, where a working call
ends it at 7.4–8.5 s because the peer has our CM. So on those eleven the peer
did not lock our CM and the tone is downstream of the long ANSam rather than its
cause. The metric still yields a number from every call, which is what makes a
small A/B readable on a rig where half the calls fail — but it is a one-way
indicator, not a classifier.

The levels are asymmetric, and measured with one tool in one unit: **our CM
leaves at −20.1 dBFS while the peer's ANSam ARRIVES at −18.1 to −18.7 dBFS** —
after the ~6 dB loop loss that peer reports itself in Table 14's `trn1d_gain` —
so it transmits some 8 dB hotter than we do. spandsp's V.21 presets declare
−14 dBm0 and nothing in the tree could change it, while this same tree already
boosts V.34 Phase 2 from −14 to −10 with the comment "caller modem not
detecting our Phase 2". New `v8_tx_power()` and `ME_V8_TX_POWER_DBM0` make it
settable, unset by default. Whether −10 shortens the peer's latency is a live
A/B, not an inference.

### There are two V.8 failure modes, and the spandsp fix covers one of them

Splitting the sixteen `status=4` calls by the two indicators above:

* **Eleven** end ANSam at a fixed 10.7 s and carry the pure 2250 Hz tone from
  10.9 s. Nothing V.21 follows ANSam, so **no JM exists to decode** — replaying
  two of them, `cm_jm_decode_saved()` never fires because no candidate is ever
  stored, and the flow log reads CI×16, `'ANSam/' recognised`, CM×11,
  `Timeout waiting for JM`.
* **Five** end ANSam at the normal 7.4–7.5 s — so the peer *did* hear our CM —
  and are followed by weak, intermittent V.21 channel 2: at 50 ms resolution the
  1650/1850 Hz fractions sit at 0.0–0.2 with two bursts reaching 0.53–0.57, where
  a clean JM alternates near 0.5. Replaying one (`rf-trn1d-ab-194155Z/
  trn8004-r2`) the flow log reads **`Decoding single CM/JM candidate after
  timeout` → `JM recognised from single saved candidate`**: the calling-role
  fallback added to `V8_CM_ON` fires and recovers the JM, where before it went
  straight to `V8_STATUS_FAILED`.

**So that fix is vindicated on the second mode and is irrelevant to the first.**
An earlier note here said it had no demonstrated effect; that was measured only
against tone-mode calls. It cannot be shown to carry the call all the way to
V.8 success from these taps, because the live call was torn down at ~14.6 s and
the replay runs out of audio while the state machine is in `V8_CJ_ON` — but the
branch it replaces was an immediate failure, and it now sends CJ and proceeds.

### Ja failures start in PP, not in the descriptor parser

The RasFinder DIL must not be cached as a peer profile.  On the preserved call
that decodes Ja, Phase 3 reports a PP mean residual of **0.338**, locks TRN at
**507/512 descrambled ones (99%)**, and finds the CRC-valid Table 12 descriptor.
On a call that fills all 24 Ja hypothesis rings but finds no descriptor, PP is
already broken: residual **1.145**, TRN only **272/512 (53%)**.  Searching the
failed rings for the known fixed Table 12 header leaves 9–13 wrong bits in the
best 51-bit window.  This is corrupted symbol recovery, not a framing or CRC
edge case.

One generic conflict was real.  V.34 10.1.3.6 equation 10-1 gives PP points at
multiples of 30 degrees, while the generic primary-training carrier loop snaps
every symbol to the nearest four-point/QPSK target.  It was running at the same
time as the supervised PP equalizer and fighting its known PP target.  In the
V.90 receive path the four-point loop now stands down throughout PP acquisition
and conditioning; once PP is aligned, carrier correction uses that exact PP
target instead.  On the known-good RasFinder capture the PP residual improves
from **0.338 to 0.274** and the Ja descriptor remains CRC-valid.  The change is
V.90-only: applying it to ordinary V.34 regressed the 2800/21600 matrix row.
The full test suite and that row pass with the scope corrected.

This does not rescue the preserved bad capture (residual improves only to
1.082 and TRN remains 55%), so another PP acquisition defect remains.  Sweeping
all 48 PP target phases and the initial pulse-shaper phase does not produce a
healthy TRN lock; do not turn either into a peer-specific setting.

Live confirmation is `reference-data/artifacts/rf-pp-carrier-20260929-r1`: with no stored DIL
descriptor and no peer-specific Ja timer, the receiver parsed a CRC-valid
RasFinder descriptor (`N=192, LSP=120, LTP=120`) and started Sd from that event.
The peer later retrained while we were waiting for S during Jd, so this proves
the generic PP/Ja receive path on hardware but not data mode.


**RETRACTION and live result (2026-09-29, later).**  The correction above
raised `ME_V90_JA_HEURISTIC_FALLBACK_MS` to 1000 ms on the claim that the
peer "held Ja for 5.4 s", i.e. never detected our early Sd.  That was a
mis-aligned read of the RX tap: the 5.4 s stretch is **ANSam** (2100 Hz, the
same position in the working calls' taps).  Located properly, every call --
working and failing, early Sd or late -- shows the peer's Phase 3 burst, then
~3.7-4.2 s of silence after Ja: it DID detect Sd.  The change is reverted
(500 ms again); it had no evidence left.  Live at 1000 ms, two calls reached
Phase 3 (`reference-data/artifacts/rf-jafb1000-20260929-r2`, `-r3`) with Sd after the parse
and neither completed Phase 3, which agrees.

What the five RasFinder Phase 3 calls actually separate on, read from the RX
taps (S = 10.1.3.7's three lines, fraction of a 40 ms block):

| call | peer's first S | RTP pkts >20 ms late | fill frames in signal | after the quiet |
|---|---|---|---|---|
| rf-maxpow-c1 (S, Phase 4) | low 0.91 | 0 | 2 | S low 0.98 |
| rf-padrep-a1 (S, Phase 4) | low 0.93 | - | 2 | S low 0.98 |
| rf-pp-carrier-20260929-r1 | high 0.89 | - | 26 | unclear |
| rf-jafb1000-20260929-r2 | high 0.91 | 502 (max 269 ms) | 23 | **S high 0.98** |
| rf-jafb1000-20260929-r3 | high 0.89 | - | 27 | Tone A + guard (retrain) |

Two confounded differences: the peer ran Phase 3 on the high carrier, and
today's bearer had heavy jitter, so the near-zero jitter buffer concealed
~25 frames inside the signal (against 2).  **r2 is the sharp case: the peer
sent S, on the high carrier, and live we missed it; `v90_engine_replay --dial`
of the same tap detects it 14124 Jd symbols in and moves to J'd, robustly
(start offsets +0.02..+0.20 s all detect it, with and without flow logging).**
The receiver is fed directly by `me_rx_g711()` and the tap is written there,
concealment fill included, so the input was identical; the divergence is
live-only and not yet explained.  The replay interleaves RX and TX 1:1, which
live under this jitter does not -- that is the first thing to instrument.

## Why r2 missed a real S: the T/2 eye flip deferred by PP was thrown away (2026-09-29)

**Instrument.** `ME_IO_SCHEDULE=<path>` records every `me_rx_g711()`/
`me_tx_g711()` call (kind, count, CLOCK_MONOTONIC ns; 16 bytes through a
buffered FILE, safe on a live call), and `v90_engine_replay --schedule <path>`
feeds a tap back in that exact interleaving, paced on the recorded clock.
Replayed with its own schedule, `reference-data/artifacts/rf-iosched-20260929-r3` reproduces
the live call to within a few ms (Sd +18604 vs +18611).  What it showed: live
pjmedia hands RX over as **two 80-sample calls per 20 ms tick** and TX as one
160-sample call; the plain replay feeds 160/160.

**Reproduction.** On r2's tap with a synthetic 80+80/160 schedule the replay
misses S exactly as the live call did; with 160/160 it catches it.  Not the
TRN hypothesis (it does tie -- hyp 3 vs 8 both 100%, chunking picks which --
but pinning hyp 8 from the CRC-valid descriptor changes nothing).  The S
detector's own counters show the cause: at the peer's S the 80-sample run
reads 180-degree steps (dom=2 25/32, rev 26/32), the 160-sample run S's
+/-90 alternation (alt 30/32).  That is sampling at the eye crossing.  In
both runs the T/2 eye chooser measured "other phase" during PP and, under the
PP guard, logged it and **discarded** it; the next clear vote comes only from
the analogue modem's S itself (128T, half a 256-symbol window), so whether the
flip landed before S depended on window alignment.

**Fix.** The deferred flip is kept pending and applied at the first half-baud
after PP -- *only* in the V.90 digital modem's receiver
(`v90_mode && !calling_party`).  Plain V.34 measured the other way: applied
there it costs 2800/21600 u-law its whole payload (0 bits vs 16421), because
the move lands on an equalizer PP has just trained at the old instant.
`ME_V34_EYE_PP_DEFER=0` restores discarding.  A/B on the r2 80/80/160
schedule: fix -> S after 14124 Jd symbols, DIL, Ri; off -> "no S after 24796"
as live.  `make test` passes.  `rf-maxpow-c1` replays unchanged (S, DIL, Ri);
`rf-padrep-a1`'s smooth replay misses S with and without the fix (the flip is
never deferred there) -- a pre-existing replay gap in the other direction,
and without a schedule recording it cannot be replayed in its live order.
Live confirmation still owed; record new calls with `ME_IO_SCHEDULE`.

**RasFinder session 2026-09-29 (late): a late PP lock was ours; V.8 and early-Sd are open.** Fourteen calls. **V.8 failed on 7** with the known exactly-2250 Hz tone, and louder CM (`ME_V8_TX_POWER_DBM0=-10`) changed nothing: the peer's ANSam still ran its full ~5.3 s. Our transmit is **byte-identical up to CM** with the 09-28 calls that all passed V.8, and the CM start delay (1.1-3.8 s after ANSam) does not separate pass from fail over 47 calls, so this is rig-side. **Fixed: PP conditioning started late.** 10.1.3.6's PP is 288T and conditioning takes 232 of them; correlation onset to PP-start detection measures 27-43 bauds on every call whose PP residual is 0.2-0.4 and **79-150 on every call at 0.50-1.06** -- an eye flip during acquisition resets the phase lock (`locked 12` at detection in all three worst), so the 232 bauds run into TRN. Conditioning now starts at the excess (c499277f, V.90 digital RX only, `ME_V34_PP_ONSET_TRIM=0` disables; plain V.34 3200/21600 A-law breaks without that scoping). Replayed it improves all five late calls (padrep-b1 0.504 -> 0.341, which then reaches Phase 4 CPt) and leaves timely ones unchanged. **It does not fix `rf-0929s-p10-3`**, which fails PP at every trim, eye setting and whole-sample shift on a line `v90_phase4_upstream_grade.py` measures at 31.6 dB -- a second cause, open. **Open, not acted on: early Sd.** On `rf-0929s-t8k-4` the energy-gap Ja gate fired on the 80 ms TRN->Ja silence 0.7 s into Phase 3, the 500 ms bound released Sd 0.63 s into the peer's Ja, and the peer kept transmitting Ja ~4 s longer as if it never saw it; the descriptor parsed 3.5 s into Phase 3 (2.5-3.5 s on all 13 RasFinder calls that parse). That contradicts the 09-29 retraction ("the peer detected our Sd even when early") and n=1 cannot settle it. `ME_V90_TRN1D_SYMBOLS=8004` was tried live for Jd airtime (the peer retrains 4.3-4.4 s after TRN1d start when Jd has had <2 s) but no call reached a clean test of it. -- `docs/v90_rasfinder_phase4.md`

- **2026-09-29 evening: the V.8 "2250 Hz" failures are two things, one ours and fixed.** The 2250 Hz tone is the peer's own V.22 answer signal (unscrambled ones: 2400 Hz stepping -90 deg per 600-baud symbol) -- it abandoned V.8; not the ATA/PBX as earlier notes said. (a) **We missed JMs the peer sent** (v34-c1/d1/f1, rf-0929s-trim-3): the SIP RX jitter buffer, "fixed" with jb_*=0, was really pjmedia's ADAPTIVE default from 0 prefetch, so each arrival spike (149 ms, zero loss) ran it dry and inserted 160-1440 samples of fill before the late audio played -- breaking V.21 sync, and a timing step for every later phase. In most 09-29 calls, none on 09-28. Fixed 74563a6d: fixed 200 ms, no discard (`ME_JB_MS`, 0 = old); live A/B inserted runs 0/5 calls vs 6. (b) **The peer never hears our CM** (ANSam runs its full 5.4 s): CM bit-identical and same level on pass and fail, louder CM no help -- the path toward the peer. Late that evening the RasFinder stopped answering (ringback then silence in the RX tap); stop testing when that shows.

- **2026-09-29 night: dial the RasFinder hunt group 3999, and what V.8/Phase 3 now look like.** 3999 rings its three ports in turn, which exposed spandsp's ten-repetition CI cap (~10 s): V.8 failed on ringback before any port answered. V.8 now sends CI through ringback for up to ~60 s like a real modem (46544d05, `ME_V8_CI_REPEATS`; `ME_V8_ON_EARLY_MEDIA=0` waits for 200 OK instead). A CM restart after 1.5 s without JM was measured HARMFUL (V.8 1/6 vs 4/6) and ships off (`ME_V8_CM_RESTART_MS`). Remaining failures, both on the path TOWARD the peer: V.8 all-or-nothing per call (peer hears CM within ~1 s or never), and in Phase 3 the peer detects our Sd (Ja ends ~0.4 s after it) but never accepts Jd -- silent, then Tone A ~4.4 s into Jd (rf-cmr0-5), with our Jd CRC-clean on our own TX tap. The ATA's RTCP shows 0 loss and 13.6/25.9 ms avg/max jitter in our direction, so the packets arrive; suspect the ATA's modem-passthrough (VBD) switch -- voice-mode EC/NLP or adaptive playout toward the peer -- before any more receiver work.

- **2026-09-29 late night: the rig-side knobs, and our V.8 transmit audited -- it is correct, and no ghosts (without the fast-detect experiment).** Asterisk (22.5.1) is clean for this path: all four endpoints `allow=ulaw` only, `direct_media=no` but no transcode, no jitter buffer, `fax_detect`/T.38 off, no DENOISE/AGC/VOLUME. Its 3999 hunt only advanced on BUSY and had a malformed `GotoiF` at 8409; now it advances on anything but ANSWER (8416 -> 8423 -> 8409 -> 9898, 20 s each; backup `extensions.conf.bak-20260929`). On the VG224 the three hunt ports differed (2/16 EC+NLP off, 2/23 EC+NLP+CN off, 2/9 all voice defaults), nothing set `incoming called-number` so the modem dial-peer 8999 may not match inbound calls, and `modem passthrough nse` needs a Cisco peer to complete. User applied: 2/9 matched to the others, CN off, `incoming called-number .`, `no dtmf-relay rtp-nte`, and later 2/9 output attenuation -6 -> 0. **None moved anything**: V.8 4/6 (before, `rf-cmr0`) -> 1/6 (3999, all answered on 8416, `rf-ata`) -> 2/6 (8409 at 0 dB, `rf-8409-att0`); every failure is the full ~5.3 s ANSam then 2250 Hz, and every call reaching Phase 3 ends `no S after 24796 Jd`. The rig then degraded (0/6 controls, answered-then-dropped calls, 1 s billed). Still unverified on the ATA: whether passthrough actually engages -- `show call active voice` during a failing call is the check.
  **Our V.8 transmit, demodulated independently of spandsp** (own V.21 discriminator): CI `c1`, CM `c1 65 12 10 2a 47 8d` -- call function first, modulation octet with b5 set plus two extensions, LAPM, PCM availability (V.90/92 digital), PSTN access digital; syncs per Table 1; 980/1180 Hz; ~-14 dBm0. The peer's JM (`c1 65 12 10 2a 0d 0d 27`) decodes cleanly when sent. **The TX tap is byte-identical on pass and fail calls from 0 to 7.6-7.8 s**, past the peer's decision, so nothing our V.8 reacts to separates them. Timing: spandsp's ANSam/ detector (`modem_connect_tones.c`) reports only after THREE >=425 ms reversal cycles, 1.38 s after onset, so we send one more CI burst into ANSam (plus four CI ahead of CM) and CM starts ~2.4 s into ANSam, Te then 1 s -- all within V.8 8.1.1. Two A/Bs, both refuted: `ME_V8_NO_CI=1` (clean CM, no CI overlap) 0/4 vs 0/4 (`rf-noci-ab-*`); detecting after ONE cycle (CM ~1.3 s sooner) 0/6 vs 0/6 on calls that detected properly (`rf-ansamfast-*`) -- **and it ghosted on 2 of 8, firing on the ringback's decay (440/480 Hz, no 2100 at all) and sending CM 3 s before the real ANSam: the three-cycle rule is what rejects that, so it is a guard, not just caution.** Not kept. Conclusion: our V.8 is conformant, stable and identical call to call; the per-call variation is past our transmitter (ATA passthrough state, loop, or the RasFinder). Next evidence: the ATA's `show call active voice`, or another modem on the port. Also noted: Phase 3 levels differ from the Eicon reference by ~12 dB -- we send TRN1d at the peer's requested U_INFO 78 (RMS 3772) where the Eicon ignores U_INFO and uses W=64/TRN1d 48 (RMS 924); Jd fields match the Eicon's except lookahead (1 vs 3). `v90_analogue_rx_test --trace` now prints the last valid Jd frame's 72 bits.

## 2026-09-30: from a wired host, Phase 3 had three defects of ours

Calls placed from tower (wired, see `docs/v8_conformance_audit.md`) pass V.8
every time and expose Phase 3 without the Wi-Fi jitter that masked it.

1. **The p3_demod Ja scanner starved the media thread.** It ran a PP-trained
   p3_demod over 800 ms for both upstream rates and both carriers every
   80 ms -- ~40x real time of demodulation.  On tower that is 2-3 s of CPU
   per second of Phase 3; pjmedia dropped ~200 received frames (4 s) in the
   middle of Phase 3.  Proven with a pcap: RTP arrived complete and in
   order, the RX tap was 213 frames short, and the Ja descriptor reached the
   receiver as 80 ms chunks out of order (each chunk matching a CRC-valid
   descriptor from an older call 100%, at shifted offsets).  On this Mac the
   same scanner cost ~0.8x real time.  Now one pass (INFO1a's rate, the
   receiver's carrier) every 160 ms; `ME_V90_P3_JA_SCAN=full|0`.  Replay CPU
   for 15 s of audio: 2.53 s -> 0.40 s (Mac); tower Phase 3 6.9 s -> 0.3 s
   for 3 s of audio.  SmartLink recordings reach V.90 data mode either way.
2. **The peer's 9.3.2.7 S comes on the other carrier.**  It starts ~1.9 s
   into our Jd -- in time -- but on the high carrier (0.98 of the energy on
   the 320/1920/3520 Hz lines) while our receiver can be on low.  The
   constellation-domain S detector missed it on three calls of four, Jd
   expired after 24796 symbols and the peer retrained.  New
   `v34_rx_watch_v90_jd_s()` looks for S on the line on both carriers while
   the engine is transmitting Jd (`v34_v90_arm_jd_s_watch()`);
   `ME_V90_JD_S_WATCH=0` disables.
3. **The T/2 eye chooser moved the symbol instant straight after PP** on
   calls where PP locked late (low carrier; residual 0.65-0.68), landing on
   the equalizer PP had just trained, and the Ja then never demodulated.  The
   chooser is now held from PP start until Ja is accepted (V.90 digital RX
   only); after Ja it runs again, because on high-carrier calls its flip
   during DIL is what lets Phase 4 decode the CPt.  Ja parses on all 12 RasFinder
   recordings tried (tower and Wi-Fi era, both carriers), was 10;
   `ME_V90_P3_EYE_AFTER_PP=1` restores the old behaviour.

Live from tower with 1 and 2: 3 of 3 calls reach Phase 4 CPt.  Open: Phase 4 --
the peer sends CPt, we send TRN2d and MP, it retrains without sending CP.

## 2026-09-30 (later): Phase 4 from tower -- the retrain is a fixed deadline, our TRN2d/MP is conformant, and one CPt defect of ours

**The peer's retrain is pinned to TRN2d start.**  From tower, every call that
reached TRN2d retrained at **TRN2d start + 2240 ms (+/-1 ms)**, whatever we
varied:

| varied | values | retrain after TRN2d start |
|---|---|---|
| TRN2d length (`ME_V90_TRN2D_SYMBOLS`) | 2040T, 12000T, 12000T (MP from +240 / +1500 ms) | 2241, 2241, 2240 ms |
| MP upstream offer (`ME_V90_UPSTREAM_MAX_BPS`) | 31200, 19200, 19200, 9600 | 2240, 2240, 2241, 2240 ms |
| shaper convention | `ME_V90_SHAPER_METRIC=codec`, `ME_V90_SHAPER_LD=0` (new, test-only) | 2241, 2241, 2241 ms |

At K=12 an MP is ~4.5 ms, so the 2040T call sent ~390 MP frames before the
deadline.  The peer never acted on one.  It does detect our R-bar-i: calls
where we never sent it (CPt missed) retrain 4.92 s after Ri instead.
Allowing for the 267 ms tap round trip (echo delay below), the peer's own
interval is ~1.97 s -- 9.4.1.3's 2000 ms for MP.

**Our TRN2d/MP is conformant, by a decoder that shares nothing with v90.c.**
`tools/v90_trn2d_spec_decode.py` is a V.90 5.4 receiver written from the
Recommendation text (descending labels, modulus decode, Sr=1 sign recovery
through Tables 3/4 and the Figure 2 trellis without knowing the shaper's
rule, GPC descrambler, Table 16 framing, the reflected CRC that validates the
peer's own CPt).  On the live transmit tap of `rf-tower-t2-2040-10715`:
TRN2d **100.00% ones over 5780 bits**, **444 of 444 MP frames CRC-valid**
(Type 0, drn 13, mask 0x0fff).  Negative control: starting one sample late
reads 62.7% ones and no MP.  So SmartLink reaching data mode was not a
lenient peer forgiving a wrong convention -- the encoding is right against the
text.  The two CPt frames compared field by field differ only where
expected: RasFinder drn 9, six identical 4-point constellations
{61,84,96,102}, codec-output set {47,69,81,87} (bit 128, a real ~6 dB loop
loss); SmartLink drn 15, one 8-point constellation, no pad.  Both Sr=1, ld=1,
a1=1.0, a2=b1=b2=0.  No shaper ties either (`V90_SHAPER_TIE_LOG=1`: none in
~2000 frames; the two first-rule metrics typically differ ~4x).

**The ATA carries it intact.**  The VG224's hybrid echo in our RX tap is a view
of the downstream as played onto the line.  A 48-tap fit over Phase 3 Jd
(peer silent) explains 99.7% of the received power at a delay of 2136
samples; projected through that same filter, the TRN2d/MP-era echo has gain
**1.00 +/- 0.07** at the **same 2136-sample delay**.  Linear, no playout
slip between Phase 3 and Phase 4.

So the peer receives a conformant, unslipped TRN2d/MP and still does not find
MP.  What remains is on its side of the loop or a convention outside the
text; nothing we have varied moves it.

**Fixed: CPt missed on a third of calls.**  On 3 of 9 tower calls the peer sent
CPt -- `rf-tower-shp-codec-5136` carries **11 CRC-valid CPt frames** by the
independent checker -- and our receiver accepted none, so the peer never saw
R-bar-i and retrained out of Ri.  Offline it only reproduced with
`v90_engine_replay --split` (new), which feeds RX as the two 80-sample calls
per tick a live pjmedia call makes; fed whole 160-sample frames the same
audio is accepted.  Two defects:

1. The T/2 eye chooser's per-call cap of four flips was spent by the end of
   Phase 3, one of them on a half-silent window (sums 45.7 vs 43.5), so the
   CP stage could not correct its phase.  And its magnitude vote is biased
   there: the equalizer is frozen from Phase 3 at the current phase.  In
   `V34_RX_STAGE_V90_CP` the chooser now judges each phase by the
   *differential* angle to the 90-degree grid (CPt is 4-point DPSK), leaves a
   phase under 15 deg rms alone, and gets two flips of its own on entry
   (`ME_V90_CP_EYE_ANGLE=0` restores the magnitude vote).
2. On the call where that half-silent flip landed during Phase 3 training,
   the saved taps leave BOTH phases white (26 deg) at the CP stage.  No
   symbol instant rescues it; letting the CP-stage taps adapt on the
   constant-modulus CPt does.  `ME_V90_CP_ADAPT_STARTUP` is now default ON
   (`=0` restores the freeze).

Six calls x {whole, split} feeds, CPt accepted per cell (deterministic, 3/3
or 0/3 in triplicate): old defaults 8/12, angle only 11/12, angle + adapt
**12/12**.  Resetting the equalizer instead of adapting fails.  Live on the
new defaults the first call accepted CPt 1.28 s into Ri -- and then retrained
at TRN2d + 2241 ms like every other.

## 2026-10-17: Phase 3 media-callback stalls were the DIL hypothesis sweep

Three RasFinder calls in parallel from CT 117 on vmhost (`tools/soak/rf_par.sh`-style: own SIP/RTP port, PTY and
account per lane, 6311-6313) logged 20-150 ms media callbacks in `v34_rx` stage 10 (PHASE3_WAIT_S), which move
the AudioCodes playout (see 2026-10-16 above). `modem_engine.c` now times four sections of `me_rx_audio` and
appends them to the "slow media callback" line (`[lec .. v34_rx .. ja .. dil ..]`). Result: the Ja scan was
already on its worker (0 ms); **`v90_dil_capture_try_v34_hypotheses()` ran its 24-hypothesis search inline,
16-90 ms, growing with the captured Ja bits**, every 512 new bits. It now does `ME_V90_DIL_HYP_PER_CALL`
(default 1) hypotheses per callback, same order and logic, cursor reset per call. Replays (hypothesis 8 recovered,
same downstream outcome on four taps) are identical; live worst callback 90 -> 28 ms, 6 of 9 calls with none over
20 ms. Traps hit on the way: `v90_engine_replay` forces `ME_V90_P3_JA_SCAN_SYNC=1`, so it shows an inline Ja scan
the live server does not have (override with `=0`); the clang-only `-O2` for v34rx.c needs `make CC=clang`
(CT 117 builds with gcc by default and had no `.v34-opt-cflags`); CT 117's checkout had local edits that made
`git pull` abort silently for the whole session (the build looked fresh).

Phase 3 "peer retrains with Tone A" (same day, no stalls): a rejected S is a real Tone A, not a mis-read S. On
connecting calls the peer's S arrives near sample 102-105k at rx_rms 1.4-2.5k with band2400 0.01-0.03; on failing
calls a loud signal (rx_rms 4.7-5.2k, band2400 0.43) arrives at the same moment, i.e. the peer abandons Phase 3 and
retrains instead of sending S after our Jd. Not explained by: Ja time (+11.9..20.6 s on both), TX pacing
(rtp-tx.csv gaps identical good vs bad), TX clock (-4..-74 ppm on both), parallel vs solo (2/6 solo, ~1/3 of
45 parallel), host (Mac mini 1/6, CT 117 2/6 same hour). **Not a regression:** interleaved A/B on CT 117,
solo, alternating arms: current main 4/6 V.90 data (3 with the MULTITECH login prompt) vs d99f144d 0/6.
So V.90 data yield on this rig swings 17-67% by the hour on the same code; judge changes on 12+ interleaved calls.
Artifacts: CT 117 `/root/ab`, `artifacts/rf{p,q,c,d,e,f,s}-*`; `tools/soak/rf_par.sh` (3 lanes) and `rf_serial.sh`.

### 2026-10-17: is it our Phase 3 timing? (§9.3.2.7's 4500 ms) -- shorter TRN1d does not help

Our TX tap is **byte-identical** between a call whose Jd the peer accepts and one it rejects, from Sd through the
first 1.7 s of Jd (cur-2-6312 vs cur-1-6311, 800-byte chunk search; only the S-terminated end differs), and TX
pacing in the first 1.6 s of Jd is the same (9.8 irregular packets mean, accepted and rejected). The peer's
reaction times cluster in 20 ms steps: S accepted at Jd+1.70-1.82 s or +2.34-2.42 s, Tone A at +1.8-2.0 s, +3.9 s,
+4.45 s. Sd+S-bar+TRN1d (2.54 s) + the first Jd decision (1.7 s) is ~4.25 s of the 4.5 s the analogue modem has
from the end of Ja, so a deadline was the obvious suspect. Tested: ME_V90_TRN1D_SYMBOLS 12000 / 16008 / 20004,
three lanes in parallel, arms rotated over 6311-6313, 8 calls per arm (`tools/soak/rf_trn1d_ab.sh`):
S accepted first 2/2/4 of 8, Tone A first 3/3/2, V.90 data 2/1/4. **More margin did not help; the default 20004T
was best**, so the deadline hypothesis is not supported (n=8 per arm; direction, not proof). Untested: longer
TRN1d (24000/28800), which would test the opposite reading (the peer's equaliser wants more training).

Longer TRN1d, same method (24000 / 28800 / 20004, 8 calls per arm): S accepted first 5 / 4 / 3 of 8, Tone A first
1 / 2 / 2, V.90 data 3 / 1 / 1. Pooled over both sweeps, S accepted first by TRN1d length: 12000 2/8, 16008 2/8,
20004 7/16, 24000 5/8, 28800 4/8. A weak upward trend to 24000 (25% -> 62%) but every arm is within one binomial SD
(~17 points at n=8) of its neighbours, so this is **not** a demonstrated lever; it would need ~30 calls per arm to
separate 44% from 62%. Raw per-call results: CT 117 `/root/trn`, `/root/trn2`.

### 2026-10-17: Phase 3 transmit level A/B -- lowering it is clearly worse

Our Sd/S-bar/TRN1d/Jd run at the peer's INFO1a U_INFO (78, RMS ~3772), ~12 dB hotter than the Eicon reference
(W=64 / TRN1d 48, RMS 924). New diagnostic `ME_V90_P3_UINFO_DELTA` (Ucodes subtracted from U_INFO; 16 = one u-law
segment = 6 dB). Three lanes in parallel, arms rotated over 6311-6313, 12 calls per arm, CT 117 `/root/lvl`:

| delta | U_INFO | S accepted first | Tone A first | no S/Tone A at all | V.90 data |
|---|---|---|---|---|---|
| 0 | 78 | 4 | 2 | 6 | 4 |
| 16 (-6 dB) | 62 | 1 | 9 | 2 | 1 |
| 30 (-12 dB) | 48 | 0 | 1 | 11 | 0 |

Monotonic and well outside noise at n=12 (S accepted 33% / 8% / 0%): **the peer wants the level it asked for or
more, and the Eicon-style low TRN1d does not carry over to the RasFinder.** Next lever: negative deltas (hotter).

Hotter, same method (delta 0 / -8 / -16 = U_INFO 78 / 86 / 94, 12 calls per arm, CT 117 `/root/lvl2`):

| delta | U_INFO | S accepted first | Tone A first | no event | V.90 data |
|---|---|---|---|---|---|
| 0 | 78 | 8 | 1 | 3 | 3 |
| -8 | 86 | 6 | 2 | 4 | 3 |
| -16 | 94 | 6 | 1 | 5 | 0 |

**No gain above the requested level**: the default sits on a plateau (lower is monotonically worse, hotter is flat
within noise; V.90 data drops to 0/12 at +12 dB, probably Phase 4 headroom, not examined). The same arm (delta 0)
scored S-first 4/12 in the earlier batch and 8/12 here, 1.5 hours apart: **the rig's Phase 3 pass rate drifts by
2x between batches on identical code**, so only interleaved arms within one batch are comparable. Phase 3 level is
closed as a lever; the transmit side has now been shown identical, conformant, pace-neutral, and level-neutral
above U_INFO.


## 2026-10-20: current working-tree modulation checks

See `rasfinder_modulation_tests_20261020.md` for the 12-call wired CT 117
matrix over 6311–6313, including source/binary provenance and retained taps.
Direct V.34 reaches LAPM/login on 3/3 at 24000 (6312 later loses decoding);
V.22bis on 2/3 at 2400. V.32bis times out training on 3/3. V.90 delivers
login at 48000/21600 on 6313 and holds about 61 s without a logged resync;
6312 reaches 45333/21600 but decodes white and follows a peer retrain, while
6311 reaches login only through V.34 fallback. No improvement in V.90
success rate is demonstrated by this small historical comparison.

## 2026-10-22: cac07c32 on vmhost, default-rate V.90 offers

**No usable V.90 connection in six completed calls.** One call enters
V.90 DATA at 46666 downstream / 31200 upstream but never establishes LAPM
or delivers the login banner. Two other calls reach login only after
V.34 fallback; one carries all ten checked echoes, the other loses decoding
after the first. These are current-revision hardware results, not an A/B or
evidence that the Phase 2 transmitter corrections improved or regressed
acquisition.

### Build and method

Calls ran on wired vmhost CT 117, 22 October 2026, 14:41–14:45 NZDT.
The isolated source-only checkout is `/tmp/rf-cac07c32-20261022`, at
`cac07c32fbb3400367473733f50e2940f6205d2a`; existing checkouts and services
were left alone. The container's root disk had only about 296 MB free,
so the build and captures use its memory-backed `/tmp`. They have been
copied to retained local storage below.

The build uses vendored SpanDSP and pjproject, clang, and the normal
selective `v34rx.c` optimization. Fresh-checkout prerequisites were handled
before compiling the application: configure the vendored libraries, supply
Automake's missing `INSTALL` document, generate the V.22bis CC filter
headers before the optimized V.34 RX target, and retain the configured
Linux host stamp so a retry does not delete those headers. Automake
regenerated two tracked `Makefile.in` files; their patch is retained.
`git diff --exit-code -- '*.c' '*.h'` passes: there are no DSP source edits.
The focused `v34_phase4_16pt_test` passes on this Linux build before calls.

Tested application SHA-256:
`5b70c853b1e9ef3599f5ad133cb8f2440eb2c39415ee227c40c917a9f66c648e`.

Two rounds each run three parallel calls, one per 6311/6312/6313, with
accounts 2901/2902/2903, SIP ports 5090/5092/5094 and RTP ports
15000/15100/15200. Each lasts 90 seconds after ATD; the rounds are separated
by 90 seconds without calls. PCMU is forced, `AT+MS=V90,1` selects the offer,
LAPM XID compatibility is automatic, and **upstream rates use defaults**:
the earlier 21600 diagnostic ceiling is not set. No live SpanDSP flow log
is enabled. All six owned server processes terminate cleanly, return code 0.

After CONNECT and a readable `login:`, the exclusive PTY driver sends up
to ten distinct 16-byte ASCII tokens and checks each against newly received
bytes. Backspaces erase each token before the next one. No CR or credentials
are submitted. This grades short application round trips, not raw BER,
uncompressed throughput or a bulk transfer. LAPM may conceal corrected
physical errors; it cannot make an absent echo pass this check.

### Completed calls

| Call | Result |
| --- | --- |
| r1 / 6311 | V.90 DATA at +24.952 s, 46666/31200. No DTE CONNECT, LAPM or login. Upstream discontinuity recovery and peer retrains follow. |
| r1 / 6312 | V.34 fallback; initial 31200 B1 rejected, then `CONNECT 24000/ARQ/V34/LAPM/V42BIS`. Login and 10/10 exact 16-byte echoes (160 bytes), from dial +40.5 to +75.3 s. No logged decoding loss before our termination. |
| r1 / 6313 | No DTE CONNECT/login. Retrains; a V.34 DATA entry at 28800 is rejected by the B1 SNR gate. |
| r2 / 6311 | No DTE CONNECT/login. A V.34 DATA entry at 31200 is rejected by the B1 SNR gate; further training fails/retrains. |
| r2 / 6312 | V.34 fallback, `CONNECT 24000/ARQ/V34/LAPM/V42BIS`, login. First 16-byte echo passes at dial +40.2 s; next two fail. Receiver stops decoding, attempts §11.6 recovery then §11.5 retrain; a later 19200 DATA entry also loses decoding. |
| r2 / 6313 | No DTE CONNECT/login. V.90 training times out and falls back to V.22bis without connecting before termination. |

### Our transmit tap and bearer limits

An independent 1200-Hz DBPSK demodulator checks the first 20 seconds of
each **our-TX** tap, then applies the V.90 Table 7/9 CRC, fill and field
layouts. All six contain a CRC-valid INFO0d declaring -10 dBm0, PCMU and
final fill `1111`. Recovered contiguous INFO1d frames have valid CRC/fill
and **109-bit spacing**, as required by 8.2.3.1. Some groups cross the
20-second analysis boundary or have a frame not recovered; this is not
a claim to have decoded every transmitted repeat. Large gaps between
separate recovered groups are retained in the JSON, not mislabelled as
padding. The expected corrected wire contracts are present in real output.

Across 21,818 received and 26,601 transmitted RTP packets, every recorded
payload type is 0 (PCMU), with no sequence or checked timestamp
discontinuities. Maximum local packet spacing is 30.75 ms RX and 54.21 ms
TX. These local traces do not prove byte-exact transport at the far end or
rule out downstream loss/playout changes. Callback pacing remains a live
concern: r1/6311 spends 32.7 ms in an RX callback during B1 acquisition.

On that V.90 DATA call the pre-MP estimate is 35.6 dB, so the default
offer remains 31200. The T/3 B1 template's best fit is only 22.6%; the
fallback acquisition publishes data, but LAPM never establishes and V0
parity/discontinuity recovery follows. This identifies upstream acquisition
and recovery as unfinished work; it does not establish that rate policy,
bearer pacing or the RasFinder caused the failure. The old October 20 batch
used an upstream cap and a different source snapshot, so it is not a control.

Retained local evidence:
`reference-data/artifacts/rf-cac07c32-20261022/` contains the manifest,
tested binary, dependency/application/focused-test build logs, generated
build patch, per-call PTY bytes, both G.711 taps, both RTP CSVs, interactions,
call runner, independent analysis and TX-frame checker, results and hashes.
These recordings are retained hardware evidence, not added regression fixtures.
