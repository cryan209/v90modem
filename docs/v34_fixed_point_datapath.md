# The fixed-point V.34 datapath

The V.90 upstream receiver and the plain V.34 receiver share one per-symbol
chain.  On the desktop it runs in float; this note records the integer port of
that chain -- what is converted, what deliberately is not, how to build and
compare the two, and the measurements that decided each of those choices.

The motivation is a part without an FPU.  The load is dominated by the T/3
front end: 97 complex taps at 9600 Hz is 931k MAC/s, an order of magnitude
above the FSE's 67k, so the receive RRC is the kernel that decides whether
this receiver fits at all.

## Build

The flag changes both `v34rx.c` and `spandsp/private/v34.h`, so the two modes
cannot share objects -- the makefile targets do a full rebuild of spandsp as
well as the tree, and a stale `.o` here is the usual "impossible values at
runtime" failure this repo's makefile invites (see `CLAUDE.md`).

```bash
make fixed     # integer datapath
make float     # back to the default everything else assumes
```

`.build-mode` records which is current.  Each binary reports its own datapath
at startup:

```
[ME] V.34 datapath: fixed point
```

That line is read back **from libspandsp**, not from the flag the application
object saw, because `-DV34_FIXED_POINT` arriving through `CFLAGS` can leave the
library and the application disagreeing with no other symptom.  Any A/B of the
two arms should assert on it; `tools/soak/v90_datapath_ab.sh` does.

## What is integer, and what is not

Under `V34_FIXED_POINT`, in `v34rx.c`:

| site | kernel | format |
|---|---|---|
| NCO / carrier mix | incremental phase | integer phase accumulator |
| receive RRC | 97 complex taps at 9600 Hz | Q1.30 taps |
| ring conversion | once per **input** sample | block-floating (see below) |
| FSE | 21 taps per symbol | taps Q1.30, accumulator Q1.46 |
| NLMS tap update | decision-directed | wide accumulator |

The NCO is integer for a reason beyond cost: the float path recomputes the
angle from a running output count that reaches ~5.8M on a 600 s call, which a
float mantissa cannot hold -- those lines are `double` precisely because of
that, and an incremental integer phase is both cheaper and better conditioned.

**The B1 supervised least-squares solve stays in `double`, and that is a
measured decision, not an omission.**  Over three real 42x42 systems dumped
from a live acquisition the condition number is 2.8-2.9e5 -- 18.1 bits.  The
bit budget then leaves nothing: 63 bits, minus the pre-divide shift, minus
~11 bits of intra-solve growth, minus 18 bits of conditioning, is ~6 bits of
solution accuracy at the best usable format (Q24; above it the `a << SHIFT`
before the divide overflows int64, below it there are too few fractional
bits).  Measured against a float residual of 1.3e-13:

```
shift   relative residual   |x_fixed - x_float| / |x_float|
Q20     9.18e-03            5.4e+01
Q24     5.73e-04            3.1e+00
Q28     7.09e+03            1.3e+04
```

**Read the second column, not the first.**  Q24's residual of 5.7e-4 reads as
tolerable while its taps differ from the float solution by 310% -- an
18-bit-conditioned system has many `x` that fit B1 in-sample, and whether such
a tap set generalises is exactly what the receiver's own B1 out-of-sample
check exists to catch.  Judging this by residual alone would have shipped a
plausible number and a broken equalizer.

The hybrid is the engineering answer: per-symbol work integer, this solve in
double.  It runs once per acquisition -- 24 times in a 600 s call -- so it is
not on the path the FPU-less part cannot afford.

## Block floating point

A baked-in Q17.14 for the sample ring matched float on four of the five
recorded rate-matrix calls and regressed on `rate19200-r1`.  The cause is
dynamic range across the corpus, not across a call: formats derived from one
recording, where mean symbol power is 740, against a corpus spanning 740 to
1.75 -- 21:1 in amplitude.  The ring's binary point is now chosen **per
acquisition from the peak actually present in the ring**.  Taps stay Q1.30
(they scale inversely, and the largest seen reaches ~0.12 against that
format's limit of 2) and the FSE's output shift is unchanged, because the
product lands at Q(rshift) whatever rshift is.

```
call        FLOAT                 FIXED Q17.14          FIXED + BFP
19200-r1     820 / 0.383 / 40.2%  1109 / 0.656 / 22.1%   820 / 0.383 / 40.2%
24000-r1     578 / 0.038 /100.0%   578 / 0.038 /100.0%   578 / 0.038 /100.0%
26400-r1    1180 / 0.619 / 35.9%  1169 / 0.656 / 36.4%  1180 / 0.619 / 35.9%
28800-r1    1607 / 0.660 / 24.0%  1608 / 0.661 / 23.8%  1607 / 0.660 / 24.0%
31200-r1    2716 / 0.664 /  2.0%  2716 / 0.666 /  2.0%  2716 / 0.664 /  2.0%
```

(windows / median sym err / clean.)  That table is from commit 814eab3a.  It
does NOT reproduce: re-run against the tree it shipped in, the float arm
matches its float column on all five rows while the fixed arm reads
1117 / 0.658 on 19200-r1 -- the pre-BFP number.  The cause was the tap-copy
defect in the next section, not the ring format; BFP is still right and still
needed, but it was never what made 19200-r1 match.  Re-run a claim like this
against the tree that ships it.

## The integer taps are a SEPARATE COPY, and everything else must say so

The integer FSE does not read `v90_t3_fse[]`.  It reads `v90_t3_fse_fx[]`,
narrowed from the wide accumulator `v90_t3_fse_acc[]`, which is seeded from
the float array **once** -- at prime time, under `v90_t3_fx_primed` -- and
thereafter advanced only by the integer NLMS.

So any other code that replaces the float taps has no effect at all in a
fixed-point build unless it clears that flag.  There are five such writers,
and every one of them is a recovery mechanism:

- the equalizer restore, `memcpy(v90_t3_fse, v90_t3_fse_good, ...)`, three sites
- the blind CMA loop
- a fresh B1 acquisition, `memcpy(v90_t3_fse, best_coeff, ...)`

Until this was fixed, **the fixed-point receiver had no recovery at all** --
not a degraded one, none.  `v90_t3_fse_taps_replaced()` now clears the prime
flag at each of those sites, so the next symbol re-seeds the integer taps and
re-chooses the ring's binary point from the level now present.  It is a no-op
in a float build.

**The defect is invisible to every metric taken at acquisition.**  On
rate19200-r1 the two arms are identical there -- same sample 137539, 100% fit,
out-of-sample 0.015, symbol power 52.0, because the B1 solve is `double` in
both -- and they track each other to 0.01 of the lattice for nine tenths of
the call.  What differs only appears once something has gone wrong and the
receiver tries to fix it.  Per 20000-symbol block:

```
float  0.10 0.10 0.12 0.12 0.11 0.11 0.11 0.11 0.11 0.33 0.69 0.74 0.18 0.10 0.10 ...
fixed  0.12 0.10 0.12 0.12 0.11 0.11 0.11 0.11 0.11 0.38 0.77 0.76 0.76 0.76 0.76 ...
```

Both meet the same disturbance in the recording; float recovers and fixed
stays white for the remaining 150 s.

**Read a whole matrix before naming a cause.**  26400's fixed arm scored
BETTER than float (0.190 against 0.619) for exactly the reason 19200 scored
worse: a receiver that never restores keeps a filter float would have thrown
away, which flatters the row where the restore was the wrong call and destroys
the row where it was the right one.  A one-sided reading of that table sends
the search after an arithmetic defect that does not exist.

## Confirmation on a set that was not used to find the defects

The rows above are the recordings the debugging ran on, so on their own they
cannot rule out having tuned to them.  `goal-matrix-115515Z` also holds a
second capture of each rate, used for nothing during that work.  Both arms
built from the same tree, both pinned to the same handover per row:

    row               float          fixed          delta
    19200-r2@349.5     460/0.013      460/0.013     +0.000
    21600-r2@468.0    2068/0.664     1986/0.664     +0.000
    24000-r2@22.5     1107/0.656      973/0.617     -0.039
    26400-r2@86.5     2304/0.666     2304/0.668     +0.002
    28800-r2@22.5     1138/0.670     1138/0.669     -0.001
    31200-r2@86.5     2428/0.666     2451/0.666     +0.000

21600 does not appear in the r1 set at all, so that row is a rate the fixes
were never measured against.  **19200-r2 is the one to read**: the rate whose
r1 recording carried the entire regression decodes at 0.013 here, identically
in both arms.

**But most of this table cannot demonstrate a correct decode, and a delta of
+0.000 does not mean what it looks like.**  Four of the six rows sit at
0.664-0.670 in BOTH arms -- that is white (0.667 is the figure for symbols
unrelated to the lattice), so those rows say the two datapaths agree, not that
either works.  A receiver that fails exactly as float fails still scores
+0.000.  The rows carrying information are 19200-r2 and 24000-r2.

## Two method notes worth more than the fix

**A kernel test that passes is not evidence the kernel is right in situ.**
The synthetic LMS test *cannot* reproduce the Q17.14 failure -- sweeping its
amplitude 1000/250/50/20 gives identical results at Q17.14 and at a chosen
rshift, because at every level in that test Q17.14 still has ample bits.  It
passed at 114 dB while the receiver it feeds collapsed.  The confirmation had
to come from `rate19200-r1` itself.

**Keep the control arm and re-run it.**  The first attempt at parameterising
the LMS shift computed `>> -4` at rshift=14 -- undefined behaviour that
quietly destroyed the previously working case (residual 2.3e-09 -> 2.0, taps
114 dB -> 60 dB).  It showed up only because the R=14 control was re-run;
without it, it would have shipped.

## Live against the rig

The offline matrix is deterministic and is the instrument that finds defects.
The live A/B answers a different question -- whether the integer receiver
carries a real call -- and it does.  A fixed-point build reaches V.90 data
mode on the first Phase 4 attempt at 31200 upstream / 52000 downstream, zero
retrains, B1 fit 100%, and delivers ~21000 intact upstream lines.

Two alternated runs against the SmartLink rig, five calls a side, one binary
per arm, `ME_*` untouched.  Every call in both runs reached data mode, so
**the arithmetic does not affect the handshake**; what it affected was whether
a call that lost the eye ever got it back.

    run                       fixedpt                    floatpt
    before the two fixes      355.0s / 47875 lines       566.5s / 101697 lines
    after                     558.3s / 90860 lines       423.1s / 66467 lines

**Do not read the reversal as the integer path being better.**  It is not, and
the honest conclusion is that the two are no longer distinguishable here: the
float arm of the second run contains a 14.0 s call that delivered nothing, and
one bad call moves a five-call total more than any arithmetic does.  What the
rig contributes swamps what the datapath does, in both directions.

The mechanism is better evidence than the totals.  Before the fixes the fixed
arm's bad calls were TERMINAL -- 0.0 s clean, or one clean run and then white
for the rest of the call -- because nothing could put the equalizer back.
After them its imperfect calls break and RECOVER (109.8 s clean over an 86.3 s
longest hold; 104.9 s over 83.1 s), which is the restore, the blind loop and
re-acquisition doing their job.  The float arm shows the same shape, i.e. both
arms are now simply experiencing the line.

Read clean TIME and intact U-lines, never byte percentages: an unlocked
receiver emits garbage, which inflates the byte total on exactly the calls
that delivered least.

## Comparing the two

Offline, on a recording:

```bash
make fixed-compare REC=artifacts/<run>/live-rx.g711
```

The arithmetic differs, so the logs cannot be identical -- what has to match is
the acquisition structure (B1, E, DATA, Ja counts) and the decode quality
(median symbol error, bad shell frames).

Live, against the rig:

```bash
make fixed && cp sip_v90_modem /tmp/v90-datapath-bin/sip_v90_modem.fixedpt
make float && cp sip_v90_modem /tmp/v90-datapath-bin/sip_v90_modem.floatpt
tools/soak/v90_datapath_ab.sh artifacts/datapath-ab-$(date -u +%H%M%SZ) /tmp/v90-datapath-bin 3 3
```

Arms alternate rather than run in blocks, so a drifting rig cannot masquerade
as an effect, and each call asserts from the server log that the arm which ran
is the arm that was asked for.
