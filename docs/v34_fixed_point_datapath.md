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

(windows / median sym err / clean.)  All five match float exactly with BFP,
window count included.

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
