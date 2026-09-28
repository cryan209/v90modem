# RasFinder Phase 4: CRC-anchored measurements, 2026-09-28

V.90 data mode is not established. The RasFinder accepts our Phase 3, sends
CPt, terminates that group after our barred Ri, then retrains while we repeat
MP. V.90 9.4.2.2 permits SCR after CPt; 9.4.2.3 then requires CP. We have not
reliably recovered a data-mode CP from this peer.

## Correcting the receive diagnosis

The previous claim that an independent demodulator reads the failing region
at 5.75 degrees was measured at RX-file 28–30.4 seconds in
`artifacts/rf-maxpow-c1/live-rx.g711`. That is **after** this call's Phase 4
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
  artifacts/rf-maxpow-c1/live-rx.g711 21 25.4 \
  --tx artifacts/rf-maxpow-c1/live-tx.g711 --echo-window 23.5 25.2
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

Captures are under `artifacts/rf-cp-investigation/` and remain local evidence,
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

## Our Phase 4 transmit is conformant on the failing call itself

`v90_analogue_rx_test --phase4-trace artifacts/rf-maxpow-c1/live-tx.g711
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
