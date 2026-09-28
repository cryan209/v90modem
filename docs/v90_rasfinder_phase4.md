# RasFinder Phase 4: CRC-anchored measurements, 2026-09-28

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
on `artifacts/rf-maxpow-c1/live-rx.g711`), SCR windows being those reading
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

Live confirmation is `artifacts/rf-pp-carrier-20260929-r1`: with no stored DIL
descriptor and no peer-specific Ja timer, the receiver parsed a CRC-valid
RasFinder descriptor (`N=192, LSP=120, LTP=120`).

**Correction: Sd did NOT start from that event.** The `[TRACE]` lines put Sd
at +16733 ms and the parse at +17063 ms: the 500 ms
`ME_V90_JA_HEURISTIC_FALLBACK_MS` bound (anchored at the energy-gap
suppression, +16231) fired first, 332 ms before the descriptor landed.  The
RasFinder never saw that Sd: the RX tap shows it transmitting Ja
continuously for 5.4 s (RMS ~4000, no post-Ja silence) and then retraining,
while our Jd ran its full 24796T §9.3.1.5 budget.  It is lenient about
§9.3.2.4's 1500 ms (it held Ja far past it) but evidently does not arm its Sd
detector that early.  Both earlier RasFinder calls that got S (`rf-maxpow-c1`,
`rf-padrep-a1`) started Sd *after* the parse.  The default is now **1000 ms**:
it covers this call's 832 ms, and it is the most §9.3.2.4 allows once the
anchor's lag behind Ja start, Sd's 48 ms and the one-way delay are counted.
Verified by real-time replay (`v90_engine_replay --dial`, NOT `--fast`: the
bound is wall-clock): at 500 the replay reproduces the live Sd at anchor+501
ms; at 1000 Sd goes out 1 ms after the parse, anchor+914.  So the margin is
~90 ms on n=1, and whether the peer then answers Jd with S still needs a
live call.
