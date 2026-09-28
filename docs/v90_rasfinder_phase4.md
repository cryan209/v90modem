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
