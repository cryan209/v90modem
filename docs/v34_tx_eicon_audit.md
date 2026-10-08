# Native V.34 transmit defects found during Eicon investigation

Reviewed and corrected 8 October 2026 against ITU-T V.34 (02/1998).
These are demonstrated native-transmitter defects. They are not yet an
explanation of every V.90 failure: its B1/data entry is a separate path.

## Nonlinear encoding was in the feedback loop

V.34 9.6.2, Figure 7 and Table 11 define a linear precoder prediction
from the previous unprojected x(n), followed by the specified rounding and
quantization. Clause 9.7 applies the nonlinear projection to the resulting
transmitted x(n). The receiver selects it through MP/MPh.

`v34_get_mapping_frame()` instead applied `v34_non_linear_encoder()` to
p(n) whenever nonlinear encoding was selected and `nl_x_warp` was false.
The native `data_baud_init()` always set that flag false. Thus the emitted
signal was not projected, and the precoder's p(n), c(n), y(n), and trellis
history could differ from the normative encoder. With zero coefficients,
the feedback remained zero but the required output projection was still
absent. With nonzero coefficients, both errors were possible.

The fix removes the feedback operation entirely, enables the existing
clause 9.7 output projection at the native handover, and normalizes its
projected power as required by the note in 10.1.3. The V.90 handover already
enabled that output projection. Removing the incorrect feedback operation
also subsumes the previous V.90 normalization-probe workaround: the raw
mapper now always produces normative unprojected x(n).

Running the new seam test against the original `v34tx.c` fails at the first
clean-state nonlinear B1 symbol: `(1.45868,-7.29341)` instead of
`(1.54782,-7.73908)` at 21600 bit/s, GPA, expanded shaping, with a nonzero
precoder. This failure is distinct from the tiny calibration discrepancy
recorded in `eicon_v90_upstream_power.md`.

## B1 did not clear the precoder tap delay line

Clause 10.1.3.1 explicitly resets the precoding filter tap delay line before
B1. Native `data_baud_init()` reset p and c but retained the x history used
by `precoder_tx_filter()`. After a previous data transmission, retrain,
renegotiation or half-duplex primary handover, the next B1 could therefore
start with old x(n-p), despite the reset of the other encoder states.

The fix clears the entire x history before B1. Injecting stale history with
nonlinear encoding disabled demonstrates the independent failure on the
old source at frame 0/symbol 1: `(-2.03603,-2.69707)` instead of
`(-2.32689,-2.44776)`. V.90's separate `v34_seed_tx_data()` entry already
clears this history.

## Checks

`v34_phase4_16pt_test` includes the production transmitter so the tests call
the actual static native `data_baud_init()` and `get_data_baud()`, rather
than merely seeding the offline mapper. It checks both caller/GPC and
answerer/GPA, 12000/21600/31200 bit/s, nonlinear encoding off/on, and
clean/stale precoder history: 24 cases. The nonlinear output is evaluated
from equations 9-33 through 9-35 against a raw linear mapper reference.
The V.90 seam is also compared as a reset/order control, with its scrambler
explicitly matched to the native role before calibration.

Both demonstrated failures pass after the correction. All 420
`v34_data_test` cases, the full PCM loopback suite, and `make v34-duplex-test`
pass. Additional 3200-baud/21600-bit/s duplex runs pass both G.711 laws
with zero payload bit errors in either direction. The private header
changed only in documentation; SpanDSP and modem
dependents were rebuilt, including the previously problematic split RX files.

The earlier `make test` run stopped at the independent K56flex A-law 32k
noise case, so a full-suite pass is not claimed. No bearer, gain, DSP timing,
scrambler polynomial or pulse-shaping constant was changed.

## Hardware follow-up

A forced native V.34 call used the rebuilt modem against Eicon endpoint
7910; evidence is `artifacts/eicon-native-v34-tx-fix-20261008-u1/`.
It sends INFO1c at +7490 ms and remains waiting for INFO1a until the call
ends. No B1/data handover, menu, or checked echo occurs. This hardware
trial never exercises either correction and therefore cannot establish
that the data corruption has improved. No gateway or peer configuration
was changed.

## Independent actual-transmit audit and remaining hardware failure

The independent Python encoder `tools/v34_encoder_oracle.py` implements
clauses 9.1, 9.3–9.6 and B1 clause 10.1.3.1 without importing production
shell, constellation or convolutional tables. Against actual call dumps:

* `eicon-encoder-truth-20261008/01-v90-ulaw`: 12000 bit/s,
  coefficients (28,-2), (-19,7), (19,-12); all 68816 raw Q9.7 symbols match.
* `eicon-encoder-truth31200u-20261008/01-v90-ulaw`: 31200 bit/s,
  coefficients (58,-2), (-41,3), (40,-7); all 51856 raw symbols match.

Both calls pass one exact 201-byte echo, fail the next, and receive real
1200 Hz Tone B from the Eicon. Their outgoing bit dumps contain respectively
273 and 225 complete CRC-valid HDLC frames, with no bad complete frames or
aborts. The oracle includes B1 and the 139 idle mapping frames preceding the
V.42 capture; this alignment was verified against the complete symbol stream.
It does not verify nonlinear projection, pulse shaping or the card's decisions.
These checks narrow the investigation; they do not establish interoperability.

In the earlier direct fixed-playout capture, upstream decoded samples match
continuously from local TX 18.000 to 36.3975 seconds (147180 samples; 46
changes between the two zero codewords). Downstream is byte-exact from local
RX 20.000 to 38.080 seconds. Each direction uses a separately verified sample
offset; the tap origins are independent. Comparison JSON files are saved in
`artifacts/eicon-direct-fixed-audio-20261008-r3/`. Later upstream mismatches
require separate alignment analysis before attributing a bearer fault.

## Simultaneous card capture: an altered-audio burst precedes CRC errors

`artifacts/eicon-card-encoder-20261008/` captures one additional PCMU V.90
call with Eicon Audio1 and eye tracing. It passes one 201-byte echo, fails
its second, and the card eventually initiates retraining. The independent
encoder matches all 58640 raw symbols at 31200 bit/s, coefficients (24,0),
(-15,9), (18,-11), with 131 pre-capture idle mapping frames. All 402 complete
outgoing HDLC frames have valid FCS; none are aborted.

At the card input, the initially constant card-minus-local-TX offset is 899
samples. Decoded upstream audio diverges at local TX 18.936625 seconds,
returns at 19.020000 seconds, and matches thereafter through 30 seconds.
The two nonzero mismatch runs cover 666 samples (one intervening sample
happens to match). The first two card CRC indications occur immediately
following the Audio1 blocks containing that burst, at trace times
16:0356:846 and 16:0356:909. This association uses record order/sample count,
not an assumed common tap time origin. Later constant-offset changes of
80 samples occur around card 30.55 and 31.02 seconds, with further CRC errors.

Local RTP TX has no sequence or timestamp gaps; maximum send interval is
24.154 ms. Remote RTCP reports seven transmitted packets lost, with loss
periods 60–80 ms. Those aggregate statistics cannot locate the loss, but
are consistent with the first altered-audio burst. This call therefore
provides a concrete bearer corruption mechanism, even though the previous
fixed-playout capture still needs separate investigation. No persistent
card, gateway or server configuration was changed. Card trace was stopped
normally with q after call completion.

## Fixed-playout CRC origin resolved

Record-order alignment of the preserved fixed-playout capture locates its
first bad CRC immediately after card sample 36.6925 seconds, corresponding
to local TX 36.399625 at the established offset. The first altered waveform
sample is local TX 36.397500: only 17 samples before the preceding Audio1
record ends. Thus the previously verified exact interval ends just before
the first CRC error, rather than demonstrating a CRC failure during intact
delivery. Subsequent audio has altered samples and temporarily different
alignment (2423 rather than 2343 samples), then returns to the original
offset. This removes that capture as evidence for encoder failure over a
clean bearer. It does not certify our transmitter or identify the network
component responsible.

## Native Eicon Phase 4: J(16) was always classified as J(4)

The LAN native PCMA capture has repeated exact receive words 0x89B0:
V.34 10.1.3.3/Table 18's 16-point request. The old detector computes d16=0,
d4=1, then applies `(d16 + 1) >= d4` to force the selection to four-point.
Because these patterns differ by exactly one bit, the heuristic prevents
selection of 16-point even on a perfect word. Eight sustained exact windows
were logged as confirmed J(4), so our Phase 4 TRN/MP/E used a constellation
the Eicon had not requested. Independent CRC-valid four-point MP decoding
at the card therefore did not demonstrate agreement with its requested mode.

The nearest-pattern classifier now preserves J(16); acquisition still requires
the existing sustained confidence checks. Recorded-schedule replay changes
all four attempts to confirmed 16-point. A regression checks the production
classifier's exact Table 18/19 words and the resulting MP symbol count.

## Selecting 16-point exposed missing Phase 4 power normalization

The J-only live correction still fails. Its actual TX tap's Phase 4 RMS is
approximately 10064 PCM units rather than the four-point signal's ~3182:
10 dB higher. The raw 16-point table has average energy 10 times the
four-point table's. Clause 10.1.3 requires these signals to use the selected
power level; the previous duplex Phase 4 TRN/MP/E generators did not
compensate that table energy. The half-duplex control-channel data generator
already has the same 1/sqrt(10) compensation independently.

A common duplex Phase 4 point helper now normalizes 16-point TRN, MP and E.
Four-point signals and the underlying spec coordinate tables are unchanged.
A regression grades mean symbol energy as well as constellation selection.
With both corrections, the next Eicon native PCMA call completes training,
LAPM and payload echo: MP settles a2c=21600, c2a=31200. Full call grading
and the PCMU confirmation follow below. The ten-row duplex matrix passes
with zero payload errors in both directions after each correction.

## Hardware confirmation after both Phase 4 fixes

`artifacts/eicon-lan-jpower-20261008/` (PCMA) and
`artifacts/eicon-lan-jpower-ulaw-20261008/` (PCMU) each pass all ten exact
201-byte echoes. Both negotiate a2c=21600 and c2a=31200 bit/s, establish
V.42 LAPM/V.42bis, and finish with Eicon DSP aborted/CRC 0/0. Neither logs
a retrain. Both directions report zero RTP loss. These runs exercise the
corrected native nonlinear/B1 handover as well as the J and power fixes.

After the final code changes, the Phase 4 production regression, the
10-row V.34 duplex matrix, and the full PCM loopback suite pass. Python
diagnostic tools compile and git diff --check passes. The earlier full
make test failure in the independent K56flex A-law noise case remains;
no full make test pass is claimed.
