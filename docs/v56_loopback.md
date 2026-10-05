# V.56 loopback and impaired-line measurements

`v56_loopback_test` runs two independent V.34 modems through an offline,
sample-preserving synthetic line. Both directions transmit a repeating
511-bit sequence and are graded separately after 64 consecutive bits match
a phase of the sequence. Once acquired, the checker never re-locks: bit
slips, noise errors, and a receiver that diverges remain visible.

The default is **one million checked bits in each direction**, with a
240-second simulated deadline. A missing connection cannot pass with a
zero-error, zero-bit result. The first 64 matching bits and earlier search
bits are excluded from the graded count.

## Running it

```sh
make v56-test                       # fast, strict regression
./v56_loopback_test                 # million bits, 2400 baud / 9600 bit/s
./v56_loopback_test --law alaw
./v56_loopback_test --baud 3200 --rate 21600 --snr-db 30 --seed 7 --json
./v56_loopback_test --loss-db 3 --delay 80 --echo-db 20 --bits 16000
make v56-sweep                      # long diagnostic sweep; keeps failed rows
```

`--delay` is one-way delay in **8 kHz samples**, so 80 means 10 ms one way
and 20 ms round trip. Every input sample produces exactly one output sample;
delay lines are initially silent and no samples are inserted or discarded.

The transmit level is -12 dBm0. `--loss-db` attenuates the remote signal;
`--snr-db` sets Gaussian noise relative to the nominal attenuated receive
level (`noise dBm0 = -12 - loss - SNR`). It is not a measurement of the
instantaneous waveform, and G.711 quantization remains an additional
impairment. Noise is continuous through silence and startup. Each direction
has a distinct deterministic noise seed. No noise is added by default.

The synthetic near-end echo is the receiver's own transmitted waveform,
delayed by `--echo-delay` (default 2136 samples) through the three taps
0.80, -0.45, 0.25, normalized for the specified return loss. The existing
`v34_line_ec` runs when echo is enabled; `--no-echo-cancel` provides its
control arm. These taps are harness parameters, not an ITU local-loop model.
Clipping is counted before G.711 encoding in each direction.

The simulated analog waveform path is remote delay/AD-EDD filtering/attenuation plus local
echo plus noise, then a single G.711 encode/decode. It deliberately damages
an **offline analog test channel**. It adds no processing to live SIP/RTP
and must not be used as a digital V.90/V.91 PCM channel.

Exit status is 0 only when both directions complete with zero bit errors;
1 means measured errors, carrier loss, simulated timeout, or a settled-rate mismatch; 2 means invalid
configuration. `--json` emits one result to stdout; DSP diagnostics go to
stderr. Fields `ab_*` mean caller A -> answerer B; `ba_*` mean B -> A.
`*_synced=false` makes a zero synchronization timestamp explicitly invalid.
The `a_mp_*_bps` and `b_mp_*_bps` fields record both peers' views of the
settled directional MP rates. Both must agree with `requested_bps` to pass;
`rates_available=false` makes zero rate fields invalid.
Synchronization times have the receiver callback's 20 ms block resolution.

## Calibrated AD/EDD filters

```sh
./v56_loopback_test --ad 5 --edd 3 --bits 16000 --json
make v56bis-filter-test             # verify all 18 committed FIRs
make v56bis-sweep                   # 36 rows, both laws, 30 s deadline
python3 tools/v56_sweep.py --channels off,1:1,5:2,9:3 \
  --cases 2400:9600 --snr off --delays 0 --bits 16000 --seconds 30
```

`--ad` selects 1, 5, 6, 7, 8 or 9 from V.56bis Table A.10; `--edd` selects
1, 2 or 3 from Table A.11. Both arguments are required. The selected filter
runs identically in both directions, before noise and G.711 quantization.
These are AD/EDD combinations, not complete named network-model rows.

The 513-tap FIRs have a nominal reference delay of 256 samples (32 ms) per
direction, in addition to `--delay`. The measured reference delay is included
in the calibration report. Bulk delay is limited to 7679 samples when filtering
is enabled, so the whole convolution fits in the history ring. JSON records
`ad`, `edd` and `filter_nominal_delay_samples`; zero AD/EDD means bypass.

[reference.json](../tools/v56bis/reference.json) preserves the published cells,
including nulls for unspecified values. AD is interpolated in dB and referenced
to 1000 Hz; envelope delay is interpolated in milliseconds and referenced to
1800 Hz (V.56bis 3.2, 3.4). The generator integrates delay to phase before
inverse transformation: multiplying frequency by the varying delay would
produce the wrong group delay. Endpoint holds and tapers toward DC/Nyquist
are engineering extensions; unspecified table regions have no numeric
calibration claim.

[The generator](../tools/v56bis_filters.py) verifies the rounded C coefficients
using direct frequency response and its analytic derivative, independently of
the synthesis FFT. Every specified knot and a 25 Hz interior grid must meet the
Annex A tolerances. Regenerate deliberately with
`python3 tools/v56bis_filters.py`; `--check` fails for stale coefficients.
`--report <path>` saves every profile's maximum error and failure list.
The C self-test also checks an impulse through the actual streaming FIR,
bulk delay and G.711 path, catching reversed taps or sample indexing errors.

All 18 profiles meet the table tolerances: the largest measured AD deviation
is 0.665 dB (in the wider-tolerance band), and the largest EDD deviation is
0.075 ms. This is numerical validation of the FIR response; modem acquisition
and payload recovery are separate measurements. SNR still refers to the
nominal received level, rather than a new measurement after frequency shaping.

## Sweeps and evidence

```sh
python3 tools/v56_sweep.py --cases 2400:9600 --laws ulaw,alaw \
  --snr off,40,30,24 --delays 0,80 --seeds 1,2 --bits 16000
python3 tools/v56_sweep.py --cases 3200:21600 --laws ulaw \
  --snr 40,34,30,26,22 --delays 0 --seeds 1,7 --require-clean
```

Each sweep creates a new `artifacts/v56/<UTC timestamp>/` directory with
`results.jsonl`, individual stdout files, and DSP logs. `--output <new-dir>`
chooses a destination; an existing directory is not overwritten. Every row
records the command, line configuration, seed, settled MP rates, elapsed
simulated time, checked bits/errors, BER, and outcome. A simulated timeout
with zero errors is retained as a failure, not included as a clean connection.

The default sweep measures two rate profiles, both laws, four noise settings,
and two delays. It returns 0 when the measurements ran successfully, even
when some lines failed: error/timeout/carrier-loss/rate-mismatch rows are expected in a
performance sweep. `--require-clean` makes any such row an exit-1 regression.
A crashed child, malformed result, or wall-clock timeout always exits 2.
`--timeout` bounds each child's wall-clock runtime separately from the
simulated `--seconds` deadline.

For reproducibility the runner removes inherited `ME_` and `V34_` diagnostic
environment switches. Direct binary runs still honour the DSP's environment
switches. Compare identical seeds and profiles when testing a DSP change;
use multiple seeds and sample delays to assess acquisition sensitivity.

## Relationship to the Recommendations

Sources: V.56bis (08/1995), especially clause 1, Tables 1-6 and Annex A;
V.56ter (08/1996), particularly 6.3.1, 6.3.1.5, and Annex A. The latter PDF
and `Software.zip` are inside
`ITU Docs/T-REC-V.56ter-199608-I!!ZPF-E.zip`.

This first harness follows the synchronous BER test's use of the 511 pattern
and grading only after synchronization. SpanDSP calls the corresponding
nine-stage generator `BERT_PATTERN_ITU_O153_9` (x^9 + x^5 + 1). It supplies
independent phases in the two directions. `--self-test` checks the exact
511-bit period, 256 ones per period, distinct acquisition signatures,
injected-error counting, transparent codec baseline, delay, attenuation, echo, and noise-seed
repeatability. The make target also exercises actual modem startup and data.

It is **not full V.56ter conformance or V.56bis network-model coverage**:

- The calibrated AD/EDD filters cover Tables A.10/A.11, but there is no
  complete EO-EO/local-loop matrix, frequency offset, phase jitter,
  impulse noise, or clock drift yet.
- Line profiles are custom synthetic impairments, not named Table 1 rows;
  no likelihood-of-occurrence weighting or NMC percentage is reported.
- V.56ter 6.3.1.5 continues to a statistical test limit and records residual
  BER. Our default strict finite-bit regression is not that statistical
  stopping procedure; it can measure BER with the sweep runner.
- This is a synchronous datapump test without V.8, PTY/V.14, V.42 or
  compression. Async character/message error and end-to-end file throughput
  require the corresponding framing and protocol layers.
- The supplied `.TST` files are file-transfer/compression payloads (Annex A),
  not audio, line impulse responses, or substitutes for the 511 sequence.
- The older V.56 comparative test procedure is not implemented in full.

Do not reuse SpanDSP's generated AD/EDD labels as evidence of full channel
coverage. In `spandsp-master/spandsp-sim/make_line_models.c`,
`generate_ad_edd()` has the EDD/phase calculation commented out and sets
phase to zero. Its three EDD variants therefore do not implement three
different delay-distortion curves. This harness uses its own phase-integrated
filters and validates the committed coefficients against Annex A independently.

## Initial verification (2026-10-05)

The self-test and short regressions passed in both laws at 2400/9600 and
3200/21600, including a 40 dB SNR / 3 dB loss / 80-sample delay run and a
20 dB echo-return-loss run. A no-impairment baseline carried one million
checked bits in each direction with zero errors.

The full 32-case sweep (seed 1; one million bits per direction) produced:

| Requested profile | Noise SNR | Laws / one-way delays | Outcome |
|---|---|---|---|
| 2400 baud / 9600 bit/s | off, 40, 30, 24 dB | both laws; 0 and 80 samples | 16/16 exact |
| 3200 baud / 21600 bit/s | off, 40, 30 dB | both laws; 0 and 80 samples | 12/12 exact |
| 3200 baud / 21600 bit/s | 24 dB | both laws; 0 and 80 samples | 4/4 completed with errors |

Both peers' MP rate views were checked against the requested rate in every
passing row. The retained report is
`artifacts/v56/20261005T091944648376Z/results.jsonl`.
Additional negative checks verified simulated no-acquisition timeout,
strict-sweep exit 1, wall-clock-timeout exit 2, and invalid-option rejection.
These are offline regression results; they establish neither analog
hardware interoperability nor population coverage.

The normal build and all targeted checks completed. An optional macOS
AddressSanitizer/UBSan binary stalled even on `--help` in this environment;
no sanitizer validation is claimed. The full existing modem suite was not
rerun for this standalone harness addition.

## AD/EDD verification (2026-10-05)

All numeric reference cells were compared with text extracted from the primary
PDF. All 18 generated filters passed calibration, and the streaming impulse
self-test and complete `make v56-test` regression passed. AD-5/EDD-3 in both
laws is included in that strict regression.

The first AD/EDD matrix used 2400 baud / 9600 bit/s, both laws, all three
EDD curves, no noise or bulk delay, 16,000 checked bits per direction and a
30-second simulated deadline:

| AD curves | Rows | Result |
|---|---:|---|
| 5, 9 | 12 | zero errors, 16,000 bits each way |
| 1, 6, 7, 8 with EDD-1 or EDD-3 | 16 | no synchronization before deadline |
| 1, 6, 7, 8 with EDD-2 | 8 | carrier loss before synchronization |

Report and calibration evidence:
`artifacts/v56/20261005T093458560193Z/results.jsonl` and
`filter-calibration.json` in the same directory. These are startup outcomes;
zero checked bits do not establish a BER. A filter-free 256-sample one-way
delay control passed with zero errors. The failing AD/EDD rows are retained
for receiver/startup investigation; their cause has not been isolated.
