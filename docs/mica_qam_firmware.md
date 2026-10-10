# MICA resident QAM engine (and overlay 8E, V.32/V.32bis)

`mica_qam.c` / `mica_qam.h` / `mica_qam_tables.h`, tested by `mica_qam_test`
and checked instruction-for-instruction by `tools/mica_qam_*_oracle.py`
against MicaEmu's isolated C53 core (`--mica ../MicaEmu`). Test-only; not
linked into the modem.

These routines were first lifted as K56flex (in `k56flex.c`, documented in
`k56flex_implementation.md`) because MicaEmu's reverse-engineered K56flex
spec, "Draft 0.23", recorded them; the clause numbers in comments are that
draft's. Most of them are MICA's shared resident QAM receive and transmit
engine, and the overlay that drives the verified paths, 8E, is the V.32/V.32bis
datapump. The arithmetic below is verified; the K56flex attributions in the
older section text are superseded by the next section. Moved and renamed on
2026-10-10: `k56flex_feedback_*` -> `mica_qam_*`, `k56flex_response_rx_*` ->
`mica_qam_word_rx_*`, `k56flex_startup_tx_*` -> `mica_v32bis_tx_*`, and
`tools/k56flex_*_oracle.py` -> `tools/mica_qam_*_oracle.py`. Artifact paths
(`artifacts/k56flex-response-20261009/`) are unchanged.

## Overlay 8E is MICA's V.32/V.32bis datapump, not K56flex (2026-10-10)

**Correction to the section below and to earlier sections that cite "bank
8E" as K56flex.** Overlay 8E is not loaded by the K56flex branch. 0C7F's
loader 2BF6 loads module 8C or 8D (by processor role), and that is the
K56flex PCM module. Overlay 8E (with 8F, or 80/81 when DM EEAC bit 10 is
set) is loaded by 2C05 from the general answer path at 98F5/1FB6/9DEB. The
two share the D600 window, so 8E code cannot run while 8C is resident. A
status bit 8E sets in DM 8F31 is therefore not a K56flex gate source.

What 8E contains is V.32bis (Recommendation V.32bis, 1991):

- its transmitter is fixed at 2400 symbols/s on an 1800 Hz carrier (§2.1);
- 2 bits a symbol, differential, 4800 bit/s, are the rate-signal
  format of §5.3 (Table 2 coding);
- the "training word" 8990 packs Table 5's rate signal R LSB-first (B4, B7,
  B8, B11, B15 set). 89B0 adds B5, "4800 bit/s enabled";
- the "response collector" headers 8880 and 888F are the synchronising
  bits B0-B3, B7, B11, B15 of R (B0-B3 = 0000) and of E (Table 6, B0-B3 =
  1111);
- the collector's 5/23 and 18/23 descramblers are V.32's GPA and GPC (§4).

So the 2400-baud startup transmitter below, the E259 gate trace and the
1D0E/E259 response collector are V.32bis code, verified against the
original instructions. The arithmetic stands; the K56flex attribution does
not. The resident 13BE/1402 path that expects an R-format word (1D05,
8990 with B5 variable) is reached only from 1242, the branch 0C7C takes
when DM 8FAA bit 0 is CLEAR, i.e. the side that does not load the PCM
module. It is probably the non-PCM/fallback path, but that is not traced.

The K56flex upstream DATA direction is the client's transmitter. The
Rockwell K56flex image reports client TX rates up to 33600 (see below), and
V.90's own upstream is V.34-derived, so V.34 is the likely candidate. Which
MICA receiver module 8C uses after DC3B's data activation (resident 6582
runs every cycle) has not been traced.

## Upstream response collector (2026-10-09)

`mica_qam_word_rx_dibit()` now implements resident 1D27's repeated 16-bit
response acquisition beside the existing extended 24-bit report collector.
It consumes already-sliced two-bit decisions, least significant bit first,
with an explicitly selected 5/23 or 18/23 descrambler. It tests the 888F/8880
header at each dibit, then compares two further repetitions eight dibits apart;
a mismatch resumes sliding search. The implementation preserves the firmware's
confirmation timing and accepted latch. Unsupported taps are rejected at init.
This is the receive boundary underlying Draft 0.23 clause 11, not a recovered
waveform demodulator or an assignment of response words to Table 1a gates.

`tools/mica_qam_word_rx_oracle.py` compiles the production collector in a
temporary directory and compares it with original MICA 1D0E/1D27/1D9E
instructions after every four supplied bits. Its 1,536 cases cover both taps,
32 response words, eight dibit offsets, invalid headers and a changed repetition
followed by recovery. Both acceptance timing and accepted words must match.
The report records the firmware hash and isolated emulator build identity:

```sh
python3 tools/mica_qam_word_rx_oracle.py \
  --output artifacts/k56flex-response-20261009/oracle.json
make mica_qam_test && ./mica_qam_test
```

The normal C tests also cover valid and invalid responses with both taps and
all eight dibit offsets. All five K56flex test binaries pass, including the
existing noisy A-law client regression. No live gate is enabled by this change:
receiver mode routing, waveform acquisition, odd bit phase, response-to-status
mapping and a real client call remain necessary before claiming CONNECT.

## Firmware gate trace (2026-10-09)

The remaining receive path can be investigated directly from the MICA firmware.
`tools/mica_qam_gate_oracle.py` now executes bank-8E's E259 collector loop and
its DEB3..DEBF caller branch, with the original resident 1D0E/1D27/1D9E
instructions. Repeated scrambled 8990 reaches DEBF, which sets **8F31 bit 11**,
after 48 supplied bits; repeated 8991 times out at E07B after 19204 supplied
bits without setting the gate. No accepted latch or status bit is injected.
The fixture stubs the sample pipeline, detector result, setup and scheduler
yield, so it establishes the bit-to-first-gate control flow, not PCM reception.
This narrows Draft 0.23 Table 1a / clause 11's previously open receive boundary.

The original B31D also returns **8990 or 89B0**, never FFFF. Its inputs are
DM 8FA8 (index into six words at PM B317), 8FA9 (comparison value) and EEAC
bit 12. The table is `[5, 5, 5, 6, 6, 7]`. With bit 12 clear, comparison
value >= the indexed threshold selects 89B0 and stores it at 8FB8. Otherwise
it returns 8990 and clears 8FB8. Thirty-six original-instruction trials cover
all indices, values immediately below/at/above each threshold and both flag
values. The physical meanings and initialization of these inputs still need
tracing; the probe's existing FFFF fixture has not been replaced by a guessed
selection.

Static controller evidence also distinguishes the later gates:

| Status write | Firmware path | Observed prerequisite |
|---|---|---|
| bit 11 | DEBF | E259 repeated 16-bit response success |
| bits 12+13 | DF47 | alternate DF45 entry; caller still needs tracing |
| bit 13 | DF30 | 1CD7 collector succeeds and bank-88 45B7 returns nonzero |
| bit 14 | DF80 | 23BF detector loop returns nonzero after intervening 1DBB monitoring |
| bit 10 | E17E / E1B9 | detector/energy loop; not a report-word bit |

The table is a static address map except for the independently executed first
row. There are additional writes at E00C/E044/E09D/E1A5/E1E3; their entry
routing must be established before assigning a single meaning to a status bit.
In particular, simply connecting every accepted response to the next gate would
skip firmware detector and timing requirements.

Reproduce with the retained isolated MicaEmu core:

```sh
python3 tools/mica_qam_gate_oracle.py \
  --output artifacts/k56flex-response-20261009/gates.json
```

Next firmware work: trace callers of DF45 and E169/E1AB, identify the B31D
input producers, then run raw received PCM through the selected 39E0/6910/
5A54/58F2 pipeline with its actual initialization and clock loops. The latter
is the major missing implementation, rather than the repeated-word format.

## Feedback coordinate decoder lift (2026-10-09)

`mica_qam_slice()` and `mica_qam_dibit()` lift the bank-8E
E4A9 feedback receiver's nearest-point and differential decisions (resident
5903/5961, BC84/BC88/BC90 tables). The two first-quadrant points are (12953,0)
and (0,12953); first-point ties and signed quadrant labels follow the shipped
tables. These are explicit equalized-coordinate interfaces, not a PCM input
or a new assumption that this mode runs throughout the whole startup.
The original DSP distance arithmetic is only verified in the bounded domain
of the coordinate fixture; C uses wider intermediates to avoid overflow.

`tools/mica_qam_coordinate_oracle.py` compares the C decisions against the
retained original-instruction coordinate fixture. It reads sibling fixtures
without modifying them and writes evidence under this repository. Checks
cover axis/sign/tie cases and the bank-8E decisions feeding repeated 24-bit
reports. The resident diagonal constellation is deliberately a separate
firmware fixture, not substituted by the new bank-8E slicer. Ordinary C tests
exercise all four points, all sixteen differential transitions and tie rules.

```sh
python3 tools/mica_qam_coordinate_oracle.py \
  --output artifacts/k56flex-response-20261009/coordinates.json
make mica_qam_test && ./mica_qam_test
```

These functions move the implemented receive boundary from supplied dibits to
supplied equalized coordinates. Original 5A54's complex rotation is the next
bounded stage upstream; actual coefficient generation, 6910 filtering, AGC,
carrier/timing state and correct call-stage selection remain required for
received G.711 octets to reach these coordinates.

The production comparison passed 450 nearest-point coordinate decisions and
1,536 bank-8E report acquisitions; all intervening differential decisions
matched the original 5961 output. The separate resident fixture passed too
(3,072 total original report acquisitions). This remains coordinate-level
evidence and does not establish raw PCM acquisition or hardware CONNECT.

## Rotor and forward FIR lift (2026-10-09)

`mica_qam_rotate()` now lifts original 5A54's complex multiply,
including ZALR's half-LSB, signed bias and modulo-32-bit accumulation/high-word
stores (Draft 0.23 clause 7.15). It accepts supplied signed phasor coefficients;
it does not generate them or assert carrier lock. The original-instruction
comparison passes 1,024 cases / 2,048 complex pairs across signed extrema,
random inputs, biases and ring phases. The original fixture additionally
checks 404 joined rotor/nearest-point cases with the resident constellation.

`mica_qam_fir()` lifts 6910's forward stage for the recovered
48-tap/two-row descriptor (clause 7.16), taking a logical 256-word complex
input ring and supplied 192 coefficient words. Each row reads the same past
48 complex samples and has its own complex coefficients. Signed rounding,
row selection and modulo-16-bit stores match the firmware in the tested domain
without accumulator overflow. The function consumes no input samples and
performs no coefficient adaptation; those responsibilities belong to the
surrounding receiver state machine. Wide C intermediates avoid undefined
signed overflow outside the fixture's verified domain, without claiming
firmware equivalence there.

`tools/mica_qam_rotation_oracle.py` and `tools/mica_qam_fir_oracle.py` compile
these production routines and compare them with original instructions through
the retained MicaEmu fixtures. The forward FIR comparison passes 2,048 cases /
4,096 complex outputs, with every input-ring phase, impulse ages/signs,
constant/alternating signals and random inputs. The original fixture also
passes 2,048 joined FIR/rotor/resident-slicer cases. That joined fixture is an
original-firmware composition, not an end-to-end C receiver or live waveform.
Evidence records the production source and firmware hashes and emulator build.

```sh
python3 tools/mica_qam_rotation_oracle.py \
  --output artifacts/k56flex-response-20261009/rotation.json
python3 tools/mica_qam_fir_oracle.py \
  --output artifacts/k56flex-response-20261009/fir.json
make mica_qam_test && ./mica_qam_test
```

The ordinary C tests pass too and exercise all 256 ring phases, different
complex rows and negative/positive half-LSB rounding. Remaining upstream work:
6910's adaptive coefficients, 4580's serial resampler/gain path, actual phasor
and clock-loop initialization and updates, then joined startup-stage receive
and gate tests against raw G.711. Neither seed coefficients nor isolated
forward arithmetic establishes receiver convergence or a real K56flex call.

## Sequential FIR adaptation lift (2026-10-09)

`mica_qam_adapt()` now lifts original 6910's sequential coefficient
sweep and 6B54's history refresh (Draft 0.23 clauses 7.16 and 7.22). It updates
one tap in both complex rows, advances the history index/count and refreshes
three regressor groups on reaching tap 48. The supported descriptor has m70=2,
m73=m74=0, m77=0..4 and m78=0..3, matching the retained firmware fixture.
History, input and error rings are explicit caller-owned logical arrays.

`tools/mica_qam_adapt_oracle.py` compares production coefficient values, sweep
state and refreshed history against original instructions in 3,000 cases,
covering all 48 taps and 800 history refreshes. For the first 1,500 cases it
also feeds the newly updated C coefficients into the C forward FIR and compares
all four outputs against the original same-call forward stage. All checks and
the ordinary K56flex core tests pass. This establishes update-before-filter
ordering as well as isolated adaptation arithmetic.

```sh
python3 tools/mica_qam_adapt_oracle.py \
  --output artifacts/k56flex-response-20261009/adapt.json
make mica_qam_test && ./mica_qam_test
```

One boundary remains explicit: when tap 48 meets m77=0, the DSP's history-copy
repeat count is FFFF (65,536 iterations). A bounded 256-word history model
cannot represent that path faithfully; C returns -1 before changing anything.
It also rejects unsupported descriptors and out-of-workspace history reads.
A negative relative history index becomes out of workspace on the next call;
it is not silently wrapped into the local array. The original absolute PM
pointer arithmetic is retained modulo 16 bits by the oracle's comparison.

The forced firmware replay previously kept its adaptation sweep disabled;
these tests supply nonzero sweep state and therefore do not show when a real
startup enables adaptation or that the receiver converges. Connecting the
actual error-ring producer (1C43), setup and resampler remains required.

## Six-tap resampler and gain lift (2026-10-09)

`mica_qam_resample()` lifts 51CE/4580/5219 with all 64 shipped
six-tap coefficient rows (Draft 0.23 clause 7.17). The extracted rows are in
`mica_qam_tables.h`, identified by their firmware SHA256. The API takes
an explicit 128-word lane ring, logical cursors, available count, table phase,
shift, gain, bias and slip sign. It writes the 256-word complex ring and updates
both cursors/counts exactly as the fixture does. A successful block clears the
slip. Invalid state, insufficient output space or unsupported filter overflow
returns -1 without partial writes. No buffering changes physical sample counts.
The input lane's relation to physical PCM still requires dispatcher tracing;
this interface does not declare the lane words to be G.711-decoded audio.

The production comparison passes all 1,536 ordinary cases / 13,824 output words
across every table, three slip signs, cursor wrap and two block sizes. A second
1,536-case comparison uses extreme signed gains and also passes. This found a
necessary instruction-order correction: SPM=2 scales the multiply result by 16
**modulo 32 bits**, then four APAC operations individually saturate the
accumulator. Clamping the combined 64*r*gain expression gives the wrong sign
when that scaled product wraps. The C implementation and independent extended
fixture now preserve the original order. The source/fixture comparison records
firmware, production source, table and emulator-build identities.

```sh
python3 tools/mica_qam_resampler_oracle.py \
  --output artifacts/k56flex-response-20261009/resampler.json
python3 tools/mica_qam_resampler_oracle.py --saturation \
  --output artifacts/k56flex-response-20261009/resampler-saturation.json
make mica_qam_test && ./mica_qam_test
```

Ordinary C tests cover bias, slipped output count, both cursor wraps,
transactional invalid-count rejection and scaled-product overflow before gain
saturation. The remaining integration boundary includes the serial-lane
producer/dispatcher, timing-error generation, table-phase/gain acquisition,
error-ring construction and joining these routines into a receive startup
that accepts a real client's waveform. No hardware CONNECT is established.

## Timing detector, correction and phase bookkeeping (2026-10-09)

`mica_qam_timing()` lifts original 45B8/460E/535B's three-tap timing
detector, smoothing, reference-phasor correlation, loop integrators, deadband,
fixed phase steps and periodic rate folding (Draft 0.23 clause 7.17). It takes
an explicit DP-11B register image and logical complex ring. The register-image
interface preserves firmware offsets while initialization and stage dispatch
are still being recovered. This is not yet an engine receiver API.

`tools/mica_qam_timing_oracle.py` compares all 128 register words against
original DSP instructions in 4,096 cases covering positive/negative/deadband,
freeze, early-return and rate-fold paths. The fixture excludes 9,334 trials
whose independent model leaves signed-32-bit bounds. Production currently
returns -1 without committing state on modeled accumulator overflow; saturation
and acquisition/convergence beyond the bounded fixture remain unverified.
The implemented loop constants retain the firmware's values.

`mica_qam_phase()` lifts original 542D's modulo-32-bit correction
subtraction, flag gating, pending coarse-boundary state and unconditional
correction/scratch clears. A separate original-instruction check passes 3,072
full-width cases. The timing oracle also runs the original 542D immediately
after original 45B8 and compares it with C phase bookkeeping fed by the
already-compared timing state: all 4,096 joined timing/phase cases pass.
Ordinary C tests exercise the frozen persistent-rate path and pending-slip
preservation; the K56flex core test passes.

```sh
python3 tools/mica_qam_timing_oracle.py \
  --output artifacts/k56flex-response-20261009/timing.json
python3 tools/mica_qam_phase_oracle.py \
  --output artifacts/k56flex-response-20261009/phase.json
make mica_qam_test && ./mica_qam_test
```

These checks establish how supplied timing state changes correction and phase,
not a locked clock against an actual client. Next integration work is original
5149/51B4/68D3 receiver setup and the 68E9 task/65A6/6C50/resampler cadence,
plus 1C43's decision-error producer and 56D2's carrier coefficient updates.
The digital serial ISR independently decodes each G.711 octet into its lane
ring, but that alone does not establish the scheduler, block cadence or full
raw-PCM-to-report path. Hardware CONNECT remains unverified.

## Timing initialization and block cadence (2026-10-09)

`mica_qam_timing_init()` lifts 533F's complete reset with either
shipped BD01 or BD05 table (Draft 0.23 clause 7.17). The original-instruction
comparison passes 128 randomized register images, checking all retained and
cleared words. Selection of BD05 here is a full reset with that table; the
actual DC5D transition uses 52EE's constants-only load and must not invoke this
reset as its replacement.

`mica_qam_block_count()` lifts 6C50's four SUBC steps, remainder,
8CAB available-count generation and block counter in a DP119 image. All
4,096 firmware fixture cases match, including quotients above 15 where the
four-step arithmetic must not be replaced by unrestricted division. The
physical interpretation as quotient/remainder is restricted to the fixture's
nonnegative domain with quotient below 16. This function implements the front
of 6C50 only, not the subsequent block processing, pointer copy or FIR calls.
The C cadence test also checks sample conservation over repeated one-sample
steps and three-sample blocks; the K56flex core suite passes.

```sh
python3 tools/mica_qam_init_oracle.py \
  --output artifacts/k56flex-response-20261009/init.json
python3 tools/mica_qam_block_oracle.py \
  --output artifacts/k56flex-response-20261009/block.json
make mica_qam_test && ./mica_qam_test
```

The setup trace exposes an important remaining distinction. 6BF4 sets
m44=2/m45=3, copies 6B89 into DAA9, and 6C37..6C42 expands **DAB7** then
clears its coefficient workspace. This is the block/input FIR used before
51CE resampling. The report-symbol pipeline's **DAE7** descriptor is a
separate configuration; its verified 48-tap/two-row forward function is not
automatically the missing input FIR. Raw-lane integration must recover DAB7's
actual configuration, coefficient evolution and 4AF3 processing, rather than
feed serial samples through the DAE7 function by assumption. Full startup,
clock/carrier acquisition and hardware CONNECT remain unverified.

## DAB7 setup and input subtraction recovered (2026-10-09)

`tools/mica_qam_input_setup_oracle.py` now executes original 6BF4 and its
6A0B/6A15/6A98 setup calls with explicit relocated coefficient/workspace
pointers and profile-selector zero. The expanded DAB7 descriptor establishes:
source pointer cell **8CC8**, error pointer cell **8CCC**, output pointer cell
**8CC9**, 48 taps, three complex output rows and output shift nine. The earlier
static notes did not distinguish error and output pointer roles. All 288
coefficient words are zero after setup. The original pointers become A000
(source), D400 (error) and D928 (output); workspace values in the fixture remain
relocations, not an assertion about the physical input interface.

Following 6C50 through 4A8F shows a separate predictor-processing/subtraction
path before 51CE. The original 4AAA..4ABD block captures two raw lane words,
subtracts two predicted words, and writes the residual back to the lane ring;
DM 8F4F bit 4 bypasses subtraction. `mica_qam_residual()` lifts that
block (Draft 0.23 clause 7.18 receive boundary). It matches original instructions
on **1,032 cases**, both bypass settings, signed extrema and seeded random
inputs, including modulo-16-bit differences. The raw capture words are also
checked in the original fixture. The predictor/rotation preceding subtraction
and its adaptive enable state are not substituted by these isolated checks.

The correct fixture supplies raw input through AR4, raw capture storage through
AR2 and predicted values through AR7. Operand suffixes select the register for
the *following* access: treating AR2 as the predicted stream here would test
words just overwritten with raw input. This register ordering is now checked
by observed writes, not inferred from pointer names.

```sh
python3 tools/mica_qam_input_setup_oracle.py \
  --output artifacts/k56flex-response-20261009/input-setup.json
make mica_qam_test && ./mica_qam_test
```

This corrects the previous integration plan: DAB7 is not simply another report
equalizer that can be placed in series with DAE7. It belongs to a prediction
and residual path, with different source geometry and output scaling. Its
physical source producer, predictor mixing in 4A8F and 4AF3 adaptation still
need to be joined before enabling a full receiver. No claim of echo-canceller
convergence, clock/carrier acquisition or hardware CONNECT follows from this
setup and subtraction evidence.

## Joined predictor, residual and error rotation (2026-10-09)

`mica_qam_predictor()` now lifts original 4A91..4AF0 as a joined
sample operation (Draft 0.23 clause 7.18 input-processing boundary). It rotates
the supplied predictor output with a supplied phasor, subtracts that prediction
from the lane pair (or honors 8F4F bit 4's bypass), then rotates the residual
with the conjugate phasor into the error pair. Forward mixing uses the existing
5A54-equivalent rotor; inverse error products are evaluated directly so a
signed-minimum phasor component need not be negated in a 16-bit variable.

The input-setup oracle now executes **4A91 through 4AF0 unchanged**, with
explicit pointer setup, SPM=1, OVM clear and optional diagnostic word 8CD8 zero.
It compares predicted samples, residual lane words, original raw-capture writes
and conjugate error words against the joined C operation in **512 cases**
(256 seeded signal/phasor trials in each bypass arm). All cases pass, alongside
the earlier 1,032 isolated subtraction cases and the K56flex core suite.
This adds an actual instruction-sequence composition to the input-path checks;
it does not supply the source/phasor generator or assert adaptive convergence.

Reproduce using the existing input-setup command; its JSON now includes
`joined_predictor_subtraction_cases` with source pairs, phasors, predictions,
residuals and error words. The physical source ring producer, DAB7's distinct
forward geometry and 4AF3's adaptation control remain the next boundaries.
No live receive gates are enabled by this change and hardware CONNECT remains
unverified.

## DAB7 forward geometry verified and implemented (2026-10-09)

`mica_qam_predictor_fir()` now implements the distinct DAB7 forward
stage from original 6910 (Draft 0.23 clause 7.16 input-path descriptor). It
reads a logical **8,192-word source ring** using p-2-2t and p-1-2t for the two
components, with 48 taps and **three complex coefficient rows** (288 words).
Outputs use `(sum + 131072) >> 18` modulo 16 bits, rather than DAE7's >>13.
The source pointer is not advanced by this function; coefficient adaptation
and its producer are separate responsibilities. C uses wide accumulation,
while firmware equivalence is verified only without accumulator overflow.

`tools/mica_qam_predictor_fir_oracle.py` executes the original 6BF4 setup and
whole 6910 with adaptation zero. **96 impulse cases** independently check every
tap/component, all three rows, source-ring addressing and output scale. Another
**128 randomized full source rings** compare production C against original
instructions. All **1,344 output words** match, and the K56flex core suite
passes. The output scratch addresses are D928 plus reverse3(0..5); 8CC9 ends
at D928 plus reverse3(6). That matches the three successive complex pairs
consumed by the predictor-mixing stage's reverse-carry index four.

```sh
python3 tools/mica_qam_predictor_fir_oracle.py \
  --output artifacts/k56flex-response-20261009/predictor-fir.json
make mica_qam_test && ./mica_qam_test
```

The source ring and phasor producers, DAB7-specific adaptive state (different
from DAE7's two-row sweep) and 4AF3 control still require recovery. This is
verified forward arithmetic with supplied coefficients, not continuous
waveform acquisition or a demonstrated client connection.

## Predictor phase/phasor production (2026-10-09)

`mica_qam_predictor_increment()` now lifts the original 6D93..6DA8
loop filter in DP119. Words 33/34 supply the measured phase-error term,
36/37 the configured gains, and 38/39 retain the integrator. The gain
provenance is explicit: 6CC4 loads 37/36 from program-table entries and
9024/DBE8 copy profile words into 8CB6/8CB7. The lift preserves unsigned
low-word multiplication, SPM-scaled SPH, rounding before the arithmetic
three-bit shift, and modulo-32-bit accumulation. The returned increment
feeds the phase tail. These are recovered firmware operations, not a new
choice of DSP constants.

`tools/mica_qam_predictor_phase_oracle.py` now compares that original routine
with production C and composes its result with the original phase tail.
All 2,048 full-width randomized cases match the increment, integrator,
scratch word, phase and phasor. Evidence is in
`artifacts/k56flex-response-20261009/predictor-controller.json`.
The enclosing mode-15 4AF3 path is now recovered below. The alternate
error-acquisition mode is also recovered in the section below; this does not
establish carrier acquisition or CONNECT.

The mode-15 front end is now lifted as
`mica_qam_predictor_correlate()`: 4B59..4B86 accumulate the dot
product and signed cross product of three complex pairs from 8CCF and
D920 into the two 32-bit accumulators at D930. SPM=1 wrapping precedes
the arithmetic five-bit shift; retained accumulators also wrap.
`tools/mica_qam_predictor_correlation_oracle.py` checks all four accumulator
words against original instructions in 2,048 full-width randomized cases,
including overflow. All match; evidence is in
`artifacts/k56flex-response-20261009/predictor-correlation.json`.
The subsequent timer, 6D4E angle conversion and accumulator reset are not
included in this helper; the complete mode-15 helper below composes them.

`mica_qam_predictor_phase()` lifts 4BB1..4BCD's modulo-32-bit
phase advance and rounded 512-entry phasor lookup (Draft 0.23 clause 7.18).
The increment is the accumulator supplied by the preceding 4AF3 controller,
not an assumed physical frequency. Phase is DP119 words 3A/3B; the rounded
index is `(high_phase + 64) >> 7` modulo 512. Outputs 3C/3D use cosine indices
i and i+384, respectively. The table is supplied from the retained live data
dump; its initialization and controller convergence are not claimed here.

`tools/mica_qam_predictor_phase_oracle.py` executes the original phase tail and
compares C phase and phasor words in **2,048 full-width randomized cases**.
All pass, with firmware, data-dump, production-source and emulator-build hashes
retained. The input-setup oracle now also checks the 6BF4/6C9C controller reset:
phase/increment zero, phasor (7FFF,0), and enable word 8CD7=7FFF. The one-sample
setup's earlier 6BE2 clear is not the state left by the three-sample 6BF4 path.
Static enable/disable entries are 6CBC (7FFF) and 6CC0 (0); 4AF3 bypasses its
controller when this word is zero. The K56flex core suite passes.

```sh
python3 tools/mica_qam_predictor_phase_oracle.py \
  --output artifacts/k56flex-response-20261009/predictor-phase.json
python3 tools/mica_qam_input_setup_oracle.py \
  --output artifacts/k56flex-response-20261009/input-setup.json
make mica_qam_test && ./mica_qam_test
```

The ring-capture dispatch is now bounded statically: 1916/1941 copies 8CC8
into 8CED and installs 19FF at callback 8CE9. 19FF transitions that callback
to 1A22, which calls 1A8C/1A99 and then 1A55/1A62. These routines consume ring samples for capture rather than filling the ring;
the copy direction is now independently checked below. The
4AF3 increment estimator and physical source-ring generation still need
recovery before this becomes a continuous input receiver. Live CONNECT is
still unverified.


## Source writer versus capture consumer resolved (2026-10-09)

The earlier source-fill interpretation of 19FF/1A22 was wrong. Original 1A8C
copies from the source ring through 8CED into a destination named by a capture
pointer cell, advances the source read cursor and returns the destination end.
`tools/mica_qam_capture_copy_oracle.py` executes the original in 128 cases,
checking both four-word and 96-word copies, destination values, returned end,
source cursor and **unchanged source memory**. It does not run the full capture
callback scheduler.

The actual observed writer is original **4A6C**, called by 9331/9356/9503/9525.
It mixes a supplied complex pair (the 9331 caller uses 8C0F/8C10) with two
program coefficients selected by phase pointer 8C1B, writes two words through
**8C91**, and advances that cursor. It shares the A000..BFFF ring used by DAB7's
read cursor 8CC8. This places the recovered ring writer before the verified
predictor FIR; it does not establish which symbols a complete call supplies.

`mica_qam_source_pair()` lifts the mix and logical cursor update
(Draft 0.23 clause 7.18 boundary). `tools/mica_qam_source_pair_oracle.py`
compares C ring writes/cursor against original 4A6C in **1,024 cases**, using
supplied pairs, carrier coefficients and randomized ring positions. All pass;
source scaling is division by 8192 with positive half-LSB rounding. The fixture
also checks the original phase-pointer wrap for its explicit one-step profile;
the C pair operation leaves carrier-phase evolution to its caller.

```sh
python3 tools/mica_qam_capture_copy_oracle.py \
  --output artifacts/k56flex-response-20261009/capture-copy.json
python3 tools/mica_qam_source_pair_oracle.py \
  --output artifacts/k56flex-response-20261009/source-pair.json
make mica_qam_test && ./mica_qam_test
```

Remaining: symbol/carrier producers feeding 4A6C during actual startup, full
4AF3 increment estimation and DAB7 adaptation state, then a continuously
scheduled input path. The K56flex core suite passes, but neither these supplied
symbol cases nor ring-copy fixtures establishes a real modem connection.


## Complete predictor controller, firmware mode 15 (2026-10-09)

`mica_qam_predictor_angle()` lifts 6D4E and both of its callees,
6831 and 6890. The former produces a 512-unit full-quadrant angle, and
6890 supplies the finer small-error result when the normalized real component
exceeds four times the imaginary magnitude. PM0320..0326's seven coefficients
are retained exactly: -5984, 22404, -107, -1003, 10446, -1, 6493.
The code preserves the 15-step normalization limit, rotate-through-carry,
restoring division, polynomial product scaling, rounding, signed quadrant
folding and mode-dependent output scale. In particular, the all-zero input
has the firmware's actual result; it is not replaced with a generic atan2
special case. These operations recover the firmware implementation associated
with Draft 0.23 clause 7.18; they do not infer new protocol constants.

`tools/mica_qam_predictor_angle_oracle.py` executes original 6D4E with both
original polynomial routines and compares its returned accumulator with C.
All **4,217 cases** match: 121 boundary combinations, including zero and
INT32_MIN, plus 4,096 full-width random complex accumulators.

`mica_qam_predictor_control15()` now composes the entire original
4AF3 path with EE9B=15. It obeys enable word 57 and loop bypass word 2C,
accumulates three complex correlations, decrements the unsigned timer at
4E, and converts the accumulated correlation when the previous timer is zero.
That call resets the timer to 40 and clears the four correlation words;
subsequent updates therefore occur every 41 calls. It runs 6D93 and preserves
4BA9..4BB0's additional proportional contribution before updating phase and
phasor. During the intervening calls phase advances by the retained
integrator. Disabling the controller leaves phase and phasor untouched.

`tools/mica_qam_predictor_control_oracle.py` compares the original complete
routine with production C in **4,096 independent cases** and **32 sequences
of 128 successive calls**, checking the phase-error words, integrator,
phase, phasor, timer and retained correlation. All 8,192 calls match.
The supplied samples, reference pairs and gains are fixture inputs; this is
controller equivalence, not proof of carrier acquisition on a modem line.
The non-15 4B07..4B57 acquisition branch is now recovered below. DAB7
adaptation and integration into a continuously scheduled upstream receiver
remain open.

```sh
python3 tools/mica_qam_predictor_angle_oracle.py \
  --mica ../MicaEmu --output artifacts/k56flex-response-20261009/predictor-angle.json
python3 tools/mica_qam_predictor_control_oracle.py \
  --mica ../MicaEmu --output artifacts/k56flex-response-20261009/predictor-control15.json
make mica_qam_test && ./mica_qam_test
```

The reports retain firmware, production-source and emulator-build hashes.
The K56flex core suite passes after the lifts. Hardware CONNECT is still
unverified and the live path is not enabled by these helpers.


## Alternate predictor controller recovered (2026-10-09)

`mica_qam_predictor_control()` now lifts the full 4AF3 path for
EE9B values other than 15. 4B07..4B57 measures energy over the six 8CCF
samples, updates retained gain word 35 within the original bounds, calculates
the signed cross product with the six bit-reversed AR5 reference words,
filters the resulting error into words 30/31, and forms words 33/34 for
6D93. The existing loop filter and phase/phasor lift complete the call.
Enable and bypass words 57/2C retain their original behavior. Input pairs
and reference pairs are passed in logical order; the oracle supplies the
original AR5 storage through seven-bit-reversed offsets from its base.

This path depends on PMST.TRM=1, as used by the recovered firmware fixture:
SATL performs an arithmetic shift selected by explicit TREG1 writes from
words 2C and 32. Changing the multiply register does not change those shifts.
The lift preserves unsigned energy multiplication, signed cross products,
32-bit wrap before shifts, signed bounds, and the exact SACH/SACL scaling.
No generic AGC or carrier loop is substituted. The firmware operations
correspond to Draft 0.23 clause 7.18's predictor processing.

`tools/mica_qam_predictor_normal_oracle.py` compares original 4AF3 with C
in **4,096 independent cases**, varying non-15 mode values, enable/bypass,
all energy shift counts, full-width state and signed input/reference words.
It then checks **32 sequences of 128 successive calls**. All 8,192 calls
match the error/filter/gain, integrator, phase/phasor and scratch words.
The evidence report retains firmware, production-source and emulator-build
hashes. `make mica_qam_test && ./mica_qam_test` also passes.

```sh
python3 tools/mica_qam_predictor_normal_oracle.py \
  --mica ../MicaEmu --output artifacts/k56flex-response-20261009/predictor-control-normal.json
```

Both 4AF3 controller branches are now lifted and independently checked.
DAB7 coefficient adaptation, physical source/reference production and the
continuous upstream scheduler still require integration/recovery. These
checks supply vectors and configuration; they do not demonstrate acquisition
from a live waveform or a client modem CONNECT.


## DAB7 predictor adaptation profile recovered (2026-10-09)

`mica_qam_predictor_adapt()` lifts the active DAB7 profile installed
by original 6CF8/6ADD following 6BF4. Unlike DAE7's one-tap update, this
profile executes 48 tap updates in each of three complex coefficient rows
per 6910 call. Its descriptor has m70=1, m73=47, m74=0, m77=24 and
m78=95. Coefficients are single 16-bit words (m6E=0) and the adaptation
shift m75 is zero. Error pairs are taken from the logical 128-word error
ring at phase-6 through phase-1. The coefficient arithmetic preserves the
original SPM=1 complex multiply, accumulator wrap and rounded high-word store.

At tap 48, original 6B54 rebuilds two groups of two 24-word history blocks.
With the recovered m6B=0C00 and m6A=0400, these blocks read the logical
8192-word source ring at `phase-6-2*group+component-4*j`. AR3 retains each
group's origin while AR2 walks a copy; conflating those pointers gives the
wrong second component. The history prefix is retained, not cleared.
The C helper represents the retained PM window in 512 words, with refresh
at index 256. The tap walk advances by 48, subtracting 95 when its two-state
counter expires. 69B3 subtracts one once after the entire sweep; the final
history index is the initial index plus 23. This is materially different
from applying the single-tap DAE7 state update 48 times.

The helper supports this recovered profile with initial tap 0 or 48 and
remaining 0 or 1; unsupported state returns -1 before mutation. Other
adaptation profiles and firmware scheduler gates are not implicitly enabled.
These are recovered MICA operations for Draft 0.23 clause 7.18's predictor.

`tools/mica_qam_predictor_adapt_oracle.py` executes original 6BF4 and 6CF8
setup, then the unchanged 6910 adaptation and forward filter. In **128
supplied source/error/history fixtures, each run for four successive calls**,
all **512 joined sweeps** match production C: all 288 coefficient words,
all 512 retained PM words, the three descriptor state words and the six
forward outputs. Both refresh and non-refresh entry states are covered.
Input/error magnitudes are bounded to keep the forward accumulator checks
within the previously recovered FIR scope. The output cursor is returned to
the fixture's base before each call; continuous output scheduling is not
being tested. Evidence retains firmware, source and emulator-build hashes.

```sh
python3 tools/mica_qam_predictor_adapt_oracle.py \
  --mica ../MicaEmu --output artifacts/k56flex-response-20261009/predictor-adapt.json
make mica_qam_test && ./mica_qam_test
```

The core suite passes after the lift. Continuous predictor/source/error
scheduling and live upstream integration remain open. These supplied-vector
checks do not prove adaptive convergence, modem training or hardware CONNECT.


## Joined three-pair predictor block verified (2026-10-09)

`mica_qam_predictor_block()` composes the recovered 6C78 -> 4A8F
-> 4AF3 processing body. The active-profile adaptation consumes the retained
error ring first; the updated forward FIR then produces three complex pairs.
The predictor rotates those pairs, subtracts them from the raw lanes (unless
8F4F bit 4 bypasses subtraction), and writes three conjugate-rotated residual
pairs into the error ring. Only then does phase control advance the phasor.
The same phasor is used across all three pairs in the block. Mode 15 correlates
predictions with the saved original raw pairs; other modes use the residual
pairs as AR5 references. Prediction words also update DP119 4F..54, matching
8CCF's alias into that state. This preserves the recovered Draft 0.23 clause
7.18 predictor sequencing rather than chaining independently supplied stages.

`tools/mica_qam_predictor_block_oracle.py` executes unchanged original 6C50,
6910, 4A8F and 4AF3 in **128 bounded randomized single-block cases**, with
both controller modes and subtraction bypass settings. It compares all 288
coefficients, 512 history words, adaptation state, six residual/prediction
words, the full 128-word error ring, and controller state. Every comparison
passes. The optional capture callback is replaced by a standalone
RET target in the fixture; source and raw ring contents are supplied, so
this does not verify capture or source generation, or upstream acquisition.

The joined fixture also checks 6C50's actual cursor/counter effects. With
three available pairs and 44=2/45=3, it advances the logical 8192-word source
cursor by four before the FIR, advances prediction/output and raw/error
cursors by six words, increments word 46 by two, and records three consumed
pairs in word 2B. The production block accepts the post-advance source phase;
its caller still owns these scheduling steps and firmware adaptation gates.

The SARAM code image matters here: PMST=0032 maps 4A8F..4BD1 through data
128F..13D1. The fixture loads those original instructions from retained
`flex.data`, whereas the isolated predictor/controller fixtures execute
with PMST=0002 through the external program image. Leaving SARAM empty made
the first joined run execute the wrong code. Firmware and retained-data
hashes are recorded in the report.

```sh
python3 tools/mica_qam_predictor_block_oracle.py \
  --mica ../MicaEmu --output artifacts/k56flex-response-20261009/predictor-block.json
make mica_qam_test && ./mica_qam_test
```

The core suite passes. Multiple-block continuous scheduling, the actual
source generation/capture callbacks and integration with live upstream training remain
open. No hardware CONNECT or adaptive convergence is established by these
supplied-ring checks.


## Consecutive predictor scheduling recovered (2026-10-09)

`mica_qam_predictor_run()` extends the joined body with 6C50's
cadence for the recovered 44=2/45=3 profile. Word 9 supplies available pairs;
word 42 retains the 0..2-pair remainder between calls. The original four
SUBC steps determine how many three-pair blocks run. Each block increments
word 46 by two, advances the logical source cursor by four, and advances
raw/error cursors by six and the eight-word prediction/output cursor by six.
Word 2B records the consumed pairs and word 43 follows the original block
countdown through FFFF. Zero-block calls preserve DSP and cursor state while
retaining the new remainder. Unsupported profile/cursor/adaptation state is
rejected before any processing. At most 45 newly available pairs are accepted
per call, preserving the firmware's four-step block-count scope.

Logical cursors replace physical DSP addresses in the C API; the controller
state retains the recovered DP119 words. A null adaptation pointer lets the
caller apply the firmware's adaptation gates while continuing forward
processing and phase control. The source ring is supplied by the caller:
the native scheduler still does not execute the optional capture callback.
These are recovered 6C50 operations associated with Draft 0.23 clause 7.18.

`tools/mica_qam_predictor_run_oracle.py` runs the unchanged original 6C50
through **16 sequences of 24 consecutive calls**. Availability varies from
zero through 36 pairs, including both zero-block and multiple-block calls.
The 384 calls process **2,295 blocks**, comparing all coefficient/history,
raw/error rings, adaptation/controller state, retained remainder, counters
and source/raw/error/output cursors after every call. Source, raw, error and
output ring wraparound occur within the sequences. Both controller modes,
subtraction bypass and enabled/disabled adaptation are covered. The disabled
fixture uses the original zero adaptation descriptor word. The original disabled-capture entry 1A79 (RET) is used, and the bounded
source/raw rings are supplied; no live input or adaptive convergence is claimed.

```sh
python3 tools/mica_qam_predictor_run_oracle.py \
  --mica ../MicaEmu --output artifacts/k56flex-response-20261009/predictor-run.json
make mica_qam_test && ./mica_qam_test
```

The evidence report records firmware/data, production-source and emulator
hashes. The core suite passes. Actual source generation, upstream acquisition
and integration into live modem training remain open; hardware CONNECT is
still unverified.


The callback at DP119 word 69 (8CE9) is a capture consumer, not the physical
source producer: 1916 installs 19FF, which advances through 1A22 to the
previously verified 1A8C ring-copy consumer. Its default 1A79 entry is an
original RET. The consecutive-call fixture now uses that original disabled
entry directly. Earlier references to this as a source-producing callback
were incorrect. Physical source generation remains the separate 4A6C writer
and its symbol/carrier callers.


## Source carrier and phase cycles recovered (2026-10-09)

`mica_qam_source_init()` lifts 9320's preset installation from
PM650C..652C. The eleven records retain the exact start/end/step values:
profile 0 holds its phase with step zero; the remaining records select
forward or reverse walks through 12-, 15- or 35-pair carrier cycles.
Duplicate records are preserved. `mica_qam_source_emit()` now
executes the complete 4A6C source writer using the retained 124 carrier words
at PM6490..650B, so its caller no longer supplies an arbitrary phasor.
DP118 words 1B..1E retain current phase, start, stop and step. Source symbols
remain caller-supplied; 930D's transmit-mode selection and the upstream
symbol-producing dispatches are not inferred from the preset indices.
The tables carry the retained firmware hash and are associated with the
recovered Draft 0.23 clause 7.18 predictor processing.

4A89 is a conditional *delayed* return. Its two delay slots contain 4A8A's
cursor store, which therefore executes on every call. The logical source
cursor always advances by two; only the phase reset is conditional on the
new phase matching the stop value. Reading that store as conditional led
to a pending-pair hypothesis, refuted by the original-code comparison before
completion. The existing pair writer's unconditional cursor advance was
correct. No source sample is dropped or duplicated to accommodate phase cycles.

`tools/mica_qam_source_emit_oracle.py` runs original 9320 and 4A6C across
all eleven presets for 128 successive calls each: **1,408 calls and 74 phase
resets** match C. Full-range signed input pairs exercise the original fixed
point arithmetic. The fixture starts near the end of the source ring,
checks both written words, the cursor and phase after every call, and verifies
that neighboring ring words are untouched. The report retains firmware,
production-source and emulator-build hashes.

```sh
python3 tools/mica_qam_source_emit_oracle.py \
  --mica ../MicaEmu --output artifacts/k56flex-response-20261009/source-emit.json
make mica_qam_test && ./mica_qam_test
```

This recovers the carrier/phase producer used by the source writer. The
symbol mapper and transmit-mode dispatch feeding 8C0F/8C10 (or 0067/0068),
their actual invocation cadence and live receiver integration remain open.
Hardware CONNECT is still unverified.

## Startup symbol mapper joined to the source writer (2026-10-09)

The original 97A2/97A8 and 97C3 routines now have a C lift,
`mica_qam_startup_symbol()`, for Draft 0.23 clause 4.12's
parameter-record transmit path. Absolute mapping stores the low two bits
as the quadrant; differential mapping adds them to the previous quadrant
modulo four and replaces the symbol's low bits. The upper two bits select
the retained DM72A0 base pairs `(1,1), (-3,1), (1,-3), (-3,-3)`.
The amplitude in DP118 word 3E multiplies these with low-word wrapping;
PM97BB supplies the four exact quadrant rotations. Original 97B6's
amplitude table contains 9159 and 4096.

The B3CA transmit profile installs 97A2 and 97C3 as callbacks, with 93EA
between them; B1BC/B300 switch the quadrant callback to 97A8. The 9341
dispatcher calls these before passing words 8C0F/8C10 to 4A6C. This is
provenance for the mapper, not a recovered complete transmit dispatcher.

`tools/mica_qam_startup_symbol_oracle.py` matches **2,048 cases** against
the original DSP, including both mapping modes, all nibbles/quadrants,
eight amplitude values and randomized full-word inputs. The joined
`tools/mica_qam_startup_source_oracle.py` executes the original mapper and
4A6C together at explicit SPM=0: **1,408 calls and 74 phase resets**
match C across all eleven carrier presets, including ring wrap.

`mica_qam_source_emit_scaled()` preserves the caller's product
scaling selection. The joined startup fixture uses SPM=0; the earlier
full-range source-writer fixture uses SPM=1, which remains the behavior
of `mica_qam_source_emit()`. These fixture settings do not establish
the scaling mode of every live dispatcher. SPM=2 is represented by the
helper but has not been independently exercised by these fixtures.

```sh
python3 tools/mica_qam_startup_symbol_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/startup-symbol.json
python3 tools/mica_qam_startup_source_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/startup-source.json
make mica_qam_test && ./mica_qam_test
```

The bit producers at 8F6F/8F74/BE68, actual dispatcher cadence and the
2C27/9367 pulse-shaping path remain open. The fixtures supply symbol
nibbles; they do not establish a complete on-wire handshake or hardware
CONNECT. Live upstream receiver integration remains required.

### Startup word-history producers recovered

`mica_qam_startup_bits()` now lifts original 8F6F and 8F74.
8F6F shifts word 1 into word 2 and installs the supplied input word.
8F74 saves the old word 2 in word 79, shifts word 1 into word 2, and
sets word 1 to the low word of `(history >> 9) XOR (history >> 14)
XOR input`, where history is word 1 followed by word 2. The original
delayed return's final XOR/store are included. B3CA installs 8F6F;
B3BD selects 8F74 or the separate BE68 path under its profile gates.

`tools/mica_qam_startup_bits_oracle.py` compares **4,096 calls** with
the original routines: each mode has 1,024 randomized initial histories
and 1,024 consecutive updates. Words 1, 2 and 79 match after every call.
This supersedes the statement above that 8F6F/8F74 are still unrecovered.
BE68 and the selection/extraction of mapper nibbles from this word
history remain open, as do pulse shaping and live integration.

```sh
python3 tools/mica_qam_startup_bits_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/startup-bits.json
```

### Word extraction joined to startup source generation

The alternate BE68 producer is now lifted as
`mica_qam_extended_bits()`. It saves old word 2 in word 79,
shifts word 1 into word 2, and includes the original scratch word 12.
Its three sequential masked feedback steps use masks 03E0, 7C00 and
8000 with a five-bit left shift. **2,048 original-DSP comparisons**
cover randomized histories and consecutive calls.

`mica_qam_startup_take()` lifts the bounded 90FF extraction
path for widths 1..15 and remaining-bit counts 0..15. It invokes the
selected history producer only when the symbol needs another queued
word, extracts the low requested bits from the retained history, zeros
the following symbol word and updates word 4 modulo sixteen. It returns
whether the caller's next queue word was consumed. Queue refill, queue
pointers and multiword extraction are not included in this helper.
The original routine matches across **2,880 cases** covering all widths,
remaining counts and all three producer selections.

`tools/mica_qam_startup_pipeline_oracle.py` joins original 90FF,
97A2/97A8, 97C3 and 4A6C, rather than supplying mapper nibbles. It
matches C for **1,408 successive source calls and 74 phase resets**
across all eleven carrier presets, two-bit symbols, both quadrant modes
and the three producers. The fixture supplies queued words, amplitude
4096 and explicit SPM=0, and checks history, bit remainder, mapper
state, source outputs, source cursor, phase and untouched neighbors.
History is initialized at the start of each profile in both implementations.

```sh
python3 tools/mica_qam_extended_bits_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/extended-bits.json
python3 tools/mica_qam_startup_take_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/startup-take.json
python3 tools/mica_qam_startup_pipeline_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/startup-pipeline.json
make mica_qam_test && ./mica_qam_test
```

This supersedes the open producer/extraction items above for these bounded
paths. Pulse shaping, live dispatcher timing, queue integration and upstream
acquisition remain required before a foreign K56flex modem can CONNECT.

### Overlay 88 startup carrier rotation

The call through 2C27 to overlay 88 address 43D1 is a complex carrier
rotation, not a pulse shaper. 2C27 temporarily changes PMST mapping and
dispatches the address supplied in ACC. 43D1 loads two PM words at
DP118 word 13, rotates the complex pair at AR2 with SPM=1 and rounding
8000, and advances the phase pointer by two. Word 2D supplies the restart
pointer and word 2E the exclusive end. It leaves SPM=0 for its caller.

`mica_qam_startup_rotate()` lifts this bounded phase path and
uses the existing exact complex rotator. Its caller supplies the two
retained PM coefficients. `tools/mica_qam_startup_rotate_oracle.py`
matches **4,096 original-code cases**, with full-range signed symbols
and coefficients and every position in a sixteen-pair phase cycle.
Both output words and the phase pointer match, including phase wrap.
The fixture runs actual overlay 88 code, not runtime.asm's empty overlay
window. Coefficients are supplied test inputs; this does not yet recover
the profile-selected carrier tables or 9367's output/sample behavior.

```sh
python3 tools/mica_qam_startup_rotate_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/startup-rotate.json
make mica_qam_test && ./mica_qam_test
```

Earlier references to 2C27/43D1 as pulse shaping were imprecise. The
following 9367 stage and its callers remain to be recovered before claiming
the complete startup waveform. Live modem CONNECT remains unverified.

### Startup gain/history writer recovered

Original 9367 is now `mica_qam_startup_output()`. Following
43D1 it runs at SPM=0, multiplies the rotated imaginary and real words
by DP118 word 11, and writes them in that order with SACH shift 2.
There is no rounding or saturation. AR0=32 gives a 64-word bit-reversed
physical history ring; the C helper exposes the equivalent logical cursor
and advances it by two. **4,096 original-code comparisons** exercise all
64 cursor positions, full-range signed gain/symbols and ring preservation.

The joined 43D1 -> 9367 fixture also matches **4,096 cases** for both
rotated words, carrier phase, the entire history ring and its cursor.
It lets the original rotator set SPM=0 before the writer, proving this
handoff rather than imposing the writer's mode separately.

```sh
python3 tools/mica_qam_startup_output_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/startup-output.json
python3 tools/mica_qam_startup_rotate_output_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/startup-rotate-output.json
make mica_qam_test && ./mica_qam_test
```

9367 writes intermediate symbol history, not final PCM. The subsequent
8270 routine reads that history with MADS FIR processing, phase/rate
accumulators and a separate output ring. Its coefficient selection and
sample accounting remain to be lifted. This narrows the previously open
output-stage item without claiming a complete startup waveform or CONNECT.

### Startup sample-count prefix recovered

`mica_qam_startup_sample_count()` lifts 8274..827E at SPM=0
for bounded nonnegative operands and a representable five-bit quotient.
Original 8275 multiplies the caller's symbol count by DP118 word 27,
stores its low word in word 57, adds the retained remainder in word 4C,
and performs five SUBC steps against word 28. Word 4C becomes the new
remainder; word 55 holds the sample count minus one. The C helper
rejects unsupported operand ranges before changing state.

`tools/mica_qam_startup_count_oracle.py` executes the original prefix
and returns at 827F, before the FIR loop. **4,060 cases** match product,
remainder and count-minus-one, including the 10/3 ratio at every
remainder and symbol counts 1..8, plus randomized bounded ratios.
For consecutive one-symbol calls at 10/3 with initial remainder zero,
the count pattern is 3, 3, 4; the fractional part must be retained.

```sh
python3 tools/mica_qam_startup_count_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/startup-count.json
make mica_qam_test && ./mica_qam_test
```

The fixture stops before sample generation, so it does not establish
8270's FIR output, coefficient-address dispatch, history-read phase,
output-ring accounting or the live caller schedule. Those remain open.

### Startup FIR arithmetic recovered

`mica_qam_startup_fir()` lifts the 829E..82A3 arithmetic at
SPM=0 and OVM=0. MADS visits caller-selected PM coefficients forward
and the normalized 64-word history backward. Each instruction adds the
previous product before computing the next; the final LTA adds the last
pending product. SACH shift 4 stores the low sixteen bits of the arithmetic
sum shifted right twelve, without rounding or saturation. Accumulator
wrapping cannot change these selected bits, so the C helper computes the
sum in a wide integer and preserves the exact low-word result.

`tools/mica_qam_startup_fir_oracle.py` matches **2,048 cases** against
the transplanted original FIR instruction sequence, including tap counts
1..64, every logical history cursor, signed impulses and full-range
random histories/coefficients that exercise accumulator wrap. It also
checks the entire history remains unchanged. This tests the arithmetic
sequence, not the surrounding 8270 coefficient dispatch or sample loop.

```sh
python3 tools/mica_qam_startup_fir_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/startup-fir.json
make mica_qam_test && ./mica_qam_test
```

The coefficient bank selected through 8297..829D, phase-dependent history
advance, live profile tables and complete 8270 output accounting remain
open. Hardware CONNECT is still unverified.

### Startup phase and coefficient-bank selection

`mica_qam_startup_phase()` lifts 828B..8299 with a normalized
64-word history cursor. It adds word 28 to the signed phase in word 4D,
subtracts word 27 on a boundary crossing, and advances history by two
logical words only on that crossing. The coefficient-pointer lookup
address is the resulting phase plus word 2A. The caller still reads that
lookup entry to obtain the actual PM coefficient bank.

82B4 initializes word 4D to word 27 minus word 28. The original phase
arithmetic is signed; the fixture also exercises negative supplied phases.
The helper preserves them rather than forcing an unsigned phase modulo
the limit.
Its bounded interface accepts a positive limit at most 32767 and an
increment no larger than that limit, rejecting unsupported inputs before
changing state.

`tools/mica_qam_startup_phase_oracle.py` executes the original phase
prefix, stopping before 829B's coefficient lookup. **4,096 comparisons**
match the retained ACCB phase, coefficient lookup address in AR4 and
history cursor, including all history positions, negative phases and
boundary crossings. The fixture enters with ARP=2, as the real loop does;
otherwise the bit-reversed advances act on the wrong auxiliary register.

```sh
python3 tools/mica_qam_startup_phase_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/startup-phase.json
make mica_qam_test && ./mica_qam_test
```

This recovers the bounded phase selection separately from the FIR. Actual
profile coefficient tables and the joined 8270 sample loop still need
verification before treating the startup waveform as recovered.

### Joined 8270 sample loop verified (2026-10-10)

`mica_qam_startup_samples()` joins the three pieces above into the
whole original 8270: the sample-count prefix, then for each sample the phase
step, the coefficient-bank fetch and the FIR, writing a 128-word output ring.
Its banks are phase-major: bank `p` is what the DM pointer at word 2A + `p`
addresses. The bank is selected by the phase **after** that sample's step.

`tools/mica_qam_startup_samples_oracle.py` runs original 8270 from a call
stub and matches **512 blocks**: the count, words 4C/4D/55/57, the history
and output cursors, the whole output ring, the unchanged history and the
external sample counter at DM 8EEA. Half of the cases use the 10/3 ratio with
44 taps. The other half draw random ratios up to 12 and 1..64 taps. Sample
counts cover 1..31, the full range BRC can represent. Every case permutes the
DM pointer table, so a match requires reading the banks through the pointers.
Using the pre-step phase for the bank fails on the first case.

The first version of this fixture could never pass: it put the sample counter
at DM 7770, which is slot 7 of the 7700..777F output ring, so the original's
counter increment overwrote one output word. The counter now lives at 7880.

`mica_qam_test` pins the same properties without MicaEmu: the 3, 3, 4 pattern
at 10/3, one history pair per symbol, output-ring wrap, floor (not truncation)
in the FIR store, and rejection without mutation.

```sh
python3 tools/mica_qam_startup_samples_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/startup-samples.json
make mica_qam_test && ./mica_qam_test
```

This supersedes the "joined 8270 sample loop" items left open above. Still
open: the live profile's words 27/28/29/2A and its PM coefficient tables, the
caller that schedules 8270, and how the output ring and DM 8EEA reach the
codec. The fixture supplies coefficients and geometry, so the result is
verified arithmetic, not a recovered startup waveform. Hardware CONNECT is
still unverified.

## Overlay 8E transmitter: 8F23/94BA/82B4/83A3 setup (2026-10-10)

Overlay 8E (V.32bis, see above) sets up its transmitter in three
places (D698, D6D5, D886), always the same way:

| Call | What it sets (DP118 words) | Source |
|---|---|---|
| 8F23(0) | 27..32 from profile record 0 | PM 8F96[0] -> 8F9C |
| 94BA(1) | 2C = 1; 2D/2E carrier start/end; 13 = 2D (94B6); 4F; 2F..31 | PM 8FE4 + 4*2B + 2*2C; PM 2ECD + 3*4F |
| 82B4 | history ring cleared (base DM 8ED5), 1A = base, 19 one word behind; 4C = 0; 4D = 55 = 27 - 28; 4E = 6116 | |
| 83A3 | DM E504 + p = base + 46 p for p = 0..(word 27); overlay 0x11 loaded at base (DM 8EEF); 2A = E504, 29 = 2D (46 taps) | |

The six profile records at PM 8F9C are the V.34 symbol rates at 8 kHz:
10/3, 35/12, 20/7, 8/3, 5/2 and 7/3 samples per symbol (2400, 2743, 2800, 3000,
3200 and 3429 baud). The general setup at 18C5 copies the B3CA record into
words 1F.. and then calls 8F23(word 71) and 94BA(word 72), so the same
transmitter engine runs at whatever symbol rate and carrier the caller
negotiated. **Overlay 8E's three sites alone hard-code profile 0 (2400 baud) and
carrier select 1, as V.32bis requires.** Profile 0 has two carriers:
select 0 (PM 8FFC, three phasors stepping -120 degrees a symbol) is 1600 Hz,
and select 1 (PM 9002: 0, -90, 180, +90 degrees) is 1800 Hz. **K56flex passes select
1, so its startup carrier is 1800 Hz.** 43D1 applies the phasor per symbol;
the 10-phase, 23-complex-tap bank in overlay 0x11 does the rest of the
modulation, so its per-phase sums swing with the carrier. 8F23's record says
44 taps (2B) and 83A3 overrides it with 46. The final rotor table entry
(p = 10) is built but never selected.

`mica_v32bis_tx_init()` / `mica_v32bis_tx_symbol()` reproduce that
setup and the per-symbol chain 90FF, 97A2|97A8, 97C3, 43D1, 9367, 8270, with
the tables extracted into `mica_qam_tables.h`.
`tools/mica_qam_v32bis_tx_oracle.py` runs **the original setup routines**
with only the 2D3A overlay loader stubbed (overlay 0x11 preloaded at the
base), and compares all 128 DP118 words, the cleared history and the
E504 table with the C initializer. It then runs the original symbol chain
on that setup. Over 24 configurations x 384 symbols (30,720 samples) every
DP118 word (bar physical cursors and the fixture's queue pointers), both
rings and the DM 8EEA sample counter match after every symbol.

Two defects of ours this exposed:

- `mica_qam_startup_take()` did not leave word 12 = 1. 90FF stores 1
  there at 9157 to build the `1 << width` mask, after whichever producer ran,
  so it also overwrites BE68's scratch value. No earlier oracle compared
  word 12 after 90FF; the take oracle now does (its inputs randomise it).
- The producer selection lives in word 21 (B3CA/B3BD write it); the
  initializer now stores it there.

MicaEmu's `native/mica_v32bis_tx.c` / `verify_mica_v32bis_tx.py`
predate this. They supply the geometry instead of running the setup, offer a
3200-baud mode the module never selects, and patch overlays 00..0A in as
"eleven carrier presets" for 43D1. Those are not carrier tables, so that
fixture verifies arithmetic only. (The eleven presets in
`k56flex_source_profiles` are 4A6C's source-writer table at PM 650C; 931F
selects preset 0 for K56flex. That table is real and separate.)

Independent check (`mica_qam_test`): with random dibits the output has 10
samples per 3 symbols, one source word per 8 dibits, and a 600..3000 Hz
passband 46-53 dB above 300 and 3300 Hz, symmetric about 1800 Hz. Within the
band the spectrum tilts up by 1-2 dB (centroid about 1970 Hz). That is a
property of the shipped filter, not explained here.

```sh
python3 tools/mica_qam_v32bis_tx_oracle.py --mica ../MicaEmu \
  --output artifacts/k56flex-response-20261009/startup-tx.json
make mica_qam_test && ./mica_qam_test
```

Still open: the module's own width (1F), mapping mode, producer, amplitude
(3E) and gain (11) for this path; the oracle sweeps them as inputs. The
B3CA record (`2, 1, 8F6F, 97A2, 93EA, 97C3`), which 18CE copies into
words 1F..26 on the general path, gives width 2, the 8F6F producer and
absolute mapping there. Overlay 8E never copies it, so the K56flex values
are unconfirmed. Also open: what words 2F..31 and 4E mean,
the 4A6C source write each symbol, the 92FF zero-symbol prefill (D6AF runs
18), the cadence from DA51/933B, and how the output ring reaches the codec.
**This is not the K56flex upstream data rate.** With 2 bits a symbol this is
V.32bis's 4800 bit/s differential rate-signal channel, on the (9159, 9159)
points that the same overlay slices for R and E. The upstream DATA direction is the client's transmitter, which
MICA receives. The Rockwell K56flex image (kewsast3) reports its TX rate
from the same $2F28 table as the PCM ladder, whose indices 0..16 are 300 ...
28800, 31200, 33600. That table is shared with the client's V.34 mode, so it
shows the client can report up to 33600, not which rate a K56flex call
reached. The data-mode upstream receiver, and its symbol rate (presumably
selected like 18C5 does, from negotiated words, rather than fixed), has NOT
been identified. Everything recovered on the receive side so far (4-point
decisions, response/report collectors, the 5A32 loop) is the signalling path.
The firmware also does not show which handshake segments use this
transmitter on the wire; that needs a capture.
