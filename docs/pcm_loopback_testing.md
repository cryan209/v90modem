# V.90/V.92 PCM loopback measurements

The dedicated PCM matrix measures the V.90 digital-to-analogue downstream
and the V.92 analogue-to-digital PCM upstream as **separate directed
components**. It uses native coding and independent receive state, retains
failed rows, and grades exactly the requested number of seeded pseudorandom
bits. It complements the V.56 V.34/V.32bis/V.22bis line harness.

## Commands

```sh
make pcm-loopback-test                # strict datapump matrix + procedures
make pcm-data-test                    # strict datapump rows + failure controls
make pcm-procedure-test               # V.8/startup and V.92 recovery procedures
make pcm-matrix                       # diagnostic: all configured PCM rates

./pcm_ber_test --mode v90-downstream --law ulaw --drn 22 --sr 3 --bits 1000000
./pcm_ber_test --mode v92-upstream --law alaw --drn 9 --bits 1000000

python3 tools/pcm_sweep.py --all-rates --seeds 1,7 --chunks 37,80,160
python3 tools/pcm_sweep.py --modes v92-upstream --v92-drns 9 \
  --noise-rms 0,20,60,120 --delays 0,83 --seeds 1,7
```

`make test` includes `pcm-data-test`; its existing PCM tests cover the
procedures too. The focused `pcm-loopback-test` runs both groups together.
Every matrix run creates a fresh directory under `artifacts/pcm/`. Each row
has a JSON result, raw stdout and stderr. `manifest.json` records the arguments
and executable SHA-256, and `summary.json` records outcomes. `--output`
selects a new destination; an existing directory is never overwritten.

A diagnostic sweep exits 0 when all measurements ran, even if some measured
connections failed. `--require-clean` returns 1 for any measured failure.
A crash, invalid child result or wall timeout returns 2. `--timeout` is a
per-row wall-clock bound. No result with missing acquisition or incomplete
payload can pass; a zero-bit row has `ber: null`.

## V.90 downstream

The harness configures separate CPt and data CP profiles. V.90 Table 14's rate
encodings differ: CPt carries `drn + 8` bits per six symbols, while CP carries
`drn + 20`. The CPt uses DRN 12; data CP spans DRN 1..22, or 28000..56000 bit/s.
These are configured profiles, not a rate chosen by a DIL/CP negotiation.
The JSON explicitly records `rate_source: configured_profile`.

The receiver starts at Ri, sees the barred-Ri transition, demaps 400 frames of
TRN2d, validates a CRC-bearing acknowledged Type-0 MP and recognises Ed.
The **production data-frame transmitter** then sends 48 B1d frames and the
seeded payload continuously through the same scrambler/mapper state. The
analogue-role Phase 4 receiver validates B1d and delivers unpacked payload
bits to a fixed-position checker. It must reach DATA with a valid MP,
48 B1d frames, zero B1d errors and zero demap failures. These signals follow
V.90 §§8.6 and 9.4; the prescribed peer CP is supplied by the harness.

All four Sr values (0..3) are swept. Sr=0 uses differential signs; Sr=1..3
exercises spectral shaping. Lookahead is zero in this matrix. Constellations
use the full Ucode mask. This is a synthetic high-capacity profile, not a
claim about the rate or power a measured analogue line would permit.

**The downstream DS0 bytes are copied unchanged to the receiver.** There is
no G.711 re-encoding, gain adjustment, resampling, noise or filtering in this
path. This tests the codec-codeword seam of the analogue receiver, not its
D/A reconstruction, analogue equalizer or clock recovery. `--noise-rms` is
rejected for V.90. To measure analogue impairment there, a separate D/A and
analogue receiver path is required; the V.56 filter cannot be applied to DS0
codewords.

## V.92 PCM upstream

V.92 Table 30 and §6.1 encode the rate as `(drn + 17)*8000/6`; DRN 1..19
spans 24000..48000 bit/s. A twelve-symbol frame carries `2*(drn + 17)` bits.
The synthetic CPd profile uses power-of-two moduli, an odd 64-point
constellation, the 16-state trellis, no precoder/prefilter coefficients and
CPd gain 0xffff. This tests §6.4's native GPA/modulus/trellis waveform path.
It does not negotiate or measure a CPd profile for a particular line.

The analogue-role waveform transmitter sends §8.7.1's 48 B1u frames followed
by payload. Waveform samples are scaled by 120 linear units, rounded,
saturated if necessary, and encoded **once** by the simulated network G.711
A/D. The digital receiver decodes those bytes and feeds its B1u acquisition,
seven-tap equalizer and native upstream data decoder. Successful acquisition,
all requested bits and zero rejected frames/errors are required. The checker
never searches for another payload alignment after B1u.

`--noise-rms` adds deterministic AWGN before the simulated network A/D, in
signed-linear sample units. It is not a dB SNR measurement or an ITU network
model; it applies to the B1u/payload stimulus, with silent leading delay and
flush padding. The seed controls both payload and an independently maintained
noise-generator state. G.711 quantization remains present even at zero noise.
Clipping and B1u correlation are reported separately from BER.

## Timing, corruption and failure checks

`--chunk` changes only RX callback boundaries (1..160 samples). `--delay`
prefixes 0..8192 silent 8 kHz samples and preserves the entire stimulus.
It tests acquisition/frame alignment at different callback offsets, not a
round-trip echo estimate or a full delayed call dialogue. No samples are
inserted or dropped from the generated stimulus.

`--corrupt-sample` flips one transmitted G.711 sign bit as a negative control.
It is deliberate wire corruption, not a permitted live-bearer transformation
or a model of analogue noise. The regression injects it after B1d/B1u and
requires a failure with actual bit errors. Other checks cover odd bit counts,
1/37/80/160-sample callbacks, deterministic noise, invalid configurations,
strict-sweep failures and wall timeouts. Diagnostic DSP environment switches
(`ME_`, `V34_`, `V90_`, `V92_`, `VPCM_`) are removed by the sweep runner.

## Procedure coverage and boundaries

`vpcm_loopback_test --pcm-procedure-tests` runs existing V.90 V.8/Phase 2,
V.92 QC exchange, Phase 3 transitions, native TRN2u reception and mapped
V.92 Phase 4 (CPt/SUVu/CPu, CPd/SUVd, Ed/B1d) checks, in both laws where the
underlying test accepts a law. It also runs the coupled native V.90 Phase 3/4
startup sessions. `pcm-procedure-test` adds the V.92 procedure-evaluation,
modem-on-hold, hold-line/retrain, rate-negotiation, R-signal and Tone-A tests.

The older `--v92-e2e-call` startup contract is **not** evidence of a complete
native V.92 PCM-upstream call: its coupled startup deliberately runs V.90,
and its default data transport uses compatibility mapping. It remains a
separate existing test. The new matrix reports `full_call: false` and separates
V.90 downstream, V.92 PCM upstream and startup/procedure coverage explicitly.
Neither matrix proves SIP/PTY/V.42 integration, foreign-modem interoperability,
V.56 statistical conformance, or complete native V.92 call startup.

## Verification, 2026-10-05

Full diagnostic matrix: seed 1, 37-sample callbacks, no added delay/noise,
16000 checked payload bits per row. The final transmitter uses the production
V.90 data-frame API, rather than a compatibility data mapper.

| Directed component/profile | Rows | Outcome |
|---|---:|---|
| V.90 DRN 1..22, Sr 0..3, both laws | 176 | all exact |
| V.92 DRN 1..13, both laws (24000..40000 bit/s) | 26 | all exact |
| V.92 DRN 14, both laws (41333.333 bit/s) | 2 | 24 µ-law / 69 A-law bit errors |
| V.92 DRN 15..19, both laws (42666.667..48000 bit/s) | 10 | no B1u acquisition; no BER |

These failures are properties of the tested synthetic CPd/G.711/receiver
combination. They are not a measured universal maximum V.92 line rate; their
cause has not been isolated. They remain in `pcm-matrix` and are excluded from
the strict clean subset, with no relaxation of acquisition/error criteria.

Evidence: `artifacts/pcm/20261005T101003185695Z/results.jsonl` and its manifest.
The strict 120-row subset uses V.90 DRN 1/9/22 with all Sr values, V.92 DRN
1/9/13, both laws, seeds 1/7 and callbacks 37/160. All passed; evidence:
`artifacts/pcm/20261005T101009575058Z/results.jsonl`. The procedure target and
five measurement-integrity regression tests also passed.

Longer checks also passed: one million payload bits per row for V.90 DRN 22
(56000 bit/s), all Sr values and both laws (eight rows), and V.92 DRN 1/9/13,
both laws (six rows). Reports are
`artifacts/pcm/20261005T101103775956Z/results.jsonl` and
`artifacts/pcm/20261005T101102738783Z/results.jsonl`, respectively. The existing
`vpcm_loopback_test --all-tests` suite passed after the changes. The entire
repository `make test` suite was not rerun.
