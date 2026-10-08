# V.34 3429-baud dense-data receiver repair (2026-10-08)

The plain duplex receiver now carries 33600 bit/s at 3429 baud without
payload errors in the tested linear PCM, PCMU and PCMA loopbacks. This is
an offline receiver result, not a claim of 33600 hardware interoperability.

## Failure and repair

The mapper/decoder passed all 420 cases, but the complete modem accumulated
errors even over linear PCM. Short runs could pass and subsequently corrupt
thousands of bits. With the repair disabled in the same binary, a million-bit
PCMU run counted 454823/467534 errors; with it enabled both directions were clean.

Scalar B1 calibration left the dense constellation dependent on the Phase-4
equalizer solution. The receiver now reuses its existing supervised B1 fit
for plain duplex 3429-baud modes at 31200/33600 bit/s. V.34 clauses
11.4.1.1.5 and 11.4.1.2.5 provide the known B1 frame; clause 10.1.3.1 defines
the data symbols used to condition the equalizer. Buffered B1 samples are
re-equalized and replayed to preserve decoder and scrambler state. Data LMS
adaptation then uses a smaller gain (0.03 rather than the training gain).

The passband Godard timing loop also introduced data-dependent corrections
on this dense path. A normalized, averaged Mueller/Muller decision-directed
PI loop now tracks the data clock through the existing fractional-sample
scheduler. It takes ownership after Phase 4; timing is held during the short
B1 fit. This tracks clock drift rather than freezing timing for the call.
No transport samples are inserted or discarded, and no RRC coefficients,
scrambler constants or G.711 processing were changed.

Defaults apply only to plain duplex V.34 at 3429 baud and N >= 13.
`ME_V34_B1_BATCH_EQ=0` disables the new conditioning and its associated
defaults for comparison. `ME_V34_DATA_MM_TIMING`, `ME_V34_DATA_MM_GAIN` and
`ME_V34_DATA_EQ_STEP` permit diagnostic overrides.

## Validation

`artifacts/v34-3429-diagnosis-20261008/fixed-summary.json` records eight
sample delays (0, 3, 7, 11, 17, 23, 31, 40) in each of three bearers, with
one million received bits per direction:

- Linear PCM: 8/8 rows, zero errors in both directions.
- PCMU: 8/8 rows, zero errors in both directions.
- PCMA: 7/8 rows, zero errors in both directions. Delay 3 fails the earlier
  Phase-4 MP acquisition and never reaches B1/data; that startup defect remains.

The 23 completed rows check over 46 million payload bits. The clock controls
in `fixed-clock-summary.json` pass one million bits per direction at ±50 ppm
in linear PCM, ±10 ppm in PCMU and +10 ppm in PCMA. PCMA at -10 ppm fails
startup; larger drift/startup tolerance is not established.

`V34_DUPLEX_PPM` adds an optional windowed-sinc clock offset to the synthetic
analogue test channel before G.711 encoding. It changes only the offline
harness. The makefile asserts five long regression rows covering both laws,
a delayed PCMA acquisition and both signs of linear clock drift.

The rebuilt Tower binary separately passed one million bits per direction
at 33600 in both G.711 laws. Strict 33600 Eicon calls on Tower still failed:
its INFO1a selected 3200 baud and projected N=13 (31200), despite our INFO1c
offering 3429/N=14. Preserved calls are in
`artifacts/eicon-v34-33600-fixed-20261008-r1/`.

The broad `make test` run found three K56flex client assertions on its A-law
32k noisy PRIME row; those reproduce with this repair disabled. A subsequent
`make -i test` was interrupted after stalling in the multi-page fax Class 2
receive test. Therefore the complete suite is not reported as passing.
