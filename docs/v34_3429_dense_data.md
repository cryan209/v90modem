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

`reference-data/artifacts/v34-3429-diagnosis-20261008/fixed-summary.json` records eight
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
`reference-data/artifacts/eicon-v34-33600-fixed-20261008-r1/`.

The broad `make test` run found three K56flex client assertions on its A-law
32k noisy PRIME row; those reproduce with this repair disabled. A subsequent
`make -i test` was interrupted after stalling in the multi-page fax Class 2
receive test. Therefore the complete suite is not reported as passing.

## Live: the MT5634SMI's B1 switching phase (2026-10-29)

The first live 3429 calls came when the plain V.34 start profile became a
ceiling (3429/33600, `modem_engine.c` `g_v34_start_baud`) and the line probe
picked the symbol rate.  Against the RasFinder (an MT5634SMI, Lucent) every
3429 call failed the same way: MP decoded cleanly (|z| sd 0.016, 0.7 degrees
from the 4-point grid), then B1 correlated 0.22-0.24 at every offset and
trellis state, the B1 check retrained and the call often died.  The data
symbols were not corrupt -- fitted to the lattice they carry ~34-35 dB, the
same as this peer's 3200/24000 data -- so the template was wrong.

The template follows 8.2 exactly (`SWP` 14A5 MSB-first, counter zeroed
before the data frame).  This peer sends one mapping frame more than E's 20
bits before B1 (the +8 symbols `v34_rx_b1_search_symbols()` already allows
for), and its switching pattern counts that frame, so its B1 runs the pattern
one mapping frame ahead.  Wherever r == P every mapping frame is a high frame
and the two patterns coincide, which is why 3200/24000 (SWP FFFF) always
worked and 3429/33600 never did.  With the template's counter one frame
ahead, B1 is found at +8, trellis state 13, correlation 1.000, on all five
recorded failing calls (both lines).  `v34_rx_b1_search()` now tries one
frame ahead when the 8.2 phase finds nothing (`b1_swp_frames`), and the
deframer runs data at the phase it matched.  Regression row:
`reference-data/artifacts/rf-mt5634-v34-3429-swp`, which fails without the
fix.

Live A/B (alternated, same lines): with the fix 3429/33600 both ways with
login and every probe echoed; the 3200-ceiling build 24000.  The peer's
"retrain" on the failing calls was only it following ours.
