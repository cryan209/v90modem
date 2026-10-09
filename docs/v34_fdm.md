# V.34 FDM: N unmodified 8 kHz modems over one wideband channel

`v34_fdm_test` is the "poor man's OFDM" experiment. N ordinary
`v34_state_t` pairs (caller and answerer, exactly as `v34_duplex_test`
runs them) each get a 4 kHz slot of a 48 or 96 kHz linear channel. A
channel bank moves each pair's 8 kHz line signal into its slot and back out.
Nothing but the wideband waveform crosses the seam, so every slot trains,
probes its own slot in Phase 2 and negotiates its own rate from the
waveform alone.

The question it answers: how far does a bank of V.34s get on a wide channel
before something OFDM-like is needed? The OFDM estimate for a good 96 kHz
interface was ~1 Mbit/s; the V.34 bank's ceiling is 12 x 33.6 kbit/s each way.

## Channel bank

Slot k is `[4000k, 4000k+4000)` Hz. A real 8 kHz signal fills 0..4000 Hz,
so the shift must be single-sideband or each slot's negative-frequency image
lands on its neighbour:

- TX: `x[n] e^{-j pi n/2}` puts the wanted half at -1960..+1880 Hz at 8 kHz.
  L-fold polyphase interpolation keeps only that half. `2 Re{u e^{j2pi(4000k+2000)m/fs}}`
  is added into the wideband stream.
- RX: the mirror image, i.e. mix down, then low-pass and decimate, then
  `2 Re{v e^{+j pi n/2}}`.

One Kaiser low-pass prototype (cutoff 1960 Hz at the 8 kHz baseband,
`FDM_TAPS` per 8 kHz sample, default 96, beta 7) serves both directions. The
bearer is four-wire: two separate wideband streams, as a stereo sound card
looped L->L and R->R gives. So there is no echo path. The composite is
scaled to `FDM_LEVEL_DBFS` (-20) and quantised to 16 bits, which puts the
sound-card floor in the loop.

## Results (2026-10-10, offline, 96 kHz, 12 slots)

| per-slot request | outcome | aggregate (both directions) |
|---|---|---|
| 3200/21600 | 12/12 clean | 518.4 kbit/s |
| 3200/26400 | 12/12 clean | 633.6 kbit/s |
| 3429/28800 | 12/12 clean | 691.2 kbit/s |
| 3429/33600 | **12/12 clean** | **806.4 kbit/s (403.2 each way)** |
| 3200/28800, 96 taps | 12/12 clean | 691.2 kbit/s |
| 3200/31200, 96 taps | 11/12 (one direction of one slot: 128 errors) | -- |

Each run takes ~7 s simulated and ~7 s CPU, so the bank runs at about real
time for 24 modems.

- **The slots do not interfere.** A single slot alone fails exactly where a
  full bank fails. 3200/28800 on one slot, swept over `FDM_DELAY`, passes
  11/16 at 160 taps, 16/16 at 96 and 14/16 at 320. That is not monotonic in
  filter quality, because filter length moves the total group delay. Group
  delay sets where symbols land on the 8 kHz grid, and that is the known
  acquisition coin flip of this row: plain `v34_duplex_test 3200 28800`
  fails at `V34_DUPLEX_DELAY=6` and 11. **Score a row as a pass rate over
  `FDM_DELAY`, never one run.**
- **Start the slots staggered** (`FDM_STAGGER_MS`, default 37). Started in
  lockstep, twelve modems send the same training tones at the same instant
  and the composite peaked coherently: 22 dB crest and 112 clipped samples
  at -22 dBFS RMS. The crest figure the harness prints averages RMS over the
  stagger lead-in too, so read clipping, not crest.

## Bit loading by V.34's own negotiation

`FDM_SLOT_SNR=50,48,45,42,40,38,35,32,30,27,24,20` (in-band noise per slot,
dB under that slot's own signal) at a 3429/33600 request:

- **Default:** every slot asks for 33600. The probe does not see
  signal-proportional noise (the old Phase-2 finding). Slots at >= 38 dB
  carry it cleanly, 35 dB takes errors, and below that the slots fail.
- **`ME_V34_TRN_RATE=1`:** each slot's MP takes the Phase-4 TRN
  measurement. Rates fall with the noise: 28800/26400 at 48 dB, 24000 at
  32-35 dB, 21600 at 27-30 dB, 19200 at 24 dB, 16800 at 20 dB. 11 of 12
  slots ran clean (513.6 kbit/s); slot 3 did not train, the coin flip above.
  The measurement saturates near 34 dB, which is the known limitation in
  `v34tx.c`: it reads the receiver's own Phase-4 convergence. So clean slots
  ask 24000-28800 where they would carry 33600.

So V.34 already does per-slot bit loading. The gap to OFDM is its 33.6 kbit/s
constellation cap (a 90 dB slot still gets ~10 bits/symbol), the roll-off
and guard band in each slot, and a rate measurement that cannot read above
~34 dB.

## Not done

- No sound card yet. The next step is the same bank on CoreAudio in stereo
  loopback (`FDM_TAP` writes the two wideband streams for a first look).
- No clock offset between the wideband ends (`FDM_DELAY` gives fractional
  8 kHz delays only).
- No echo, i.e. two-wire operation.
- No byte striping across slots: payload is per-slot PRBS.
- 48 kHz, 6 slots, 3429/33600: 6/6 clean, 403.2 kbit/s aggregate, from one run. Not swept.
