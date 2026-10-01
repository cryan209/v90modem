# Plan: V.92 Phase 3 upstream receiver over a real analogue loop

Goal: the digital side's V.92 Phase 3 receiver (`v92_p3_rx.c`) acquires
Ru/uR, trains on TRN1u and decodes Ja's DIL descriptor when the analogue
modem is on a real 2-wire loop behind a single codec (VG224 -> SIP -> us),
so that 9.5.1.1.3 releases Sd. On the byte-exact SIP loopback it must behave
exactly as it does today.

Status: steps 1-3 done, 2026-10-01. Each step lists what it changes, how it is
measured, and the result that completes it. Do the steps in order: each one
produces the instrument the next one is graded by.

## What is known (measured, not assumed)

Fixture: `artifacts/apple-v92-sip-r4` (Apple USB modem as the V.92 analogue
modem dialling our digital side, `ME_V92_PCM_UPSTREAM=1`, MD=0, U_INFO=78,
u-law). The receiver is armed at G.711 sample 81440.

- `tools/v92_p3_probe` replays `server/live-rx.g711` through `v92_p3_rx.c`
  and reproduces the live failure exactly. Ru is acquired at 85008, uR at
  85272, TRN1u is entered at 85323, then `trn1u_ones_low` (48% against a
  75% gate). The receiver rehunts and never fails, so the live log is silent.
- **TRN1u is a known sequence.** V.92 8.5.7: the signs are the GPA
  scrambler (6.3) fed ones, "initialized to zero prior to the transmission
  of TRN1u", with 0 -> +L_U. No decisions are needed to know what was sent.
- `tools/v92_trn1u_bound.py` fits a least-squares T-spaced equaliser to that
  known sequence. Held-out results on r4:

  | taps | best start offset | held-out sign error | SNR |
  |---|---|---|---|
  | 1 (raw sign) | -31 | 14.2% | 0.8 dB |
  | 21 | -24 | 0.67% | 7.4 dB |
  | 41 | -20 | 0.17% | 8.4 dB |
  | 21, 600-symbol window | -24 | 0.00% | 10.6 dB |

  Four conclusions:
  1. **The receiver's 48% is mostly ours.** At the true start the raw sign
     is right 86% of the time. Two things turn that into 48%:
     - TRN1u is declared ~25-31 symbols late.
     - The lock test runs signs through the self-synchronising descrambler,
       which turns each sign error into about three bit errors.
  2. **The channel is linearly equalisable.** 21-41 taps give a usable
     2-level eye.
  3. **Clock drift matters.** Shorter windows fit better, consistent with
     the 163 ppm offset measured between the Apple codec and the VG224
     (`docs/apple_usb_modem_sm56.md`). A fixed filter is not enough; the
     receiver has to track timing.
  4. **Level and bearer are not the problem.** Ru arrives as a clean
     1333 Hz line, TRN1u's energy is flat to Nyquist, and its DC offset is
     0.8 counts.

- The answer for Sd is different. 8.4.4's Sd puts two thirds of its energy at
  4000 Hz and the loop removes it. That is the analogue side's problem
  (`v90a_sd_line()` handles it in the V.90 analogue role). It is in scope
  only as the next blocker (step 9).

## Constraints

- No change to G.711 handling, transmit timing or sample accounting
  (CLAUDE.md constraints 1 and 3). The equaliser and interpolator change
  *where* a symbol is observed, never how many codewords are consumed.
- Ideal bearer behaviour unchanged. On a byte-exact DS0 the equaliser must
  converge to a unit centre tap. The existing `v92_startup_test` and
  `vpcm_loopback_test --all-tests` rows must produce identical decisions.
- Knob `ME_V92_P3_EQ` (default on once step 8 passes; `0` restores the raw
  sign path) so a regression can be isolated with one variable.
- Cite 8.5.x / 9.5.1.1.x in comments and commit messages.

## Steps

### 1. A tracked fixture and a failing test -- DONE 2026-10-01

- **Fixture.** `artifacts/v92-loop-upstream/live-rx.g711` is r4 bytes
  80000..99999 (2.5 s), tracked with `git add -f`. Its README gives the
  provenance, the rebased positions (arm 1440, Ru 5008, Ru-bar 5272) and the
  descriptor the analogue side sent (`measurement-120x66`: N=120, LSP=12,
  LTP=11).
- **The step-1 caveat is resolved.** The analogue side sends exactly 2040T
  of TRN1u, then repeats Ja for 12000T, so the fixture contains Ja several
  times over.
- **Test.** `v92_p3_rx_line_test` requires a CRC-valid Ja carrying exactly
  that descriptor. A missing fixture fails; it does not skip.
- **Deviation from the plan: the test is NOT in `make test` yet.** Following
  `eicon-rx-test`'s precedent (a red-by-default suite stops being read), it
  runs as `make v92-loop-rx-test` with `--expect-failure`. In that mode it
  passes only while:
  - Ru and Ru-bar are still acquired at 5008/5272 (+/-16),
  - TRN1u is entered,
  - and Ja does not decode.

  So it goes red on an acquisition regression *and* on the fix. Move it into
  `make test`, without the flag, at step 6.
- **Control: the receiver's back end works.** The equalised-sign control
  (an LS-seeded 41-tap equaliser whose decisions are written back as
  codewords) makes the *unchanged* receiver pass TRN1u and decode the right
  descriptor at 17733. The test, run on that control file, reports `PASS`
  without the flag and "expected failure did not occur" with it. Two things
  are left for later steps:
  - It needed 79 Ja rejects first. That is step 6's to explain.
  - The receiver also takes a false Ru -> Ru-bar -> TRN1u lock at 18214,
    inside the repeated Ja.

### 2. A synthetic loop channel the loopback cannot provide -- DONE 2026-10-01

Every existing V.92 receive test is fed a byte-exact DS0, which is why none
of this showed up.

- **`v92_line_channel.c`** (test-only, not in `SRCS`) sits between the
  analogue modem's 16 kHz audio and the network ADC. It applies, in order:
  1. an FIR at 16 kHz,
  2. a fractional A/D sampling phase,
  3. a ppm clock offset (Blackman-windowed sinc interpolation),
  4. additive Gaussian noise.

  G.711 quantisation stays with the caller, once. With an ideal
  configuration it passes every other input sample exactly, as the harness's
  network ADC always has.
- **The channel is the real r4 loop.** `tools/v92_fit_line_channel.py`
  fits our own 16 kHz upstream (`v92_p3_rx_line_test --dump-tx`) to the
  fixture.
  - TRN1u anchors the fit, since its scrambler is zero-initialised and ours
    and the call's are the same symbols.
  - The script searches alignment and clock offset and solves the taps by
    least squares, scored held out.
  - Taps vs held-out R²: 32 taps 0.988 (19 dB), 64 taps 0.997 (25 dB),
    96 taps 0.9985 (28 dB), 128 and 160 taps 0.9989 (~29.5 dB). The plateau
    sits at about the line's own noise floor.
  - The clock offset is only loosely determined in these windows, landing
    anywhere between +100 and +200 ppm, but its sign and size agree with the
    163 ppm measured off the Phase 2 carrier.
  - Shipped: 96 taps over 1800 symbols, written to the generated table
    `v92_line_channel_r4.h`, normalised to unit gain at 1333 Hz.
  - |H| is fairly flat in magnitude (0.85 at 300 Hz, 1.30 at 3400 Hz).
    What defeats the slicer is its phase response.
- **The model is as hard as the real line, not harder.** Graded with
  `tools/v92_trn1u_bound.py` on `--dump-row` output:

  | stream | raw sign err | 21 taps | 41 taps |
  |---|---|---|---|
  | real fixture | 13.4% | 8.2 dB | 9.3 dB |
  | synthetic r4 loop | 13.8% | 10.5 dB | 12.6 dB |
  | synthetic loop + phase .5 + 163 ppm + 25 dB, A-law | 14.6% | 8.6 dB | 10.4 dB |

  The equalised held-out sign error is 0-0.4% everywhere.
- **Fourteen rows in `v92_p3_rx_line_test`** (`make v92-loop-rx-test`): the
  fixture plus thirteen synthetic.
  - Ideal u-law/A-law, phase 0.5 alone, +200 ppm alone and 25 dB noise alone
    all decode Ja today. Without ISI the raw slicer survives each of these.
  - Every row with the loop's ISI fails `trn1u_ones_low`, exactly as the
    recording does:
    - the loop alone
    - phase 0.25/0.5/0.75
    - +/-200 ppm
    - 25 dB noise
    - an A-law row with phase 0.5, +163 ppm and 25 dB noise together
  - Each row carries its expected outcome today, `expect_pass_today`.
    `--expect-failure` checks each row against it, so a row that starts
    passing is flagged rather than missed. Without the flag, every row must
    pass.
- **Also seen:** even on the ideal row the receiver enters TRN1u ~30 symbols
  late (1007 against ~977). So the late TRN1u start on the fixture is the
  receiver's detection latency, not something the loop does. Step 3 removes
  it.

### 3. Find TRN1u's first symbol from its known sequence -- DONE 2026-10-01

Replace "uR ended, so TRN1u starts here" with a correlation:

- After uR is detected, correlate the raw received signal against the
  known TRN1u reference over a window around the nominal start (±64
  symbols, 256 symbols long).
- Take the peak. A 2-level known sequence with 23-bit GPA structure has a
  sharp autocorrelation, so the peak is unambiguous even at 14% raw sign
  error.
- Record the start. It is also 9.5.1.1.10's modulo-12 frame anchor for the
  second TRN1u.

Done when the fixture's start lands within ±2 symbols of the bound tool's
1-tap offset, and on every synthetic row.

**Result.**

- `trn1u_align()` in `v92_p3_rx.c` runs once the nominal start + 64 + 256
  symbols have arrived.
  - It correlates received **signs** against the 8.5.7 reference. Signs
    only, because the receiver is not told the G.711 law and the MSB is the
    sign in both.
  - It moves `trn1u_start` and records `trn1u_nominal_start`,
    `trn1u_align_offset`, `trn1u_align_score_x1000` and `trn1u_inverted`
    (line polarity, which `trn1u_process()` now honours).
  - It replays the descrambler from the aligned start with a zero register,
    which is the transmitter's own initial state there.
  - A peak under 0.300 keeps the nominal start.
- Positions, all exactly the bound tool's 1-tap figure, asserted to ±2 by
  `make v92-loop-rx-test`:
  - fixture 5292 (nominal 5323, score 0.734);
  - ideal rows 976 (nominal 1007, 1.000; 0.914 at +200 ppm);
  - r4 loop rows 979-980 (scores 0.57-0.74).
- **The scores already separate real TRN1u from false locks.** The
  fixture's false Ru lock inside Ja (18222) and the loop rows' rehunts score
  0.14-0.195, against 0.57-1.0 for real TRN1u. That is what step 5 can gate
  on.
- **Two things had to change with it, and neither moves any outcome:**
  - **The "eq" fallback in the TRN1u ones checks was removed.** It ran
    `p3_demod`, a V.34 passband demodulator, over 24 hypotheses on baseband
    PCM. It had never run, because it needed one more codeword of history
    than `enter_trn1u()` buffered, so it returned -1 every time. The deeper
    alignment history woke it up, and it "passed" the fixture's false lock
    at 73%.
  - **The Ja search cadence is now counted from the old 23-symbol seed**
    (`ja_buf_lead`). The extra 65 symbols of prehistory had moved every
    144-symbol search probe, and with it the instant Ja is declared and Sd
    starts, 65 symbols earlier. That alone broke `v92_startup_test`'s
    u-law, measured-DIL, reconstructed-audio case with an analogue `Sd-bar
    timeout`. The analogue side's Sd acquisition still fits at score 1.000
    there, and the core then never sees Sd-bar. So **the analogue receiver's
    Sd-to-Sd-bar handling depends on where Sd falls relative to its 64-symbol
    acquisition grid.** That is a separate defect in
    `v92_analogue_audio.c`, not addressed here.
- Unchanged: `v92_startup_test` 51/51, `vpcm_loopback_test --all-tests`,
  `v92_proc_eval_test`, and every synthetic row's outcome. TRN1u still fails
  on the loop, as it should until steps 4-5.

### 4. Train an equaliser on the known sequence and track timing

New front end, `v92_p3_eq.c`, linked wherever `v92_p3_rx.o` is (`SRCS`
plus each `*_OBJS` that needs it).

- **Equaliser:** symbol-spaced, 31 taps by default.
- **Initial taps:** a block least-squares solve over the first 256 TRN1u
  symbols against the known reference, the computation the bound tool
  already does. This avoids spending most of 2040T on LMS convergence.
- **Then:** data-aided normalised LMS against the reference for the rest of
  TRN1u.
- **Timing:** a fractional interpolator in front of the equaliser, driven by
  a slow-averaged Mueller and Muller error. Reuse the pattern in
  `v92_trn2u_demod_feed_adaptive()`, including its note that the
  instantaneous M&M value at one sample per symbol is too jittery to apply
  directly. Its bound: ±500 ppm. Its gain: enough to follow 163 ppm, i.e.
  one sample every ~0.77 s.
- **Output:** per-symbol soft value and sign decision, sign agreement with
  the reference over the last 256 symbols, and equalised SNR.
- **Done when:** on the fixture, sign agreement is >= 99.5% from symbol 256
  of TRN1u to its end. Equalised SNR must be within 2 dB of the bound
  tool's 41-tap figure for the same window (more is fine, since the bound
  tool has no timing tracking).

### 5. Replace the TRN1u lock metric

`trn1u_ones_low` runs descrambled ones, a metric that roughly triples the
error rate.

- Gate instead on sign agreement with the known reference: >= 95% over 256
  symbols, after the step-4 block solve.
- Keep the descrambled-ones figure as a logged diagnostic only.
- Report the reject with both numbers so the probe says which one failed.
- **Done when:** the fixture passes the gate, the ideal rows still pass,
  and a deliberately wrong start (start + 3) fails it.

### 6. Decode Ja from equalised decisions

Ja (8.5.4) is scrambled and differentially encoded, seeded with the final
TRN1u symbol, still ±L_U. `v92_ja_dil_search()` currently reads signs from
raw codewords (`ja_sign_from_sample()`).

- Give it a variant that takes a sign/decision buffer. Do not fake codewords
  with a forced MSB.
- Keep the equaliser running decision-directed through Ja; there is no
  reference past TRN1u.
- 9.5.1.1.3 only requires "after receiving the first 2040T ... condition its
  receiver to receive Ja". So keep the rolling Ja window (6144 symbols) and
  search the equalised stream the same way the raw one is searched now.
- **Done when:** the fixture decodes a CRC-valid Table 20 descriptor (or,
  per the step-1 caveat, the synthetic rows do), and the ideal rows decode
  the same descriptor as today, bit for bit.

### 7. Hand the trained equaliser on

The rest of Phase 3's upstream arrives on the same channel:

- Su/Su-bar (8.5.6, levels ±√(3/2)·L_U and 0)
- the second TRN1u (9.5.1.1.10)
- CPt (8.5.3)

The second TRN1u is also scrambler-zero-initialised, so it is a second known
training interval.

- Today `v92_su.c` reads raw codewords and the CPt demod
  (`v92_trn2u_demod`) has its own 5-tap adaptive filter.
- Route both through the step-4 equaliser state: freeze through Su, retrain
  on the second TRN1u.
- Grade on the synthetic rows. There is no live fixture yet: r4 never got
  that far.
- **Done when:** the impaired rows of `v92_startup_test` reach Phase 4 CPt
  on the digital side.

### 8. Engine integration and diagnostics

- Use the new front end in `modem_engine.c`'s V.92 Phase 3 path, behind
  `ME_V92_P3_EQ`.
- Extend `v92_p3_rx_report_progress_locked()` to log once per stage:
  - the TRN1u start found
  - sign agreement and equalised SNR
  - timing drift in ppm
  - the Ja search outcome
- A live call must never again say only "armed" and then nothing.
- `tools/v92_p3_probe` gains `--eq` / `--no-eq` for one-variable A/B on any
  recording.
- **Done when:** `make test` is green with the knob at both settings, and
  the fixture test passes only with it on.

### 9. Live verification, and the next blocker

- Re-run `artifacts/apple-v92-sip-r4/run.sh` into a new directory, with the
  server on tower as before.
- **Success:** the server log shows the Ja descriptor parsed and `Sd`
  started within 9.5.1.1.3's 500 ms, with Sd present in the server's own
  `live-tx.g711` (check the tap, not the log).
- Expect it to stop next on the analogue side's Sd detection. V.92's
  analogue controller (`v92a_t`) has not had the `v90a_sd_line()` treatment
  the V.90 analogue role got, and the loop removes two thirds of Sd. That is
  the next plan, not this one.

### 10. Separate, parallel: lock the analogue modem's clock

V.92 6.2 makes the upstream symbol rate 8000 symbol/s "derived from the
digital network". Our Apple analogue side free-runs at 163 ppm off the
VG224. Step 4's timing loop has to cope anyway: during the first TRN1u
the analogue modem has only Phase 2 to estimate the clock from. Even so,
a conformant analogue modem slaves its transmit clock to the downstream.

- Drive the existing fractional clock adjustment in `v92a_audio_t` from a
  downstream clock estimate. The Phase 2 CC carrier gives ±1 ppm (measured,
  `docs/apple_usb_modem_sm56.md`).
- This is analogue-side work. It does not block steps 1-9 and should not be
  mixed into the same commits.

## Out of scope

- Quick Connect.
- §9.7 retrain discrimination.
- Upstream rate selection.
- The analogue side's downstream receiver: Sd at Nyquist, TRN1d CMA (step 9
  names it as the next blocker).
