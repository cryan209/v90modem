# V.90 live interop rig (d-modem / slmodemd)

Files here support the live V.90 interop test against a real SmartLink
softmodem DSP, used to validate the digital-modem server against an actual
analogue-modem peer without physical hardware.

## `d-modem/d-modem.c`

Patched copy of the AonCyberLabs **D-Modem** bridge (`d-modem.c`) that runs in the `d-modem` container. Key changes over upstream:

- **8000↔9600 rational (6/5) resampler.** slmodemd's DSP runs at 9600 Hz;
  the SIP/RTP path is 8000 Hz G.711. The conversion must preserve *both* exact
  8 kHz symbol timing *and* near-Nyquist energy — V.90's `Sd` signal
  (`{+W,+0,+W,-W,-0,-W}`) carries most of its energy at 4 kHz.
  - `dmodem_put_frame` (net→DSP): 257-tap windowed-sinc polyphase interpolator,
    cutoff at the exact 4 kHz input Nyquist, six output phases, with a
    one-frame pipeline delay for inter-frame FIR continuity. The long
    fractional-delay kernel preserves the near-Nyquist content required by
    both Sd detection and SmartLink's Phase-4 TRN2d PDSNR check; the earlier
    12-tap kernel passed Sd but added excessive TRN2d interpolation error.
  - `dmodem_get_frame` (DSP→net): linear interpolation (upstream V.34 is band
    limited well below 3.4 kHz).
- **u-law preferred** codec ordering (the SmartLink blob is a US-market u-law
  build); run the server with `SIP_FORCE_PCMU=1`.
- Debug taps: `/tmp/dm_to_dsp.raw` (9600 Hz, what the DSP receives) and
  `/tmp/dm_from_dsp.raw` (8000 Hz, the DSP's transmit after downsampling).

Also patched (not copied here): `slmodemd/modem_cmdline.c` (`-e` option
declared `MANDATORY,STRING` to fix an upstream arg-parsing bug).

## Required env vars for a successful Phase 3 run

- `ME_V90_J_LOOKAHEAD_BITS=3000` — SmartLink's Phase-3 `Sd`-detection window
  is tighter than the normal (score-gated, 6000-bit-floor) `J`/`Ja`
  confirmation path in `v34rx.c` can meet: measured live, the normal path
  takes ~1.6-1.7s from entering the `J`-wait RX stage to confirming `Ja`,
  which is enough for SmartLink's own Phase3Demodulator to give up waiting
  for our `Sd`-to-`S̄d` transition (~3.2s from when it starts transmitting
  `Ja`) and retrain — the call never gets past "no S after Jd symbols;
  resyncing to WAIT_JA". Setting this to 3000 (~470ms) cuts the same gap to
  ~140ms, which is fast enough that SmartLink reliably detects `Sd` and
  later answers our `Jd` with a real `S`. This is the purpose-built escape
  hatch already documented next to the normal path in `v34rx.c` ("test
  rigs that need latency compensation") — do **not** change the global
  default (`0`, i.e. disabled) for this, since the strict path it bypasses
  exists to avoid a real prior false-positive bug (mid-TRN score-25/32
  matches launching `Sd` early) against real hardware modems.
- `ME_V90_JD_RESYNC_SYMBOLS=24000` — still required even with the lookahead
  fix above. SmartLink answers our `Jd` with `S` at ~19296 symbols (~2.4s)
  into the `Jd` wait; the 12000-symbol (1.5s) default fires first and resets
  us to `WAIT_JA` before that `S` ever arrives, undoing the lookahead fix.
  24000 (3s) gives headroom above the observed 19296 without being
  needlessly long.
- `ME_V90_SD_DELAY_MS=750` — **required in addition to the two above**, not
  optional. With only the two above, our side's own state machine does
  reach `S detected`/`J'd`/DIL cycling — but that is a false positive: cross-
  checked against SmartLink's own `-d9` log for the same calls, its
  `V90Demodulator: Error Energy` stays exactly `-0.000` (not noisy-small,
  literally unchanging) for the *entire* `WaitForSd` window, meaning it
  never actually received our `Sd` at all. Root cause, measured via wall-
  clock-synchronized timestamps on both sides: with `ME_V90_J_LOOKAHEAD_BITS`
  making our response fast, we start transmitting `Sd` (which only lasts
  54ms) about 763ms *before* SmartLink's own `V34HSHAKE: txstate
  JTXMIT=>JaTXMIT` / `WaitForSd`-arm sequence has even happened — its
  receiver was never listening yet. What we detected as "S" afterward was
  most likely SmartLink's own post-timeout `SILENCERETRAIN`→`TONE_AB`
  signal (a real V.34 phase-reversal tone, structurally similar to `S`),
  not real progress. §9.3.1.3 explicitly permits the digital modem to hold
  up to 500ms after detecting Ja before sending `Sd`; 750ms was measured
  live to align correctly with this specific peer.
  **Verified as real** (not another false positive) via SmartLink's own
  log: `Error Energy` shows a genuine equalizer-convergence curve (large
  initial error decaying to a small residual, e.g. `+2749`→`+3272`→
  `+1638`→...→`+3` over ~7s), `Agc Gain low > Setting DIL overflow
  protection`, a new `V90Phase3Demodulator: waitForJd` state transition,
  and — on the best run — SmartLink actually transmitting its own `End of
  CP #1 tx` (a genuine Phase 4 signal), matching our own log reaching
  `Phase 4: Ri` / `waiting for valid CPt`.

With all three set, a call has been confirmed (once, cleanly, with
multi-source verification on both sides) to progress genuinely through
`Sd`→`S̄d`→`TRN1d`→`Jd`→`S`(real)→`J'd`→Phase 4 `Ri`→CP exchange, well past
every prior stall point. A second immediate repeat attempt was
inconclusive — SmartLink's own log stopped abruptly ~160ms into that call's
`WaitForSd` window with no further output, looking like a rig-side hiccup
rather than a protocol failure, but not yet independently confirmed. This
session made 30+ back-to-back calls against the rig, well past the ~17-call
fatigue point noted from a prior session — pace further live verification
accordingly (a few calls at a time, not a rapid batch) and don't over-read
a single inconclusive run either way.

The shutdown-time SIGSEGV (`process_return_code: -11` in `manifest.json`,
whenever the interop harness's SIGINT lands after a call) is a separate,
still-open bug, not caused by any of these env vars.

## Phase 4 MP frame CRC failure (open, not yet root-caused)

With all three Phase 3 env vars above set, calls reliably reach real
Phase 4: `Ri`, `TRN` (95-100% ones-lock), explicit `J'` confirmation, and
an MP preamble lock at 17-18/18 score — but every MP0 frame then fails
CRC (`crc_ok=0 fill_ok=0`), cycling through all 8 domain/tap/order
hypothesis combinations without ever validating one, and the call
eventually retrains/hangs up. The failure is **perfectly deterministic**
— identical received frame bits every single call.

Three plausible causes were tested live and eliminated; don't retest
these without new evidence:

1. **Frame boundary slip / bit-level corruption.** Hand-decoding the
   actual received bits against Table 20/V.34 shows all structural
   checks pass early (`start17/type18/reserved19/start34/start51/start68`
   all correct) but errors accumulate later in the frame (`fill85..87`
   wrong, CRC wrong) — ruled out a slip or 1-2 bit flip via Python
   simulation against the real captured frame (no slip in ±1/±2, no
   single- or double-bit flip over the full payload+CRC region, produces
   a valid CRC). Also found and fixed a real but insufficient gap while
   investigating this: all four MP slip-recovery paths in `v34rx.c`
   structurally excluded bits 17-18 from correction (started at bit 19)
   — widened to start at 17, verified via the same simulation and live
   that it does not change the outcome.
2. **CMA equalizer drift during MP** (`ME_V34_FREEZE_CMA_DURING_MP=1`,
   built for a similar-looking symptom on the USR Courier in an earlier
   session). Tested live: changes which hypothesis gets tried but the
   decoded frame still fails CRC.
3. **Symbol-timing (Godard TED) drift during MP**
   (`ME_V34_FREEZE_TIMING_DURING_MP=1`, new). `V34_TRACE_DIAGNOSTICS=1`
   (temporarily flipped in `v34rx.c`, reverted after) showed
   `pri_symbol_sync`'s `baud_phase`/`ted_corr` sitting rock steady at
   exactly `0.0`/`0` for the entire preceding TRN period, then firing one
   large, sustained `eq_put_step` correction right at the TRN→MP boundary
   and settling into a new equilibrium — a very plausible-looking cause,
   since carrier tracking is already frozen during MP for the same class
   of risk but this timing loop wasn't. Tested live with the freeze
   applied: confirmed `eq_put_step` genuinely stayed frozen through the
   transition, but the decoded MP frame bits came out byte-for-byte
   identical anyway. Ruled out.

Because the corruption is unaffected by freezing any of the adaptive/
tracking loops and is perfectly deterministic, it looks like a **logic-
level bug**, not a signal-quality or convergence issue — most likely in
how the differential decoder or scrambler state carries over from TRN
into MP (§10.1.3.9/V.34's MP sequence uses the same scrambled-bit mapping
as TRN, differentially encoded; the encoder is meant to initialize from
TRN's final symbol). The log's own `"diff dibits collapsed"` fallback
message before falling back to absolute decode is consistent with this.

4. **Scrambler polynomial/tap selection.** Checked this specifically
   since V.90 §5.3/§6.5 *reverses* the plain-V.34 call/answer scrambler
   assignment (digital modem always uses GPC, analog modem always uses
   GPA, regardless of who actually originated the SIP call) — confirmed
   the code already accounts for this correctly (`v34rx.c` ~line 6690,
   "V.90 reverses this assignment" comment, computes `correct_tap=4` for
   our role and correctly carries it into
   `mp_phase4_default_scrambler_tap`). The very first, highest-confidence
   MP lock attempt (score 17-18/18) already uses this correct tap and
   still fails CRC, so this isn't it either — the retry-mode cycling
   through tap=17 seen in the logs is just defensive fallback search, not
   evidence the default is wrong.

Next step would be comparing raw constellation points across the TRN→MP
handoff against the expected differential-decode sequence symbol-by-
symbol (a reference-decoder comparison), not another env-var experiment
or spec-citation check — everything checkable from the spec text and
existing code structure has now been verified correct.
