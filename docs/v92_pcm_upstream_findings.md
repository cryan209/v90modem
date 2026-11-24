# V.92 PCM upstream against the d-modem/slmodemd rig — findings

**Historical investigation.** The July conclusion below that this rig cannot
validate PCM upstream was superseded by the later startup/receiver fixes in
[v92_quick_connect.md](v92_quick_connect.md). The November
[bit-level and trellis investigation](v92_upstream_trellis.md) reproduces the
peer's ideal transmitted symbols and identifies our higher-rate decoder limitation.
Do not treat the old malformed-transmitter conclusion as the current rig status.

Investigation of 2026-07-21. Outcome: **we can negotiate real V.92 with this rig, but
it cannot validate V.92 PCM upstream — its transmitter for that path is malformed.**
The startup work is sound and worth keeping; the upstream needs a different peer.

Related: `rig/slmodemd/README.md` (peer-side patch), `rig/d-modem/d-modem.c` (taps),
`docs/v92_short_phase1_plan.md`.

## 1. Why the peer always chose V.90

The peer confirmed V.92 capability at INFO0 and then selected V.90 every call. Its own
`-d9` log says why, in the same instant it consumes our INFO1d:

```
upstream selection: local cap - 1, remote cap - 1, requested in info1 - 32, isPCM - 0
P2 REPORT : V92 capabilities: local=1 , remote=1 , selected=90
```

PCM upstream is V.92's only data-pump gain over V.90, and INFO1d **Table 17 bit 70** —
the bit advertising it — was hardcoded to zero in `v34tx.c`. The peer correctly
concluded there was nothing left to select.

`v34_set_v92_pcm_upstream_capability()` now drives that bit, behind
`ME_V92_PCM_UPSTREAM`, **off by default**: our upstream data path is still V.34/V.22bis,
so setting it claims a data-mode receiver we do not have.

## 2. The peer's "V.92Lite" refusal is one hardcoded-clear bit

With bit 70 set, the peer refused and retrained:

```
we got PCM upstream under V.92Lite (after Info1d), retraining to V.34 upstream...
```

Disassembly of `V34GiveINFO1dBits` (slmodemd 0x2aa30):

```
isPCM = local_cap && remote_cap && (our INFO1d bits[0xe] & 0x20)
if (!isPCM) return 0
p = *(v34obj + 0xac3c)                    /* = m->dp_runtime */
if ((signed char)p[2] < 0) return 0       /* PCM upstream ACCEPTED */
log("... under V.92Lite ..."); InitiateRetrain(90); return 1
```

`dp_runtime` byte 2 is a flags bitfield: bit 4 = V.92 mode, bit 6 = `DSPINFO[8]&1`,
**bit 7 = PCM upstream permitted**. `dp_runtime_create` clears bit 7 unconditionally
(the `andb $0x7f` sits on the merge point of both branches) and nothing in the linked
binary ever sets it. So the blob's PCM-upstream path — including
`V34SetINFO1aBits`'s `PCM Upstream is selected (info1)...` — is complete but
**unreachable**. "V.92Lite" is one dead bit, not missing code.

`rig/slmodemd/v92-pcm-upstream.patch` sets it. With both sides advertising:

```
P2 REPORT : V92 capabilities: local=1 , remote=1 , selected=92
VPcmV34Main: PCM Upstream is selected (info1)...
```

## 3. The first real Table 19 INFO1a falsified three assumptions

`get_strict_v90_info1a_locked()` had been written from the spec text with no ground
truth. The first genuine Table 19 frame (raw `3f c0 89 fd 0f 09 44`):

| field | code expected | actual |
|-------|---------------|--------|
| `raw_12_17` | 0 | **63** |
| `raw_32_33` | 0x2 | **0** |
| upstream rate code | 3..5 | **6** |

Upstream rate code **6 = 8000 sym/s is the PCM upstream itself** — V.92 replaces the
V.34 upstream carrier rather than naming one. That makes 6 the one unambiguous marker
that the peer selected V.92. Anything else now falls through to the V.90 rules even
under a mutual-V.92 INFO0 contract, which is the demotion path.

## 4. Phase 3 fixes (clause 9.5.1.1)

- **MD=0 short flow (9.5.1.1.1).** `v92_p3_rx` had no knowledge of INFO1a's MD length
  and always ran the MD-bearing path, burning its 8000-sample timeout waiting for a
  second Ru/uR pair a peer signalling MD=0 never sends. Fixed; `md_timeout` 2 → 0.
- **Ja is a CRC-valid descriptor, not an energy edge (9.5.1.1.3).** The V.90
  energy-gap heuristic fired on the V.92 upstream's pre-Ja gaps and started Sd during
  the peer's own training — which is why the peer logged `Error Energy = -0.000` for
  entire calls. Gated off under V.92.
- **TRN1u minimum (9.5.1.1.3).** `TRN1U_MIN_SOFT_T` was 256 against the spec's 2040,
  so soft mode searched for Ja *inside* the peer's TRN1u.

## 5. What the upstream signal actually is

Measured over the Phase 3 window (10.0–14.5 s) with a **pre-interpolation tap** added
to `dmodem_get_frame` (`/tmp/dm_from_dsp_9600.raw`, the DSP's native 9600 Hz output —
`dm_from_dsp.raw` is written *after* the 6/5 resample and cannot separate "resampler
destroyed it" from "DSP never produced it"):

- Strong cyclostationary line at exactly **1600 Hz in x²**, with **no periodicity in
  x** (autocorrelation at lags 5/6/12 ≈ 0; 6-sample fold coherence 0.009). That
  signature is a linear modulation at **1600 symbols/second**.
- Band-limited ~600–3600 Hz. V.34 controls show 2333/2343 Hz lines by comparison.
- 1600 sym/s is neither a V.34 rate nor the 8000 sym/s V.92 PCM upstream requires.

**Conclusion:** we enabled a transmit path that had never executed in this build, and
it emits a signal matching neither the spec nor any receiver worth building. No
receiver work on our side will complete a V.92 PCM-upstream call against this rig.
Treat V.92 upstream captures from it as invalid truth data.

## 6. Hypotheses that were tested and refuted — do not re-chase

| hypothesis | how it died |
|---|---|
| Equalizer loses lock across TRN1u | `pll_alpha` is declared and never applied in `p3_demod.c` — a real latent bug — but wiring it in left the numbers **byte-identical**. Reverted. |
| Sd mistimed vs the peer's 2.6 s demodulator lag | `ME_V90_SD_DELAY_MS=2600` changed nothing; the peer's window was open ~15 s of an 18 s call. |
| Peer never transmits in Phase 3 | `txstate SILENCE` is only the **V.34 handshake** machine; the V92Modulator has the line. Measured RMS is a steady ~1250 throughout. |
| Signal is PCM-domain ±LU we mis-descramble | 8.5.7 does say TRN1u is **not** differentially encoded (contrast Jd/Jp in 8.6.2/8.6.3), and `trn1u_process()` differences anyway — a real doc/code discrepancy. But brute-forcing all four invert × differential combinations never exceeds **53%** ones where the spec demands ~100%. Reverted. |
| The 6/5 linear interpolator destroys the upstream | The 1600 Hz line is strongest **already at 9600 Hz pre-interpolation**. Exonerated. |
| The 1600 Hz line is Phase-3-specific | Present everywhere (+22 dB in V.34 controls). An earlier reading to the contrary came from listing only top-5 lines. |

## 7. Method notes

- **Work offline, not on the rig.** `tools/v92_p3_rx_smoketest.c` drives `v92_p3_rx`
  over a saved capture; `vpcm_decode --p3` and `--p3-symbol-export` give per-symbol
  I/Q. Seconds per iteration and deterministic, vs ~2 min per rig call with real
  run-to-run variance.
- `v92_p3_rx_smoketest.c` has no makefile target; build it by hand with
  `-I. -Ispandsp-master/src -Ispandsp-master/src/.. -I$(brew --prefix libtiff)/include`
  and link `spandsp-master/src/.libs/libspandsp.a -ltiff -lssl -lcrypto -lm`.
- The RTP path is transparent: DSP tap and our received capture match to 0.1 dB.

## 8. 2026-07-23: the upstream descrambler polynomial was wrong (GPC where GPA belongs)

Found during a V.90 §5 conformance audit and fixed tree-wide. V.92 §6.3 mandates
**GPA = eq 7-2/V.34 = 1 + x^-5 + x^-23** (delay taps 5/23 → `reg>>4 ^ reg>>22`)
for everything the analogue modem transmits, but every V.92-native upstream
descrambler in the tree — `gpa_descramble()` and `gpa_descramble_t17_bit()` in
`v92_p3_rx.c`, `v92_gpa_scramble()`/`v92_trn2u_descramble()` in `v92_trn2u.c`,
`ja_descramble_reg_bit()` in `v92_ja_decode.c` — read `reg>>17` = delay tap 18 =
**GPC**. Root cause: GPA's positive-power form x^23 + x^18 + 1 was misread as
having a delay tap at 18. In positive-power form the x^18 term corresponds to the
**5-symbol** delay (x^23 · x^-5); the degree-N-term-equals-tap reading is exactly
backwards. The `v92_trn2u.h` descrambler-mode enum had its GPA/GPC prefixes
swapped the same way (both register reflections).

**Validation.** The §6 TRN1u check cannot be re-run meaningfully: the forced-
upstream captures it was measured on are invalid truth data (§5 — 1600 sym/s
non-signal), so no polynomial decodes them. The decisive evidence is in-tree and
live-measured on the same peer family under the same analogue-modem-GPA mandate
(V.90 §6.5 ≡ V.92 §6.3): spandsp `v34rx.c`'s V.90 branch records that
SmartLink's real upstream resolves at **96–99% ones with tap 4 vs ~52% with
tap 17**, and spandsp's role assignment (`v34tx.c`: answerer tx tap = 4, caller
tx tap = 17) fixes the tap↔polynomial mapping. The §6 refuted-hypothesis row
("never exceeds 53% ones") brute-forced invert × differential **at fixed
tap 17** — the polynomial was never varied, and ~53% is the wrong-tap signature,
though on those captures the malformed transmitter suffices as explanation.

**State after the fix:** all upstream sites use taps 4/22; the enum labels now
mean what they say (results recorded against the old labels mean the other
polynomial); TX and RX moved together so the trn2u loopback tests still pass.
The offline Agere/Motorola/USR QC sweeps (`docs/v92_phase4_implementation.md`)
cannot discriminate the polynomial yet — they fail at symbol recovery upstream
of the descrambler — and live V.92 Phase 3 validation remains blocked by the
peer's V.90-form INFO1a, so a live all-ones TRN1u confirmation is still pending.

## A peer that rounds its precoder feedback or offsets its negative points (2026-11-24)

D-Modem's interop notes (d-modem.github.io/D-Modem/interop-details/, ESS
ES56H-PI) report two departures from §6.4.2 in that analogue modem, in CPd
point units where its grid step is 16: its precoder rounds the feedback down
to the grid, x(n) = u(n) + 16 floor(fb/16); and with no precoder
coefficients each negative point is sent one step (16) farther out. Before
they corrected for it, B1u had about 5% of its bits wrong. Our receiver has
no correction for either. `V92_UPSTREAM_QUIRK_FB_FLOOR` and
`V92_UPSTREAM_QUIRK_NEG_STEP` (`v92_upstream_data.h`; on our analogue
transmitter with `V92_TEST_UPSTREAM_QUIRKS=1/2/3`) reproduce them.

**Engine pair, no added noise:** all eight arms (quirks 0-3, precoder default
or forced with `ME_V92_PRECODER=2`) connect at 48000 up, lock B1u by
decoding, and pass a V.14 payload byte-exact. 16 point units are about 8 DS0
on that line; the CPd's closest spacing is 80 units (about 40 DS0) against
4.4 DS0 of noise. The engine pair's `--loop-noise-db` cannot test further:
it adds noise both ways and the V.14 start-up junk fails the payload check
without any quirk.

**Upstream alone:** the equalised B1u receiver, fed the engine's 96-point
CPd (4G = 22, drn 19, 48000) through mu-law with added Gaussian noise
(sigma in DS0), 400 data frames, six paired seeds per cell, gives these bit
error rates:

| | sigma 6 | 8 | 10 | 12 |
|---|---|---|---|---|
| no precoder, spec peer | 0 | 0 | 8.0e-4 | 6.2e-3 (2/6 no lock) |
| no precoder, negative points one step out | 0 | 7.2e-4 | 2.2e-3 | 4.2e-2 (2/6 no lock) |
| p1 = (0.0002, -0.0001), spec peer | 3.5e-2 | 3.8e-1 | no lock | no lock |
| p1 = (0.0002, -0.0001), feedback rounded down | 4.7e-1 | no lock | no lock | no lock |

Either departure costs about 2 DS0 of noise tolerance; the rounding also
starts errors at sigma 4 (1.2e-1) where the spec peer has none. The precoded
rows show something independent of ESS: with coefficients the receiver
decodes symbol by symbol (no Viterbi), and loses 4 DS0 against the same
peer. With p1 = (0.10, -0.05) and no channel ISI nothing locks at all,
because x(n) is then off the codec's levels (the reason the engine sends
coefficients only where the feedback pays, `modem_engine.c` near
`V92_PRECODER_MIN_GAIN`).

**Viterbi for precoded CPds (2026-11-24).** Every CPd we send has LZ1 = 0
and LP2 = 0, so the 6.4.2 feedback and the prefilter's feed-forward taps
depend only on past x(n), which the receiver has already observed: it
strips them before any decision (x = (v - sum z2(k) x(n-k)) / z2(0),
u = x - sum p1(k) x(n-k)) and runs the same 16-state trellis on u.
`v92_upstream_viterbi_supported()` names the CPds this covers; one with z1
or p2 taps still slices symbol by symbol. Same harness, same seeds:

| | sigma 6 | 8 | 10 | 12 |
|---|---|---|---|---|
| p1 = (0.0002, -0.0001), spec peer | 0 | 0 | 8.0e-4 | 6.0e-3 |
| p1 = (0.0002, -0.0001), feedback rounded down | 0 | 1.6e-4 | 3.0e-3 | 2.8e-2 |

The precoded spec-peer row now equals the unprecoded one, and the ESS
rounding costs the same 2 DS0 as the other quirk. **What this does not change:
live calls.** With the Phase 3 equaliser carried into data mode (every live
precoded call), the engine arms the receiver with the coefficients stripped
(`modem_engine.c`, `me_v92_arm_b1u_locked_locked()`): the channel's own
postcursor ISI, which the precoder was designed against, takes the feedback
back out before the equaliser's output, so that receiver already ran the
trellis. The harness above has no ISI, so it exercises the receiver only
on the raw path that skips the equaliser. Live slmodemd A/B the same day (3 calls a
side, alternated): identical precoded CPd, B1u locked by decoding, about
38000 lines each way on both builds. `v92_b1u_lock_test`
covers it with p1 = (0.10, -0.05) at two phases and two B1u lengths; the
old decoder got 686 bits wrong or never locked on those cases.

## The ESS modem itself, on D-Modem's offline rig (2026-11-24)

`half_v90modem` against D-Modem's `half_ess` (ESS's own driver,
`docs/dmodem_interop.md`) is the first time the receiver has met the real
peer rather than the quirks above. The B1u failure there began with two
separate problems.

**Ours: no Ed after a late acknowledgement (§9.6.1.1.4).** ESS's SUVu' came
more than 100 ms after our CPd, so §9.6.1.1.3 had us repeating CPd, and once
its CPu arrived every repeat was a CPd'. §9.6.1.1.4 sends Ed once we have
sent "a CPd or SUVd sequence with the acknowledgement bit set" and received
the peer's ack, but `v92_ack_sent` was only set by an SUVd', which the repeat
never sends. So the peer's SUVu' changed nothing: 275 CPd's went out until the
B1u deadline, and ESS sat in SUVu' waiting for Ed. Everything measured on
the upstream before this fix was ESS's SUVu', not B1u. A CPd' now counts
too (`v90_build_v92_cpd_native()`). `vpcm_loopback_test`'s "Ed after a late
acknowledgement" covers the timeout path: Ed now follows 60 symbols after
the SUVu', where before it stayed in CPd' past 1720 symbols. SmartLink acks
inside the window, so it never hit this.

**Then: ESS's precoder.** With Ed out, ESS sends E2u and B1u, but B1u still
never locks. The TRN2u equaliser is clean right up to that point (0.025-0.034
from the four levels with feedback, 0.12-0.17 feed-forward only, in ±1/±3
units). From B1u on, its output has no structure at all: rms 1.32, 0.075 from
the nearest of the CPd's levels where white is 0.090. Undoing the live p1,
either sign, does not help. Three CPd changes, all taken from D-Modem's own
ESS code (`dsp/v92.cpp`), did not lock it either:
- Moduli M with LC >= 2M and LC >= M(1 + sum |p1|) + 2 (`pf_rate()`), so the
  precoder has room inside the set (drn 2, 25333).
- Points on a uniform grid (2j + 1) x step, as D-Modem's precoder-only
  `design()` uses.
- Both together.

D-Modem's own modem fails the same way:
- **Precoder only** (`DSP_V92_PREFILTER=0`): its CPd at 41333, M 36,
  sum |b| 0.27, gave "B1u does not come through", with 1071 zeros in
  2976 bits at mse 0.0004. The signal lands on the grid but the bits are
  wrong. It then drops PCM upstream.
- **Prefilter** (64 taps, the default): 40000-41333, as its notes say.
- **No precoder** (`DSP_V92_PRECODE=0`): 29333, M 13, 52 points; B1u is
  clean after 2-3 frames, on three runs.

So against ESS, a precoded CPd without a real prefilter does not work for
either modem. An unprecoded one does, but ours cannot design one on this
line: our data mode keeps only the feed-forward taps (see
`me_v92_data_slicer_locked()` for why feedback on data decisions was taken
out against slmodemd), and those leave sigma 490 DS0 where the feedback
solve reads 98. The CPd design then gives up ("cannot arm B1u receiver").
The fix is one of two receiver features, neither of which we have:
- decision feedback in data mode for an unprecoded CPd;
- a prefilter design (z2 taps with the points on G.711 levels), D-Modem's
  route to 41333.

### Decision feedback in data mode, and answering ESS's Ru (2026-11-24)

The first of those two features is now built. Each part below was needed
before ESS's upstream carried data.

- **The fallback.** A precoded B1u that misses its deadline sets
  `g_v92_plain_upstream` for the rest of the call (`v92_b1u_deadline_expired_locked()`).
  From the retrain on, the CPd is unprecoded and data mode keeps the
  decision feedback on the data levels (`me_v92_plain_dfe()`;
  `ME_V92_PRECODER=3` forces it from the first attempt). The rate is priced
  on the feedback equaliser's sigma. The 2 s refits solve feed-forward and
  feedback taps together.
- **Refits wait for B1u's lock.** A decision-directed solve with feedback,
  run before lock, settles on its own decisions: 0.005 LU from the levels
  while nothing decodes.
- **The coefficient section must be present.** A CPd with Table 30's bit 20
  clear made ESS transmit exact silence after Ed. It reads the absent
  section as LZ2 = 0, so §6.4.2's prefilter output sums no taps. The CPd now
  carries D-Modem's identity section: LP1 = 4 zero taps, LZ2 = 4 with
  z2(0) = 32767, LZ1 = LP2 = 0 (`v90_set_v92_upstream_identity_filters()`).
  The B1u receiver treats that section as unprecoded.
- **§9.8.1.2: answering the analogue modem's renegotiation.** In μ-law the
  above passes the row at 29333-30666 up, with 71-76 DS0 from the levels
  over a 20 s stream. In A-law ESS sent Tone A about 9 s into every call.
  With D-Modem's own digital modem, ESS sends Ru and drops the downstream
  50666 -> 49333 (§9.8.2.1). We only looked for Ru after our own Rd, so its
  Ru went unanswered and ESS retrained. Capping our downstream at 44000 or
  48000 stopped the retrain (20 s streams both ways, 0 frames rejected), which
  confirmed it. Data mode now watches the equaliser output for Ru (48
  symbols of the period-6 sign pattern; `me_v92_ru_watch_symbol_locked()`).
  It clamps the decoder on Ru and answers R-bar-u with Rd (§9.8.1.2.2), then
  carries on as for our own renegotiation (§9.8.1.2.3 -> 9.8.1.1.2).
- **The renegotiation keeps the feedback.** `v92_p3_eq_restore_feedback()`
  zeroes the feedback taps, which suits the feed-forward-only data mode. In
  the feedback mode the feed-forward taps were solved together with the
  feedback, and zeroing it left TRN2u 1340 DS0 from its levels: rms 1.08 LU,
  no CPd buildable (drn < 1). `v92_p3_eq_reopen_feedback()` keeps the pair.
  The exchange then reaches TRN2u sigma 0.024 LU, CPu, CPd', SUVu', Ed and
  B1d.

### ESS A-law: the decision feedback ran away (2026-11-24)

In A-law, ESS's B1u used to fail in two ways:
- **On the levels, but nothing decoded.** This happened on about half of the
  startup attempts. The output sat 0.046-0.053 LU from the nearest level,
  against 0.020-0.025 on attempts that locked.
- **Far off the levels.** This happened on every attempt after a
  renegotiation. The output was 3.5-3.9 LU rms and 0.71 LU from the levels.

Neither was the CPd, ESS's mapping or a scale error. Identical CPds both
locked and failed, and no gain from 0.5 to 5 fits the failing values to the
constellation.

The dumps of the B1u search's input (the equaliser output from the CPd
acknowledgement on) show what happens. SUVu' sits on its four levels. Then,
within about 150 symbols, the output turns into a sign-alternating
oscillation that grows to ±5 LU. That is a decision-feedback runaway. On the
dense data levels a wrong decision goes back through the feedback taps. Where
the gain of sum fb_k z^-k is above 1, the error comes back larger.

The TRN2u joint solves came in two shapes:
- **One dominant tap**, fb ≈ (-0.74 or -0.82, small ...). These peaked at
  1.01-1.05 and locked, or missed narrowly at 1.085.
- **An alternating, slowly decaying tail**, fb ≈ (-0.20, +0.17, -0.18, +0.15,
  -0.14, +0.13, -0.10, +0.07). This peaked at 1.12-1.14 at fs/2 and never
  locked. Every renegotiation's solve had this shape, and so did some startup
  ones.

On four-level TRN2u, where every decision is right, both shapes fit equally
well (0.015-0.031 LU). Only the dense levels tell them apart.

**The fix.** When data mode keeps the feedback, the last joint solve is
solved again with ridge on its feedback columns, raised until the
feedback's peak gain is at most 0.9 (`V92_DATA_FB_GAIN_MAX`,
`v92_p3_eq_limit_feedback()`). Every later joint refit is held to the same
limit. Solves that peaked at 0.93-1.14 come out at 0.75-0.89, with
0.026-0.072 LU on TRN2u.

**Results.** Forced unprecoded at drn 6 (`ME_V92_PRECODER=3
ME_V92_UP_DRN=6`), 6 calls:
- 17 of 21 B1u attempts locked, after renegotiations as well. Before, about
  half of the startup attempts locked and none after a renegotiation.
- The 4 that missed were 0.028-0.054 LU from the levels.

On defaults, the precoded B1u still fails first, the retrain locks, and 2 of
3 calls passed the 20 s stream both ways at 48000-49333 down. The third
locked B1u, ran data, then failed on a B1u after a later renegotiation
(0.069 LU, limited peak gain 0.837) and fell back to V.34. On the same build
the SmartLink V.92 (both laws), SmartLink V.90, PCTel, SM56, Intel and ESS
μ-law rows pass.

**ESS's renegotiations are its own, not ours (2026-11-24).** Against
D-Modem's digital modem (`half_open8`) the same A-law row renegotiates 1-3
times a call too. The first Ru always comes at symbol 40091, about 5 s into
data, and each one steps the downstream down (50666 -> 49333 -> 48000).
D-Modem's upstream stays at about 40000 through all of them, and all 5 of
its calls passed. Against us, ESS steps the same way and sometimes back up
again (drn 17 -> 16 -> 17). Each call also has one renegotiation of our own,
asked for 6 s into data: the data-mode residual reads 112-150 DS0 against a
CPd designed on 71-110. That is the existing §9.8 redesign on the data-mode
figure, and those calls pass.

On a580829a's defaults 3 of 6 calls passed. The failures:
- A B1u after one of ESS's Rus misses (0.05-0.07 LU from the levels), the
  retrain falls back to V.34, and V.34 against ESS fails.
- Both startup B1u attempts miss, and PCM upstream is withdrawn.
- Three Rus come in quick succession at data start, LAPM never comes up, and
  ESS hangs up after 50 s.

So the gap is our upstream receiver: about 30000 where D-Modem gets about
40000, and too little margin for every B1u to lock.

Two things were tried, and neither helped:
- **Pricing the CPd on the gain-limited TRN2u solve.** The design sigma rose
  to 116 DS0, data mode then read 155-166, the rate did not change, and 1 of
  3 calls fell back to V.34.
- **A 3% B1u budget over the whole 16-frame window.** Every lock already
  reads 1.0000 ones, and the misses die with many errors, so no attempt was
  saved.

ESS's driver debug (`ESS_DEBUG=1`) shows only FIFO counters.

D-Modem does not run ESS's PCM upstream in A-law at all.

### The receiver gap was the A/D's sampling phase: Jp's Su extension (2026-11-24)

Our upstream against ESS was about 30000 where D-Modem's is about 40000
because **we always sent Jp with a zero Su extension** (Table 22 bits 18:33).
V.92 §8.6.4 says why that field exists: the digital modem cannot change the
central office A/D's sampling phase, so §9.5.1.1.6-8 has it measure the
phase on Su and ask the analogue modem, in Jp, to extend Su by a fraction of
a symbol. With zero, the A/D samples the analogue modem's pulse wherever its
clock happens to fall.

ESS's fell half a symbol off the peak. The channel fitted on TRN2u
(`ME_V92_PF_LOG=1`) had two equal taps where one belongs: -2761 and -3446
(the main tap 52% of the response's power). Interpolated half a sample later,
the same response is 97% one tap. Unequalised that is 0.4 dB of SNR. A 64-tap
prefilter would have reached only about 30 dB on it. The G.711 quantisation
of all that ISI (62 DS0 of the 63 DS0 fit error) is noise no receiver can
remove afterwards.

**The measurement.** Su's fundamental is 1333 1/3 Hz, and its phase on the
absolute six-slot clock of `v92_su` gives the channel's delay modulo one
symbol. Whole symbols are frame alignment, which TRN1u sets.
`v92_su_jp_phase()` sums the bin over the Su blocks before Su-bar.

**Su-bar's half symbol.** §9.5.2.1.7 sends Su-bar for 24.5T, so the analogue
modem's timing moves T/2 later there and stays moved. To check, fitted on
the line taps of two calls with Jp = 0 (TRN1u found by correlating against
§8.5.7's scrambled ones):

| | first Su | returned Su | TRN1u #1 | TRN1u #2 |
|---|---|---|---|---|
| SmartLink | 0.753 | 0.255 | 0.744 | 0.257 |
| ESS | 0.074 | 0.550 | 0.524 | 0.552 |

These are delay fractions in T. In both, the second TRN1u is the returned
Su's phase, which is the first Su's plus T/2. Measuring the returned Su
directly means holding Jd until enough of it has arrived (it starts 24.5T
after the Su-bar that ends Jd). One extra Jd repetition made SmartLink
Tone-A retrain in 4 of 4 calls, so we add the T/2 to the first Su instead.

**SmartLink applies the fraction backwards.** It advances where Table 22 says
extend. Asked for 0.247T, its second TRN1u came 0.247T earlier and its
response was nearly one tap (1362 with neighbours 23 and -225). ESS
extends, as the text says. Only 0 and T/2 mean the same both ways, so by
default the measured extension is **rounded to 0 or T/2**. That leaves at
most T/4 on either kind of modem, a main tap of about 86% or more. Sent
exactly, the fraction put SmartLink T/2 off: 1159 against 558, and two
Tone A retrains. D-Modem's digital modem always sends 0x8000 ("back to the
phase of the first TRN1u"), which is the same robustness by another route,
but without the measurement. `ME_V92_JP_PHASE=exact` sends the fraction
unrounded, `=off` sends zero (as before), and a number in [0, 1) sends that
number.

**What ESS needed next.** On the peak, ESS's channel is one tap
(-4885, every other tap under 40) and the precoder has nothing to do. Two
things broke that case:
- The design then sent **no coefficient section**. ESS's data fitted our
  levels (0.021 LU) but never decoded (26 of 33 B1u ones, 4 of 4 calls). With
  an explicit identity section, as the fallback path already sent, it locks.
- The **feed-forward-only** data path ran the integer-timing B1u training.
  On that flat channel TRN2u's timing loop had wandered 11.6 samples, and
  rounding it left the taps 0.36 sample off, so the reference was never
  found. An unprecoded CPd now always keeps the decision feedback, held to
  the 0.9 peak gain. slmodemd always precodes (its 9600 Hz high-pass), so
  that path is unchanged for it.

**Results.** STREAM=10, D-Modem's rig:

| Row | Before | Now |
|---|---|---|
| ESS A-law | 3 of 6 calls, ~30000 up | 3 of 3, B1u at 37333 first time, 48000-52000 down |
| ESS μ-law | passes, 29333-30666 up | 3 of 3, 37333 up |

ESS's own renegotiations still step the rate down during a call (to 29333 at
worst). μ-law missed its first B1u in 2 of 3 calls (27 of 33 ones, 0.032 LU),
and the retrain locked 37333 both times. On the same build SmartLink V.92
(both laws, 34666 up, as before), SmartLink V.90, PCTel, SM56 and
both Intel rows pass, 9 of 9, twice. `v92_su_test` checks the measurement
over 120 delay, alignment, polarity and law cases.
