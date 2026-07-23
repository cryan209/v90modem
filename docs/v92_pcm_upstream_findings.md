# V.92 PCM upstream against the d-modem/slmodemd rig — findings

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
