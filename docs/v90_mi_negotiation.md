# V.90 Mi (Modulus) Negotiation

## Summary

CLAUDE.md's "Key Implementation Status" section used to say:

> **Mi negotiation** — hardcoded to Mi=128 (7 magnitude bits per symbol); should be negotiated during training

This was stale (CLAUDE.md has since been corrected to point here). **The
downstream Mi negotiation path — the one that matters for normal V.90
operation — is already implemented, spec-accurate, and live-wired to the
real audio path.** `#define V90_MI 128` at `v90.c:24` is dead code
(referenced nowhere else) and does not describe how the mapper actually
gets its constellation size.

There is a real, still-open gap, but it's narrower than "Mi is hardcoded",
and it splits into a straightforward part and a genuinely open one:

1. The coupled-training **test harness** always offers the maximal
   constellation instead of one derived from its own DIL analysis — a
   contained, mechanical fix (see [Fix 1](#fix-1-test-harness-mask-mechanical)).
2. The live **V.92 upstream CPd** offer is also always maximal, and fixing
   it for real requires a signal this codebase doesn't clearly have wired
   up yet — the digital modem's own measurement of *its* upstream receive
   quality, which is conceptually different from the downstream DIL
   analysis (see [Fix 2](#fix-2-live-v92-cpd-open-question)).

## Glossary

Spec terms used throughout, all from ITU-T V.90 clause 5/8 unless noted:

| Term | Meaning |
|---|---|
| **Ucode** | 0–127, an index into the 128 positive-magnitude PCM code levels (independent of sign and of µ-law/A-law bit encoding — the codebase's law-agnostic "which level" identifier). |
| **Uchord _c_** | One of 8 groups of 16 consecutive Ucodes (`Uchord1` = Ucode 0–15, ..., `Uchord8` = Ucode 112–127) — the grouping Table 14's constellation mask and Table 12's DIL descriptor both use. `v90_dil_uchord_index(ucode) = ucode >> 4` (`v90.c:1720`). |
| **Ci** | The set of Mi actual PCM codes usable in data-frame interval _i_ (0–5) — a subset of the 128 Ucodes, sent as a 128-bit mask. |
| **Mi** | `\|Ci\|` — the modulus (constellation size) for data-frame interval _i_. Computed live as `vpcm_cp_mask_population(mask)` (`vpcm_cp.c:610`), never a fixed constant. |
| **D, K, S, Sr** | Per 6-symbol data frame: D = total input bits, K = bits into the modulus encoder, S = D−K sign bits, Sr = of those S, how many are spent on spectral-shaping redundancy instead of user data. See [Rate ↔ bits per frame](#rate--bits-per-frame-541-table-2). |
| **DFI** | "Data-Frame Interval" index array (`dfi[6]` in `vpcm_cp_frame_t`) — for each of the 6 symbol positions in a frame, which of the (up to 6) transmitted constellations applies. |
| **drn** | "Data Rate Number", 0–22 (V.90) or up to 28 (some V.91/V.92 contexts) — the single integer the CP/MP frames actually negotiate; rate and D derive from it. |
| **CP / CPt / CP′ / CPs** | Control Parameter sequences (Table 14, §8.5.2), sent **by the analogue modem to the digital modem**, carrying the Ci masks + drn + Sr + codec type + spectral-shaping filter coefficients for either Phase 4 training (CPt) or data mode (CP). CP′ = acknowledged; CPs = silence-requested (params unused). |
| **CPd** | V.92-only. Sent **by the digital modem to the analogue modem** — the reverse direction, for the V.92 PCM Upstream extension (§10.7/V.92, Table 30). This is the one direction where *this software* legitimately originates a constellation offer. |
| **DIL** | Digital Impairment Learning sequence (§8.3.1/8.4.1). The analogue modem sends a *descriptor* (via sequence Ja) telling the digital modem what pattern to transmit; the digital modem transmits it; the analogue modem observes what actually arrived (post-D/A/loop/A/D) to measure impairment. |
| **N, LSP, LTP, SP, TP, H_c, REF_c** | DIL descriptor fields — see [DIL descriptor format](#dil-descriptor-format-table-12) below. |

## Why the direction matters (ITU-T V.90 §5.4.1/5.4.3)

V.90 has asymmetric roles. This software is the **digital modem** (network
side, no A/D conversion). The real client is the **analogue modem** (has a
D/A→loop→A/D hop and can therefore measure the actual line impairment).
Per §5.4.3:

> Mi is equal to the number of positive levels in the constellation to be
> used in data frame interval i **as signalled by the analogue modem** using
> the CP sequences defined in 8.5.2.

So for the downstream data path, **the analogue modem measures the line and
tells the digital modem what Mi to use** — this software is supposed to be a
*consumer* of Mi, not the party deciding it. That consumption path is the one
that's fully built.

The Phase 4 startup sequence (§9.4, condensed to the parts relevant here)
makes the direction explicit — note which side sends CP/CPt and which side
just receives and acts on it:

```
Digital modem (this software)          Analogue modem (real peer)
------------------------------          ---------------------------
Ri (U_INFO, +++---) ------------------>
                     <------------------ CPt (Phase-4 training constellation)
barred Ri (U_INFO, ---+++), then TRN2d ->
MP (modulation params) ---------------->
                     <------------------ CP (data-mode constellation)
MP′ (ack) ------------------------------>
                     <------------------ CP′ (ack), then E (20-bit marker)
Ed -------------------------------------->
B1d (data, using CP's constellation) -->
                     <------------------ B1 (data)
```

Both `CPt` (Phase 4 training) and `CP` (ongoing data mode) carry the same
Table 14 structure; the digital modem's job in both cases is "receive,
validate CRC, extract Ci/Mi per interval, configure the mapper" — never
"decide Mi". See `ITU Docs/T-REC-V.90-199809-I!!PDF-E-1.pdf`, clause 5.4
(encoder/modulus algorithm), clause 8.5.2 + Table 14 (CP bit fields), and
clause 9.4 (this exchange sequencing) for the normative source.

## CP wire format (Table 14, abbreviated)

Reproduced here so the mapping from spec bits to `vpcm_cp_frame_t` fields
(`vpcm_cp.h:18-37`) doesn't require the PDF open side-by-side:

| CP bits | Field | `vpcm_cp_frame_t` member |
|---|---|---|
| 0:16 | Frame sync (17 ones) | (consumed by `v90_cp_rx_put_bit`, not stored) |
| 19 | 0 = CPt, 1 = CP | `v90_compatibility` |
| 20:24 | drn (0–22); rate = `(drn+20)×8000/6` in CP, `(drn+8)×8000/6` in CPt | `drn` |
| 30 | Silence requested (→ CPs; params unused) | (checked by `v90_cp_diag_is_strict`, `v90_cp_rx.c:24-29`) |
| 31:32 | Sr (spectral-shaping redundancy, 0–3) | `shaping_redundancy` |
| 33 | Acknowledge bit | `acknowledge` |
| 35 | Codec type: 0=µ-law, 1=A-law | `codec_alaw` |
| 36:48 | Analogue→digital rate capability mask | `upstream_rate_mask` |
| 49:50 | Spectral-shaping look-ahead frames | `shaping_lookahead` |
| 52:67 | TRN1d RMS ratio (transmitter vs. codec D/A output) | `trn1d_gain_q3_13` |
| 69:101 | Spectral-shaping filter coefficients a1,a2,b1,b2 | `shaping_a1/a2/b1/b2_q1_6` |
| 103:127 | DFI: 6 × 4-bit constellation index, one per data-frame interval | `dfi[6]` |
| 128 | Set if TX constellations differ from D/A-output constellations | `codec_constellations_differ` |
| 137:271, ... | Constellation mask 0 (128 bits, one per Ucode), then mask 1, up to mask 5 | `masks[6][16 bytes]` |
| (+γ per extra mask if bit 128 set) | Corresponding codec-output masks | `codec_masks[6][16 bytes]` |
| final 16 bits | CRC + fill | (validated by `vpcm_cp_decode_diag`, discarded) |

`v90_cp_rx.c:94` hard-codes the DFI bit offsets (`{103, 107, 111, 115, 120,
124}`) directly from this table to determine, before the frame's CRC-checked
length is even known, how many constellations are present (`max_idx + 1`) —
that's what makes the variable-length parsing in
`v90_cp_candidate_length()` (`v90_cp_rx.c:5-15`) possible.

## DIL descriptor format (Table 12)

The analogue modem tells the digital modem what to transmit for the DIL test
via a descriptor sent over sequence `Ja` (§8.3.1). `v90_dil_desc_t`
(`v90.h:25-34`) mirrors it field-for-field:

| Descriptor field | Spec meaning | Struct field |
|---|---|---|
| N (0–255) | Number of DIL-segments | `n` |
| LSP, LTP (1–128) | Lengths of the Sign Pattern / Training Pattern | `lsp`, `ltp` |
| SP (LSP bits) | Per-symbol sign: 0=negative, 1=positive | `sp[128]` |
| TP (LTP bits) | Per-symbol: 0=send reference REF_c, 1=send the segment's training Ucode | `tp[128]` |
| H1..H8 | Segment length for Uchord _c_: `Lc = (Hc+1)×6` symbols | `h[8]` |
| REF1..REF8 | Reference Ucode for Uchord _c_'s segment (the "safe" symbol sent when TP=0) | `ref[8]` |
| Training Ucodes ×N | The actual Ucode under test in each of the N segments | `train_u[255]` |

Digital-side TX construction reads this directly:
`v90_dil_uchord_index(training_ucode)` picks which of the 8 `h[]`/`ref[]`
entries governs the current segment, `s->dil.sp[pos % lsp]` and
`s->dil.tp[pos % ltp]` pick the sign and reference-vs-training choice per
symbol, all in the codeword-generation hot path (`v90.c:1781-1789`).

**Important:** the descriptor is authored by the analogue modem *before*
the DIL transmission happens — it says what pattern to test, not what was
measured. The actual impairment measurement (comparing what the analogue
modem's A/D received against what the digital modem was asked to send) is
the analogue modem's job, off in a black box this codebase never sees; the
result comes back later as a CP mask, not as an annotated DIL descriptor.
Keep this in mind for [Fix 2](#fix-2-live-v92-cpd-open-question) below — it
rules out one tempting shortcut.

## Deriving Mi from a measured DIL (`v90_dil_measure.c`)

Fix 1 below describes the harness offering a maximal constellation instead of
one derived from its own DIL analysis. The missing piece was never the
consumption path — it was that nothing turned *received* DIL into Ci.

`v90_dil_measure.c` does that, and the chain is short:

1. **Measure.** The descriptor is an input (§8.4.1 sends it in Ja; §9.3.2.9 has
   the analogue modem "receive the DIL sequence it requested"), so every
   expected codeword is known. Per transmitted Ucode, record what arrived —
   **per data-frame interval**, because §9.3.2.10 lets the peer stop early and
   because robbed-bit signalling hits one DS0 phase in six.
2. **Build Ci.** Walk the ladder upward in interval *i* and keep a Ucode only
   when its received level clears the previous kept one by the noise margin.
   A pad compresses neighbours onto one received code; RBS takes the LSB and
   merges them. Both drop out here.
3. **Mi = |Ci|**, per interval.
4. **Gate on §5.4.3.** The modulus encoder needs ∏Mi ≥ 2^K, so the largest
   supportable `drn` is the largest whose K does not exceed Σlog₂(Mi).
5. **Rate is then §5.4.1**: D = drn + 20 bits per 6-symbol frame, ×8000/6.

On a synthetic line with a 3 dB pad and RBS in one slot of six, from **half** a
DIL pass:

```text
coverage=50%, 31 Ucodes measured, gain=-3.0 dB, RBS slot mask=0x04,
Mi=[26 26 16 26 26 26] -> drn=13, 44000 bps
```

The robbed interval carries 16 points where the clean ones carry 26, which is
exactly why §5.4.3 makes Mi per-interval rather than one number per frame.

### A DIL descriptor can be self-consistent and still be useless

Segment lengths are multiples of 6 and TP restarts at every segment boundary,
so each data-frame interval is pinned to a fixed subset of TP bit positions. If
LTP shares a factor with 6 and none of that interval's TP bits are set, the
interval receives **only** REFc for the whole DIL and nothing is learned about
it. The first descriptor tried here did exactly that — LTP 12 left interval 1
landing only on TP bits 3 and 9, both zero — and the measurement correctly
reported Mi = 1 for it.

`v90_dil_rate_plan_t.intervals_unprobed` reports this, because the fix is a
different descriptor (LTP coprime with 6), not a lower rate. Anything that
authors a DIL descriptor here has to check it.

### Alignment must not be searched blindly

G.711 is self-similar across chords — Ucodes *u* and *u+16* differ by exactly a
factor of two — so a DIL stepping one Ucode per segment reproduces its own
shape 16 segments later, scaled. Normalised correlation cannot see a scale
factor, and that is the property wanted (a pad *is* a scale factor). A blind
search is therefore ambiguous by one chord and no level-based score fixes it.
§9.3.2.8-9 removes the need: the analogue modem sends S and then receives the
DIL it just requested, so `v90_dil_measure_align()` takes `from`/`span`.

### Still not applied

§8.5.2's cap on average constellation power, and any noise estimate beyond the
caller's `level_margin`. The masks are a starting point for a CP frame, not a
finished one.

## Rate ↔ bits per frame (§5.4.1, Table 2)

Two counts matter per 6-symbol data frame (`v90_state_t`'s `data_mapper_d`/`_k`/`_s`):

- **D** — total serial input bits per frame. Rate = `D × 8000/6` bps.
- **K** — of those D bits, the ones that go into the modulus encoder
  (§5.4.3). The rest, `S = D − K`, are sign bits; some of `S` may instead be
  spent as spectral-shaping redundancy `Sr` (0–3, trading data-carrying sign
  bits for DC-blocking/AC-coupling margin), leaving `S − Sr` sign bits
  actually carrying user data.

The code parameterizes this by `drn` (0–22): `D = drn + 20`
(`v90.c:653`), and `vpcm_cp_drn_to_k()` (`vpcm_cp.c:444`) gives
`K = drn + 14` for the `Sr = 0` case (all 6 sign bits carry data). Table 2's
full form allows K to run a few bits lower than this at the same D/rate,
if `Sr > 0` is negotiated — `v90_configure_data_mapper()` reads the real
`Sr` from the peer's CP frame (`cp->shaping_redundancy`, `v90.c:654`)
rather than assuming 0.

| drn | D = K+S | K (Sr=0, S=6) | rate (bps) |
|---:|---:|---:|---:|
| 1 | 21 | 15 | 28,000 |
| 4 | 24 | 18 | 32,000 |
| 7 | 27 | 21 | 36,000 |
| 10 | 30 | 24 | 40,000 |
| 13 | 33 | 27 | 44,000 |
| 16 | 36 | 30 | 48,000 |
| 19 | 39 | 33 | 52,000 |
| 20 | 40 | 34 | 53,333 |
| 21 | 41 | 35 | 54,667 |
| 22 | 42 | 36 | 56,000 |

(Steps evenly for every `drn` 1–22; only representative rows shown.)

This is why the `∏Mi ≥ 2^K` check (`v90.c:668`) is the actual gate on rate:
K is fixed by the negotiated `drn`, so a line that can't support enough
distinguishable levels across the 6-symbol frame to represent K bits — too
few usable Ucodes per interval — simply can't accept that `drn`, regardless
of how the mask is spread across the 6 intervals.

## Digital-side rate ceiling: the Jd capability mask

The digital modem consumes Mi (above), but it is **not** passive about rate.
Sequence **Jd** (Table 13) carries a 23-bit *downstream data-signalling-
rate capability mask* that the digital modem transmits, and the analogue modem
must not request (in CPt/CP) a `drn` whose rate exceeds what Jd advertises. So
Jd is the one downstream-rate lever *this* side controls directly — a ceiling on
the peer's `drn` choice, complementary to the peer's own line-measurement floor.

`v90_build_jd()` (`v90.c:1709`) builds this mask. Mask bit _k_ (k=0..22, at Jd
bit `18+k` for k≤15 and `19+k` for k≥16) advertises rate `28000 + k×8000/6` bps.
Uncapped it sets all 23 bits (`0x7FFFFF`, matching SmartLink's
`V90Jd::setRatesMask(0x7fffff)`).

### `ME_V90_MAX_DOWNSTREAM_RATE` — interop backoff knob

`v90_max_downstream_rate_bps()` (`v90.c:1680`) reads `ME_V90_MAX_DOWNSTREAM_RATE`
(bps). When set, `v90_build_jd()` clears every mask bit whose rate exceeds the
cap — keeping the 28000 floor bit (k=0) so the mask stays valid and contiguous —
via `v90_jd_rate_bit_enabled()` (`v90.c:1702`). Unset/0 → full mask (default, no
behavior change). A one-shot `[V90] Jd downstream-rate cap: …` line
(`v90.c:1768`, gated by `jd_rate_cap_logged`) records the effective cap in the
capture. Verified mask values (standalone round-trip of the exact bit-packing):

| cap (bps) | emitted mask | mask bits | top drn ≈ |
|---:|---:|---:|---:|
| 0 / unset | 0x7FFFFF | 23 | (peer-limited) |
| 46666 | 0x007FFF | 15 | 15 |
| 40000 | 0x0003FF | 10 | 10 |
| 33333 | 0x00001F | 5 | 5 |
| 28000 | 0x000001 | 1 | 1 |

(Jd mask bit _k_ advertises the same rate that data-mode `drn = k+1` uses, so
e.g. cap 46666 → top mask bit k=14 → the peer may go no higher than drn 15.)

### Why cap: the Phase-4 PDSNR gate

When downstream training completes CPt → TRN2d but the analogue peer **rejects**
TRN2d at its Phase-4 PDSNR gate (residual error plateaus above its threshold and
it retrains), one cause is a fixed per-symbol TX displacement whose magnitude
only exceeds the decision boundary at dense constellations. Capping `drn` forces
a sparser constellation with larger minimum distance, so the same residual falls
inside the decision regions and the gate clears — proving the end-to-end path
reaches DATA. This is both a workaround and a diagnostic: **if a low cap reaches
DATA, the residual is a per-symbol displacement (sign / spectral-shaping-sign
class), not a rate/mask decode error** — the sign-independent B1d acceptance
oracle (`vpcm_decode.c:16714`) cannot see that class, so a successful backoff is
the cleanest evidence for where to look next.

The peer applies its own floor independently: in the SmartLink hardware runs it
chose `drn=15` (~46.7k) even with the full mask, so the cap only changes the
outcome when set strictly **below** the peer's own pick (below ~46.7k here).

### Backoff / re-tighten ladder

Start aggressively robust, confirm DATA, then re-tighten toward break-even:

```
ME_V90_MAX_DOWNSTREAM_RATE=33333   # drn 5,  D=25 — max headroom, prove the path
                          40000    # drn 10, D=30
                          44000    # drn 13, D=33
                          46666    # drn 15 — SmartLink's uncapped pick
```

Success in the trace: the `[V90] Jd downstream-rate cap` line appears; the
far-end CPt returns `drn < 15`; the TRN2d residual plateaus below the peer's
gate with no retrain; the session reaches DATA. The break-even cap (the highest
that still reaches DATA) quantifies how much of the sign/shaper residual must be
removed to recover full rate.

## What's already real: downstream Mi consumption

End-to-end pipeline, live audio to modulus encoder:

**1. QAM bit recovery** — in the live SIP call path (not just tests):

```c
/* modem_engine.c:2239 */
v34_set_put_phase4_bit(g_v34, v90_live_cp_bit, NULL);
```

wires SpanDSP's V.34 receiver's per-bit demodulator output straight into
the CP frame synchronizer.

**2. Frame sync + variable-length CP parsing** — `v90_cp_rx_put_bit()`
(`v90_cp_rx.c:57-129`) finds the 17-bit sync + start bit, then at bit 129
reads the 6 DFI fields to compute `max_idx` (highest constellation index
referenced) and derives the frame's true bit length from it:

```c
/* v90_cp_rx.c:93-113, condensed */
if (rx->bit_count == 129) {
    for (i = 0; i < 6; i++)
        /* read 4-bit DFI field at dfi_pos[i] = {103,107,111,115,120,124} */
        ...;
    rx->target_bits = v90_cp_candidate_length(max_idx + 1,
                                              rx->constellation_points,
                                              rx->bits[128] != 0);
}
```

Once `target_bits` is reached, the full frame goes to
`vpcm_cp_decode_diag()` for bit-field decode + CRC check, and
`v90_cp_diag_is_strict()` (`v90_cp_rx.c:17-30`) applies acceptance filters
(drn in range, not a silence request, codec type matches expectation).

**3. Delivery into V.90 state** — `v90_live_cp_frame()`
(`modem_engine.c:1696-1715`) and `v92_live_p4u_frame()`
(`modem_engine.c:1729-…`) call `v90_set_phase4_cp()` (`v90.c:2753`) or
`v90_set_v92_cpu()` (`v90.c:3241`).

**4. Mapper configuration** — `v90_configure_data_mapper()` (`v90.c:639-676`):

```c
/* v90.c:653-669, condensed */
s->data_mapper_d = cp->drn + 20;
s->data_mapper_sr = cp->shaping_redundancy;
s->data_mapper_s = V90_FRAME_LEN - s->data_mapper_sr;
s->data_mapper_k = s->data_mapper_d - s->data_mapper_s;
for (int i = 0; i < V90_FRAME_LEN; i++) {
    int constellation = cp->dfi[i];
    int m = vpcm_cp_mask_population(cp->masks[constellation]);   /* Mi, live */
    product *= (uint64_t)m;
}
if (product < (1ULL << s->data_mapper_k))
    return false;   /* peer's masks can't carry the negotiated K */
```

This is a direct, live implementation of Mi per interval and the spec's own
`2^K ≤ ∏Mi` validity inequality — nothing here is a fixed 128.
`v90_configure_phase4_mapper()` (`v90.c:573`) is the Phase-4-training
(CPt-driven) sibling of this same logic.

**5. Modulus encoder + mapper** — `v90_map_scrambled_frame()`
(`v90.c:678-712`) is a direct transcription of the spec's §5.4.3 modulus
encoder and §5.4.4 mapper:

```c
/* v90.c:695-706, condensed — one pass builds all 6 codewords */
for (int i = 0; i < V90_FRAME_LEN; i++) {
    int constellation = cp->dfi[i];
    int m = vpcm_cp_mask_population(cp->masks[constellation]);   /* Mi */
    int label = (int)(r % (uint64_t)m);                          /* Ki = Ri mod Mi */
    r /= (uint64_t)m;                                             /* Ri+1 = (Ri-Ki)/Mi */
    ucode = v90_cp_constellation_ucode(cp, i, label);             /* §5.4.4 mapper */
    sign = (scrambled[i] & 1) ^ sign;                             /* §5.4.5.1 differential */
    frame[i] = v90_pcm_signed_codeword(s->law, ucode, sign);
}
```

`v90_cp_constellation_ucode()` (`v90.c:490-508`) implements the §5.4.4
labelling rule exactly — walking the mask from Ucode 127 down to 0 so that
label 0 is always the *largest* PCM code in the constellation, per spec.

If a real analogue modem sends a CP frame with a restricted mask (say, 96
Ucodes on the robbed-bit-affected interval and 128 on the other five), this
pipeline **already** derives per-interval Mi from it and encodes data frames
accordingly.

## DIL analysis today is test-harness-only, not live

An important nuance for anyone extending this: `v90_analyse_dil_descriptor()`
(`v90.c:1956-2036`) — the function that scores a DIL descriptor's shape
into `impairment_score`, `robbed_bit_limited`, `recommended_downstream_drn`,
etc. — **is never called from `modem_engine.c`**. It's used exclusively by
the coupled-training test harness (`vpcm_v90_session.c`, via
`vpcm_v92_select_profile_from_dil()`, `vpcm_v90_session.h:139`), which
constructs a synthetic `v90_dil_desc_t` to represent a test scenario and
runs it through this analysis to decide what `drn` the simulated CP offer
should carry.

In the live path, the received DIL descriptor (`v90_parse_dil_descriptor()`,
fed from real captured Ja audio at `modem_engine.c:1844`) is used only to
drive DIL *transmission* (§8.4.1 codeword generation) — its shape is never
scored for impairment in production today. This matches the spec's actual
division of labor (the descriptor says what to test, not what was found;
the analogue modem is the one that observes the result), but it does mean
[Fix 2](#fix-2-live-v92-cpd-open-question) below can't simply "call
`v90_analyse_dil_descriptor` on the live descriptor" — that function was
never meant to observe real impairment, only to interpret a test-authored
one.

## The actual gap

There are exactly two places in the tree where this software **builds** a
constellation offer instead of consuming one, and both skip impairment
analysis entirely in favor of the maximal mask.

### Fix 1: test-harness mask (mechanical)

`vpcm_cp_enable_all_ucodes(cp_frame.masks[0])` at `vpcm_v90_session.c:643`,
`:1443`, and `:1491`. The harness (`vpcm_v90_run_coupled_training()`,
`vpcm_v90_session.c:593`) simulates *both* sides of a call for loopback
testing, so it's standing in for the analogue modem's role. It already
calls `vpcm_v92_select_profile_from_dil()` to pick the DIL-driven `drn`,
but always hands that a full 128-Ucode mask — meaning the test suite
currently never exercises the mapper against a genuinely restricted mask
end-to-end, only against drn (rate) changes.

This one is straightforward: `v90_analyse_dil_descriptor()`'s output is
already available at these call sites (that's where `dil_analysis` comes
from), so extending it to also emit a mask and using that mask instead of
`vpcm_cp_enable_all_ucodes()` is a self-contained change with no live-signal
question attached — see [Proposed design](#proposed-design) below.

### Fix 2: live V.92 CPd (open question)

`v90_build_v92_cpd_frame()` (`v90.c:368-395`) builds the V.92 CPd frame —
*not* the same direction as the downstream CP above. V.92 adds a **PCM
Upstream** mode where the digital modem's receiver needs the analogue
modem to transmit with a constellation *this side* can reliably decode —
so for that one direction, the digital modem legitimately is the party
proposing Mi. Currently it unconditionally builds all odd Ucodes 1–127:

```c
/* v90.c:382-393 */
for (int ucode = 1; ucode < 128 && points < V92_CPD_MAX_POINTS; ucode += 2) {
    int16_t linear = v90_pcm_to_linear(s->law, ucode_to_pcm_positive(s->law, ucode));
    if (linear <= 0) continue;
    out->points[0][points++] = (uint16_t)linear;
}
for (int i = 0; i < 12; i++)
    out->moduli[i] = (uint8_t)(2 * points > 255 ? 255 : 2 * points);
```

regardless of any measured receive impairment. **This is the one hardcode
that actually reaches a real analogue modem over the wire in production.**

Unlike Fix 1, this doesn't have an obvious existing signal to hook. The
downstream DIL/CP mechanism described above measures the *downstream* D/A
path and reports back to the digital modem — it says nothing about how well
*this software's own receiver* decodes the analogue modem's upstream
transmission, which is what a real CPd offer should be based on. V.92
(`ITU Docs/T-REC-V.92-200011-I!!PDF-E.txt`) defines an upstream training
signal in the same Phase 3 exchange — `TRN1u`, sent by the analogue modem
— which is presumably what the digital modem's receiver is meant to
characterize before proposing a CPd constellation (line ~9752–9850 of that
text dump, Figure 10/V.92). This codebase already has `v92_phase3_decode.c`
/ `v92_p3_rx.c` handling `TRN1u`-related decode, but whether either produces
anything resembling a receive-quality metric usable for mask construction
is **not yet verified** — that's the concrete next research step before a
real fix here can be designed with confidence, not something to guess at
by analogy with the downstream path.

## Proposed design

**Fix 1 (test harness), concrete:**

1. Extend `v90_analyse_dil_descriptor()` — or add a companion
   `v90_dil_recommended_mask()` — to walk `desc->train_u[]` (the actual
   per-segment training Ucodes the test scenario declares as "seen") and
   call `vpcm_cp_mask_set()` per confirmed Ucode, instead of only bucketing
   into the 8 coarse `used_uchords` counts it computes today. Fall back to
   `vpcm_cp_enable_odd_ucodes()` (`vpcm_cp.c:473-482`, the same helper
   already used for robbed-bit-safe slots) for any interval the analysis
   flags `robbed_bit_limited`.
2. Replace the three `vpcm_cp_enable_all_ucodes()` call sites in
   `vpcm_v90_session.c` with the new mask, keyed per-interval the same way
   `dfi[]`/`drn` already are.
3. Sizing/format: no wire-format change — `masks[6][16 bytes]` already
   exists in `vpcm_cp_frame_t`; this only changes which bits get set.

**Fix 2 (live CPd)** should not be attempted until the research step above
(what does this codebase's `TRN1u` receive path actually measure, if
anything) is done — see [Fix 2](#fix-2-live-v92-cpd-open-question). Once a
real receive-quality signal is identified, the integration shape should
mirror Fix 1: build a mask from that signal, feed it into
`v90_build_v92_cpd_frame()` in place of the fixed odd/1..127 loop, following
the same cap-down pattern V.91's `v91_select_drn()` already uses for rate
(`v91.c:1490-1514`) — but producing a mask instead of a single integer.

## Validation

- `vpcm_loopback_test.c` already exercises `v90_set_phase4_cp` /
  `v90_configure_data_mapper` against hand-built `vpcm_cp_frame_t` values,
  including deliberately mismatched/invalid ones (search `data_cp`,
  `mismatched_cp` in that file) — a restricted-mask test case (e.g. Mi=64
  on one interval, Mi=128 on the rest, confirm `product < 2^K` rejection
  and successful encode when it isn't) belongs alongside these, and can
  land independently of either fix above since it only exercises the
  already-real consumption path.
- After Fix 1, `vpcm_v90_run_coupled_training()`'s existing scenarios
  (`make test`) should be extended with at least one case where the
  simulated DIL analysis produces a genuinely restricted mask, to confirm
  the full loopback (DIL → analysis → mask → CP → mapper → encoded data)
  actually round-trips — today's coverage stops at drn changes.
- `make v34-tone-matrix` and the broader `make test` suite are unrelated to
  this change but must stay green per existing project convention (see
  `docs/p3_demod_rrc_frontend_plan.md`'s validation sections for the
  pattern this project uses).
- Fix 2 has no meaningful test until the open question above is resolved —
  a synthetic "measured upstream quality" input can be faked for unit
  testing the mask-construction logic itself, but validating that it's the
  *right* signal requires either the real V.92 spec mechanism or a real
  captured V.92 PCM-Upstream call.

## Non-goals

- Do not change the downstream RX consumption path (`v90_configure_phase4_mapper`,
  `v90_configure_data_mapper`, `v90_map_scrambled_frame`,
  `v90_cp_constellation_ucode`) — it is correct and matches §5.4.3/5.4.4
  exactly today.
- Do not touch the legacy `v90_encode_octet_to_codeword()`/
  `v90_decode_codeword_to_octet()` path (`v90.c:435-488`). It's
  unconditionally 7-bit/128-point by design — it's the non-negotiated
  compatibility encoder, documented as such in `v90.h:390-414`, distinct
  from the live negotiated `v90_tx_data_frame_codewords()` path.
- `#define V90_MI 128` (`v90.c:24`) can be deleted in the same change as a
  cleanup — it has no readers.
- Do not attempt Fix 2 by reusing the downstream DIL analysis machinery
  by analogy — the [DIL descriptor format](#dil-descriptor-format-table-12)
  section above explains why that signal doesn't describe upstream receive
  quality at all.

## References

- `ITU Docs/T-REC-V.90-199809-I!!PDF-E-1.pdf`: clause 5.4 (encoder, modulus
  algorithm, Table 2 rate/K table), clause 8.3.1/8.4.1 + Table 12 (DIL
  descriptor), clause 8.5.2 + Table 14 (CP bit fields), clause 9.4 (Phase 4
  CP/CPt exchange sequencing).
- `ITU Docs/T-REC-V.92-200011-I!!PDF-E.txt` (pre-extracted text): Table 30
  (CPd bit fields, ~line 8590), Figure 10/V.92 Phase 3 exchange incl. TRN1u
  (~line 9752).
- `v91-rate-adaptation` memory / `v91_select_drn()` (`v91.c:1490-1514`): the
  existing DIL-driven-cap precedent Fix 1 follows, and the template Fix 2
  should follow once its input signal is known.
