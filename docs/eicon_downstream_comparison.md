# Comparing our downstream against a working V.90 digital modem

## Why this comparison is possible

`../modem-dsp-emu` runs the Eicon Diva Server PRI card's own shipped V.90
firmware under emulation. Its captures include a USR Courier that **connects**:

```text
artifacts/interop/courier-2185n-dil/selector-call1.modem.log
  CONNECT 32000/ARQ/V90/LAPM
artifacts/interop/courier-2185n-dil/selector-call3.modem.log
  CONNECT 42666/ARQ/V90/LAPM
```

That gives us the one thing archaeology on our own captures cannot: the same
peer model, one digital side it accepts and one (ours) it does not. The card's
transmit stream is a true DS0 µ-law capture, so codewords are recoverable
exactly, with no analogue-recording caveat.

The two projects share no code. Nothing here is copied from the card; the
firmware is Eicon's. It is used as an **oracle** — a second implementation whose
observable behaviour disambiguates the Recommendation — not as a source.

The fixtures and their provenance are in
[`../artifacts/eicon-digital-downstream/`](../artifacts/eicon-digital-downstream/README.md).

## The headline

**The card transmits 30000T of TRN1d — 3750 ms. We transmit 2496T — 312 ms.**
Twelve times less. Everything below is either the evidence for that or a
hypothesis it replaced.

## Finding 1 — TRN1d is 3750 ms, and it is spec-exact GPC

§8.4.5 defines TRN1d as the PCM codeword whose Ucode is U_INFO, with signs from
applying binary ones to the §5.3 scrambler, **initialized to zero**. §5.3 makes
that scrambler V.34's GPC.

Generating that sequence and sliding it over the whole capture locks
**bit-for-bit**, on both calls independently:

| | call 1 | call 3 |
|---|---|---|
| GPC lock | **100.0%** | **100.0%** |
| starts | 8420.9 ms | 8460.4 ms |
| runs | **30005T (3750.6 ms)** | **30005T (3750.6 ms)** |
| magnitudes over the run | `{48}` | `{48}` |
| U_INFO | 48 | 48 |

100% over a 512-symbol window and then a 30005-symbol unbroken run is not a
statistical artefact. (30000T is 5000 six-symbol frames exactly; the trailing 5
is where the *next* signal happens to keep matching before diverging.)

Against §9.3.1.4 — TRN1d minimum 2040T, and Jd must start within 4000 ms of
TRN1d starting — the card uses **3750 of the 4000 ms budget**, leaving 250 ms.
We use 312 ms and leave 3688 ms unused:

| | TRN1d | share of the §9.3.1.5 budget |
|---|---|---|
| §9.3.1.4 minimum | 2040T (255 ms) | 6% |
| **ours** (`v90_trn1d_len()`, `v90.c:99`) | 2496T (312 ms) | 8% |
| **the card** | **30000T (3750 ms)** | **94%** |

The comment at `v90.c:85` reasons toward exactly this and stops short:

> §9.3.1.4 makes TRN1d a *minimum* of 2040T … so sending the bare minimum
> leaves ~3.7 s of headroom unused. §9.3.2.5 has the analogue modem condition
> its equaliser on the *first 2040T*, so transmitting exactly 2040T gives a peer
> that arms even slightly late a short training sequence and no way to recover.

That reasoning was right, and the correction applied (2040 → 2496) was about 1%
of the available headroom. The card spends 94% of it.

This fits the observed failure without further assumption. Per the note at
`v90.c:91`, the Courier detects our Sd→S̄d and goes silent per §9.3.2.4
(correctly), then **never decodes Jd**, never sends S, and retrains at its
§9.3.2.7 deadline — which is what a peer whose equaliser never converged would
do.

### The knob exists but cannot reach the card's value

`ME_V90_TRN1D_SYMBOLS` already exists. Its guard is `parsed <= 16000`
(`v90.c:121`), i.e. 2000 ms, on the stated grounds that "much above ~2 s starts
eating the Jd window §9.3.2.7 entitles the peer to use." The working
implementation uses 3750 ms, so that ceiling is contradicted by the oracle and
is what blocks the experiment.

**Next step:** raise the clamp to admit the card's value and dial the Courier at
`ME_V90_TRN1D_SYMBOLS=30000`. This is a live test, not an offline one — it needs
hardware, and it is the single highest-value call to make.

## The full downstream timeline, and what else is comparable

Segmenting both sides by (Ucode-0 density, distinct magnitudes, sign
periodicity) gives the card's whole Phase 3, and ours beside it:

| stage | card (connects) | ours (fails) | comparable? |
|---|---|---|---|
| pre-Phase 3 silence | 3000 ms | 2900 ms | yes — match |
| Sd | **384T** | 384T | yes — **match** |
| S̄d | **48T** | 48T | yes — **match** |
| TRN1d (const-mag, GPC signs) | **30000T (3750 ms)** | 2496T (312 ms) | yes — **12× short** |
| Jd (const-mag, scrambled+diff signs) | 943T (118 ms) | peer-gated | **no** |
| DIL (66T segments, descending level) | 1972 ms | 100 ms | **no** |
| Ri (const-magnitude, `+++−−−`) | 571 ms | 1000 ms | **no** |
| multi-level (data) | 12200 ms | 11400 ms | **no** |

**Only the first four rows mean anything.** §9.3.1.3 has the digital modem send
Sd, S̄d and TRN1d unconditionally after Ja, so those are transmitted into the
void and are directly comparable. **Jd then continues until the analogue
modem's S arrives** — and on our call it never does: our own decode shows
`Jd zone … 9744.4 ms`, us sitting in Jd waiting. Every stage after TRN1d is
therefore gated on a peer response we do not get, and its duration is a
*consequence* of the failure, not a candidate cause.

Reading those last rows as defects would be the trap that cost `modem-dsp-emu`
three leads in one session (handoff §2: "do not compare two ends without
checking both are in the same phase"). They are recorded here so nobody
re-measures them and gets excited.

So: **TRN1d is the only defensible transmit-side fix these captures support.**

### Sd and S̄d are confirmed correct

Measured on the card, both calls: Sd is `{+W, +0, +W, −W, −0, −W}` with
W = Ucode **64**, running **384T**, followed by **48T** of S̄d. §8.4.4 defines
W = Ucode(16 + U_INFO), and the card's TRN1d runs at U_INFO = 48 — so
16 + 48 = 64 checks out, and the card is internally consistent and spec-exact.

`V90_SD_REPS 64` and `V90_SD_BAR_REPS 8` are right. Do not touch them.

### The remaining value in these captures is on the receive side

Decoding the card's Jd, J'd, DIL and Phase 4 is *not* confounded — a
reference decode is a reference decode regardless of what our transmitter did.

That is where the work since has gone. The sparse descending-level stage is
identified: it is **DIL**, 66T segments stepping the training Ucode down the
ladder. Getting there required fixing the Sd detection that was mis-anchoring
everything after it (Finding 3) and then rewriting DIL acquisition, which could
not read a DIL sent once (Finding 4). DIL is still the first stage our receive
path cannot read, for a narrower reason now (Finding 5).

## Jd — decoded on both sides, and it is almost entirely correct

`tools/v90_jd_decode.py` recovers the Jd frame from a DS0 capture. Getting there
needs three things §8.4.2 states but never puts together: TRN1d has to be
located first (the differential encoder is seeded with "the final symbol of the
transmitted TRN1d"), the scrambler is *not* reinitialized at the boundary (so
the descrambler's 23-bit history is TRN1d's own tail, and TRN1d's signs are its
scrambled bits directly, being undifferentiated), and the CRC is delegated to
10.1.2.3.2/V.34 without restatement.

The decode validates: **72-bit period at 98–99%**, all three of Table 13's fixed
fields exact, CRC valid, and the **identical frame on both independent card
calls**.

| Table 13 field | card (connects) | ours |
|---|---|---|
| 0:16 frame sync | `11111111111111111` | same |
| 17 start bit | `0` | same |
| 18:33 rate mask | `1111111111111111` | **same** |
| 35:46 rate mask (contd) | `111111000000` | **same** |
| 47 training constellation | `0` (4-point) | same |
| 48 renegotiation constellation | `0` (4-point) | same |
| **49:50 spectral-shaping lookahead** | `11` → **3** | `10` → **1** |
| 52:67 CRC | `0x66cf` **valid** | `0x76ee` **valid** |
| 68:71 fill | `0000` | same |

**Bits 0–49 are identical.** Our framing, scrambling, differential encoding and
CRC convention all match a working implementation exactly. Of the four differing
bits — 50, 55, 62, 67 — three are inside the CRC field and are consequences of
the fourth.

So Jd is not broken, and it is not the Courier blocker: a peer that rejected our
Jd would be rejecting a frame whose sync, framing and CRC are all correct.

### The one real difference: we advertise the minimum shaping lookahead

`v90_build_jd()` hardcodes it, and says so:

```c
/* Bits 49:50 — spectral shaping lookahead: 1 (minimum mandatory) */
```

The card advertises 3. Both are legal — §8.4.2's field is "a number between 1
and 3".

**Do not simply change this to 3.** It is a capability claim about our own
encoder, and `vpcm_cp.c`'s guards (`shaping_lookahead != 1` at `v90.c:1115`,
`> 3` at `v90.c:1220`) suggest the mapper implements lookahead 1 only.
Advertising 3 while shaping to 1 would be a lie the peer acts on, and the place
it would surface is Phase 4. The honest sequence is: implement lookahead > 1 in
the shaper, then raise the advertisement to match. Recorded as a genuine
capability gap, not a quick fix.

### CRC formulation, confirmed empirically

§8.4.2 delegates the CRC to 10.1.2.3.2/V.34. Reproduced against a working
implementation, so this can be relied on:

> CRC-16-CCITT, polynomial `0x1021`, init `0xFFFF`, over the two 16-bit
> information groups (bits 18:33 and 35:50), transmitted **MSB-first** in bits
> 52:67.

Our `v90_build_jd()` already matches this. Note the field is MSB-first: reading
it LSB-first makes a valid CRC look broken, which cost a step here.

### Jd duration is peer-gated — do not compare it

The card sends 943T and 1015T (13–14 frames) on its two calls; we sent 1231T on
this one and 9744 ms on another. That spread is not a defect: §9.3.1.5 runs Jd
until the analogue modem's S arrives. The card's peer answers; ours does not.

## Finding 2 — our decoder mislabelled Ri as "Sd" — **fixed**

`vpcm_decode --v90` used to report, on the card's stream:

```text
[14261.1 ms] V.90 Sd (sign-pattern): 758 reps (4548 symbols, 568.5 ms)
```

**This is not Sd.** §8.4.4 makes Sd `{+W, +0, +W, −W, −0, −W}` — one third of
its symbols are Ucode 0. Measured over that exact span: 4547 of 4548 symbols are
Ucode 48 and **one** is Ucode 0. A constant-magnitude signal cannot be Sd.

Nor is it TRN1d, as an earlier revision of this section claimed. TRN1d is the
GPC-locked run at 8420.9–12171.5 ms (Finding 1). The 14261.1 ms region sits
*after* DIL and is **Ri** — §8.6.4 makes Ri the U_INFO codeword at a `+++−−−`
sign pattern, which is a constant magnitude and a period-6 sign pattern, i.e.
exactly what was measured. Sd and Ri share that sign pattern; only the Ucode-0
content tells them apart, which is what the detector was not checking.

The `--dil-scan` output labelled the same span a "codeword-exact DIL run" with
"training Ucodes: 1 unique (first: 48)", so two different reports disagreed
about one region and neither was right.

Fixed — see Finding 3 for the three separate causes. The card's Sd now decodes
at 8366.9 ms, and the whole Phase 3 chain behind it. `make eicon-rx-test` now
pins the DIL gap instead (Finding 4).

## Finding 3 — three receive-path defects hid the card's Sd — **fixed**

The card's Sd was there all along, at 8366.9 ms on call 1 and 8406.4 ms on
call 3, spec-exact at 64 reps. Three independent faults kept us from seeing it,
and all three had to be fixed before any of them showed:

**(a) The exact-codeword matcher demanded `−0`.** §8.4.4 writes Sd as
`{+W, +0, +W, −W, −0, −W}`, and `sd_pat[]` compared all six slots as exact
codewords. But G.711 Ucode 0 has two codewords for one level at the far D/A,
and the card transmits `+0` (µ-law `0xFF`) in **every** zero slot of both Sd and
S̄d — on both calls:

```text
T66935   +64 [bf] +0 [ff] +64 [bf] -64 [3f] +0 [ff] -64 [3f]
```

Our transmitter follows the Recommendation's `−0` literally, which is defensible
and is left alone; a *receiver* that requires it cannot read a working peer.
`v90_sd_slot_match()` now matches the level on the zero slots and the full
codeword on the W slots, which is where §8.4.4 carries information.

**(b) The U_INFO search started at 67.** Table 12 says "U<sub>INFO</sub> shall be
greater than 66", and the scan enforced it. That clause binds the *analogue*
modem populating INFO1a; it is not a fact about the line. Both these calls —
which the Courier answered `CONNECT 32000` and `CONNECT 42666` — run at
**U_INFO = 48**, confirmed by W = 16 + U_INFO = 64 holding exactly across Sd.
The floor is now 10; the peer's choice is not ours to police.

**(c) The sign-pattern fallback matched `+++−−−`, which real Sd is not.**
With both zero slots positive, the card's Sd signs are `+ + + − + −`. So the
fallback could never fire on genuine Sd — and, having no Ucode-0 requirement, it
fired on Ri instead (Finding 2). It now rejects a candidate with no Ucode 0 in
it, citing §8.4.4's one-third.

A fourth, smaller fault surfaced once the chain ran: TRN1d and Jd are both the
U_INFO codeword, and the matcher counted the magnitude run as all TRN1d,
reporting 30948 symbols. It now descrambles §8.4.5's scrambled ones and splits
at the lock, giving **30005T TRN1d + 943T Jd** — reproducing Finding 1 through
our own receive path for the first time.

Result, both fixtures, matching an independent segmentation of the captures:

```text
[ 8366.9 ms] V.90 Sd: W_UCODE=64 (U_INFO=48), 64 reps (384 symbols, 48.0 ms)
[ 8414.9 ms] V.90 S̄d: 8 reps (48 symbols, 6.0 ms)
[ 8420.9 ms] V.90 TRN1d: 30005 symbols (3750.6 ms) at U_INFO=48, descrambled to ones
[12171.5 ms] V.90 Jd+J'd: 943 symbols (117.9 ms), ~13 Jd frame repetitions
```

## Finding 4 — DIL acquisition needed the cycle twice — **fixed**

With Phase 3 anchored correctly, the next stage is DIL, at 12289.4 ms on call 1
(12337.9 ms on call 3), running ~1971 ms to where Ri starts.

`v90_dil_rx.c` could not decode it, and the reason was **acquisition**, not the
descriptor. Generating a DIL needs no acquisition: the generator is handed the
descriptor. The decoder had to find one, and did it by autocorrelation —
locate an exactly-periodic run, then fit a descriptor to a single cycle.
`dil_best_run_at()` extended a candidate only while `x[i] == x[i+c]`, and
`v90_dil_rx_scan()` kept it only if the run spanned `2 * c`. **The cycle had to
appear twice.** §8.4.1 lets the analogue modem terminate DIL on any segment
boundary, and the Courier does so after roughly one pass, so it never did.

Measured with our *own* generator, so nothing depended on reading the card —
a card-shaped descriptor, varying only how much was transmitted:

| cycles sent | before | after |
|---|---|---|
| 0.75 | OK, cycle 12 (**wrong**) | partial pass decoded |
| 1.00 | OK, cycle 12 (**wrong**) | **exact** |
| 1.75 | OK, cycle 12 (**wrong**) | **exact** |
| 2.00 | exact | **exact** |
| 3.00 | exact | **exact** |

Note what the old path did below the cliff: it did not fail, it **returned
success with a wrong cycle**, fitting a 12-symbol local repeat (LSP = LTP = 12)
and calling it the DIL. So we did not merely fail to read a real DIL, we
silently mis-decoded one; the 280-symbol "DIL run" seen on the card's capture
is the same behaviour.

### What replaced it

Acquisition is now structural. §8.4.1 restarts SP and TP at every segment
boundary, so **the first training symbol sits at the same offset in every
segment** — consecutive first-occurrences are therefore exactly one segment
apart, whatever that shared offset is. Segment spacing comes out of the data
without any assumption that the cycle repeats; the shared offset is the single
remaining unknown, bounded by the gap back to the previous segment's last
training symbol, and settled by re-expansion.

The obvious rule — cut where a third Ucode appears — is wrong, and the card
shows why: its segments open with REF symbols, so the first symbol carrying a
new training Ucode is 8 symbols past the boundary, not on it.

Re-expansion now checks the **whole region** rather than one cycle of itself.
That is what makes the structural path safe where the periodic one was not: a
descriptor fitted to a short accidental repeat reproduces that repeat and
nothing else, so it cannot survive. The periodic search is kept as a fallback
for shapes the walk cannot segment.

`vpcm_loopback_test`'s `V.90 DIL decoded from a single pass` pins this, with no
lead or tail padding — a single pass is exactly the DIL and nothing else, which
is also the hardest case for finding where the region starts.

### Why no test caught it

`vpcm_test_dil_roundtrip_case()` sets `body = cycle * 2 + cycle / 2` — always
**2.5 cycles**, just past the cliff — under a comment that states the
assumption outright:

> Short cycles need more repetitions to clear the decoder's minimum run length
> (**real DILs run for seconds regardless of cycle size**).

Real DILs do run for seconds. The card's runs 1.97 s. It is still only one
cycle, because a long cycle and a repeated cycle are not the same thing, and
only the second one is what acquisition needed.

### What is genuinely unrecoverable from one pass

Two canonicalisations stop being distinguishable when the cycle arrives once,
and both are information that is simply not on the wire:

- A descriptor whose training-Ucode sequence is itself periodic decodes to that
  shorter period. The clean-line Ja profile is the example: its Ucodes are
  `(chord << 4) | offset` over 8 chords and 8 offsets, so the sequence repeats
  every 64 segments while N is 125 — and the first 125 segments of a
  64-segment cycle are the same symbols.
- H<sub>c</sub> and REF<sub>c</sub> for a chord the DIL never trains in.

## Finding 5 — the card's DIL does not decode, and that matters less than it looks

The acquisition fix does not make the card's DIL decode. Two things are worth
separating here, because they were conflated for most of this investigation.

### Descriptor recovery is not a modem function

DIL is Digital Impairment Learning, and **the analogue modem is the one that
asked for it**:

> §8.4.1 — "The parameters necessary for the digital modem to form the DIL are
> sent to it by the analogue modem using the DIL descriptor defined in 8.3.1."
>
> §9.3.2.9 — "The analogue modem shall **receive the DIL sequence it
> requested** in Ja."

So on the analogue side the descriptor is an *input*. The receiver already
knows every codeword that was meant to arrive; its job is to measure what did.
Recovering a descriptor nobody told us is what offline analysis of somebody
else's capture needs — it is not a step in a V.90 start-up, and building the
analogue role does not depend on it.

§9.3.2.10 makes the same point from the other end: the analogue modem stops DIL
as soon as it "has received **enough** of the DIL sequence". A partial pass is
the designed behaviour, not a degraded one. Every DIL-segment measures one
training Ucode independently, so half a cycle measures half the ladder.

`v90_dil_measure.c` does that job, and
`vpcm_loopback_test`'s *V.90 DIL impairment measurement from half a pass*
demonstrates it on a deliberately halved DIL carrying a 3 dB pad and robbed-bit
signalling in one DS0 phase of six:

```text
PASS: V.90 DIL impairment measurement from half a pass
      (ulaw, coverage=50%, 31 Ucodes measured, gain=-3.0 dB,
       RBS slot mask=0x04, 26 usable / 5 collapsed)
```

The pad and the RBS slot both come back exactly, from half a pass, and the
usable/collapsed split is the output that constellation selection actually
wants (`docs/v90_mi_negotiation.md`).

**One property worth knowing:** alignment cannot be found by a blind search.
G.711 is self-similar across chords — Ucodes *u* and *u+16* differ by exactly a
factor of two — so a DIL stepping one Ucode per segment reproduces its own
shape 16 segments later, scaled. A scale-invariant score cannot tell them
apart, and scale-invariance is exactly what is wanted (a pad is a scale
factor). §9.3.2.8-9 removes the problem: the analogue modem sends S and then
receives the DIL it just asked for, so it searches a window, not a capture.

### What still blocks the card, and what it is worth

Descriptor recovery on the card's capture remains open. Its training-symbol
first-occurrences are 66 symbols apart (H<sub>c</sub> = 10 — not the 132 an
earlier revision recorded, which is two segments) with REF<sub>c</sub> = Ucode
0, and the training Ucode steps down the ladder: 84, 83, 82, 81, 80, 79, …

At a uniform 66T segmentation — and at the best of all 66 alignments — only
**188 of 239** segments carry the two Ucodes §8.4.1 allows. **51 carry three**,
concentrated where the ladder reaches the bottom of the range:

```text
T99378   +0   +1   +0   +0   +0   +0
T99384   -2   +0   +0   +0   +0   +1
```

The leading explanation is that H<sub>c</sub> is per-chord and the card's ladder
crosses chords, so a uniform 66T is the wrong length down there and is merging
short chord-0 segments. That is unconfirmed; read "three Ucodes per segment" as
"at the segmentation we have tried", not as a claim about the card.

This is now a **tool** limitation rather than a modem one. It blocks reading
somebody else's DIL out of a capture — useful for exactly this kind of
archaeology — but it does not block the analogue role, which never needs it.
`make eicon-rx-test` still pins it, at that reduced priority.

## Withdrawn — do not re-derive

- ~~"The card transmits 4548T of Sd where §9.3.1.3 says 384T."~~ **Wrong.** That
  region is Ri (Finding 2), and the card's Sd is 384T exactly where §9.3.1.3
  puts it (Finding 3). Nothing in this capture shows the card violating
  §9.3.1.3, and `V90_SD_REPS 64` is not implicated. The SmartLink justification
  at `v90.c:75` stands unchallenged.
- ~~"A scrambler convention mismatch — seed, tap or polarity — explains why we
  cannot read the card's TRN1d."~~ **Disproved.** GPC, zero-initialized, matches
  100% (Finding 1). There is no convention error. Our "0 TRN1d symbols" is a
  search-and-labelling failure (Finding 2), not a disagreement about the
  scrambler. The GPA/GPC distinction CLAUDE.md warns about is not in play.
- ~~"Alignment the descrambler infers from our own S̄d length."~~ Withdrawn:
  `V90_SD_BAR_REPS` is referenced only by the transmitter; no receive path reads
  it.
- ~~"Long Sd predicts success."~~ Never survived its own census — the card sent
  the same long signal on call 2, which returned `NO CARRIER`. Withdrawn on its
  own terms even before Finding 1 removed the premise.

Four hypotheses, all killed by measurement. `modem-dsp-emu`'s handoff §0.5 is
worth importing as a habit: its measurements reproduce, its interpretations
repeatedly have not. Finding 1 is a measurement, verified twice. Treat the
causal story attached to it as still unproven until a live call says otherwise.

## Why our own tests could never have found this

Our receive path — `p3_demod`, `v90_dil_rx`, `v90_cp_rx`, and the
Sd/TRN1d/Jd/DIL decoders in `vpcm_decode` — is *analogue-side* code: those are
the signals an analogue modem receives from a digital one. Until these fixtures
existed it had only ever been fed by our own transmitter. In
`vpcm_loopback_test.c` that is `v90_dil_generate_codewords`, `v90_tx_codewords`,
`v90_tx_data_frame_codewords` and `v90_generate_trn2d_codewords` — all ours.

A wrong-but-self-consistent assumption is invisible to that harness by
construction: both ends agree, the suite goes green, the peer still fails.
CLAUDE.md's "passing them says nothing about hardware interop" is the weaker
form; the stronger form is that loopback cannot *in principle* catch it.

Finding 2 is an instance. Our Sd detector fires on a constant-magnitude signal;
our own Sd is never constant-magnitude, so the false positive can only be
triggered by someone else's transmitter.

## The fixture

`make eicon-rx-test` (`tools/eicon_rx_conformance.py`):

```text
[FAIL] call1-connect-32000.ulaw
         control: codeword-exact DIL run 14261.1-14829.6 ms
         control: Sd recovered, 758 reps
         TRN1d: 0 symbols descrambled (need >=48)
[FAIL] call3-connect-42666.ulaw
         control: codeword-exact DIL run 14317.9-14882.6 ms
         control: Sd recovered, 753 reps
         TRN1d: 0 symbols descrambled (need >=48)
```

Read that "control: Sd recovered" line with Finding 2 in mind — it is the
mislabel, not a pass. The control is doing its job in the narrow sense (codeword
recovery works, so the zero is a real failure) but its Sd arm is asserting
something false, and should be tightened to require Ucode-0 density near 1/3.

Not in `make test`: it encodes a known-open defect, and a suite that is red by
default stops being read. `--expect-failure` inverts the exit status, so the
target is green while the defect stands and goes loud if the control breaks or
the defect is fixed.

## Reproducing the segmentation

`tools/v90_downstream_segment.py` recovers the whole Phase 3 timeline from the
bytes, independently of the C receive path — which is the point, since it is
where `eicon_rx_conformance.py`'s expected offsets come from and it must not be
able to share a bug with the code under test:

```bash
python3 tools/v90_downstream_segment.py artifacts/eicon-digital-downstream/*.ulaw
```

```text
UINFO = 48  ->  W = Ucode 64 (§8.4.4)
Sd:     T66935       384T      48.0 ms  (64 reps)
S̄d:     T67319        48T       6.0 ms  (8 reps)
TRN1d:  T67367     30005T    3750.6 ms  (GPC-locked)
Jd:     T97372       943T     117.9 ms
DIL:    T98315     15774T    1971.8 ms
Ri:     T114089     4572T     571.5 ms  (§8.6.4)
data:   T118661    97499T   12187.4 ms
```

## Reproducing Finding 1

No build needed — it is a property of the bytes:

```python
import pathlib
d = pathlib.Path("artifacts/eicon-digital-downstream/call1-connect-32000.ulaw").read_bytes()
signs = [((b ^ 0xFF) >> 7) & 1 for b in d]
mags  = [(b ^ 0xFF) & 0x7F for b in d]

def gpc_ones(n, taps=(18, 23)):          # §5.3 -> V.34 GPC, zero-initialized
    s = []
    for i in range(n):
        v = 1
        for t in taps:
            v ^= s[i - t] if i - t >= 0 else 0
        s.append(v)
    return s

off = 67367                              # 8420.9 ms
run = 0
for i, r in enumerate(gpc_ones(len(d) - off)):
    if signs[off + i] != (r ^ 1):
        break
    run += 1
print(run, sorted(set(mags[off:off + run])))   # -> 30005 [48]
```
