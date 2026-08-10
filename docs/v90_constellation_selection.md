# Choosing the downstream constellation, and the rate that follows

How a measured DIL becomes Ci, Mi, a `drn` and a bit rate — and which of the
three constraints in play actually sets the answer.

Implemented in [`../v90_dil_measure.c`](../v90_dil_measure.c). The consuming
side (CP frame → per-interval Mi → §5.4.3 modulus encoder) was already built
and live; see [`v90_mi_negotiation.md`](v90_mi_negotiation.md) for that half.

## Who does this, and why it is not descriptor recovery

DIL is Digital Impairment Learning, and the analogue modem is the one that
asked for it:

> §8.4.1 — "The parameters necessary for the digital modem to form the DIL are
> sent to it by the analogue modem using the DIL descriptor defined in 8.3.1."
>
> §9.3.2.9 — "The analogue modem shall **receive the DIL sequence it
> requested** in Ja."

So the descriptor is an **input** on this side. The receiver already knows
every codeword that was meant to arrive; its job is to measure what did.
Recovering a descriptor nobody told us is a different problem —
[`v90_dil_rx.c`](../v90_dil_rx.c), used for offline analysis of somebody else's
capture, and not a step in a V.90 start-up.

§9.3.2.10 also caps how much data there will be: the analogue modem stops DIL
once it "has received **enough** of the DIL sequence". A partial pass is the
designed behaviour. Every DIL-segment measures one training Ucode
independently, so half a cycle measures half the ladder and is a usable answer.

## The chain

1. **Measure.** Per transmitted Ucode, record what arrived — the dominant
   received Ucode, its mean level, and the **spread** of that level. All of it
   per data-frame interval, because robbed-bit signalling alters one DS0 phase
   in six and §5.4.3 makes Mi per-interval for exactly that reason.
2. **Build Ci.** Walk the ladder upward in interval *i*, keeping a Ucode only
   when its received level clears the last kept one by the required
   separation (below).
3. **Mi = |Ci|.**
4. **Apply §8.5.2.** Drop points from the top until the set fits Table 15.
5. **Gate on §5.4.3.** The modulus encoder needs ∏Mi ≥ 2^K, so the largest
   supportable `drn` is the largest whose K does not exceed Σlog₂(Mi).
6. **Rate is §5.4.1**: D = drn + 20 bits per 6-symbol frame, ×8000/6.

## The three constraints, and which one binds

### Impairment — what arrives distinguishable at all

A digital pad compresses the ladder so neighbouring Ucodes land on one received
code. Robbed-bit signalling takes the codeword LSB in one DS0 phase, merging
adjacent levels in that interval only. Both fall out of step 2 as points that
never enter Ci.

From **half** a DIL pass, with a 3 dB pad and RBS in one slot of six:

```text
coverage=50%, 31 Ucodes measured, gain=-3.0 dB, RBS slot mask=0x04
impairment only: Mi=[26 26 16 26 26 26] drn=13 44000 bps
```

The robbed interval carries 16 points where the clean ones carry 26 — the
per-interval Mi doing the job it exists for.

### Noise — thins the constellation from the *bottom*

Two points are only distinguishable if their received levels differ by enough
relative to the noise on them. The threshold is `noise_sigmas × √(σ₁² + σ₂²)`
— the *difference* of two noisy observations has to clear it, not either one
alone.

G.711's levels are spaced geometrically, so they sit closest together at the
bottom of the ladder. That is where a given absolute noise merges them, and it
is why noise is a low-end constraint.

### §8.5.2 average power — thins it from the *top*

The Recommendation gives the formula explicitly and Table 15 caps it against
the digital modem's maximum transmit power (INFO0d bits 33:37):

```text
                     Σ_{i=0..5} Σ_{j=0..Mi-1} p_i,j · n_i,j
average power  =  ─────────────────────────────────────────
                                  6 · 2^K
```

`p_i,j` is the square of the Table 1 linear value of level *j* in constellation
*i*. `n_i,j` is how often the §5.4.3 modulus encoder uses that level across all
2^K input words, taken at R₀ = 2^K − 1. **Levels are not equiprobable** —
treating them as if they were misstates the power of exactly the large-Mi sets
that matter.

Power is dominated by the largest levels, so this is a high-end constraint.

## Measured: neither constraint alone is usually the story

Sweeping §8.5.2 alone, clean channel, DIL probing the whole ladder:

| max tx power | points dropped | Mi | drn | rate |
|---|---:|---:|---:|---:|
| none | 0 | 121 | 22 | 56 000 |
| −6 dBm0 | 19 | 118 | 22 | 56 000 |
| −12 dBm0 | 117 | 101 | 22 | 56 000 |
| −16 dBm0 | 184 | 90 | 22 | 56 000 |

**It costs constellation, not rate.** §5.4.3 needs only about 64 points per
interval for drn 22, and G.711 packs plenty of levels down low, so removing the
top still leaves margin even at Table 15's tightest entry.

Now the same cap on a line that is also noisy (σ ≈ 40 linear, 3σ margin):

| constraint | Mi | drn | rate |
|---|---:|---:|---:|
| noise ignored | 114 | 22 | 56 000 |
| 3σ margin | 65 | 22 | 56 000 |
| 3σ margin + §8.5.2 at −12 dBm0 | 34 | 16 | **48 000** |

Read the middle row carefully: noise removed 335 points and the rate did not
move, because 65 is still above the ~64 that drn 22 needs. Power alone would
not have moved it either. **Together they take it to 48 000.**

That is the point of doing both: the constraints squeeze from opposite ends of
the same ladder, and the rate is set by what survives in the middle. Modelling
either one alone gives an answer that is not just optimistic but structurally
misleading — it suggests there is headroom where there is none.

Both outcomes are pinned in `vpcm_loopback_test`
(*V.90 DIL impairment measurement from half a pass*, *V.90 constellation
trade-off, noise vs §8.5.2 power*).

## Two traps

### A DIL descriptor can be valid and still measure nothing

Segment lengths are multiples of 6 and TP restarts at every segment boundary,
so each data-frame interval is pinned to a fixed subset of TP bit positions. If
LTP shares a factor with 6 and none of that interval's bits are set, the
interval receives **only** REFc for the whole DIL and nothing is learned about
it.

The first descriptor tried here did exactly that — LTP 12 left interval 1
landing only on TP bits 3 and 9, both zero — and the measurement correctly
reported Mi = 1. `intervals_unprobed` reports it as its own condition, because
the fix is **a different descriptor (LTP coprime with 6)**, not a lower rate.
Anything authoring a DIL descriptor here has to check it.

### Alignment cannot be searched blindly

G.711 is self-similar across chords — Ucodes *u* and *u+16* differ by exactly a
factor of two — so a DIL stepping one Ucode per segment reproduces its own
shape 16 segments later, scaled. Normalised correlation cannot see a scale
factor, and that is the property wanted, because a pad *is* a scale factor. A
blind search is therefore ambiguous by one chord and no level-based score fixes
it.

§9.3.2.8-9 removes the need: the analogue modem sends S and then receives the
DIL it just requested, so it knows when DIL starts.
`v90_dil_measure_align()` takes `from`/`span` for that reason, and a wide
search only breaks ties toward the earliest offset — honest, not correct.

## Still open

- **The power reduction is greedy.** It drops the highest-magnitude point
  first, which buys the most power for the least rate. That is a reasonable
  heuristic, not an optimum: a different set of the same size may fit the same
  power budget with better spacing. Nothing here searches for it.
- **Noise is measured, not predicted.** σ comes from the spread of received
  levels during DIL. That is the right thing for a stationary channel and says
  nothing about impulse noise or anything that arrives later.
- **`noise_sigmas` is a caller's policy, not a derived target.** Mapping a
  wanted symbol-error rate to a σ multiplier is not done here.
- **§8.5.2's other clause is not applied** — the one holding data-mode average
  power within 3 dB of the Phase 4 constellation's. That constrains CP against
  CPt, so it belongs with whatever builds both.
