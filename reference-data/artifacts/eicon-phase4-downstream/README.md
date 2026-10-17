# A foreign V.90 Phase 4 downstream

`run79-ri-rbar-trn2d.ulaw` — 30 s of µ-law from an Eicon Diva Server under
emulation, captured on the analogue side of a live call (2026-08-11, run 79),
starting at 15.000 s of the call. It is the first §9.4 downstream in this tree
that we did not generate ourselves, which is what makes it worth keeping: every
other Phase 4 test here is fed by `v90.c`, so both halves share any wrong
assumption and agree anyway.

What is in it, at the offsets an independent scan puts them at:

| from | to | signal |
|---|---|---|
| — | 15.446 s | the tail of Phase 3 |
| 15.446 s | 15.767 s | **Ri**, §8.6.4 — 2568T of Ucode 22 in `+ + + − − −` |
| 15.767 s | 15.770 s | **R̄i** — exactly four repetitions of `− − − + + +`, 24T, as §9.4.1.2 requires |
| 15.770 s | end | **TRN2d**, §8.6.5 |

Ri is Ucode 22. §8.6.4 says it is "the single PCM codeword whose Ucode is
U<sub>INFO</sub>", and this call announced U<sub>INFO</sub> = 48 — so this card
does not honour that field either, the same way it ignores it when choosing Sd's
W (`docs/v90_analogue_role.md`). The Ri hunt in `v90_analogue_phase4.c` learns
the level from the wire and does not care, which is why it acquired anyway.

**The TRN2d does not decode, and the reason is now measured.** The question this
fixture was kept to answer — whether the card declines the constellation our CPt
named, or our six-symbol frame grid is off at the R̄i seam — resolves to neither
of the phrasings it was posed in. `v90_analogue_rx_test` now runs against this
file and pins all of it:

* **The grid is right.** Our receiver puts TRN2d's first symbol at sample 6135.
  An independent scan of the Ri sign pattern agrees: the `+ + + − − −` grid
  reverses to `− − − + + +` after 2544 symbols and runs 24 more, ending at 6135.
  (Our `r_symbols` counter reports Ri as 2556T, because the reversal detector
  needs two of R̄i's four repetitions before it fires and charges those 12
  symbols to Ri. The TRN2d start is unaffected.)
* **Nothing is outside the constellation.** Splitting the demap failure by cause
  — the receiver now counts "codeword not in the constellation" separately from
  "§5.4.3 modulus value needs more than K bits" — gives **out-of-constellation 0**
  and modulus overflow on **every** frame. The earlier "13% of its symbols are
  outside every constellation the CPt named" was measuring the wrong thing: a
  frame that will not demap is not evidence that any symbol was rejected.
* **The card maps TRN2d at K ≈ 31 — the data K, not CPt's K = 24.** Its TRN2d
  uses ~40 magnitudes (Ucodes 0–39) with the same support in all six intervals,
  white (autocorrelation < 0.012 at every lag 1–8) and stationary across the
  whole 30 s. Reading each six-symbol frame back through the §5.4.3 modulus
  decoder over `Mi = 39 39 37 39 39 39` (run 79's data constellation) gives
  modulus values that reach **2^31.6**, median **2^31.3** — 99.6 % of frames
  exceed 2^24. So the card encoded TRN2d with K ≈ 31, and a receiver holding it
  to CPt's declared K = 24 overflows on every frame, which is exactly what the
  test measures.

**This was ours, and it is now fixed.** Run 79's `docs/v90_analogue_role.md`
shows the analogue side chose a data constellation `Mi = 39 39 37 39 39 39`
(K = 31, 49333 bps) and built **CPt from those same full masks**, only relabelling
`drn` to declare K = 24 (`v90_analogue_phase4_build_cp`). §8.6.5 says TRN2d is
mapped with the CPt set, and a digital modem takes K from the constellation it is
handed — so a CPt whose masks still hold 2^31 points *is* a K = 31 offer no matter
what `drn` says. The card faithfully mapped TRN2d at that capacity; our own K = 24
receiver then could not demap it. The fix reduces CPt to a genuine subset whose
`prod(Mi)` is commensurate with K ≤ 24 (dropping the lowest-amplitude points, which
keeps §8.5.2's power relation on the safe side), so the offer is self-consistent.

This capture predates that fix and is kept as the record of the K = 31 TRN2d the
old inconsistent CPt provoked. Live re-verification against the card (which should
now map TRN2d at K = 24 and let MP through) is the open confirmation — it needs the
`../modem-dsp-emu` rig, not this decoder.


## Correction (2026-09-28): the K ≈ 31 reading was an artefact

The section above concludes that the card mapped TRN2d at K ≈ 31, from modulus
values "reaching 2^31.6". **That metric measures the radix capacity of the
mask, not the information content of the stream, and cannot distinguish
anything.** Measured: `max(r)` reads 2^31.71 against a 2^31.712 capacity under
all four label/digit conventions, and under all 720 digit-significance
permutations. It is the same class of trap as "0.667 is white" and
"22.5° is white" elsewhere in this project — quote the spread, or the count,
not the extreme.

The instrument that works here is **entropy**. The sum of the card's
per-column magnitude entropies is **27.77 bits/frame**, which bounds the joint
entropy and therefore K. Our own mapper at K = 31 measures 31.00. So the
card's TRN2d cannot be carrying 31 bits per frame, and K ≤ 27.

Entropy also says the card is not running a plain §5.4.3 modulus mapping over
this mask at all. A mixed-radix mapper leaves every column either exactly full
or exactly empty, with the truncation confined to one column — ours at K = 24
reads 5.284 / 5.284 / 5.284 / 5.282 / 2.927 / 0.000, interval 5 pinned to a
single Ucode in 100% of frames. The card's six columns are all *partially*
filled:

| interval | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| entropy, bits (max 5.285) | 4.746 | 3.588 | 5.205 | 4.966 | 4.983 | 4.278 |
| share of U39 (uniform = 2.5%) | 7.7% | 50.5% | 4.6% | 18.2% | 6.3% | 36.4% |

with the excess piled on the **largest** Ucode the mask allows.

What the fixture does still establish, and it is worth keeping:

* **Extraction and grid are validated against known content** — Ri reads
  Ucode 22 for exactly 2544 symbols with `+++---`, and R̄i `---+++` for
  exactly 24, so sample 6135 and the Ucode mapping are right.
* **The card honours our CPt mask exactly.** Its TRN2d support is 39 Ucodes
  per interval spanning 0..39 with 35 absent, and never a Ucode above 39 —
  our mask and nothing else.

The most consistent reading of the rest — a reading, not a measurement — is
**clipping against a mask that was too small**. Run 79's CPt was built under
the erroneous 11.77 dB pad and named only Ucodes 0..39, a **−32 dBm0**
constellation (RMS 100 in magnitude units, ×4 = 400 against Table 15's 16017
at 0 dBm0). A card whose target amplitudes exceed that mask pins them at the
top, which is exactly the rising density observed: U0..30 ≈ 1.7% each,
U32..38 ≈ 3.9%, U39 20.6%.

**Consequence: this fixture grades acquisition, not mapping conventions.** It
was run as an oracle for our TRN2d conventions — 8.6.5 initialises the
scrambler, differential encoder and shaping memory to zero, so a matching
implementation descrambles to ones from the first frame with no warm-up — and
it returns 50.06%, which under a stream this distorted says nothing about our
conventions either way. **Our TRN2d conventions remain unvalidated against any
foreign transmitter.** The CPt-subset fix this capture was cited for stands on
§8.6.5 and Table 17 by inspection; only its evidence narrative changes.
