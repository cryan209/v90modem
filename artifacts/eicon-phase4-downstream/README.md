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

**The TRN2d does not decode yet.** 13% of its symbols are outside every
constellation the CPt named, so every six-symbol frame is rejected and none of
§8.6.5's scrambled ones come back. Whether that is the card declining to use
the constellation we asked for, or our six-symbol frame grid being off by a
symbol at the R̄i seam, is the open question this fixture exists to answer.
