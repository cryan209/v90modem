# A V.92 Phase 3 upstream off a real analogue loop

The digital side's own G.711 receive tap (`VPCM_G711_TAP_DIR`'s
`live-rx.g711`) from one live call in the real V.92 topology:

    Apple USB modem (V.92 analogue modem, ext 6004)
      -> 2-wire loop -> Cisco VG224 FXS (the one codec in the path)
      -> SIP G.711 u-law -> sip_v90_modem on tower (V.92 digital modem, ext 6000)

It is the only upstream in this repository that crossed a real analogue
loop. Every other V.92 receive test is fed a byte-exact DS0, which hides ISI,
fractional sampling phase and clock offset. That is the point of this file.
Read by `v92_p3_rx_line_test` (`make v92-loop-rx-test`).

Raw G.711 u-law codewords, 8000 Hz, no header, 20000 samples (2.5 s).

```text
sha256  e36bac83d80d2d731d3d8293785ee9d1c35e97cebfe030530eaddbef06b9267f  live-rx.g711
```

## Provenance

Bytes 80000..99999 of `artifacts/apple-v92-sip-r4/server/live-rx.g711`
(local only, recorded 2026-10-01 by that directory's `run.sh`). Sample `n` here
is sample `n + 80000` there.

Call parameters:

- `ME_MODE=v92 ME_V92_PCM_UPSTREAM=1`.
- INFO1a: U_INFO=78, MD=0, Table 18 linear PCM upstream.
- The engine armed the V.92 Phase 3 raw receiver at sample 81440, i.e. **1440**
  here.

## What it contains

What the analogue side transmitted (`v92_analogue_phase3.c`):

1. Ru 384T and Ru-bar 24T.
2. **Exactly 2040T of TRN1u.** Per V.92 8.5.7 this is GPA scrambler output,
   zero-initialised and fed ones, 0 -> +L_U. Its sign sequence is therefore
   known from the first symbol.
3. Ja repeated for 12000T until its 9.5.2.2.1 Sd-bar timeout. The digital side
   never sent Sd.

The DIL descriptor in Ja is the one it logged as
`measurement-120x66`: N=120, LSP=12, LTP=11.

Measured positions (fixture sample index):

| event | sample | source |
|---|---|---|
| Ru acquired | 5008 | `v92_p3_rx` |
| Ru-bar acquired | 5272 | `v92_p3_rx` |
| TRN1u entered by the receiver | 5323 | `v92_p3_rx` |
| TRN1u actual first symbol | ~5292-5299 | `tools/v92_trn1u_bound.py` (1-tap offset -31, 21/41-tap -24/-20) |
| first CRC-valid Ja (equalised signs) | 17733 | control below |

Results:

- **Today's receiver**
  - Rejects TRN1u with `trn1u_ones_low` (48% against a 75% gate), then
    rehunts.
  - Against the known reference the raw sign is wrong 14% of the time.
  - A 41-tap least-squares equaliser takes that to 0.17%, held out.
- **Equalised-sign control.**
  - Method: an LS-seeded 41-tap equaliser, data-aided over TRN1u and
    decision-directed after it. Each sign it decides is written back as a
    codeword with that sign, and the result is fed to the *unchanged*
    receiver.
  - Outcome: it passes TRN1u and decodes the descriptor above at 17733.
  - So with correct signs the existing state machine and Ja search work. What
    is missing is the front end (`docs/v92_p3_rx_line_plan.md`).
  - It took 79 Ja rejects to get there, which is for step 6 to explain.
- **A second, separate oddity.** The receiver later takes a false Ru -> Ru-bar
  -> TRN1u lock at 18214/18215/18222, inside the repeated Ja: Ru-bar after one
  symbol of "Ru".
