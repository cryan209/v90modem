# V.34 training carrier loop: it was undamped (2026-11-26)

Found while making the answer modem keep its Phase 3 receiver into Phase 4
([v34_phase4_j_dashed.md](v34_phase4_j_dashed.md)). Kept, the Phase 3 carrier
rate turned TRN to 50% about 700T in, because it was 2.4 Hz off nominal in a
loopback with no frequency offset.

## The loop

`process_primary_symbol()` (`v34rx.c`) runs a decision-directed,
proportional-plus-integral carrier loop in every primary training stage
(Phase 3 and Phase 4). The error is the cross product with the nearest
4-point decision. The gains are in DDS units per symbol:
- `carrier_track_i` is added to the rate.
- `carrier_track_p` is added to the phase.

They are set in `reset_primary_rx_frontend_for_phase3()` (`v34tx.c`) and
in `v34_rx_restart()`. As imported from SpanDSP they were `i = 5000`,
`p = 40000`.

At 3429 baud that makes:
- the integral step 0.0093 Hz per radian of error per symbol, a natural
  frequency of ~0.004 rad/symbol (period ~0.45 s);
- the proportional step 0.003° per unit of error, a damping of ~0.007.

The loop is an undamped oscillator. The other SpanDSP receivers run p at
500 to 5000 times i (V.17 100-1000 / 500000, V.22bis 8000 / 8000000, V.29
200 / 1000000); V.34's ratio was 8.

## Measured (v34_duplex_test 3429/33600 μ-law, 36 dB, no frequency offset)

- **Transmitters.** Both are exactly at the nominal 1959.184 Hz in every
  stage.
- **The loop wanders.** In `PHASE3_WAIT_S` the rate climbed to 1961.6 Hz on
  the answer side and fell to 1956.6 Hz on the caller side, ringing about
  ±0.4 Hz. Over delays 5, 17 and 23, Phase 3 ended 0.7–2.8 Hz off.
- **The signal does not.** With the loop frozen in `WAIT_S` at 1957.94 Hz,
  the symbols turned at +1.3 Hz, which puts the input at the nominal
  carrier.
- **Ruled out:**
  - The CMA equaliser: with Phase 3 CMA off the rate still reached
    1962.3 Hz.
  - Timing drift: one TED correction unit is 1/192 sample, 0.46° of
    carrier, and the walk was 13 units.
- **Not used in Phase 4.** Phase 4 resets the rate to nominal and freezes
  tracking for TRN, so the error never reached data mode. That is why it
  hid: only a receiver kept from Phase 3 inherits it.

## The fix: `p = 2000000` (damping ~0.35)

Phase 3 now ends within 0.28 Hz on the same three delays. A transient of up
to ~2 Hz remains early in `WAIT_S`.

| `V34_DUPLEX_DELAY` | HEAD (p = 40000) | p = 1000000 | p = 2000000 | p = 4000000 |
|---|---|---|---|---|
| 2400/21600 10 dB, 0–47 | 48 | 48 | 48 | 46 |
| 3429/28800 36 dB, 0–47 | 48 | | 48 | 48 |
| 3429/31200 38 dB, 0–47 | 44 | 46 | 47 | 46 |
| 2400/21600 10 dB, 48–95 | 46 | | 47 | |
| 3429/31200 38 dB, 48–95 | 43 | | 44 | |

**Plain V.34 only.** The V.90 receiver shares this loop. With
p = 2000000 there, the batch CP decoder no longer found the 428-bit CP in
the CX93001 replay in `tests/fast.list`
(`cx93001-unqualified-e-20261023`). So `v90_mode` keeps `p = 40000` until
V.90 is measured on its own, and that includes the V.90-declined V.34
fallback. V.90 has its own Phase 3 PP tracking (`v90_mode ||
hdx_primary_resync`), and its Phase 3 rate error has not been measured.

Over the 240 calls, p = 2000000 passes 234 against 229. p = 4000000 lost two
10 dB calls that never trained, to phase noise. Data mode does not use
these gains.
