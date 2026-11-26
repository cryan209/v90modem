# V.34 Phase 4: the answer modem's J′ detection

How our answer modem finds the caller's J′ at the start of Phase 4, why the
default detector fires on noise, and the opt-in `V34_P4_ANSWER_J_FRAMING=1`
that detects it for real (2026-11-26).

## What the caller sends

§11.4.1.1.1: the call modem sends J until it sees the answer modem's
S-to-S̄, then one J′ and then TRN, with no break. J (Table 18) and J′
(Table 19, `0x899F`) are 16-bit patterns on the same 16-bit period,
scrambled and differentially encoded on the 4-point constellation
(§10.1.3.3, §10.1.3.4); J′ lasts 8T. TRN is scrambled ones mapped
absolutely (§10.1.3.8). So, to the answer modem, Phase 4 opens with a
continuous J that changes into J′ and then TRN.

## Why the default detector fires on noise

The detector in `v34rx_phase4_trn.c` searches 1536 descrambler hypotheses
and fires on the first 24/32 window match against J′ whose last 16 bits
are within 3 of J′. It has never seen J. Two defects of ours, stacked:

1. **The receiver is reset under a running signal.** The answer modem's
   Phase 4 entry (`phase4_rx_conditioning_init()` from the Phase 4 wait,
   `v34tx.c`) zeroes carrier phase, symbol timing and equaliser history
   while the caller's J is still on the line. It re-acquires for about
   260T, which is all of J. In a symbol dump (3429/33600 μ-law,
   `V34_DUPLEX_DELAY=23`), magnitudes run from 0.04 to 1.96 and the angles
   spread across the whole circle until TRN. An offline scan of those
   dibits finds J under no map, tap or bit order (best 61%). TRN then
   descrambles to 100% ones from symbol ~265.
2. **The decode could not read J anyway.**
   - It took the difference of the already-differential dibit a second
     time.
   - It tried only the first 8 of the 24 dibit maps. J decodes under
     hyp 8 (q → −q), as the Phase 3 J detector already locks it.
   - It compared only against J′, which J matches on 12 of every 16 bits.
     During J it therefore sits at exactly the 24/32 firing threshold.

With the receiver kept from Phase 3, the same dump decodes J at 100%
offline (hyp 8, differential, tap 17, b0,b1), but the online detector still
fired at bit 34 on a chance match.

The default timing (a noise match about 17T after Phase 4 entry) is what
the default pass rates were measured on. `PHASE4_TRN_READY_MIN_BAUD`
counts from that event, so do not change it piecemeal.

## The opt-in: `V34_P4_ANSWER_J_FRAMING=1`

- **Receiver.** The answer modem keeps its Phase 3 receiver into Phase 4,
  carrier rate included. Before the carrier loop was damped
  ([v34_carrier_loop.md](v34_carrier_loop.md)), Phase 3's rate was 2.4 Hz off
  nominal (1961.6 Hz against 1959.2 Hz) in a loopback with no offset, and
  kept, it turned the constellation until TRN fell from 98% to 50% about
  700T in (3429/28800, 36 dB, delay 17). The first version reset the rate
  to nominal; with the damped loop, resetting it or not gives the same
  sweep.
- **J framing.** The receiver locks on a descrambled stream whose last 32
  bits are two identical, correctly framed J periods (4- or 16-point J).
  Random bits do that about once in 2^16 × 2048 tries. It then classifies
  each 16-bit frame:
  - J: keep going.
  - J′ (within 2 bits): fire. TRN starts with the next symbol.
  - Nearly all ones: J′ was hit by errors. Fire, with TRN taken as 8T old.
  - Anything else: lose the lock.
- **Fallback.** With no lock by 192 bits, it falls back to the old search.
- **Not affected.** The §11.6 responder enters the same stage with no J
  and keeps the old detector either way (`phase4_jf_expect_j`).

Measured at 3429/33600 μ-law, delay 23: J framed at bit 56 on hyp 8 (the
same map, tap and order as Phase 3), and J′ framed at bit 550, where TRN
begins.

## Results (loopback, `V34_DUPLEX_DELAY` 0–47, same machine)

Before the carrier loop was damped:

| Arm | 2400/21600 10 dB | 3429/28800 36 dB | 3429/31200 38 dB | Total |
|---|---|---|---|---|
| default (HEAD b3ed85f1 + baf083cc) | 48 | 48 | 44 | 140/144 |
| framing, receiver reset | 48 | 48 | 41 | 137/144 |
| framing, receiver kept with Phase 3 carrier rate | 45 | 47 | 40 | 132/144 |
| framing, receiver kept, carrier rate reset to nominal (eea66c86) | 48 | 48 | 42 | 138/144 |

With the damped carrier loop (`carrier_track_p = 2000000`):

| Arm | 2400/21600 10 dB | 3429/28800 36 dB | 3429/31200 38 dB | Total |
|---|---|---|---|---|
| default | 48 | 48 | 47 | 143/144 |
| `V34_P4_ANSWER_J_FRAMING=1` (Phase 3 rate kept, or reset: identical) | 48 | 48 | 43 | 139/144 |

The opt-in loses delays 10, 18, 24 and 30 on the 38 dB row, which sits on
the 31200/33600 rate boundary; 22 fails in both arms. So it stays opt-in.
The cause is not yet known: `PHASE4_TRN_READY_MIN_BAUD` and the TRN
scoring skip were tuned on the noise-timed event, and a correct J′ starts
them about 260T later. Loopback cannot show whether a correctly timed TRN
count helps a real caller; that needs the rig. With the switch off, result
lines are byte-identical to the default build.
