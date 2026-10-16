# V.91 conformance audit (ITU-T V.91, 05/99)

Audited 2026-10-16 by reading the whole Recommendation (`ITU Docs/T-REC-V.91-199905-I!!PDF-E.pdf`,
~870 lines of text, 14 pages) against `v91.c`, `vpcm_v91_session.c`,
`vpcm_cp.c` and the live path in `modem_engine.c` (`v91_live_*`).  Code was
read, not run; no test was executed for this note.

## Headline

The start-up *signals* are mostly right.  The **data-mode line encoding is not
V.91** -- it is a bring-up codec that only interoperates with itself -- and the
control channel, retrain, escape-to-V.34 and timers are absent from the live
engine.  Nothing has met a foreign V.91 implementation; I know of none to
test against.

Rough scoring by clause (done = matches the text, partial = right shape or
only on the loopback/session path, missing = not there):

| Clause | Subject | State |
|---|---|---|
| 6.1 / 6.2 | Rates 28000..64000 step 8000/6, K 15..42, 8000 baud | done (`vpcm_cp_drn_to_k`: K = drn+14, drn 1..28) |
| 6.3.1 | GPC scrambler on the primary channel | **missing in data mode** (used only for SCR/CP/J/PHIL) |
| 6.3.3 - 6.3.8 | V.90 5.4 encoder: bit parsing, modulus encoder, mapper, differential encoding of sign bits, sign assignment, mux | **missing** (see below) |
| 6.3.9 / 6.4 | Control channel (96-bit sequence, Table 3) | **missing** |
| 6.5 | Transparent mode | partial: codewords pass through; not offered by the live engine |
| 7.1 | B1 = 12 codewords from mapping binary ones through the data encoder | **non-conformant** (bring-up mapper) |
| 7.2 | CP (Table 4) | partial / **non-conformant layout** (see below) |
| 7.3 | DIL; default DIL (Table 5) | done (`v91_default_dil_init` matches N=125, LSP/LTP 12, SP 0FC0, TP 0FFF, H=1, training 124,0,123,1...62) |
| 7.4 - 7.7 | Eu / Es / Ez / Em lengths and initialisation | done (12/12/24/12 symbols); init rules covered by loopback tests only |
| 7.8 | INFO (Table 6) | done as a frame; live engine only ever uses default DIL, no control channel, no transparent request |
| 7.9 / 7.10 | J / PHIL | partial: in `v91.c` and the session/loopback path, **not in the live engine** |
| 7.11 | SCR, multiple of 6 symbols | done |
| 8.1 | V.8 selection, 75 +/- 5 ms silence | done (V.8 `V8_PSTN_PCM_MODEM_V91` picks it; `V91_PHASE1_SILENCE_SYMBOLS` 600) |
| 8.2 | Start-up Figures 1-3 | Figure 1 only (both default DIL) live; Figures 2-3 on the session path |
| 8.3 | Escape to V.34 (no INFO' in 1.5 s, no B1 in 10 s, INFO0a/c handshake) | **missing in live engine**: a failure sets `ME_HANGUP` |
| 8.4 - 8.5 | Retrain, 10 s timer, loss of frame sync | flags only (`retrain_timer_active`); no timer, no INFO re-entry live |
| 8.6 | Cleardown via drn=0 or control channel bit 16 | partial: startup only; a peer CP with drn=0 is treated as a startup failure |
| 8.7 | Transparent grant needs verified unrestricted 64k channel | partial: only LSB-robbing is detected, no polarity/other checks (see 6.5 NOTE) |
| 8.8 | 4-wire leased line | n/a (same start-up) |
| 9 | Test loop 3 (V.54) | **missing** |
| 5 | Circuits, DTE interface | PTY via `data_interface.c` with V.14/V.42 covers 5.2 |

## Data-mode encoding is not V.91 (the important gap)

6.3 says the encoder "is defined in 5.4/V.90 and Figure 1/V.90" with Sr = 0,
S = 6.  `v91_tx_codewords` / `v91_rx_codewords` (`v91.c`, ~1560-1700) instead:

- split K as evenly as possible over the six frame intervals
  (`v91_bits_per_interval`: `k/6` plus one for the first `k%6`), where V.90
  5.4.2 parses by the modulus-encoder shell structure;
- feed those bits straight in as an **ordinal into the CP mask** -- no
  modulus encoder (5.4.3);
- use the sign bit as raw user data -- no differential encoding of sign bits
  (6.3.6: "applied continuously to all sign bits"), no sign-assignment
  (5.4.6), no mux (5.4.7);
- apply **no scrambler** to the data (6.3.1).

`v91_select_b1_ucode` is labelled "Bring-up mapper" in the source: it picks a
codeword from K/interval/position arithmetic rather than "binary ones mapped
using the selected data mode" (7.1).

Net effect: the two-instance call at 52000 proves our own TX and RX agree
with each other, and the robbed-bit alignment work sits on top of that.  A
spec-conformant peer would see a differently coded, unscrambled stream.  The
fix is mostly reuse: V.90 already has the 5.4 encoder (`v90.c`, `V90_TX_DATA`
modulus encoder) and V.91 only differs in Sr = 0, S = 6 and symmetric use, so
this should be a wiring job, not a new DSP block -- the same pattern the
`vpcm_*` layer already uses for V.92.

## CP layout

`vpcm_cp_encode_bits_fixed_fill` writes the **V.90 Table 14 CP**.  V.91
Table 4 keeps the same sync/drn/ack/constellation structure but:

- bits 30:32, 35:50, 52:67, 69:84, 86:101, 129:135 are "reserved, set to 0 by
  the transmitter".  We drive them with V.90 content (silence request,
  shaping redundancy, codec A-law at 35, upstream rate mask 36:48, TRN1d
  gain 52:67, shaping coefficients).  A conformant receiver ignores them, so
  this is tolerable on the wire, but a V.91 CP should have them zero.
- the live engine **repurposes `upstream_rate_mask` (bits 36:48) for the
  robbed-cadence alignment request**.  That is our private extension inside
  reserved bits; only our own receiver understands it.
- the length is wrong: Table 4 runs `272+gamma` start bit, 16 CRC bits, one
  fill bit, then zeros "to the next multiple of 6 bits" (SCR/CP are mapped six
  symbols at a time).  The shared encoder pads a fixed 3 bits
  (`156 + 136 n` total), which for one constellation is 292 against the
  spec's 294.  `v91_live_rx_*` also requires the exact length, so a peer's CP
  would not parse.
- the CRC span (which bits are excluded) was written for V.90 and has not
  been checked against the V.91 text.

## Control channel (6.3.9, 6.4, Table 3)

Not implemented at all.  INFO bit 27 (request control channel) is encoded and
decoded but nothing honours it, and the spec requires the peer to supply it
"if and only if" requested.  That removes: digital-slip detection (the 16-bit
pattern 0111011111111010), in-band cleardown (bit 16), retrain warning
(bit 17), the 54-bit secondary channel, and the 125 bit/s it costs (every 64th
data frame, all six sign bits, differentially encoded but not scrambled).

This is also why sample slips are fatal to our live alignment: V.91 itself
expects a slip detector, and our robbed-cadence phase tracking
(`g_v91_*_abs_pos`) is the stand-in.

## Procedures

- 8.2.1.8: CP within 1.5 s of Eu/Em -- not enforced.
- 8.3: INFO' within 1.5 s and B1 within 10 s, else escape to V.34 with
  INFO0a/INFO0c -- not present; `v91_live_rx_fail_locked` retrains the V.91
  state flag and sets `ME_HANGUP`.
- 8.4/8.5: 10 s recovery timer exists as a boolean only.
- 8.7: the live INFO never sets bit 40/41, so transparent 64k is never
  requested live.  The session path negotiates it and denies it on a detected
  robbed trunk.

## What is solid

Start-up signal construction and parsing (Ez/INFO/Eu/DIL/SCR/CP/Es/B1 as
signals), the INFO frame, the default DIL, DIL-driven DRN selection, and
undeclared robbed-bit detection with odd-ucode restriction are all coded
against the text and exercised by the loopback tests
(`vpcm_loopback_test.c`) and `make v91-serial-pair-test`.

## Suggested order of work

1. Replace the bring-up data mapper with the V.90 5.4 encoder (Sr=0, S=6),
   scrambled and differentially coded per 6.3, and make B1 real.  Highest
   value: without it nothing else can interoperate.
2. Fix the CP: V.91 Table 4 field set, reserved bits zero, multiple-of-6
   length; move our robbed-cadence request out of the reserved CP bits (send
   it in the control channel's secondary data or a private INFO bit nobody
   else reads).
3. Control channel (Table 3) with slip detection, then retrain/cleardown on it.
4. 8.3 timers and escape to V.34, 8.4/8.5 recovery timer, J/PHIL/Em in the live
   engine, transparent request live.
5. V.54 loop 3 (clause 9) last.

## Peer availability

The V.91 text pins the data-mode encoder to V.90's by reference (6.3.2 -
6.3.8), uses V.90's DIL (7.3, 7.9) and the V.90 INFO0 handshake for the V.34
escape, and its CP is V.90's with the V.90-only fields reserved.  It is
V.90's downstream applied in both directions over a 4-wire digital path
(Appendix I: ISDN or T1/E1 end to end).  No third-party V.91 implementation
is known to us; the only on-wire reference is the text, so the way to
validate 6.3 without a peer is a spec-only decoder in the style of
`tools/v90_trn2d_spec_decode.py`, run on our own transmit tap.
