# V.92 Quick Connect: short Phase 1 and short Phase 2

Reference: `ITU Docs/T-REC-V.92-200011-I!!PDF-E.pdf`, 8.2.5 (TONEq), 8.3.1
(ANSpcm), 8.3.6 (QTS), Tables 2-5, 9.2.4 (answer modem is digital modem),
9.2.5 (ODP/ADP bypass); V.250 6.8.7/6.8.8 (`+PQC`, `+PSS`).

`+PQC` (V.250 6.8.7) selects what is on: **0** short Phase 1 and 2, **1**
short Phase 1, **2** short Phase 2, **3** neither (the factory value, because
no foreign peer has been qualified). `+PSS=2` forces the full startup. Both
need V.92 on offer (`AT+MS=V92`).

## Short Phase 1: what is implemented

The **digital modem answering a V.8 call** (clause 9.2.4.1), with `+PQC` 0 or
1. It needs the digital V.90/V.92 role and a G.711 bearer.

The chain, in `v92_qc.c` (a pure codeword-in/codeword-out module) and driven
from `modem_engine.c`:

1. V.8 decodes the caller's QC1a while ANSam is still playing. spandsp's
   `v8.c` reports a new `V8_STATUS_V92_QUICK_CONNECT` the moment that frame
   completes (QC1a is followed by CM, but the 9.2.4.1 response does not wait
   for it), gated by the new `v8_parms.v92_quick_connect`.
2. The engine stops feeding V.8 and `v92_qc_t` owns the line:
   QCA1d (Table 5: 70 bits, V.21(H), P = our LAPM offer, LM = ANSpcm level),
   75 ms silence, QTS (128 x 6 symbols at the UQTS the caller's WXYZ selected,
   Table 2), QTS\ (8 x 6), then ANSpcm (Tables 6-10, -12 dBm0) until TONEq.
3. TONEq (980 Hz, 50 ms of 10 ms blocks holding at least 70 % of their energy
   in the 980 Hz bin and at least about -38 dBm0) stops ANSpcm; 75 ms of
   silence follows (600 codewords exactly: the module returns the short
   count, so the silence is not rounded up to an RTP frame), and the engine
   enters Phase 2 as if V.8 had just reported V.90/V.34 (the V.8 result
   handler is called with a synthetic result carrying the caller's QC octet,
   so INFO0d still advertises V.92).
4. No TONEq within 2 s of the end of QCA1d (9.2.4.3): the engine restarts V.8
   (ANSam). A second QC1a on the same call is not answered.

Frame alignment (8.3.6): the first QTS codeword is frame interval 0 and the
digital modem keeps that alignment. `v90_set_tx_frame_phase()` tells the Phase
3 generator which interval each codeword is in, and Sd (whose first symbol is
+W) waits for interval 0.

`make` builds `v92_qc_test` (each signal against the decoders already
validated on captures: strict QCA1d on V.21(H), `v92_detect_qts_sequence`,
`v92_anspcm_table_decode`, TONEq detection, the 2 s limit, Sd alignment) and
`v92_qc_engine_test`, which runs the whole answering engine against a scripted
analogue modem in both laws, with TONEq, without TONEq, and with `+PQC=3`
(the QC1a must be ignored). Both are in `tests/fast.list`.

## Short Phase 1, analogue caller (9.2.1.1, 9.2.1.3)

With `+PQC` 0 or 1 and the analogue role (`ME_V90_ROLE=analogue`), V.8's
caller sends QC1a (Table 2: analogue, QC, P = our LAPM offer, WXYZ = 0101, so
UQTS 70) ahead of CM. When QCA1d (Table 5) comes back, V.8 reports it and the
engine stops CM and goes silent (`v92_qca_t`): once ANSpcm has been heard for
1 s it sends TONEq (980 Hz) for as long as ANSpcm lasts; when ANSpcm has gone
(three 10 ms blocks without it; its phase reversals smear one or two), 75 ms of
silence, then Phase 2 as the analogue role starts it after V.8, with the
received-audio history kept for the digital modem's early INFO0d. No ANSpcm
within 6 s of QCA1d returns to V.8.

The one trap: a digital answerer that has Quick Connect off still answers a
QC1a with the older V.8-embedded QCA octet, ahead of its JM. V.8 therefore
reports QCA1d only when the carrier drops after it (Table 5 ends in ten ONEs
and then silence, 9.2.4.1); a JM preamble following cancels it.

Between two of our engines the whole of Quick Connect (short Phase 1 and 2)
now runs: `engine_pair_test ... --both-at AT+PQC=0` reaches V.92 data mode in
both laws and in V.90 data mode.

## Short Phase 2 (9.4), both roles

With `+PQC` 0 or 2 each modem's INFO0 carries the V.92 capability bit and the
short-Phase-2 request (INFO0d bits 27 and 26, INFO0a bits 26 and 27). When both
sequences carry both bits, and the INFO0 exchange was error-free, the Phase 2
ranging is replaced by one Tone transaction (`v34tx.c`, `v92_short_*`):

- digital: INFO0d, Tone B; once Tone A is detected and Tone B has run 50 ms, a
  Tone B phase reversal, 10 ms more Tone B, silence (9.4.1.1.3);
- analogue: Tone A; on detecting that reversal, 40 ms later its own Tone A
  reversal, 10 ms more Tone A, then INFO1a (9.4.2.1.3/.4);
- digital: on the Tone A reversal, round trip delay = the interval minus 40 ms
  (9.4.1.1.4), silence, then INFO1a and Phase 3.

There is no probing, no L1/L2 and no INFO1d. Two consequences are handled
explicitly: the analogue modem has no INFO1d row to take the upstream carrier
from, so INFO1a's Table 19 bit 33 carries it (the digital side applies it, and
the engine's V.90 INFO1a check accepts raw 32:33 = 2 only in this case); and
the analogue modem picks symbol rate 3200 by default and PCM upstream
(Table 18) when it is configured for it, since the digital modem has said nothing
about that.

Measured on two engines (`engine_pair_test`, analogue role against digital):
Phase 2 from the first INFO0 to the start of Phase 3 takes 0.46 s against 1.78 s
for the full procedure, in V.92 PCM-upstream mode (48000/56000), V.90 data mode
and A-law.

Timers and recovery (9.4.1.2.2/.3): if the Tone A reversal or INFO1a has not
arrived 2500 ms after the digital modem's reversal, or Tone A has not appeared,
the digital modem sends Tone B and the full procedure continues; the analogue
modem gives up on the digital modem's reversal after 5 s and does the same. Either
way the full Phase 2 is then kept for the rest of the call. `V92_SHORT_TEST_NO_REPLY=1`
makes the analogue modem ignore the reversal so that path runs end to end
(`tests/fast.list`). A retrain omits INFO0 and never starts a short Phase 2.

Not done: the INFO0 error-recovery loops (9.4.1.2.1: repeated INFO0, bit 28) do
not lead into the short procedure, they leave the call on the full one; and
nothing here has met a foreign V.92 modem.

## Not implemented (either procedure)

- **QC2/QCA2** (the V.8bis family, 9.2.1.2 and 9.2.4.2) and the cases where
  the answerer is an analogue modem (QCA1a, Figures 7/8, 9.2.1.1/9.2.1.4), or
  the digital answerer takes the analogue role (9.2.4.1).
- **ODP/ADP bypass** (9.2.5) on the live path: the P bit is sent and read, and
  the existing LAPM handling takes the caller's P bit, but no V.42 detection
  phase is skipped here.
- No foreign peer has been qualified. A V.92 analogue modem that sends QC1a
  (the Motorola, USR and Agere captures in `docs/v92_short_phase1_plan.md` do)
  is the next test; the factory `+PQC=3` keeps the working slmodemd flow
  unchanged until that is done.
