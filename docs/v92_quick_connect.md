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

## QC2 / QCA2: Quick Connect started by V.8bis (9.2.1.2, 9.2.4.2)

With `ME_V8BIS=1` and `+PQC` 0 or 1 the same sequences start from V.8bis. The
digital modem's CRe is answered by the analogue modem's **QC2a**, a V.8bis-framed
message on V.21(H) (Table 3: first octet 0x2D = type 1011 and revision 0100 in
transmission order, then WXYZ, P, QC, analogue); the digital modem stops CRe and
answers with **QCA2d** on V.21(L) (Table 12: LM, P, QCA, digital), 75 ms of
silence, QTS, QTS\, ANSpcm, and from there it is the sequence above.

- `v8bis_modem` (`cfg.qc2`) hands a type-1011 frame to the engine as an event
  instead of to the V.8bis state machine, and holds a CRe that finds a station in
  the Initial State instead of answering it. `v8bis_modem_send_frame()` puts a
  frame in place of whatever is queued; `v8bis_modem_release_cre()` gives the held
  CRe to the state machine after all.
- The analogue modem waits 1 s after QC2a for QCA2d. Without it the CRe is
  released and V.8bis goes on (the `+PQC=3` answerer case), so a caller with
  Quick Connect on costs a V.8bis call against such an answerer one second.
- The digital modem resumes at the silence after QCA2d, less the idle samples
  that block already held (`v8bis_modem_tx_idle_tail`), so the 75 ms is exact.
- If no TONEq arrives in the 2 s after QCA2d the digital modem sends ANSam and V.8
  (9.2.4.1), as for QCA1d.

An ANSam heard first still sends the caller to the QC1a route; QCA2a (an analogue
answerer) and the digital modem taking the analogue role are not handled.

## Both modems analogue (9.2.1.1, 9.2.1.2, 9.2.3; Figures 7 and 8)

`ME_V90_ROLE=analogue` on both ends, `+PQC` 0 or 1. There is no PCM leg, so the
short Phase 1 ends in an ordinary V.34 Phase 2 and the only tone the two need is
ANSam in place of ANSpcm.

- **Answerer** (`v92_qc_start_analogue_answer`): QC1a (or, over V.8bis, QC2a) is
  answered by QCA1a (Table 4) or QCA2a (Table 5), then V.8 sends ANSam after its
  own 75 ms of silence (`v8_parms.ansam_no_delay` drops the 750 ms hold that
  otherwise precedes ANSam). While ANSam plays the TONEq detector listens
  (`V92_QC_WATCH`); a CM goes to V.8 as usual (9.2.3.4). TONEq ends ANSam, then 75
  ms of silence and V.34 Phase 2.
- **Caller** (`v92_qca_start_ansam`): QCA1a/QCA2a makes it stop CM, go silent, and
  send TONEq once ANSam has been heard (200 ms after QCA1a, 1 s after QCA2a, 9.2.1.1
  and 9.2.1.2); when ANSam is gone, 75 ms of silence and V.34 Phase 2.
- The result handler had assumed an analogue caller always meets a digital answerer.
  It now takes the V.34 path when the peer offers only the analogue role, whether
  or not Quick Connect is on; without that the two analogue modems did not
  connect at all.

Not covered: a digital caller (QC1d, QC2d), which this engine never is.

## A digital modem calling (9.2.2; 9.2.3.1 and 9.2.3.3 for the answerer)

The digital caller already sent QC1d with CM whenever V.92 was on offer (octet
0x45). With `+PQC` 0 or 1 it now acts on the reply:

- **QCA1a** (from V.8; Table 4, whose W0XYZ is the UQTS for our QTS) makes it stop CM
  and, after 75 ms of silence, send QTS, QTS\ and ANSpcm until TONEq comes, then 75
  ms of silence and Phase 2 as the V.90 digital role. Over V.8bis, a CRe is answered
  by QC2d (Table 11) and QCA2a (Table 5) plays the same part.
- The analogue answerer answers QC1d / QC2d with QCA1a / QCA2a (WXYZ 0101, UQTS 70) and
  then, silent, waits up to 2 s for ANSpcm for 1 s (9.2.3.3), sends TONEq until it
  stops, 75 ms of silence, Phase 2. A digital answerer that hears QC1d / QC2d now does
  the same ("may take on the role of the analogue modem", 9.2.4.1/.2): `g_qc_role_swap`
  marks the call, the QCA1a/QCA2a reply and the ANSpcm wait are the analogue answerer's,
  and the analogue role is borrowed for the rest of the call (`g_v90a_role_borrowed`, restored
  at the next call), so Phase 2 onward is the analogue modem's. Two default-role engines
  connect after it (V.92 56000/48000).

An analogue-role engine ANSWERING a digital caller runs the real Phase 2 and Phase 3/4:
`g_v90a_answering` makes `start_v34_training` initialise the borrowed SpanDSP engine as the
analogue ("calling") side whatever the call direction (docs/v90_analogue_role.md). The pair tests
for these cases are ordinary CONNECT tests, short Phase 2 included.

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

INFO0 recovery (9.4.1.2.1, 9.4.2.2.1): when the digital modem repeats INFO0d (bit 28 set
after a correct INFO0a) or the analogue modem repeats INFO0a, both then "proceed
according to 9.4.1.1.3" -- the short procedure carries on. `v34tx.c` calls
`v92_short_begin` at the points where the recovery hands back to Tone B (the
acknowledged-exchange exit and the retry-guard exit), when the armed flag and the
received INFO0 bits 26/27 still agree. `V92_SHORT_TEST_INFO0_RECOVERY=1` makes the
digital modem take the recovery once (`tests/fast.list`). Before this the recovery
quietly left the call on the full Phase 2 on the digital side, which the analogue
side then waited out.

Not done: nothing here has met a foreign V.92 modem.
