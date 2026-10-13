# V.92 Quick Connect: short Phase 1, digital answerer

Reference: `ITU Docs/T-REC-V.92-200011-I!!PDF-E.pdf`, 8.2.5 (TONEq), 8.3.1
(ANSpcm), 8.3.6 (QTS), Tables 2-5, 9.2.4 (answer modem is digital modem),
9.2.5 (ODP/ADP bypass); V.250 6.8.7/6.8.8 (`+PQC`, `+PSS`).

## What is implemented

The **digital modem answering a V.8 call** (clause 9.2.4.1). Off by default;
`AT+PQC=1` (or `0`) turns it on, `AT+PQC=3` (the factory value) off, and
`+PSS=2` forces the full startup. It needs V.92 on offer (`AT+MS=V92`), the
digital V.90/V.92 role and a G.711 bearer.

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

## Not implemented

- **Short Phase 2** (9.4, `+PQC=0/2` as V.250 defines them). `+PQC=0` here
  enables short Phase 1 only and `+PQC=2` is ERROR. After a successful short
  Phase 1 the call runs the full Phase 2 (INFO0d bit 26 stays clear).
- **QC2/QCA2** (the V.8bis family, 9.2.4.2) and taking the analogue role in
  Quick Connect (9.2.4.1 "may take on the role of the analogue modem").
- The **analogue caller** side of 9.2.1 (QC1a, TONEq): the scripted peer in
  `v92_qc_engine_test.c` is the only analogue-side implementation.
- **ODP/ADP bypass** (9.2.5) on the live path: the P bit is sent and read, and
  the existing LAPM handling takes the caller's P bit, but no V.42 detection
  phase is skipped here.
- No foreign peer has been qualified. A V.92 analogue modem that sends QC1a
  (the Motorola, USR and Agere captures in `docs/v92_short_phase1_plan.md` do)
  is the next test; the factory `+PQC=3` keeps the working slmodemd flow
  unchanged until that is done.
