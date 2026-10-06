# V.8bis engine: scope

Source: V.8bis (08/96) in `ITU Docs/`, plus V.92 (11/2000) clauses 8.2-8.3 and 9.2 for the QC2/QCA2 users. Written 2026-10-06. Nothing here is implemented yet.

## What exists

| Piece | State |
| --- | --- |
| `v8bis_decode.c` / `phase12_decode.c` | Offline signal scan (MRe/MRd/CRe/CRd/ESi/ESr), HDLC message decode, V.92 QC2 identification. Not in `SRCS`. Has a known defect catalogue (`v8bis_decode_improvement_plan.md`). |
| `k56flex_v8bis.c` | A working sample-driven V.8bis state machine: Goertzel tone detector, tone generator, V.21 via SpanDSP `fsk_tx`/`fsk_rx`, HDLC. **K56flex-private**: recovered oscillator constants, two transactions only (CRe/CRd/CL/MS, no ESi, no MR, no CLR), a 14-octet K56 payload. Engine hooks at `me_k56_start_locked()`, rx at `modem_engine.c:10128`, tx at `:13000`. |
| SpanDSP | Has `fsk`, `hdlc`, V.8, V.80. **No V.8bis.** |
| Spec tables | Tone plan, HDLC framing, message types, NPar/SPar coding, transactions 1-13 are all in the PDF. |

So the transport (tones, V.21, HDLC, FCS) is mostly a generalisation of `k56flex_v8bis.c`; the new work is the standard transaction state machine and the information-field codec.

## Proposed module layout

1. `v8bis.c/.h`: the engine, sample-driven like the K56flex one (`rx(amp,len)`, `tx(amp,len)`, state, result). Pure, no engine globals, so it links into test harnesses.
2. `v8bis_ie.c/.h`: information-field codec. Identification field (type, revision, NPar(1) with the V.8 codepoint), SPar/NPar trees (8.2 tree rules, 8.4 tables 5-1..5-3), NS field, length rules (8.6). Both encode and decode. Replaces the decoder's ad hoc parsing.
3. Engine glue in `modem_engine.c`: start before V.8, hand over to V.8 / V.25 / shortened V.8 at the end. Reuse the K56flex hand-off shape (`me_k56_progress_locked()`).
4. `k56flex_v8bis.c` stays as is (it is deliberately non-standard), but shares the tone detector and generator by extraction into a small `v8bis_tones.c`.

## Work items

**A. Signals (clause 7.1)**
- Dual-tone generator and detector for 1375+2002 (initiating) and 1529+2225 (responding), 400 ms, plus segment 2 identifying tone (650/1150/400/1900/980/1650), 100 ms. +/-250 ppm, +/-2% duration.
- The detector must classify on segment 2, tolerate voice and OGM, and cope with two initiating signals less than 0.5 s apart (10.2.1).
- CRe/MRe transmit 12-15 dB below normal (7.1.4), with the higher-power retransmit rule in 10.2.2.
- ES signals: ESi/ESr, where the message preamble is segment 2 (7.2.4).

**B. Messages (7.2)**
- V.21(L) from the initiator, V.21(H) from the responder. 100 ms marking preamble, 2-5 opening flags, 1-3 closing flags, FCS per 7.2.7, bit stuffing, invalid-frame rules (7.2.9).
- Messages: MS, CL, CLR, ACK(1), ACK(2), NAK(1..4). CL immediately followed by MS with no gap (note 1 to Table 7).

**C. Information fields (clause 8)**
- Identification field and NPar(1)/SPar(1)/NPar(2) trees, standard info (modulations, protocols, data-compression, etc. per Tables 5-1..5-3), NS field, segmentation via ACK(2) (9.10, "additional information available").
- Receivers must ignore what they do not understand (8.3.1, 8.2 compatibility note).

**D. Transactions and state machine (clause 9, Figures 14/15)**
- Transactions 1-13 (Table 7). Initiator and responder roles, answering vs calling, the Initial V.8bis State, the 5 s state timeout (9.8), NAK(1) on invalid frames, ACK(1) suppression (9.7), echo-suppressor 1.5 s silence after ESi (9.4).
- Automatic-answer procedures (10.2): >=400 ms silence then CRe/MRe, retransmit rules, 3 s listen, calling-station handling of ANS/ANSam arriving first (exit to V.25/V.8).
- Start-up handoff (9.9): V.8, shortened V.8, V.25. The MS receiver becomes the answer modem regardless of who called (this inverts roles from the SIP call direction and needs thought in the engine, like the V.90 role notes).

**E. Engine integration**
- Config: `ME_V8BIS=0|1|auto`, `AT+MS`/V.250 hook if wanted, role from `g_calling_party`.
- G.711 constraint: all of this is ordinary in-band tone/FSK, so it goes through the same linear-to-codeword path V.8 uses. No new passthrough concerns.
- Must not disturb existing calls: default off, and when on, a peer that does not support V.8bis must fall through to V.8 within a bounded time (the NZ USR, Courier, SmartLink and RasFinder rigs all start V.8 straight away today).

**F. Verification**
- Unit: codec round-trips plus an independent oracle (as with the CRC oracles in `v92_startup_test`): decode our own output with the offline `v8bis_decode.c` and, separately, hand-built frames from the spec's Figure 4.
- Pair test: two engines over G.711 (like `v32bis_engine_pair_test`) covering every transaction in Table 7, both answer/call roles, with delay and jitter, NAK paths, ACK(2) segmentation, and ANS-before-ESi.
- Add the K56flex pair test as a regression so the extraction does not break it.
- Foreign evidence: replay recorded V.92 captures that contain QC2/QCA2 through the new receiver and compare with the offline decoder's calls (the strict-decode plan already lists which captures carry them).
- No hardware yet: the CX93001 at 6004 is the candidate, and it is the only V.92 peer known to want QC2.

## Stage 1 status (2026-10-06)

Done and tested (`v8bis_test`, in `make test`, ~2 s): `v8bis_tones.[ch]`, `v8bis_msg.[ch]`, `v8bis_ie.[ch]`. Not yet linked into `SRCS`. Still to do inside stage 1's remit: nothing; the offline decoder (`v8bis_decode.c`) has NOT been moved onto `v8bis_ie` yet and keeps its catalogued defects, so it remains diagnostic only.

Findings worth carrying into stage 2: the tone detector is robust (see CLAUDE.md for numbers) but only has a 3-block confirmation, so `detect_sample` lands ~60-80 ms into segment 2 -- a responder that must answer inside the preamble window should budget for that; and the framing receiver reports invalid frames by class (7.2.9) so 9.8's NAK(1) rule can be driven directly.

## Stage 2 status (2026-10-06)

Done and tested (`v8bis_fsm_test`, in `make test`, under 0.1 s): `v8bis_fsm.[ch]`, covering Figures 14 and 15, all 13 Table 7 transactions, ACK(1) suppression (9.7), NAK(1)/(2)/(3), ACK(2) segmentation (9.10) and the 9.8 five-second rule. Two layers remain for stage 3: the sample-level modem (tone generator/detector from stage 1, V.21 FSK tx/rx, the 100 ms mark preamble and ES handling) that turns actions into audio and audio into events, and the engine glue. The FSM's action list is that layer's contract: SIGNAL, MESSAGES (with ES and the 1.5 s gap flag), MS_MODE (with who sent MS, the start-up procedure, and whether to send ANS/ANSam next), INITIAL.

Spec readings worth re-checking against a real peer: transaction 6's shape (Figure 14 and Table 7 disagree, Table 7 followed), CLR carrying capabilities, and NAK(1) leaving any state.

## Staging

1. **Stage 1, codec and tones** (A, B, C). Testable with no state machine. Fixes the offline decoder's defect list at the same time, since both can share `v8bis_ie`.
2. **Stage 2, state machine** (D) for transactions 1-3 and 12/13 first (CRe/CRd/CL/MS/CLR, the ones QC2 and normal answer use), then 4-11.
3. **Stage 3, engine glue and pair tests** (E, F), V.8 handoff only; shortened V.8 and V.25 after.
4. **Stage 4, V.92 QC2/QCA2** on top (separate scope): short Phase 1 uses these as the V.8bis messages with V.92 Tables 3/5/12/14 payloads.

## Risks and open questions

- **Spec only, no foreign V.8bis peer in the corpus** except whatever the QC2 captures hold. The K56flex notes record that tick-calibrated constants were guessed; the standard has real numbers, which helps, but timing tolerances between two real stations are unverified.
- **Clause 11 (DTE-DCE, V.25ter Annex A)** is out of scope for the first cut (AT-level control of V.8bis). It matters only if the DTE is meant to drive it.
- **ANSam vs ANS vs V.8bis ordering** on an automatic-answer call: the answerer speaks first, so with `ME_V8BIS` on, V.8 CI timing on the SIP path (the RasFinder hunt group, ringback) interacts. Needs an explicit decision on when the calling side starts V.8bis versus CI.
- **Offline decoder is not a safe oracle**: it has the catalogued defects (dedup by msg_type, reserved-bit rejection, no channel validation). Fix those first or treat it as diagnostic only.
- Size: roughly 2.5-3.5 kLOC (tones ~400, messages ~300, IE codec ~900, state machine ~900, glue ~300) plus tests of similar size. Stage 1 alone is about a quarter of it.
