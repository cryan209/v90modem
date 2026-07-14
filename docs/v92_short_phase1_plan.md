# V.92 Short Phase 1 Rewrite Plan

## Goal

Rework V.92 short-Phase-1 decoding so startup classification is driven by the exact
Recommendation V.92 signal structures and procedures, not by heuristic mixed-signal
interpretation or recovery-biased candidate selection.

The immediate priority is clauses 8.2, 8.3, and 9.2 of
`ITU Docs/T-REC-V.92-200011-I!!PDF-E.pdf`.

## Principles

- Treat `QC1/QCA1` and `QC2/QCA2` as different decode families.
- Accept startup signals only from strict whole-sequence matches.
- Do not let soft/recovery decoding steer primary Phase-1 classification.
- Derive analogue-vs-digital side from Phase 1, not later DIL or Phase 3 evidence.
- Preserve a chronological per-channel Phase-1 event timeline before arbitration.

## Plan

### 1. Audit every short-Phase-1 signal against the exact V.92 tables

For each of:

- `QC1a`
- `QCA1a`
- `QC1d`
- `QCA1d`
- `QC2a`
- `QCA2a`
- `QC2d`
- `QCA2d`

record:

- exact modulation and channel expectations
- exact bit positions and fields
- repeat and trailing requirements
- what the current code path actually decodes
- what is missing, conflated, or incorrect

### 2. Split implementation into two decode families

#### `QC1/QCA1`

Decode as full framed V.21 sequences including:

- ten ones preamble
- `0101010101` sync
- start bit
- exact payload bits
- repeated frame where required
- trailing ones where required

#### `QC2/QCA2`

Decode as V.8bis identification-field signals, not as shortened `QC1`-style frames.

### 3. Make strict whole-sequence validation primary

A short-Phase-1 signal should only be accepted if the whole required structure matches:

- preamble and sync lock
- frame bits at the correct offsets
- repeated field structure matches the table
- trailing ones are present where required
- modulation and channel are consistent

### 4. Downgrade soft/recovery decoding to diagnostic-only

Soft or recovery candidates may be logged, but must not drive:

- startup role classification
- analogue-vs-digital side selection
- stereo short-Phase-1 pairing
- Phase12 short-Phase-1 progression

### 5. Rebuild stereo arbitration from explicit strict `a` and `d` candidates

Instead of one collapsed winner per side, compare:

- best strict analogue candidate per side
- best strict digital candidate per side
- family consistency (`1` vs `2`)
- `QC` vs `QCA` complementarity
- analogue `UQTS`
- digital `LM`

Accept only complementary valid pairings.

### 6. Separate Phase-1 event timeline from startup interpretation

Maintain a chronological per-channel Phase-1 event list for:

- early call/answer tones
- `CRe/CRd`
- V.8/V.8bis messages
- strict short-Phase-1 candidates

Only after that should startup interpretation and stereo reconciliation happen.

### 7. Rework Phase12 short-Phase-1 procedure from clause 9.2

Implement the real procedure branches for:

- call modem is analogue
- call modem is digital
- answer modem is analogue
- answer modem is digital

including:

- `QC1 -> CM`
- `QC2 -> silence`
- `QCA` response handling
- `QTS/QTS\`
- `ANSpcm`
- `TONEq`
- `75 +/- 5 ms` silence
- timeout and fallback behavior

### 8. Only after strict startup works, retune degraded-capture handling

Possible later recovery logic:

- limited rollback for missing first preamble ones
- bounded alignment search around a strict lock
- degraded-capture recovery modes

But these must remain secondary and must not define the primary startup result.

### 9. Validate on QC recordings first

Use QC captures as the primary truth set and verify:

- accepted bit sequences can be printed exactly
- `QC1/QCA1` are not confused with `QC2/QCA2`
- side selection comes from Phase 1 only
- low-energy echo-side candidates do not win

### 10. Use NC recordings only as secondary controls

Treat NC captures as mixed-behavior controls that may legitimately follow V.90 or
normal V.8/V.8bis startup logic.

## Immediate next step

Produce a spec audit table for all eight short-Phase-1 signals:

- spec requirement
- current code path
- observed defect
- required rewrite action

## Status

- Step 1 completed on 2026-04-05.
- Step 2 completed on 2026-04-05.
- Step 3 completed on 2026-04-05.
- Step 4 completed on 2026-04-05.
- Step 6 completed on 2026-04-05.
- Step 7 first stage completed on 2026-07-10 (clause 9.2 procedure evaluator
  over the Phase-1 timeline; see "Step 7 Progress" below).
- Completed work:
  - read clauses 8.2, 8.3, and 9.2 of `ITU Docs/T-REC-V.92-200011-I!!PDF-E.pdf`
  - verified the exact table structures for `QC1a`, `QCA1a`, `QC1d`, `QCA1d`, `QC2a`, `QCA2a`, `QC2d`, `QCA2d`
  - mapped those structures against the current decode paths in:
    - `v92_short_phase1_decode.c`
    - `phase12_decode.c`
    - `v8bis_decode.c`
    - `vpcm_decode.c`
  - identified the specific gaps and conflations listed below
  - added strict `QC1/QCA1` whole-sequence validators in `v92_short_phase1_decode.c`
  - switched the primary Phase12 short-P1 accept path to use strict sequence validation
  - kept soft/recovery `QC1/QCA1` decoding only as secondary diagnostic evidence
  - preserved separate strict analogue and strict digital `QC1/QCA1` candidate sets in Phase12
  - switched stereo arbitration to extract from those strict candidate sets instead of a single
    collapsed short-P1 winner
  - changed strict short-Phase-1 acquisition to search inside detected Phase-1 V.21 burst
    windows first, instead of sweeping the whole post-ANS span
  - merged and pruned short-Phase-1 burst windows by relative signal energy before running the
    strict `QC1/QCA1` scanner
  - kept the strict whole-sequence accept rule unchanged while moving recovery logic further out
    of the startup acquisition path
  - added a chronological internal Phase-1 event timeline in `phase12_decode`
  - preserved pre-ANS `V.8bis`/`V.92` message events such as `CRe/CRd/QC2/QCA2` instead of
    collapsing them immediately into one summary winner
  - added timeline entries for tones, `CM/JM/CJ`, and strict V.92 short-Phase-1 detections
  - sorted that timeline chronologically and exposed it in the raw `vpcm_decode --phase12`
    output path

## Step 4 Outcome

### What changed

- `detect_v92_short_phase1()` now takes the per-channel Phase-1 V.21 burst windows and searches
  those windows first for strict `QC1/QCA1` sequences.
- Short-Phase-1 windows are merged with the existing Phase-1 repeat gap and pruned with a
  relative signal-energy cutoff before the strict scanner runs.
- Full-span scanning is now only the fallback when there are no usable Phase-1 burst windows.

### Why this matters

This keeps startup classification aligned with real observed Phase-1 FSK energy and reduces the
chance of locking a strict sequence onto mixed or empty post-ANS regions.

### Current observation

On `Agere-SV92-QC.wav`, the debug run now shows the strict search staying inside actual Phase-1
burst windows instead of sweeping the whole post-ANS interval. That improves acquisition hygiene,
but it also confirms the next remaining problem: we still need a cleaner chronological Phase-1
event timeline to separate early tone, `CRe/CRd`, V.8/V.8bis, and short-Phase-1 events.

## Updated Next Step

Proceed to Step 7:

- rework the Phase12 short-Phase-1 procedure against clause 9.2 using the cleaner event timeline
- stop deriving startup state from mixed summary flags when the ordered event story disagrees
- make `QC1 -> CM`, `QC2 -> silence`, `QCA` response handling, and short-P1 follow-up transitions
  operate on explicit timeline/state progression

## Step 6 Outcome

### What changed

- `phase12_result_t` now carries a fixed-size chronological Phase-1 event list.
- `detect_call_initiation_signals()` now preserves individual `V.8bis` and `V.92` message events
  from the temporary call log instead of only keeping one best pre-ANS summary.
- `phase12_decode_phase1_with_codewords()` now builds a sorted timeline covering:
  - Phase-1 tones
  - pre-ANS `V.8bis` / `V.92` message events
  - `CM/JM/CJ`
  - strict V.92 short-Phase-1 detections
- `vpcm_decode --phase12` now prints that `Phase 1 timeline` section in raw output mode.

### Why this matters

This separates the actual observed startup chronology from later role/arbitration logic, which is
the groundwork we need before rewriting the clause 9.2 procedure handling.

### Verification

- `make vpcm_decode` passes.
- A full capture-based verification run is still pending; the Agere QC file takes long enough that
  the bounded CLI run used here did not complete before timeout.

## Step 1 Audit

### Summary

Current code uses two different families:

- `QC1/QCA1`:
  decoded by `v92_decode_short_phase1_candidate()` in `v92_short_phase1_decode.c`
- `QC2/QCA2`:
  decoded as V.8bis identification fields by `v92_decode_qc2_id()` in `v8bis_decode.c`

This split is directionally correct, but the implementation still fails to preserve several
normative requirements from the tables and procedures:

- `QC1/QCA1` is still effectively frame-centric instead of sequence-centric
- `QCA1a/QCA1d` trailing ten-one requirement is not enforced
- `QC1a/QC1d followed immediately by CM` is not enforced as a signal property
- `QC2/QCA2` is currently reduced to a parsed message event and loses timing/channel
  semantics when handed to Phase12
- Phase12 later treats `QC1` and `QC2` as interchangeable anchors too early

### Audit Table

| Signal | Spec structure | Current code path | Observed defect | Required rewrite action |
| --- | --- | --- | --- | --- |
| `QC1a` | V.21(L), full 60-bit sequence: ten ones, sync, start bit, analogue/QC/P, `W0XYZ1`, repeated ten ones, repeated sync, repeated payload `000PW0XYZ1`; transmitted once and followed immediately by `CM` | `v92_decode_short_phase1_candidate()` unpacks frame bits `20:29` and optional repeated bits `50:59`; scanner in `phase12_decode.c` scores preamble/sync separately | Decoder helper accepts based on frame fields and optional repeat, not as a single exact 60-bit object; `followed immediately by CM` not enforced in decode; only later analogue-chain check loosely looks for `CM` within `1200 ms` | Build strict `QC1a` whole-sequence validator including exact repeat form and explicit immediate `CM` rule |
| `QCA1a` | V.21(H), full 70-bit sequence: same framing, payload `W0XYZ1`, repeated payload `001PW0XYZ1`, plus trailing ten ones at `60:69` | Same `QC1/QCA1` helper and scanner | Trailing ones at `60:69` are not checked; minimum accepted length can be `30` or `60`, which is insufficient for the normative `QCA1a` structure | Add strict `QCA1a` validator requiring the trailing ten ones |
| `QC1d` | V.21(L), full 60-bit sequence: ten ones, sync, start bit, digital/QC/P, `000LM1`, repeated ten ones, repeated sync, repeated payload `010P000LM1`; transmitted once and followed immediately by `CM` | Same `QC1/QCA1` helper and scanner | Same frame-centric reduction as `QC1a`; immediate `CM` rule is not enforced for digital form; Phase12 may accept `QC1d` without tying it to the `CM` part of the procedure | Build strict `QC1d` whole-sequence validator including exact repeat form and explicit immediate `CM` rule |
| `QCA1d` | V.21(H), full 70-bit sequence: same framing, payload `000LM1`, repeated payload `011P000LM1`, plus trailing ten ones at `60:69` | Same `QC1/QCA1` helper and scanner | Trailing ones at `60:69` are not checked; accepted as if a generic 30/60-bit frame decode were sufficient | Add strict `QCA1d` validator requiring trailing ten ones |
| `QC2a` | V.8bis signal structure with identification field: message type `1011`, revision nibble, `WXYZ`, reserved bit 12 `0`, `P` at bit 13, `QC` at bit 14, analogue at bit 15 | `v92_decode_qc2_id()` in `v8bis_decode.c`, then event collection into `phase12_decode.c` temporary log | Field parsing is correct at the ID-octet level, but Phase12 only stores the first matching event and loses the richer V.8bis context; `fsk_ch` and exact start-up sequencing are not consumed later | Keep QC2 decode as a V.8bis family, but carry full context into Phase12 rather than collapsing to a name plus payload value |
| `QCA2a` | V.8bis identification field like `QC2a`, but bit 14 `1` for `QCA`, bit 15 `0` for analogue | Same V.8bis path | Same issue as `QC2a`; the message is parsed but then treated as a generic anchor with little procedural structure attached | Preserve V.8bis timing/channel context and use it in the 9.2 procedure state machine |
| `QC2d` | V.8bis identification field: message type `1011`, revision nibble, `LM` in bits `8:9`, bits `10:12 = 000`, `P`, `QC`, digital modem bit set | Same V.8bis path | Field parsing is correct, but later Phase12 behavior still treats QC2 as mostly just another short-P1 label and not as a distinct V.8bis-start branch with its own silence/timeout behavior | Carry QC2d as a distinct procedural branch in Phase12 |
| `QCA2d` | V.8bis identification field like `QC2d`, but bit 14 `1` for `QCA`, bit 15 `1` for digital | Same V.8bis path | Same issue as `QC2d`; the parsed message survives only as summary metadata, not as a strict procedure state | Carry QCA2d as a distinct procedural response in Phase12 |

### Detailed Findings

#### 1. `QC1/QCA1` path is still frame-centric

Current helper:

- `v92_decode_short_phase1_candidate()` in `v92_short_phase1_decode.c`

What it does:

- packs bits `20:29` into `frame1_bits`
- optionally packs bits `50:59` into `frame2_bits`
- decodes those frame fields into analogue/digital, `QC/QCA`, `LAPM`, `WXYZ`, `LM`

What it does not do as a single strict object:

- require the exact full normative sequence length per signal
- require that accepted `QCA1a/QCA1d` candidates include trailing ones `60:69`
- treat `QC1/QCA1` as one indivisible locked sequence

The surrounding scanner in `phase12_decode.c` does score:

- ten ones at `0:9`
- sync at `10:19`

but the overall architecture still separates “lock scoring” from “frame meaning” too much.

#### 2. `QCA1a/QCA1d` trailing ten-one field is not enforced

Spec requires:

- `QCA1a` has trailing ten ones at `60:69`
- `QCA1d` has trailing ten ones at `60:69`

Current code:

- minimum length check is `bit_len >= 30`
- repeat handling only looks at `50:59`
- no explicit validation of `60:69`

This is a clear normative gap.

#### 3. `QC1a/QC1d followed immediately by CM` is not encoded as a strict startup property

Spec requires:

- `QC1a` is transmitted once and followed immediately by `CM`
- `QC1d` is transmitted once and followed immediately by `CM`

Current code:

- analogue side only: `finalize_v92_analog_short_phase1()` treats `QC1a -> CM` as valid
  if a `CM` appears within roughly `1200 ms`
- digital side: no equivalent strict `QC1d -> CM` requirement is enforced

This is much looser than “followed immediately by CM”.

#### 4. `QC2/QCA2` field parsing is better, but Phase12 still collapses it too early

Current V.8bis QC2 helper in `v8bis_decode.c` correctly distinguishes:

- `QC2a/QCA2a`
- `QC2d/QCA2d`
- analogue `WXYZ/UQTS`
- digital `LM`

But in `phase12_decode.c` the result is reduced to:

- `name`
- `sample`
- `digital`
- `qca`
- `uqts_ucode`
- `lm_level`

and only the first matching event is kept.

That loses:

- richer message context
- exact V.8bis channel metadata
- repeated or competing observations

#### 5. Phase12 follow-up still treats `QC1` and `QC2` as interchangeable anchors too early

Current follow-up logic in `detect_v92_short_phase1_followup()` chooses anchor sample from:

- `v92_short_p1_sample`
- else `v92_qc2_sample`

and then derives:

- effective `UQTS`
- effective `LM`

This is useful as a temporary bridge, but it is not equivalent to the clause 9.2 branch logic.
`QC1` and `QC2` are different startup procedures and should not be reduced to “same follow-up
anchor with different payload source”.

### Concrete conclusions from Step 1

- The current split between `QC1/QCA1` and `QC2/QCA2` is conceptually correct.
- The `QC1/QCA1` implementation is missing exact whole-sequence enforcement.
- The `QC2/QCA2` implementation is missing full procedural integration.
- The most obvious table violation today is missing enforcement of the `60:69` trailing ones in
  `QCA1a/QCA1d`.
- Another obvious procedure violation is that `QC1a/QC1d followed immediately by CM` is not
  enforced tightly enough.

## Next Step

Step 2 should start by replacing the generic `QC1/QCA1` framed helper with four strict validators:

- strict `QC1a`
- strict `QCA1a`
- strict `QC1d`
- strict `QCA1d`

and only then re-thread those strict results into the stereo and Phase12 startup flow.

## Step 2 Completion

### Completed

Implemented strict whole-sequence validation for the `QC1/QCA1` family in
`v92_short_phase1_decode.c`:

- strict `QC1a`
- strict `QCA1a`
- strict `QC1d`
- strict `QCA1d`

The strict validator now requires:

- ten ones at `0:9`
- sync `0101010101` at `10:19`
- first 10-bit frame at `20:29`
- repeated ten ones at `30:39`
- repeated sync at `40:49`
- repeated 10-bit frame at `50:59`
- trailing ten ones at `60:69` for `QCA1a/QCA1d`

The primary short-P1 accept path in `phase12_decode.c` now uses this strict whole-sequence
validator instead of the generic frame-only helper.

Soft `QC1/QCA1` decoding remains present only as diagnostic evidence and does not drive the
primary startup winner path.

### Verification

- `make vpcm_decode` passes
- strict debug run on `Agere-SV92-QC.wav` shows the accepted primary winners are now strict:
  - `QC1a` on one side
  - `QCA1a` on the other side
- soft digital candidates are still visible in debug output, but they no longer steer startup
  classification

### What Step 2 did not solve

Step 2 intentionally tightened the `QC1/QCA1` path only. It did not yet:

- rework `QC2/QCA2` beyond their existing V.8bis parsing path
- rebuild stereo arbitration from separate strict analogue and strict digital candidate sets
- solve the remaining Phase-1 interpretation problem on QC captures where strict analogue
  sequences still beat soft digital candidates

### Updated Next Step

The next implementation step remains Step 3 in the working sequence, but its content is the idea
originally described in Step 5, brought forward because Step 2 showed that strict analogue
`QC1/QCA1` winners still dominate and a single collapsed winner per side is not sufficient.

So the new Step 3 should rebuild stereo arbitration from explicit strict candidate sets:

- best strict analogue `QC1/QCA1` candidate per side
- best strict digital `QC1/QCA1` candidate per side
- existing `QC2/QCA2` V.8bis candidates per side

and choose only complementary family-consistent pairings without allowing soft recovery paths to
define the startup result.

If additional work is still needed after that, Step 4 should continue the remaining work from the
original Step 3 area, namely the digital short-Phase-1 chain and its procedural integration once
stereo pairing is based on the correct strict Phase-1 candidates.

## Step 3 Completion

### Completed

Implemented the brought-forward Step 3 work described from the original Step 5 idea:

- Phase12 now preserves separate strict analogue and strict digital `QC1/QCA1` candidate sets
- stereo arbitration now extracts from those explicit strict sets instead of depending on a single
  collapsed short-P1 winner per side
- soft/recovery candidates are not used for stereo short-P1 pairing

This work was implemented in:

- `phase12_decode.h`
- `phase12_decode.c`
- `vpcm_decode.c`

### Verification

- `make vpcm_decode` passes
- on `Agere-SV92-QC.wav`, the stricter candidate-set path no longer promotes a short-P1 pair from
  recovery-biased candidates

### Observed outcome

The immediate effect of Step 3 is that the previous apparent short-P1 pair on the Agere QC file
disappears under strict candidate-set handling:

- no accepted strict short-P1 result is retained in the final report
- stereo arbitration falls back to `tie`
- V.92 startup classification is no longer being forced by weak or recovery-only candidates

This is a useful result because it shows the decoder is no longer manufacturing a complementary
pair from degraded evidence. It also means the remaining work is now squarely in the Phase-1
acquisition and interpretation path, not in later stereo collapsing.

### Updated next step

Step 4 should continue the original Step 3 area, now on top of the stricter startup foundation:

- improve Phase-1 acquisition only enough to recover true strict candidates
- keep recovery paths diagnostic-only
- continue with the digital short-Phase-1 chain and its procedural integration only after strict
  Phase-1 candidates can be recovered reliably

## Step 7 Progress (2026-07-10)

### What changed

Implemented the first stage of the clause 9.2 procedure rework: an explicit
timeline-driven procedure evaluator instead of mixed summary flags.

- `phase12_decode.h` gained `p12_v92_proc_result_t`: an ordered step trace
  (`p12_v92_proc_step_t` with per-step clause reference, signal name, timing
  status `observed/late/missing/unobservable`, sample and note), a matched
  figure (Figures 3–8/V.92), the call/answer side forms, a terminal outcome
  (`short-phase2`, `v34-phase2`, `v8-fallback`, `v8bis-fallback`,
  `incomplete`), and a Phase 2 handoff sample.
- `p12_eval_v92_clause92_procedure()` in `phase12_decode.c` walks the sorted
  Phase-1 event timeline and matches the 9.2.1–9.2.4 branch structure:
  - start condition (1 s ANSam for `QC1`, 50 ms CRe for `QC2`)
  - `QC1 -> CM` "followed immediately" with an explicit immediate window
    (QC1 end + 150 ms) and a graded late window instead of the old flat
    1200 ms accept
  - `QCA` response with the normative 1 s bound for `QCA2` and a CM-repeat
    window for `QCA1`
  - digital-side chain `75 ± 5 ms silence -> QTS/QTS\ -> ANSpcm`
  - analogue-side `TONEq` and the `75 ± 5 ms` silence to Phase 2 handoff
  - fallbacks: `JM` after `QC1` without `QCA1` resolves to V.8; missing
    `QCA2` resolves to V.8bis (or V.8 when plain ANS follows `QC2`);
    Figures 7/8 (both analogue) resolve to V.34 Phase 2
- `p12_reconcile_v92_proc()` makes the ordered story primary: a complete
  short-phase2 story supplies the Phase 2 handoff when the legacy chain did
  not, and a fallback story clears stale `v92_digital_chain_valid` /
  handoff summary flags.
- The Phase-1 timeline now also carries `QTS`, `ANSpcm` and `TONEq` events,
  so the follow-up chain is part of the ordered event story.
- Fixed a real ordering defect: `finalize_v92_analog_short_phase1()` and
  `detect_v92_short_phase1_followup()` select their procedure branch from
  timeline events, but the timeline used to be built only at the very end of
  Phase-1 decode — so strict `QC1/QCA1` and `CM` events were invisible to
  branch selection (the `QC1 -> CM` rule could never fire, and only
  `QC2/QCA2` V.8bis-derived events could anchor a branch).  The timeline is
  now built before branch selection, with the false-CM ANS-window discard
  moved ahead of it.
- Initialised `v92_short_p1_uqts_ucode` and `stereo_short_p1_partner_sample`
  to `-1`; both were memset-zero and passed `>= 0` guards, silently masking
  the QC2/partner UQTS fallback in the follow-up chain.
- `vpcm_decode --phase12` raw output prints the new procedure trace
  ("V.92 clause 9.2 procedure" section) with per-step clause references and
  the terminal outcome.

### Verification

- `make vpcm_decode` passes.
- Baseline-vs-new output diff on five captures (`Agere-SV92-QC`,
  `Agere-SV92-NC`, `Motorola-SM56-V92QC`, `Motorola-SM56-V92NC`,
  `USR-Message-V92NC`): identical except the new `TONEq` timeline events.
  No startup classification changed.
- The evaluator itself was exercised with a synthetic-timeline harness
  (scratchpad-only, includes `phase12_decode.c` directly) covering:
  complete Figure 3 (outcome short-phase2, handoff = TONEq end + 75 ms,
  reconcile supplies the handoff), `QC1d`+JM without QCA (v8-fallback and
  stale chain flags cleared), `QC2a` without QCA2 (v8bis-fallback), late CM
  (step graded late, outcome incomplete), and Figure 7 both-analogue
  (v34-phase2).  All checks pass.

### What Step 7 still needs

- On the QC captures the evaluator currently has nothing to anchor on
  because strict `QC1/QCA1` acquisition still fails there (the known Step 4
  remainder).  Once acquisition recovers true strict candidates, the
  procedure trace will light up on real captures and the remaining 9.2
  branch behaviours (QCA response transmission windows, `QTS/QTS\` gap
  verification against the 768T/48T durations, ODP/ADP bypass in 9.2.5)
  should be tightened against them.
- Phase 2 interpretation should eventually consume `v92_proc.outcome`
  directly instead of the reconciled legacy flags.

## Step 4 Continuation — Strict Acquisition Unblocked (2026-07-10)

### Root cause of the acquisition failure

The strict scanner was never shown the QC1/QCA1 audio:

1. `detect_phase1_v8()` deleted every CH1 FSK burst overlapping the ANS
   tone window ("the caller never sends CM during ANS") before the short-P1
   scan ran — but per Figures 3/4 of V.92 the call modem transmits QC1
   followed by CM *while* the answerer's ANSam is still playing.  The QC1a
   burst always overlaps ANS and was always discarded.
2. The "ANSam" procedure window (ANS start + 950 ms) only opened when the
   15 Hz AM classifier fired.  Real V.92 QC captures routinely classify as
   `ANS/PR`, so the window that covers QC1a never existed and only the CRe
   window (which ends long before QC1a) was scanned.
3. The scan stopped after the first channel with a primary winner, so the
   answer side's QCA on CH2 was never scanned when CH1 found the QC.

The earlier "near-miss QCA1d" diagnostics were red herrings — bit-level
dumps show they were substrings of the JM message body, not degraded
QCA1d sequences.

### What changed

- Short-P1 CH1 scanning uses a pre-filter snapshot of the CH1 bursts; the
  ANS-overlap filter itself now only drops bursts *contained in* the ANS
  window (bursts extending >= 200 ms past ANS end are real signal), which
  also lets the CM immediately following QC1a decode.
- The ANSam procedure window opens for any detected answer tone >= 1 s
  (strict sequence validation protects the accept path) and extends 1 s
  past the apparent tone end.
- Both V.21 channels are always scanned; a second channel's strict
  candidates merge into the per-form strict slots without replacing the
  primary winner.
- A decoded CM/JM whose start falls inside a strict short-P1 sequence span
  is dropped as a misread (a QCA1d body reliably decodes as a bogus JM
  with nonsense fields).
- All saved CM/JM observations go on the Phase-1 timeline (the summary hit
  keeps only the most recent decode, which hid the CM that immediately
  follows QC1a behind a later V.8-retry CM).
- The ans-end fallback TONEq detector rejects hits overlapping a strict
  QC1+CM span — TONEq and the V.21 CH1 mark are both 980 Hz.
- Role detection and stereo cross-channel resolution use the strict
  short-P1 form (QC* = call side, QCA* = answer side) ahead of tone
  heuristics.
- Clause 9.2 evaluator refinements: the stereo partner's form is inferred
  as the complement of this side's expected form; the ANSam start
  condition accepts a >= 1 s answer tone with an "AM not classified" note;
  and a new terminal outcome implements the 9.2.3.3/9.2.4.3 TONEq-timeout
  fallback (QCA observed, no QTS/ANSpcm/TONEq chain, and a CM/JM restart
  >= 1.5 s after the QCA end resolves to v8-fallback with a "V.8 retry
  after TONEq timeout" step).

### Verification (capture truth set)

- `Motorola-SM56-V92QC`: strict QC1a at 5286.5 ms (WXYZ=7 → UQTS 74) on
  the right/caller channel and strict QCA1d (LAPM=1, LM=-15 dBm0) at
  5978.0 ms on the left/answerer channel; QC1 -> CM verified at 5519.4 ms
  ("immediately follows QC1"); both channels agree on Figure 3; stereo
  arbitration reports the complementary pair and correct sides
  (Right=analog caller, Left=digital answerer).
- `Motorola-SM56-V92NC`: strict pair QC1a 4911.8 ms / QCA1d 5591.2 ms.
- `USR-Message-V92NC`: strict pair QC1a 5364.8 ms / QCA1d 6038.9 ms.
  (The NC captures legitimately contain short Phase 1: the exchange runs,
  times out without QTS/ANSpcm/TONEq, and the late CM matches the
  9.2.4.3 V.8 retry — the evaluator resolves all three captures to
  v8-fallback with an auditable step trace.)
- What used to be reported on these files as "JM" with junk fields
  (pcm=1/7, 272 ms) was bit-verified to be the QCA1d sequence; it is now
  decoded strictly and the bogus JM is dropped.
- `Agere-SV92-QC` / `Agere-SV92-NC`: no strict QC1/QCA1 (the Agere call
  starts from CRe, i.e. the QC2/V.8bis family — its procedural decode is
  the remaining gap); outputs unchanged apart from the richer CM
  observation timeline.
- Synthetic clause 9.2 evaluator tests still pass.

### Remaining

- QC2/QCA2 (CRe-family) procedural integration for the Agere captures.
- QTS/ANSpcm detection requires G.711 codewords; WAV-only decodes cannot
  confirm the digital-side chain, so a completed quick connect currently
  reads as "incomplete" unless TONEq+ANSpcm are visible.

## QC2/QCA2 (CRe Family) Integration (2026-07-10)

### Root causes of the QC2 decode gap

1. **Wrong bit conventions in the ID-field parser.**  Tables 12/13/V.92
   write field bit sequences in TRANSMISSION order, while HDLC octets are
   assembled LSB-first.  The wire sequence "1011" (message type) packs to
   0xD, not 0xB; "WXYZ" packs W into octet bit 0 (the UQTS table indexes W
   as MSB); "LM" packs L into bit 0 (level code is 2L+M).
   `v92_decode_qc2_id()` gated on `msg_type == 0xB` and read WXYZ/LM
   unreversed, so no real QC2/QCA2 frame could ever pass.  Bit-verified
   against a CRC-valid QC2a capture frame (octets `2D 29`: type 1011,
   rev, WXYZ=1001 → UQTS 78, P=1, QC, analogue).
2. **Type-0xD frames were skipped as "unknown"** in
   `v8bis_collect_msg_events()` before the QC2 parse ran.
3. **The SpanDSP FSK path missed some frames entirely** (the Agere caller's
   QC2a on CH2 produced no bits at all), the same acquisition failure class
   as the QC1 story.

### What changed

- Fixed the ID-field bit order in `v92_decode_qc2_id()` and recognized
  QC2/QCA2 frames before the unknown-type skip.
- New `v8bis_scan_qc2_bitstream()`: HDLC deframe (flags, destuffing,
  CRC-16 residual check) of an already-demodulated 300 bit/s stream,
  returning only CRC-valid QC2/QCA2 identification frames.
- The phase12 short-P1 scanner now runs that deframe once per demod
  alignment (invert x bit-rate x clock-phase), so QC2/QCA2 acquisition
  gets the same aligned-block robustness as QC1/QCA1; CRC-valid hits are
  recorded into `call_init` (fill-if-unseen), the timeline, and the
  V.8-misread drop spans (a QC2a HDLC body reliably misdecodes as a bogus
  JM, as on the Agere right channel).
- Two-stage procedure support: per 9.2.1.2/9.2.2.2, ANSam after the QC2
  exchange moves the procedure to 9.2.1.1.  When both families appear in
  one call (the Motorola captures do exactly this), family 1 is the
  operative story and the evaluator emits the CRe/QC2/QCA2 exchange as
  prologue steps; `p12_phase1_find_short_p1_event()` prefers family-1
  anchors so branch selection and stereo pairing are not stolen by the
  superseded first stage.
- Stereo arbitration: the per-side signal extractor now also surfaces
  QCA2x candidates, a family-2 CRC-verified pair no longer requires the
  family-1 `analog_ready` gate, and QC2/QCA2 evidence weighs into role
  resolution (+/-15) alongside strict short-P1 (+/-20).
- The clause 9.2 evaluator accepts CRe or CRd as the family-2 start
  condition, gates partner hints on family match, and invalidates the
  Phase 2 handoff estimate for fallback outcomes; the ans-end TONEq
  fallback also rejects hits with a CM/JM observation starting inside the
  tone span (the 980 Hz run is the message's mark preamble).

### Verification (capture truth set)

- `Agere-SV92-QC`: both channels now tell the complete Figure 5 story —
  CRe (1140/1260 ms) -> QC2a at 1973.2 ms (CRC-valid, UQTS 78, LAPM) on
  the right/caller channel -> QCA2d at 3093.2 ms (LM -12 dBm0) on the
  left/answerer channel -> no QTS/ANSpcm/TONEq -> CM at ~4.9 s = V.8
  retry -> v8-fallback.  Stereo pair `family=2 qca_opposed=yes`, roles
  Right=analog(caller)/Left=digital(answerer).  The QCA2d was decodable
  by the old path all along and rejected only by the 0xB gate; the QC2a
  needed the new aligned bitstream scan.
- `Motorola-SM56-V92QC/V92NC`: now reveal a two-stage short Phase 1 —
  CRe/CRd + QC2a (UQTS 74) + QCA2d prologue, then ANSam -> QC1a -> CM ->
  QCA1d -> TONEq timeout -> V.8 retry.  Family-1 pairing, figure 3 and
  roles all preserved.
- `USR-Message-V92NC`: unchanged single-stage figure 3 story.
- `Agere-SV92-NC`: byte-identical output (plain V.8 call, no QC content).
- Synthetic clause 9.2 evaluator tests still pass.

## Waveform-Structure QTS/ANSpcm Confirmation (2026-07-10)

### Why the codeword detectors could not fire on line captures

The codeword-exact `v92_detect_qts_sequence()` / `v92_detect_anspcm_sequence()`
assume the G.711 stream is the DS0 tap the digital modem emitted.  On an
analog line capture the signal passed the D/A, line and A/D: bit-level
inspection of the Motorola capture shows the QTS arriving as a perfectly
stable 6-sample pattern `-64 -33 +61 +64 +33 -60` — the channel filter
reshapes the nominal `[+u, +0, +u, -u, -0, -u]` template beyond any
per-symbol ucode tolerance.  What survives an LTI channel is structure:

- QTS: 6-sample periodic and antisymmetric at 3 samples (`x[k+3] = -x[k]`,
  only odd harmonics of 1333 Hz), QTS\ a polarity flip;
- ANSpcm: digitally generated 2099.7 Hz tone with an exact 301-sample
  period (adjacent-period correlation measures +1.000 on the capture),
  phase reversal every 3612 samples.

### What changed

- New `v92_detect_qts_waveform()` and `v92_detect_anspcm_waveform()` in
  `v92_short_phase1_decode.c` implement those structure tests on the
  linear waveform.  The ANSpcm detector estimates the true lag (+/-2
  samples, resampled-capture clock offset) and tracks drift; the phase
  reversal lands mid-window and destroys two adjacent period pairs, so up
  to two tentative misses are carried and rolled back.  Levels cannot be
  measured from a gain-unknown capture, so `out->level` carries the LM
  hint from the QCA1d/QC2 decode.
- `detect_v92_short_phase1_followup()` runs the codeword-exact detectors
  first and falls back to the waveform detectors; the QTS waveform path
  also runs when no UQTS value is known (the structure test needs none).
- The clause 9.2 TONEq-timeout fallback now fires whenever TONEq is
  missing after the QCA — even when the digital side did transmit its
  QTS/ANSpcm chain — since a missing TONEq answer aborts the quick
  connect either way (9.2.4.3).
- `v92_anspcm_level_to_str()` guards negative (unknown) levels.

### Verification

- `Motorola-SM56-V92QC` left/digital channel: QTS observed at 6313.6 ms
  with 777 samples — the nominal 768T — and ANSpcm at 6413.0 ms for
  1730.8 ms, ending right at the 2 s post-QCA1d timeout that precedes the
  V.8 retry CM at 8226.8 ms.  The digital-side chain is now fully
  confirmed: `QCA1d -> QTS -> ANSpcm -> (TONEq missing) -> CM retry`,
  outcome v8-fallback with only TONEq missing.
- `Motorola-SM56-V92NC`: QTS 777 samples at 5926.8 ms, ANSpcm 1655.5 ms.
- `USR-Message-V92NC`: QTS 777 samples at 6374.4 ms, ANSpcm 903.0 ms —
  a different vendor producing the same 768T QTS confirms the detector
  locks the real signal.
- No false hits on the Agere captures (their QC2 stage aborted before any
  chain ran) or the plain-V.8 Agere NC; synthetic evaluator tests pass.

### Remaining after this step

- The QTS\ polarity flip is not being separated (qts_bar_reps=0): the
  antisymmetry run ends at ANSpcm start, so QTS\ either overlaps the
  measured 777 samples or the flip transient ends the run early.
  Diagnostic only.
- A successful quick connect (TONEq answered, Phase 2 short startup)
  still has no capture in the truth set; the short-phase2 outcome remains
  verified by synthetic tests only.
- ODP/ADP bypass (9.2.5) not yet modeled.

## ANSpcm-Anchored V.8 Retry Search (2026-07-14)

### The bug

The clause 9.2 terminal-outcome branch searched for the post-timeout V.8
retry CM/JM starting at a fixed `qca_end + 1500 ms` guard.  On
`USR-Message-V92QC` the digital channel's retry CM lands at 8421.8 ms while
the guard computed 8445 ms — missed by 23 ms — so the channel's outcome
degraded to `incomplete` while the analog channel (whose own retry CM at
8465.2 ms cleared the same guard by just 20 ms) said `v8-fallback`: two
contradictory stories for one call.

### What changed

- The retry search anchor is now derived from the observed signal instead
  of a fixed offset: a CM cannot be the V.8 retry while the digital side is
  still transmitting its continuous ANSpcm (9.2.4.3), so when the ANSpcm
  was observed on this channel the search starts exactly at its end.  The
  `qca_end + 1500 ms` guard remains the fallback when no ANSpcm is visible.
- The stereo hint now carries the digital side's ANSpcm end sample
  (`digital_anspcm_end` -> `stereo_short_p1_partner_anspcm_end`) so the
  analog channel — which never sees the ANSpcm itself — anchors its retry
  search at the same place and both channels identify the same physical
  retry burst.

### Verification

- `USR-Message-V92QC` left/digital channel: retry CM at 8421.8 ms now
  recorded as the 9.2.4.3 step; outcome `incomplete` -> `v8-fallback`,
  agreeing with the right channel.  Every channel of the six-capture V.92
  truth set now resolves to a terminal outcome.
- `USR-Message-V92NC`: both channels now pick the same retry burst
  (7461.9/7477.9 ms — the caller's first CM after ANSpcm ends at 7377 ms).
  The old guard skipped it and reported the *second* retry (9851.8/9949.2).
- Motorola QC/NC, both Agere captures, and three non-V.92 captures
  (USR-Sportster, Motorola-SM56, xircom-v90) are byte-identical.
- `make test` (data stack, V.42 link, V.34 Phase 2 decode, full PCM-modem
  loopback suite) passes.

## QTS/QTS\ Separation + Committed Test Suite (2026-07-14)

### QTS\ separation (commit 37aa040)

The waveform QTS detector's 3-sample boundary tests (antisymmetry /
sameness) never fired on real captures because the channel smears the
QTS -> QTS\ flip transient across several samples.  Key identity: QTS\ is
-QTS, and since QTS is 3-sample antisymmetric, -QTS equals QTS shifted by
3 samples — so per-period **correlation sign** against a template folded
from the early periods separates them robustly: QTS periods correlate at
+1, QTS\ at -1, ANSpcm falls inside the +/-0.75 gate.  All four
QTS-bearing captures (Motorola QC/NC, USR QC/NC) now report
`reps=130 qts_bar=8` — the nominal 128+8 with two leading smear blocks on
the QTS side.  Empirically the Motorola boundary block measures corr
-0.33 (junk) with 7 clean -1.000 blocks after it.

### Committed regression tests (commits cc6f3c5, 5b6275d, a6dd55b)

`v92_proc_eval_test` (in `make test`) includes `phase12_decode.c` directly
(the evaluator is static) with real `window_energy`/`tone_energy_ratio`
implementations mirroring vpcm_decode.c.  Covers:

- clause 9.2 evaluator with fabricated timelines: figure-3 short-phase2
  success + handoff sample, V.8 retry anchored at ANSpcm end (local and
  partner-hint), CM-during-ANSpcm rejection, incomplete, both-analog
  V.34-Phase-2, stale-flag reconciliation;
- waveform QTS/QTS\ splitting on FIR-filtered synthetic signals;
- **audio-level follow-up chain**: fabricated QCA1d anchor + synthesized
  QTS/QTS\/ANSpcm/TONEq waveforms through
  `detect_v92_short_phase1_followup()`, asserting TONEq detection,
  digital-chain validation, handoff, and the short-phase2 evaluator
  outcome (plus a no-TONEq negative).  TONEq has never fired on a line
  capture, so this is its only end-to-end coverage.

Gotchas encoded in the test: fabricated timeline events need
`source="V.92"` (the branch selector filters on it), and the makefile
needs an explicit `v92_proc_eval_test.o: phase12_decode.c` dependency or
evaluator changes don't rebuild the test (verified by mutation).

### Remaining after this step

- A real capture of a successful quick connect for the truth set.
- ODP/ADP transmission-side behavior (decode-side verdict landed earlier).

## First-Stage QC2 Chain Recovery + Call-Log Story (2026-07-14, commit 438c944)

### The Agere claim was wrong

The earlier "their QC2 stage aborted before any chain ran" verification
note is corrected: `Agere-SV92-QC`'s digital modem DID transmit its chain
— QTS at 2997.4 ms (128+8 exactly) and 1655.5 ms of ANSpcm at -12 dBm0,
matching the level the QCA2d frame advertised.  The QTS could not be a
V.8bis tone artifact: 1375 Hz drifts off the 6-sample sign pattern within
~30 samples, while 128 clean reps require exactly 1333.3 Hz.  It was
missed because **V.8bis frame stamps trail the actual transmission** — the
QCA2d is stamped at 3093.2 ms, after the QTS.  The family-2 waveform
search now scans backward from the stamp, and the evaluator records a QTS
shortly before the trailing stamp as observed-in-order (9.2.4.2).

### Two-stage first-stage chains (Motorola)

The Motorola QC/NC prologue (CRe -> QC2a -> QCA2d) also ran its own
digital chain (QTS 128+8 at ~3195 ms + ANSpcm that blends into the ANSam
restart).  A new scan around the QCA2d stamp stores it in
`v92_stage1_*` fields; the clause 9.2 trace gains first-stage 9.2.4.2
QTS/ANSpcm steps; outcome and operative story unchanged.

### Call-log integration

`phase12_merge_to_call_log` now emits the full V.92 story: strict
QC1a/QCA1d signals with UQTS/LM/LAPM detail (deduped against the richer
V.8bis id_field emitter for family-2), QTS/QTS\ with rep counts, phase12
ANSpcm, first-stage chain events, and a "Clause 9.2 outcome" event.  New
`phase12_v92_digital_span()` gives the V.90 Sd sign-pattern scans a
suppression range covering both chains and the stereo partner's chain
(QTS is sign-identical to 128 Sd + 8 S~d reps).  Removing the
misattribution unmasked the genuine post-fallback V.90 Sd later in the
same calls (e.g. USR-QC left at 13105 ms, Motorola QC at 13271 ms) that
the first-match scan had been swallowing.

Known cosmetic leftover: family-2 QC2a/QCA2d appear twice in the call log
(once from the V.8bis id_field emitter, once from call_initiation) — this
duplication predates these changes.
