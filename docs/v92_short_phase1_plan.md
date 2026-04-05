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

Proceed to Step 6:

- build a clearer chronological per-channel Phase-1 event timeline
- keep tone, `CRe/CRd`, V.8/V.8bis, and strict short-Phase-1 events separate before arbitration
- use that timeline to understand why the QC files still appear mixed even after stricter
  short-Phase-1 acquisition

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
