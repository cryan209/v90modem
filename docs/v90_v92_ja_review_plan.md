# V.90/V.92 Ja Review And Fix Plan

This document compares the repository's current Ja decoding paths against:

- `ITU Docs/T-REC-V.90-199809-I!!PDF-E-1.pdf`
- `ITU Docs/T-REC-V.92-200011-I!!PDF-E.pdf`

The focus here is narrow:

- Ja sequence structure
- DIL descriptor decoding inside Ja
- differences between live/runtime C decode, offline C decode, and Python tools
- a staged implementation and test plan to bring the decoders closer to exact
  Recommendation behavior

## Scope

The relevant clauses are:

- V.90 `8.3.1` Ja
- V.90 `8.4.1` DIL
- V.90 `9.3.1.3` to `9.3.1.6`
- V.90 `9.3.2.4` to `9.3.2.10`
- V.92 `8.5.4` Ja
- V.92 `8.6.1` DIL
- V.92 `9.5.1.1.3` to `9.5.1.1.11`
- V.92 `9.5.2.1.3` to `9.5.2.1.11`

## Spec Summary

### V.90 Ja

Per V.90 `8.3.1`, Ja:

- consists of repetitions of the DIL descriptor
- may terminate without completing the final DIL descriptor
- uses the modulation defined by `10.1.3.3/V.34`

The V.90 DIL descriptor in Table 12 includes:

- 17 ones frame sync followed by a zero start bit
- `N`, reserved-zero bits, `LSP - 1`, `LTP - 1`
- framed `SP` and `TP` patterns with a zero start bit every 16 bits
- framed `H1..H8`
- framed `REF1..REF8`
- framed training Ucodes
- CRC and fill bits

### V.92 Ja

Per V.92 `8.5.4`, Ja:

- consists of 24 binary ones followed by repetitions of the DIL descriptor in
  Table 20
- uses the same modulation as `TRN1u`
- is scrambled and differentially encoded
- initializes the differential encoder memory with the final symbol of the
  preceding `TRN1u`
- may terminate without completing the final DIL descriptor
- shall be an integer multiple of 12 bits long

Table 20 extends the base V.90 descriptor with:

- a downstream data signalling rate capability mask
- a continuation mask field
- another start bit
- CRC
- fill bits to the next multiple of 12 bits

## Current Implementations

### C runtime/live path

Primary files:

- [modem_engine.c](/Users/scottcryan/v90modem/modem_engine.c)
- [spandsp-master/src/v34rx.c](/Users/scottcryan/v90modem/spandsp-master/src/v34rx.c)
- [v90.c](/Users/scottcryan/v90modem/v90.c)

Observed behavior:

- `v34rx.c` captures descrambled Ja aux bits from the Phase 3 receive path.
- `modem_engine.c` buffers those bits and scans for `17 ones + 0`.
- on a candidate preamble, it calls `v90_parse_dil_descriptor()`
- if parse succeeds, it stores the descriptor for later DIL generation

Important limitation:

- the runtime path only knows how to parse the V.90 base descriptor
- it does not parse the V.92 Table 20 extension
- it does not verify the V.92 24-one Ja prefix
- it does not verify the V.92 modulo-12 Ja sequence length

### C offline/analyzer path

Primary files:

- [v92_ja_decode.c](/Users/scottcryan/v90modem/v92_ja_decode.c)
- [v92_ja_decode.h](/Users/scottcryan/v90modem/v92_ja_decode.h)
- [vpcm_decode.c](/Users/scottcryan/v90modem/vpcm_decode.c)

Observed behavior:

- `v92_ja_dil_search()` scans a codeword window for likely descriptor starts
- it performs sign-differential decode and descrambling
- it first tries strict V.90 parse via `v90_parse_dil_descriptor()`
- if that fails, it tries a V.92 Table 20-aware parse helper
- if no hard parse succeeds, it can still return a soft lock based on heuristics

Important limitation:

- this path is an analyzer/recovery decoder, not a strict Ja conformance check
- it does not require the V.92 24-one Ja prefix before the descriptor
- it scores candidates using sync, reserved-zero, and CRC nearness metrics
- its V.92 helper accepts CRC Hamming distance up to 2 instead of exact CRC only

### Python tools

Primary files:

- [tools/recover_ja_descriptor.py](/Users/scottcryan/v90modem/tools/recover_ja_descriptor.py)
- [tools/check_ja_dil_consistency.py](/Users/scottcryan/v90modem/tools/check_ja_dil_consistency.py)
- [tools/validate_ja_sequence.py](/Users/scottcryan/v90modem/tools/validate_ja_sequence.py)
- [tools/decode_ja_bits.py](/Users/scottcryan/v90modem/tools/decode_ja_bits.py)

Observed behavior:

- Python has exact V.90 base descriptor parsing
- Python also has exact V.92 Table 20 tail parsing
- Python checks V.92 zero-fill to modulo-12 in the parser
- `validate_ja_sequence.py` explicitly checks for a 24-one prefix and whether a
  captured bitstream length is a multiple of 12

Important limitation:

- the recovery scripts are still heuristic search tools
- they are not currently used to drive the runtime or primary C decode path

## Comparison Matrix

| Item | Spec requirement | C runtime/live | C offline | Python |
| --- | --- | --- | --- | --- |
| V.90 Table 12 base descriptor | exact parse | yes | yes | yes |
| V.92 Table 20 extension | exact parse | no | partial yes | yes |
| differential decode seeded from prior symbol | required | implicit in Rx capture path | yes | partly tool-dependent |
| descrambling | required | implicit in Rx capture path | yes | yes |
| V.92 24 leading ones before descriptor | required | no | no | yes |
| V.92 integer multiple of 12 bits | required | no | not enforced at sequence level | yes |
| exact V.92 CRC | required | no V.92 parse | no, tolerant up to HD 2 | yes |
| soft/recovery mode separated from strict mode | desirable for diagnostics | no | partly | partly |

## Clause-Level Findings

### 1. V.90 Table 12 descriptor parsing is in good shape

`v90_parse_dil_descriptor()` is close to the Recommendation for the base
descriptor:

- sync and start bit
- reserved-zero enforcement
- framed pattern extraction
- framed H/REF extraction
- training Ucode extraction
- exact CRC

This should remain the shared strict base parser.

### 2. Live/runtime Ja handling is still V.90-shaped

The runtime path in `modem_engine.c` effectively assumes:

- if we see `17 ones + 0`, we may already be at a descriptor start
- the correct parser is the V.90 base parser

That is acceptable for V.90, but incomplete for V.92 because the Recommendation
defines Ja as:

- 24 ones
- then repeated Table 20 descriptors
- with modulo-12 sequence shaping

### 3. The offline C path mixes strict parsing and recovery heuristics

`v92_ja_dil_search()` is useful for difficult captures, but it currently mixes:

- exact parsing
- near-match scoring
- polarity inversion search
- window anchoring
- soft-lock output

This is good analyzer behavior, but not good primary conformance behavior.

The most important spec mismatch is that the V.92 parser helper tolerates near
CRC matches instead of requiring exact CRC equality.

### 4. Python currently has the strictest V.92-specific logic

The Python tools already implement several V.92-specific checks that the C
paths do not:

- 24-one Ja anchor handling
- exact Table 20 tail parsing
- modulo-12 fill validation

That makes the Python tools a good temporary oracle for test vectors while the
C implementation is tightened.

## Design Principles For Fixes

- Keep a strict Ja parser and a recovery analyzer as separate layers.
- Reuse the existing V.90 base descriptor parser for the common Table 12 part.
- Move V.92-specific sequence validation into explicit code paths instead of
  inferring it from descriptor nearness metrics.
- Do not let soft-lock or near-CRC recovery drive the live modem behavior.
- Keep the runtime/live path and offline path aligned on one strict parser.
- Preserve tolerant recovery tools, but mark them diagnostic-only.

## Fix Plan

### Step 1. Split strict parsing from recovery scoring

Introduce a clear distinction between:

- strict Ja sequence validation
- strict descriptor parsing
- recovery search and soft-lock ranking

Expected code shape:

- one strict V.90/V.92 Ja parser API
- one analyzer wrapper that may search, score, or repair around that parser

### Step 2. Add a strict V.92 Ja sequence validator in C

Implement a strict validator that enforces:

- V.92 24-one prefix
- descriptor start immediately after the prefix
- exact Table 20 tail structure
- exact CRC equality
- zero fill to modulo-12

This validator should not:

- accept near CRC
- drift the candidate start arbitrarily
- succeed on descriptor-only evidence when the Ja sequence framing is wrong

### Step 3. Keep the existing V.90 strict parser as the shared base parser

Refactor so that:

- the V.90 base parser remains strict and reusable
- the V.92 parser calls the same base logic for the Table 12 portion
- V.92-specific tail and sequence rules are added on top

### Step 4. Upgrade the runtime/live modem path

Replace the current runtime-only `17 ones + 0 -> V.90 parse` flow with:

- role/protocol-aware Ja mode selection
- strict V.90 path when the call is V.90
- strict V.92 path when the call is V.92

For V.92, runtime should wait for:

- 24-one Ja preamble
- a strict Table 20 descriptor parse

and only then commit the pending DIL descriptor.

### Step 5. Downgrade soft-lock to diagnostic-only

The existing soft-lock behavior is valuable for offline analysis and debugging.
It should remain available, but:

- it must not be used by the live modem path
- it should be clearly labeled non-conformant/recovery-only
- tests should distinguish strict pass from recovery hint

### Step 6. Unify Python and C expectations

Once the strict C parser is in place:

- keep the Python tools as external validation helpers
- align the Python and C parse rules on exact CRC and exact fill handling
- make disagreement between Python strict parse and C strict parse a test failure

## Test Plan

### A. Unit tests for strict descriptor parsing

Add focused tests for:

- valid V.90 Table 12 descriptor
- valid V.92 Table 20 descriptor
- reserved-zero violations
- bad sync
- bad start bits
- bad CRC
- invalid `N = 0` with `LSP/LTP != 1`
- incorrect modulo-12 V.92 fill

Suggested targets:

- C unit tests around `v90_parse_dil_descriptor()`
- new C unit tests around the strict V.92 Ja parser
- Python cross-check fixtures for exact same bitstreams

### B. Sequence-level tests for V.92 Ja

Add tests that verify:

- 24 leading ones are required
- the descriptor begins immediately after the 24 ones
- truncated sequences fail strict validation
- non-modulo-12 sequences fail strict validation
- exact same descriptor passes when framed correctly

### C. Search/analyzer tests

Retain analyzer tests, but separate them from strict parser tests:

- strict parse success
- strict parse failure with soft-lock available
- no viable candidate

These tests should confirm that:

- recovery mode can still rank candidates
- recovery mode does not get mistaken for strict conformance

### D. Runtime integration tests

Add integration coverage for the live path:

- V.90 Ja capture yields a committed pending DIL only after strict V.90 parse
- V.92 Ja capture yields a committed pending DIL only after strict V.92 parse
- malformed Ja does not poison runtime DIL state

### E. Loopback and corpus tests

Use the existing captures and generated bit dumps to build a small Ja corpus:

- known-good V.90 Ja
- known-good V.92 Ja
- corrupted CRC V.92 Ja
- missing 24-one prefix V.92 Ja
- malformed modulo-12 V.92 Ja
- polarity-inverted or timing-shifted captures for recovery-only checks

For each corpus item, record:

- strict parse expected result
- analyzer/recovery expected result
- whether runtime should accept or reject it

## Implementation Order

1. Add the document-backed strict/recovery split.
2. Add C unit tests for V.90 and V.92 Ja parsing.
3. Implement strict V.92 Ja sequence validation in C.
4. Change offline C decode to report strict result separately from soft-lock.
5. Change runtime/live modem path to use strict V.90/V.92 parsers only.
6. Add Python-vs-C cross-check tests on shared fixture bitstreams.
7. Add capture-based integration tests.

## Immediate Next Patch Set

The first patch set should stay small and low-risk:

1. Introduce a strict V.92 Ja parser helper in C with exact CRC.
2. Add unit tests for valid and invalid Table 20 examples.
3. Keep `v92_ja_dil_search()` unchanged as a recovery wrapper, but make it call
   the new strict helper first.
4. Add result flags so callers can distinguish:
   - strict V.90 success
   - strict V.92 success
   - recovery-only soft lock

Expected result:

- no immediate behavior regression for offline analysis tools
- a clean strict parser to move into the live path next
- the beginning of a test corpus that locks in the spec behavior

## Status

- Reviewed on 2026-04-11.
- No code changes have been made yet from this plan.
- Current conclusion:
  - V.90 base descriptor parsing is mostly sound
  - V.92 strict Ja handling is currently strongest in Python tooling
  - C runtime/live handling needs V.92-aware strict parsing
  - C offline handling should keep recovery mode, but separate it from strict
    conformance
