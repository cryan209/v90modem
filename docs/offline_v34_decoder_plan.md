# Offline V.34/V.90 Decoder Plan

## Goal

Move `vpcm_decode` away from driving the full SpanDSP V.34 modem state machine
as if it were a live endpoint, and toward an offline analysis pipeline that:

- segments the waveform first
- runs only the decoders relevant to each segment
- reuses SpanDSP demod/parsing code where it is valuable
- owns the orchestration and stopping conditions locally

The target is not "replace all of SpanDSP". The target is "stop using the full
real-time modem loop when we only need handshake decode."

## What We Already Have

The codebase already reduces dependence on SpanDSP in several important areas.

### Existing offline or semi-offline pieces

- `p3_demod.[ch]`
  - local Phase 3 demodulator for V.34/V.90/V.92
  - carrier recovery, symbol timing, differential decode, descrambling
  - explicitly intended to replace SpanDSP dependence for Phase 3 analysis
- `vpcm_decode.c`
  - `snapshot_v34_info0a_from_rx_state()`
  - `snapshot_v34_info1a_from_rx_state()`
  - `map_v34_received_info0a()`
  - `map_v34_received_info1a()`
  - local rescue/merge logic around partial INFO recovery
- `vpcm_decode.c`
  - local MP parsing helpers:
    - `v34_parse_mp_from_raw_bits()`
    - `v34_parse_mp_from_frame_bits()`
    - `v34_infer_mp_rates_from_info_sequences()`
- `vpcm_decode.c`
  - offline post-Phase-3/V.90 helpers:
    - `offline_v90_decode_jd_bits()`
    - `decode_jd_stage()`
    - `decode_ja_dil_stage()`
    - `decode_post_phase3_codewords()`
- `v8` path
  - local targeted burst/window search already exists in `vpcm_decode.c`
  - SpanDSP also already exposes an offline-oriented `v8_decode_rx()` path

### What this means

We are not starting from scratch. We already have:

- a custom offline Phase 3 decoder
- local post-Phase-3 decoders
- local wrappers that reinterpret SpanDSP output into offline-friendly results
- multiple places where `vpcm_decode` already chooses "targeted decode" over
  "let SpanDSP run everything"

The remaining gap is mostly Phase 2 and control-flow ownership.

## SpanDSP Boundary Recommendation

### Keep using directly

These are good DSP building blocks or already shaped for analysis:

- `v8_decode_rx()` in `spandsp-master/src/v8.c`
- `modem_connect_tones_rx()`
- `fsk_rx()`
- CRC helpers
- low-level tone/power helpers where convenient

### Stop treating as the primary API

- `v34_init()`
- `v34_rx()`
- `v34_tx()`
- the full V.34 TX/RX stage machine as the main decode path

Those remain useful as fallback/reference behavior, but they are the wrong
top-level abstraction for offline decoding of long WAV files.

### Best extraction target from SpanDSP V.34

The most useful receive-side internal functions in `spandsp-master/src/v34rx.c`
are:

1. `info_rx()`
2. `put_info_bit()`
3. INFO boundary/slip recovery helpers near `put_info_bit()`
4. `l1_l2_analysis()`
5. `cc_rx()`

`primary_channel_rx()` is much heavier and more entangled. We should not start
there because we already have `p3_demod` covering much of the Phase 3 use case.

## Desired Offline Pipeline

### Stage A: coarse segmentation

Input:

- linear PCM samples
- optional derived G.711 codewords

Output:

- candidate windows tagged as:
  - pre-V.8 / tone activity
  - V.8 / V.21 burst region
  - V.34 Phase 2 candidate
  - L1/L2 probe region
  - Phase 3 candidate
  - post-Phase-3 PCM/codeword region

Suggested features:

- RMS envelope
- short-window peak energy
- dominant tone bins
- reversal density
- V.21 pair energy
- rough carrier-presence gating

This stage should decide where to decode, not how to decode.

### Stage B: V.8 decode

Preferred approach:

- keep current local targeted V.8 search
- optionally wrap SpanDSP `v8_decode_rx()` for monitoring-only decode inside
  preselected windows

### Stage C: V.34 Phase 2 offline decoder

This is the main new work.

Target scope:

- INFO0a / INFO0d
- Tone A / Tone B and reversals
- L1/L2 timing
- INFO1a / INFO1d

Do not include:

- full live modem progression
- TX-side behavior
- data mode

Implementation direction:

- create `v34_phase2_decode.[ch]`
- either:
  - extract the relevant receive-only logic from `v34rx.c`, or
  - port the needed algorithms into a local offline decoder

The decoder should accept a bounded sample window and return:

- decoded INFO messages
- tone/reversal timestamps
- confidence and recovery diagnostics
- next likely handoff point into Phase 3

### Stage D: Phase 3 offline decode

Use the existing local `p3_demod` path as the primary engine.

This is already the strongest part of the "less SpanDSP-dependent" design.

### Stage E: post-Phase-3 decode

Keep and extend the existing local codeword-based decoders:

- `decode_jd_stage()`
- `decode_ja_dil_stage()`
- `decode_post_phase3_codewords()`

This area is already aligned with the offline architecture.

## Staged Implementation Plan

### Phase 1: formalize the current hybrid model

Goal:

- make the current architecture explicit

Tasks:

- add a `v34_predecode_result` or similar struct that carries:
  - INFO0/INFO1
  - phase boundaries
  - tone/reversal times
  - confidence
- route `run_decode_suite()` through a single predecode object instead of
  repeatedly rediscovering the same milestones
- keep current SpanDSP-backed Phase 2 path, but make it look like one decoder
  module instead of inlined orchestration

Success criteria:

- one shared "Phase 2/phase boundary" result object
- no repeated full predecode passes for print/call-log/visualizer consumers

### Phase 2: isolate Phase 2 into its own module

Goal:

- separate Phase 2 from the rest of `vpcm_decode.c`

Tasks:

- create `v34_phase2_decode.[ch]`
- move current INFO mapping, rescue, and event interpretation there
- preserve current SpanDSP-backed implementation behind a narrow API

Suggested API shape:

```c
typedef struct {
    bool info0_seen;
    bool info1_seen;
    bool phase3_seen;
    int info0_sample;
    int info1_sample;
    int phase3_sample;
    int tone_a_sample;
    int tone_b_sample;
    int tone_a_reversal_sample;
    int tone_b_reversal_sample;
    int confidence;
} v34_phase2_result_t;

bool v34_phase2_decode_window(const int16_t *samples,
                              int total_samples,
                              int sample_rate,
                              v91_law_t law,
                              bool calling_party_hint,
                              v34_phase2_result_t *out);
```

Success criteria:

- `vpcm_decode.c` no longer owns detailed Phase 2 state reconstruction logic
- existing behavior preserved

### Phase 3: add offline window finder ahead of Phase 2

Goal:

- stop feeding whole files into the Phase 2 decoder

Tasks:

- build a `v34_phase2_window_finder.[ch]`
- detect candidate INFO/tone/L1-L2 windows from:
  - envelope
  - tone persistence
  - reversal activity
  - CC-like carrier presence
- run Phase 2 decode only inside those windows

Success criteria:

- long quiet or low-value tails are not scanned by Phase 2
- decoder cost scales with active handshake duration, not full WAV duration

### Phase 4: replace the SpanDSP-backed Phase 2 engine

Goal:

- use SpanDSP receive-side algorithms without running full `v34_rx()`

Tasks:

- extract or port:
  - `info_rx()`
  - `put_info_bit()`
  - INFO recovery helpers
  - `l1_l2_analysis()`
  - possibly `cc_rx()`
- hide this behind the same `v34_phase2_decode_window()` API from Phase 2
- keep the current `v34_rx()`-driven path as a debug/fallback mode

Success criteria:

- Phase 2 decode no longer requires `v34_tx()`
- offline decode flow is owned locally
- the SpanDSP dependency is at the algorithm level, not full modem-session level

### Phase 5: unify handoff across the pipeline

Goal:

- make Phase 2 -> Phase 3 -> post-Phase-3 transitions explicit

Tasks:

- define a shared handoff structure with:
  - role
  - phase start/end samples
  - baud/carrier hints
  - U_INFO
  - V.90/V.92 capability inference
- let:
  - Phase 2 feed Phase 3
  - Phase 3 feed Jd/Ja/DIL/post-Phase-3 decode

Success criteria:

- no repeated boundary inference in downstream decoders
- one canonical flow graph for offline decode

## Suggested File Layout

- `v34_phase2_decode.h`
- `v34_phase2_decode.c`
- `v34_phase2_window.h`
- `v34_phase2_window.c`
- optional later:
  - `v34_phase2_spandsp_bridge.c`
  - `v34_phase2_offline_rx.c`

This keeps the initial extraction small and lets us switch implementations
behind a stable API.

## Recommended Immediate Next Step

Implement Phase 1 and Phase 2 first:

1. create a `v34_phase2_result_t`
2. move current INFO/tone/rescue logic behind `v34_phase2_decode_window()`
3. make `run_decode_suite()`, call-log generation, and visualizer all consume
   the same predecode result

That gets us a cleaner architecture right away, even before we replace the
SpanDSP-backed implementation.

## Non-Goals For The First Cut

- replacing all of SpanDSP V.34 RX at once
- touching Phase 3 custom demod unless required for handoff cleanup
- rewriting MP RX or full data-mode RX
- changing modem-engine runtime behavior

The objective is to improve `vpcm_decode` as an offline analyzer, not to
rebuild the entire runtime modem stack.
