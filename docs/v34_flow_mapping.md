# V.34 Flow Mapping: SpanDSP, `modem_engine`, and D-Modem

This note maps three views of the same startup:

- `modem_engine.c`: high-level call state used by this project.
- SpanDSP V.34 internals: the actual TX/RX phase machine.
- `D-Modem` decompilation docs: an outer runtime dispatcher that helps explain where handshake, data pump, and retrain logic sit.

The `D-Modem` material does not map 1:1 onto SpanDSP internals. It is still useful because it exposes the orchestration layer around V.34 processing that SpanDSP does not describe clearly.

## Primary local sources

- `modem_engine.c`
- `spandsp-master/src/v34tx.c`
- `spandsp-master/src/v34rx.c`
- `spandsp-master/src/spandsp/private/v34.h`

## External reference set

From `cryan209/D-Modem` branch `pjsip2.15`:

- `doc/VPcmV34Progress_target_walkthrough.md`
- `doc/VPcmV34Progress_jump_tables.csv`
- `doc/VPcmV34Progress_branch_map.md`
- `doc/V34SetupModulator_state_graph.md`
- `doc/V34SetupDemodulator_state_graph.md`
- `doc/TimingV34_target_walkthrough.md`
- `doc/v34modeminit_state_graph.md`

## High-level mapping

| Layer | Meaning |
|---|---|
| `ME_V8` | Pre-V.34 capability exchange. |
| `ME_TRAINING` | Entire SpanDSP startup from INFO0 through MP exchange. |
| `ME_DATA` | Post-training steady data mode. |
| `VPcmV34Progress` handshake/modulate loop | Roughly equivalent to SpanDSP’s pre-data TX/RX state machine. |
| `VPcmV34Progress` data/retrain paths | Roughly equivalent to SpanDSP post-startup data path plus retrain handling. |

In this tree, `modem_engine` enters V.34 by calling `v34_init(...)` and then mostly lets SpanDSP run its internal state machine. See `start_v34_training()` in `modem_engine.c`.

## Engine state to SpanDSP phase mapping

| `modem_engine` state | SpanDSP activity |
|---|---|
| `ME_V8` | V.8 negotiation, before `v34_init(...)`. |
| `ME_TRAINING` | INFO0, A/B tone exchange, L1/L2, INFO1, Phase 3, Phase 4, MP. |
| `ME_DATA` | Entered when SpanDSP reports training complete via status callback. |

Important consequence: `modem_engine` does not expose Phase 2, 3, and 4 separately. If you want more visibility, it needs instrumentation against SpanDSP TX/RX stages rather than more outer state values.

## SpanDSP phase map

The authoritative phase names are in `spandsp/private/v34.h`.

### Phase 2 / control-channel startup

| ITU / flow concept | SpanDSP TX | SpanDSP RX | Notes |
|---|---|---|---|
| INFO0 exchange | `V34_TX_STAGE_INFO0` / `V34_TX_STAGE_INFO0_RETRY` | `V34_RX_STAGE_INFO0` | Initial capability exchange. |
| Answer tone A / !A | `V34_TX_STAGE_INITIAL_A`, `FIRST_A`, `FIRST_NOT_A`, `SECOND_A` | `V34_RX_STAGE_TONE_A` | Answerer-side startup tone sequence. |
| Caller tone B / !B | `V34_TX_STAGE_FIRST_B`, `FIRST_NOT_B`, `SECOND_B` | `V34_RX_STAGE_TONE_B` | Caller-side startup tone sequence. |
| L1/L2 probing | `V34_TX_STAGE_L1`, `V34_TX_STAGE_L2` | `V34_RX_STAGE_L1_L2` | Round-trip / probing area. |
| INFO1 exchange | `V34_TX_STAGE_PRE_INFO1_A`, `V34_TX_STAGE_INFO1` | `V34_RX_STAGE_INFO1C`, `V34_RX_STAGE_INFO1A` | Final parameter exchange before primary channel training. |
| Half-duplex control channel | `V34_TX_STAGE_HDX_*` | `V34_RX_STAGE_INFOH`, `V34_RX_STAGE_CC` | Present but not relevant for current full-duplex path. |

### Phase 3 / primary-channel training

| ITU / flow concept | SpanDSP TX | SpanDSP RX | Notes |
|---|---|---|---|
| S / !S preamble | `V34_TX_STAGE_FIRST_S`, `FIRST_NOT_S`, `SECOND_S`, `SECOND_NOT_S` | `V34_RX_STAGE_PHASE3_WAIT_S` then `V34_RX_STAGE_PHASE3_TRAINING` | RX waits for a robust S pattern before treating training as active. |
| Optional MD | `V34_TX_STAGE_MD` | still Phase 3 RX | This implementation uses a placeholder timing path, not a real vendor waveform. |
| PP | emitted by `pp_baud_init()` during Phase 3 | still `V34_RX_STAGE_PHASE3_TRAINING` | Not represented by a dedicated enum stage. |
| TRN | `V34_TX_STAGE_TRN` | `V34_RX_STAGE_PHASE3_TRAINING` | Main equalizer/scrambler conditioning area. |
| J | `V34_TX_STAGE_J` | detected in `V34_RX_STAGE_PHASE3_WAIT_S` / training logic | Used as the Phase 3 to 4 transition cue. |
| J' | `V34_TX_STAGE_J_DASHED` | detected later as a Phase 4 readiness cue | SpanDSP treats J and J' differently to avoid early transition mistakes. |

### Phase 4 / final training and MP

| ITU / flow concept | SpanDSP TX | SpanDSP RX | Notes |
|---|---|---|---|
| Wait/silence before answerer phase 4 signaling | `V34_TX_STAGE_PHASE4_WAIT` | `V34_RX_STAGE_PHASE4_TRN` | Answerer RX is already conditioned for TRN/MP, not a fresh S detector. |
| S | `V34_TX_STAGE_PHASE4_S` | `V34_RX_STAGE_PHASE4_S` | 128T. |
| S-bar | `V34_TX_STAGE_PHASE4_NOT_S` | `V34_RX_STAGE_PHASE4_S_BAR` | 16T transition marker. |
| TRN | `V34_TX_STAGE_PHASE4_TRN` | `V34_RX_STAGE_PHASE4_TRN` | Waits for explicit far-end J'/TRN confirmation before MP. |
| MP / MP' | `V34_TX_STAGE_MP` | `V34_RX_STAGE_PHASE4_MP` | Parameter exchange before data. |

## D-Modem dispatcher to SpanDSP mapping

This mapping is approximate. The names and split points are different, but the control intent lines up well enough to be useful.

| D-Modem artifact | Likely SpanDSP equivalent |
|---|---|
| `VPcmV34Progress` main state dispatch | Top-level runtime around the full V.34 datapath. |
| `B3_HANDSHAKE_MOD_LOOP` with `v34handshak`, `modulatevector` | SpanDSP startup path from INFO0 through MP. |
| `B4_RUNPCM_DISPATCH_JT1` with `runPcmModem()` result dispatch | Post-startup demod/data path and transition handling. |
| Post-state watchdog / delay selector tables | SpanDSP timeout, retrain, and phase guard behavior, but implemented externally in D-Modem. |
| `VPcmV34InitiateRetrain` callouts | Analogous to retrain/error recovery hooks a full V.34 implementation needs after data mode. |
| `V34SetupModulator` / `V34SetupDemodulator` | Similar role to SpanDSP setup of baud/carrier/filter configuration before training. |
| `TimingV34` | Similar role to timing-loop/equalizer update logic used during training and data. |

The biggest conceptual difference is this:

- D-Modem exposes an outer supervisor that explicitly dispatches handshake, PCM modem run loops, watchdogs, and retrain windows.
- SpanDSP hides more of that inside the modem instance, exposing mainly callbacks and status.

## What D-Modem clarifies usefully

### 1. V.34 is not just one training loop

`VPcmV34Progress` shows a supervisor split between:

- handshake/modulation,
- datapump/data processing,
- watchdog and delay selection,
- retrain and renegotiation arms.

That matches what is easy to miss when reading only `modem_engine.c`, where everything collapses into one `ME_TRAINING` state.

### 2. Retrain logic is an outer control concern

The jump-table outputs in `VPcmV34Progress_jump_tables.csv` show explicit return-code driven transitions to retrain reasons and delayed restart windows. That is a useful reference if this project later needs stronger recovery behavior than “training timeout then hang up”.

### 3. Modulator/demodulator setup is separated from runtime progress

`V34SetupModulator` and `V34SetupDemodulator` are distinct from `VPcmV34Progress`. That mirrors the distinction in this tree between:

- one-time `v34_init(...)` plus power/carrier choices in `modem_engine.c`,
- ongoing internal SpanDSP TX/RX stage progression.

## Implications for this project

### Current `modem_engine` visibility is too coarse

`ME_TRAINING` covers all of:

- INFO0/INFO1 exchange,
- tone reversals,
- L1/L2,
- Phase 3,
- Phase 4,
- MP.

If interoperability debugging is the goal, the useful next step is to log SpanDSP TX/RX stage changes, not to add more guesses around the wrapper.

### The most valuable instrumentation points are:

- SpanDSP TX stage transitions.
- SpanDSP RX stage transitions.
- `received_event` transitions such as `INFO0_OK`, `L2_SEEN`, `S`, `J`, `J_DASHED`, `PHASE4_TRN_READY`.
- final MP acceptance and selected rate.

### D-Modem is strongest as a control-flow reference

The decompiled docs are much less useful for exact DSP math than for:

- where watchdogs live,
- when retrain is armed,
- how setup and progress are split,
- what the outer modem supervisor is expected to coordinate.

## TRN and MP implementation notes

This is the most important part of the current SpanDSP V.34 startup.

## SmartLink `dsplibs.o` crosswalk

The original `slmodemd` binary is not present in this repo, but two local sources are enough to reconstruct the practical flow:

- `slmodem call to courier modem.md`
- decompiled `D-Modem` notes for `VPcmV34Progress`, `v34handshak`, and related VPCM symbols

### Architecture shape

SmartLink did not expose V.34 as one flat state machine.

The rough layering is:

- `vpcm_run()` as the outer adapter entrypoint
- `VPcmV34Progress()` as the top-level V.34 supervisor
- `v34handshak` as the handshake engine
- `modulatevector`, `datapumpv34`, and `adaptecho` as worker paths under that supervisor

That is much closer to a supervisor-plus-subengines design than to SpanDSP’s more self-contained modem instance.

### Observed TRN/MP crosswalk

| SmartLink / log name | Meaning in SmartLink flow | Closest SpanDSP stage |
|---|---|---|
| `SSEG` | Phase 3 S preamble | `V34_TX_STAGE_FIRST_S` / `V34_TX_STAGE_SECOND_S` |
| `SBARSEG` | Phase 3 S-bar / !S segment | `V34_TX_STAGE_FIRST_NOT_S` / `V34_TX_STAGE_SECOND_NOT_S` |
| `PPSEG` | Phase 3 PP sequence | `pp_baud_init()` / PP emission inside Phase 3 |
| `TRNSEG4` | Main Phase 3 TRN training block | `V34_TX_STAGE_TRN` |
| `JTXMIT` | J transmit block after Phase 3 TRN | `V34_TX_STAGE_J` |
| `TRNSEG4A` | Phase 4 answerer TRN before MP | `V34_TX_STAGE_PHASE4_TRN` |
| `XMITMP` | MP / MP' transmit block | `V34_TX_STAGE_MP` |
| `DET_SYNC` | receive-side sync detector microstate | roughly `V34_RX_STAGE_PHASE4_TRN` or early `V34_RX_STAGE_PHASE4_MP` conditioning |
| `DET_INFO` | receive-side info/frame detector microstate | roughly MP preamble/frame acquisition inside `V34_RX_STAGE_PHASE4_MP` |

Important caveat:

- SmartLink `SSEG` / `SBARSEG` appear in both Phase 3 and Phase 4 contexts depending on where the outer supervisor currently is.
- The SmartLink names are segment-oriented and reused across phases.
- SpanDSP splits more explicitly by phase in the enum names.

### What the SmartLink logs show

Observed from `slmodem call to courier modem.md`:

- `SILENCE -> SSEG -> SBARSEG -> PPSEG -> TRNSEG4`
- echo adaptation starts during `TRNSEG4`
- `TRNSEG4 -> JTXMIT`
- on `JTXMIT`, RX changes from `WAIT` to `RECEIVE`
- echo canceller gets frozen on J transmit
- later the flow reaches `TRNSEG4A`
- during `TRNSEG4A`, the microstate flips `DET_SYNC <-> DET_INFO`
- then TX advances `TRNSEG4A -> XMITMP`
- once MP is detected, logs show repeated `MP detected, starting MP' txmit` and MP retransmit cycles

That gives this practical sequence:

`SSEG -> SBARSEG -> PPSEG -> TRNSEG4 -> JTXMIT -> TRNSEG4A -> XMITMP -> E -> DATA`

Closest SpanDSP sequence:

`FIRST_S/FIRST_NOT_S/SECOND_S/SECOND_NOT_S -> PP -> TRN -> J -> PHASE4_WAIT/S/S-bar/TRN -> MP -> E -> DATA`

### SmartLink-specific behavioral differences

Compared with SpanDSP, SmartLink appears to do three notable things differently.

#### 1. Echo adaptation is explicitly integrated into TRN/J timing

The logs show:

- echo adaptation start during `TRNSEG4`
- echo adaptation middle during continued training
- explicit echo freeze at `JTXMIT`

SpanDSP’s current V.34 path in this repo does not expose that style of tightly staged echo-control behavior. The wrapper instead disables the adaptive canceller and uses a notch filter.

#### 2. MP receive appears to be driven by microstate oscillation

The repeated `DET_SYNC <-> DET_INFO` transitions during `TRNSEG4A` and `XMITMP` suggest SmartLink uses a narrower alternating receive microstate model:

- sync detector state
- info/frame detector state

SpanDSP instead keeps a larger hypothesis-heavy `V34_RX_STAGE_PHASE4_MP` implementation with:

- preamble scoring,
- tap/order/domain retry,
- slip recovery,
- semantic validation.

#### 3. MP retransmission is very explicit

The SmartLink logs show repeated:

- `V34MP -MP sequence,...`
- `MP detected, starting MP' txmit`
- `Starting txmit MP again(n)`

So SmartLink clearly loops MP/MP' transmission under flag control until both sides converge.

That is functionally similar to SpanDSP’s `mp_seen` / `mp_remote_ack_seen` logic, but SmartLink’s logs make the repeated retransmission loop much more explicit.

### Practical interpretation

If you are trying to understand how `dsplibs.o` implemented TRN and MP, the main takeaways are:

- TRN is not just a passive training burst. In SmartLink it is tightly coupled to echo adaptation timing.
- J transmit is a hard transition point where RX mode and EC behavior change.
- MP acquisition is supervised by alternating sync/info microstates rather than a single large decoder state.
- MP is retransmitted aggressively until remote detection and acknowledgement conditions are satisfied.

### Why this matters for SpanDSP comparison

SpanDSP is closer to the spec structure in naming:

- Phase 3 TRN
- J / J'
- Phase 4 S / S-bar / TRN
- MP

SmartLink is easier to reason about operationally:

- segment names,
- explicit echo milestones,
- explicit microstate flips,
- explicit MP retransmit loop.

That makes SmartLink a good behavioral reference even when the internal DSP math differs.

### TRN transmit path

TRN TX is implemented in `get_trn_baud()` and initialized by `trn_baud_init()`.

- Phase 3 TRN uses the scrambler output directly, not differential encoding.
- 4-point TRN uses two scrambled bits per symbol.
- 16-point TRN uses four scrambled bits per symbol, split into `I` and `Q`.
- Full-duplex Phase 3 TRN length is hard-coded to `2048` bauds before switching to `J`.
- After TRN completes, TX initializes the J differential encoder from the final TRN symbol, which matches the spec intent.

Important local detail:

- When TRN ends, RX state is forcibly reset back to `V34_RX_STAGE_PHASE3_WAIT_S` and a large amount of J/TRN/MP conditioning state is cleared.
- That reset is not just cleanup. It is required so the later `S` detection actually runs during `J`.

Relevant code:

- `spandsp-master/src/v34tx.c`: `get_trn_baud()`, `trn_baud_init()`

### Phase 4 TRN transmit path

Phase 4 answerer-side TRN is in `get_phase4_baud()`.

- TX sends `S` for `128T`.
- Then `S-bar` for `16T`.
- Then reinitializes the scrambler to zero before Phase 4 TRN.
- Phase 4 TRN runs for at least `512` bauds.
- TX does not move to MP on local timing alone. It waits for RX to raise `V34_EVENT_PHASE4_TRN_READY`.
- A guard path forces MP if TRN runs excessively long without confirmation.

That means the real handoff to MP is RX-driven, not just TX-duration-driven.

Relevant code:

- `spandsp-master/src/v34tx.c`: `get_phase4_baud()`, `phase4_wait_init()`

### TRN receive path

The TRN receive side is more heuristic than the transmit side.

Phase 3 and Phase 4 TRN RX both try to infer a stable hypothesis for:

- symbol interpretation hypothesis,
- scrambler tap,
- bit order,
- sometimes absolute vs differential phase domain.

The key mechanism is not direct “known-sequence correlation” in the simple sense. Instead, RX scores descrambled output by how strongly it looks like the expected TRN ones distribution after the correct hypothesis is applied.

Important thresholds:

- `PHASE4_TRN_READY_MIN_SCORE = 70`
- `MP_TRN_PRELOCK_SCORE_MIN = 70`

Phase 4 specifically:

- RX waits for explicit far-end `J'` plus enough TRN evidence.
- Once TRN confidence is high enough, RX raises `V34_EVENT_PHASE4_TRN_READY`.
- RX also snapshots the best TRN hypothesis and carries it forward into MP acquisition.

This carry-forward is critical: MP detection is bootstrapped from the TRN lock state.

Relevant code:

- `spandsp-master/src/v34rx.c`: Phase 3 J/TRN logic
- `spandsp-master/src/v34rx.c`: Phase 4 TRN handoff into MP search

### MP transmit path

MP TX is comparatively straightforward.

- `mp_sequence_tx()` serializes the MP frame fields.
- TX starts with `MP0` (`type = 0`), not `MP1`.
- The frame includes sync bits, inserted start bits, type, rates, signalling-rate mask, trellis choice, shaping/non-linear flags, ack bit, and optional precoder coefficients for `MP1`.
- CRC is computed across the 16-bit information payload blocks, excluding sync/start/fill bits.
- `get_mp_or_mph_baud()` scrambles the serialized MP bits and sends them through 4-point differential encoding.
- Once a valid far-end MP is received, TX flips the acknowledge bit and regenerates the frame as `MP'`.
- TX does not send `E` until both:
  - local MP has been acknowledged, and
  - a valid far-end MP with acknowledge set has been received.

Relevant code:

- `spandsp-master/src/v34tx.c`: `mp_sequence_tx()`
- `spandsp-master/src/v34tx.c`: `get_mp_or_mph_baud()`
- `spandsp-master/src/v34tx.c`: `mp_or_mph_baud_init()`

### MP receive path

MP RX is the most interop-heavy part of the implementation.

The receive flow is:

1. Use TRN-derived hypothesis state as the initial guess.
2. Scan for MP preamble:
   - `17` ones,
   - start bit `0`,
   - then the MP type bit.
3. Lock a hypothesis when preamble score is strong enough.
4. Reconstruct a whole MP frame with inserted start bits.
5. Check:
   - CRC,
   - fill bits,
   - start-bit placement,
   - semantic validity of parsed MP contents.
6. If valid, parse MP and apply negotiated settings.

What makes this nontrivial is that RX actively searches over:

- hypothesis index,
- scrambler tap,
- bit order,
- phase domain,
- recovery by bit-slip or start-bit-boundary correction.

The implementation explicitly includes:

- preamble-score gates,
- hint-only mode from the TRN/J hypothesis,
- fallback retry modes that rotate domain/order/tap,
- bit-slip recovery,
- boundary-slip recovery,
- double-boundary-slip recovery,
- boundary brute-force recovery,
- “keep current hypothesis and reacquire locally” behavior after some rejects.

That is all strong evidence that MP interoperability has been difficult and the code has evolved into a robust heuristic decoder rather than a minimal reference implementation.

Relevant code:

- `spandsp-master/src/v34rx.c`: `mp_preamble_score()`
- `spandsp-master/src/v34rx.c`: `mp_phase4_rotate_retry_mode()`
- `spandsp-master/src/v34rx.c`: `mp_unlock_after_reject()`
- `spandsp-master/src/v34rx.c`: `process_rx_mp()`
- `spandsp-master/src/v34rx.c`: `mp_semantic_ok_phase4()`
- `spandsp-master/src/v34rx.c`: `V34_RX_STAGE_PHASE4_MP`

### What looks solid

- MP frame serialization and parsing are reasonably clear and spec-shaped.
- CRC handling is explicit and correctly excludes framing bits.
- TX-side MP/MP' acknowledge behavior is coherent.
- Phase 4 TRN to MP handoff is consciously gated by far-end confirmation instead of pure timeout.

### What still looks fragile

- Phase 4 MP receive is highly heuristic.
- Success depends on carrying the right TRN hypothesis forward.
- There are many recovery modes, which is good for field interop but also a sign the lock process is not fundamentally simple or stable.
- Several comments describe interop guards, fallback timeouts, and forced progress paths.

### Practical interpretation

If a V.34 call is failing late in training, the most likely trouble spot is not TRN transmit generation. It is:

- TRN hypothesis formation on RX,
- MP hypothesis lock,
- MP frame boundary reconstruction,
- or semantic rejection after CRC/fill pass.

That is where additional instrumentation will pay off fastest.

## Recommended next code step

If you want this mapping to become operational rather than documentary:

1. Add debug helpers in SpanDSP V.34 to stringify `tx.stage`, `rx.stage`, and `received_event`.
2. Emit logs only on change.
3. Surface those logs through the existing `SPAN_LOG_FLOW` path.
4. Correlate them with `modem_engine` state transitions and the existing `trace_phase(...)` output.

That will give a much cleaner view of where a call is failing than the current single `ME_TRAINING` bucket.
