# V.8bis Decode Improvement Plan

## Goal

Fix the specific structural and algorithmic defects in `v8bis_decode.c` and
`phase12_decode.c` that prevent reliable V.8bis QC2/QCA2 identification during
V.92 short Phase 1 startup.  Every fix is tied directly to an ITU-T spec
requirement.

Primary references:

- `ITU Docs/T-REC-V.8bis-199608-S!!PDF-E-1.pdf` — V.8bis (08/96)
- `ITU Docs/T-REC-V.92-200011-I!!PDF-E.pdf` — V.92 (11/2000)

---

## Background: QC2/QCA2 signal structure

V.92 §8.2.2 / §8.2.4 / §8.3.3 / §8.3.5 define four V.8bis identification
messages:

| Signal | V.21 channel | Who sends it | Digital? | QCA? | V.21 mod |
|--------|-------------|--------------|----------|------|----------|
| QC2a   | CH2 (H)     | analogue (responding) | no | no | V.21(H) |
| QCA2a  | CH1 (L)     | analogue (initiating) | no | yes | V.21(L) |
| QC2d   | CH2 (H)     | digital (responding)  | yes | no | V.21(H) |
| QCA2d  | CH1 (L)     | digital (initiating)  | yes | yes | V.21(L) |

The expected pair depends on which modem answered (and is therefore the
"initiating" V.8bis station):

- Digital modem answers: digital modem sends CRe → analogue responds with
  **QC2a on CH2** → digital acknowledges with **QCA2d on CH1**
- Analogue modem answers: analogue modem sends CRe → digital responds with
  **QC2d on CH2** → analogue acknowledges with **QCA2a on CH1**

All QC2x messages arrive on CH2 (V.21 H, 1750/1850 Hz).
All QCA2x messages arrive on CH1 (V.21 L, 1080/1180 Hz).

All four messages use msg_type = `0xB` (1011b) in the V.8bis identification
field (V.92 Tables 3, 5, 12, 14).

---

## Defect Catalogue

### Defect 1 — Critical: dedup eliminates the QCA2 partner of QC2

**Spec reference:** V.92 §8.2 and §8.3; V.8bis §7.2 (distinct messages on
distinct channels must both survive to the call log).

**Location:** `v8bis_decode.c` `v8bis_collect_msg_events()`, the inner dedup
loop (lines ~642–667).

**Problem:** The dedup key is `msg->msg_type` alone:

```c
if (last_emitted_type[k] == msg->msg_type
    && abs(msg->sample_offset - last_emitted_sample[k]) < 3200) {
    is_dup = true;
```

QC2a (CH2, msg_type=0xB) and QCA2d (CH1, msg_type=0xB) share the same
msg_type.  They occur ~200–400 ms apart — well inside the 3200-sample
(400 ms) window.  The second one to arrive is dropped.

The same bug applies to the partial-frame dedup block that immediately follows.

**Fix:** Include the FSK channel in the dedup key:

```c
uint8_t dedup_key = msg->msg_type | (uint8_t)(msg->channel << 4);
```

Use `dedup_key` instead of `msg->msg_type` when building `last_emitted_type[]`
and when comparing.  Apply the same change to the partial-frame dedup block
(same structure, `last_partial_type[]`).

---

### Defect 2 — Significant: reserved-bit checks are too strict

**Spec reference:** V.8bis §8.2 compatibility note — "receivers shall parse
all information blocks and ignore information that is not understood."  V.92
Tables 3, 5, 12, 14 NOTE — "The receiving modem shall ignore [the revision]
field."

**Location:** `v8bis_decode.c` `v92_decode_qc2_id()` (~lines 330 and 340).

**Problem:** Two hard return-false guards reject valid frames:

```c
/* QC2a / QCA2a: bit 12 reserved */
if ((b & 0x10U) != 0)
    return false;

/* QC2d / QCA2d: bits 10:12 reserved */
if ((b & 0x1CU) != 0)
    return false;
```

Real hardware sometimes sets reserved bits non-zero.  Silently discarding these
frames means QC2/QCA2 events are never reported.

**Fix:** Replace each hard-reject with a logged warning that sets a new
`reserved_bits_set` flag in `v92_qc2_id_t` and continues decoding:

```c
out->reserved_bits_set = ((b & 0x10U) != 0);  /* analogue case */
/* do NOT return false */
```

Add `bool reserved_bits_set;` to `v92_qc2_id_t`.

Callers and the call-log detail string should include this flag when set so
the anomaly is visible in output without blocking decode.

---

### Defect 3 — Significant: no V.21 channel validation for QC2/QCA2

**Spec reference:** V.92 §8.2.2 (QC2a: V.21(H)), §8.2.4 (QCA2a: V.21(L)),
§8.3.3 (QC2d: V.21(H)), §8.3.5 (QCA2d: V.21(L)).

**Location:** `v8bis_decode.c` `v92_decode_qc2_id()` and the call-log emitter
that follows.

**Problem:** The identification function decodes the payload fields and names
the signal correctly but never checks that the decoded signal arrived on the
expected V.21 channel.  A QC2a decoded from CH1 or a QCA2a from CH2 is
anomalous — it indicates a channel-assignment error or a false decode — but
currently nothing flags it.

**Fix:**

1. Add `int expected_channel;` to `v92_qc2_id_t` and populate it:
   - QC2x (not QCA): expected_channel = 1 (CH2, V.21 H)
   - QCA2x (QCA): expected_channel = 0 (CH1, V.21 L)

2. Add `bool channel_mismatch;` to `v92_qc2_id_t` and set it if
   `msg->channel != expected_channel`.

3. In the call-log detail string, append `channel_mismatch=yes` when set.
   Do not suppress the event — just mark it.

4. In `phase12_decode.c` `detect_call_initiation_signals()`, when selecting
   which QC2/QCA2 event to promote into `call_init`, prefer events that have
   `channel_mismatch=no` (i.e. parse the `channel_mismatch` field from the
   detail string, or carry the flag directly).

---

### Defect 4 — Moderate: no preamble validation before HDLC flag

**Spec reference:** V.8bis §7.2.4 — "Each message shall begin with 100 ms ±
2% of continuous V.21 marking frequency" (= 30 marking bits at 300 bps).

**Location:** `v8bis_decode.c` `v8bis_hdlc_put_bit()` (~line 489).

**Problem:** The HDLC receiver accepts a frame-start flag after any number of
marking bits, including zero.  This allows false frame locks from short noise
bursts that accidentally produce an HDLC flag pattern without the mandatory
30-bit marking preamble.

**Fix:**

1. Add `int preamble_ones;` to `v8bis_hdlc_rx_t` (count consecutive ones
   before the first opening flag).

2. In `v8bis_hdlc_put_bit()`, increment `preamble_ones` each time
   `!rx->in_frame` and the received bit is 1.  Reset it on any 0 bit while
   still outside a frame (a 0 bit that isn't a flag means the run is over).

3. When the first flag fires (transitioning `in_frame` from false to true),
   record `rx->preamble_ones` into a new `rx->frame_preamble_ones` field.

4. In `v8bis_hdlc_commit_frame()`, if `rx->frame_preamble_ones < 24`
   (i.e. less than 80 ms of marking at 300 bps — allowing for ±2% timing
   tolerance beyond the nominal 30 bits), record the frame as a partial with
   a new reason `V8BIS_PARTIAL_SHORT_PREAMBLE` instead of silently accepting
   it.

   Threshold of 24 bits (instead of the spec's 30) accommodates the ±2%
   tolerance and SpanDSP's carrier-up latency.

---

### Defect 5 — Moderate: opening flag count not validated

**Spec reference:** V.8bis §7.2.5 — "At least two but no more than five flags
shall be sent to begin each message."

**Location:** `v8bis_decode.c` `v8bis_hdlc_put_bit()`.

**Problem:** Every flag unconditionally closes the previous frame and opens a
new one.  There is no check that at least two consecutive flags preceded the
information field.  A single flag (which can arise from noise coincidence with
the 0x7E pattern) creates a frame context with no validation requirement.

**Fix:**

1. Add `int opening_flag_count;` to `v8bis_hdlc_rx_t`.

2. In the flag-detection branch of `v8bis_hdlc_put_bit()`:
   - If `!rx->in_frame`: set `in_frame = true`, set `opening_flag_count = 1`,
     clear frame buffer.
   - If `rx->in_frame` and `rx->frame_byte_count == 0` (no data bytes yet,
     so this flag is another opening flag): increment `opening_flag_count`.
   - If `rx->in_frame` and `rx->frame_byte_count > 0` (data is present, so
     this is the closing flag): call `v8bis_hdlc_commit_frame()`.

3. In `v8bis_hdlc_commit_frame()`, add a partial-reason
   `V8BIS_PARTIAL_SINGLE_FLAG` if `rx->opening_flag_count < 2`.

4. Include `opening_flag_count` in the partial-frame detail log.

---

### Defect 6 — Moderate: phase12_decode.c discards all QC2/QCA2 events after the first

**Spec reference:** V.92 §8.2, §8.3 — QC2 and QCA2 are paired signals
carrying independent information from each modem; both must survive.

**Location:** `phase12_decode.c` `detect_call_initiation_signals()` lines
~1620–1637.

**Problem:** The loop that promotes V.92 events into `call_init` breaks after
the first matching QC2/QCA2 event:

```c
result->call_init.v92_qc2_seen = true;
...
break;   /* ← discards QCA2 partner */
```

Only one of the pair (QC2 or QCA2) is saved; the other is silently lost.
The `v92_qc2_*` fields in `p12_call_init_t` have no slot for the partner.

**Fix:**

1. Add a second set of fields to `p12_call_init_t` for the partner:
   ```c
   bool v92_qca2_seen;
   int  v92_qca2_sample;
   char v92_qca2_name[16];
   bool v92_qca2_digital;
   int  v92_qca2_uqts_ucode;
   int  v92_qca2_lm_level;
   ```

2. Remove the `break` and instead dispatch on the `QCA` flag:
   - Summary starts with "QCA": fill `v92_qca2_*` fields.
   - Summary starts with "QC" (but not "QCA"): fill `v92_qc2_*` fields.

3. Initialize the new fields in `phase12_result_init()` (ucode/lm to -1).

4. Export the QCA2 partner event in `phase12_merge_to_call_log()` and
   `p12_append_phase1_events_from_call_init()` analogously to the existing
   QC2 export.

5. In the effective-uqts_ucode / effective-lm_level calculation, prefer
   `v92_qca2_*` fields when the QCA2 partner is from the digital side (i.e.
   QCA2d carries LM, which is the ANSpcm level that matters for Phase 2
   initialisation).

---

### Defect 7 — Minor: sample offset uses chunk-start, not exact carrier-on sample

**Spec reference:** Not a spec compliance issue; affects accuracy of the Phase
1 event timeline.

**Location:** `v8bis_decode.c` `v8bis_hdlc_put_bit()` (~line 493):

```c
if (bit == -2)
    rx->carrier_on_sample = rx->current_chunk_sample;
```

**Problem:** `current_chunk_sample` is the sample index at the start of the
current 160-sample chunk.  The SpanDSP FSK carrier-up event can fire at any
point within that chunk, introducing up to 20 ms (~160-sample) error.

**Fix:** SpanDSP's `fsk_rx` does not expose a sub-chunk sample offset for the
carrier-up event.  The practical fix is to record the midpoint of the chunk
rather than its start, halving the worst-case error:

```c
if (bit == -2)
    rx->carrier_on_sample = rx->current_chunk_sample + 80;
```

This is a minor improvement; full sub-sample accuracy would require replacing
the bulk `fsk_rx()` call with a sample-by-sample loop, which has a
significant performance cost and is not warranted.

---

## Priority and sequencing

| Priority | Defect | File | Risk of regression |
|----------|--------|------|--------------------|
| 1 (do first) | Defect 1 — dedup channel key | `v8bis_decode.c` | Low — isolated to dedup table key |
| 2 | Defect 2 — reserved-bit strict checks | `v8bis_decode.c` | Low — changes false to log+continue |
| 3 | Defect 6 — QC2/QCA2 partner retention | `phase12_decode.c` | Medium — adds fields, removes break |
| 4 | Defect 3 — channel mismatch flag | `v8bis_decode.c` + `phase12_decode.c` | Low — diagnostic only |
| 5 | Defect 4 — preamble validation | `v8bis_decode.c` | Low — new partial reason only |
| 6 | Defect 5 — opening flag count | `v8bis_decode.c` | Low — new partial reason only |
| 7 | Defect 7 — sample offset midpoint | `v8bis_decode.c` | Trivial |

Defects 1, 2, and 6 are the ones most likely causing the QC2/QCA2 events to
be invisible in the Phase 1 timeline.  Start there.

---

## Verification approach

After each defect fix:

1. Run `make vpcm_decode` — must build cleanly.

2. On the Agere QC capture (`Agere-SV92-QC.wav`):
   - Both a QC2x event and a QCA2x event must appear in the Phase 1 timeline.
   - They must be on different V.21 channels (one CH1, one CH2).
   - Neither should be marked as a dedup survivor suppressing the other.

3. Confirm that `channel_mismatch=no` on both events (Defect 3 fix).

4. Confirm that the `v92_qca2_seen` flag is set and `v92_qca2_*` fields are
   populated (Defect 6 fix).

5. On an NC capture: confirm no spurious QC2/QCA2 events appear that were
   not present before the fix.

---

## Status

- All 7 defects implemented and build-verified on 2026-04-05.
