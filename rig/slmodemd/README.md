# slmodemd rig patches

Patches applied to the SmartLink slmodemd on the private interop rig (container
`d-modem`, sources under `/src/slmodemd/`). Not upstreamed. See `rig/README.md`
for the d-modem side.

## `v92-pcm-upstream.patch`

Makes the DSP blob accept a peer's V.92 PCM-upstream offer instead of refusing it.

### Why it is needed

The blob implements PCM upstream — `V34SetINFO1aBits` contains both
`V.34 Upstream is selected (info1)...` and `PCM Upstream is selected (info1)...`.
But `V34GiveINFO1dBits` gates it:

```
isPCM = local_cap && remote_cap && (peer INFO1d Table 17 bit 70)
if (!isPCM) return 0
p = *(v34obj + 0xac3c)                  /* = m->dp_runtime */
if ((signed char)p[2] < 0) return 0     /* accept PCM upstream */
log("we got PCM upstream under V.92Lite ..."); InitiateRetrain(90); return 1
```

`dp_runtime` byte 2 is a flags bitfield:

| bit | meaning | written by |
|-----|---------|------------|
| 4 | V.92 mode | `vpcm_create` (set when requested DP is 92) |
| 6 | `DSPINFO[8] & 1` | `dp_runtime_create` |
| 7 | PCM upstream permitted | `dp_runtime_create` — **cleared unconditionally** |

`dp_runtime_create` clears bit 7 on both branches of its `DSPINFO[0xc]` test, and
nothing anywhere in the linked binary ever sets it. So the PCM-upstream path is
complete but unreachable. This patch sets the bit after `dp_runtime_create`.

### Use

Off unless `SLM_V92_PCM_UPSTREAM` is set in slmodemd's environment. It enables code
that has plausibly never executed in this build, so keep it opt-in and always A/B it
against an unpatched run.

```sh
docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz \
  -e SLM_V92_PCM_UPSTREAM=1 -e DM_RESAMPLER=sinc \
  d-modem sh -c "/src/slmodemd/slmodemd -d9 -e /src/d-modem > /tmp/slm.log 2>&1"
```

Confirm with `grep "PCM upstream force-enabled" /tmp/slm.log` (expect
`dp_runtime flags[2] = 0x90` — bit 7 plus bit 4 for V.92).

The digital side must also advertise PCM upstream, or `isPCM` is never true and the
gate is not reached: run `sip_v90_modem` with `ME_V92_PCM_UPSTREAM=1`, and dial with
`AT+MS=92,0,300,56000`.

### Apply / roll back

```sh
docker exec d-modem sh -c "cd /src/slmodemd && patch -p2 < /tmp/v92-pcm-upstream.patch && make"
# rollback:
docker exec d-modem sh -c "cd /src/slmodemd && cp modem.c.bak-prev92up modem.c && make"
# or just restore the binary: cp slmodemd.bak-prev92up slmodemd
```

## TRN2d reference capture (`tools/smartlink_trn2d_reference_wrap.c`)

Settles which of sign / level / timing the peer's Phase-4 Error Energy plateau
(~300-380 vs ~110-146 during DIL) is made of, in the peer's own units.

**What disassembly claimed (2026-07-23) — sign part DISPROVED by the round-3
live pairs, keep only the level-table part.** The static read of
`V90Phase4Demodulator::trn2dKnownDemod()` (dsplibs.o `.text` 0x25de0) was:

```
reference = sign(received) * table[(idx-1) mod 6][law(|received|)]
```

with `law()` = `linear2alaw`/`linear2ulaw` (A-law: `xor 0xD5`; u-law:
`0xFF - x`) and `table` = the per-interval 128-entry int16 level table the
TRN2 design (`findPadGain`) produced, at `[[this+0x3514]]` with the law flag
at `+0xa95c` of that object. The level-table half held up against the
captured pairs; the `sign(received)` half did NOT — the round-3 pairs show
the reference signs mismatching the received signs 50% of the time against
our deterministic +++--- Ri, which `sign(received)` could never do. The
reference sign is therefore DATA-AIDED: the peer regenerates the
transmitter's §5.4.5 sign sequence, and the §5.4.5.6 shaper-metric
convention (`ME_V90_SHAPER_METRIC`, ~31% sign divergence between readings)
is fully visible to its Error Energy. The strict transmitted-levels reading
is now the v90.c default (`ME_V90_SHAPER_METRIC=codec` restores the old
far-codec metric).

**Hook placement:** the compiler inlined trn2dKnownDemod into
`getV90Decision`/`getV92Decision` — dsplibs.o contains **zero relocations**
against it, so interposing on it can never fire (the out-of-line copy is dead
code). The wrap instead interposes on the dispatcher
`V90Phase4Demodulator::getDecision(short)` (`.text` 0x27780, called via
R_386_PC32 relocations from `V90Equalizer::process`) and calls the blob's own
dead-but-linkable trn2dKnownDemod copy for the reference BEFORE forwarding.
The wrap streams interleaved int16 pairs `[received, reference]` to
`/tmp/smartlink-trn2d-pairs.s16` for every Phase-4 `getDecision` call
(`TRN2D_REF_SYMBOLS` caps the count, default 200000; the reference is only
meaningful during TRN2 — segment offline).

### Relink

```sh
# copy the wrap source in
docker cp tools/smartlink_trn2d_reference_wrap.c d-modem:/src/slmodemd/

# inside the container: alias the ORIGINAL DISPATCHER at its .text offset,
# compile the hook (slmodemd is 32-bit i386), and relink with the hook object
# BEFORE dsplibs_trnref.o plus --allow-multiple-definition so the hook's
# getDecision definition wins.  Verify offsets if dsplibs.o ever changes:
#   objdump -t dsplibs.o | grep getDecision       # expect .text 0x27780
docker exec d-modem sh -c "cd /src/slmodemd && \
  objcopy --add-symbol \
    _ZN20V90Phase4Demodulator11getDecisionEs_original=.text:0x27780,global,function \
    dsplibs.o dsplibs_trnref.o && \
  gcc -m32 -O2 -c smartlink_trn2d_reference_wrap.c -o trn2d_wrap.o && \
  gcc -m32 -Wl,--allow-multiple-definition -o slmodemd_trnref \
    modem_main.o modem_cmdline.o modem.o modem_datafile.o modem_at.o \
    modem_timer.o modem_pack.o modem_ec.o modem_comp.o modem_param.o \
    modem_debug.o homolog_data.o dp_sinus.o dp_dummy.o \
    trn2d_wrap.o dsplibs_trnref.o sysdep_common.o"
# verify the wiring (hook wins the symbol, all 3 equalizer call sites bind
# to it, hook forwards to the alias):
docker exec d-modem sh -c "cd /src/slmodemd && \
  nm slmodemd_trnref | grep getDecisionEs && \
  objdump -d slmodemd_trnref | grep -c 'call.*<_ZN20V90Phase4Demodulator11getDecisionEs>'"
```

### Run

Rig side (note the deployed d-modem binary may still default to zoh — the env
overrides it either way; rebuild per `rig/README.md` when convenient):

```sh
# container has no ps/pkill/python3 - kill via /proc scan
docker exec d-modem sh -c 'for d in /proc/[0-9]*; do \
  cmd=$(tr "\0" " " < $d/cmdline 2>/dev/null); \
  case "$cmd" in *slmodemd*|*socat*) kill ${d#/proc/} 2>/dev/null;; esac; done; true'
docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz \
  -e DM_RESAMPLER=sinc -e DM_RS_HEADROOM=0.25 \
  d-modem sh -c "/src/slmodemd/slmodemd_trnref -d9 -e /src/d-modem > /tmp/slm.log 2>&1"
sleep 3   # slmodemd must create /dev/ttySL0 before socat opens it
docker exec -d d-modem sh -c "socat TCP-LISTEN:5556,reuseaddr,fork FILE:/dev/ttySL0,raw,echo=0"
# ALWAYS verify the AT bridge before trusting a run (from the Mac):
(printf 'AT\r'; sleep 3) | nc -w 6 tower.net.cryan.nz 5556    # expect OK
```

`tools/trn2d_call_batch.sh` automates the whole cycle (restart rig, dial,
poll for TRN2REF / NO CARRIER, early-exit on success) — expect to need it;
see the Phase-3 THIRD_S race below.

Server side (Mac) — baseline call (strict transmit metric is now the
default), then the A/B call adds `ME_V90_SHAPER_METRIC=codec` to restore the
old far-codec metric:

```sh
ME_V8_ANSWER_TONE=ansam_pr SIP_FORCE_PCMU=1 \
ME_TRAINING_TIMEOUT_MS=300000 \
ME_V90_J_LOOKAHEAD_BITS=3000 ME_V90_JD_RESYNC_SYMBOLS=48000 ME_V90_SD_DELAY_MS=750 \
VPCM_G711_TAP_DIR=artifacts/v90-hardware/$(date -u +%Y%m%dT%H%M%SZ)-trn2d_ref \
./sip_v90_modem --sip-server asterisk.net.cryan.nz --username 6001 --password 6001 \
  --pty /tmp/v90modem
```

`ME_TRAINING_TIMEOUT_MS=300000` stops our own 60 s fallback from capping
Phase-4 attempts at ~3 per call. Confirm hook liveness with
`docker exec d-modem grep TRN2REF /tmp/slm.log`, then retrieve:

```sh
docker cp d-modem:/tmp/smartlink-trn2d-pairs.s16 .
docker cp d-modem:/tmp/slm.log .
```

### Read-out

The reference LEVELS come from the peer's own findPadGain table, but the
reference SIGNS are data-aided (predicted transmitter sequence — see the
revision above), so sign analysis needs TX alignment against live-tx.g711.
`tools/trn2d_pairs_analyze.py` computes all of:

- **windowed mean `(rx-ref)^2`** — must reproduce the peer's logged
  `Error Energy` curve in its own units (sanity + unit calibration).
- **per-designed-level residual** `|rx|-|ref|` grouped by `|ref|` — a scalar
  or per-Ucode offset here is a pad/resampler-gain error against the
  `findPadGain` table; a fat symmetric residual with mean ~0 is
  ISI/equalizer misconvergence instead.
- **sign agreement** — NOT guaranteed; on a real (post-DIL, post-CPt) TRN2d
  window this directly measures whether the peer's predicted sign sequence
  matches ours, i.e. whether our §5.4.5.6 metric convention matches its
  reference. ~0% mismatch = convention right; ~31% = the codec-vs-transmit
  fork; ~50% = degenerate window (peer demodding Ri, no DIL happened — the
  rounds-3/4 captures are all this) or miswired capture.

Because the reference signs are predicted, `ME_V90_SHAPER_METRIC` DOES
change what the peer measures on a real TRN2d window; the strict transmit
reading is the default as of 2026-07-23.

**RESOLVED 2026-07-23 evening — transmit metric CONFIRMED on a real window.**
Batch-2 call 5 (`artifacts/v90-hardware/20260723T021036Z-trn2d_ref_transmit_metric/`,
`pairs-call5.s16` + `slm-call5.log`, with `ME_V90_DIL_PROFILE=smartlink-adi-qc`
forcing DIL past the Ja-parse lottery): full DIL study → peer designed and
TRANSMITTED CPt {91,87,83,79,72,65,53,33} → our `CPt accepted; TRN2d
(12000 mapped symbols, D=23, K=18)` → captured pairs show **0.0% sign
mismatch over the entire ~2.8 s real window** (~50% in the preceding Ri
region, exactly as the decision tree predicts).  Peer Error Energy through
our TRN2d+MP: **11-17, no plateau** (codec-metric era: pinned 300-380);
timing +0.057 ppm; peer decoded our MP and sent data-mode CP #1-#5.
New (furthest-ever) blocker: the peer's CPs arrive with acknowledge
already set and our strict `v90_set_phase4_cp()` data path rejects an
ack'd CP before a plain one (`accepted=0`), so we never answer Ed/B1d and
the peer hits `Phase4 TimeOut` → DP=90 retrain.  Fix the CP-ack
acceptance path next.

**CP-ack fix VALIDATED + FIRST COMPLETED V.90 HANDSHAKE (2026-07-23,
batch-4 call 2, `.../b4/`).** With CP'-first acceptance in
`v90_set_phase4_cp()`: peer transmitted a real CPt, then CP' (`kind=CP
bits=700 drn=19 ack=1 accepted=1` — first shot), we answered MP' → Ed →
B1d → **`V.90 startup complete (upstream V.34 31200 bps, downstream PCM
52000 bps)`**, and the peer logged `enter Data Phase, Rate = 52000
[bps]`.  Both sides in V.90 data mode for ~10 s.  The pairs reproduce
the metric verdict on a second constellation: 0.0% sign mismatch for
the entire 3.0 s mapped window (TRN2d through data phase).
**Next blocker — upstream rate agreement:** the peer died with
`Tx bit rate - 33600, Rx bit Rate - 0` → `vpcm: Link Error` ~10 s in.
`v90_build_mp_type0()` echoes the peer's own CPt
`upstream_rate_mask` back verbatim in MP bits 36:48 (and bits 24:27's
max-drn), never intersecting it with the rate our V.34 upstream
receiver actually trained at (31200 here) — so the peer picks 33600 and
transmits into a mismatched pump.  Cap the echoed mask/drn at
`v34_get_current_bit_rate()` before building MP.

### Phase-3 THIRD_S deadlock (why calls died before Phase 4)

The earlier "THIRD_S race" reading (peer starts its S, our
detection+turnaround latency decides) is DISPROVEN, by two independent
captures on 2026-07-23:

- G.711 taps of 7 straight failed attempts show ZERO upstream S energy in
  any Jd window — the peer never transmitted its §9.3.2.7 S at all.  (Those
  7 were the other family: `IndicateJdReceived` never fired, peer silently
  hit its Jd deadline and retrained — Ja tail → ~2.7 s silence → INFO0a →
  tone A.)
- The tower log of a `IndicateJdReceived`-reaching call shows the peer's TX
  state going `JaTXMIT => SILENCERETRAIN` at the drop — no S-transmit state
  ever entered.

The actual mechanism was a deadlock we caused: §9.3.2.7 lets the analogue
modem wait **up to 5000 ms from silence start** before transmitting S, but
`ME_V90_JD_RESYNC_SYMBOLS=24000` capped our Jd at 3.0 s (≈ silence+3.3 s).
We abandoned Jd while the peer was still lawfully waiting; its "Error
Energy explosion" (12 → ~1145, then `drop to V34` ~120 ms later) is
timestamp-aligned with our own Jd ending, not with any mismatched
reference.  Our Phase-3 S detector has therefore never been shown a real S.

Fix: `ME_V90_JD_RESYNC_SYMBOLS=48000` (6.0 s) covers the peer's full
discretionary window; §9.3.1.4 puts no upper bound on Jd duration on the
digital side.  Residual attempt failures to still expect: the peer failing
to decode Jd at all (retrain family above — includes the
`Error Energy = -0.000` WaitForSd misses and the ~2500-flat unconverged
equalizer), and possibly our 750 ms `ME_V90_SD_DELAY_MS` brushing the
peer's 1500 ms §9.3.2.4 Sd-S̄d deadline (two of the seven taped attempts
show Ja persisting through our Sd — shorten the delay if that family
dominates a batch).  Batch calls with `tools/trn2d_call_batch.sh`.

### 2026-07-23: rate cap + Phase-2 rework LIVE-VALIDATED — 3/6 calls to sustained data mode

6-call batch at HEAD 8b06612 (fresh `docker restart d-modem`, sinc resampler,
standard env recipe, `ME_V90_SD_DELAY_MS=750`), artifacts in
`artifacts/v90-hardware/20260723T045437Z-phase2_tones_ratecap/`:

- **Calls 1/2/5: complete V.90 handshake to data mode** (upstream V.34
  31200 / downstream PCM 52000) in ~26 s, `[V90] MP upstream rate cap:
  31200` fired (first live firings of the 8ae49e1 fix) and the peer entered
  data with `Tx bit rate - 31200` — equal to the cap — every time.  The
  33600-into-a-31200-pump `Link Error` at ~10 s is gone: holds measured
  >240 s, ~470 s (ended only by batch teardown; Error Energy ~3.2 flat
  throughout), and >240 s.
- **Phase 2: zero aborts in all 5 real calls**, including the retrained
  re-handshakes inside calls 3/4.  The durable-milestone rework (8b06612:
  `v90_phase2_reversals_consumed`/`l2_consumed` replacing `received_event`
  as sequencing memory, §9.2.1.2.4 900 ms+RTD third-reversal recovery
  replacing the empirical timeout) removed the INFO1a-timeout/
  reversal-thrash lottery entirely.
- Calls 3/4: identical late failure — peer builds CPt, fires two
  recoverable DP=90 retrains (both followed), then `drop to V34` → Link
  Error.  The post-CPt `exitPhase3 delayedRetrainRequest` family is now
  the sole remaining protocol blocker.
- Call 6: rig fatigue (d-modem dial died at SIP level 1.5 s after ATD;
  nothing reached the server) — the documented ~5-call degradation.

### 2026-07-23: FIRST END-TO-END V.90 DATA TRANSFER — PTY throughput soak

Soak harness in `tools/soak/` (PTY pump + far-DTE socket pump + orchestrator
with early abort + integrity analyzer).  Result (final/v2attempt8 in
`artifacts/v90-hardware/20260723T055443Z-pty_soak/`): **99.98% of bytes
delivered downstream** (351177/351232), 38825 pattern lines in perfect
sequence (0 dup / 0 out-of-order, 0.5% line loss at teardown), ~40 kbps
sustained at the offered load over the full 110 s three-phase schedule,
readable byte-exact data at the far DTE.

Hard-won facts (three soak iterations to get here):
- **V.14 LSB-first character order is correct** — live A/B: the msb
  experiment (`ME_V90_DATA_BIT_ORDER=msb`) delivers every byte bit-reversed
  at the far DTE in otherwise perfect sequence.  slmodemd's DSP-layer
  `rx pattern` debug bytes print bit-reversed relative to its DTE output —
  never infer the wire convention from that log.
- **`AT\N0` in the dial string is required**: without it slmodemd's V.42
  auto-detect chews on raw test data forever and never issues CONNECT to
  its DTE (the call looks connected at the DSP but the DTE side is dead).
- Reversing at the mapper-byte boundary is doubly wrong (relocates V.14
  start bits) — produces pseudo-random DTE output at the right char rate.
- **Upstream data is confirmed unwired**: far DTE transmitted 212 KB of
  V.34 31200 upstream data; 0 bytes reached the PTY.  This is the next
  data-path feature (V.34 upstream RX → data stack → PTY).
- **Phase-4 "Null MP" coin flip is the availability bottleneck**: every
  call logs `V90Phase4Modulator: ERROR: Null MP @ end of TRN2d`; survivors
  are the calls where `V90MP: MP detected` follows ~4.6 s later during the
  CP barrage.  Losers carousel (peer retrains; our fixed 1550 ms
  `ME_V90_SD_DELAY_RETRAIN_MS` misses its rushed re-ranging window every
  time) and die.  Observed win rate: 3/6 morning, ~1/8 evening; the
  identical `Error Energy +3191.517` on losing calls is the deterministic
  carousel signature, not a distinct failure.  Two fix avenues: make the
  peer's MP lock reliable (study what differs in the winning calls' MP
  alignment), and adaptive retrained-attempt Sd timing so lost flips
  recover.

### 2026-07-23 late: upstream data path WIRED — structure validated live, decode alignment open

V.90's upstream Phase 4 is the CP dance, not a V.34 MP exchange, so SpanDSP's
mp_seen-gated E detector could never fire for the answerer: the RX sat in
PHASE4_MP forever and data-mode upstream delivered exactly 0 bytes.  Now:
`v34_v90_prepare_upstream_data()` (new; sets rate/trellis/parms WITHOUT
touching the hypothesis lock that keeps the live phase-4 bit tap alive) is
called when the peer's acknowledged CP' is accepted, the engine watches the
same tap for the peer's E (20 consecutive ones, §9.4.2 `CP' CP' E B1→data`),
then `v34_begin_rx_data()` flips the RX to DATA.

Live result (upstream/v2attempt6 in the 20260723T055443Z-pty_soak dir):
"upstream RX data prepared (31200 bps)" → "upstream E detected" → **PTY
received 192 KB upstream (was 0)** at ~19-25 kbps while downstream stayed
clean (99.4%).  BUT the decoded stream is garbled: 0 U-lines in either bit
order, and bytes arrive even while the far DTE is idle — correct decode of
V.14 idle marks would produce *no* deframed bytes, so the decoded bitstream
itself is wrong, not just byte framing.  Suspects, in order: (1) mapping-
frame/E-boundary alignment (the CP'-fill ones-join can shift the 20-ones
fire point, and B1's mapping-frame boundary sets the permanent frame
grouping); (2) data transform rotation (scale 70 was measured on reference
31.2k calls and is probably right); (3) superframe/V0 alignment.  Debug
artifact preserved: `upstream/v2attempt6/live-rx.g711` (winner call at the
tail) + the 192 KB garbled `rx_pty.bin`.  Next: offline sweep of begin
offset (0..15 symbols) x rotation (0..3) x conjugate against that capture
until U-lines / idle-ones emerge, then encode the offset into the live E
handler.  Also gated the DATA-stage per-frame stderr RMS log behind
`V34_DATA_FRAME_RMS_LOG` (~400 lines/s — a media-clock hazard of the
buffered-tap class).

### 2026-07-23 latest: offline upstream alignment sweep — decode chain mostly works

The live-garble question was moved fully offline.  `winner-stereo.wav` (in
`.../pty_soak/upstream/v2attempt6/`) is the winning call cut from the taps
(L=digital TX, R=analog RX; the ANSam-onset cut script is
`tools/soak/../cut_winner_call.py` pattern in session scratchpad).  New
vpcm_decode env hooks (committed): `VPCM_V90_ASSUME_MP_RATE` (bypass PCM-side
MP recovery with the known negotiated parameters), `VPCM_V90_PHASE3_START_
SAMPLE` (window the replay to one attempt), `VPCM_V90_BOUNDARY_RANGE/STEP`
(fine boundary sweep).  Reproduction:

```
VPCM_V90_ASSUME_MP_RATE=31200 VPCM_V90_PHASE3_START_SAMPLE=1697000 \
VPCM_V90_PHASE4_JPRIME_SAMPLE=1757000 VPCM_V90_FORCE_DATA_SAMPLE=1813754 \
VPCM_V90_BOUNDARY_SWEEP=1 VPCM_V90_NATIVE_ONLY=1 VPCM_V34_UPSTREAM_DIAG=1 \
./vpcm_decode --all --wav winner-stereo.wav
```

Anchoring trick: TRACE phase-ms ↔ WAV samples via the DATA state-change ME
trace (`phase_ms=0 g711_rx=9602790` ↔ WAV sample 1812018; counters ≈ tap
byte offsets because tap and counters share the server-instance epoch).

Findings: boundary locks at **1813752-54** (score 67 vs 18 noise floor;
1-sample fine sweep adds nothing).  Transform: scale 70 correct, rotation
0 vs 2 near-tied (differential coding absorbs it), conjugate 0.  The decoded
stream at the best config is **mostly-correct idle ones** — long clean 0xff
runs (one alignment: transient then 34 straight ff bytes) broken by bursty
errors at ~mapping-frame spacing, plus a permanent ~24-bit windup transient
at the boundary that dominates the first64 score (score is misleading —
read the aligned_hex dumps).  So the DATA decode chain fundamentally works;
the remaining upstream-garble work is (a) the bursty per-mapping-frame error
source at 31200@3200 (V0/superframe bit handling and shell unmap are the
suspects), (b) skipping the windup transient before scoring/DTE delivery.

### Upstream garble root-cause narrowed: DATA-stage symbols are off-constellation

Systematic elimination on winner-stereo.wav (all offline, deterministic):
- Descrambler taps 4/17, both shift conventions: raw pre-descrambler stream
  is already 50% — not the scrambler.
- Boundary: fine 1-sample sweep ±40 around the E estimate with corrected
  post-windup scoring (ones in decoded bits 300..1300 — the old first64
  metric sat inside the Viterbi windup and was blind): flat ~51% everywhere.
- Transform: scale x rotation x conjugate sweep with corrected scoring —
  flat ~51%.
- parms: aux channel w=0 confirmed correct (internal rate code 24 = plain
  31200, no +200 bps aux).
- Decisive: fitting the exported equalized data-region symbols to the
  odd-integer Q9.7 constellation grid across scales 15..120 — NO scale
  beats the uniform-random baseline (0.565 vs 0.577 rms).  The CP/MP-region
  symbols do cluster (4-point-ish), so demod is sane pre-DATA.

Conclusion: the symbol stream consumed in V34_RX_STAGE_DATA is not
decision-point samples — either the DATA-stage equalizer/timing output
genuinely collapses at the stage switch (adaptation freezes there:
DD-tracking is gated PHASE3_WAIT_S..PHASE4_MP only), or the QAM export tap
misrepresents what put_mapping_frame consumes.  NEXT STEP: dump
s->mapping_frame_buf (the exact int16 Q9.7 values entering
v34_put_mapping_frame) env-gated, offline AND live, and check grid fit;
then either fix the DATA-stage demod/tracking or the buffering.  Sweep
tooling committed: VPCM_V90_NATIVE_MAX_BITS, VPCM_V90_NATIVE_BITS_OUT
(decoded bitstream dump; idle = all-ones ground truth), post-windup sweep
scoring, VPCM_V90_BOUNDARY_RANGE/STEP.

### Upstream demod root cause FOUND (spec-confirmed): the V.90 answerer RX has no coherent lock

mapping_frame_buf dump (V34_DATA_FRAME_DUMP env, exact int16 Q9.7 into
v34_put_mapping_frame) confirms the decoder input is off-constellation mush
(mean |v| 63 grid units vs max 43, no grid fit at any scale).  4th-power
analysis of the equalized symbols shows coherence ~0.02 in the DATA region
AND in the CP region — the receiver is phase-locked NOWHERE in Phase 4.
CP/MP decode succeeds anyway because §8.5.2/V.90 makes those sequences
DIFFERENTIALLY encoded ("modulated according to 10.1.3.9/V.34; the
scrambler and differential encoder are initialized to zero"), and the
24-hypothesis machinery absorbs rotation — the Phase-3 "TRN lock" and
Phase-4 "MP hypothesis lock" are bit-domain latches, not carrier locks.
Data mode (absolute trellis+mapping per clause 6/V.90 → V.34) is the first
consumer that needs true coherence.  Eliminated en route: pre-emphasis
change at B1 (none — 9.2/V.34 line 2191: "All signals in Phases 3 and 4
are transmitted using the selected symbol rate, carrier frequency,
pre-emphasis filter and power level"), scale/rotation/conjugate, boundary,
descrambler, aux bits.

Landed (necessary, not yet sufficient):
- `v34_begin_rx_data()` now applies the 10.1.3.1/V.34 B1 exception: B1 is a
  reset-state data frame carrying the superframe-final V0 sync inversions,
  and the trellis starts from state zero (was: ordinary frame zero — the V0
  inversion pattern was out of phase for the whole connection).
- Decision-directed carrier tracking in V34_RX_STAGE_DATA (same
  Im(sym x conj(target)) detector as training, target = nearest odd-integer
  constellation point, error normalized to sin(dphi); 90-degree ambiguity
  absorbed by the differential quadrant bits).  ME_V34_DATA_CARRIER_TRACK=0
  disables.  DD can HOLD a lock but cannot ACQUIRE from the current unlocked
  state — offline repro still decodes 50%.

THE REMAINING WORK (a coherent-receiver workstream, not a patch): data-aided
acquisition before DATA — the peer's TRN (scrambled ones, known) and B1
(known symbol frame, 10.1.3.1/V.34) both provide known-symbol windows to
estimate true carrier phase/equalizer response and seed the loops; then the
existing freeze machinery holds through MP and the new DD tracking holds
through data.  All offline-testable against winner-stereo.wav with the
committed repro before any live call.
