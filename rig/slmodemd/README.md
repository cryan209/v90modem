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
