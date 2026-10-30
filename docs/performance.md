# Real-time performance and callback baselines

The performance goal is predictable RTP pacing, not just low average CPU.
A callback which blocks for 100 ms delays the transmitted stream even if the
process spends the rest of the second idle. Preserve G.711 octets and sample
accounting; do not fix a scheduling problem by inserting/deleting samples,
resampling, or adding an elastic audio buffer.

## Compact per-call measurements

`modem_engine.c` collects wall-time histograms for `me_rx_g711()` and
`me_tx_g711()` regardless of verbose logging. On SIP disconnect, or clean engine
shutdown, it writes at most six `[PERF]` summary lines per call:

- `call`: process-local call epoch, not a SIP Call-ID;
- `dir`: `rx` or `tx`;
- `phase`: state at callback entry, grouped as `training` (V.8 or training),
  `data`, or `other`;
- `calls`, `audio_ms`, `wall_ms`, and `mean_us`;
- `p95_us_upper` / `p99_us_upper`: approximate histogram upper bounds;
- `max_us`: exact maximum measured wall time;
- `over_budget`: callbacks longer than their block's duration at 8000 Hz;
- `transitions`: callbacks whose engine state differed between entry and exit.

The histogram has four subdivisions per power of two in microseconds.
Percentiles are upper bounds, capped by the exact maximum, **not exact
percentiles**. Sub-microsecond timings are rounded into the first bucket.
Counters use fixed memory (about 6.3 KiB per engine), with no per-callback
allocation or summary formatting. State and histogram access share the engine
state mutex; timing includes the entry and final bookkeeping lock waits.
Call epochs reject updates from callbacks spanning teardown/new-call reset.
Only callbacks completed before teardown's snapshot are included.

**Scope limitations:** this is G.711 engine wall time, including nested linear
DSP and RX link publication, not CPU time or a complete transport profiler.
It excludes the final histogram update and optional verbose output itself.
Direct `me_rx_audio()` / `me_tx_audio()` callers, separate 16 kHz frontends,
worker CPU, and PJSIP work outside the engine wrappers are not included.
RX and TX are measured independently: zero over-budget callbacks does not
prove their combined work plus transport fits one packet deadline. A state
transition that leaves and returns to the same state within one callback is
not counted. Training and retraining share the training bucket; per-retrain
subphase attribution remains future work.

`ME_SLOW_CALLBACK_US` controls only the optional verbose detail threshold,
not the block-derived deadline. It is cached at each call boundary. At most
four slow-callback detail messages are emitted per call; the summary still
counts all over-budget callbacks. Quiet calls skip the inner DSP section
clock reads and the training TX diagnostic RMS sample scan/square root.
Neither optimisation changes the DSP invocation or captured sample stream.

Read summaries with:

```bash
python3 tools/media_perf_report.py modem.log.1 modem.log
python3 tools/media_perf_report.py --json modem.log > perf.json
```

Rows remain separate by log/call/direction/phase. The tool does not average
percentiles across calls or peers. A missing summary is an error, not a zero
latency score. A crash or forced kill cannot produce a clean end-of-call
summary; use external profiling when investigating those failures.

## Repeatable offline baseline

Build the current working tree with vendored libraries:

```bash
make media_perf_test v90_engine_peer engine_pair_test
./media_perf_test
python3 tools/test_media_perf_report.py
```

For a small V.34 training-plus-data baseline:

```bash
ENGINE_PEER_QUIET=1 ENGINE_PAIR_KEEP_LOGS=1 ./engine_pair_test \
  --both-at 'AT+MS=V34' --expect V34 \
  --payload-printable PERF --payload-reps 32 --hold-seconds 3 --seconds 40
```

The test prints the two retained peer log paths. Pass those paths to
`tools/media_perf_report.py`. Repeat at least three times sequentially, then
repeat with `--alaw`. `ENGINE_PEER_QUIET=1` is a test-driver switch: ordinary
engine-pair tests retain their verbose default. The production server is
already quiet unless verbose diagnostics are requested.

Use the same machine, compiler/library build, block size, payload, diagnostic
environment and host load for comparisons. Some legacy symbol/codeword/text
traces still do synchronous I/O. Use a private `ME_DUMP_DIR` for each run and
keep its storage constant across arms. The G.711 and training PCM taps below
use background writers; keep their configuration constant unless capture
overhead is the A/B variable. Record revision and dirty-tree status. Alternate A/B
arms, let every run finish, and require payload correctness as well as timing.
Use an OS profiler (`sample`/Instruments on macOS, `perf` on Linux) for CPU
hotspots; `wall_ms / audio_ms` is not CPU utilisation. Use an OS timing tool
which accounts for children when measuring worker/process CPU.

A local six-run smoke baseline (three per law) exchanged V.34 payload in all
six calls. This is a working-tree sanity check, not an optimisation speedup
claim, clean regression baseline, or hardware interoperability result. The
host and vendored DSP tree had unrelated ongoing changes. Live qualification
must run on the wired rig, following [hardware interop](v90_hardware_interop.md).

## Background diagnostic captures

`async_capture.c` isolates disk I/O for these diagnostic copies:

| Switch | Raw files (unchanged formats) |
|---|---|
| `VPCM_G711_TAP_DIR` | `<dir>/live-rx.g711`, `<dir>/live-tx.g711` |
| `ME_G711_CAPTURE` | `<prefix>.rx.ulaw` (legacy name, also used for A-law) |
| Non-empty `ME_DUMP_DIR` | `<dir>/v34_rx.raw`, `<dir>/v34_tx.raw` (selected training PCM) |

Training PCM now requires a non-empty directory, for headless and GUI engines
alike: a standalone engine no longer silently records to `/tmp` without it.
Use `ME_DUMP_DIR=/tmp` to retain the old destinations, but use private
directories when running multiple engines. For standalone engines,
unset/empty disables PCM capture. The multi-line supervisor supplies a private
`ME_DUMP_DIR` automatically when unset/empty, so its workers still enable these
dumps. An explicitly supplied directory must already exist. Other diagnostics
using this directory have their own activation rules.

The settings are latched at engine initialisation. Setup creates/truncates the
files on the control thread. Use unique paths per engine. The multi-line
supervisor isolates both the tap directory and `ME_G711_CAPTURE` prefix per
worker (including their status sidecars); explicit `%L` templates also work.
Each G.711 stream has a **256 KiB diagnostic queue**, and each training PCM
stream **512 KiB** (two bytes per 8 kHz sample for the same headroom), plus a
private writer thread and its stack. Disabled streams allocate no
queue or worker. Queue storage is touched during setup to avoid initial
producer-side page faults (this is not `mlock` or a hard real-time guarantee).
The media producer copies a whole block and publishes it using lock-free
atomics; it does not allocate, acquire a mutex, signal a condition variable,
log, or access the filesystem. One serialized producer owns each stream.
The worker polls at 10 ms intervals and does the actual disk writes. This is
latency isolation, not a claim of reduced total CPU: enabled workers add
polling and stack overhead. Unsupported non-lock-free atomic implementations disable capture, rather than
falling back to locks or synchronous writes. Only regular files are accepted.
Setup also takes non-waiting advisory file locks, held until close, before
truncation. An active recording (including a raw hard-link alias) cannot be
overwritten by another cooperating capture handle. A conflict/unsupported
lock disables that stream, never waits or falls back to media-thread I/O.
Use unique paths regardless: locks protect individual files, not ownership
of an RX/TX pair, and unrelated tools can ignore advisory locks.

**Fail-stop, not lossy continuation:** if the queue fills, reject the entire
new block and all subsequent blocks for that stream until engine restart.
The worker drains the already accepted prefix. If a disk write fails, retain
only the prefix actually written and reject later producer blocks. Never
append later bytes after a missing block. Live samples are never dropped,
inserted, modified or held waiting for diagnostic storage by this policy. Each
stream fails independently, including setup: a failed RX tap does not disable
an available TX recording or the independent prefix capture.

Each raw file has a `<raw-path>.status.json` sidecar:

- `recording`: setup succeeded, but there is no clean completion certificate;
- `complete`: clean close with no recorded failure;
- `incomplete`: capture failed; raw bytes are only a prefix;
- `offered_bytes`, `accepted_bytes`, `written_bytes`, `rejected_bytes`, `error`:
  final counters and the first failure's errno number;
- `prefix_only`: this capture format never silently removes internal gaps.

On disk errors, accepted bytes can remain unwritten. Total missing capture
bytes are `offered_bytes - written_bytes`, not just `rejected_bytes`.

Require successful setup for this engine and unique/fresh output paths first:
a failed setup can leave another engine's or an earlier run's files intact.
Only metadata written at orderly close is final: a running worker's failure
snapshot can precede further rejected blocks. For a complete recording require
`state == "complete"`, `error == 0`, `rejected_bytes == 0`, and equal offered,
accepted, written and actual raw-file byte counts. A missing, malformed or
`recording` sidecar is **not** evidence of complete capture. A forced kill
can leave queued bytes unwritten. Sidecar creation is required before
truncating/enabling the raw sink, and its previous completion certificate is
invalidated before a new recording starts. Completion describes successful
application writes/close, not power-loss durability; no `fsync` is performed.
Files remain process-wide across calls, like the old taps; restarting the
engine overwrites them. Files and `recording` sidecars are created at setup,
even if no selected training samples are subsequently offered.

**PCM is not a continuous line recording.** The same filtered RX samples and
unmodified TX tap points are retained, as native-endian signed 16-bit values
(little-endian on current Mac/Linux hosts). Only the original selected
training intervals are recorded; V.90 phase gaps remain intentional. Preserve
and read the verbose `RX dump mark` / `TX dump mark` sample positions before
interpreting a tone as belonging to a handshake phase. Marks count offered
samples; after a capture failure, marks beyond `written_bytes / 2` are not in
the retained prefix. An incomplete PCM write can end in a partial sample;
only its whole-sample prefix can be analysed. Capture handles, positions and
TX diagnostic RMS accumulators reset with engine initialisation, rather than
leaking function-static state into a later engine lifetime.

`me_flush_diagnostic_captures()` is control-thread-only: it waits for the accepted
prefix at entry to reach `write()`, without `fsync` or a completion certificate.
Clean engine destruction drains and joins the workers and finalizes metadata.
The historical `me_flush_g711_taps()` name remains an alias which now also
drains training PCM, including existing media-down control callers.
Storage stalls can delay control-thread drain/shutdown, **not the producer**.
Offline peers/replays no longer perform per-frame flush barriers: their
control loops drain in bounded batches well below queue capacity, preserving
complete captures even for cheap, unpaced simulated bearers. No such barrier
is added to a live media callback. Other unpaced producers can still overflow
a bounded capture; check the sidecar rather than assuming a passing payload
test proves its diagnostic recording complete.

The queue bounds memory, **not recording size on disk**. G.711 taps remain
opt-in and grow while enabled. The stdout rotation utility does not rotate
them. Other codeword/symbol dumps and synchronous text traces have not all
been migrated; do not assume every diagnostic is now off-clock. Verbose phase
marks themselves remain synchronous; normal quiet calls do not format them.

Validation: deterministic worker stalls, overflow and permanent fail-stop,
wraparound, short writes/EINTR, disk-full errors, flush/close/reopen, missing
sidecars, FIFO rejection, and exclusion/reopen of active paths and hard-link
aliases are covered by `async_capture_test`. Framed engine
tests exercise V.21, clear-channel and symmetric x2 paths in both laws,
verifying byte-exact RX/TX and prefix taps, unchanged transmitted
octets with captures enabled, complete recording of a long unpaced bearer and
both fast replay modes (normal/scheduled), safe setup failure, and an unclean kill leaving
`recording` rather than a false completion certificate. These are offline
checks, not hardware interop or a measured live speedup.

`tools/test_training_pcm_capture.py` generates V.34 payload calls in both laws,
checks PCM completeness and phase-mark offsets, and verifies identical G.711
TX in deterministic training replays with PCM enabled/disabled. It also checks
setup failure isolation, disabled capture, and two engines contending for the
same dump files without truncating the active recording. A saved pre-change executable
and the new build, given the same generated 10 s mu-law training schedule,
produced identical **61,760 RX PCM bytes, 62,080 TX PCM bytes, 80,000 RX and
80,000 TX codewords, and 76 phase marks**. This verifies preservation of those
tap points, not hardware interop or a CPU/latency speedup. Inputs were generated
privately for validation; no retained recording is required by the test.

## Next optimisation targets

1. **Finish diagnostic I/O isolation.** Migrate the remaining synchronous
   capture paths using the same explicit failure/accounting policy. Remaining
   codeword/per-symbol taps and replay schedule recording need separate review
   of their producer/lifecycle contracts. Never block the audio path to preserve a diagnostic recording.
   The rotating stdout sink bounds storage, but does not make generation or
   writing of the underlying diagnostic messages asynchronous.
2. **Measured lock contention.** Attribute long waits before shortening critical
   sections. Do not replace locks with unsynchronised shared DSP state.
3. **Selective DSP optimisation.** Profile on the slow wired host, then optimise
   one routine at a time. The makefile records replay/interop regressions from
   broad SpanDSP optimisation; blanket `-O3`, `-ffast-math`, FMA contraction or
   architecture changes are not safe shortcuts. Validate transmitted taps,
   decoded bits and acquisition pass rates.
4. **Simultaneous setup.** Measure CP worker CPU and concurrent training bursts
   independently of steady-state callback latency; see [multi-line](multi_line.md).

No protocol constants, sample counts, transmit timing rules or codec
conversions were changed by the measurement/quiet-diagnostics work.
