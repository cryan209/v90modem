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
environment and host load for comparisons. In particular, captures still do
synchronous I/O: use a private `ME_DUMP_DIR` for each run and keep its storage
constant across arms. Record revision and dirty-tree status. Alternate A/B
arms, let every run finish, and require payload correctness as well as timing.
Use an OS profiler (`sample`/Instruments on macOS, `perf` on Linux) for CPU
hotspots; `wall_ms / audio_ms` is not CPU utilisation. Use an OS timing tool
which accounts for children when measuring worker/process CPU.

A local six-run smoke baseline (three per law) exchanged V.34 payload in all
six calls. This is a working-tree sanity check, not an optimisation speedup
claim, clean regression baseline, or hardware interoperability result. The
host and vendored DSP tree had unrelated ongoing changes. Live qualification
must run on the wired rig, following [hardware interop](v90_hardware_interop.md).

## Next optimisation targets

1. **Diagnostic I/O isolation.** Replace capture-path media-thread writes and
   flushes with a bounded diagnostic-only queue and background writer. Define
   loss counters and discontinuity metadata first: a dropped diagnostic
   record must not silently produce an apparently continuous audio tap.
   Never block the audio path to preserve a tap, and never drop live audio.
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
