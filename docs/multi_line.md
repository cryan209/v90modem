# Multi-line server (`--lines N`)

The engine keeps one call's state in file-scope statics, so a line is a
process. `sip_v90_modem --lines N ...` re-executes itself N times, one worker
per line, and supervises them (`line_supervisor.c`). This is the gap
analysis's process-isolated option (item 1): multiple calls are supported
concurrently, with one engine/PTY/SIP/RTP instance per worker. Do not raise a
worker's PJSUA call limit and share the DSP globals between calls. Supporting
multiple calls *inside one worker* would require a separate context refactor.

```bash
./sip_v90_modem --lines 4 --sip-server asterisk.net.cryan.nz --username 6001 --password 6001 --pty-link /tmp/v90modem
```

## What each worker gets

Worker i runs with the supervisor's arguments, plus these:

| Setting | Worker i |
|---|---|
| SIP port | `--local-port` + i (default 5060 + i) |
| RTP ports | `--rtp-port` + 100·i (default 4000 + 100·i) |
| console | `<--pty-link>.i` (default `/tmp/modem0.i`), or `<--control-link>.i` and `<--data-link>.i` |
| stored profiles | `<--profile>.i` |
| account | the i-th entry of a comma-separated `--username` / `--password`, or the only entry |
| environment | `%L` in any value becomes i; diagnostic directory/file paths without `%L` are separated automatically (below) |

**Diagnostics.** Each line must have separate files, not just separate DSP
memory. `%L` is expanded using the original value: a template is not then
suffixed a second time. In particular, `ME_DUMP_DIR=/tmp/dump.%L` means
`/tmp/dump.0`, not `/tmp/dump.0/line0`. Missing directory parents are created.

- `ME_DUMP_DIR` and `VPCM_G711_TAP_DIR` without `%L` get a `line<i>`
  subdirectory. If `ME_DUMP_DIR` is absent/empty, each worker gets a private
  `v90modem-line<i>-XXXXXX` directory under `TMPDIR` (or `/tmp`), rather than
  all workers overwriting the legacy `/tmp` PCM filenames. The V.90 Phase 3
  receiver dump also honours this directory instead of a shared absolute
  `/tmp/v90_phase3_rx.raw`. Optional G.711
  recording is **not** enabled unless requested.
- Known diagnostic **files** without `%L` get `.line<i>` appended:
  `ME_IO_SCHEDULE`, `DS_RX_BIT_DUMP`, `DS_TX_BIT_DUMP`, `DS_TX_FRAME_DUMP`,
  `ME_V90_UPSTREAM_SYM_DUMP`, `ME_RX_BIT_DUMP`, `V90_CP_ACCEPT_DUMP`,
  `V90_CP_BIT_DUMP`, `V90_RENEG_BIT_DUMP`, `V90_JA_BIT_DUMP`,
  `V90A_RX_CODEWORD_DUMP`, `V90A_T2_DUMP`, `V90A_SYM_DUMP`.
- Other environment values retain the explicit `%L` convention; boolean
  dump switches are not treated as filenames. Values are not truncated at
  1024 bytes. An isolation/setup failure stops that worker before SIP starts.

For example, `ME_IO_SCHEDULE=/tmp/sched.%L` gives each line its own file.
Explicit per-line paths can still be reused across calls/recycles; they are
not a per-call recording archive.

**Ports.** The entire SIP and RTP/RTCP allocation is checked before any worker
starts. Overflow, malformed ports and overlapping pools fail immediately
instead of leaving a partially working pool in a restart loop. Default bases
remain disjoint through 40 lines before SIP 5060+i meets RTP 4000+100*i. For a larger
pool, choose disjoint bases, for example `--lines 64 --rtp-port 10000`.
Split consoles require both links and cannot be combined with `--pty-link`.
`--lines N --help` prints help once without starting workers.

**Output.** Workers' stdout and stderr are copied to the supervisor's, a line
at a time, prefixed `[Li]`. A busy worker yields after at most sixteen reads
per drain, so it cannot monopolize other lines' output, restarts or signal
handling. After a worker exits, its finite remaining output is drained to
EOF. Supervisor pipe descriptors close on exec instead of accumulating in
other workers. A slow external stdout consumer can still apply backpressure;
this is fairness, not a hard real-time logging guarantee.

**Restarts.** A worker that exits is started again: after 1 s, or after 10 s
when it has failed three times within 5 s of starting.

**Shutdown.** SIGINT/SIGTERM stop all workers; any still running after 5 s
is killed.

**`--recycle`** makes every worker exit when its call is over, and the
supervisor starts a fresh one. Every call then begins in a clean process,
which side-steps per-process static state and the PJ_ETOOMANY leak noted in
the project history.
- The cost is that the line's PTY is recreated after each call.
- A DTE must reopen the symlink: suitable for auto-answer server lines (a
  getty, a BBS), not for a terminal program held open across calls.

`--gui`, `--gui-web` and `--gui-loopback` cannot be combined with `--lines`.

## Inbound distribution: the PBX

The lines are ordinary SIP user agents, so how calls reach them is the PBX's
choice. There are two arrangements.

- **One number, N contacts.** All lines register the same account, and the
  PBX forks each INVITE to every contact.
  - A busy line answers `486 Busy Here`. Idle lines ring, and the first to
    answer wins.
  - So that they do not all answer at once, line i's ring cadence is offset
    by 500·i ms. The lowest idle line reaches its S0 ring first, and the PBX
    cancels the others.
  - Asterisk PJSIP must accept N contacts on the AOR:

    ```ini
    [6001]
    type=aor
    max_contacts=8
    remove_existing=no
    ```

  - With the default `max_contacts=1`, each registration replaces the last
    one, and only one line is reachable.
- **One number per line.** `--username 6001,6002,6003,6004 --password
  p1,p2,p3,p4` registers each line as its own extension. A hunt group or queue
  on the PBX then spreads calls across them.

Outbound calls are per line: each line's console dials independently.

Neither arrangement has been tried against the rig's Asterisk yet, which needs
the AOR change above. The local test below uses no registrar.

## Test: `tools/multi_line_test.py`

- **Setup.** One supervisor with N lines answering on 127.0.0.1, with no
  registrar: each worker listens on its own port. N ordinary caller
  instances each dial a different line, all at once.
- **Checks.** Every line's console echoes after CONNECT. A startup barrier
  keeps early callers connected until all callers have trained. Each caller
  must receive exactly its own payload (not just find it inside garbage).
  `--payload-rounds N` sends N distinct blocks on each call, and a completion
  barrier keeps the calls open while their peers finish. Nonblocking short
  PTY writes are completed; failure aborts the startup barrier rather than
  leaving other callers stuck.
- **`--recycle`.** It checks that every line got a fresh worker, then runs a
  second round on the new workers.
- **`--restart-line I`.** Intentionally kills that owned worker during exact
  payload, stops only its original caller, and redials the fresh worker while
  peers continue. Requires at least two lines, 64 payload blocks, one round
  and no `--recycle`. The destroyed call is explicitly reported as lost; it
  is not counted as a successful call or in-place protocol recovery.
- **Executable identity.** Each invocation freezes a private executable copy
  for both the supervisor and callers, with SHA256 printed in the log.
  `--binary PATH` selects the source image; concurrent builds cannot silently
  mix caller/worker versions across a recycle or restart.
- **Shutdown.** The supervisor is stopped, and no worker may outlive it.
- **`--io-schedule`.** Records each process's RX/TX timing and reports its
  longest gap.

| Run (this Mac, 10 cores, callers on the same host) | Result |
|---|---|
| 3 lines V.22bis | ok, all three up together |
| 3 lines V.22bis `--recycle`, 2 rounds | ok, fresh worker per call |
| 4 lines V.34 | ok, CONNECT 24000 on all four |
| 4 lines V.90 (callers analogue) | ok, CONNECT 56000 on all four, ~1.5 CPU-s per worker to data mode |
| 8 lines V.90 | 7/8 every run: line 6 fails |

**Historical 8-line failure (the earlier build):**
- One line with line 6's payload fails on its own:
  `--lines 1 --mode v90 --payload-offset 6`.
- The echo goes wrong after its first line: a character-framing slip, then
  garbage. The digital side logs an upstream "abrupt discontinuity".
- The I/O schedules show no media stall (longest gap 45 ms, against the 200 ms
  jitter buffer) and balanced sample counts.
- So it is a content-dependent defect in the V.90 upstream data path. It is
  filed for its own investigation, not covered up here.
- At the time, that kept V.90 out of the lists; the current slow tier now
  includes the stronger eight-line V.90 regression below.

### 2026-10-29: current concurrent-call validation

The historical line-6 result **does not reproduce in the current working
tree**. An initial unchanged-harness run also passes 8/8, so do not attribute
that difference to the supervisor changes or infer which earlier DSP fix
resolved the old short test. The stronger test now gives:

| Local software-peer run | Result |
| --- | --- |
| 8 V.90 lines, 32 blocks/call, recycle, 2 rounds | **16/16 calls**, 14,392 exact echoed bytes each; **230,272 bytes total** |
| 4 V.34 lines, 8 blocks/call, recycle, 2 rounds | **8/8 calls**, 3,552 exact echoed bytes each; **28,416 bytes total** |
| 3 V.22bis lines, recycle, 2 rounds (fast tier) | **6/6 calls**, 416 exact echoed bytes each |

All eight V.90 calls are connected together for at least 20 seconds per
round. In the capture-enabled run, every server line has its own G.711 tap
directory with no shared root tap and its own nonempty Phase 3 PCM dump.
A fresh recycled worker truncates its per-line G.711 files at initialization;
those empty final tap files are **not** retained payload evidence or a
per-call archive. Exact payload results come from the DTE checks, not these
post-recycle taps. The reported final-round schedules have
balanced RX/TX sample counts. V.90 and V.34 matrices also ran concurrently on
this Mac. This exercises real local SIP/RTP and software modems, **not eight
physical analogue modems, PBX hunt-group routing or RasFinder qualification**.
The existing physical single-call continuity failures remain open.

`line_supervisor_test` directly tests the production static helpers: template
handling, directory creation, private defaults, file isolation, long values,
bounded draining, close-on-exec pipes and port layouts. Before the fixes, all
five environment cases and the busy-drain case fail. The Python harness tests
cover short writes, recycled PTYs whose symlink was already replaced, startup
failure, help/preflight, and a SIP-free two-worker fixture: killing worker 0
restarts only worker 0, preserves worker 1's PID, and shutdown leaves no
fixture workers alive. It is a process-lifecycle test, not active-call crash
recovery.

**648/648 fast rows pass** (`test-logs/multi-call-isolation-final/`, 129 s).
The eight-line V.90 row is also listed in `tests/slow.list`; the whole slow
matrix was not rerun here.
The first whole-tier invocation exceeded its outer 240-second tool budget;
it is not counted as a pass. A complete fresh invocation with a sufficient
budget produced the result above. Small retained logs/provenance are under
`reference-data/artifacts/multi-call-isolation-20261029/`; large disposable
local captures are under `artifacts/multi-call-isolation/`. Tests ran in the
shared working tree with unrelated pending HDX/K56flex changes; those are not
part of the supervisor change or physical qualification.

### 2026-10-30: active-worker failure isolation

The SIP-free lifecycle fixture is now complemented by **active data calls**:

```bash
python3 tools/multi_line_test.py --lines 3 --mode v90 \
  --payload-rounds 64 --restart-line 0 --io-schedule
# Also exercised a nonzero worker and a different modulation:
python3 tools/multi_line_test.py --lines 4 --mode v34 \
  --payload-rounds 64 --restart-line 2 --io-schedule
```

In a clean detached `596c2776` worktree plus only the harness/list changes,
with fresh SpanDSP and top-level binaries:

| Active-worker fault | Completed calls, excluding the intentionally lost original call |
| --- | --- |
| 3 V.90 lines, kill worker 0 | Two untouched callers plus the replacement each return **28,856 exact bytes**; **86,568 total**. All three are connected together for **20.1 s** after replacement. |
| 4 V.34 lines, kill worker 2 | Three untouched callers plus the replacement each return **28,856 exact bytes**; **115,424 total**. All four overlap for **23.7 s** after replacement. |

Each destroyed original call had returned 416 exact bytes before SIGKILL;
those two partial calls are **intentional losses**, not passes. Other worker
PIDs remain unchanged. The surviving callers are not restarted or redialled:
they finish their original payloads across the fault and replacement training.
The replacement is a new call, not recovery of the destroyed SIP dialog.

The same isolated build repeats the eight-line/two-round V.90 check:
**16/16 completed calls, 230,272 exact bytes**, all eight connected together
for at least 20.6 s per round. **649/649 fast rows pass in 118 s**. Ten Python harness
unit tests include executable freezing, targeted fault injection and rejection
of an unrelated worker restart. The V.90 active-fault command is in the slow
tier. `--group multi-line-sip` filters these harness rows; groups are labels,
not scheduling locks, so use `-j1` when separate harness runs are desired.
The selected three-case run used `-j2`: the eight-line V.90 and three-line
fault cases overlapped. No quiet-load performance benefit is inferred.

The build reuses the same vendored pjproject libraries and retained input
fixtures, but excludes concurrent working-tree DSP/rate-expectation edits.
A preceding shared-tree whole-tier attempt had six non-owned DSP/expectation
failures and one executable-change failure; all six V.22bis payloads in that
last row had passed before the identity guard rejected the changing image.
An initial detached attempt accidentally included concurrent rate-expectation
hunks and lost its shared log directory during another runner's cleanup;
it is incomplete, not a pass. The corrected run uses only owned list hunks,
a private log directory and the frozen executable strategy above.

Logs, owned patch, failed-attempt summaries and provenance are retained in
`reference-data/artifacts/multi-call-active-20261030/`. These remain **local
software SIP/RTP tests**, not physical modem continuity, PBX hunt routing,
authenticated service access or a guaranteed host capacity.

## CPU

The 2026-10-10 live measurement in `docs/gap_analysis_2026-10.md` applies per
line: about 6.3% of a core in data mode, and about 6 CPU-s of call set-up,
most of it in the V.90 CP worker. Lines are processes, so the OS spreads them
across cores. Set-up bursts collide only when many calls reach Phase 4 at
once.
