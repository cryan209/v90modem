# Multi-line server (`--lines N`)

The engine keeps one call's state in file-scope statics, so a line is a
process. `sip_v90_modem --lines N ...` re-executes itself N times, one worker
per line, and supervises them (`line_supervisor.c`). This is the gap
analysis's "cheap" option (item 1). Hoisting the globals into a per-call
context remains the proper one.

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
| environment | `%L` in any value becomes i; `ME_DUMP_DIR` without `%L` gets a `line<i>` subdirectory |

**Diagnostics.** The environment rule matters because the file-naming
diagnostics (`ME_IO_SCHEDULE`, `ME_DUMP_DIR`, ...) name one file per process.
Without it, every worker would write the same one. For example,
`ME_IO_SCHEDULE=/tmp/sched.%L` gives each line its own file.

**Output.** Workers' stdout and stderr are copied to the supervisor's, a line
at a time, prefixed `[Li]`.

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
- **Checks.** Every line's console echoes after CONNECT. Every caller must get
  its own payload back byte-exact, and the calls must overlap in time.
- **`--recycle`.** It checks that every line got a fresh worker, then runs a
  second round on the new workers.
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

**The 8-line failure is not the server.**
- One line with line 6's payload fails on its own:
  `--lines 1 --mode v90 --payload-offset 6`.
- The echo goes wrong after its first line: a character-framing slip, then
  garbage. The digital side logs an upstream "abrupt discontinuity".
- The I/O schedules show no media stall (longest gap 45 ms, against the 200 ms
  jitter buffer) and balanced sample counts.
- So it is a content-dependent defect in the V.90 upstream data path. It is
  filed for its own investigation, not covered up here.
- That is why the test lists use V.22bis (fast) and V.34 (slow).

## CPU

The 2026-10-10 live measurement in `docs/gap_analysis_2026-10.md` applies per
line: about 6.3% of a core in data mode, and about 6 CPU-s of call set-up,
most of it in the V.90 CP worker. Lines are processes, so the OS spreads them
across cores. Set-up bursts collide only when many calls reach Phase 4 at
once.
