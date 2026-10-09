# CLAUDE.md

Kept in sync with `AGENTS.md` (identical apart from the file names).
The dated lab notebook that used to live here is `docs/project_history.md`.

## What this is

A SIP/G.711 softmodem. Its main role is the V.90/V.92 **digital** side: an
analogue modem dials in, we answer, train over RTP and bridge the data to a
PTY. It also runs V.34, V.32bis, V.22bis, V.91, the V.90/V.92 analogue role,
clear channel (V.110/V.120), and fax (T.31 class 1, T.32 class 2.0, V.34 HDX).
The RTP payload **is** the DS0 the far-end D/A sees; most rules below follow
from that.

## Constraints that silently break the modem

1. **Never transcode G.711.** No resampling, VAD/CNG, echo cancellation or gain
   in the audio path; codewords pass through byte-exact
   (`PJMEDIA_HAS_PASSTHROUGH_CODECS` in `pj_config_site.h`).
2. **Never "clean up" DSP constants.** Scrambler taps, RRC/Godard tables and
   timing thresholds are ITU values. V.34 GPC (`1 + x^-18 + x^-23`) and GPA
   (`x^-5` tap) coexist deliberately (`v90.c:439`).
3. **Sample-rate error is fatal.** Don't add buffering that changes sample
   accounting, and don't splice samples into the received stream.
4. **Cite the spec.** ITU PDFs are in `ITU Docs/`; give clause numbers in
   comments and commit messages.

## Build and test

- `make` builds `sip_v90_modem` and the test binaries. SpanDSP (`spandsp-master/`)
  and pjproject (`pjproject/`) are vendored; never substitute system copies.
- `make test-fast` runs the quick tier (`tests/fast.list`), `make test-slow`
  the long matrices (`tests/slow.list`), `make test` both. They run through
  `tools/run_tests.py`: parallel, each test in its own `TMPDIR`/`ME_DUMP_DIR`,
  every failure reported with its log under `test-logs/`. Add a test by adding
  a line to a list; scratch files must use `test_tmp()` (`test_tmp.h`), not a
  fixed `/tmp` name.
- Offline/loopback tests say nothing about hardware interop.
- New source file: add it to `SRCS` **and** every relevant `*_OBJS` list.
- If behaviour contradicts the source after a header change, `make clean` and
  check what really rebuilt (stale objects have cost whole sessions).

```bash
./sip_v90_modem --sip-server asterisk.net.cryan.nz --username 6001 --password 6001 --pty-link /tmp/v90modem
```

Configuration: `AT+MS` and the other V.250 commands on the PTY are the
supported interface; `--mode`/`ME_MODE` set the power-on offer. The `ME_*`,
`V34_*` and `V90_*` environment variables are listed and classified in
`docs/env_knobs.md` -- most are diagnostics or experiments, not settings.

## Orientation

- `modem_engine.c` drives every call; `sip_modem.c` is the SIP/RTP side;
  `data_interface.c` the AT/PTY side; `data_stack.c` V.14/V.42/V.42bis/V.44.
- `vpcm_*.c` is the shared V.PCM layer under V.90 and V.92: fix protocol logic
  there, not twice.
- Not every decoder is live: check `SRCS` in `makefile` before assuming a
  change affects real calls.
- `*_rrc.h`, `*_godard.h`, `v34_*_tables.h` are generated tables.
- Each module's header comment states its role and spec clauses; read it first.
- `docs/` has one note per topic; read the relevant one before changing a phase.
  `artifacts/` and `captures/` are local and gitignored except the three fixture
  directories `make test` reads (`git add -f` a new fixture).

## Method -- traps that have each cost more than one session

- **Check our own transmit tap before blaming a peer.** Most "peer defects"
  here were ours.
- Receive-path filters run after the RX tap, so recordings can't show them, and
  `v34_duplex_test` bypasses the engine. A loopback/live gap points at the engine.
- Function-scope `static` state lives for the process, and one server handles
  many calls. Reset per-call state per call.
- One event slot is shared by several receiver events (`received_event`); test
  durable flags, not the last event.
- Metrics that read "white": distance-to-grid 0.667, angle-from-family 22.5
  degrees (quote the spread). A constant dibit descrambles to all ones and
  matches any low-entropy reference: print the dibit histogram.
- Score acquisition changes as pass rates over `V34_DUPLEX_DELAY`, live A/Bs
  with alternated arms and one variable; never score a run before it finishes.
- Logs: `[V90]` and `[ME]` interleave out of order, so order events by
  `[TRACE +Nms]`. Two modems' logs have different clocks: compare taps instead.
- A replay cannot judge anything after our first divergent transmission.
- Never run `ME_V34_SPAN_FLOW_LOG=1` on a live call. Use `pgrep -x`, not `-f`.
- Live rig calls go from tower (wired), not this Mac's Wi-Fi; see
  `rig/README.md` and `docs/v90_hardware_interop.md`.

## Process

- Commit straight to `main` (single maintainer, linear history), and push.
- Record new findings in the topic doc in `docs/`, not here.

`AGENTS.md` is the parallel copy of this file for other coding agents; mirror guidance changes there.
