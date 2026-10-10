# Gap analysis: what stands between this tree and a general-purpose softmodem

Written 2026-10-05.

**Status, same day:** steps 1 (AT+MS / `--mode` configuration) and 2 (V.32bis
on live calls, V.25 and V.32bis Annex A automode) are on `main`. Those
sections below describe the tree as it was before them.

The target is one softmodem that works in two roles:

- **Voice-modem mode.** The SIP/G.711 bearer behaves like an ordinary phone
  line. We are one of two analogue-style modems and support the usual ladder:
  V.21/Bell 103 up through V.34, plus V.90/V.92 as the *analogue* modem, plus
  fax.
- **PCM server mode.** We are the *digital* modem (V.90/V.92/V.91), answering
  many callers at once.

Everything below was checked against the current source (`SRCS` in `makefile`,
the dispatch in `v8_result_handler()`, `sip_modem.c`) and against the test
suite run on this date. Interop claims come from the dated entries in
`CLAUDE.md` and `docs/`, and are attributed to them.

## 1. Where it stands today (measured)

- **Build.** A fresh Linux clone did not build without three local fixes, all
  committed with this note:
  - `spandsp-master/test-data/itu/tiff-fx/Makefile.in` was gitignored, so
    configure failed. A stub is now force-added.
  - The link lacked `-luuid`; the makefile now adds `pkg-config --libs uuid`.
  - `v34_gardner_test.c` used `uint32_t` without `<stdint.h>`.
- **Build prerequisites.** The readme's apt package list really is required:
  libtiff, libjpeg, alsa, opus, ffmpeg, v4l and uuid.
- **`make test`.** 86 of the 87 steps pass, in ~150 s on 4 cores.
  - The one failure is `k56flex_client_test` row "RC loop + HP + noise, A 32k":
    the client fails in PRIME on Linux x86-64.
  - That test uses its own seeded RNG, so this looks like a floating-point
    margin difference against the macOS/ARM host it was written on.
  - It is the first test in the recipe, so on Linux it stops `make test`
    before anything else runs.

## 2. Capability matrix

| Modulation | Engine (live call) | Offline / loopback | Foreign hardware |
|---|---|---|---|
| V.90 digital (server) | yes, default | full Phase 1-4, data | **SmartLink/slmodemd: data mode, LAPM, V.42bis, 52000/31200.** RasFinder: Phase 4 abort (peer never sends CP). USR Courier: retrains after DIL/Jd. |
| V.92 digital | opt-in (`ME_MODE=v92`, `ME_V92_PCM_UPSTREAM`) | startup, 9.8/9.9/9.10/9.11 in harnesses | PCM upstream never validated: the slmodemd transmitter for it is malformed, and no other peer has been tried. MH, QC and short phases not proven live. |
| V.91 | yes (V.8 PCM bit) | loopback | none |
| V.90 analogue (client) | yes (`ME_V90_ROLE=analogue`) | yes | Eicon emulator: reaches V.42 detection, then a peer retrain. Over a real loop the codeword receiver has no timing loop. |
| V.92 analogue | wired (`v92a`) | ideal-bearer harness | stops at Sd-bar timeout over a real loop. Phase 4 audio DATA assertion known-failing in the full `v92_startup_test` audio case. |
| V.34 duplex | yes | 2400-3429 baud, up to 28800, echo, 11.6 | SmartLink: data mode, LAPM. RasFinder: `CONNECT 19200`, held 300 s calls. Intermittent MP'/E; first ask too high. |
| V.34 half-duplex (fax) | probe only (`ME_V34_FAX_PROBE`) | control channel end to end | Canon reaches control-channel data. **T.30 Annex F absent.** |
| V.32bis / V.32 | **yes** (2026-10-05): V.8 when it is the carrier, V.32bis Annex A automode otherwise, `AT+MS=V32B`/`V32` | clause 6/8 dialogue; `engine_pair_test` engine against engine, both laws | none |
| V.22bis / V.22 | yes (V.8, Annex A automode, pre-V.8 USB1 both roles; `AT+MS=V22B`, `AT+MS=V22` = V.22 at 1200) | SpanDSP; `engine_pair_test` incl. V.22 vs V.22bis and V.8-less peers | HSF loop: V.22bis carried traffic |
| V.21, V.23, Bell 103, Bell 212A | **not wired** (FSK presets exist in SpanDSP) | none | none |
| Fax class 1 / 2.0 (V.17/V.29/V.27ter/V.21) | yes, via T.31 / T.32 | `fax_class_test`, `fax_class2_test` | **none** |
| x2, K56flex | experimental | receive replay | neither completes a call |
| V.42 LAPM / V.42bis / V.44 | yes | yes | LAPM + V.42bis live; V.44 never offered by a peer |
| MNP 2-5 | **absent** | | |

## 3. Gaps, ranked by what blocks the stated goal

### P0: product and architecture blockers

1. **The engine handles one call per process.**
   - `modem_engine.c` holds its call state in ~327 file-scope statics, and
     `sip_modem.c` tracks a single `g_call_id`/`g_ringing_call`.
   - A second INVITE overwrites the ringing call.
   - A PCM server is, by definition, a pool of lines. Two options:
     - *Cheap:* one process per line (one registration and PTY each) under a
       supervisor that answers and spreads calls across a worker pool.
     - *Proper:* hoist the globals into a per-call `me_ctx_t`. This is
       mechanical but large; 12.8k lines touch them.
   - Do the cheap one first.
2. **Configuration lives in environment variables, not in the modem.**
   - There are 603 distinct `getenv` knobs.
   - The modulation family comes from `ME_MODE`/`--mode`. There is no
     `AT+MS` (modulation selection), and `--mode` does not accept `v22` even
     though `ME_MODE=v22` exists.
   - Unknown command-line flags are silently ignored. For example,
     `--pty` silently becomes `/tmp/modem0`, and
     `docs/v90_hardware_interop.md` still tells people to pass `--pty`.
   - A usable softmodem needs `AT+MS=<mod>,<auto>,<min>,<max>` to drive the V.8
     offer and fallback ladder, S-registers for the few knobs users really
     change, and a hard error on unknown flags.
   - The hundreds of experiment knobs should then become a small set of
     validated defaults.
3. **Linux portability of the test suite.** See section 1. The suite should
   run each binary independently, or `k56flex_client_test` should be fixed or
   moved, so that one platform-sensitive row does not hide 86 others.

### P1: voice-modem mode is missing most of the legacy ladder

4. **There is no V.25 / V.32 automode.**
   - A non-V.8 caller goes straight to V.22bis (`V8_STATUS_NON_V8_CALL`), and
     V.8 failure just hangs up.
   - Real answer-side automode sends ANS. With no CM it listens for:
     - V.32 AA
     - V.22 USB1 / V.22bis S1
     - V.21 and Bell 103 marks
     - Bell 212A
   - It then starts the matching datapump.
   - The calling side has the mirror problem: an answerer that sends plain ANS
     with no ANSam gets CM it does not understand.
   - Without this, any pre-1994 modem, and many embedded or industrial ones,
     cannot connect.
5. **V.32bis/V.32 is finished in SpanDSP but not wired.**
   - `v32bis.c` runs the full clause 6 dialogue, the tone phases with measured
     NT/MT, Note 3 echo-canceller training, and clause 8 renegotiation.
   - Its duplex harness passes in `make test`.
   - What is missing is engine integration: a `ME_MOD_V32BIS`, V.8's
     `V8_MOD_V32` bit, V.25 AA/AC detection (item 4), the V.14/V.42 data stack
     and the PTY.
   - This is the biggest coverage win for the least DSP work.
6. **V.21, V.23, Bell 103 and Bell 212A are not wired.**
   - V.21, V.23 and Bell 103 are SpanDSP `fsk.c` presets, about 200 lines of
     engine glue.
   - Bell 212A is V.22's modulation with a different answer tone and no
     guard tone, so it is a small variant on the V.22bis path.
7. **No MNP 2-5.** It matters only for peers from before V.42 that do not
   support LAPM. Low priority, but some V.22bis-era equipment needs it.

### P1: PCM server mode only works against one peer family

8. **V.90 digital against non-SmartLink peers.**
   - RasFinder: the peer abandons Phase 4 at TRN2d + 2240 ms and never sends
     CP, although our TRN2d and MP were verified against the text.
     `docs/v90_rasfinder_phase4.md` says to suspect the VG224's
     modem-passthrough path.
   - Courier: Jd is not accepted.
   - These two are the interop work that decides whether "V.90 server" means
     anything beyond slmodemd.
9. **V.90 upstream reliability.**
   - What is still open: busy-start frame-phase acquisition, and recovery
     from an eye collapse.
   - `docs/v90_upstream_data_path.md` carries the current state.
   - The live/replay divergence is now closed, so this can be worked offline.
10. **V.92 digital is unproven live.**
    - PCM upstream needs a peer whose transmitter is not malformed. The
      CX93001 at 6004 is the named candidate.
    - QC / short Phase 1-2 are offline analysers only.
    - Modem-on-hold has never met a real V.92 modem.
11. **V.34 robustness.** All of the following come from the dated
    `CLAUDE.md` entries:
    - The rate is configured, not measured. The first ask is typically
      31200, which goes white.
    - The MP'/E exchange is intermittent.
    - A retrain taken from Phase 4 can deadlock in `FIRST_B_SILENCE`.
    - Phase 4 receive at 2400 baud is weak.
    - Clause 11.7 cleardown is missing entirely; clause 11.6 is the nearest
      neighbour.
    - The 3429/9600 rows still do not train.

### P2: fax

12. **T.30 Annex F (V.34 fax) is absent.**
    - The clause 12 modem layer reaches control-channel data against a real
      Canon, and nothing above it consumes that channel.
    - `docs/t30_annex_f_v34_fax.md` lists exactly what F.3 requires.
13. **Class 1 / 2.0 have no hardware interop at all.** One real
    fax-to-softmodem call in each direction and each class would establish it.

### P3: secondary

14. **x2 and K56flex** are archaeology. Neither completes a call; leave them
    behind their opt-in modes.
15. **Analogue front ends (HSF, Apple SM56).**
    - These show the stack working over real loops, which is what
      voice-modem mode means when the bearer is a physical line.
    - The V.90/V.92 analogue receivers still lack continuous symbol-timing
      tracking. Measured: 163 ppm against the SIP side's clock.
    - The V.92 Phase 3 receiver has no equaliser.

## Progress since this note

- **2026-10-05, items 2, 4 and 5 (mostly) done.**
  - `AT+MS` (V.250 6.4.1) now selects the carrier, automode and rate limits
    for the next call. Unknown command-line flags are an error, and `--mode`
    takes `v22` and `v32`.
  - V.32bis runs in the engine.
  - V.32bis Annex A automode connects pre-V.8 V.32bis and V.22bis modems in
    both roles.
  - `engine_pair_test` grades all of it with two whole engines.
  - Still open from those items: the hundreds of experiment knobs are still
    environment variables.
- **New finding from the same harness.** V.34 engine-against-engine does
  not train: both ends stall in Phase 2's INFO0 exchange. The bare
  `v34_duplex_test` does train, so the fault is in the engine's V.34 path.
  Reproduce with
  `./engine_pair_test --seconds 60 --expect V34 --both-env ME_MODE=v34`.

## 4. Suggested order

1. Build fixes (done with this note). Still to do: make `make test` run
   every binary even when one fails, and fix `k56flex_client_test` on x86-64.
2. AT/configuration layer: `AT+MS`, strict command-line parsing,
   `--mode v22`, and a documented short list of supported knobs.
3. V.25 answer/originate automode plus the V.32bis engine integration. Do
   these together, because automode is what selects V.32bis.
4. FSK family (V.21, V.23, Bell 103) and Bell 212A on the same automode
   detector.
5. Multi-line PCM server via process-per-line, then measure CPU per line.
   `docs/v90_rasfinder_phase4.md` already shows the Ja scanner at ~40x real
   time once, so a per-line budget matters.
6. Interop campaign for the V.90 server (RasFinder, Courier) and V.92 PCM
   upstream (CX93001). Use the existing taps-and-replay method.
7. V.34: measured-SNR rate selection, the 11.7 cleardown, and the Phase 4
   retrain deadlock.
8. T.30 Annex F; real-machine class 1/2 fax tests.

Steps 2-4 are mostly integration of DSP that already exists and is tested.
Steps 6-8 are where the open signal-processing problems are.

## CPU per line, measured live (2026-10-10)

One live V.90 call against the d-modem/slmodemd rig, from tower: `main` at
175ccb33, `-O2 -g`, `--verbose`, μ-law, `CONNECT 54666` down / 31200 up, 128 s
of bidirectional payload (13429 U-lines, 43141 D-lines). Tower is a Ryzen 7
5700X (8 cores, 16 threads) with other load on it (load average ~3).
`tools/soak/cpu_sampler.py` read every thread's utime+stime from `/proc`
every 0.5 s. Phases are aligned to the server log's wall clock.

| Window | Duration | CPU (% of one core) |
|---|---|---|
| Registered, idle | 49 s | 0.3 |
| V.8 | 6.6 s | 0.5 |
| Phase 2-4 training | 21.1 s | 29.1 average |
| Data mode, both directions | 128 s | 6.6 (6.3 in the steady 100 s) |

- **The real-time work is cheap.** The engine runs in pjmedia's `clock`
  thread, at 5-6% through training and data and 10% at its peak.
- **The one burst is the strict live CP worker** (`v90_cp_live_worker`,
  `modem_engine.c`). It runs at ~98% of a core for the ~3.5 s between TRN2d
  and the first valid data-mode CP, about 4.3 CPU-s per call. Each attempt
  re-demodulates the whole buffered waveform: 147200-156080 samples, about
  18 s of history, at 16 timings. A new attempt starts 320 samples (40 ms)
  after the last one finishes, so the thread never idles. It is a separate
  thread, so the media clock was not starved on this call.
- **Rough capacity.** At 6.3% a line in data mode, about 15 lines per core
  are steady-state. Call setup costs about 6 CPU-s, 4.3 of them in the CP
  worker. That only matters when many calls reach Phase 4 together.
- **Caveats.** This is one call, with verbose logging included, on a shared
  host. The Ja worker (Phase 3) was ~5% and is not a factor.
- **If density matters,** bound the CP worker's search to recent waveform
  rather than all 18 s. The batch search keeps pre-transition CPt on purpose
  (see the classifier comment), so that change needs its own live A/B.
