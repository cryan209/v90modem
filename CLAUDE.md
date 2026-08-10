# CLAUDE.md

Gotchas and repo-specific knowledge for this project. Kept in sync with `AGENTS.md`; identical apart from this header.

## What this is

A SIP-based V.90/V.92 **digital-side** modem server. An analog client modem dials in over SIP; this software answers, runs the V.90 handshake over G.711 RTP, and bridges the data connection to a PTY.

The RTP payload **is** the DS0 PCM stream the far-end D/A converter sees. Most of the constraints below follow from that one fact.

## Constraints that silently break the modem

1. **Never transcode G.711.** No resampling, VAD/CNG, comfort noise, echo cancellation, or gain/AGC anywhere in the audio path. PCMU/PCMA codewords pass through byte-exact, for V.90, V.91 and V.92 alike. Passthrough is enabled via `PJMEDIA_HAS_PASSTHROUGH_CODECS` in `pj_config_site.h`.
2. **Never "clean up" DSP constants.** Scrambler taps, RRC coefficients, Ucode/µ-law tables and timing thresholds are ITU-spec values. Two scramblers coexist deliberately: downstream uses V.34 GPC (`1 + x^-18 + x^-23`), *not* the answer-modem GPA (`x^-5` tap). Conflating them is the classic silent breakage here — `v90.c:439`.
3. **Sample-rate error is fatal.** `clock_recovery.c` exists because a few ppm of drift destroys the constellation. Don't add buffering that changes sample accounting.
4. **Cite the spec when changing protocol logic.** ITU-T PDFs are in `ITU Docs/`. Reference section numbers in comments and commit messages, as the existing code does.

## Build and run

`make` builds `sip_v90_modem` and the test binaries. `make test` runs the suite (`data_stack_test`, `v42_link_test`, `v34_phase2_decode_test`, `v92_proc_eval_test`, `vpcm_loopback_test --all-tests`). Python harnesses are separate targets: `v32bis-ref-test`, `v32bis-datapump-test`, `v91-serial-pair-test`.

```bash
./sip_v90_modem --sip-server asterisk.net.cryan.nz --username 6001 --password 6001 --pty-link /tmp/v90modem
```

The PTY flag is `--pty-link`, not `--pty`; passing `--pty` is silently ignored and you get the default `/tmp/modem0`.

The automated tests only cover offline and loopback paths — passing them says nothing about hardware interop, which is verified manually against a real analog modem (`docs/v90_hardware_interop.md`).

Adding a source file means updating `SRCS` **and** the relevant `*_OBJS` lists in `makefile`; the test binaries link overlapping subsets, so a new dependency often needs adding in several places.

## Platform (macOS Apple Silicon)

- SpanDSP 3.0 builds from source in `spandsp-master/`. Do not substitute a system spandsp.
- `make` prefers the in-repo `pjproject/` tree when present, falling back to Homebrew.
- Both vendored trees are host-specific; `make` auto-`distclean`s and reconfigures when OS/arch changes.
- Override `ARCH_SUFFIX` only if linking a Homebrew pjproject with a different suffix: `ls "$(brew --prefix)/lib"/libpj-*.a | sed 's/.*libpj-//' | sed 's/\.a//'`

## Orientation

Layering that isn't obvious from filenames:

- `vpcm_*.c` is the shared V.PCM-family layer (CP helpers, G.711 stream, session and coupled-training plumbing) sitting under both the V.90 and V.92 paths. Protocol fixes usually belong here, not duplicated into `v90.c` and the `v92_*` modules.
- Not every decoder is live. `p3_demod.c` *is* — via `v90_cp_live.c` and `v92_p3_rx.c`. But `v34_phase2_decode.c`, `v34_info_decode.c`, `v8bis_decode.c`, `phase12_decode.c`, `v92_short_phase*_decode.c` and `v92_anspcm_decode.c` are absent from `SRCS` and reachable only from `vpcm_decode` and the test harnesses. Check `SRCS` in `makefile` before assuming a change affects real calls.
- `*_rrc.h`, `*_godard.h` and `v34_*_tables.h` are generated coefficient tables. Data, not hand-editable code.
- `tools/` holds Python analysis and replay scripts; `captures/` and `artifacts/` hold recorded line audio and run outputs from hardware sessions.
- The ADSP-2181 emulator the firmware replays run on (`tools/adsp2181emu/`) is **not** built by the top-level `makefile`, and `libadsp2181.dylib` is gitignored — run `make -C tools/adsp2181emu` before trusting a replay.
- Two replay harnesses drive that emulator and they disagree past the INFO page: `tools/eicon_info_replay.py` uses `LiveKernelModem`, while live captures (`eicon_adsp_sip.py --native-mips`) and `tools/v90_dpcm_replay.py` use `create_native_mips_modem()`. Only the native one reproduces the live card on V.90 page 14 — it needs `unicorn`, so run it under `/tmp/eicon-venv/bin/python`. Session 50 in `docs/eicon_adsp_firmware_analysis.md` records what mixing them up costs.

Each module's header comment states its role and the spec sections it implements — read that before the `.c`.

## Current status

- **V.90 training (Phases 1–4, §8–9)** — real per-spec implementation, not a stub. Phase 3 emits genuine PCM codewords (Sd/S̄d/TRN1d/Jd/DIL); Phase 4 runs a negotiated modulus mapper with real MP/CP CRC framing. Open work is reliability against noisy line audio rather than loopback — `docs/p3_demod_rrc_frontend_plan.md`.
- **V.34 upstream** — SpanDSP V.22bis (2400 bps) is a working placeholder; the V.34 upstream encoder is not wired into the live path. The V.34 *upstream RX* (analogue→digital data path) is live: the dedicated `V34_RX_STAGE_V90_CP` stage acquires CPt/CP with CMA frozen (so the payload doesn't drift over the 214-symbol frame) and `v34_v90_prepare_upstream_data()` reconfigures the RX for 3200-baud data at the V90_CP→DATA seam (V.90 skips the ordinary V.34 MP exchange that would otherwise do this). Open work: data-mode stability after E/B1 (peer retrain shortly after entering data) and, if a peer uses V.34 precoding (`mp.type != 0`), decoding the upstream MP to seed `rx.h[]` (currently zeroed).
- **Mi negotiation** — consuming a peer CP frame into per-interval Mi and running the §5.4.3 modulus encoder is fully implemented and live; it is not hardcoded. The gap is narrower: when *this* side originates a constellation offer (V.92 upstream CPd, and the coupled-training harness's simulated CP) it offers the maximal Ucode set instead of deriving one from DIL/impairment analysis — `docs/v90_mi_negotiation.md`.
- **Known hardware blocker** — the Courier retrains after DIL: `docs/v90_phase3_s_and_rbs_false_positive.md`, `docs/courier_firmware_analysis.md`.
- **TRN1d may be 12x too short** — a working V.90 digital modem (the Eicon card, captured in `artifacts/eicon-digital-downstream/`) transmits **30000T / 3750 ms** of TRN1d, spending 94% of the §9.3.1.5 budget; `v90_trn1d_len()` sends 2496T / 312 ms. Verified bit-for-bit on two independent connecting calls. The `ME_V90_TRN1D_SYMBOLS` clamp (<=16000, `v90.c:121`) blocks testing the card's value. Untested live, and the leading candidate for why the Courier never decodes our Jd — `docs/eicon_downstream_comparison.md`.
- **Receive path has never seen a foreign downstream** — every receive test is fed by our own transmitter, so a wrong-but-self-consistent assumption is invisible to it. `make eicon-rx-test` points the analogue-side decoders at the card's downstream; it currently mislabels the card's constant-magnitude TRN1d as Sd, which §8.4.4 forbids (Sd is 1/3 Ucode 0).

`docs/` holds a design note or investigation write-up per phase. Read the relevant one before changing that phase.

`AGENTS.md` is the parallel copy of this file for other coding agents; mirror guidance changes there.
