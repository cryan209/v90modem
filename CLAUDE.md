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

**The makefile has no header dependencies** (`%.o: %.c` at `makefile:243`, no `-MMD`). Editing a `.h` does not rebuild the `.c` files that include it, so a struct field added or reordered leaves stale objects linked against the old layout — the symptom is impossible values at runtime, not a link error. `rm` the affected `.o` files (or `make clean`) after touching any header.

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
- The Eicon Diva Server card work — the ADSP-2181 emulator, the MIPS firmware harness, the DIAL/INFO/V90D investigation tools and their 69-session analysis log — now lives in its own project, `../modem-dsp-emu`. It shares no code with this one: it runs the card's shipped firmware under emulation, rather than implementing the spec. `tools/cx_at.py` and `docs/courier_firmware_analysis.md` stayed here, and that project references them.
- `tools/` holds Python analysis and replay scripts; `captures/` and `artifacts/` hold recorded line audio and run outputs from hardware sessions.

Each module's header comment states its role and the spec sections it implements — read that before the `.c`.

## Current status

- **V.90 training (Phases 1–4, §8–9)** — real per-spec implementation, not a stub. Phase 3 emits genuine PCM codewords (Sd/S̄d/TRN1d/Jd/DIL); Phase 4 runs a negotiated modulus mapper with real MP/CP CRC framing. Open work is reliability against noisy line audio rather than loopback — `docs/p3_demod_rrc_frontend_plan.md`.
- **V.34 upstream** — SpanDSP V.22bis (2400 bps) is a working placeholder; the V.34 upstream encoder is not wired into the live path. The V.34 *upstream RX* (analogue→digital data path) is live: the dedicated `V34_RX_STAGE_V90_CP` stage acquires CPt/CP with CMA frozen (so the payload doesn't drift over the 214-symbol frame) and `v34_v90_prepare_upstream_data()` reconfigures the RX for 3200-baud data at the V90_CP→DATA seam (V.90 skips the ordinary V.34 MP exchange that would otherwise do this). Open work: data-mode stability after E/B1 (peer retrain shortly after entering data) and, if a peer uses V.34 precoding (`mp.type != 0`), decoding the upstream MP to seed `rx.h[]` (currently zeroed).
- **Mi negotiation** — consuming a peer CP frame into per-interval Mi and running the §5.4.3 modulus encoder is fully implemented and live; it is not hardcoded. The gap is narrower: when *this* side originates a constellation offer (V.92 upstream CPd, and the coupled-training harness's simulated CP) it offers the maximal Ucode set instead of deriving one from DIL/impairment analysis — `docs/v90_mi_negotiation.md`.
- **Known hardware blocker** — the Courier retrains after DIL: `docs/v90_phase3_s_and_rbs_false_positive.md`, `docs/courier_firmware_analysis.md`.
- **V.90 analogue role — V.8, Phase 2 and Phase 3 run in a live call** — `ME_V90_ROLE=analogue` on an *originating* call (V.90 puts the analogue modem on the calling side; an answering call falls back to V.34 and says why). V.8 offers the role (only `pcm_modem_availability` flips to `_ANALOGUE`; `pstn_access` keeps `DCE_ON_DIGITAL` because it describes our access, not our role; V.92 octet omitted), SpanDSP's V.34 caller path runs Phase 2 — INFO0a is bit-for-bit the V.34 INFO0, and `prepare_v90_info1a()` already emits Table 10 with U_INFO — and the engine takes the modulator over at the seam where SpanDSP's transmit stage reaches `FIRST_S`, which is where §9.3.2.1 starts. Phase 3 is three modules: `v90_analogue_tx.c` (S, S̄, PP, TRN, Ja carrying the Table 12 descriptor, SCR, through SpanDSP's modulator via a new `v34_tx_start_external_symbols()`), `v90_analogue_rx.c` (Sd, S̄d, TRN1d, Jd, J'd, DIL as a **codeword** state machine — the downstream is the DS0 stream, so this role has no V.34 receiver, and SpanDSP's stops being fed at the seam), and `v90_analogue_phase3.c`, whose four events are the whole of §9.3.2's dependency on the far end. `g_v90` is never created on such a call, which is what keeps the digital transmitter out of it. Phase 4 (§9.4) does not exist: the engine logs the DIL measurement and the constellation it implies, then hangs up (`ME_V90_ANALOGUE_HOLD=1` keeps the call up for capture). `v90_analogue_rx_test` in `make test` runs the receive path against `artifacts/eicon-digital-downstream/` — the only downstream here we did not generate — reading Sd 64 reps, S̄d 8, TRN1d 30000T and 13–14 CRC-valid Jd frames, then driving the transmitter to Phase 4 off that real signal. **Two conventions the fixture caught, both invisible to loopback: Sd's zero slots must match on level, not codeword** (a working modem sends +0 in every zero slot, and since those slots match Sd and S̄d equally, counting them hides the §9.3.2.4 transition entirely), **and TRN1d is not differentially encoded** — §8.4.5 puts the scrambler output straight on the sign, and only Jd adds §8.4.2's differential encoder; decoding TRN1d differentially yields a clean stream of zeros and truncates it. **Live against the Eicon card under emulation (2026-08-10) the analogue side now completes Phase 2 and drives the card onto its V.90 page** (`INFO_RX complete=1`, `bootpage 7 -> 14 V.90 DPCM`). Three fixes got there, each found by a call: §9.2.2.1.8 — the analogue modem transmits **Tone A** after L1/L2 and answers with INFO1a only after INFO1d (SpanDSP took the V.34 call-modem branch and sent INFO1a immediately, which deadlocks, because §9.2.1.1.7 has the digital modem waiting to detect Tone A before it sends INFO1d); this card **never sends INFO1d at all** and waits for INFO1a, whose bits 37:39 pick its V.90 page, so `ME_V90_ANALOGUE_INFO1D_TIMEOUT=info1a` sends it anyway (opt-in — it is not one of §9.2.2.2.4's two responses, and the conformant INFOMARKSa *replaces* the Tone A the peer wants); and INFO1a is now repeated as a §8.2.3.1 group (4 by default, `ME_V90_ANALOGUE_INFO1A_REPEATS`) exactly as the digital side already repeats INFO1d. **It then stops for a reason that is not ours**: the card enters page 14 with `tx_ptr=0x0000` and transmits no Sd at all, which is the sibling project's own Session 47 defect (page-local transmit state not re-established across the boot-page change). `SIP_FORCE_PCMU=1` is required for these runs: a transcoded DS0 cannot carry Phase 3. Default is unchanged — `docs/v90_analogue_role.md`.
- **TRN1d may be 12x too short** — a working V.90 digital modem (the Eicon card, captured in `artifacts/eicon-digital-downstream/`) transmits **30000T / 3750 ms** of TRN1d, spending 94% of the §9.3.1.5 budget; `v90_trn1d_len()` sends 2496T / 312 ms. Verified bit-for-bit on two independent connecting calls. The `ME_V90_TRN1D_SYMBOLS` clamp (<=16000, `v90.c:121`) blocks testing the card's value. Untested live, and the leading candidate for why the Courier never decodes our Jd — `docs/eicon_downstream_comparison.md`.
- **Receive path against a foreign downstream — Phase 3 reads, DIL does not** — every other receive test is fed by our own transmitter, so a wrong-but-self-consistent assumption is invisible to it. `make eicon-rx-test` points the analogue-side decoders at the Eicon card's downstream. Sd, S̄d, TRN1d (30005T) and Jd now decode at the offsets an independent segmentation puts them at, on both fixtures. **DIL is two different jobs — do not conflate them.** On the analogue side the descriptor is an *input*: §8.4.1 sends it to the digital modem in Ja and §9.3.2.9 has the analogue modem "receive the DIL sequence it requested", so the receiver measures impairment rather than recovering anything, and §9.3.2.10 lets it stop once it has "enough". `v90_dil_measure.c` does that job — from **half a pass** it recovers a 3 dB pad and the RBS slot exactly, then turns what arrived into Ci/Mi/drn/rate under a measured noise margin and §8.5.2's Table 15 power cap. Noise thins the constellation from the bottom of the ladder (G.711's levels sit closest together there), §8.5.2 from the top (largest levels dominate the power sum), and measured it is the **combination** that moves the rate — either alone usually costs constellation and no bit/s. Two traps are documented and worth knowing before authoring a DIL descriptor: LTP must be coprime with 6 or an interval goes unprobed, and alignment must never be searched blindly. See `docs/v90_constellation_selection.md`. Never attempt blind alignment: G.711 is self-similar across chords (Ucodes u and u+16 differ by exactly 2×), so a one-Ucode-per-segment ladder repeats its shape 16 segments later and no scale-invariant score separates them; §9.3.2.8-9 means the receiver searches a window instead, hence the from/span API. `v90_dil_rx.c` solves the *other* job, recovering a descriptor from someone else's capture. Its acquisition no longer needs the cycle twice (a single pass decodes exactly, pinned by `V.90 DIL decoded from a single pass`), but the card's low-Ucode region still defeats it: at a uniform 66T segmentation 51 of 239 segments carry three Ucodes where §8.4.1 allows two, most likely because Hc is per-chord and the ladder crosses chords. A tool limitation, not a modem one — `docs/eicon_downstream_comparison.md`, Finding 5.

`docs/` holds a design note or investigation write-up per phase. Read the relevant one before changing that phase.

`AGENTS.md` is the parallel copy of this file for other coding agents; mirror guidance changes there.
