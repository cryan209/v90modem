# Taking the analogue side of a V.90 call

This software has always been the **digital** modem: it injects PCM codewords
downstream and consumes a constellation the peer chose. The analogue role is
the other half — the side with a D/A → loop → A/D hop, which is therefore the
side that measures the line and decides the constellation (§5.4.3).

Wanting it is not academic. Every receive test in this tree is fed by our own
transmitter, so a wrong-but-self-consistent assumption is invisible; and the
analogue side of the loopback harness is the grader for the entire digital-side
suite, so if a convention is wrong in shared `vpcm_*` code, both ends agree and
the peer still fails. Running our analogue side against a real digital modem
breaks that circularity.

## Status: V.8, Phase 2 and Phase 3 run in a live call

`ME_V90_ROLE=analogue` now takes a call as far as the end of Phase 3. Phase 4
(§9.4) does not exist, so the engine says so and hangs up rather than running
out the training timeout.

```bash
ME_V90_ROLE=analogue VPCM_ME_VERBOSE=1 ./sip_v90_modem \
    --sip-server <registrar> --username <ext> --password <pw> \
    --pty-link /tmp/v90modem
```

Then `ATD<the digital modem's extension>` on the PTY. `ME_V90_ANALOGUE_HOLD=1`
keeps the call up after Phase 3 for capture; `ME_V90_ANALOGUE_DIL` picks the
descriptor to request (`measurement` by default, or `none`, `default-ja`,
`courier-style`, `smartlink-adi`, `smartlink-adi-qc`);
`ME_V90_ANALOGUE_UINFO` sets U_INFO (78 by default);
`ME_V90_ANALOGUE_SCR=1` transmits SCR rather than silence during DIL
(§9.3.2.9 permits either).

### The four modules

- [`v90_analogue_tx.c`](../v90_analogue_tx.c) — the §9.3.2 signals: S, S̄, PP,
  TRN, Ja carrying the Table 12 DIL descriptor, SCR.
- [`v90_analogue_rx.c`](../v90_analogue_rx.c) — the §8.4 signals coming back:
  Sd, S̄d, TRN1d, Jd, J'd and DIL, as a codeword state machine. The downstream
  *is* the DS0 stream, so there is no V.34 receiver in the analogue role at all.
- [`v90_analogue_phase3.c`](../v90_analogue_phase3.c) — the join, which is
  where §9.3.2's conditional structure lives: Ja ends on the Sd-to-S̄d
  transition, S starts on Jd, S̄ on J'd, and §9.3.2.10's S/S̄ pair on having
  enough DIL. Four events, and nothing else crosses.
- `modem_engine.c` — V.8 offers the role, SpanDSP runs Phase 2, and
  `prepare_v90_analogue_phase3_locked()` takes the modulator over at the seam.

### Where the engine hands over

SpanDSP's transmit stage reaching `FIRST_S` is the end of its Phase 2: INFO1a
has gone out and V.34's own S/S̄/PP/TRN would be next. §9.3.2.1 starts from the
same point with V.90's sequence, so the modulator is taken over there and
SpanDSP's Phase 3 never runs. Its *receiver* stops being fed at the same
moment — what arrives from then on is PCM codewords, and a V.34 demodulator
fed those reports events about signals that are not present.

`g_v90` — the digital-side Phase 3 state — is never created on an analogue-role
call. Every digital Phase 3/4 site in the engine is already guarded on it, so
leaving it unbuilt is what keeps the digital transmitter out of a call that
announced itself analogue, rather than a flag each of those sites would have to
remember to check.

Two things the engine had to get right beyond that:

- **U_INFO has to agree with INFO1a.** The analogue modem chooses the Ucode the
  digital modem will train Sd and TRN1d on (§8.4.4, §8.4.5) and announces it in
  Table 11 bits 25:31 — and its own receiver has to expect W = 16 + U_INFO.
  SpanDSP had the field hard-coded to 78, so `v34_set_v90_u_info()` was added
  and both ends of the engine read the same value.
- **The Phase 2 notch moves.** §8.2.3.1 puts the analogue modem's INFO at
  2400 Hz and the digital modem's at 1200 Hz — the mirror of the digital role,
  so the notch that removes our own CC echo goes on 2400, not 1200.

### What a run prints

`VPCM_ME_VERBOSE=1` logs each §9.3.2 stage as it is reached on both sides, and
at the end the measurement and what it implies:

```text
[ME] V.90 analogue RX: TRN1d (Sd 64 reps, S̄d 8 reps, TRN1d 0T, Jd 0 frames)
[ME] V.90 analogue TX: silence (post-Ja)
[ME] V.90 analogue Phase 3 complete: … 13 Jd frames (4-point), DIL 4200 symbols
[ME] V.90 analogue DIL measured: 65 Ucodes, 65 usable, gain -3.01 dB, …
[ME] V.90 analogue constellation: Mi = … drn=…, 44000 bps
```

That last line is the reason for the whole exercise: on the analogue side the
constellation decision is ours, derived from a measurement rather than offered
blind (docs/v90_mi_negotiation.md).

## Live against a real digital modem: through Phase 2, into the V.90 page

Dialled the Eicon card under emulation (`../modem-dsp-emu`, `./run
native-tower`) on 2026-08-10, both ends G.711 µ-law. The card completes Phase 2
with this side as the analogue modem, reads our INFO1a, and switches its
firmware to the V.90 page:

```text
[adsp] TrnProgress 0x0042 -> 0x0044 -> 0x0046 -> 0x004f  INFO_RX complete=0x0001
[adsp] overlay request page 14 V.90 DPCM -> 0x026a served
[adsp] bootpage 7 INFO (V.34/V.90 phase 2) -> 14 V.90 DPCM
```

Getting there took three fixes, each one found by a call:

1. **§9.2.2.1.8 — Tone A before INFO1a.** SpanDSP took the V.34 call-modem
   branch at the end of L1/L2 and sent INFO1a straight away. V.90 swaps the
   roles: the analogue modem is the *calling* party, and after probing it
   transmits Tone A and conditions its receiver for INFO1d (§9.2.2.1.8), only
   answering with INFO1a once that arrives (§9.2.2.1.9). Table 11 shows why the
   order matters — INFO1a bit 25 selects the carrier and bits 26:29 the
   pre-emphasis for the digital-to-analogue direction, both answers to what
   INFO1d offered. The digital modem waits to detect Tone A before sending
   INFO1d (§9.2.1.1.7), so sending INFO1a first deadlocks both ends.
2. **This card never sends INFO1d at all.** Decoding its own transmit
   (`./vpcm_decode --v34` on the capture) finds INFO0d at 4830 ms and no INFO1d
   anywhere in 26 s. It sits in its INFO receive states — `INFO_RX event=1,
   complete=0`, over and over — waiting for INFO1a, whose bits 37:39 are what
   its firmware reads to choose the V.90 page (the decision chain is pinned in
   the sibling project's Session 194–195). §9.2.2.2.4 offers two responses to a
   missing INFO1d, retrain or INFOMARKSa, and neither reaches this peer:
   INFOMARKSa *replaces* the Tone A it is waiting for. `ME_V90_ANALOGUE_INFO1D_TIMEOUT=info1a`
   sends INFO1a instead. Opt-in, because it is not one of the two.
3. **One INFO1a is not enough.** §8.2.3.1 permits a group of INFO sequences,
   and the digital side of this same code already repeats INFO1d four times for
   exactly that reason. INFO1a went out once, and a peer that arrives at its
   receive state late never completes a frame. It now repeats, four times by
   default and `ME_V90_ANALOGUE_INFO1A_REPEATS` more.

The recipe that reaches page 14:

```bash
cd ../modem-dsp-emu && ./run native-tower --run <n>      # answers as 6001
ME_V90_ANALOGUE_INFO1A_REPEATS=40 \
ME_V90_ANALOGUE_INFO1D_WAIT_MS=12000 \
ME_V90_ANALOGUE_INFO1D_TIMEOUT=info1a \
SIP_FORCE_PCMU=1 ME_V90_ROLE=analogue VPCM_ME_VERBOSE=1 ./sip_v90_modem \
    --sip-server <registrar> --username 6000 --password 6000 \
    --local-port 5062 --rtp-port 4100 --pty-link /tmp/v90modem-analogue
printf 'ATD6001\r' > /tmp/v90modem-analogue
```

`SIP_FORCE_PCMU=1` is not optional: the first attempt negotiated A-law against
a µ-law endpoint, and a transcoded DS0 cannot carry Phase 3 at all.

### Where it stops now, and why it is not ours

The card enters page 14, runs its Phase 3 states `0x0060` → `0x0074` for about
four seconds, sends **no Sd at all** (`./vpcm_decode --v90` on the capture finds
none, and that scan tries every U_INFO), then falls back to `0x0024` and
retrains. `run54.adsp.csv` says why, at the page boundary:

```text
boot=0x0007 trn=0x004f  tx_ptr=0x3764  tx_value=0x0000
boot=0x000e trn=0x0060  tx_ptr=0x0000  tx_value=0x0000   <- and zero for the whole page
```

The V.90 page comes up with no transmit source wired to it. That is the
emulator harness's own documented defect — the sibling project's Session 47,
"page-local transmit state that the real host re-establishes across a boot-page
change and our supervisor does not" — and it is in `../modem-dsp-emu`, not
here. Our receiver hunting Sd and finding none is the correct behaviour against
a peer that is transmitting nothing.

So the analogue role is blocked on the *instrument*, not on the modem. Against
real hardware the same build would carry on into Phase 3; there is no evidence
either way yet, because the only digital modem in the lab is this one.

## What is left

1. **A digital peer that transmits Sd.** The emulated card does not, for a
   reason in its own harness (above). Either fix that in `../modem-dsp-emu` —
   the boot-page 7 → 14 transmit handoff — or find another digital modem.
2. **Phase 4 (§9.4)** — CPt, then a CP built from the measurement
   `v90_analogue_phase3_measurement()` already returns, then E/B1.
3. **Data mode** — receive the mapped downstream instead of transmitting it.
4. **Retrain (§9.5.2.1)** — §9.3.2.4's and §9.3.2.7's deadlines are detected
   and reported, but the response to them is a hang-up, not a retrain.

## When it still falls back to V.34

Two cases, and both are refusals to advertise something this side cannot honour:

- **An answering call.** V.90 puts the analogue modem on the calling side —
  §9.2 has it send INFO0a and INFO1a — and SpanDSP's Phase 2 keys the analogue
  INFO variants off `calling_party`. An answering call taking this role would
  be lying about which INFO it is going to send, so it falls back and says so.
  Dial out to use the role.
- **No usable DIL descriptor.** The analogue modem authors the DIL, and
  §9.3.2.9 measures exactly what it asked for. A preset that will not load
  leaves nothing to ask for.

Default is unchanged: without the environment variable the engine is the
digital modem exactly as before.

## What V.8 had to change (§Table 5)

| field | digital role | analogue role |
|---|---|---|
| `pstn_access` | `V8_PSTN_ACCESS_DCE_ON_DIGITAL` | *unchanged* |
| `pcm_modem_availability` | `V8_PSTN_PCM_MODEM_V90_V92_DIGITAL` | `V8_PSTN_PCM_MODEM_V90_V92_ANALOGUE` |
| V.92 QC/QCA octet | `0x45` / `0x47` | omitted |

Only the availability field moves. `pstn_access` answers a different question —
how this DCE reaches the network — and the answer does not change with the role
we choose to play: the media is G.711 over SIP, so `DCE_ON_DIGITAL` stays set
because it is true. The V.92 octet encodes the *digital* modem's capabilities
and has no analogue equivalent wired up, so an analogue-role offer omits it
rather than guessing.

A peer could in principle object that a digital access has no analogue loop on
which to learn impairment. That is an empirical question, and the lab case this
exists for — dialling the Eicon card with both ends on G.711 — is exactly where
to answer it.

## What is already built

The measurement half of the analogue role is done and tested, because it turned
out to be separable from the signalling:

- **Impairment measurement** — [`../v90_dil_measure.c`](../v90_dil_measure.c).
  Given the DIL descriptor we sent, measure what came back: per-Ucode,
  per-data-frame-interval level, spread, and the RBS slot mask.
- **Constellation and rate** — Ci per interval under a measured noise margin
  and §8.5.2's Table 15 power cap, then `drn` and bit rate.
  [`v90_constellation_selection.md`](v90_constellation_selection.md).
- **Descriptors to send** — [`../v90_dil_presets.c`](../v90_dil_presets.c),
  with `v90_dil_desc_validate()` to catch a descriptor that would measure
  nothing in an interval.

So the analogue side already knows what to ask for and what to do with the
answer. What it cannot do is conduct the conversation.

## The chokepoint is not the modulation

It is worth being precise about this, because "we have no V.34 transmitter" is
the wrong summary and would send someone down a very long road for no reason.

**SpanDSP's V.34 transmitter is real and already in the live path.**
`spandsp-master/src/v34tx.c` is 7 285 lines, `modem_engine.c` calls `v34_tx()`
during training and in `ME_DATA` for plain V.34 calls, and V.34 data transmit
works today. It also already builds the analogue side's Phase 2 INFO: not the
plain V.34 `prepare_info1a()`, but `prepare_v90_info1a()` and
`v90_info1a_sequence_tx()` (`v34tx.c:958`, `:1146`), which emit Table 10/V.90
with U<sub>INFO</sub> in bits 25:31 and the two symbol-rate fields. They are
selected by `v90_mode && calling_party` in `info1_baud_init()`. INFO0a needs
nothing: Table 8 is bit-for-bit the V.34 INFO0.

There is no DIL descriptor length in INFO1a — Table 11 has no such field. The
descriptor travels in Ja and Ja alone (§8.3.1), which is why the Ja transmitter
was the piece worth building first.

What was missing sat *on top of* the modulation, and is now built:

1. ~~V.90's additions to INFO1a~~ — present in the vendored tree.
2. ~~The Phase 3 analogue signals~~ — `v90_analogue_tx.c`, with the GPA
   scrambler (§8.3) and the §10.1.3.3/V.34 differential encoder Ja shares
   with J.
3. **A role switch in SpanDSP's V.90 mode.** Resolved by not needing one for
   Phase 3: `v34_tx_start_external_symbols()` points the modulator's per-baud
   symbol source at a callback and leaves the transmit state machine, and the
   receiver, alone. That mattered more than it sounds — SpanDSP's Phase 3 TX
   stages advance on *RX* state (`get_trn_baud()` reaches into `s->rx` to decide
   when TRN ends), and in the analogue role there is no V.34 receiver to
   advance them: the downstream is PCM.

## Testing it without a second modem

`eicon_adsp_sip.py`, in the separate `../modem-dsp-emu` project, runs the Eicon
card's own shipped V.90 firmware as a SIP endpoint, answer-only. `sip_modem.c` can originate (ATD via
`pjsua_call_make_call`), so an analogue-role build can dial the card directly —
no Asterisk in the path, no hardware. That gives a real digital peer whose
internal state is readable per-sample (`TrnProgress`, `Rstatus`, eye words) via
its `adsp.csv`, which is a far better failure signal than the Courier's silence.

Before that, `v90_analogue_tx_test --write-ulaw <file>` writes the modulated
Phase 3 with the peer's cues delivered on a timer, so `./vpcm_decode --p3` can
be pointed at it. That already confirms the carrier and PP independently (288
symbols, six blocks, correlation 0.96 at 3200 baud high). It does *not* confirm
Ja: that tool cannot segment Ja from TRN — both are scrambled 4-point streams —
which is the same limitation as Finding 5 in
[`eicon_downstream_comparison.md`](eicon_downstream_comparison.md), not a
statement about the transmitter. The Ja check that does hold is in
`v90_analogue_tx_test` itself, which demodulates its own symbols back to a
CRC-checked descriptor.
