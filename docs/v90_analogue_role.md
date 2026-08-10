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

Getting there took three fixes, each one found by a call. **Two of them are
now obsolete** — see "Measured live against the card (run 55)" below: with the
§9.2.2.1.3–9.2.2.1.6 choreography corrected the card sends INFO1d unprompted,
so neither `ME_V90_ANALOGUE_INFO1D_TIMEOUT=info1a` nor a raised
`ME_V90_ANALOGUE_INFO1A_REPEATS` is needed, and the latter is now harmful.
They are kept here because the reasoning is what led to the real fix:

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

### Where it stops, and why the evidence points back at us

The card enters page 14, runs its Phase 3 states `0x0060` → `0x0074` for about
four seconds, sends **no Sd at all** (`./vpcm_decode --v90` finds none, and that
scan tries every U_INFO), then falls back to `0x0024` and retrains.
`run54.adsp.csv` shows `tx_ptr` going `0x3764` → `0x0000` at the page boundary
and staying there, which matches the sibling project's Session 47 note about
page-local transmit state.

**Superseded — see run 55 below.** This section's conclusion was right: the
divergence was on this side, in the tone choreography. The card now advances to
`0x007a` and validates our Phase 3, though it still transmits no Sd.

**Do not read that as an emulator defect.** The same card under the same
emulation has reached full V.90 data mode against two different analogue
modems and produced a `CONNECT` from a CX93001 (Sessions 87, 190, 237). A peer
that behaves correctly gets Sd out of it. Three things say the remaining
divergence is on this side:

1. **The workarounds are the symptom.** A conformant analogue modem does not
   need `ME_V90_ANALOGUE_INFO1D_TIMEOUT=info1a`, and does not need INFO1a
   repeated forty times. The card sends INFO1d to real modems; that it does not
   send one to us means our Phase 2 is still not presenting what §9.2.1.1.7
   requires before it will — Tone A at the right moment, after the L1/L2 it is
   expecting.
2. **§9.3.1.1–9.3.1.3 make Sd conditional on Ja.** The digital modem conditions
   its receiver to detect Ja, waits up to 500 ms, and only then transmits Sd.
   A card sitting in page 14 with nothing to transmit is consistent with never
   having detected our Ja, and our Ja has no foreign grader: `v90_analogue_tx_test`
   demodulates it with our own inverse mapping, which cannot catch a convention
   that is wrong but self-consistent — the exact trap the Eicon fixture caught
   twice on the receive side.
3. **Our analogue side did not complete Phase 2 against our own digital side
   either** — analogue stalled at `FIRST_B_INFO_SEEN`, digital at
   `V90_PHASE2_B_INFO0_SEEN`, each waiting for a reversal from the other. That
   was a reproduction with no card in it, and it is **now fixed**; see the
   section below.

## The §9.2.2.1.3–9.2.2.1.6 tone choreography (fixed 2026-08-10)

The analogue role was running SpanDSP's V.34 **call modem** tone stages —
`FIRST_B`, `FIRST_B_INFO_SEEN`, `FIRST_NOT_B_WAIT`, `FIRST_B_SILENCE`, … —
because `initial_ab_not_ab_baud_init()` picked the stage family off
`calling_party` alone.

The tone *frequency* was right by accident: `v34_restart()` already gives the
V.90 analogue modem the 2400 Hz CC carrier, which is Tone A. What was wrong was
the **timetable**. The rule the code was missing:

> Which tone a modem transmits follows its **role**, not the call direction.
> V.34 gives Tone A to the answer modem (§11.2.1.2) and Tone B to the call
> modem (§11.2.1.1). V.90 keeps both tones and both timetables and hands them
> to the other end of the call: the analogue modem is the *calling* party and
> runs §9.2.2.1, which is §11.2.1.2 word for word with INFO0d/INFO1d in place
> of INFO0c/INFO1c; the digital modem answers and runs §9.2.1.1, which is
> §11.2.1.1.

So the Tone A transmitter is the side where `calling_party` and `v90_mode`
agree — the same predicate `v34rx.c` already used to pick the receive carrier.
The whole correct analogue choreography was therefore already in the tree, as
the V.34 answer-modem A family; it just was not reachable from the V.90
analogue role. Three changes in `v34tx.c`:

1. **`initial_ab_not_ab_baud_init()` routes on the role.** V.34 caller →
   `FIRST_B`; V.90 digital answerer → `V90_PHASE2_B`; everything else (V.34
   answerer, V.90 analogue) → `INITIAL_A`.
2. **End of the analogue's own L1/L2 goes to `second_a_baud_init()`**, not
   `pre_info1_a_init()`. §9.2.2.1.5's probe is only the *first* of the two
   rounds. Off the end of L2 the analogue modem owes 50 ms of Tone A, a
   reversal, 10 ms more and then silence (§9.2.2.1.6), so §9.2.1.1.6 can time
   its Tone B reversal against it; only after that reversal does it receive the
   digital modem's L1/L2 (§9.2.2.1.7) and finally hold Tone A for INFO1d
   (§9.2.2.1.8). Jumping straight to `PRE_INFO1_A` skipped the entire second
   round — the digital modem sat in `SECOND_B` waiting for a Tone A reversal
   that never came, and so never sent INFO1d.
3. **`FIRST_A` waits for the peer's tone, not just its INFO0.** §9.2.2.1.3 and
   §11.2.1.2.3 are identical here: *"After Tone B is detected **and** Tone A has
   been transmitted for at least 50 ms"*. The code reversed on `INFO0_OK`
   alone, which is only the same thing when the peer's tone is already up by the
   time its INFO0 decodes. **In V.90 it never is:** §9.2.1.1.1–9.2.1.1.2 have
   the digital modem start Tone B *after* it receives INFO0a, so our INFO0d
   decode precedes its Tone B by a whole INFO0a length. Reversing there put the
   reversal in front of the digital modem's tone detector before it had its 20
   bauds of steady Tone A to measure against — the reversal was swallowed and
   both sides waited on each other. `TONE_SEEN` before a good INFO0 remains the
   §9.2.2.2.1 error case; after one it is the §9.2.2.1.3 trigger.

   **This one is V.90-only, deliberately.** §11.2.1.2.3 says the same thing,
   but the V.34 answer modem cannot test it: `process_rx_info0()` parks its
   receiver in `V34_RX_STAGE_TONE_B`, whose detector never publishes
   `TONE_SEEN` — the assignment in `v34rx.c` is commented out — so requiring it
   there would deadlock plain V.34. The V.90 analogue modem lands in
   `V34_RX_STAGE_TONE_A` instead (`calling_party` is set), which does publish
   it. V.34 keeps the `INFO0_OK` shortcut, which is sound for it because the
   call modem's Tone B is already running by the time INFO0c decodes.

A fourth defect surfaced next to these, on the digital side: the
`FIRST_B_SILENCE` recovery branch for "analogue L1/L2 arrived after a missed
second reversal" set `stage = V90_WAIT_TONE_A_REV` without moving
`current_getbaud`. That stage's case lives in `get_initial_fdx_a_not_a_baud`,
so the digital modem fell off the end of the B switch every baud and held
Tone B for the rest of the call. Fixed alongside.

Both halves now mesh, in both laws:

```text
analogue  INITIAL_A -> FIRST_A -> FIRST_NOT_A -> FIRST_NOT_A_REV_SEEN -> SECOND_A
          -> L1/L2 -> POST_L2_A -> POST_L2_NOT_A -> A_SILENCE -> PRE_INFO1_A -> INFO1a
digital   V90_PHASE2_B -> V90_PHASE2_B_INFO0_SEEN -> FIRST_NOT_B_WAIT -> FIRST_NOT_B
          -> FIRST_B_SILENCE -> FIRST_B_POST_REV_SILENCE -> SECOND_B -> SECOND_B_WAIT
          -> SECOND_NOT_B -> L1/L2 -> INFO1d
```

INFO1d and INFO1a both decode CRC-clean and both sides enter Phase 3.

The regression test is the coupled harness in `vpcm_loopback_test.c`
(`test_spandsp_v90_info_startup_over_analog_g711`), which drives a V.90
analogue caller against a V.90 digital answerer through a G.711 channel. It is
the only in-tree check that the two halves of §9.2.1.1/§9.2.2.1 mesh, it costs
about 0.1 s, and it now runs as part of `make test` rather than staying behind
`--experimental-v90-info`. Its pass condition changed with the fix: the digital
side is no longer asked for `INFO1_OK` on INFO1a, because `v34rx.c`
deliberately clears that event as it runs `v90_enter_phase3_from_info1a()` so a
stale one cannot block Phase 3 J detection. Its evidence of having read INFO1a
is U_INFO, an INFO1a field (§8.2.3.2 Table 10). What the *analogue* side owes
is `caller_saw_info1`: §9.2.2.1.9 forbids it sending INFO1a until INFO1d has
arrived, so that is what separates a completed Phase 2 from one that timed out
and sent INFO1a anyway.

### Measured live against the card (run 55, 2026-08-10)

Four calls into `../modem-dsp-emu` `./run native-tower`, both ends G.711 µ-law
over the lab registrar. The results are larger than the loopback suggested.

**Phase 2 completes with no workarounds at all, in 2823 ms.** Before this fix
it needed all three of `ME_V90_ANALOGUE_INFO1D_TIMEOUT=info1a`,
`ME_V90_ANALOGUE_INFO1A_REPEATS=40` and `ME_V90_ANALOGUE_INFO1D_WAIT_MS=12000`,
and took 7203 ms. The run that produced the numbers below set none of them:

```bash
SIP_FORCE_PCMU=1 ME_V90_ROLE=analogue VPCM_ME_VERBOSE=1 ./sip_v90_modem \
    --sip-server <registrar> --username 6000 --password 6000 \
    --local-port 5062 --rtp-port 4100 --pty-link /tmp/v90modem-analogue
printf 'ATD6001\r' > /tmp/v90modem-analogue
```

**The card sends INFO1d.** This is the headline: "this card never sends INFO1d
at all" was recorded above as a property of the card, and it was ours.
§9.2.1.1.7 has the digital modem wait to detect Tone A after the L1/L2 it is
expecting before it sends INFO1d, and the analogue side was never transmitting
the §9.2.2.1.6 Tone A that section waits for. With the choreography right it
arrives unprompted — a 109-bit frame, CRC 0, and its contents are exactly what
a digital modem's line probe should yield:

```text
Rx - info CRC result 0x0 (target_bits=93)
Rx - info raw bytes: 00 20 68 80 00 01 40 07 00 80 e0 37
Rx INFO1c:
  Baud rate 2400 use high carrier ... max data rate = 24000bps
  Baud rate 3200 use low carrier  ... max data rate = 31200bps
Tx - INFO1d received after 275 bauds of Tone A, sending INFO1a (9.2.2.1.9)
```

(`Rx INFO1c` is the log label; §8.2.3.2 Table 9 makes INFO1d identical to
INFO1c, so the same decoder prints it.) The offline `--v34` scan of the card's
transmit misframes this one; our own receiver's CRC over a 109-bit frame is the
authority, and only the digital modem sends 109 bits.

**Workarounds 2 and 3 above are obsolete, and 3 is now actively harmful.**
`ME_V90_ANALOGUE_INFO1A_REPEATS=40` spends 4801 ms transmitting INFO1a repeats
the card no longer needs, which pushes our Phase 3 start from 7.4 s out to
11.8 s — past the card's page-14 dwell, so we began transmitting S at the exact
moment it gave up. Leave both unset.

**The card gets further into page 14.** It used to run `0x0060 → 0x0074` and
send nothing. It now runs

```text
0x0060 -> 0x0062 -> 0x0068 -> 0x0070 -> 0x0072 -> 0x0074 -> 0x0076 -> 0x007a
```

in 300 ms, with `DI_control=0x2000[rx0_valid]` at 0x0072 — it is receiving and
validating our Phase 3. It then holds 0x007a for ~3.8 s, raises
`flow_blocked`, and falls back to page 7 / `0x0024`.

**Still no Sd at this point — and that turned out to be ours too.** See the
next section: it was the upstream carrier. (The `0 payload / 3596 mark fill`
line cited here originally is *not* evidence about the line — it is the
B-channel data source, and says nothing about what the ADSP transmits.)

`ME_V90_ANALOGUE_HOLD=1` now also holds the call open on the §9.3.2
deadline path, not only after a complete Phase 3. That path is the one a
capture most needs held: the question it answers is what the digital modem does
*after* our Ja deadline, and hanging up there destroys it. Our deadline fired
at 9.8 s while the card was still advancing.

### The upstream carrier, and what it unlocked (run 57/58, 2026-08-11)

"No Sd" above was wrong twice over, and both were ours.

**The upstream carrier was hardcoded.** `prepare_v90_analogue_phase3_locked()`
set `cfg.high_carrier = true` with a comment claiming INFO1d directs the
upstream high. It does not. §8.2.3.2 Table 9 makes INFO1d identical to V.34's
INFO1c, and V.34 §10.1.2.3.4 has INFO1c's per-symbol-rate block carry the
carrier and pre-emphasis **for the answer modem's transmitter** — which here is
us, since §9.2.2.1.9 puts the analogue modem in the V.34 answer-modem role. The
card asks for the *low* carrier at 3200 baud, so our entire Phase 3 went out at
1920 Hz against the 1829 Hz its receiver was tuned to. 91 Hz is not something
carrier recovery pulls in. Its §9.3.1.1 Ja detector had nothing to find, so
§9.3.1.3 never fired and it transmitted **nothing** — measured as 4.5 s of pure
`0xFF`, RMS 0, for its whole page-14 residency (run 56).

The same bug ran through `s_not_s_baud_init()`, whose `calling_party` branch
excluded `v90_mode` and so fell to the "no INFO1 received" default: 3 dB down
and unemphasised, on a call where INFO1d had decoded cleanly and asked for 0 dB.
Both now read INFO1d.

With the carrier right the card transmits a textbook downstream:

```text
[ 9278.9 ms] V.90 Sd: W_UCODE=64 (U_INFO=48), 64 reps (384 symbols, 48.0 ms)
[ 9326.9 ms] V.90 S̄d: 8 reps (48 symbols, 6.0 ms)
[ 9332.9 ms] V.90 TRN1d: 30005 symbols (3750.6 ms) at U_INFO=48, descrambled to ones
[13083.5 ms] V.90 Jd+J'd: ~11019 Jd frame repetitions
```

and its states run on past `0x007a` for the first time — `0x007b → 0x007c →
0x0080 → 0x00b0` — reporting `SNRatio 31.5 dB` and `upstream quality 0x0067` on
what we send it. 30005T of TRN1d is the same 94% of the §9.3.1.5 budget the
`artifacts/eicon-digital-downstream/` fixtures spend, against our own 2496T.

**Then we still could not see it, for a second reason: W.** §8.4.4 puts Sd at
W = Ucode(16 + U_INFO), and U_INFO is the analogue modem's choice, announced in
INFO1a bits 25:31. `v90_analogue_rx.c` built the six Sd codewords from the value
it had announced and accepted nothing else. **The card does not honour it:**
told U_INFO = 78 it transmits Sd at W = 64, and told 48 it transmits at W = 35.
Both in-tree fixtures — calls a Courier answered with CONNECT — also run at
W = 64. A receiver pinned to what it requested is blind to all of them, and
loopback can never show it, because our own transmitter reads the same variable.

So acquisition now takes W off the wire. The structure is what identifies Sd:
four W slots at one common non-zero level with signs + + − −, and two zero
slots. `vpcm_decode`'s offline scanner had already reached the same conclusion
from the other direction, by scanning every U_INFO. §8.4.5's TRN1d level is
derived from the acquired W as well (W − 16), since the two are locked 16 apart
whatever the peer chose — without that the S̄d→TRN1d boundary still missed.

Live result: `Sd 64 reps, S̄d 8 reps, TRN1d 24000T+`. Ja ends on the §9.3.2.4
Sd→S̄d transition as it should. What remains is the TRN1d→Jd seam — see "What
is left".

### The in-tree pair

Also worth running, and the fastest loop of the three:

```bash
SIP_FORCE_PCMU=1 VPCM_ME_VERBOSE=1 ./sip_v90_modem --sip-server <registrar> \
    --username 6001 --password 6001 --local-port 5064 --rtp-port 4200 \
    --pty-link /tmp/v90modem-digital        # then ATS0=1
SIP_FORCE_PCMU=1 ME_V90_ROLE=analogue VPCM_ME_VERBOSE=1 ./sip_v90_modem \
    --sip-server <registrar> --username 6000 --password 6000 \
    --local-port 5062 --rtp-port 4100 --pty-link /tmp/v90modem-analogue  # ATD6001
```

### Phase 3 completes against the card (run 7, 2026-08-11)

The whole of §9.3.2 now runs in a live call, ending in a measurement:

```text
[ME] V.90 analogue RX: TRN1d   (Sd 64 reps, S̄d 8 reps)
[ME] V.90 analogue RX: Jd      (TRN1d 30006T)
[ME] V.90 analogue TX: S (awaiting J'd)
[ME] V.90 analogue TX: S-bar (post-J'd)
[ME] V.90 analogue RX: DIL     (20 Jd frames)
[ME] V.90 analogue TX: Phase 4
[ME] V.90 analogue Phase 3 complete: Sd 64 reps, S̄d 8 reps, TRN1d 30000T,
     20 Jd frames (4-point), DIL 4200 symbols
[ME] V.90 analogue DIL measured: 65 Ucodes, 39 usable, gain -11.77 dB,
     RBS slots 0x00, coverage 53%
[ME] V.90 analogue constellation: Mi = 39 39 37 39 39 39, drn=17, 49333 bps
```

and the card walks `0x00b0 → 0x00b1 → 0x00b2 → 0x00b3 → 0x00b4 → 0x00b6 →
0x00c0` in 940 ms rather than stalling 23 s at `0x00b1`. Three fixes, and the
first two are the same mistake in two places.

1. **TRN1d's level is not 16 Ucodes below Sd's.** §8.4.4 puts Sd at
   Ucode(16 + U<sub>INFO</sub>) and §8.4.5 puts TRN1d at Ucode(U<sub>INFO</sub>),
   so on a peer that honours U<sub>INFO</sub> the two indices are 16 apart, and
   `sd_set_w()` derived one from the other. What the card actually holds is the
   *level* ratio those indices imply, not the index gap:

   | stream | Sd Ucode | µ-law level | TRN1d Ucode | level | ratio |
   |---|---|---|---|---|---|
   | `call1-connect-32000.ulaw` | 64 | 495 | 48 | 231 | 2.14 |
   | live, U<sub>INFO</sub> = 48 | 35 | 123 | **22** | 57 | 2.16 |
   | §8.4.5 applied to the live W | 35 | 123 | 19 | 45 | 2.73 |

   Both are 6.6 dB. The arithmetic and the ratio agree exactly when Sd lands on
   a segment boundary (mantissa 0) — which is what Ucode 64 is, and what *both*
   in-tree fixtures use. So the rule read as correct on every offline test this
   tree has. The S̄d→TRN1d boundary is now matched structurally instead: S̄d
   carries only two magnitudes, W and zero, so any third one is the boundary,
   and the level is taken from the wire the same way W already was. It is
   confirmed rather than assumed — a level that does not descramble to §8.4.5's
   ones inside 256 symbols is abandoned.

2. **A failed Jd search left the TRN1d descrambler stranded.** On finding no
   frame in the window the receiver went back to reading TRN1d without
   re-seeding, so the register sat `SCRAMBLER_HISTORY` bits behind the stream,
   which guarantees another false break a couple of dozen symbols later, which
   fails the same way. One corrupted octet put it in that loop for the rest of
   the call. `trn1d_resume()` re-seeds out of the sign buffer and re-scans from
   just past the break, which is possible now that TRN1d is decoded through an
   explicit cursor rather than as a side effect of arrival.

3. **J'd needs a witness that a slipped octet cannot move.** The Jd walk is
   positional — 72 bits per frame from where the frame was located — so an
   inserted or lost octet moves every boundary after it. Inside Jd that costs
   one frame and re-locks on the next §8.4.2 sync run, which is why 27 of the
   card's 38 frames still decoded; landing on the last frame it eats J'd's
   twelve zeros instead, and DIL is never entered. Measured: the card sent a
   textbook DIL for 23 s while this receiver sat in Jd through all of it. §8.4.1
   gives the second witness for free — DIL sweeps the ladder the descriptor
   asked for and leaves the Jd codeword on its first symbol, so eight
   consecutive codewords off that level are J'd having ended, and the first of
   them is DIL's first symbol.

**Clock recovery must not slip a DS0 stream.** `sip_modem.c` inserted or
dropped one octet per frame on the DPLL's signal, in both the linear and the
G.711 paths. On a modulated carrier that is what clock recovery is *for* — the
timing loop absorbs it. On the digital modem's DS0 output it is data
corruption: the octets are the constellation, and one extra shifts every §5.4
data frame after it and desynchronises the §5.3 scrambler. It fired one to two
times a second on this lab path. `me_rx_g711_slip_permitted()` now returns false
for the whole of an analogue-role call (the predicate is `g_v90a != NULL`, so
V.8 and Phase 2 are untouched), and the drift is left to show up somewhere that
can absorb it.

### Capturing and replaying a call

`ME_G711_CAPTURE=<prefix>` records the received octets to `<prefix>.rx.ulaw`
byte for byte as the engine consumed them, and

```bash
./v90_analogue_rx_test --trace <file.ulaw> [u_info]
```

replays any downstream through the Phase 3 receiver with a stage trace. That
pair is what turned this from a live-call problem into an offline one: an
engine log cannot tell a receiver that has not reached Jd yet from one
oscillating in and out of it, and the trace can.

### Phase 4 exists, and the call reaches it (2026-08-11)

`v90_analogue_phase4.c` is the §9.4.2 receiver and the §8.5.2 CP builder;
`v90_analogue_tx.c` gained the §9.4.2 transmit stages, which are the same
modulation as Ja — §8.5.2 sends CP "according to §10.1.3.9/V.34", which is J's
— so they are fed from the same symbol pump rather than a second one beside it.
The handover happens inside `v90_analogue_phase3.c`, at the one moment both of
its conditions hold: the transmitter has finished §9.3.2.10 *and* a measurement
exists. Both matter and neither implies the other — a zero-length DIL reaches
the first without the second — and nothing after R̄i can be read without the
second, because §8.6.5 and §8.6.3 map TRN2d and MP with the CPt the measurement
produces.

Live, the engine now says:

```text
[ME] V.90 analogue Phase 4 started (§9.4.2.1): CPt drn=22 (30 bits/frame),
     CP drn=17 (37 bits/frame, 49333 bps), Sr=1, u-law
[ME] V.90 analogue Phase 4 RX: Ri (Ri 73T, …)
[ME] V.90 analogue TX: CPt
```

and the card advances past `0x00c0` to `0x00c2 → 0x00c4`, which is new.

**Two things Table 14 sets traps for, both handled in
`v90_analogue_phase4_build_cp()`.** CP's rate field is (drn + 20) bits per
six-symbol frame and CPt's is (drn + 8), so the *same line rate is a different
number in each*, and getting it wrong puts the digital modem's mapper on a
different frame length than our demapper. And §8.5.2 caps CP's average power at
3 dB above CPt's — satisfied by construction here, by naming the same
constellations in both and changing only the rate. CPt's field cannot always
reach CP's D (22 is its maximum, so 30 bits per frame); where it cannot, CPt
runs at the highest rate it can express, which is always encodable over the
same constellation and is more than TRN2d and MP need.

### Where Phase 4 stops: the card never leaves Ri

The card's entire Phase 4 downstream is two codewords, ±Ucode 22, in the
`+ + + − − −` of §8.6.4 — 79 979 runs of exactly three signs over thirty
seconds. It is a textbook Ri and it never becomes R̄i, so there is no TRN2d and
no MP. §9.4.1.1 has the digital modem "send signal Ri … and condition its
receiver to receive a CPt sequence" and §9.4.1.2 sends R̄i only "after receiving
a CPt sequence", so **the card is not accepting our CPt**. That is the next
thing to establish, and the leading suspicion is its content rather than its
framing: the DIL came back 11.77 dB down, so the levels our masks name may not
be levels this card will transmit.

That capture also cost a receiver fix worth recording. The stream carries 24
anomalies in those thirty seconds — runs of one and five signs where every
other run is three — and a three-symbol reversal test fires on them and reports
a §9.4.2.2 transition that never happened. §8.6.4's R̄ reverses *every* sign, so
a real reversal disagrees on every symbol, while a slipped octet shifts the
alignment and disagrees on some slots only. The test is now two whole periods
of consecutive disagreement, and a disagreement run that ends before that
re-acquires the alignment instead of carrying a wrong one.

`v90_analogue_rx_test` covers the receiver against a Phase 4 downstream
generated by `v90.c` — the digital side of this same tree, which is the pairing
a real call has — through Ri, the R̄i transition, TRN2d demapping to §8.6.5's
ones, and a Table 16 MP′ with its CRC. What that cannot check is the convention
questions the Eicon fixtures answer for Phase 3: there is no foreign Phase 4
downstream in this tree, so §8.6.4's level and §8.6.5's frame alignment are
still agreed between two halves of one codebase.

## What is left

1. **Get the card to accept our CPt**, which is what stands between here and
   TRN2d/MP. See above.
2. **B1 (§9.4.2.5)** — E is transmitted; B1 is one data frame of scrambled ones
   through the *data mode* mapper, which the analogue role does not have (the
   upstream data path is still SpanDSP's V.22bis placeholder). The transmitter
   holds silence there rather than send something that is not B1.
3. **Data mode** — receive the mapped downstream instead of transmitting it.
4. **Retrain (§9.5.2.1)** — §9.3.2.4's and §9.3.2.7's deadlines are detected
   and reported, but the response to them is a hang-up, not a retrain.
4. **U<sub>INFO</sub> = 78, the default, does not work against this card.** At
   78 it blocks at `0x007a` and falls back to page 7; at 48 it completes. The
   runs above all set `ME_V90_ANALOGUE_UINFO=48`. Why the announced value
   changes a decision the card makes about a signal it chooses its own level
   for is not established.

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
