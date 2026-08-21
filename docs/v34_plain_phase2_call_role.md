# Plain V.34 against d-modem: the peer's call-role Phase 2 does not complete

Status as of 2026-08-21.  Rig and dial recipe: `tools/soak/v34_lapm_call.sh`
(peer forced to V.34 only with `AT+MS=34,0,2400,33600`, automode off), server
run as

```
VPCM_ME_VERBOSE=1 ME_V34_SPAN_FLOW_LOG=1 SIP_FORCE_PCMU=1 ME_MODE=v34 \
ME_DATA_FRAMING=lapm ME_V8_ANSWER_TONE=ansam_pr VPCM_G711_TAP_DIR=<dir> \
./sip_v90_modem ... --pty-link /tmp/v90modem
```

`tools/v34_phase2_timeline.py <tap.g711>` segments a G.711 tap into carrier
presence, which CC carrier (1200 Hz = call modem, 2400 Hz = answer modem) and
phase reversals.  **Use it rather than reading the two modems' logs against
each other** -- the peer's log clock and ours differ by a per-call offset, and
computing that offset wrong is what sent one round of this investigation
chasing a probe overhang that does not exist.  Two taps, one clock, no offset.

## Fixed here

11.2.1.2.6's post-L2 Tone A phase reversal is conditional on Tone B having
been detected, and we transmitted it off the end of our own L2.  11.2.1.1.3
has the call modem silent from its first Tone B reversal until it has received
L1/L2, and 11.2.1.1.5 has it raise Tone B only after that, so it is not
listening when our L2 ends.  Measured: our reversal at 7565 ms, the peer's
Tone B at 7630 ms.  `V34_TX_STAGE_POST_L2_WAIT_TONE_B` now holds Tone A until
Tone B is seen.  Before the fix our receiver then read the peer's INFO0c
repeats as its L1/L2 probe and "analysed" them; after it, the 11.2.1.2.7 path
runs on a real Tone B reversal.

## The blocker, and why it is not ours

Every call, the peer declares

```
V34HSHAKE: microstate RX_PHASE3_CALL=>TX_PHASE2_CALL
V34HSHAKE: microstate TX_PHASE2_CALL=>DET_SYNC
Repeated info0 is detected, errorrecovery is initialized in TX_PHASE2_xxx
```

20 ms after entering `TX_PHASE2_CALL`.  **It fires while our transmitter is
silent** (we are in `A_SILENCE` at that point), and a DPSK sync search over
our own transmit tap finds exactly the two INFO0a frames we meant to send and
no others.  Nothing we put on the line causes it.

The 11.2.2.1.1 recovery it enters has one exit, receiving an INFO0a, and that
exit is self-defeating in this role: 60 ms after accepting the frame,
`TX_PHASE1_CALL` reads it as a repeat and re-enters the recovery, which leaves
again on `Tone AB detected ending errorrecovery`, re-asserting the same cached
frame -- 7 rounds observed, 0.4-2.6 s apart, always the identical octets,
until it retrains and clears the call.

All three possible answers were measured, and all three fail:

| Our answer to the repeated INFO0c | Peer |
|---|---|
| INFO0a with bit 28 set (11.2.2.2.1, conformant) | livelocks in `TX_PHASE1_CALL` |
| INFO0a with bit 28 clear | livelocks identically -- the check is on repetition, not on the acknowledgement bit |
| no INFO0a (11.2.2.2.1's "detects Tone B and has received INFO0c" branch) | never leaves `DET_SYNC`; 13 s of silence, then Link Error |

`ME_V34_INFO0_RETRY=ack|noack|none` selects between them; `ack` is the
default and the conformant one.

Reproduced on two peer binaries (`slmodemd` and the older
`slmodemd.bak-prev92up`), so it is in the shipped SmartLink DSP, not in the
locally applied patches.

**Why the V.90 work never hit this.**  The peer runs a different Phase 2 state
family in each role -- `TX_PHASE1_ANS`/`RX_PHASE1_ANS` versus
`TX_PHASE1_CALL`/`RX_PHASE1_CALL` -- and only the answer-role one completes.
It raises the *identical* spontaneous "Repeated info0 ... TX_PHASE2_xxx" and
then recovers cleanly, going on to transmit L1/L2 (see any d-modem
`test-artifacts/*/slmodemd-live.log`).  V.90 §9.2.2 hands the analogue modem
the V.34 answer modem's timetable, so on a V.90 call this peer is in the role
that works -- which is why V.90 reaches data mode on the same rig and plain
V.34 never has.

## The way through, and what blocks it

Give the peer the answer role: we originate, it answers.  That is
`tools/soak/v34_originate_call.sh`, and it does not work yet for a reason
outside this repo -- **d-modem has no inbound call path at all**.  Its pjsua
setup registers `on_call_state` and calls `pjsua_call_make_call`; there is no
`on_incoming_call` callback and no `pjsua_call_answer`, so an INVITE to 6000
is never delivered to the DSP (`ATS0=1` is accepted and nothing rings).
Adding an accept-and-answer path to `/src/d-modem.c` on the rig is the next
step for this goal.
