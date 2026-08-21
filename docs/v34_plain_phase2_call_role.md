# Plain V.34 against d-modem: Phase 2 and Phase 3 now complete

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

## Superseded: the "peer's call-role Phase 2 cannot complete" reading

**That was wrong, and the evidence for it was ours.**  Everything below the
next heading is kept because the measurements are real and the traps are worth
knowing, but the conclusion it reached is not.  What the peer needed after the
11.2.2.2.1 INFO0 recovery was the *first* Tone A phase reversal of 11.2.1.2.3,
not the probe: that reversal is what makes it answer with its own Tone B
reversal, fall silent per 11.2.1.1.3, and only then arm its probe receiver.
See the commit "V.34 Phase 2 completes against the SmartLink peer".

Phase 3 then failed for a second reason, also ours and also invisible to
loopback: **10.1.3.7's S-bar was never rotated**.  The code wrote the 180
degree rotation as `lastbit.re = -lastbit.re`, which is a rotation only where
the imaginary part is zero, and the alternation lands on the zero-real-part
point every time -- so S-bar went out identical to S and the S-to-S-bar
transition, which 11.3.1.1.2 hangs the call modem's entire equalizer training
off, was not on the wire at all.  `tools/v34_phase3_verify.py` is what found
it: PP correlates at 0.97 against 10.1.3.6's own definition, so PP, the symbol
rate and the carrier were all right, and stepping back through the symbols
before it showed the S alternation running unbroken into PP.

Live, with both fixed: `S-S1 is detected, rxsymcnt = 128` where it read 150-152
before, `equerr` 25132 -> 68 -> 60 where it had been pegged at 32767 from the
first reading on every call, and the peer goes on to transmit its own PP, TRN
and J, detect our Phase 4 S, and complete the MP exchange -- it logs `MP
detected, starting MP' txmit`.

**Where it stops now:** our Phase 4 receiver does not decode the peer's MP.
Our own Phase 4 TRN ones-lock reaches 99%, so the receiver is tracking, but the
88-bit MP frames come out with a few bit errors each and no CRC ever validates,
so we never send MP-prime and the peer retrains.  That is the frontier
`docs/v34_spec_gap.md` already names -- foreign-modem data mode after E/B1 --
reached for the first time.

## Where the Phase 4 MP exchange stands, measured

`V34_MP_RX_DUMP=<file>` writes one line per Phase 4 symbol -- the differential
and absolute quadrant decisions and the symbol magnitude -- and
`tools/v34_mp_offline.py` reads it back and tries every interpretation of those
dibits against the frame CRC.  That is the right oracle and the only one:
10.1.3.9 leaves nothing to search once J has given the constellation, but
**neither a TRN of scrambled ones nor MP's own 17-bit all-ones frame sync can
tell one bit order from the other**, so a preamble-only lock can settle on the
wrong order over a garbage body -- which is exactly what the live receiver did
(`ord=b1,b0` after four retries).

What one call's dump (20021 symbols) shows, reading the ones-fraction of the
descrambled stream in 400-bit windows:

| symbols | descrambled ones | magnitude | what it is |
|---|---|---|---|
| 0 - 12800 | 27-55% | 1.0 | not MP -- no 17-ones sync at any spacing |
| ~12800 | -- | 0.04 | the peer goes silent |
| 13200 - 16800 | 100% | 1.0 | the peer's Phase 4 TRN |
| 17200 - 17600 | 78-94% | 0.85 | end of TRN |
| 18000+ | 100% | 3.3 | a loud tone: it has retrained |

So on this call the peer's Phase 4 TRN begins *after* our MP receive window has
already been open for several seconds, and it retrains without our ever seeing
an MP frame.  Relaxing the preamble search to allow two bit errors finds only
chance-level hits at random spacings in the non-TRN region, under both bit
orders -- there is no MP there to decode.

The sequencing is what to work on next, not the decoder: our own Phase 4 TRN
runs 4703 bauds before we transmit MP (the receiver needs it --
`PHASE4_TRN_READY_MIN_BAUD` was swept and every lower value costs matrix rows),
and the peer needs to see our MP before it will send MP-prime.  On the one call
where it did see it (`c29`), its log reads `MP detected, starting MP' txmit`
followed 20 ms later by `SILENCERETRAIN`.

## The old reading, and the measurements behind it

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
So the rig was given one, and it works -- and the other direction turns out to
be blocked in the peer as well.

## The rig now takes inbound calls, and it does not help

`/src/d-modem.c` on tower (backups `d-modem.c.bak-pre-inbound`,
`d-modem.bak-pre-inbound`) now has:

* `on_incoming_call`, answering with 200.  **Listen mode is keyed on an empty
  `argv[1]`**, which is exactly right: slmodemd's `socket_start()` forks
  d-modem with `m->dial_string`, which ATD fills in and ATA leaves empty.  In
  listen mode the account registers (`register_on_acc_add`) so the PBX can
  route to it, and no outbound call is placed.  Sample flow only starts when
  the media is up, so slmodemd's answer datapump stays stalled in its read
  until the call actually connects.
* `DM_TX_GAIN` and `DM_RX_GAIN`, linear gains on the DSP's output and on what
  reaches it.  `DM_RS_HEADROOM` cannot serve the second purpose -- it is
  capped at 1.0 because it is folded into the resampler kernel to stop the
  loop model clipping.  Both default to 1.0, and the outbound path is
  otherwise untouched: re-verified after the patch, the peer still dials,
  completes V.8 and reaches the same call-role recovery.

`tools/soak/v34_originate_call.sh` drives it (`ATA`, wait for the peer's
REGISTER, then dial, retrying because the PBX does not always route to a
freshly-registered 6000 -- some attempts are answered before they reach the
peer, whose log then records no INVITE at all).

**The peer answers the call and then never leaves `V8_ANS_SEND_ANSAM`.**  We
hear its `ANSam/`, send CM eleven times, and time out waiting for JM.  Its V.8
answer path does not respond to CM.  Level is not the cause and was measured
out: `DM_TX_GAIN=4` (its ANSam measured -35 dBm0 at our end, about 12 dB below
its own calling-mode V.21), `DM_RS_HEADROOM=1.0` and `DM_RX_GAIN=6` (+15.6 dB
into its DSP) each changed nothing.  Nor is it the `AT+MS` configuration: V.34
only, and V.92 by default after `AT+MS=11,1,300,33600` is rejected, behave
identically.

So both directions are dead in the same peer, each in a role its firmware
never exercises:

| Peer's role | Blocked at |
|---|---|
| SIP caller (V.8 caller, V.34 **call** modem) | `TX_PHASE2_CALL` INFO0 recovery, above |
| SIP answerer (V.8 **answerer**, V.34 answer modem) | `V8_ANS_SEND_ANSAM`; never acts on CM |

The one configuration this peer completes is the one V.90 puts it in: SIP
caller, so V.8 caller, and V.34 **answer** modem because §9.2.2 hands the
analogue modem the answer modem's timetable.  Plain V.34 cannot reproduce that
pairing -- V.34 ties the role to the call direction -- so V.34-only to data
against this peer needs a fix in its DSP, or a different peer.
