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

## Data mode and V.42 LAPM reached (2026-08-22)

A plain V.34 call to this rig now completes Phase 4, enters data mode, brings
up V.42 LAPM, and carries error-free payload: 76 numbered lines written to our
PTY arrived at the peer's DTE contiguous and byte-exact (`artifacts/
v34-lapm-20260822T-c50`).  Four things stood in the way, and the last two were
not in the modem at all.

**1. The peer's MP was decoding all along; the CRC check read it backwards.**
`tools/v34_mp_offline.py` computed the frame CRC MSB first, as 10.1.2.3.2's
generator runs, and then read the received CRC *field* LSB first.  Those two
readings are the bit reflection of one another, so a perfect frame fails.  Read
the same way round, the c31 dump gives 101 Type 1 frames exactly 188 bits
apart, 99 of the 100 complete ones bit-identical, and 100 CRC-valid.  Nothing
was ever wrong with what arrived.

**2. The MP dibit transform is not searchable.**  10.1.3.9 generates the
4-point MP as in 10.1.3.3, which advances the point index, and
`training_constellation_4` is ordered so an increasing index rotates
*clockwise* while the receiver measures the increment counter-clockwise: the
recovered dibit is always the negation of the transmitted one
(`MP_HYPOTHESIS_DIFF_INVERSE`), fixed by the encoder and the table rather than
by the channel.  The V.90 CP decode already pinned exactly this; plain V.34 MP
now does too.  Left to the search it settled on hypothesis 18 with the bit
order swapped -- neither a TRN of scrambled ones nor MP's own 17-ones sync can
tell one bit order from the other, so a wrong lock survives the preamble and
dies at every CRC.  Suppressing the retry rotation as well was tried and is
*not* in: with the hypothesis pinned it is redundant, and together the two cost
the 2400 matrix rows 103 and 418 bit errors.

**3. THE BLOCKER: the receiver refused any first MP that was not Type 0.**  The
gate was tuned against this tree's own transmitter, which sends Type 0.  The
type bit says whether the frame carries 11.4.1.2's precoder coefficients, and a
modem that precodes sends MP1 from its first frame -- every one of the 87
CRC-valid frames in the c37 dump is Type 1, present in the same dibits the live
receiver was fed while it locked nothing and timed out at 20000 bauds.  Nothing
in 10.1.3.9 or 11.4.1.2 orders the types; the CRC is what rejects a false lock.
With the gate gone, MP1 is accepted, MP' goes out, E is detected, B1 correlates
at 0.996 and data mode starts.  The loopback matrix *improves*: ten of twelve
rows recover payload with zero errors, which is every rate that trains at all,
in both laws.

**4. We asked for a rate we cannot receive.**  Our MP advertised 21600 because
that is the configured start profile; nothing measured the channel.  The peer's
receiver did measure it, read `equerr 5610`, and chose 7200 for its own receive
direction.  At 21600 -- 2400 baud, expanded shaping, an 896-point
constellation -- the symbols entering the mapper are **white**: mean squared
distance to 9.x's odd-integer grid is 0.67, the figure for symbols with no
relation to the lattice at all, and *no* scale or rotation improves it (swept
over `V34_DATA_FRAME_DUMP`, the best of every gain from 0.4 to 2.5 and every
angle is 0.657).  The same call path at 4800 reads 0.25 and decodes.  So the
receiver is good for a small constellation and not a dense one, and
`ME_V34_BPS=4800` is what the payload run used.  **Rate selection driven by a
measured receive SNR, as the peer does, is the open work here.**

**5. V.42 "unsupported peer" was correct.**  `tools/soak/v34_lapm_call.sh` sent
`AT\N0`, which disables error control, so the peer never ran V.42's detection
phase.  `DS_RX_BIT_DUMP` settled it in one read: its data-mode output was 87%
ones with a 17249-bit run -- idle mark.  `NPARM` now selects the mode (the peer
rejects `\N3` with ERROR and does LAPM anyway), and `KEEP=1` holds the call
after LAPM instead of breaking out of the wait loop, since killing the dial
closes the peer's serial and drops DTR before a payload test can run.

### Instruments

Plain V.34 data mode had no diagnostics at all; it has two now, and they answer
different questions.  `Rx - DATA: distance to grid` says whether the waveform
arrives decodable.  `Rx - DATA: shell index over k bits` is 9.6.3.3's r0 bound,
which owes nothing to the content, so it separates wrong grouping from wrong
symbols.  Read together: grid small + shell 0% is a working data mode, grid
small + shell high is correct symbols grouped wrongly, grid large is a fault
before the mapper.  `DS_RX_BIT_DUMP` and `ME_DATA_HOLD=1` cover the V.42 layer.

### Still open

* Only 4800 bps is proven.  21600 does not decode (item 4).
* The handshake is intermittent: two of five calls reached data mode, the rest
  died in the Phase 2 INFO0 recovery livelock already documented above.
* 76 of 300 lines were delivered before the hold expired; throughput and
  sustained stability are unmeasured.

## The RasFinder, dialling out as the V.34 call modem (2026-09-28)

A second peer, and the first to exercise the call-modem Phase 2 path against
something other than the SmartLink DSP.  `ATD8416` from a second server
instance; `tools/soak/rasfinder_call.sh` places one call with taps and a log,
so an A/B is the same command twice.  **Leave 60-90 s between calls** -- a
close redial comes back `V8 result: status=Call negotiation failed`.

**The peer answered nothing at all, for two reasons, and both were ours.**

**1. 11.2.1.1.7's INFO1c was sent off the end of our own L2.**  The clause
sends it "after the call modem detects Tone A and has received the local echo
of L2", and that ordering is not a formality: 11.2.1.2.8 has the answer modem
receive this L1/L2 *first* and raise Tone A only then.  Measured on the two
taps, our L2 ended at 11.905 s and the RasFinder's Tone A began at 12.34 s, so
INFO1c went out 435 ms before the peer was conditioned to receive it -- after
which it held Tone A for the remaining 47 s of the call
(`artifacts/rf-v34-a1`).  New `V34_TX_STAGE_POST_L2_WAIT_TONE_A` holds silence
until Tone A appears, bounded by `ME_V34_POST_L2_TONE_A_WAIT_MS` and falling
through to the old behaviour on expiry.  **The evidence has to be the 2400 Hz
bin fraction, not an event flag**: by this point the receiver has consumed the
11.2.1.2.6 Tone A and advanced to `V34_RX_STAGE_INFO1A`, so its Tone A
detector is no longer running at all; `l1_l2_signal_init()` invalidates the
measurement so a pre-probe reading cannot satisfy the wait.  This is the exact
mirror of `V34_TX_STAGE_POST_L2_WAIT_TONE_B`, which the answer modem already
needed for the same reason.

**2. INFO1c declared every symbol rate but one unusable, and the default was
a leftover.**  10.1.2.3.4's rows were gated on the configured start symbol
rate, and `g_v34_start_baud` was 2400 because the receive-band notch at our
own transmit carrier needed 150 Hz of carrier separation and 3200 gives 91 --
a constraint that stopped existing on 2026-08-22 (see
`docs/v34_data_mode_rates.md`), when the notch was measured to be the thing
capping the live rate and `v34_update_echo_policy()` learned to drop it
whenever the transmit carrier falls inside the receive band, which at 3200
baud it does.  So a default call offered the answer modem one row and five
zeros, and this peer answers that with nothing (`artifacts/rf-v34-b1`: a
CRC-clean INFO1c, then 45 s of Tone A).  Default is now 3200 -- not 3429,
which is the one row the symbol-rate matrix has never trained.  With it the
same peer returns INFO1a and the call runs S, S-bar, PP, TRN, J, Phase 4 and
**data mode at 31200 bit/s** (`artifacts/rf-v34-d2`).
`ME_V34_INFO1C_ALL_RATES=1` offers every row independently of the profile,
which is the more faithful reading of 10.1.2.3.4, and is default off because
it lets the answerer select 3429 and costs the 2743/9600 u-law duplex row its
training.

**3. The Phase 4 TRN ones-lock can be an artefact, and it is anti-correlated
with success.**  10.1.3.8's TRN is scrambled ones, so its dibits are
pseudo-random and all four must appear.  A receiver whose carrier loop has not
acquired sees the SAME dibit every symbol, and one of the 24 hypotheses turns
that constant into a constant -- a near-perfect "ones" score on a signal that
is not TRN at all.  The two outcomes are separated by the histogram and not by
the score: the call that reached data mode read **57% ones over dibits
997/1217/1037/1212**, and the call that never validated an MP frame read
**88% over 442/3440/437/144** (`artifacts/rf-v34-d2` and `-c3`).  The failing
one is 77% on one dibit, a residual carrier of 571 Hz -- which is 2400 minus
the 3200-baud low carrier to three figures.  That score gates
`PHASE4_TRN_READY_MIN_SCORE`, so a diverged receiver walked into the MP search.
The histogram is now on the `Phase 4 TRN: best` line and the MP gate requires
no single dibit above half the window (`ME_V34_TRN_DIBIT_SPREAD=0` restores
the old gate and keeps the log).

**4. A recovery that re-asks for the rate that just failed can only reproduce
it.**  Live at 31200 the data mode measured 0.619 from the grid where 2/3 is
white; the 11.6 renegotiation rebuilt Phase 4 from the same MP offer, was
given 31200 again and went white again, inside 4.5 s -- while the peer's own
MP asked for 21600 in its own direction on the same line.
`v34_rx_rate_backoff_locked()` now steps our receive direction down two N per
attempt, from what the last MP exchange settled rather than from the start
profile, leaving the other direction as negotiated.  Two N because the lattice
spacing is always 2, so at fixed SNR the distance to the grid scales with the
square root of the constellation power and one 2400 bit/s step at 3200 baud is
0.75 of a bit per symbol; four attempts therefore walk 31200 -> 26400 -> 21600
-> 16800 -> 12000.  `ME_V34_RX_RATE_BACKOFF=0`,
`ME_V34_RX_RATE_BACKOFF_STEP`.

### The one clause-correct change that is NOT on by default

**11.2.1.1.6's second Tone B reversal is conditional on the Tone A reversal**,
and the code uses a flat 100 bauds of Tone B instead ("V.34: fixed timing"),
167 ms that owes nothing to the far end.  Live that fires mid-probe: the
RasFinder's post-L2 Tone A and reversal land at 10.15-10.25 s and we reversed
at 10.148 s and put L1/L2 in front of a modem that had not reversed.
**Alternated live, one variable: off 0 of 3 calls received INFO1a
(`rf-v34-h1`, `-j1`, `-j2`); on 2 of 2 received it and reached Phase 4
(`rf-v34-k1`, `-k2`).**

It is off because of our own receiver.  A conformant exchange leaves SECOND_B
at 121 bauds -- our own answerer's 11.2.1.2.6 reversal arrives there, since it
waits for this very Tone B plus 50 ms -- and swept as a pure timer the
2800/21600 u-law duplex row passes at 100, 104, 108, 112 and 116 bauds and
fails at 120 and 130.  Turning it on breaks that row and 3200/21600 A-law.
**The failure is not in Phase 2**: both arms make the identical T/2 eye-phase
decisions, and what differs is one direction's data mode, 0.668 from the grid
against 0.087.  That is a 21600 acquisition coin flip sensitive to a 30 ms
shift in when Phase 2 ends -- a pre-existing fragility this exposes rather
than causes.  Fix that and flip the default; pinning row after row to the old
timing would be editing the suite to pass.
`ME_V34_SECOND_B_WAIT_REVERSAL=1`.

The first acquisition repair is now on that A/B arm: 10.1.3.1's known B1
symbols supervise the equalizer instead of supplying only scalar phase/gain,
and the B1 carrier phase is advanced from the correlation window's centre to
the B1/DATA seam.  That makes 3200/21600 A-law clean (45 errors -> zero) and
turns 2800/21600 u-law from white/no lock into decoded payload.  The latter is
not yet clean: transmitter/receiver symbol dumps align exactly except for **one
2D symbol at DATA symbol 5839**, whose imaginary coordinate lands about 16
Q9.7 units over the decision boundary and expands through the mapper into one
35-bit burst.  The six shell-index failures are in the opposite direction and
are not its cause.  A 1.03 gain bias fixes that row but damages 3200 A-law and
3429 u-law; B1 minimax gain selection, a two-tap decision-feedback experiment,
and +/-12 pulse-shaper-phase nudges do not remove it.  Therefore the default
remains off: the remaining work is genuine dense-constellation margin for one
symbol, not another Phase-2 timing condition or a row-specific pin.

### Still open against this peer

* **Phase 2 is intermittent** and the item above is the measured reason.
* **Phase 4 MP is intermittent** even when Phase 2 completes: `rf-v34-d2`
  decoded a CRC-valid MP1 and reached data mode; `-c3`, `-f2` and `-g1`
  produced CRC failures with the sync and start bits right and the body wrong,
  i.e. noisy bits rather than a wrong interpretation.
* **31200 is white on this line** and the first rate we ask for is still the
  start profile's maximum.  Choosing it from a measured receive SNR remains
  the open item in `docs/v34_data_mode_rates.md`; the back-off above is a
  fallback, not a selection.
* ~~A retrain taken from Phase 4 deadlocks in `FIRST_B_SILENCE`.~~  **Fixed.**
  `v34_start_retrain()` did not reset the Phase 2 reversal and L1/L2
  transaction counters -- the V.90 retrain response has always called
  `v90_phase2_reset_transactions()`, plain V.34's never did -- so
  `phase2_reversal_count` was still 3 from the startup Phase 2, every further
  reversal was published as `V34_EVENT_REVERSAL_3`, and `FIRST_B_SILENCE`,
  which waits for 11.2.1.1.4's reversal as `V34_EVENT_REVERSAL_1`, could never
  fire.  The restarted Phase 2 walked V90_RETRAIN_SILENCE ->
  V90_PHASE2_B_INFO0_SEEN -> FIRST_NOT_B_WAIT -> FIRST_NOT_B ->
  FIRST_B_SILENCE and stayed there for the remaining 40 s of the call while
  the receiver went on to reach INFO1A on its own (`rf-v34-g1`, `-k1`, `-k2`,
  `-q1`, `-q3`).  That made the rate back-off useless in practice: q1 and q3
  both reached V.34 data mode at **19200 bit/s**, both correctly asked for
  14400 on the way out of a white receiver, and neither came back.  11.5.2.1
  sends both modems back to 11.2.1's tone ranging, so the ordinals start over;
  `FIRST_B_SILENCE` now also reads the durable counter, as the V.90 branch
  four lines above it has since 2026-07.
* **`ME_DATA_FRAMING=lapm` tears the call down** ~4.5 s into a white data
  mode, because V.42 detection concludes "unsupported peer" over bits that are
  not being decoded.  The default V.14 framing does not, which is why the
  back-off experiments run without it.
