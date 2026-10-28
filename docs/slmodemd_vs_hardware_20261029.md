# slmodemd against us and against the hardware answerers, 29 October 2026

slmodemd (the SmartLink DSP in CT 100, rig build, account 6003) was used as one
fixed caller.  It dialled our server (6002 in CT 117) and the four hardware
answerers (7900 Eicon Diva, 7910 MultiTech ISI, 6311/6312 RasFinder behind
the AudioCodes FXS) with identical AT settings, so a call to us can be read
against the same client's calls to real equipment.  The same driver then
swapped the caller for our own modem (`--caller ours`) against the same four
targets.  All legs are mu-law on the PBX; 7900 and 7910 are A-law ISDN behind
it, so their legs are transcoded A<->mu by Asterisk.

Driver: `tools/soak/slm_targets.py` (run on the Proxmox host), summary
`tools/soak/slm_targets_summary.py`.  Each call grades data, not only
CONNECT: numbered lines both ways against us, the Diva echo menu, an LCP
Configure/Echo exchange with the 7910's pppd, and backspaced login-field
echo probes against the RasFinder.  Artifacts: `/root/slmx/{smoke1,probe2,
auto,v34,oc-auto,oc-v34}` on vmhost (host side) and in CT 117 (our server
logs and taps).  Build under test: `d6859491` in CT 117 (HEAD differs only
in the Apple coupler tools).

## Results

slmodemd as caller (automode = its default offer, V.90 enabled):

| Target | automode | +MS=34 |
|---|---|---|
| ours | V.90 54667 down / 31200 up, 4/4, every line both ways | **V.34 9600** (a2c) / 24000, 2/2, lines intact |
| 7900 | 0/4: V.8 fine, V.90 Phase 3 fails over the transcoded leg, its V.34 drop then fails | V.34 28800-31200, 3/3 echo |
| 7910 | V.90 52000-53333 on 2/6; V.8 fails on 3/6 (no ANSam; no JM; ANSam then nothing); 1 retrained to V.34 and lost data | V.34 31200, 2/2 |
| 6311 | V.34 3429 baud 33600, 4/4 | 33600, 2/2 |
| 6312 | V.34 3429 baud 33600, 4/4 | 33600, 2/2 |

Our modem as caller (V.90 analogue role toward 7900/7910, default
digital-role offer toward the RasFinder):

| Target | automode | +MS=V34,0 |
|---|---|---|
| 7900 | 0/3: Phase 3 completes, TRN2d never demaps over the transcode, retrain loop | V.34 16800 / 21600, 2/2 echo |
| 7910 | V.34 19200-24000, 3/3 (card offers no PCM to our V.8) | V.34 21600-24000, 2/2 |
| 6311 | V.90 49333-50666; 1/3 clean, 1 lost to a renegotiation retrain, 1 peer retrain -> NO CARRIER | V.34 24000, 2/2 |
| 6312 | V.90 50666; 2/3 clean, 1 lost to a renegotiation retrain | V.34 24000, 2/2 |

The 7910's pppd requires PAP and hangs up at 37 s ("Authentication failed");
the last LCP echo and one truncated frame are lost to that hangup on every
call, so 55/56 with one bad FCS is a pass.

## What this says about us

1. **Our V.34 answerer transmits a signal slmodemd will only take at 9600.**
   slmodemd's own equaliser error on our Phase 3 TRN is 38-47 (the Diva's is
   48), but from the start of our Phase 4 S/TRN it jumps to ~18600 and sits
   at ~4000 (preerr ~1000), where the Diva's recovers to ~50 within 300 ms.
   Its rate table maps 4000 to data rate 4, i.e. 9600 answer-to-call; the
   other direction trains 24000.  Checked on our transmit tap: Phase 3 and
   Phase 4 TRN have the same level (-5.0 dBm0 on this scale, no clipping) and
   the same spectrum within 1 dB, and there is no leakage of slmodemd's own
   signal into our transmit (|corr| < 0.05).  We decoded slmodemd's J as
   4-point and sent 4-point TRN, which also matches what it got from the
   RasFinder at 33600.  Still open: what in our Phase 4 stream its equaliser
   cannot follow (S/S-bar timing, the 1.7 s silence before Phase 4, or a
   transmit timing/phase step at the restart).  V.90 calls are unaffected:
   our V.90 downstream is PCM.

   **Resolved the same day: we started Phase 4 S too early.**  §11.3.1.2.6
   lets the answer modem wait up to 500 ms after receiving J; ours started S
   the moment J decoded, ~360 ms into slmodemd's J.  slmodemd freezes its echo
   canceller a fixed ~220-300 ms into its own J ("on JTXMIT - time to freeze
   echo"), and on every call to us that line was missing: it switched to J'
   before the freeze, kept adapting the canceller through Phase 4 on our
   signal, and its equalizer floored at ~4000.  The Diva and RasFinder leave
   200-400 ms more.  With a 250 ms wait (`ANSWER_PHASE4_S_DELAY_MS`, plain
   V.34 answerer only) the freeze line returns, its Phase 4 error is 30-43,
   and slmodemd takes us at 31200 (our 3200-baud cap) instead of 9600, lines
   intact both ways.  Two hypotheses were tested and ruled out on the way:
   - our Phase 4 S is off-spec -- 0/90 degrees instead of 10.1.3.7's point 0
     (45 degrees) and its 90-degree rotation, so it sits 45 degrees from the
     TRN that follows with no PP between.  Correcting it alone left slmodemd
     at 9600 (equalizer trace identical).  It is still a spec defect, but our
     own caller's Phase 4 S detector (front end reset at J', timing
     re-acquired on 128T of S) finds the S-to-S-bar junction largely by
     chance (baud 69 or 593 of a 128T S), and the correction moves
     single-delay loopback rows; it needs that receiver fixed first;
   - a Phase 4 transmit symbol-clock reset (`phase4_wait_init()` zeroes
     `baud_phase`): it runs inside the baud callback, so it moves less than a
     sample, and the taps were identical with it removed.
   Loopback: pass rates over `V34_DUPLEX_DELAY` are unchanged within noise
   (64/80 HEAD, 61/80 with the wait, eight rows by ten delays); our own
   caller now measures the answerer's transmit at 33600 where it measured
   31200 at 40 dB.

2. **Early §9.6 renegotiation costs RasFinder calls.**  On 2/4 V.90 calls to
   the RasFinder the upstream receiver reported an abrupt discontinuity
   (0.67-0.69 from the constellation, three failed local timing searches)
   about 2 s into data, stepped the offer to 28800 and sent Rd.  The RasFinder
   never answered Rd; we retrained after 32000 symbols, the call spent ~25 s
   out of data and the login banner was lost.  The two calls with no
   discontinuity reached login cleanly.  Our RTP send gaps are <= 37 ms on
   all four calls and not near the event, so this is not the known >35 ms
   DIL-callback stall.  Against slmodemd the same renegotiation only appears
   as its hangup begins (after `+++ATH`), not in a live call.

3. **Our V.34 originate rate is capped below slmodemd's.**  We never offer
   3429 baud (INFO1c 3429=0) and start from 3200/31200; our receiver then
   trains 24000 a2c.  slmodemd trains 3429 baud 33600 on the RasFinder and
   31200 on the 7910 and Diva.  On the Diva our V.34 originate reached only
   16800/21600 against its 28800/31200.

4. **V.90 to the Diva through the A<->mu transcode fails for both clients**,
   slmodemd in Phase 3, ours in Phase 4.  This is the leg, not either modem:
   our usual Diva harness uses the law-matched account (2905, A-law) and
   passes.  slmodemd cannot be offered A-law (d-modem gives PCMA priority 0;
   its DSP is a mu-law build), so a slmodemd-vs-Diva V.90 comparison is not
   possible on this rig.

## Rig notes

- CT 117 has no `socat`; the driver relays our PTY with a python raw-tty
  bridge.  Background children of `pct exec` must have every fd redirected,
  or `pct exec` never returns.
- The 7910 answers through `multitech-ppp` (pppd `require-pap`, chat
  `S0=1`) on eicon420; its journal shows each call's `Connect:` and
  `LCP: timeout` lines.  The Diva answerer logs its own CONNECT line
  (`bri-test-answerer`), which gives the card's view of each 7900 call.
- slmodemd always sends a QC before CM (`V8_ORG_SEND_QC`); the 7910's V.8
  failures happened with it, but so did its successes.
