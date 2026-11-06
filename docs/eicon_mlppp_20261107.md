# Multilink PPP over ISDN digital and V.90 links, 6–7 November 2026

What was tested: v90modem as the **client** of RFC 1990 multilink PPP against
the Eicon rig (eicon420, Diva BRI cards behind the Cisco gateway), one
`sip_v90_modem` and one `pppd multilink` per link, run from wired CT117 with
`tools/eicon_mlppp.py`. Each link is either an **ISDN digital B channel**
(`AT+MS=CLEAR,0`, `ME_DATA_FRAMING=ppp`, `ME_CLEARMODE=1`: RFC 4040 CLEARMODE
end to end, synchronous HDLC PPP on the DS0, RFC 1662 async towards pppd) or
a **V.90 call** in the analogue role (`AT+MS=V90,0`, PCMA, LAPM). Transfers
are 12 MiB (12,582,912 bytes) of incompressible SHAKE256 data over HTTP to
`tools/eicon/http_test_server.py`, checked by length and SHA-256. Every
transfer that completed was byte-exact. Rates are receiver-side goodput.

## Routing and answerers

- Digital calls need the CLEARMODE-capable endpoint 6501. Added Asterisk
  routes **17920, 17930, 17940** beside the existing 17950 (backup
  `extensions.conf.before-mlppp-clearmode-20261106`); the 79x0 voice routes
  are untouched. Cisco and Diva configuration were not changed.
- The existing pppd bundle members (`eicon-79x0-*`, profile `AT&F14`, the
  same services the Windows 95 modems use) **answer digital calls unchanged**:
  profile 14 autodetects the B-channel protocol. CONNECT 64000, LCP, PAP and
  IPCP on the first call to each of 17920, 17930, 17950.
- `isdn-7950` is the only member whose peer file says **`persist`**. Its one
  long-lived pppd ACKs MRRU but never attaches the link to the bundle: it
  Protocol-Rejects our Multi-Link (0x3d) frames and runs IPCP alone, and in
  one run its stale `isdnppp0` bundle made the next V.90 link create a
  second server bundle. Two 5-link attempts stalled at zero bytes. After
  `systemctl restart isdn-7950` (fresh pppd) the same five links bundled
  correctly. Recommended: `nopersist`, as every other member has.

## Results

| Run | Links | Download | Upload | Notes |
|---|---|---|---|---|
| 11 | 2 x ISDN B (17920+17930) | 841 s, **119.6 kbit/s** | 906 s, 111.2 kbit/s* | 30 min call, no retrains, RTP loss 0, split 50/50, RTT 141 ms |
| 12 | 2 x B + V.90 7920 | 883 s, 114.0 kbit/s | 907 s, 111.0 kbit/s* | V.90 member lost: LAPM restart after retrain (fixed below) |
| 13 | 2 x B + V.90 7920, fixed | 861 s, 116.9 kbit/s | not run | 176 kbit/s with all 3; V.90 retrain failed in Phase 2, 100 s stall |
| 15c | 3 x B + V.90 7920 + V.90 7930 | 635 s, **158.5 kbit/s** | 577 s, **174.5 kbit/s** | peak 249 kbit/s (60 s); both V.90 members resumed one retrain, lost the second |
| 14 | 3 x V.90 (7920, 7920, 7930) | aborted at 6.85 MB | not run | 152 kbit/s for 240 s (the user's 3-modem figure); two members lost to failed retrains by 360 s, 240 s at zero, then one link |

\* The first harness relay dropped writes that would block (fixed before
run 15c): the upload pushed 165.8 kbit/s of IP into a 128 kbit/s pair and
lost the excess to TCP retransmission. 15c's upload (3 B, V.90 members
already gone) is the clean figure: 58 kbit/s goodput per B channel, 91% of
64 kbit/s; download over 2 B is 93%.

Single-link digital modes (CLEARMODE, `ME_JB_MS=40`, PPP then 10 pings):

| Mode | Number | Result |
|---|---|---|
| CLEAR + sync PPP | 17920, 17930, 17950 | CONNECT 64000, PAP, IPCP, 5/5 pings, RTT 141–162 ms |
| V.120 (`ME_V120_ACK=1`) + async PPP | 17920 | CONNECT 64000, PAP, IPCP, 10/10 pings, RTT 155 ms |
| V.110 38400 | 17920 | no CONNECT: Diva sends no V.110 frames (rx 0), §7.1.2.4 S/X guard expires. BC `88 90` carries no rate adaption, so profile 14 picks HDLC; the 8 Oct passes used a profile-3 answerer |
| CLEAR + sync PPP | 17940 | NO DIAL in 8 s, the 6 Nov Q.931 timer-expiry fault, unchanged |

Line efficiency is not the limit: a decode of run 11's DS0 taps shows sync
HDLC frames occupying **99.7%** of the line in both directions (ours
upstream, the Diva's downstream), no idle flags, ~757-octet MP fragments.

## Defects found and fixed

1. **Digital PPP dropped every frame over 400 octets** (`644d170d`).
   SpanDSP's `HDLC_MAXFRAME_LEN` was 400 and `hdlc_rx/tx_set_max_frame_len()`
   clamp to it, while CC_PPP asks for 1600. LCP, PAP, pings and HTTP headers
   all fit, so every earlier digital PPP check passed; the first bulk
   transfer stalled on its first full-size TCP segment (server sent 27,858
   bytes, we got 765). Ceiling now 2048 with `HDLC_DEFAULT_MAXFRAME_LEN` 400
   so other users (T.30, V.42) are unchanged. `clear_channel_test` carries a
   1504-octet frame both ways.
2. **V.90 analogue-role retrain discarded LAPM** (`9f0fc48d`). On the Diva's
   Tone B, `restart_v90_analogue_phase2_locked()` released the data stack
   and DATA entry redid V.42 detection, XID and SABME. After a clean 11 s
   retrain back to 56000/31200 the Diva never answered the 20 XIDs, and V.42
   declared link failure 24 s later. Now matches the digital role (V.90
   9.5.2, V.34 11.5): DTE clamped, link kept. Live: 4/4 successful retrains
   resumed the existing link. fast.list has the forced-retrain pair.
3. **Harness relays lost data** under backpressure (`tools/eicon_mlppp.py`,
   `tools/eicon_ppp_direct.py`), now lossless.

## Open issues

1. **The Diva retrains our V.90 upstream every few minutes.** First Tone B at
   192–272 s into data on 8/8 V.90 members that lasted that long, the next
   ~150 s after resuming. No RTP send gap over 35 ms on those calls (13,206
   packets checked on one). The user's analogue modems hold hour-long
   sessions on the same answerers, so this is ours until shown otherwise.
   Next: A/B the upstream ceiling (28800/26400) and look for slow timing or
   level drift in the TX tap over the first 200 s.
2. **Retrain fails when we miss the Tone A reversal.** In all 8 retrains:
   reversal seen 160–180 ms after FIRST_NOT_A gives DATA in 10.6–10.8 s (5/5);
   the 2.2 s fallback gives no INFO1d decode and no recovery (3/3).
3. **A failed retrain has no deadline**, and an MLPPP bundle stalls while any
   member is dead: 10 s per successful retrain, 100–140 s per failed one
   (both pppds' 10 s x 6 LCP echo). Dropping the call when a retrain
   overruns, or shorter LCP echo on V.90 members, would cut that.
4. Latency on a digital B channel: RTT 141–162 ms with `ME_JB_MS=40`,
   315 ms at the 200 ms default.
5. The clear-channel summary line reports `rx 0 frames` in PPP mode
   (cosmetic; `ppp_rx_frames` is not printed).

Provenance: CT117 `/root/mlrun/{01,02,11,12,13,14,15,15b,15c}*` (disposable).
