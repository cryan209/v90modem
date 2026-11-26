# v90ras: 10-line v90modem dial-in server with multilink PPP

v90modem as the **server** of RFC 1990 multilink PPP, the reverse of the
eicon420 setup in `docs/eicon_mlppp_20261107.md` (there v90modem is the
client and eicon420's Diva/MultiTech ttys answer). Here one
`sip_v90_modem --lines 10 --recycle` answers, and each line's PTY has its own
`pppd multilink`, configured like the eicon420 members, so every link of one
caller joins one bundle.

Installed 2026-11-07 on CT 117 (`v90modem-dev`, 192.168.88.230, wired;
`ssh -o HostKeyAlias=vmhost root@vmhost.net.cryan.nz pct exec 117 -- ...`),
binary built from 75caa292.

## Numbers (asterisk.net.cryan.nz)

| Number | What |
|---|---|
| 2950 | pilot: hunts 2951 upward for the first idle registered line (busy or unregistered lines are skipped), `Busy` when all ten are in use |
| 2951-2960 | line 0-9 directly (SIP accounts, `endpoint-modem-dual`: clearmode, u-law, A-law in the caller's order with `operation:intersect`; `aor-single-reg`) |

Dialplan is in `[default]` after the 17940 route; backups
`/etc/asterisk/{pjsip,extensions}.conf.before-20261107-v90ras`. Since
2026-11-20 ISDN callers are answered in A-law (see "ISDN callers in A-law"
below).

## Files on CT 117

| Path | What |
|---|---|
| `/opt/v90ras/sip_v90_modem` | frozen copy of the binary (rebuilding the worktree does not swap it under live workers) |
| `/root/v90ras` | detached worktree it was built from |
| `/etc/systemd/system/v90ras.service` | the 10-line server (copy here); SIP ports 5300-5309, RTP 30000+100*i, PTYs `/run/v90ras/line.0`-`.9`, `ME_JB_MS=40`, per-call G.711 taps in `/var/tmp/v90ras/calls/line<L>-<pid>` |
| `/etc/systemd/system/v90ras-ppp@.service` | one pppd per line, instances 0-9 (copy here) |
| `/etc/ppp/v90ras-options` | shared pppd options (copy here) |
| vmhost `/etc/udev/rules.d/99-ppp-lxc.rules` | makes `/dev/ppp` mode 0666: the CT is unprivileged and maps the host's root-owned node to `nobody` |
| `/etc/v90ras/profile.0`-`.9` | power-on AT profile, copies of `v90ras-profile` (plain V.34, 21600 cap) |
| `/etc/v90ras/sip.env` | SIP account passwords (0600, not in git) |
| `/etc/ppp/pap-secrets` | an unused `v90dial` line from before `noauth` (backup `pap-secrets.before-v90ras-20261107`) |

Bundle: server 10.254.200.1, client 10.254.200.2, interface `v90ras0`,
`ms-dns 192.168.88.1`. Since 2026-11-08 16:10 the CT forwards and NATs the
client to the LAN and the internet: `/etc/sysctl.d/90-v90ras-forward.conf`
(`net.ipv4.ip_forward = 1`) and table `ip v90ras_nat` in `/etc/nftables.conf`
(masquerade 10.254.200.0/24 out eth0, with a counter; the stock file is
`nftables.conf.before-v90ras-nat-20261108`). The dial-in is still `noauth`,
so anyone who dials 2950 gets that access. Before this a client reached only
10.254.200.1 (`tools/eicon/http_test_server.py --bind 10.254.200.1` serves
transfers there).

Operate:

```bash
systemctl restart v90ras            # members are PartOf it and restart too
journalctl -u v90ras -f             # [Li] prefixes per line
journalctl -u 'v90ras-ppp@*' -f
```

## How a call runs

The worker auto-answers (S0=2) and prints `CONNECT`; the member's `chat`
waits for it, pppd negotiates (`passive`, `noauth`, `multilink`,
`mrru 1500`, same name/addresses/ifname on every member, as eicon420), and
the first member up creates `v90ras0`; later ones log `Link attached to
v90ras0`. At hangup `--recycle` ends the worker, the PTY closes, pppd sees
`Modem hangup` and exits, and systemd starts a fresh pppd that waits for the
new PTY. All members run at 115200 because every link is a modem of about
the same rate (see eicon_mlppp's note on the split if B channels are mixed in).

**Hazard:** the members run as root, so a `/root/.ppprc` (`user`,
`remotename`) is read by them too. One left there during a client test made
every restarted member demand the client's credentials and exit. Never leave
one on CT 117; pass client options on the command line.

## Results, 2026-11-07 (our client on the same CT, so no data-rate figures)

| Run | Calls | Result |
|---|---|---|
| 01 | 1 x V.90 analogue (our client, 2903) to 2950 | line 0 answered; V.90 retrained 7 times in Phase 3/4 and fell back to **V.22bis 2400** (fixed below, runs 06-07); PAP and IPCP then fine |
| 02 | 2 x V.34 (2903, 2902) to 2950, client pppd multilink | hunt put call 2 on line 1; both **33600**, RTP loss 0; server `New bundle v90ras0 created`, then `Link attached to v90ras0` |

The client and server share CT 117, so 10.254.200.1 is a local address to
the client and a ping or HTTP test over the bundle never crosses the modems
(run 01's 0.03 ms RTT). Throughput needs a client on another host: the
Windows 95 MLPPP machine is the intended one.

Run 01 failure, found and fixed the same day: the server's WAIT_JA energy
gate took §9.3.2.1's 70 ms silence before S for a TRN-to-Ja gap. It rejected
S as "Tone A" but stayed in its gap state, so PP/TRN counted as Ja, and the
500 ms Ja fallback (`ME_V90_JA_HEURISTIC_FALLBACK_MS`) started at the start
of TRN. Our client's TRN is 640 ms, so Sd went out before Ja began. The client
saw Sd 20 ms into Ja and stopped it (§9.3.2.4), no descriptor parsed, no DIL
was sent (J'd went straight to Ri), and Phase 4 retrained 7 times before the
V.22bis fallback. Replaying the client's TX tap through the server engine
reproduced it, and so did the offline pair at the production default. Every
fast.list V.90 pair row had set the fallback to 0, which hid it. Engine fix
(91f52721): a rejected energy return puts the gate back to "in signal".
Offline, the pair then parses N=120 600 ms into Ja and sends the DIL, and
fast.list has the default-fallback row.

**That alone did not fix the live call.** Live, S read band2400 below the
0.35 Tone A threshold (0.456 in the replay), so it was never rejected: the
gate took S itself as Ja and the clock started at S (9,459 ms), 500 ms
before Sd (9,980 ms), with Ja due about 780 ms after S. The fallback is a
deliberate rescue for peers whose Ja never decodes (SmartLink in August,
Apple r7), and no anchor available to the gate marks Ja onset: v34rx enters
WAIT_S mid-TRN, and no hypothesis shows TRN as ones. So **v90ras runs
`ME_V90_JA_HEURISTIC_FALLBACK_MS=0`** (in `v90ras.service`): Sd only on a
CRC-valid descriptor, as §9.3.1.3 says. In the recorded corpus every
hardware peer (MT5634, MT9234, SmartLink, Intel serial) parsed without the
fallback. A peer whose Ja does not decode now waits for the v90.c interop
fallback (6 s) and then concedes to V.34 after 20 s.

| Run | Server | Result |
|---|---|---|
| 03, 04 | 91f52721, fallback 500 | V.22bis 2400: Sd 20 ms before Ja, as before |
| 06, 07 | 91f52721, fallback 0 | **V.90 56000 down / 31200 up**, descriptor N=120 at 10.55 s, DIL sent, PPP up in 35 s (93 s before) |

**Fallback 0 was reverted the same afternoon (back to the default 500).** The
first real clients, modems on the Cisco 2911 FXS ports (callers 7800, 7801,
7802 via endpoint 6501, u-law end to end: 3.8% of received codewords lie
outside the A-law image, against 0% if transcoded), never had their Ja
decoded. In one call the modem ran S/PP/TRN/Ja for 3.65 s after the
§9.3.2.1 silence and gave up, so without the fallback it never got Sd. All
four calls then fell to V.34, where our receiver decoded white for the rest of
the call (distance to grid 0.64-0.68, SNR 15-28 dB, at 3200 and at 2400 baud),
so LAPM never came up and pppd's chat never saw CONNECT. Our own client
trains V.34 33600 against the same server. Both failures are open. The
server now keeps each call's G.711 taps (`VPCM_G711_TAP_DIR=.../line%L-%P`,
`%P`, the worker PID, expanded by the line supervisor since 7bc08170 so the
RTP CSVs land there too) so they can be replayed.

| Call (17:17-17:23) | Result |
|---|---|
| 7800 x2, 7802 | V.90 Phase 3, no J/Ja decoded, peer retrain, V.34 white, hung up after 50-55 s |
| 7801 | V.90 retrain at tx_phase 3, V.34 3200 high carrier, white for ~80 s, hung up |

Second round, 17:32-17:34, both on line 0 with full taps (replayed with
`v90_engine_replay ... ulaw --sync`, `VPCM_ME_VERBOSE=1`):

| Call | Result |
|---|---|
| 7801 | V.90 Phase 3 no Ja, peer retrain, V.34 3200 **DATA at 31200**; 40 ms later **our** B1 rate check read 17.0 dB ("carries about 14400") and retrained per 11.5; the retrain resumed DATA (live distance to grid 0.03, 28 dB), **LAPM and PPP came up** (LCP with mrru 1500), and PAP failed: the client sent user `swine`, the server's account is `v90dial` |
| 7800 | same path; B1 read 23.6 dB at 3200, asked 19200 and capped our TX too; the retrain came back at **2400 baud / 19200**, which decoded white (0.65-0.67, 24 dB) until the peer retrained and hung up |

So the modem link does work against these clients, but the B1 check fires on
both calls and costs a retrain. Its 3200-baud reading is low (17 dB on a line
that then measured 28 dB in data mode) and does not depend on
`ME_V34_DENSE_3200`: the same reading with it at 0 and 1. A cap of 19200
carried across a fall to 2400 baud is a rate that baud cannot carry. Both
are still open.

Fixed in b13a2b19 (deployed 17:47): the MP rate policy is now a ceiling on
the rates measured at the new baud, not a replacement, and the B1 check
obeys only the per-call cap of 4, not the 20 s loss dwell. Replayed, 7800's
second B1 (21.4 dB at 2400) retrains to 12000. A 17:42 redial from 7801
that ran 50 s white at 19200 (0.64-0.67, 19 dB) gets a second B1 retrain
to 14400 instead. Still open: our first MP asks 31200 on every one of these
calls, and B1 then reads 17-23.6 dB, so each call spends one or two 11.5
retrains (~8 s each) finding the rate. The first rate should come from a
measurement (docs/v34_data_mode_rates.md).

Live on b13a2b19, 17:49 and 17:51 (7802): B1 walked 28800 (27.6 dB) ->
21600 at 2400 baud (30.1 dB) -> 16800 (17.3 dB) -> 12000 through three 11.5
retrains of ~10 s each, and the modem hung up 55 s after answering both
times, just as 12000 came up. So the modem's connect timeout (about
55 s) is the budget. A B1 reading is a correlation, SNR = c^2/(1-c^2), so
0.990 -> 17 dB and 0.997 -> 22 dB, and it moves several dB between
trainings. From 4e3dfcf1 v90ras runs `ME_V34_B1_RENEG=1`, which asks for B1's
rate by a 11.6 renegotiation (~2 s, same baud) and falls back to the retrain
after 3.15 s. This is a live A/B: these modems have not yet been seen to
answer 11.6.

18:08, 7802 + 7801 (multilink), on 4e3dfcf1: **PPP and IPCP came up**.
The client got 10.254.200.2 with DNS 192.168.88.1 on bundle `v90ras0` and
stayed up 35 s. It then dropped after a retrain in data mode. The modem
did answer our 11.6: its S arrived 330 ms after ours. But our receiver
left PHASE4_S 40 ms in, because the "no alternation in 32 bauds" handoff
took the peer's still-running DATA for TRN, so 11.6.2.1 timed out into a
full retrain. Fixed in 6dec6dca: an initiator waits for S and keeps the
data-mode front end. Replayed, it decodes the peer's MP0 (CRC valid,
19200/19200) and E, and the first data window reads 0.24 from the grid
before the recording diverges.

19:07, 7802, on 6dec6dca: one slow retrain, then clean data (0.04-0.07,
30 dB, estimate 21600) and PPP up. About 9 s later the **modem** opened an
11.6 renegotiation; our answer ended in a full retrain, the data after it was
white, and IPCP never completed. The replay of this call diverges at the
first B1 (live B1 correlated 0.144), so the responder failure is not
reproduced offline yet.

**Since 19:20 v90ras offers plain V.34 capped at 21600** (`v90ras-profile`,
installed as `/etc/v90ras/profile.<line>`, `--profile /etc/v90ras/profile`).
V.90 never trained with these modems, and every rate change mid-call
(the fallback's 31200 first MP, B1's step-down, the modem's own 11.6) is
what lost calls. Calls that stayed at one moderate rate (14400, ~21600)
carried LCP/PAP and PPP. The offline pair at this setting connects 21600
in 13 s. Remove the `--profile` argument to offer V.90 again.

19:15, 7802, plain V.34: DATA 19 s after the call arrived (30+ s via V.90),
but at **3429 baud**: we (the answerer) chose 3429 both ways from our own
probe, although it rated 3429 worst (ripple 0.90 against 0.29 at 3200).
B1 read 27.6 dB, yet our receiver started 0.43 from the grid and was white
2 s later; the loss retrain followed and the modem hung up. This
reproduces offline (0.4289). Every clean decode against these modems was
at 3200. **Since 19:25 v90ras runs `ME_V34_BAUD=3200`**; the offline pair
connects 21600 at it. Our 3429 receive against them is a defect of its own.

**19:22, first working call, on bb042443.** 7802 on line 0 reached V.34 DATA
20 s after the call arrived, with PPP/IPCP up at 30 s (10.254.200.2). The
modem's own mid-call 11.6 was answered cleanly (data 0.020 from the grid,
31.6 dB, before and after). Ping over the link: **10/10, RTT ~418 ms**.
7801 then joined on line 1 ("Link attached to v90ras0", multilink with two
modems). About 30 s later line 1's decode drifted (0.04 -> 0.065 -> 0.53)
to white, and the loss retrain recovered it to 0.009. But from then on the
client answered no LCP echo on **either** link, although line 0 decoded
clean throughout and line 1 still carried the closing TermReq/TermAck.
Ping went 2/20 at ~4 s RTT, then 0/20. Both members dropped on "No
response to 6 echo-requests" at 19:25:53-58. Likely cause, not verified on
the client: multilink fragments lost across line 1's retrain wedged the
Win95 reassembly. Open: line 1's drift to white at 3200, and whether our
V.42 delivers everything across a retrain.

**Since 2ff37ab6, a member that loses data hangs up instead of retraining**
(`ME_V34_LOSS_HANGUP_AFTER_MS=10000` in the unit). Past 10 s in data mode, a
receiver that stops decoding, or the peer's 11.5 retrain tone, ends that
call. The client then drops only that link, and the bundle carries on with
the others (redial to add it back). The modem's own 11.6 renegotiations are
still answered, and line 1 at 19:34 rode one cleanly at 21600 (0.057 from
the grid, 31.7 dB). The B1 rate check in the first 10 s is unchanged.
Still open: after some peer 11.6 renegotiations our receiver ends white
(e.g. line 1 19:33:35, 0.66), which is what triggers these losses.

**20:00 and 20:01, 7800 alone, on 2ff37ab6.**
- **19:59 call, stuck in Phase 2.** It sat 48 s and the modem hung up.
  - The modem sent INFO0c, then 1.4 s of silence, then Tone B at 8.1 s (full
    level). Our answer path had reversed Tone A at 6.48 s, 40 ms after
    INFO0c, before the modem was listening. Each side then waited for the
    other's reversal.
  - 11.2.1.2.3 requires Tone B *detected* first. Fixed in 2189b0fb: wait for
    Tone B, falling back to reversing after 2.5 s. The replay now reverses at
    8.20 s.
- **20:01 call, reached data, then hung up.**
  - It entered data white (0.65 from the grid); our 11.6 fixed it (21600,
    0.015, 30.8 dB) and PPP came up.
  - The modem's own 11.6 then measured 8.3 dB, it sent the 11.5 retrain
    tone, and 2ff37ab6 hung up the client's *only* link.
  - Fixed in edafdde1: the hang-up applies only while another line has a call
    (`ME_LINE_BUSY_DIR=/run/v90ras/busy`); a lone link retrains.

**20:26-20:32, three-modem bundle, on 2189b0fb.** All three calls trained (no
Phase 2 deadlock), and the bundle reached three members at 20:28:32.
- **Line 0 (7800).** IP up at 20:27:24. The modem's 11.6 left us white; it
  retrained as the lone call, and data was back at 20:27:55, clean (0.045 from
  the grid, 23.8 dB). But **no PPP frame from the client arrived again**:
  v90ras0 RX stayed at 9 packets while TX grew, ping 0/5, and pppd logged no
  LCP echo after IP came up. The physical layer is clean, so the data link
  (V.42/V.14) doesn't resume across a retrain. Not yet diagnosed: the replay
  diverges from live (its training fails at 15.8 s), so the rig now runs
  `VPCM_ME_VERBOSE=1` and `ME_RX_BIT_DUMP` per call (c62c6dbd) to see it live.
- **Line 1 (7802).** Clean at 21600 (0.06, 31.6 dB). The modem opened 11.6
  at 20:28:08; our side never returned to data, and the modem hung up 33 s
  later.
- **Line 2 (7801).** White 14.3 s into data, then hung up by the new rule
  (another line was up).
- The repeated "only call up" message (128 per report) is fixed in 288487a6.

**21:00, on c62c6dbd.**
- **Line 0 (7800)** ran clean at 21600 the whole call (0.017-0.05 from the
  grid, no retrain), IP up at 21:00:42, ping 5/5 at ~300 ms.
- **Lines 1 and 2** (7802, then 7801) reached data but never joined the
  bundle. pppd logged "Couldn't create ppp interface v90ras0: File exists",
  exited, and the call dropped.
  - Cause: line 0's acknowledged LCP ConfReq (id 7) carried a 15-byte
    endpoint discriminator, `...1c.23.00`, with the last `00` missing. It was
    FCS-valid, so the client sent it that way. The later links sent all 16
    bytes. pppd names a bundle after the peer's endpoint, so the later links
    belonged to a different bundle, which tried to create a second
    `v90ras0`.
  - The 19:23 and 20:28 bundles all sent 16 bytes and attached.
  - Fix: `noendpoint` in `v90ras-options`. pppd ConfRejs the endpoint (RFC
    1990 makes it optional) and bundles on the peer name, which is the same
    for every link under noauth.

**21:13-21:17, on 5f3e3ff0 (noendpoint).**
- **Bundling works.** The client accepted the endpoint ConfRej, and line 1
  attached at 21:16:23 ("Link attached to v90ras0").
- **Line 1** then saw V.42 LAPM connected and clean data (0.056, 31.7 dB).
  The client sent a LAPM disconnect 1 s after PPP attached, most likely
  because line 0 was retraining at the time.
- **Line 0's first call:** after the modem's 11.6 the client resent IPCP
  every 3 s for 30 s, never hearing our replies.
- **Line 0's second call:** initial Phase 4 TRN read **4.6 dB** (Phase 3 TRN
  0.596 from the grid, PP lag8 33%), so our MP asked **7200**. Data then ran
  0.0045 from the grid at 29.7 dB, and B1 logged "carries about 21600
  bps; negotiated 7200". These modems' constant 11.6s are probably them
  raising a rate we under-asked.
- **The responder's TRN read 3.0 dB.** Cause: the T/2 eye-phase chooser
  moved the symbol instant on a front end the renegotiation had kept (eyes
  269.5 vs 251.8). Fixed in 338aa76a: no chooser while a renegotiation
  keeps the data-mode front end. Replay: 3.0 -> 23.8 dB, MP 7200 -> 19200.
- **Still open:** the initial Phase 3/4 under-measurement, and the loss
  hang-up counting a line that is still training as "another call".

**21:47, on 338aa76a.** Every TRN reading was sensible this time (28.5, 26.5,
22-26 dB). The 3 dB misreads are gone.
- **The open defect is data after OUR 11.6.**
  - Data entered at 21600, and B1 read 25.2 dB ("about 19200"). With
    `ME_V34_B1_RENEG=1` we renegotiated 40 ms into data.
  - Both MPs agreed on 19200. Our reneg MP0 does carry c2a=19200, despite
    the "asking 21600" log line, which is printed before the policy ceiling
    is applied.
  - The modem's B1 matched our 19200 template (0.997, residual carrier
    about 0), yet data was white from the first block: 0.64 from the grid,
    "shell index over k=28 bits" out of range in 23-37% of frames.
  - This reproduces offline on the tap
    (`v90_engine_replay ... --sync --at "AT+MS=V34,1,0,21600,0,21600"`,
    `ME_V34_BAUD=3200 ME_V34_B1_RENEG=1`).
- **Not the rate tables:** `engine_pair_test` at 3200/19200 and 3200/21600
  exchanges payload.
- **Not expanded shaping as such:** MP bit 32 selects the REMOTE transmitter
  (V.34 Table 13), and our receiver uses our own bit, as it should.
- **Next suspect:** mapping, superframe or trellis state carried across the
  renegotiation.
- **Not specific to renegotiation** (correcting the suspect above). The 22:04
  call's *initial* data at 19200 was white the same way: B1 0.997, shell
  index out of range in 32% of frames, 0.66 from the grid. It reproduces
  offline on tap line0-1094929.
  - Both white cases are **19200 at 3200 baud**; every clean call ran 21600.
  - `mappings_3200` matches V.34 Table 10 (19200: K=28, M 12/14).
    `engine_pair_test` at 3200/19200 passes, but both ends are our own code,
    so a symmetric error would cancel.
  - Next: decode one 19200 data frame from the tap by hand against 9.x
    (shell mapping, the 8 Q bits, superframe sync and the W/J mapping-frame
    switch, which differs at even and odd bit-rate codes).
- **Consequence:** V.42 detection runs during that white stretch, fails, and
  both modems drop to V.14 ("no error-control peer"). With no
  retransmission, PPP then dies of bit errors: at 22:33 data was clean
  (0.04, 24 dB), but our IPCP ConfReqs never got through and pppd quit,
  "No network protocols running".

**22:04 bundle, on a6048a86.** Two links bundled (line 1 attached 22:05:09).
Line 0 kept going white and was hung up after 86.8 s with line 1 up.

**The white data was our B1 conditioning gate (e9600f43).** Every white
data entry at 19200 and 21600 was DATA running on the unrefitted Phase 4 LMS
taps: `v34_rx_b1_batch_eq_enabled()` refitted them over B1 only at 26400
and up. B1 read 22 dB where the TRN refit read 31.4 dB, and the
decision-directed loops ran away. With the fit at every dense-baud rate the
recorded calls decode at 0.06-0.07 from the grid and 28-31 dB (22:04 19200
was 0.663, 21:47 21600 was 0.641). Details in `docs/v34_data_mode_rates.md`.

**23:33 fresh multilink dial, endpoint accepted (65b4f94d): 3 links bundled.**
All three links sent the endpoint `64.aa…` and attached to one v90ras0, with
no TermReq. Ping 5/5. Line 1 then dropped (23:34:58): the peer's §11.6
renegotiation produced no E, we fell back to an §11.5 retrain, and that link
was hung up. Every line sees the peer open §11.6 roughly 8 s into data.

**V.90 back on (315cb9b8).** "No Ja ever decodes" was ours. The Win95
modems send a correct Ja (V.90 8.3.1): a 2114-bit descriptor with N=192,
LSP=LTP=120, repeated until Sd. Our Phase 3 receiver demodulated it at about
10% bit errors, because blind CMA walked the equalizer from 6 to 17-19
degrees rms during Ja. So no descriptor passed its CRC, the modem retrained
after about 5.6 s, and its second INFO1a dropped V.90. With DD-LMS in
Phase 3 the two recorded calls (17:42, 17:44) decode Ja at 2.2-2.9 degrees
and send Sd. The peer's TRN runs about 2.2 s before Ja, so the parse lands
about 2.3 s after TRN locks. That is the peer's choice (9.3.2.3), not ours.
Everything past our Sd is untested live. `--profile` is gone from the unit;
`ME_V34_BAUD=3200` stays for the V.34 fallback.

## Windows 95 client

Dial-Up Networking, server type "PPP: Windows 95, Windows NT 3.5, Internet",
TCP/IP only, server-assigned address, any username and password (the
server runs `noauth` since 2026-11-07: a home test rig, and the first real
call failed only on the login). Multilink: add the second (and later) modems
under the connection's Multilink tab, each dialling 2950 (or a fixed
2951-2960 number).

## Cisco 2911 FXS gateway (TopRouter, 192.168.88.62)

The Win95 modems sit on TopRouter's FXS ports 0/3/0-0/3/3 (7800-7803) and
reach Asterisk as SIP endpoint 6501, through outbound dial-peer 6502
(`[2-9]...`, which covers 2950) or 6501 (`1.T`). Management is telnet with
`login local` (its SSH offers only SHA-1 key exchange, which current OpenSSH
refuses). Set modem-safe on 2026-11-08 and saved with `write memory`:

| Where | As found | Now | Why |
|---|---|---|---|
| FXS 0/3/x | `compand-type a-law`; the SIP leg is u-law | u-law | no law conversion between the modem and the RTP |
| 0/3/1 | `output attenuation -6` (+6 dB toward the modem) | 0 dB out, 0 dB in on all four ports | a hot downstream compresses (see the VG224 loss-plan finding) |
| 0/3/0 | adaptive playout | fixed, nominal 40 ms, on FXS, BRI 0/0/0-0/2/1 and dial-peers 6501/6502 | adaptive playout inserts and deletes samples |
| 0/3/2 | fixed playout, nominal 200 ms | the same fixed 40 ms | 200 ms only adds to the PPP round trip |
| 0/3/3 | EC on; VAD and CNG at their defaults | EC, NLP, VAD and comfort noise off on every FXS port | each one rewrites the samples |
| 6501, 6502 | `dtmf-relay rtp-nte sip-info ...`, plus `dtmf-interworking` on 6502 | removed | digit detection can take modem energy for DTMF and mute the audio. CME phones now send DTMF in-band |

**2026-11-13: 0/3/3 (7803, the MT9234) is temporarily `compand-type a-law`,
not saved,** to confirm that the µ-law setting cost the MT9234 V.90 to the
Eicon BRI lines (it did: `docs/v90_hardware_interop.md`). While it is A-law,
7803's calls into v90ras are converted on the FXS. Revert with
`no compand-type a-law` under `voice-port 0/3/3`.

Left alone: `network-clock-select 1 BRI0/1/0` (the +62 ppm reference that
our loop timing follows), the A-law BRI ports and the FXS impedance (default
600r). No live modem call has been made on the new settings yet.

First calls after the change (2026-11-08 11:14-11:20, all from 7800):

- Law: before the change, upstream codewords had **0.00%** outside the A-law
  image (4 calls on 2026-11-07, 1.2-6.8 M samples each), so 0/3/x was
  transcoding A-law <-> u-law. After it: **0.5-3%** (call line0-1103132), so
  that hop is gone upstream. The modem's CPt still sets bit 35 (A-law codec
  output) with codec constellations that differ (bit 128), so it still sees an
  A-law D/A downstream.
- V.90 via 2950: the first call from these modems to get past Ja. Ja decoded
  (N=197), then Sd, TRN1d, Jd, one DIL cycle and Ri. The modem's CPt
  (drn=16, 6 constellations, CRC good) has **constellation 5 empty** (0/0
  points, used by frame interval 5) and constellation 2 at 41 points against
  101-103 in the others. v90_configure_phase4_mapper refuses it (an Mi of 0
  breaks §5.4.1's 2^K <= prod Mi), so we never send TRN2d. The modem retrains
  to V.34 after 17 s. This reproduces offline with v90_engine_replay --sync.
  CAUSE, ours: our DIL is exact (all 13002 symbols of the cycle match the
  descriptor, starting at offset 0 mod 6 from Sd), but the engine's default
  one-cycle cap sent Ri 1.63 s into DIL. The modem went on sending SCR,
  studying our Ri as if it were DIL, and gave its 9.3.2.10 S only at 28.5 s,
  14 s later; its CPt was built from Ri (interval 2 has no Ucodes 31-80,
  interval 5 is empty). 9.3.1.6 says to repeat DIL until that S. With
  `ME_V90_DIL_AUTOTERMINATE_CYCLES=0` the replay repeats DIL, detects the S
  at 28.5 s and ends on a segment boundary; v90ras.service sets it since
  2026-11-08. LIVE 11:36 (line0-1121194): the modem's S came after two DIL
  cycles (3.2 s), CPt was accepted at once, and V.90 DATA was reached twice
  (52000 down / 16800 up), but upstream B1 was never acquired: a rejected
  frame disqualified the CP tap's E watch and nothing started the B1 search.
  Fixed in 596bdde2 (deployed 11:44; the old binary is kept as
  /opt/v90ras/sip_v90_modem.df9781d5). Its replay at the live 16800 acquires
  B1 (fit 99.4%) and decodes the upstream at 21 dB.
- x2 via 2959/2960: every call since 2026-11-08 00:04 stops at X2
  RECORD_WAIT and fails 10 s later, the same before and after the change.

2026-11-08 11:48, the first V.90 data with LAPM (line0-1122235, 7800): DIL
ended on the modem's S after about 2 cycles, CPt was accepted at once, and B1
was acquired through the batch-CP' path (fit 100%, 0.011 to the grid). The
upstream decoded at 35 dB (symbol error 0.009, 0 bad shell frames in about
114 per window), and **V.42 LAPM connected** at 52000 down / 16800 up. Each
data period then ended within 1.5-6 s on OUR "abrupt discontinuity" resync
request (§9.6 Rd), which this modem does not answer, so a retrain followed.
- 11:48:55: OURS. One RTP packet was lost (RTP timestamp +25.18 s; that call
  lost 4 of 2970 packets, against 0 in about 108k yesterday). The receive tap
  has no fill there. Instead it has a double fill at the next loss (+25.96 s;
  the same again at +37.80/+37.90 s), so the 0.74 s between were 160 samples
  short. That is a 64-symbol step, and the resync came 80 ms after it. Putting
  the 160 samples back in the tap removes that resync from the replay. The
  cause is pjmedia's jitter buffer, which reset its origin when a packet
  arrived into an empty buffer, dropping the lost slot. With
  `ME_JB_MS=40` (about one frame of real headroom) that is the usual case;
  at the 200 ms default it never happens. Fixed in the vendored
  `pjproject/pjmedia/src/pjmedia/jbuf.c` with a regression test
  (`jbuf_gap_test`): stock pjmedia steps in 215 of 240 simulated single-loss
  runs at 40 ms, the fixed one in 0 of 240 for every loss pattern tested.
- 11:49:14: not drift. The data period after the first retrain started at
  symbol error 0.043 (against 0.009 before) and lasted 1.4 s; no RTP loss.
- 11:51:10 (line0-1122312): a weak B1 fit (best of 7 windows, 0.134 to the
  grid), then the resync at once.
- The ~80 ppm T/3 offset (+0.00024 samples/symbol) is absorbed: slips count
  up while the symbol error stays at 0.009, so it caused none of these.
- Seen while tracing: under LAPM the upstream frame-phase sweep never locks
  (HDLC flags read 75% ones and carry no V.14 framing, and both lock rules
  need one or the other), so it keeps stepping through all 112 candidates
  during clean data. It did not move the symbol error in replay. OPEN whether
  the steps cost LAPM frames.

2026-11-08 12:33: a site-wide reboot. CT 117 got no DHCP lease (the router
made no offers 12:34:24-12:35:52 and dhclient gave up on IPv4), and Asterisk
dropped every endpoint-modem endpoint (the known load-order problem: fix it
with `module reload res_pjsip.so`). Both were fixed by hand at 12:57.
The reboot also reset the host's `/dev/ppp` to `0600 root:root`. CT 117 is
unprivileged, so inside it the node belongs to `nobody` and every pppd
failed with "Couldn't open the /dev/ppp device: Permission denied" (restart
counter 3078 by 14:34): no call after the reboot had a PPP peer, so LAPM came
up into nothing. `chmod 666 /dev/ppp` on vmhost at 14:36 brought all 10
members up, and `/etc/udev/rules.d/99-ppp-lxc.rules` on vmhost
(`KERNEL=="ppp", MODE="0666"`) now keeps it across reboots (checked by
resetting the node to 0600 and re-triggering udev).

2026-11-08 12:59 and 13:04 (line0-47498, with the jitter-buffer fix
c7c67863): no RTP loss and the peer's level steady, yet the V.90 upstream
went from 0.04 to 0.31 from the grid within 64 symbols, with no rotation, no
gain change and no timing offset that helped, and we requested the §9.6
resync 0.8 s into data. Just before, the line echo canceller's tracker had
moved its bulk delay 814 -> 1999 samples (+148 ms) and back, in pairs, right
after DATA began. It did this 9 times in the 12:59 call and 4 in the 13:04
one, against none at 11:48. In the 13:04 call B1 never validated. Replayed
with the tracker held (`ME_LEC_NO_TRACK=1`, one variable), the 12:59 data
period stays at 0.0074-0.0081 until the recording diverges at our live Rd.
The Cisco's playout is fixed, so the echo path cannot move and nothing is
lost by holding it: set in `v90ras.service` (CT 117 drop-in
`lec-no-track.conf`, 13:08). OPEN: what the tracker finds at about 250 ms.
Our TX repeats at 1188 samples during Phase 4 training (r = 0.95 at 17 s),
but a synthetic stationary echo with that TX does not reproduce the move.
- 11:47: one call failed in V.34 Phase 2 (INFO0 retry, no valid INFO1a, then
  our TX went IDLE and stayed silent for 38 s until the modem hung up). OPEN.

2026-11-08 14:43: **PPP up over V.90** (line 0, 52000/16800; IPCP gave
10.254.200.2, ping 3/3 at 236 ms). The lines that fell back to plain V.34
(the modem's Table 11 INFO1a) all decoded white. That was our B1-conditioning
gate, which skipped V.90-to-V.34 fallbacks; f938b387 fixes it, see
`docs/v34_data_mode_rates.md`. Still OPEN:
- Line 1 at 14:43:48 sent an INFO1d with N=0 at every baud. Its receiver
  left L1/L2 after 60 ms, behind an INFO0 retry, so there was no probe
  analysis. The modem repeated INFO1a against it for 50 s and hung up.
- The modem retrains about 30 s into V.90 data (52.7 s at 14:34, 50.6 s at
  14:43) while the upstream is clean (0.009). At 14:43 it came back to data
  in 13 s and PPP survived.

2026-11-08 15:02 (f938b387): the line 1 fallback decoded (0.09 from the
grid, 36 dB) and joined the bundle as a second link, ping 2/2. But V.42
detection failed: our B1 check opened a 11.6 (31200 -> 28800) 40 ms into
data, over the modem's ODP, so the link ran V.14 with no error correction.
95bf13ee makes the B1 check wait for detection
(`ME_V34_B1_V42_DEFER_MS`, default 4000).

2026-11-08 15:38-15:40 (95bf13ee): fallbacks now decode cleanly (19200 at
0.011 from the grid, 36 dB), and the B1 check waited for detection as
intended. Detection still failed, in 0.78 s: the fallback entered DATA at
31200, which was white (53% ones in the bit dump, with no ODP in it or
after it). Our MP had asked the INFO1a ceiling, because the fallback sent
MP at 512T, before our receiver had trained: TRN read 5-8 dB there, against
33-34 dB from the refit 2 s later. PPP still came up, over V.14 with no
error correction. 1cb06bc6 holds TRN until our receiver has trained
(V.34 11.4.1.1.2) and asks the measured rate (`ME_V90_FALLBACK_TRN_RATE`).

2026-11-08 16:06-17:20 (1cb06bc6), 28 calls from 7800/7801/7802. PPP came up
(16:08, V.90 52000/16800), but no V.90 data period lasted. Three recurring
failures:
- **7800: every V.90 data period ends at 31.4-31.5 s** (about 40 periods
  since 11:37, so before and after `ME_LEC_NO_TRACK`). The sequence
  (line0-242133, 17:11): the upstream is clean (symbol error 0.008, 0 bad
  shell frames, 75% ones = LAPM flag idle) until **28.6 s**. Then the eye
  drops in one step (lattice distance 0.838), our slip search corrects +0.17
  sample, the eye reopens (0.010), but the bits read **41% ones**, so the
  framing is wrong. V0 parity is lost at the same moment. At 31.4 s the error
  rises again, we request the §9.6 resync (Rd), and the modem answers
  with a full retrain 100 ms later. After two or three of these it falls
  back to V.34. NOT the peer's timing: `tools/measure_timing_slips.py` finds
  no whole-sample step in its transmitter (the epoch drifts smoothly, about
  104 ppm). NOT the transport: RTP had 0 loss and 0 gaps in 25833 packets.
  Loop timing holds (RX +60.2 ppm, TX +66.7 ppm against the host over
  516 s). A replay (`--sync`, the rig's env) cannot judge it: its MP asks
  12000 against the live 16800, so B1 never acquires. OPEN, and probably ours,
  because it lands at the same data time on every call.
- 7801: V.90 data ends at 10.5 s the same way, and V.42 detection fails.
  Its V.34 fallbacks go white 2-4 s into data, or after 25-48 s.
- V.34 fallbacks with LAPM run for minutes, then a peer 11.5 retrain hangs
  that member up (`ME_V34_LOSS_HANGUP_AFTER_MS`), and the client drops the
  other links about 1 s later.

**Cause of the 31.4 s retrain: the modem's own §9.6 renegotiation, which we
did not answer.** A replay with `--echo-ref` reproduces the live call exactly
(it asks the live 16800, and logs the same "0.838 -> 0.414" and "71 of 448"),
and the equalized-symbol dump (`ME_V90_UPSTREAM_SYM_DUMP`) shows the cause.
At symbol 90897 (28.6 s) the error steps from 0.01 to 1.77 in one symbol,
because the modem starts sending V.34 S: about 100 symbols that alternate
(+4,+4)/(-4,+4). There is no change in the input level, no timing step and no
equalizer drift (tap centroid 8.80-8.82 over the whole period). V.90 §9.6 lets
the analogue modem start a renegotiation, and also lets it retrain its own echo
canceller that way (CPs, bit 30). Our S-in-DATA responder is off by default
for V.90 (`ME_V90_RENEG_RESPOND`, on only for V.34), so the slip search took
the S for a timing step and moved the timing +0.17 sample. Framing was lost,
we sent our own Rd at 31.4 s, and the modem gave up and retrained. With
`ME_V90_RENEG_RESPOND=1` the replay detects the S at 28.6 s, with no false
detection in the 28 s of data before it, and answers with Rd, R-bar-d,
TRN2d and MP (§9.6.1.2.2; the instant "CPt accepted" there is only a log line,
because TRN2d comes before the new CP). A replay cannot judge what follows our
answer, so the rig now sets `ME_V90_RENEG_RESPOND=1` as a live A/B. A
remaining nit (the responder sent Rd at once instead of waiting for the
S-to-S-bar transition, §9.6.1.2.1) is fixed: it now waits for the end of the
peer's S run, which the replay sees 20 ms after S is reported (S read 30-40
ms, 128T at 3200 baud = 40 ms), with an 80 ms fallback. The same replay found
a worse defect of ours: answering also armed §9.6.1.1.1's wait for an S answer
(the S we were answering, already past), nothing cleared it across the retrain
that followed, and at +52.9 s it reported Phase 3's own S as a new peer
renegotiation, mid-startup. Restarts now clear the watch, the responder no
longer arms it, and only an S seen in DATA is answered. 7801's 10.5 s loss is probably the same
thing and is unchecked.

## Deployed 8f918728 (2026-11-09 19:10)

The rig was on 1cb06bc6 (2026-11-08). This adds the 2026-11-09 V.34 work, none
of it live-verified yet:
- 11.6 with peers that send no TRN, in both roles (884ad606). These Win95
  modems send none.
- Our own 11.5 retrain waits for the caller's silence (4cd143f5).
- 11.6 keeps the symbol clock running from DATA into S (3110aeb6).
- Calibrated TRN rate tables at 2400-3000 baud (209cd307, 8f918728).
- The data-mode timing steer holds the main tap at low rates (d793e687).

The old binary is `/opt/v90ras/sip_v90_modem.1cb06bc6`. Only `v90ras` was
restarted. The v90isdn server shares `/opt/v90ras/sip_v90_modem`, and its
running supervisor still has the old file mapped.

## Deployed f03cd55b (2026-11-09)

On top of 8f918728:
- Godard timing is out of DATA on the plain V.34 path (4945aba4). It caused
  sudden permanent white-outs on long calls.
- 2743/24000 and 2800/26400 are in the rate tables (f03cd55b).

The previous binary is `/opt/v90ras/sip_v90_modem.8f918728`. Again only
`v90ras` was restarted.


## Deployed 66365f34 (2026-11-09 20:47)

On top of f03cd55b: INFO1a reserved bits 12:17 and 32:33 are no longer
checked (V.90 Table 10, "not interpreted by the digital modem"). The Win95
modem on 7801 sets bit 33, so its V.90 requests had been rejected and every
call from 7801 fell back to V.34.

The previous binary is `/opt/v90ras/sip_v90_modem.f03cd55b`. Only `v90ras`
was restarted; the v90isdn server still has the older file mapped.

## Deployed 8bfd6695 (2026-11-09 21:03)

On top of 66365f34: SpanDSP rejects a V.90 Table 11 INFO1a whose fields are
out of range or that projects 0 bit/s for the rate it selects. A CRC-valid
frame like that came out of the MultiTech's Tone A and dropped a V.90 call to
V.34. The previous binary is `/opt/v90ras/sip_v90_modem.66365f34`. Only
`v90ras` was restarted.

## Deployed 2aa392b9 (2026-11-09 22:35)

On top of 8bfd6695: the V.90 digital side keeps INFO1a framing armed while it
waits for Tone A (§9.2.1.1.7). The Courier on 7802 sends INFO1a before our
INFO1d, and the disarmed framer lost it, so its first Phase 2 timed out and
restarted. The previous binary is `/opt/v90ras/sip_v90_modem.8bfd6695`. Only
`v90ras` was restarted; v90isdn still runs an older binary.

## Deployed c7d77d0d (2026-11-09 23:11)

On top of 2aa392b9: the V.90 upstream never offers 14400 at 3200 baud (a
14400 measured cap asks for 16800), and the upstream fall-back no longer acts
on rates with no measured SNR threshold. Every 14400 upstream from the Lucent
(7801) and Agere (7803) MultiTechs had decoded white. The previous binary is
`/opt/v90ras/sip_v90_modem.2aa392b9`. Only `v90ras` was restarted.

## Deployed 2288b8fc (2026-11-10 08:02)

On top of c7d77d0d, reverting its 14400 exclusion (45a71e6e; the fall-back
guard stays): a blind-placed V.90 upstream B1 that decodes white is searched
over all eight symbol offsets. The Agere MT9234 starts B1 7 symbols after the
4-point signal, not the 9 the blind placement assumed, so the Lucent (7801)
and Agere (7803) upstreams had decoded white at every rate. The previous
binary is `/opt/v90ras/sip_v90_modem.c7d77d0d`. Only `v90ras` was restarted.

## Deployed 6d798ffa (2026-11-10 08:16)

On top of 2288b8fc: the V.90 digital side sends INFO1d as soon as it sees the
analogue modem's 9.2.2.1.8 Tone A, without the guard/carrier "still holding
Tone A" hold. That hold had delayed INFO1d by 650 ms on every Phase 2 of an
Eicon Diva V90A call (eicon420 ttyds6, controller 4, dialling 2950), and the
Diva never answered with INFO1a. The previous binary is
`/opt/v90ras/sip_v90_modem.2288b8fc`. Only `v90ras` was restarted.

Diva V90A caller recipe (eicon420, spare `/dev/ttyds6`; only controllers
3, 4, 7 and 8 reach the Cisco, the others answer NO DIALTONE):
`AT&F5`, `AT+iQ=o4`, `AT+iO7911`, `AT+MS=V90A,0`, `ATX4`, `AT&D0&K0`, then
`ATD2950`. `tools/soak/ppp_ping_soak.py --pty /dev/ttyds6 --at ...` runs there
unchanged (copy it to eicon420; it needs only python3 and sudo).

## Deployed fcdb8284 (2026-11-10 08:26)

On top of 6d798ffa: the full V.90 Phase 2 measures RTDEd (9.2.1.1.4), used
only to extend the 9.2.1.2.4 third-reversal deadline to 900 ms + RTD. At a
bare 900 ms it fired 20 ms before the Diva's reversal (RTDEd 148 ms over
ISDN and SIP). The previous binary is `/opt/v90ras/sip_v90_modem.6d798ffa`.

## Deployed 63a0c4d8 (2026-11-10 08:32)

On top of fcdb8284: the pre-MP upstream 4-point SNR takes the best of the
newest window and windows up to 400 symbols older. Live, the newest window
ran past the Diva's CPt repeats and read 10.3 dB (36.7 dB in replay), so no
MP rate was offered and the Diva chose V.34 on the restarted Phase 2. The
previous binary is `/opt/v90ras/sip_v90_modem.fcdb8284`.

## Deployed e350541e (2026-11-10 08:50)

On top of 63a0c4d8: startup Ri runs 1600T (200 ms: T downstream is 1/8000 s;
this note first said 500 ms) before R-bar-i
acknowledges CPt (`V90_RI_MIN_SYMBOLS`, floor 192). At 192T the Diva kept
repeating CPt through our TRN2d and MP and retrained. The previous binary is
`/opt/v90ras/sip_v90_modem.63a0c4d8`.

## Deployed 89575b3e (2026-11-10 10:27)

On top of e350541e: loop timing starts each call at the frequency the last
call from the same caller converged to (`/var/tmp/v90ras/sip_v90_modem-tx-clock.tsv`;
7911 = +63.97 ppm). `ME_PCMU_ALAW_BEARER` is available but not set. Removed
the temporary `ME_V90_MAX_TX_DBM0_CODE=23` and `ME_V90_TRN2D_SYMBOLS=6000`
from `sip.env`. The previous binary is `/opt/v90ras/sip_v90_modem.e350541e`.

## Deployed deferred send (2026-11-10 11:40)

On top of 89575b3e: each frame is sent at the start of the next media tick
(`ME_TX_DEFER`). A seeded loop steers at 0.005 rad/s. A `TX tick stall`
line is logged when a tick starts late plus its bridge pass exceeds 25 ms.
`sip.env` again carries `ME_V90_MAX_TX_DBM0_CODE=23` and
`ME_V90_TRN2D_SYMBOLS=6000` (the 9 October Eicon recipe, all callers).

## Deployed upstream frame-phase fix (2026-11-10 13:21)

The V.90 upstream lock is released only on 240 accumulated shell frames at
>= 4% bad (or one window at >= 20%), and HDLC flags count as lock evidence.
The previous binary is `/opt/v90ras/sip_v90_modem.0dfb67f6`. Test the Diva
on controller 7 or 8: the NT ports of card 1331 (controllers 3 and 4) slip
about 34 ppm against the Cisco clock (docs/eicon_v90a_20261009.md).

## Deployed 809636b2 (2026-11-10 15:28)

On top of 956b5b69 (the CT sources matched that commit file for file): DIL
repeat-until-S is the built-in default, so `ME_V90_DIL_AUTOTERMINATE_CYCLES=0`
in `v90ras.service` is redundant but harmless. What changes here: DIL has a
5500 ms timeout (9.3.2.10; before, a missed S left DIL running until the
9.4.1 B1 deadline), and the S watch during DIL slides its window, so a
37-40 ms S is no longer missed on block alignment
(docs/v90_phase3_4_spec_audit.md, row 4). Built on the CT from the committed
`v90.c`, `v34rx.c` and `private/v34.h`; the CX93001 DIL fixture gives the same
result there as on the Mac. The previous binary is
`/opt/v90ras/sip_v90_modem.956b5b69`. Only `v90ras` was restarted, with no
call up.

Live the same evening, Diva V90A on controller 8: 6/6 calls on this binary
reached V.90 at 40000/31200 (one held 300 s, 292/293 pings), and 0/4 with
`ME_V90_DIL_AUTOTERMINATE_CYCLES=1` set through a runtime drop-in, arms
alternated (docs/v90_phase3_4_spec_audit.md, row 4). The drop-in is removed.

## Deployed 49a33091 (2026-11-11 01:49)

On top of 809636b2: Sd is held until two round trips after Ja began (V.90
9.3.1.3, `ME_V90_SD_RTD_HOLD`; docs/v90_phase3_4_spec_audit.md row 12). With
this server's 40 ms jitter buffer the hold is usually nothing. Three Diva
V90A calls on controller 8 straight after: 3/3 V.90 at 40000/31200 with PPP
up for the 40 s hold; one of them held Sd 35 ms (RTDEd 166 ms, Ja 357 ms old
at the parse), the other two not at all. The sources in `/root/v90ras` match
the commit. The previous binary is `/opt/v90ras/sip_v90_modem.809636b2`.
Only `v90ras` was restarted, with no call up.

## Deployed b2e98bfc (2026-11-11 08:27)

On top of 49a33091: `[V42] link disconnected: <reason>` is logged for each
LINK_DISCONNECTED (peer DISC 8.5.4, peer DM with F=0 or F=1 in data mode).
Built in `/root/v90ras` with the 12 uncommitted files that worktree already
carried (not reset, not committed); only `spandsp-master/src/v42.c` was
replaced. The previous binary is `/opt/v90ras/sip_v90_modem.49a33091`; the
new one is also kept as `/opt/v90ras/sip_v90_modem.b2e98bfc`. Only `v90ras`
was restarted, with no call up. slmodemd (CT 100, 6003) dialled 2950: V.90
31200 up / 53333 down, LAPM up at 08:29:22, still up 91 s later. A
slmodemd `+++`/`ATH` ended the rig side, but the call went to a V.90 9.6
retrain (no DISC was sent), so the DISC line did not fire on this test.

## Deployed e5fd69be (2026-11-11 08:44)

On top of b2e98bfc: a call in data mode that hears digital silence for S10
(1.4 s) hangs up (`[ME] V.34/V.90: no line signal for N ms after data mode`),
instead of retraining into nothing. Only a silence that began in data counts.
Applied as this commit's `modem_engine.c` hunk to `/root/v90ras` (its other
uncommitted files untouched). The previous binary is
`/opt/v90ras/sip_v90_modem.b2e98bfc-s10` (the b2e98bfc build); the new one is
kept as `/opt/v90ras/sip_v90_modem.e5fd69be`. Only `v90ras` was restarted.
slmodemd (CT 100, 6003) dialled 2950, LAPM connected, then `+++`/`ATH` from the
rig: the server hung up 1.4 s after the silence began (08:46:19). Before this
change the same test retrained for 3 min (see b2e98bfc).

## Deployed 4fa4d4c9 (2026-11-11 09:10)

Fixes a crash, not a tuning change: since 956b5b69 every **plain V.34** data
call died on x86 on its first data frame with K > 0 (V.34 9.3.2). The cause was
an integer divide by zero in the shell-index report in
`v34_rx_pack_output_bitstream`; dmesg showed `trap divide error`. The MultiTech
ISI on 7900 reported `CONNECT 31200 NoEC` because our process had already gone.
Before the fix it failed identically on 49a33091. arm64 returns 0 for an integer
divide by zero, so the Mac suites never saw it. V.90 calls were unaffected.
The fix was applied as a patch to `/root/v90ras` (its own uncommitted v34rx.c
work left in place). The previous binary is `/opt/v90ras/sip_v90_modem.e5fd69be-prefpe`;
the new one is kept as `/opt/v90ras/sip_v90_modem.4fa4d4c9`. Only `v90ras` was
restarted. `v90isdn` still runs the old binary, but it only matters if it
carries plain V.34.

After the fix, `tools/eicon_ppp_direct.py --law alaw --ext 7900`:
- `--mode v34`: 31200, V.42 LAPM + V.42bis, PAP up, 10% ping loss.
- `--mode v90`: the same, V.34 31200, 15% ping loss.

Asterisk has 2910 registered. `29107910` (the Diva gateway, called number 7910)
now reaches the MultiTech BRI and answers, but V.8 decodes a JM with no
modulations (offered V.34 or K56). Direct `7910` trains and gets LAPM.

## Deployed 3a3609be (2026-11-11 09:45)

The V.34 fall-forward no longer asks above the peer's MP maximum (V.34
11.4.1.1.4/.2.4). The data-entry trace now shows `peer MP allows a2c=… c2a=…`.
On the MultiTech ISI (7900 and 7910) the peer's MP caps both directions at
31200, but the B1 fall-forward used to ask 33600 0.9 s into data. That ask
always settled at 31200. The renegotiation's B1 matched no template (0.064),
and the receiver came out mis-framed under a clean eye. Before the change, a
fall-forward A/B on 7910 gave 2/2 clean calls with the ask off (0% ping loss)
and 0/2 with it on (15% loss; PPP never up). After the change: 7910 2/2, 7900
1/1, all PPP up with 0% loss, and no ask was made. The previous binary is
`/opt/v90ras/sip_v90_modem.4fa4d4c9`; the new one is
`/opt/v90ras/sip_v90_modem.3a3609be`.

Still open: our receiver mis-frames after an 11.6 B1 that is not in
10.1.3.1's reset state. That is the RasFinder fallback path in
`v34rx_data.c`: it keeps the good B1's offset and conjugation, but takes the
SWP phase and trellis state from the failed search. A peer-initiated 11.6 on
7910 at about 54 s showed the same thing.

## Deployed 8b698c2a (2026-11-11 10:00)

This fixes the open item above. After an 11.6 whose B1 matches no template,
the receiver now keeps the good B1's 8.2 switching-pattern phase (the MultiTech
ISI runs it one mapping frame ahead), opens the Viterbi across every state,
and runs V0 acquisition. Measured with a forced 11.6 at 12 s
(`ME_V34_RENEG_AFTER_MS=12000`):
- **Before:** about 1 new frame in 10 failed the shell-index check, and PPP
  lost 80% of its pings.
- **After:** 7910 2/2 and 7900 1/1, with two fallback B1s per call, the shell
  check at 12 or fewer bad frames in 18944, and 0% ping loss.

The verification build carried a diagnostic patch
(`/root/eicon-tests/diag.patch`, reverted before this build). The previous
binary is `/opt/v90ras/sip_v90_modem.3a3609be`; the new one is
`/opt/v90ras/sip_v90_modem.8b698c2a`. Only `v90ras` was restarted, with no call
up.

## Deployed 2af4189f (2026-11-11 13:12)

Diagnostics only: clear-channel calls now count HDLC aborts (`..., N aborts`
in the end-of-call line). A lost RTP packet's 0xFF fill aborts the frame it
lands in, which was counted nowhere before. No call path changed. Applied as
a patch to `/root/v90ras`; the previous binary is
`/opt/v90ras/sip_v90_modem.pre-2af4189f`, and the new one is kept as
`sip_v90_modem.2af4189f`. Restarted with no PPP call up.

## Deployed 55283044 (2026-11-11 14:50)

Diagnostics only: clear-channel PPP calls end with a `PPP totals:` line
(frames each way, bad FCS and overlong frames from the DTE). Built in
`/root/v90ras` on top of 2af4189f and kept as
`/opt/v90ras/sip_v90_modem.ppp-totals`. Restarted with no PPP call up.

## Deployed bf4a3fbb (2026-11-12 10:45)

V.90 upstream from analogue modems on their own clock (MT9234ZBA +105 ppm,
CX93001 -44 ppm): the pre-MP measurement, the 4-point B1 handoff, the
symbol-offset search and the 9.6 re-acquisition, as in
`docs/v90_hardware_interop.md` ("Analogue modems on their own clock"); and
V.92's PCM-upstream offer is withdrawn when a retrained attempt proves
nothing (`docs/v92_full_startup.md`). Built in `/root/vpcm-fix6` (6d377c76
plus the commit's four source files, checked byte for byte against
bf4a3fbb), kept as `/opt/v90ras/sip_v90_modem.bf4a3fbb`; the previous binary
is `sip_v90_modem.pre-bf4a3fbb`. Only `v90ras` was restarted, with no call
up; v90isdn takes the binary at its next restart.

The same night `tools/soak/qc_slm_pve.sh` restarted `v90ras` and `v90isdn`
nine times between 23:52 and 00:07 (its `pkill -x sip_v90_modem`, fixed in
bf4a3fbb). The journal shows no call connected on either service in the 12 h
before, so nothing was lost.

## Deployed 60ecf54c (2026-11-12 19:32)

V.90 upstream B1 re-acquisition after a retrain or renegotiation: the
out-of-sample check takes out the peer's carrier drift, the 4-point placement
is tried when a template is refused out of sample, a 4-point placement is
never committed white, an acquisition needs to be within 0.30 of the grid,
and V.90 search steps are three packets apart
(`docs/v90_hardware_interop.md`, item 7). Built in `/root/vpcm-fix6`
(bf4a3fbb plus the commit's `v34rx.c` and `private/v34.h`, checked byte for
byte), kept as `/opt/v90ras/sip_v90_modem.60ecf54c`; the previous binary is
`sip_v90_modem.pre-60ecf54c`. Only `v90ras` was restarted, with no call up;
v90isdn takes the binary at its next restart. Not yet verified live.

## Deployed 9d69c8cf (2026-11-12 20:48)

V.92: after Su-bar, Jd is held in whole repetitions until Jp's scrambler
state is one slmodemd trains on (`docs/v92_full_startup.md`, "Jd held for
Jp's scrambler state"). v90ras answers V.90/V.34 callers, where the change
does not act. Built in `/root/vpcm-fix6` (60ecf54c plus the commit's
`v90.c`, checked byte for byte), kept as
`/opt/v90ras/sip_v90_modem.9d69c8cf`; the previous binary is
`sip_v90_modem.pre-9d69c8cf`. Only `v90ras` was restarted, with no call up.

## Deployed 4f489d6a (2026-11-12 22:35)

V.90 upstream: the V0 epoch is seeded at B1's end, and the U0 trellis
constraint starts with data frame 1, not ~280 ms into DATA
(`docs/v90_upstream_data_path.md`, "The V0 epoch is known at B1's end").
Before this, a single upstream symbol error early in DATA reached the DTE as a
burst. Built in `/root/vpcm-fix6` (9d69c8cf plus the commit's `v34rx.c` and
`private/v34.h`, checked byte for byte), kept as
`/opt/v90ras/sip_v90_modem.4f489d6a`; the previous binary is
`sip_v90_modem.pre-4f489d6a`. Only `v90ras` was restarted, with no call up.
Not yet verified live.

## Deployed ae59c34a (2026-11-12 23:06)

V.90 upstream: when a template fit placed B1, the U0 trellis constraint runs
from B1's first pair, and the B1 re-park allows for the pending mapping frame.
This supersedes 4f489d6a's seed at B1's end, which could not reach errors in
data frame 0 (`docs/v90_upstream_data_path.md`, "The V0 epoch is known from
B1"). Built in `/root/vpcm-fix6` (4f489d6a plus the commit's `v34rx.c` and
`private/v34.h`, checked byte for byte), kept as
`/opt/v90ras/sip_v90_modem.ae59c34a`; the previous binary is
`sip_v90_modem.pre-ae59c34a`. Only `v90ras` was restarted, with no call up.
Not yet verified live.

## Deployed 121c6c53 (2026-11-13 00:26)

V.90 upstream fall-forward never asks above the peer's CP upstream mask
(§9.4.2.4). Live, before this, the MT9234 was asked 28800 against its 26400
and the CX93001 26400 against its 9600, on every call
(`docs/v90_hardware_interop.md`, "USB modems into v90ras after ae59c34a").
Built in `/root/vpcm-fix6` (ae59c34a plus the commit's `modem_engine.c`,
`v90.c`, `v90.h`, checked byte for byte), md5 ae2f2026…, kept as
`/opt/v90ras/sip_v90_modem.121c6c53`; the previous binary is
`sip_v90_modem.pre-121c6c53`. Only `v90ras` was restarted, with no call up.
Live, MT9234 3 of 3 pass (46666-48000/26400, 175/175, 175/175, 174/174
pings): each logs "the peer's CP enables at most 26400 bps", with no
renegotiation or retrain in 180 s (2 of 3 passed before, the third lost
after the 28800 ask).

## Deployed 5e590e24 (2026-11-15 23:33)

V.90/x2 upstream: the DATA path's zero-delay derotator holds still while
the T/3 upstream receiver's own carrier tracker runs. The two were in
cascade on the same symbols, and the older loop's own noise (0.15 degrees
RMS, excursions past 1 degree for 20 to 35 ms) sat on the upstream
(`docs/x2_implementation.md`, "Upstream at 31200"). Replayed on four taps
from `/var/tmp/v90ras/calls`, old against new: 26400 SNR +0.15 to +0.2 dB
on three, 9600 unchanged, none worse. `V34_V90_T3_DATA_DEROT=1` restores
the old loop.
Built in `/root/vpcm-fix6` (121c6c53's tree plus the commit's
`v34rx_data.c`, checked byte for byte; the commit's `modem_engine.c` hunk
is replay-only and was left out), md5 bd61363b…, kept as
`/opt/v90ras/sip_v90_modem.5e590e24`; the previous binary is
`sip_v90_modem.pre-5e590e24`. Only `v90ras` was restarted, with no call
up; ten lines registered. Not yet verified with a live V.90 call.

## Deployed 8223cf37 (2026-11-16 07:32)

V.90 9.2.1.1.8: one INFO1d, then silence; the old group of four lay over
the analogue modem's Phase 3 echo training (`docs/v90_hardware_interop.md`,
last section). `ME_V90_INFO1D_COPIES=4` restores the group.
Built in `/root/vpcm-fix6` (the 5e590e24 deploy tree plus the commit's
`v34tx.c`, which differs from 121c6c53's by this change only; md5
93191533…), binary md5 af38fa52…, kept as
`/opt/v90ras/sip_v90_modem.8223cf37`; the previous binary is
`sip_v90_modem.pre-8223cf37`. Only `v90ras` was restarted, with no call
up; ten lines registered. Verified live: the hardware Courier (FXS 7800)
dialling 2950 twice, CONNECT 50666/ARQ/V90/LAPM both times.

## Deployed ba9e61ba (2026-11-16 10:44)

V.90 Phase 2: a repeated INFO0a is no longer read as the Tone A reversal
(`docs/v90_hardware_interop.md`, last section).
`ME_V90_INFO0A_REPEAT_CHECK=0` restores the old behaviour.
Built clean in `/root/vpcm-fix6` (the 8223cf37 deploy tree plus the
commit's `v34tx.c` and `private/v34.h`, which differ from 121c6c53's by
8223cf37 and this change only), binary md5 87b6ed14…, kept as
`/opt/v90ras/sip_v90_modem.ba9e61ba`; the previous binary is
`sip_v90_modem.pre-ba9e61ba`. Only `v90ras` was restarted, with no call
up; the lines registered. Verified live: the hardware Courier (FXS 7800)
dialling 2950 four times, CONNECT 50666/ARQ/V90/LAPM each time. The
x2live rig tree on CT 117 is at ba9e61ba.

## Deployed ecb4832c (2026-11-16 22:22)

V.90 digital modem: one point before INFO0d instead of 16 bauds of carrier
(the cause of a Courier's INFO0a repeats), and the 53 ms that complete V.8's
75 ms of silence before it (`docs/v90_hardware_interop.md`, last section).
`ME_V34_INFO0_PREAMBLE_BAUDS=16` and `ME_V90_PHASE2_START_SILENCE_MS=0`
restore the old behaviour.
Built in `/root/vpcm-fix6` (the ba9e61ba deploy tree plus the commit's
`v34tx.c` and `spandsp/v34.h`, and the commit's `modem_engine.c` hunks
applied as a patch, offsets -3 and -71 lines), binary md5 8fde6f20…, kept
as `/opt/v90ras/sip_v90_modem.ecb4832c`; the previous binary is
`sip_v90_modem.pre-ecb4832c`. Only `v90ras` was restarted, with no call
up; the lines registered. Verified live: the hardware Courier (FXS 7800)
dialling 2950 six times, CONNECT 50666 V.90 each time. The x2live rig tree
on CT 117 is at ecb4832c.

## Deployed d75fc379 (2026-11-17 09:26)

V.90 digital modem: the 9.3.1.3 Sd hold now also keeps Sd's arrival no later
than 950 ms after the analogue modem starts Ja (a USR Courier's own deadline
is fixed near 960-1000 ms; `docs/v90_hardware_interop.md`). At v90ras's 40 ms
buffer (RTDEd about 150 ms) the hold is zero anyway, so nothing changes here
unless the round trip passes about 300 ms. Built in `/root/vpcm-fix6` with
the commit's `v90.c` hunk applied as a patch (`/root/cap-d75fc379.patch`),
binary md5 b5000454…, kept as `/opt/v90ras/sip_v90_modem.d75fc379`; the
previous binary is `sip_v90_modem.pre-d75fc379`. Only `v90ras` was restarted,
with no call up; the lines registered. Not live-verified.

V.90 digital modem (ed18b0f9, 2026-11-17 11:44): the Ja heuristic fallback
cannot start before the peer's TRN, so it no longer sends Sd during the
analogue modem's TRN (live against our own analogue role: 0/2 -> 4/4). It
only ever releases later than before. Built in `/root/vpcm-fix6` with the
commit's `modem_engine.c` diff applied (`/root/ja-ed18b0f9.patch`), binary
md5 4ca2f815…, kept as `/opt/v90ras/sip_v90_modem.ed18b0f9`; the previous
binary is `sip_v90_modem.pre-ed18b0f9`. Only `v90ras` was restarted, with no
call up; the lines registered. Not live-verified against v90ras's callers.

The x2live rig tree on CT 117 is at ed18b0f9 (2026-11-17 11:45).

TopRouter clock (2026-11-17 12:41): `network-clock-select 1 BRI0/1/0` had
been off since the 14-15 November x2 test, so the Diva BRIs slipped 55.6 ppm
against the router. Restored and BRI0/1/0 bounced; saved with `write memory` at 12:44, which also saved any other unsaved running-config changes; that included the 2026-11-13 `compand-type a-law` on 0/3/3, which was then reverted (`no compand-type a-law`, saved 12:45, 0/3/3 back to the default µ-law).
Check
`show network-clocks` shows BRI0/1/0 primary before any Diva test.

## Diva power code 21 (2026-11-17)

`sip.env` now carries `ME_V90_MAX_TX_DBM0_CODE=21` (-11 dBm0) instead of 23,
with `ME_V90_TRN2D_SYMBOLS=6000` unchanged. In an alternated A/B on the
mu-law route the Diva reached V.90 data on 6/6 calls at code 21 (41333/31200)
and 6/6 at code 23 (40000/31200); codes 13-17 fail its Phase 4
(docs/v90_analogue_role.md). The setting applies to every caller of v90ras,
and only the Diva has been measured at 21. Revert to 23 if another caller
loses V.90.

Courier (FT4TQOFT, FXS 7800, dialling 2950), 4 calls per code: code 21 gave
V.90 data on 2/4 (45333, 52000), code 23 on 1/4 (50666). At either code every
V.90 call fell back to V.34 within 11-20 s, the Courier itself reported
`CONNECT 26400/V34` on all 8, and the calls without V.90 went straight to V.34
19200 and stepped down to 9600-14400. So 21 costs the Courier nothing it had
at 23; its V.90 not holding is a separate, older problem. The Win95 modems
(manual dialling) and the MT9234 (unplugged) are unmeasured at 21.

## Deployed 9ca245ef (2026-11-17 20:15)

V.90 upstream B1 is also accepted on the shape of its error (>= 60% of
components within 0.5 of the grid, distance <= 0.50), not only at distance
<= 0.30: the Courier's real 28800 upstream read 0.34-0.41 and was abandoned
to a retrain into V.34 (`docs/v90_hardware_interop.md`). Built in
`/root/vpcm-fix6` with the commit's spandsp hunks (`/root/oos-9ca245ef.patch`;
the objects including `private/v34.h` were deleted first), binary md5
6cf703d1…, kept as `/opt/v90ras/sip_v90_modem.9ca245ef`; the previous binary
is `sip_v90_modem.pre-9ca245ef`. Installed by rename (the running file is
busy), only `v90ras` restarted, with no call up; the lines registered.

Not live-verified: on the 4 Courier calls straight after, the Courier itself
declined V.90 in its INFO1a (downstream code 3, 3000 baud, U_INFO 11) on every
call, so no B1 was ever checked. The same INFO1a decline was 5 of the 8
earlier v90ras Courier calls, but 0 of 2 into the x2live test server
(`courier_serial_call.py --env MODE=v90`). The v90ras service runs
`ME_V34_BAUD=3200`, `ME_V34_B1_RENEG=1` and `ME_V34_B1_RATE_MARGIN_DB=3`, which
the test server does not; that is the first place to look.

**Correction (2026-11-17 20:40): the INFO1a declines were the Courier's stored
profile, not v90ras.** The ad hoc dial script reset with `ATZ`, which loads the
Courier's NVRAM profile; from `AT&F1` (what `courier_serial_call.py` uses) it
connects `52000/ARQ/V90/LAPM/V42BIS` into 2950 on 2/2 calls, PPP starting.
That also confounds the code 21 vs 23 Courier counts above (all `ATZ`). On
those two `AT&F1` calls the deployed shape gate accepted B1 on its first
window (distance 0.376 at 75.6% near, 0.384 at 71.5%), with no retrain: the
fix is live-verified. `ME_V34_BAUD=3200`, `ME_V34_B1_RENEG=1` and
`ME_V34_B1_RATE_MARGIN_DB=3` on the test server changed nothing (52000/28800).
Dial the Courier from `AT&F1`.

Courier A/B from `AT&F1` (2026-11-17 21:00, alternated 21/23/21/23, 3 calls
per arm): V.90 + LAPM/V.42bis on 12/12. Code 21: 52000/28800 6/6; code 23:
50666/28800 6/6. So 21 is one step better for the Courier too, and stays.
Every B1 was accepted on its first window by the shape gate: distance
0.344-0.427, 68.0-75.2% within 0.5 of the grid. Five of the twelve read above
0.40, which the gate before 9ca245ef would have abandoned to a retrain.

## Deployed ecb14c36 (2026-11-18)

Phase 4 TRN coherence latch (`ME_V34_P4_TRN_COH_LATCH`, 0.5): once DD-LMS
has run on coherent decisions it keeps the taps to the end of TRN, where
a0f2450b's gate used to hand them back to blind CMA on a dip. On the
RasFinder capture rf-bbs-2 an 11.5 retrain's TRN went from 4.6 to 33.4 dB
(`docs/v34_data_mode_rates.md`). Built clean in the new worktree
`/root/deploy-ecb14c36` (from `/root/v90modem`, main at ecb14c36), not in
`/root/vpcm-fix6`. The rf-bbs-2 replay was re-run on CT 117 before install
(copy in `/tmp/rf-bbs-2`). It matched the Mac: TRN 33.3/33.4 dB, data at
24000, 0.034-0.042 from the grid. Binary md5 cd60dc75…, kept as
`/opt/v90ras/sip_v90_modem.ecb14c36`; the previous binary (9ca245ef,
6cf703d1…) is `sip_v90_modem.pre-ecb14c36`. Installed by rename, and only
`v90ras` was restarted, with no call up; the lines registered. `v90isdn`
shares the path. It was restarted at 15:17 with no PPP link up, and all
three of its processes run cd60dc75…; both lines are ready. Not live-verified: no call has met an 11.5 retrain on this
build.

## Deployed 5640b4d3 (2026-11-18)

V.34 11.6 without TRN: as initiator MP goes straight after S-bar; as
responder our TRN ends when the initiator sent none; the Phase 4 TRN SNR
ring is emptied per Phase 4; the bundle-loss 11.6 backs off before it
starts (`docs/v34_data_mode_rates.md`, "3429 baud vs slmodemd, the Diva and
the USR Courier"). Built in `/root/deploy-5640b4d3`; binary md5 b6cce84b…,
kept as `/opt/v90ras/sip_v90_modem.5640b4d3`; previous (ecb14c36, cd60dc75…)
is `sip_v90_modem.pre-5640b4d3`. Installed by rename with no call up;
`v90ras` and `v90isdn` restarted with no PPP interface up. The bundle-loss
11.6 path is new live behaviour here: watch for 11.6s that end in hang-up.

## Deployed fcdd1e78 (2026-11-18)

Two receiver fixes found against the USR Courier
(`docs/v34_data_mode_rates.md`, sections from "Why the Courier's 31200 went
white" on):
- The data-mode M&M clock loop keeps its learned frequency across 11.6/11.5
  re-entries (8779bd17).
- The line echo canceller tracker predicts only transmitted samples, and a
  move must be confirmed in a second window (fcdd1e78). Below a 150 ms bulk
  delay the tracker had been blind and moved on noise. Real moves are now
  followed about 1 s later, which matters for V.90 calls here.

Built in `/root/deploy-5640b4d3` (checked out at fcdd1e78). The binary (md5
ec1433ff…) is kept as `/opt/v90ras/sip_v90_modem.fcdd1e78`; the previous
binary (5640b4d3, b6cce84b…) is `sip_v90_modem.pre-fcdd1e78`. Installed by
rename with no call or PPP interface up, then `v90ras` and `v90isdn` were
restarted.

## Deployed d793be41 (2026-11-18)

Plain-V.34 Phase 3 now ends PP conditioning where PP ends instead of training
into TRN (`docs/v34_data_mode_rates.md`, "The Courier's Phase 3 stall after
our J"). V.90 Phase 3 is untouched. The binary (md5 3dbc7e8d…) is kept as
`/opt/v90ras/sip_v90_modem.d793be41`; the previous binary (fcdd1e78) is
`sip_v90_modem.pre-d793be41`. Installed with no call or PPP interface up, then
`v90ras` and `v90isdn` were restarted.

## Deployed 876d0ed6 (2026-11-19)

V.91 receive fix d68fb11e: at the robbed position found by the DIL the
receiver clears the LSB before demapping, so a robbed-bit trunk that writes a
1 there no longer fails B1 (`docs/v91_conformance_audit.md`, "Robbed LSB
written as 1"). No effect on the V.90/V.34 calls this server normally takes:
its paths have no robbed bits. 876d0ed6 itself only changes docs, the knob
generator and comments. Built in `/root/deploy-5640b4d3` (checked out at
876d0ed6). The binary (md5 f4cda747…) is kept as
`/opt/v90ras/sip_v90_modem.876d0ed6`; the previous binary (d793be41,
3dbc7e8d…) is `sip_v90_modem.pre-876d0ed6`. Installed by rename with no call
or PPP interface up, then `v90ras` and `v90isdn` were restarted; all ten
lines re-registered.

## Deployed 20742534 (2026-11-19)

The robbed-LSB fix moved from `modem_engine.c` into the shared `v91.c`
decoders: the LSB is cleared in every interval whose CP constellation is
odd-only (`docs/v91_conformance_audit.md`, "Robbed LSB written as 1"). Same
effect on a live call, and still none on the V.90/V.34 calls this server
takes. Built in `/root/deploy-5640b4d3` (checked out at 20742534). The binary
(md5 2dd8beb4…) is kept as `/opt/v90ras/sip_v90_modem.20742534`; the previous
binary (876d0ed6, f4cda747…) is `sip_v90_modem.pre-20742534`. Installed by
rename with no call or PPP interface up, then `v90ras` and `v90isdn` were
restarted; all ten lines re-registered.

## Deployed 83abe5a4 (2026-11-19)

Noise robustness for both analogue roles plus the digital side's V.92 PCM
upstream (`docs/v90_analogue_role.md`, "Downstream decision under loop
noise"; `docs/v92_quick_connect.md`, "Precoded upstream margin raised to
4σ"). Only the last affects calls this server answers: V.92 callers'
PCM-upstream CPd is now designed at 4σ, not 3σ. Against slmodemd on the rig
(3 alternated pairs) the upstream went 36000 → 34666, with 0 frames rejected
on both arms and the same traffic. Built in `/root/deploy-5640b4d3` (checked
out at 83abe5a4). The binary (md5 5b506c84…) is kept as
`/opt/v90ras/sip_v90_modem.83abe5a4`; the previous binary (20742534,
2dd8beb4…) is `sip_v90_modem.pre-83abe5a4`. Installed by rename with no call
or PPP interface up, then `v90ras` and `v90isdn` were restarted; both are
active and the lines re-registered. The A/B build trees `/root/margin-a` and
`/root/margin-b` and the script `/root/margin_ab.sh` (on vmhost) remain.

## Deployed 5a42a57a (2026-11-19 15:00)

V.92 9.8 rate renegotiation started by the digital side, precoder
coefficients only where the feedback equaliser gains, and the precoded
PCM-upstream margin back at 3σ (`docs/v92_quick_connect.md`, "V.92 9.8 rate
renegotiation, and the precoder only where it gains"). For V.92 callers this
server answers: against slmodemd on the rig (3 alternated pairs) the upstream
went 34666 → 36000, with 0 frames rejected on both arms and no
renegotiation needed. Built in `/root/deploy-5640b4d3` (checked out at
5a42a57a). The binary (md5 33d5792b…) is kept as
`/opt/v90ras/sip_v90_modem.5a42a57a`; the previous binary (83abe5a4,
5b506c84…) is `sip_v90_modem.pre-5a42a57a`. Installed by rename with no call
or PPP interface up, then `v90ras` and `v90isdn` were restarted; both are
active and all ten v90ras lines re-registered.

## Deployed 96d7d278 (2026-11-19 15:48)

Two analogue-role changes, both offline-verified only:
- The V.92 9.6.2/9.8 CPu is designed on codeword flips (3bdbde9b).
- The Phase 3 CP is planned on the codec's decision thresholds, with the
  zero Ucode's RMS as the noise floor instead of its folded spread, which
  read 0.60σ (96d7d278; `docs/v90_constellation_selection.md`, "Behind a
  codec").

This server is the digital side for its callers, so the analogue-role code
only runs when it dials out as the analogue modem. The noise-floor fix is in
the shared DIL planner. Built in `/root/deploy-5640b4d3` (checked out at
96d7d278). The binary (md5 f70d7543…) is kept as
`/opt/v90ras/sip_v90_modem.96d7d278`, and the previous binary (5a42a57a,
33d5792b…) is `sip_v90_modem.pre-96d7d278`. Installed by rename with no call
or PPP interface up, then `v90ras` and `v90isdn` were restarted. Both are
active and all ten v90ras lines re-registered.

## Deployed fa4c9779 (2026-11-19 19:27)

The V.90 analogue role now redesigns its §9.6 renegotiation CP on codeword
flips (`docs/v90_analogue_role.md`, "§9.6 renegotiation CP redesigned on
codeword flips"). This server only runs that code when it is the analogue
modem; answering callers as the digital side is unchanged. Offline-verified
only. Built in `/root/deploy-5640b4d3` (checked out at fa4c9779). The binary
(md5 728f14dd…) is kept as `/opt/v90ras/sip_v90_modem.fa4c9779`, and the
previous binary (96d7d278, f70d7543…) is `sip_v90_modem.pre-fa4c9779`.
Installed by rename with no call or PPP interface up, then `v90ras` and
`v90isdn` were restarted. Both are active and all ten v90ras lines
re-registered.

## Deployed d5788dae (2026-11-19 19:41)

Analogue-initiated V.90 §9.6 rate renegotiation now completes. The digital
modem's Rd answer is handled per §9.6.2.1.4 instead of restarting our
transmitter (`docs/v90_analogue_role.md`, "Analogue-initiated §9.6 fixed").
This only matters when this server is the analogue modem and has
`ME_V90_ANALOGUE_RATE_RENEGOTIATE_MS` set; it is not set on v90ras.
Offline-verified only. Built in `/root/deploy-5640b4d3` (checked out at
d5788dae). The binary (md5 fb5d4393…) is kept as
`/opt/v90ras/sip_v90_modem.d5788dae`, and the previous binary (fa4c9779,
728f14dd…) is `sip_v90_modem.pre-d5788dae`. Installed by rename with no call
or PPP interface up, then `v90ras` and `v90isdn` were restarted. Both are
active and all ten v90ras lines re-registered.

## Deployed 95e15ee4 (2026-11-19 22:40)

The V.90 upstream B1 fit now leaves out up to one mapping frame of symbols
the wire disagrees with. A Diva card's B1 after a digital-initiated §9.6
differs in its final 4D symbol, and that had bent the whole fit
(`docs/retrain_and_resync.md`, "Cause and fix"). This runs on every V.90
upstream acquisition, so it covers callers' own §9.6 too. Live-verified on
Diva controllers 7 and 8 with a one-off server before the deploy: both
renegotiations completed with no retrain. Built in `/root/deploy-5640b4d3`
(checked out at 95e15ee4). The binary (md5 6dc3d644…) is kept as
`/opt/v90ras/sip_v90_modem.95e15ee4`, and the previous binary (d5788dae,
fb5d4393…) is `sip_v90_modem.pre-95e15ee4`. Installed by rename with no call
or PPP interface up, then `v90ras` and `v90isdn` were restarted. Both are
active and the lines re-registered.

## Deployed 567ae54d (2026-11-19)

After a V.90 §9.6 rate renegotiation the engine now passes the new downstream
(and upstream) rate to LAPM (`ds_set_line_bit_rate()`, `[V42STAT] rate=`),
the CONNECT rate and +MRR. Before this they kept the startup rate
(`docs/retrain_and_resync.md`). Built in `/root/deploy-5640b4d3` (checked out
at 567ae54d). The binary (md5 4ca0dc8d…) is kept as
`/opt/v90ras/sip_v90_modem.567ae54d`, and the previous binary (95e15ee4,
6dc3d644…) is `sip_v90_modem.pre-567ae54d`. Installed by rename with no call
or PPP interface up (`/run/v90ras/busy` is a directory, so check that it is
empty). Then `v90ras` and `v90isdn` were restarted, and both are active.

## Deployed 9d94ec3d (2026-11-20)

A §9.6 renegotiation that the peer opens now gets §9.6.1's full E window.
Before this, the 4 s unanswered-attempt bound retrained a Diva's
echo-reconditioned renegotiation 0.6 s before its B1 arrived. Built in
`/root/deploy-5640b4d3` (checked out at 9d94ec3d). The binary (md5
fddf7000…) is kept as `/opt/v90ras/sip_v90_modem.9d94ec3d`, and the previous
binary (567ae54d, 4ca0dc8d…) is `sip_v90_modem.pre-9d94ec3d`. Installed by
rename with `/run/v90ras/busy` empty and no PPP interface up. Then `v90ras`
and `v90isdn` were restarted, and both are active.

## ISDN callers in A-law (2026-11-20)

ISDN calls reach Asterisk through the Cisco (endpoint 6501), whose SDP
offers PCMU before PCMA, so v90ras answered them μ-law and the Cisco
requantized the A-law B channel. That cost the Diva V.90 upstream 115-150
shell errors a call. Three changes, none in the binary:

- `pjsip.conf`: new template `[endpoint-modem-dual](!,endpoint-modem)`
  (`allow=!all,clearmode,ulaw,alaw`, `codec_prefs_outgoing_offer=prefer:pending,operation:intersect,keep:all,transcode:allow`),
  used by 2951-2960. Backup `pjsip.conf.before-v90ras-dual-law-20261120`.
- `extensions.conf`: the three v90ras Dials carry
  `b(v90ras-law^s^1(${CHANNEL(endpoint)},${CALLERID(num)}))`. `[v90ras-law]`
  sets `PJSIP_MEDIA_OFFER(audio)=alaw` for 6501 callers numbered 79XX (BRI);
  FXS 78XX callers are left alone. Backup
  `extensions.conf.before-v90ras-alaw-20261120`.
- CT 117 drop-in `/etc/systemd/system/v90ras.service.d/info0d-b39.conf`:
  `V90_INFO0D_B39=0` (also in `v90ras.service`). The Diva trains V.90 on
  A-law only when we declare μ-law. μ-law calls already send 0. (2026-11-23:
  the Cisco leg stays PCMU even with the handler, so the call crosses an
  A-μ-A round trip. The card fails V.90 against itself across that same hop,
  so the drop-in works around the route, not our A-law. See
  `docs/v90_analogue_role.md`.) Since 2026-11-23 Asterisk endpoint `6501-bri` answers
  79XX callers in A-law, so the path is transparent. On it a truthful bit 39
  gets 56000 (3/3) and this drop-in gets 37333. The drop-in is now the thing
  holding the Diva back. Removed the same day (kept as
  `/root/info0d-b39.conf.removed-20261123` on CT 117). Production 2950 as
  7911 then gave CONNECT 56000, PCMA, 0 shell errors of 19981. 10-minute PPP
  soaks into 2950 (ctrl 7, then ctrl 8, 600 s holds): both held 56000 for the
  whole 600 s, with 0 retrains or renegotiations, 0 upstream shell errors of
  247981 each, 581/582 and 578/578 pings intact, 0 corrupt, 0 bad FCS, and
  RTT median 0.34 s.

Live result: 5/5 Diva calls (ctrl 7 and 8) at 38666/31200 with 0 shell
errors, against 41333/31200 with 115-150 on μ-law. Details and the
downstream cost: `docs/v90_upstream_data_path.md`. To revert, restore the
two Asterisk backups (`module reload res_pjsip.so`, `dialplan reload`) and
remove the drop-in.

## Deployed 284585ab (2026-11-23)

The receiver decodes a μ-law leg that carries a transcoded A-law source by
the A-law values (`docs/v90_upstream_data_path.md`, "Undoing the double
companding in the receiver"). Built in `/root/deploy-5640b4d3` (checked out
at 284585ab). The binary (md5 b64e2960…) is kept as
`/opt/v90ras/sip_v90_modem.284585ab`, and the previous binary (9d94ec3d,
fddf7000…) is `sip_v90_modem.pre-284585ab`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. Then `v90ras` and
`v90isdn` were restarted, and both are active. Smoke call: the Diva as 7811
(μ-law) gets 41333/31200 with 0 upstream shell errors (136-141 before).

## Deployed 3be20424 (2026-11-23 20:57)

On top of 284585ab, this deploy adds the V.92 retrain fixes:
- 308c99ab: Sd delay 250 ms and Jp 38969c on retrained V.92 attempts.
- 3be20424: the B1u solve leaves out off-model precoder targets, and a B1u
  that has not locked 5 s after the CPd acknowledgement starts a retrain.

Both were verified against slmodemd on the Proxmox rig, not on v90ras
callers. See `docs/v92_full_startup.md`.

The deploy also carries the D-Modem interop fixes, which have no v90ras
coverage yet:
- 58f71d3b (V.91 startup) and 03801c48 (V.34).
- The opt-in x2 startup (25bf4378), which stays off by default.

Built in `/root/deploy-5640b4d3`, checked out at 3be20424. The binary (md5
11d94652…) is kept as `/opt/v90ras/sip_v90_modem.3be20424`, and the previous
one (284585ab) as `sip_v90_modem.pre-3be20424`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. `v90ras` and `v90isdn` were
restarted, and all ten lines re-registered (2951-2960 Avail).

## Deployed 36a574cd (2026-11-24 08:39)

On top of 3be20424, this deploy adds 36a574cd: the V.90 §9.2.2.2.2 Tone A
re-reversal in the analogue role, and the `V90_TEST_DROP_INFO0D` test
hook (off unless set). The v90ras and v90isdn servers run the digital role,
so their calls take the same path as before; the deploy keeps CT 117 on
`main`. No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 36a574cd. The binary (md5
e5e9e73b…) is kept as `/opt/v90ras/sip_v90_modem.36a574cd`, and the previous
one (3be20424, 11d94652…) as `sip_v90_modem.pre-36a574cd`. Installed by
rename with `/run/v90ras/busy` empty and no PPP interface up. `v90ras` and
`v90isdn` were restarted, and all ten lines re-registered (2951-2960).

## Deployed a580829a (2026-11-24)

On top of 2a827d51, this deploy adds a580829a. When data mode keeps the V.92
decision feedback (an unprecoded CPd), the feedback's peak gain is now held
at 0.9. On the dense data levels, ESS A-law's alternating feedback tail
(1.13 at fs/2) made the output run away before B1u could lock. Only calls
that took the unprecoded fallback are affected. Precoded peers that lock
never reach it. No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at a580829a
(`/root/deploy-a580829a.build.log`). The binary (md5 42693b52…) is kept as
`/opt/v90ras/sip_v90_modem.a580829a`, and the previous one (2a827d51,
680ba7a9…) as `sip_v90_modem.pre-a580829a`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. `v90ras` and `v90isdn` were
restarted, and all ten lines re-registered (2951-2960).

## Deployed 2a827d51 (2026-11-24)

On top of 04daa5a2, this deploy adds 2a827d51. Two V.92 PCM upstream
changes:
- A precoded B1u that misses its deadline makes the retrain send an
  unprecoded CPd (explicit identity coefficient section), with decision
  feedback kept in data mode.
- Data mode answers the analogue modem's own §9.8 Ru with Rd (§9.8.1.2)
  instead of ignoring it until the peer retrains.

Both were found against D-Modem's ESS peer. On the D-Modem matrix the
SmartLink precoded rows and the V.90 rows were unchanged. Precoded peers that
lock (slmodemd-class) never take the fallback. The Ru watcher fires only on
48 symbols of the R sign pattern. No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 2a827d51
(`/root/deploy-2a827d51.build.log`). The binary (md5 680ba7a9…) is kept as
`/opt/v90ras/sip_v90_modem.2a827d51`, and the previous one (04daa5a2,
4c2c9ae5…) as `sip_v90_modem.pre-2a827d51`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. `v90ras` and `v90isdn` were
restarted, and all ten lines re-registered (2951-2960).

## Deployed 04daa5a2 (2026-11-24 10:59)

On top of a6737e79, this deploy adds 04daa5a2: in V.92 Phase 4 a CPd' now
counts as an acknowledged sequence for §9.6.1.1.4. An analogue modem that
acks after the §9.6.1.1.3 window, so that we are repeating CPd', now gets Ed;
before, CPd' repeated until the B1u deadline. That was found against
D-Modem's ESS peer; SmartLink-class peers ack inside the window and are
unaffected (D-Modem matrix verdicts unchanged). No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 04daa5a2. The binary (md5
4c2c9ae5…) is kept as `/opt/v90ras/sip_v90_modem.04daa5a2`, and the previous
one (a6737e79, b34fbfff…) as `sip_v90_modem.pre-04daa5a2`. Installed by
rename with `/run/v90ras/busy` empty and no PPP interface up. `v90ras` and
`v90isdn` were restarted, and all ten lines re-registered (2951-2960).

## Deployed a6737e79 (2026-11-24 09:26)

On top of 36a574cd, this deploy adds a6737e79: V.92 PCM upstream now runs
the §6.4.4 Viterbi decoder (per-frame and streaming) for precoded CPds,
stripping the §6.4.2 feedback from the observed x(n) first, where it used
to slice symbol by symbol. Correction: it does not change live decoding.
Live precoded calls arm the receiver with the coefficients stripped (the
channel removes the feedback ahead of the equaliser), and a slmodemd A/B
(3 calls a side) was identical. Offline, the precoded spec-peer BER at sigma 8 DS0 went from
3.8e-1 to 0 (`docs/v92_pcm_upstream_findings.md`). No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at a6737e79. The binary (md5
b34fbfff…) is kept as `/opt/v90ras/sip_v90_modem.a6737e79`, and the previous
one (36a574cd, e5e9e73b…) as `sip_v90_modem.pre-a6737e79`. Installed by
rename with `/run/v90ras/busy` empty and no PPP interface up. `v90ras` and
`v90isdn` were restarted, and all ten lines re-registered (2951-2960).

## Deployed aa6fcd24 (2026-11-24)

On top of a580829a, this deploy adds aa6fcd24. In V.92 Phase 3, Jp now asks
for a Su extension (Table 22 bits 18:33), measured on Su and rounded to 0 or
T/2, where it always asked for none. The network A/D then samples the
analogue modem's pulse near its peak. Offline, ESS's channel became one tap
and its upstream went from about 30000 to 37333. Calls whose channel leaves
the precoder nothing to do now get an explicit identity coefficient section,
and keep decision feedback in data mode. Every V.92 PCM-upstream caller sees
the new Jp. No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at aa6fcd24
(`/root/deploy-aa6fcd24.build.log`). The binary (md5 26f8d330…) is kept as
`/opt/v90ras/sip_v90_modem.aa6fcd24`, and the previous one (a580829a,
42693b52…) as `sip_v90_modem.pre-aa6fcd24`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. `v90ras` and `v90isdn` were
restarted, and all ten lines re-registered (2951-2960).

## Deployed 602d76f9 (2026-11-24)

On top of aa6fcd24, this deploy adds 602d76f9, which affects V.92 Phase 3
only. When the equaliser's taps are a split-pulse inverse at the second
TRN1u, it moves the sampling instant to that TRN1u's measured pulse. It then
solves the taps there by least squares, with the timing loop held open until
that refit finishes. Offline this ended ESS μ-law's first-B1u miss (B1u locked
first time in 16 of 16 ESS calls). Calls whose taps aren't split (largest
neighbour < 0.35 of the main tap) are unchanged. No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 602d76f9
(`/root/deploy-602d76f9.build.log`). The binary (md5 0eeaee69…) is kept as
`/opt/v90ras/sip_v90_modem.602d76f9`, and the previous one (aa6fcd24,
26f8d330…) as `sip_v90_modem.pre-602d76f9`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. `v90ras` and `v90isdn` were
restarted, and all ten lines re-registered (2951-2960).

## Deployed 87a4827d (2026-11-24)

On top of 602d76f9, this deploy adds 87a4827d, which affects V.92 PCM upstream
data mode and the §9.8 renegotiations only. The held equaliser now corrects
its frequency each time a least-squares solve lands, by the drift the timing
detector measured across that solve's window. Offline against ESS, a
renegotiation's TRN2u had been up to 153 DS0 because the frequency froze up to
1.7 ppm off at data start; it is now 57-61 DS0 (15 renegotiations, 10 of 10
calls). V.90 and V.34 calls are unchanged. No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 87a4827d
(`/root/deploy-87a4827d.build.log`). The binary (md5 8df61775…) is kept as
`/opt/v90ras/sip_v90_modem.87a4827d`, and the previous one (602d76f9,
0eeaee69…) as `sip_v90_modem.pre-87a4827d`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. `v90ras` and `v90isdn` were
restarted, and all ten lines re-registered (2951-2960).

## Deployed 8b95d16e (2026-11-24)

On top of 87a4827d, this deploy adds 8b95d16e, which affects V.90 Phase 3
(§9.3.2.7) only. An analogue modem's S during our Jd is no longer thrown away
as "own-line noise" for being under 0.15 of our transmit RMS when the line
watch or the 64-symbol alternation check confirmed it. Offline, ESS's real S
came in at 0.125-0.148 and we sat in Jd until ESS retrained; ESS's V.90 row
went from failing to 8 of 8. Calls whose S already cleared the ratio are
unchanged. No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 8b95d16e
(`/root/deploy-8b95d16e.build.log`). The binary (md5 d4ef351e…) is kept as
`/opt/v90ras/sip_v90_modem.8b95d16e`, and the previous one (87a4827d,
8df61775…) as `sip_v90_modem.pre-8b95d16e`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. `v90ras` and `v90isdn` were
restarted, and the lines re-registered (registration success, both services
active).

## Deployed 6b368e4e (2026-11-25)

On top of 8b95d16e, this deploy adds 6b368e4e, which affects V.90 Phase 2
(§9.2.1.2.6) only. After INFO1d we no longer re-send it after 200 ms of
silence; a silence re-send now waits for the 700 ms + round-trip deadline.
Offline, the Agere, HSF and Broadcom soft modems go quiet for about 210 ms
before INFO1a and our early re-send landed on top of it. Peers that answer
INFO1d straight away are unchanged. No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 6b368e4e
(`/root/deploy-6b368e4e.build.log`). The binary (md5 8c1cd594…) is kept as
`/opt/v90ras/sip_v90_modem.6b368e4e`, and the previous one (8b95d16e,
d4ef351e…) as `sip_v90_modem.pre-6b368e4e`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. `v90ras` and `v90isdn` were
restarted, and the lines re-registered (registration success, both services
active).

## Deployed cb6b73e6 (2026-11-25)

On top of 6b368e4e, this deploy adds cb6b73e6, which affects V.90/V.92 Phase 2
(§9.2.1.1.7) on the digital side only. L2 used to stop after a fixed 300 ms
and wait for Tone A in silence; it now runs on until Tone A, up to 550 ms plus
a round trip, and then falls back to that wait. Offline, the Agere soft modem
measured our L2 as silence and declared the line RBS; Broadcom V.92 now
connects first time. Peers that raise Tone A within 300 ms see no change.
`ME_V90_L2_UNTIL_TONE_A=0` restores the old L2. No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at cb6b73e6
(`/root/deploy-cb6b73e6.build.log`). The binary (md5 0b6cd297…) is kept as
`/opt/v90ras/sip_v90_modem.cb6b73e6`, and the previous one (6b368e4e,
8c1cd594…) as `sip_v90_modem.pre-cb6b73e6`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. `v90ras` and `v90isdn` were
restarted, and the lines re-registered (registration success, both services
active).

## Deployed 7ae2f08c (2026-11-25)

On top of cb6b73e6, this deploy adds 7ae2f08c, which affects V.90 Phase 4 CP
acceptance (§5.4.5.6, §9.4.1.1). A CPt or CP whose spectral-shaping zero b1 or
b2 lies past unity is accepted; Agere puts b2 at −65/64. Only the poles a1 and
a2 are still held to |a| ≤ 1. Offline on CT 128, Agere V.90 now connects at
53333 instead of falling back to V.34. Peers whose coefficients were already
within range see no change. No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 7ae2f08c
(`/root/deploy-7ae2f08c.build.log`). The binary (md5 1c51f394…) is kept as
`/opt/v90ras/sip_v90_modem.7ae2f08c`, and the previous one (cb6b73e6,
0b6cd297…) as `sip_v90_modem.pre-7ae2f08c`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. `v90ras` and `v90isdn` were
restarted, and the lines re-registered (registration success, both services
active).

## Deployed 5589721f (2026-11-25)

On top of 7ae2f08c, this deploy adds 5589721f, which affects V.90 Phase 4 CP
framing (Table 14, §9.4.2.2-.4).
- `v90_cp_rx` now drops a false sync as soon as a Table 14 start bit is one,
  and hunts again through the bits that frame had collected.
- When that hunt finds a sync, it counts the false frame as superseded, not
  rejected, so the CP tap's bit mapping is no longer unlocked just before a
  real CP.
- Offline on CT 128, Agere V.90/V.92 and Broadcom V.90 connected PCM on the
  first attempt in 12 of 12 calls.
- No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 5589721f
(`/root/deploy-5589721f.build.log`). The binary (md5 b49f8400…) is kept as
`/opt/v90ras/sip_v90_modem.5589721f`, and the previous one (7ae2f08c,
1c51f394…) as `sip_v90_modem.pre-5589721f`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up.

Asterisk had restarted on its own at about 10:54, a minute before this
install, and had lost every endpoint-modem endpoint (2951 "Unable to find
object"). The first restart of `v90ras` therefore failed registration on all
ten lines with PJSIP_EFAILEDCREDENTIAL, and the lines did not retry.
- `asterisk -rx "module reload res_pjsip.so"` brought the endpoints back
  (96 loaded).
- A second restart of `v90ras` and `v90isdn` registered 2951-2960 (status
  200), and 2951 shows "Not in use".
- The binary was not the cause: the 10:37 restart of 7ae2f08c registered
  normally.

## Deployed 221960e2 (2026-11-25)

On top of 5589721f, this deploy adds 221960e2, which affects V.90 Phase 3 Ja
recognition (Table 12, §9.3.1.3).
- A Ja that matches the CX93001 template but carries a clean Table 12
  header naming a different descriptor (3 or more of the N/Lsp/Ltp field
  bits differ) is no longer answered with the CX's DIL. We wait for its
  CRC-valid descriptor instead.
- Offline on CT 128, the Conexant HSF now gets its own DIL (N=147, not the
  CX's 119). The CX93001 fixture still matches at 384/384.
- No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 221960e2
(`/root/deploy-221960e2.build.log`). The binary (md5 83720859…) is kept as
`/opt/v90ras/sip_v90_modem.221960e2`, and the previous one (5589721f,
b49f8400…) as `sip_v90_modem.pre-221960e2`. Installed by rename with
`/run/v90ras/busy` empty (it is a directory; test it with `ls -A`, not
`test -s`) and no PPP interface up. `v90ras` registered 2951-2960, and
`v90isdn` came up ready.

## Deployed 58655519 (2026-11-25)

On top of 221960e2, this deploy adds 58655519, which affects V.90 Phase 2:
the digital modem's Tone B reversal (§9.2.1.1.6).
- The handover from reversed Tone B to PCM L1 now waits (9−1)/2 bauds for the
  CC shaper's tail, then switches at that exact sample. Before, it waited for
  the end of the 20 ms `v34_tx()` block. That sent 10–30 ms of reversed tone,
  or none at all when the reversal baud fell within about 6.7 ms of the
  block's end.
- Offline on CT 128, the HSF gets INFO1a on the first Phase 2 in 9 of 9
  calls (was 1 of 6). The D-Modem matrix is unchanged at 14 of 18.
- No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 58655519
(`/root/deploy-58655519.build.log`). The binary (md5 8328d662…) is kept as
`/opt/v90ras/sip_v90_modem.58655519`, and the previous one (221960e2,
83720859…) as `sip_v90_modem.pre-58655519`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up.

Asterisk had restarted on its own about a minute before the install, again
losing every endpoint-modem endpoint ("Unable to find object 2951"), so the
first restart failed registration with PJSIP_EFAILEDCREDENTIAL.
- `asterisk -rx "module reload res_pjsip.so"` brought back 96 endpoints.
- A second restart of `v90ras` and `v90isdn` registered 2951-2960, all
  "Not in use" and Avail. v90isdn logged "SIP V.90 modem ready".

## Deployed 2c81a62c (2026-11-25)

On top of 58655519, this deploy adds 2c81a62c, which affects V.34/V.90
Phase 2: the Tone B reversal (§11.2.1.1.6, §9.2.1.1.6).
- The SECOND_NOT_B CC tail covers the plain V.34 call modem too, not only
  the V.90 digital modem.
- The tail gains one baud, so the reversed Tone B lasts 10 ms on the wire:
  9.8 ms measured, where it was 8.2 ms with 58655519's tail. The reversal
  instant is unchanged.
- Offline: HSF V.90 first-Phase-2 INFO1a 6/6. D-Modem matrix 14 of 18, data
  exact. V.34 duplex symbol-rate matrix 12/12.
- No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 2c81a62c
(`/root/deploy-2c81a62c.build.log`). The binary (md5 81a4bd9d…) is kept as
`/opt/v90ras/sip_v90_modem.2c81a62c`, and the previous one (58655519,
8328d662…) as `sip_v90_modem.pre-2c81a62c`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. Asterisk's endpoints were
present this time (2951 "Not in use"). The restart logged 20 v90ras
"registration success" lines, and v90isdn logged "SIP V.90 modem ready".

## Deployed 9e7f5cf4 (2026-11-25)

On top of 2c81a62c, this deploy adds 9e7f5cf4, which affects the INFO and
tone receiver used in every Phase 2 (V.34 §10.1.2.3.1).
- A reversal is now a phase jump of over 90 degrees across three samples, not
  two. In noise the two-sample test could miss a reversal, and the 10 dB
  `v34_duplex_test` caller lost INFO1a's sync word that way.
- Offline: INFO1a decodes at all 12 delays of the 10 dB sweep (10/12 pass,
  the other two fail later, at MP). The `v56_loopback_test` echo case passes
  12/12, as before. `test-fast` 775/775. HSF first-Phase-2 INFO1a 6/6.
  D-Modem matrix 14 of 18, data exact.
- No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 9e7f5cf4
(`/root/deploy-9e7f5cf4.build.log`). The binary (md5 1290652b…) is kept as
`/opt/v90ras/sip_v90_modem.9e7f5cf4`, and the previous one (2c81a62c,
81a4bd9d…) as `sip_v90_modem.pre-9e7f5cf4`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. Asterisk's endpoints were
present. The restart logged 20 v90ras "registration success" lines, and
v90isdn logged "SIP V.90 modem ready".

## Deployed 669bf72c (2026-11-27)

On top of d8963a70 (and its README commit 308a8a89), this deploy adds 669bf72c.
- **V.32bis call modem:**
  - When R3 names several rates (the Courier's 0x9ff0), it picks the fastest
    common rate. Before, it never sent E.
  - It answers a 7.2 retrain (AC) during start-up once a rate signal has
    arrived.
- **V.32bis, both roles:** data whose decision error stays above 0.45 rms for
  2 s retrains (7.1/7.2), at most 3 times a call.
- **V.22bis call modem:**
  - Its S1 is no longer cut to 75 ms.
  - It searches the other T/2 eye for the answer modem's S1.
- **Offline.**
  - We call D-Modem's emulated Courier: V.32bis 32/32 at 14400 and V.22bis
    24/24 at 2400, data exact. Before, V.32bis never connected and V.22bis
    came up at 1200.
  - The Courier calling us: 8/8 per modulation.
  - `test-fast` 783/783 plus the new V.22bis fixture.
  - `test-slow` failures are the courier-emu x2 probes only. Three of them
    pass on a rerun one at a time. `slow.list:67` was already failing: its
    probe passes `--serial-output`, which the pinned courier-emu f1ccd92
    rejects.
- **No smoke call was made.** v90ras answers, so the call-modem changes don't
  apply here. Answer-side calls change only by the V.32bis lost-reception
  retrain.

Built in `/root/deploy-5640b4d3`, checked out at 669bf72c
(`/root/deploy-669bf72c.build.log`; all of spandsp and the engine objects
that include its private headers rebuilt). The binary (md5 83ce6bc1…) is
kept as `/opt/v90ras/sip_v90_modem.669bf72c`, and the previous one (d8963a70,
de0c0a56…) as `sip_v90_modem.pre-669bf72c`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. The restart logged 20
"registration success" lines and 10 "SIP V.90 modem ready".

## Deployed d8963a70 (2026-11-27)

On top of fdd95bdb, this deploy adds d8963a70.
- **V.32bis S acquisition at either T/2 phase.** S alternates A and B, so at
  the eye crossing it fits half way and the timing loop took ~150 symbols to
  reach the eye centre. The search now also fits the other T/2 phase and
  moves the sampling point there when it fits better.
- **V.32bis answer modem restarts on the far end's AA during start-up**
  (7.1), once its tone phase is over: a tone at 0.9 of a pure tone, within
  6 dB of the tone phase's AA. Before, a stalled start-up sat silent.
- **Offline.** D-Modem's emulated Courier calling in V.32bis: 32/32 against
  31/32 on fdd95bdb (arms alternated), then 24/24; forced stalls
  (`V32BIS_TEST_DROP_FIRST_R=1`, not set here) 2/2 retrain and connect.
  `test-fast` 783/783. See `docs/dmodem_interop.md`, "The emulated Courier".
- **No smoke call was made.** Only V.32bis calls change.

Built in `/root/deploy-5640b4d3`, checked out at d8963a70
(`/root/deploy-d8963a70.build.log`). The binary (md5 de0c0a56…) is kept as
`/opt/v90ras/sip_v90_modem.d8963a70`, and the previous one (fdd95bdb,
a79624cd…) as `sip_v90_modem.pre-d8963a70`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. The restart logged 20
"registration success" lines and 10 "SIP V.90 modem ready".

## Deployed fdd95bdb (2026-11-27)

On top of 21da3dfa, this deploy adds fdd95bdb (and the docs-only 408ab20e).
- **V.32bis answer tone dialogue (6.2).** A tone phase entered from the
  receive path repeated its first state, so after an odd AC count our CA
  carried no phase reversal and a call modem waiting for one (6.1) sent AA
  until it gave up. CA now reverses, and AC to CA waits for an even count of
  AC symbols sent. See `docs/dmodem_interop.md`, "The emulated Courier".
- **Offline.**
  - D-Modem's emulated Courier calling in V.32bis: 9 of 12 connect 14400 with
    data exact, 0 before; the other three miss its R2 (open).
  - V.32bis rows of `tests/fast.list` 21/21; `test-fast` 783/783.
- **No smoke call was made.** Only V.32bis answer calls change; watch the
  first live ones.

Built in `/root/deploy-5640b4d3`, checked out at fdd95bdb
(`/root/deploy-fdd95bdb.build.log`). The binary (md5 a79624cd…) is kept as
`/opt/v90ras/sip_v90_modem.fdd95bdb`, and the previous one (21da3dfa,
487b6bfe…) as `sip_v90_modem.pre-fdd95bdb`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. The restart logged 20
"registration success" lines and 10 "SIP V.90 modem ready".

## Deployed 21da3dfa (2026-11-26)

On top of 52ccd4d0, this deploy adds 21da3dfa.
- **V.90 upstream B1 search.**
  - B1 is now also fitted from the scrambler and trellis state the analogue
    modem carries into it, read off its CP′/E (V.34 10.1.3.1 says to reset
    both; the ESS resets neither).
  - A CP′-anchored 4-point search that finds no end yet is retried in the
    next window. The spec's reset state is one of its candidates, so peers
    that do reset still get an exact B1 there.
  - `ME_V90_T3_NO_CARRIED_B1=1` turns the new candidates off; it is not set
    here. See `docs/dmodem_interop.md`, "ESS V.90 carries its scrambler and
    trellis state into B1".
- **Offline.**
  - D-Modem `ess_v90_ulaw`: 8/8 against 7/8, with 17/19 B1s exact against
    20/20 blind; the final build 6/6.
  - Matrix 15/18, was 14/18.
  - `test-fast` 782 passed; the one failure was the `env_knobs` check, run
    before the regenerated file, which passes now.
- **No smoke call was made.** Peers whose B1 matched a template before
  should see no change. Watch the first live V.90 renegotiations.

Built clean in `/root/deploy-5640b4d3`, checked out at 21da3dfa
(`/root/deploy-21da3dfa.build.log`). The binary (md5 487b6bfe…) is kept as
`/opt/v90ras/sip_v90_modem.21da3dfa`, and the previous one (52ccd4d0) as
`sip_v90_modem.pre-21da3dfa`. Installed by rename with `/run/v90ras/busy`
empty and no PPP interface up. The restart logged 20 "registration success"
lines and "SIP V.90 modem ready".

## Deployed 52ccd4d0 (2026-11-26)

On top of 3dc003df, this deploy adds 52ccd4d0.
- **Plain V.34 answer calls.**
  - Phase 4 S/S̄ is now the V.34 10.1.3.7 sequence.
  - Our symbol clock runs on into Phase 4 instead of resetting.
  - `V34_DUPLEX_SPEC_S=0` and `V34_P4_KEEP_BAUD_PHASE=0` restore the old
    behaviour; neither is set here.
  - V.90 calls don't use this S.
- **Offline.**
  - D-Modem HSF calling V.34: 7/8 against 2/8.
  - Matrix unchanged.
  - `test-fast` 783/783, and `test-slow` passes.
- **No smoke call was made.** This changes what every V.34 caller hears
  from us, so watch the first live V.34 answers.

Built clean in `/root/deploy-5640b4d3`, checked out at 52ccd4d0
(`/root/deploy-52ccd4d0.build.log`). The binary (md5 c6b770c1…) is kept as
`/opt/v90ras/sip_v90_modem.52ccd4d0`, and the previous one (3dc003df) as
`sip_v90_modem.pre-52ccd4d0`. Installed by rename with `/run/v90ras/busy`
empty and no PPP interface up. The restart logged 20 v90ras "registration
success" lines, and v90isdn logged "SIP V.90 modem ready".

## Deployed 3dc003df (2026-11-26)

On top of be7665c2 (a2a7b851 is its deploy note), this deploy adds 3dc003df.
- **V.92 Phase 3 receiver.**
  - Ru2 now locks on whichever period-6 phase it arrives in after MD
    (V.92 9.5.1.1.1).
  - The Su detector's energy floor is 6 dB lower (9.5.2.1.7).
  - See `docs/dmodem_interop.md`, "V.92 Phase 3 against the HSF". Only
    V.92 calls from analogue modems whose MD shifts Ru's phase, or whose Su
    is quiet, change.
- **Offline.**
  - D-Modem HSF V.92 at −12 dBm0: 4/4 against 0/4. At the default −6 dBm0
    it still fails, on the HSF's own failure.
  - Matrix 16/18, no regressions.
  - `test-fast` 783/783.
- **No smoke call was made.**

Built clean in `/root/deploy-5640b4d3`, checked out at 3dc003df
(`/root/deploy-3dc003df.build.log`, which rebuilt `v92_p3_rx.o` and
`v92_su.o`). The binary (md5 e602bbb0…) is kept as
`/opt/v90ras/sip_v90_modem.3dc003df`, and the previous one (be7665c2) as
`sip_v90_modem.pre-3dc003df`. Installed by rename with `/run/v90ras/busy`
empty and no PPP interface up. The restart logged 20 v90ras "registration
success" lines, and v90isdn logged "SIP V.90 modem ready".

## Deployed be7665c2 (2026-11-26)

On top of 280f12fd (dce4885c and 3872849c in between are docs and an
off-by-default knob), this deploy adds be7665c2.
- **V.90-declined V.34 fallback.** The MP search keeps the V.34 descrambler
  defaults instead of rotating through V.90's CP retry modes
  (`docs/dmodem_interop.md`, "The fallback's MP receiver rotated away"). Only
  calls whose analogue modem declines PCM take this path.
- **Offline.**
  - D-Modem `hsf_v90_ulaw`: 4/4 against 0/4, alternated over four line delays.
  - Matrix 15/18 (was 14/18).
  - `test-fast` 783/783.
- **No smoke call was made.**

Built clean in `/root/deploy-5640b4d3`, checked out at be7665c2
(`/root/deploy-be7665c2.build.log`). The binary (md5 5a3cb888…) is kept as
`/opt/v90ras/sip_v90_modem.be7665c2`, and the previous one (280f12fd) as
`sip_v90_modem.pre-be7665c2`. Installed by rename with `/run/v90ras/busy`
empty and no PPP interface up. The restart logged 20 v90ras "registration
success" lines, and v90isdn logged "SIP V.90 modem ready".

## Deployed 280f12fd (2026-11-26)

On top of baf083cc, this deploy adds eea66c86 and 280f12fd.
- **Plain V.34 calls.** The training carrier loop's proportional gain goes
  from 40000 to 2000000. The loop was undamped and left Phase 3 0.7–2.8 Hz
  off nominal in a loopback with no offset (`docs/v34_carrier_loop.md`).
  V.90 calls, the V.90-declined V.34 fallback included, keep the old gain.
- **Opt-in only.** `V34_P4_ANSWER_J_FRAMING=1` (answerer J′ framing) is
  off here.
- **Offline.**
  - `test-fast` 783/783.
  - Delay sweep 234/240 against 229.
  - D-Modem matrix 13/18, the same five rows as before, with data exact on
    every connecting row.
- **No smoke call was made.**

Built clean in `/root/deploy-5640b4d3`, checked out at 280f12fd
(`/root/deploy-280f12fd.build.log`). The binary (md5 9d9b5a7b…) is kept as
`/opt/v90ras/sip_v90_modem.280f12fd`, and the previous one (baf083cc,
9e92769e…) as `sip_v90_modem.pre-280f12fd`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. The restart logged 20 v90ras
"registration success" lines, and v90isdn logged "SIP V.90 modem ready".

## Deployed baf083cc (2026-11-26)

On top of ce417b72, this deploy adds baf083cc. As a V.34 call modem, which
includes the V.90-declined fallback (V.90 §9.2.1.1.8), we now send one 8T J′
before Phase 4 TRN (V.34 §11.4.1.1.1, §10.1.3.4). Before, we sent 16T: J′
followed by 8T of scrambled zeros. v90ras answers, so this reaches its calls
only through the fallback; as answerer nothing changes.
- Offline: `test-fast` 783/783. Loopback pass rates are unchanged.
- D-Modem matrix 13/18. The extra ESS V.90 failure never reaches J′
  (`docs/dmodem_interop.md`).
- No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at baf083cc
(`/root/deploy-baf083cc.build.log`). The binary (md5 9e92769e…) is kept as
`/opt/v90ras/sip_v90_modem.baf083cc`, and the previous one (ce417b72,
b4fea629…) as `sip_v90_modem.pre-baf083cc`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. The restart logged 20 v90ras
"registration success" lines, and v90isdn logged "SIP V.90 modem ready".

## Deployed ce417b72 (2026-11-26)

On top of 5849d2e1, this deploy adds ce417b72. When a V.34 caller's INFO1c
announces an MD, the plain V.34 answerer now goes silent for the MD (V.34
§11.3.1.2.4), waits for the caller's second S-to-S̄, and trains on its PP and
TRN. Before, it kept sending J through the MD and went to Phase 4 on the
second S, ahead of the caller's PP, TRN and J. Its two opt-ins,
`V34_DUPLEX_SPEC_S` and `V34_P4_KEEP_BAUD_PHASE`, are off and not set here.
- Offline: the HSF's plain V.34 on the D-Modem rig goes from 0/4 to 1/8
  (`docs/dmodem_interop.md`).
- `test-fast` 783/783. The D-Modem matrix is unchanged at 14/18.
- No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at ce417b72
(`/root/deploy-ce417b72.build.log`). The binary (md5 b4fea629…) is kept as
`/opt/v90ras/sip_v90_modem.ce417b72`, and the previous one (5849d2e1,
fdae0f0f…) as `sip_v90_modem.pre-ce417b72`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. The restart logged 20 v90ras
"registration success" lines.

## Deployed 5849d2e1 (2026-11-25)

On top of 4035f476, this deploy adds 5849d2e1. When V.34 Phase 4 resets the
receive front end, which puts the symbol instant back to phase 0, the T/2
eye chooser now gets two flips of its own and a fresh window, as the V.90 CP
stage already did. A spent per-call cap had left Phase 4 TRN at the eye
crossing.
- Offline: the 10 dB duplex row passes 96/96 over 8 noise seeds × 12 delays,
  against 94/96 before.
- `test-fast` 783/783; `test-slow` 25/26 (the existing courier-emu row).
- No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 5849d2e1
(`/root/deploy-5849d2e1.build.log`). The binary (md5 fdae0f0f…) is kept as
`/opt/v90ras/sip_v90_modem.5849d2e1`, and the previous one (4035f476,
e52b9c16…) as `sip_v90_modem.pre-5849d2e1`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. The restart logged 20 v90ras
"registration success" lines, and v90isdn logged "SIP V.90 modem ready".

## Deployed 4035f476 (2026-11-25)

On top of 780969a0, this deploy adds 4035f476. In V.34 Phase 4, after the
peer's MP' has been decoded, E (§11.4.1.x.3) is now taken on the MP frame
grid when the 20 bits are nearer E than the MP' prefix, counted in symbols:
at most 2 symbol errors, and a 2-symbol margin for 2. One noise hit in E
used to strand Phase 4 while the far end went on to B1.
- Off the grid, or without a decoded MP', E still needs all 20 ones.
- Offline: the 10 dB duplex row fails 2/96 over 8 noise seeds × 12 delays,
  against 5/96 before. The two left never accept any MP.
- `test-fast` 781/781.
- No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 4035f476
(`/root/deploy-4035f476.build.log`). The binary (md5 e52b9c16…) is kept as
`/opt/v90ras/sip_v90_modem.4035f476`, and the previous one (780969a0,
77da78d8…) as `sip_v90_modem.pre-4035f476`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. The restart logged 20 v90ras
"registration success" lines, and v90isdn logged "SIP V.90 modem ready".

## Deployed 780969a0 (2026-11-25)

On top of 9e7f5cf4, this deploy adds 780969a0. In plain V.34 Phase 3 the far
end's J now decides our Phase 4 TRN constellation (V.34 §10.1.3.8, §11.4) by
the majority of its near-exact windows, not one J period. One bit error used
to make a 4-point J read as J(16).
- When J is confirmed is unchanged.
- The V.90 digital modem's Ja path is untouched.
- Offline: the 10 dB duplex row fails 5/96 over 8 noise seeds × 12 delays,
  against 8/96 before. The three wrong-J runs pass; the other five miss E
  after MP, the same runs before and after.
- `test-fast` 777/777. The MT5634 3429 replay still finds B1 at +8.
- No smoke call was made.

Built in `/root/deploy-5640b4d3`, checked out at 780969a0
(`/root/deploy-780969a0.build.log`). The binary (md5 77da78d8…) is kept as
`/opt/v90ras/sip_v90_modem.780969a0`, and the previous one (9e7f5cf4,
1290652b…) as `sip_v90_modem.pre-780969a0`. Installed by rename with
`/run/v90ras/busy` empty and no PPP interface up. The restart logged 20 v90ras
"registration success" lines, and v90isdn logged "SIP V.90 modem ready".
