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
| 2951-2960 | line 0-9 directly (SIP accounts, `endpoint-modem`, **u-law only**, `aor-single-reg`) |

Dialplan is in `[default]` after the 17940 route; backups
`/etc/asterisk/{pjsip,extensions}.conf.before-20261107-v90ras`. The u-law-only
template means an A-law caller (the Diva path through the Cisco) would be
transcoded; add an A-law set if that is needed.

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
Not yet verified live.
