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
| `/etc/v90ras/profile.0`-`.9` | power-on AT profile, copies of `v90ras-profile` (plain V.34, 21600 cap) |
| `/etc/v90ras/sip.env` | SIP account passwords (0600, not in git) |
| `/etc/ppp/pap-secrets` | an unused `v90dial` line from before `noauth` (backup `pap-secrets.before-v90ras-20261107`) |

Bundle: server 10.254.200.1, client 10.254.200.2, interface `v90ras0`,
`ms-dns 192.168.88.1`. There is no NAT or forwarding, so a client reaches
only 10.254.200.1 (run `tools/eicon/http_test_server.py --bind 10.254.200.1`
there for transfers).

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

## Windows 95 client

Dial-Up Networking, server type "PPP: Windows 95, Windows NT 3.5, Internet",
TCP/IP only, server-assigned address, any username and password (the
server runs `noauth` since 2026-11-07: a home test rig, and the first real
call failed only on the login). Multilink: add the second (and later) modems
under the connection's Multilink tab, each dialling 2950 (or a fixed
2951-2960 number).
