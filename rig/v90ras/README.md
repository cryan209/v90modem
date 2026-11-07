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
| `/etc/systemd/system/v90ras.service` | the 10-line server (copy here); SIP ports 5300-5309, RTP 30000+100*i, PTYs `/run/v90ras/line.0`-`.9`, `ME_JB_MS=40` |
| `/etc/systemd/system/v90ras-ppp@.service` | one pppd per line, instances 0-9 (copy here) |
| `/etc/ppp/v90ras-options` | shared pppd options (copy here) |
| `/etc/v90ras/sip.env` | SIP account passwords (0600, not in git) |
| `/etc/ppp/pap-secrets` | PAP line for user `v90dial`, server name `v90ras` (backup `pap-secrets.before-v90ras-20261107`) |
| `/root/v90ras-CREDENTIALS` | the dial-in username/password for the client (0600) |

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
waits for it, pppd negotiates (`passive`, `require-pap`, `multilink`,
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
| 01 | 1 x V.90 analogue (our client, 2903) to 2950 | line 0 answered; V.90 retrained 7 times in Phase 3/4 and fell back to **V.22bis 2400**; PAP and IPCP then fine |
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
fast.list V.90 pair row had set the fallback to 0, which hid it. Fix: a
rejected energy return puts the gate back to "in signal". Offline, the pair
then parses N=120 600 ms into Ja and sends the DIL. fast.list has the
default-fallback row.

## Windows 95 client

Dial-Up Networking, server type "PPP: Windows 95, Windows NT 3.5, Internet",
TCP/IP only, server-assigned address, PAP user/password from
`/root/v90ras-CREDENTIALS`. Multilink: add the second (and later) modems
under the connection's Multilink tab, each dialling 2950 (or a fixed
2951-2960 number).
