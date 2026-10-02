# V.42 LAPM throughput

Investigation of 2026-10-03: a V.90 call to slmodemd (54666 down / 31200 up,
V.42bis on) carried about 20 kbit/s of DTE data in *both* directions.  Equal
figures in two directions with line rates 1.75x apart said the line was not
the limit.

## Instrument

`V42_STATS=<seconds>` (spandsp-master/src/v42.c, `stats_report()`) prints one
`[V42STAT]` line per period: I-frames and octets each way, retransmissions,
RRs sent, how the transmit line time was spent (I-frames, control frames, or
idle because the **window** was closed, the far end was **busy** (RNR), there
was **no data**, or other), and the acknowledgement round trip measured from
the moment an I-frame starts to the N(R) that acknowledges it.  Time is
counted in transmitted bits, so the figures mean the same live and offline.
`tools/v42_stats_summary.py <server.log>` averages the steady state.
`V42_FRAME_LOG=1` (existing) logs every S and U frame.  The soak script
`tools/soak/slm_answer_call.sh` now passes `V42_*` and `DS_*` through.

`V42_STATS` and `V42_FRAME_LOG` are diagnostics: off by default, no effect on
the protocol.

## What it measured (live, `artifacts/slm-tput-*` on tower)

Baseline (`slm-tput-base1`, 200 ms receive jitter buffer):

* acknowledgement round trip **490-510 ms**;
* downstream idle with the window closed **84%** of the line time;
* **150 I-frames per 5 s carrying ~890 octets/s -- ~29 octets per frame**
  against N401 = 128.

## Defects, in the order they were found

1. **Ours: V.42bis frames were never filled** (`data_stack.c`,
   `ds_v42_get_frame()`).  Each I-frame took at most 128 DTE octets,
   compressed them and flushed, so a 128-octet frame carried ~29 octets of
   compressed digits.  LAPM's window is k = 15 *frames* (V.42 8.4), so on a
   window-limited link compression bought nothing: 15 x 128 DTE octets per
   round trip, ~1920 B / 0.5 s ~ 30 kbit/s less framing, whatever the line
   rate -- the observed ~20 kbit/s both ways.  Now input is compressed until
   a whole frame of output is pending, and the encoder is flushed (an
   octet-aligned V.42bis FLUSH) only when the DTE has nothing more to give.
   The flush per frame also cost compression ratio.

2. **SpanDSP: T401 was not started when an I-frame went out while T403
   ran.**  `lapm_hdlc_underflow()` started T401 only if *no* timer was
   running, so after any idle spell (T403 running) a lost frame was not
   recovered for 10 s.  With many short frames the next frame's N(S) gap
   (REJ) hid it; with full frames a 1024-octet test payload is one frame and
   `data_stack_test`'s corrupted-retry case failed.  V.42 8.4.1: "If timer
   T401 is not running at the time of transmission of an I frame, it shall be
   started."

3. **SpanDSP: T401 was a fixed 1000 ms.**  With (2) fixed T401 actually runs,
   and at 2400 bit/s one 128-octet frame takes 0.44 s and its piggybacked
   acknowledgement arrives ~1.3 s after it starts, so T401 fired on frames
   that were never lost.  V.42 9.2.1 leaves T401 to the system but it must
   cover a frame and its acknowledgement: now 1000 ms plus three
   maximum-length frames at the line rate (+57 ms at 56000, +1.5 s at 2400).
   `v42_link_test`'s busy and outage cases had budgets derived from the old
   value and were widened to match.

4. **SpanDSP: new I-frames were sent during timer recovery.**  After a T401
   poll (P=1), the F=1 response sets V(S) to its N(R) (V.42 8.4.8), so an
   I-frame sent after the poll is rewound over; when the peer then
   acknowledged it, `ack_info()` saw N(R) beyond V(S) and disconnected.  Live
   (`slm-tput-fill1`): `invalid N(R)=32 with V(A)=31 V(S)=31`, call torn down.
   V.42 8.5.3: no new I-frames in the timer-recovery condition.  **Not
   reproduced offline** -- two SpanDSP ends retransmit faster than the
   acknowledgements overtake them, so the race never opens in
   `v42_throughput_test` (negative control run and recorded); the fix rests on
   the clause and on the live calls that followed, none of which disconnected.

5. **SpanDSP: an unsolicited RNR did not restart T401.**  V.42 8.4.6: on an
   RNR "restart timer T401" and poll the busy peer on its expiry.  The command
   path left only T403 running when the RNR acknowledged everything, so a peer
   that cleared its busy condition silently was not asked again for 10 s.

LAPM disconnects now log a reason (`[V42] disconnect: ...`).

## Offline tests

* `v42_throughput_test` (in `make test`): two LAPM instances, both saturated,
  through delay lines on the 8 kHz grid at 54666/31200.  At 10 ms one way
  both directions run at ~94% of line rate; at 400 ms both converge on
  ~18 kbit/s, the k x N401 / RTT bound -- the live symptom.  An errored long
  round trip must keep the link.  `v42_throughput_test <one-way-ms> [down up
  [flip-every-n-bits [outage-ms [down-source-bps]]]]` runs one row.
* `data_stack_test`: "V.42bis fills I-frames" runs numbered lines (as the
  soak sends) through V.42bis at 56000 bit/s with 400 ms each way: **2400 DTE
  octets/s before the fix -- exactly the uncompressed-window bound -- and
  10535 after**.

## Live result (tower, slmodemd, PAYLOAD=1, 90 s of payload each)

| call | jitter buffer | ack RTT | downstream | upstream |
|---|---|---|---|---|
| `slm-v90-pay2` (before) | 200 ms | -- | ~20 kbit/s | ~20 kbit/s |
| `slm-tput-fix3` | 200 ms | 539 ms | 32914 lines, 29.3 kbit/s | 15139 lines, 13.5 kbit/s |
| `slm-tput-jb40` | 40 ms (`ME_JB_MS=40`) | 377 ms | 39600 lines, 35.2 kbit/s | 17794 lines, 15.8 kbit/s |

Every line contiguous, both calls held to the end.  One call per arm.

**Read the averages with the next point**: once the frames are full the
window is closed only 5-12% of the time, and in unobstructed stretches the
downstream ran **4.2 kB/s on the wire, ~18 kB/s (~145 kbit/s) of DTE text**,
short only because the soak's own source (100 lines per 50 ms) ran dry.  The
averages are set by **slmodemd going RNR for 5-15 s at a time** (57-76% of
the line time): we poll it every T401 and it answers RNR F=1, polls us with
RNR P=1 itself, and its N(R) stays frozen -- its upstream nearly stops too.
That is the peer's receive/DTE side, not ours; why it stalls (its DTE pacing
at the ttySL0 rate, or CPU) is open and unmeasured.

The upstream is lower than before.  Its frames are full (128 octets) but the
peer sends few of them while it is stalled, and we REJ a share of them
(lost/bad-FCS frames on the upstream receiver -- the known upstream decode
quality, `docs/v90_upstream_data_path.md`).  Not a window effect.

## Jitter buffer

`ME_JB_MS` stays at 200 ms by default.  It exists because V.8 was losing JMs
to arrival spikes on a Wi-Fi host (`sip_modem.c`), and with full frames the
window rarely binds, so the 160 ms it adds to the round trip is now worth
~20% downstream on this rig.  On a wired host `ME_JB_MS=40` is measured safe
(one call, tower: 0.4 ms mean jitter); changing the default needs V.8 re-tested
on a jittery path first.
