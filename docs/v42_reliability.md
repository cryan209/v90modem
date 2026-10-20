# V.42 / LAPM and DTE data path reliability

Audit of 2026-10-20: can the error-controlled data path lose, corrupt or
stall data that the line delivered correctly?  Throughput is in
`docs/v42_throughput.md`; this note is about integrity.

## Method

`v42_stress_test` (fast tier): two data stacks (`data_stack.c`, the real
DTE-facing layer, not bare SpanDSP) across an impaired bit channel, both
directions saturated with a regenerable stream (numbered text lines with
random binary runs), every octet graded on arrival.  Impairments: independent
bit errors, random-bit bursts, periodic outages filled with noise, zeros or
marks, long round trips, and a DTE that drains a bounded receive queue slower
than the link delivers.  Plain LAPM, V.42bis and V.44 each.  A row fails on
any corrupted or lost octet, a teardown, or a delivery gap longer than the
row allows.  `v42_stress_test <row> <seed>` runs one row verbosely, and
`V42_FRAME_LOG=2` now logs I frames (N(S), N(R), P, V(S)/V(R)/V(A), retry
count) with a line-clock timestamp on every logged frame.

## What held

LAPM recovery itself is sound: 18 rows x 10 seeds, no corruption, no
teardowns, under 1e-4 BER, 64-bit bursts twice a second, 400 ms noise
outages every 5 s and 4 s outages.  The only long gaps (6-10 s at 2400 bit/s
with a burst hitting one frame in five) are a retransmission lost a second
time inside one REJ exception, which V.42 8.5.1 leaves to T401 (2.5 s at
2400): correct, so that row allows 12 s.

## Defects found and fixed

1. **Short PTY writes dropped the rest** (`modem_engine.c`, six drain sites).
   Each read up to 256 octets from the receive ring and called
   `di_write_data()` ignoring the count; a PTY holds about 1 KB (macOS), so
   whenever the downstream outran the DTE for a moment the unwritten part was
   gone.  Under V.42 those octets had already been acknowledged, so nothing
   could recover them.  This is very likely the x2 "940-byte downstream
   incomplete" that pacing the test made go away.  Now one
   `me_drain_rx_to_dte()` peeks, writes, and consumes only what the PTY took.
   `engine_pair_test --dte-stall-seconds 6` (fast tier, V.34) stops the
   answering DTE reading for 6 s under a 12.5 KB payload: HEAD delivered
   1039 octets of it, now all of it.

2. **No receiver-busy flow control.**  V.42 8.4.7 is how a receiver that
   cannot take more stops the peer; nothing ever entered the condition, and
   SpanDSP's `v42_set_local_busy_status()` only set a flag -- no RNR on
   entry, no RR/REJ on clearance.  A DTE slower than the link (V.42bis text
   expands ~4x) overran the 16 KB ring and the excess was discarded with
   "DTE RX ring overrun".  Now `ds_set_rx_room()` lets the data stack watch
   the ring and enter busy with half of it (64 KB) free, clear at three
   quarters; one V.42bis frame decodes to at most ~28 KB, and a frame already
   accepted cannot be refused.  The SpanDSP call sends RNR at once, and on
   clearance RR -- or REJ with N(R) = V(R) when an in-sequence frame was
   discarded while busy, so the peer resends at once instead of on T401.
   Negative control (`V42_STRESS_NO_FLOW=1`): 400-560 thousand octets lost
   per slow-DTE row; with it, none, and the DTE runs at its full drain rate.

3. **Rings lived for the process, not the call.**  Both engine rings were
   only initialised at start-up: octets one call left queued (DTE data not
   yet sent, received data not yet read) went out on, or up from, the next
   call.  Now, at SIP disconnect, a DTE that is still reading gets up to
   0.5 s (stopping after 100 ms without progress) to take what was received,
   then both rings and the PTY-side ring are cleared and the losses logged.

4. **Training-time junk reached the DTE once drains stopped dropping.**
   `data_stack_prepare()` runs V.14 framing over the line during training;
   what it assembled from noise used to be silently dropped by the lossy
   drain.  Fixing (1) delivered it after CONNECT (`?~?~` HDLC flags ahead of
   the payload on the V.92 ODP-bypass tests).  The receive ring is now
   cleared where data starts (`data_stack_start_online()`) and in
   `data_stack_prepare()`.

5. **LAPM accepted invalid frames** (V.42 8.1.3).  No length check -- a
   one-octet frame was parsed from stale buffer octets -- and no address
   check, so a noise burst that happened to pass the 16-bit FCS was acted on
   whatever its address.  Now I/S frames need three octets (S exactly three),
   U two, and the address must be DLCI 0 with EA set (0x01/0x03).  The
   address check alone turns ~127/128 of such frames back into nothing.

6. **FRMR was ignored** (V.42 8.5.6 says terminate).  The peer that sent it
   discards everything until a reset, so the link sat deaf both ways until
   N400 polls ran out, ~20 s.  Now it terminates at once, with a reason.

7. **DM F=0 in the disconnected state reported the link CONNECTED** without
   establishing it (Table 9 says establish).  The DTE went online over an
   entity that discarded every I frame.  Now it sends SABME.

8. **The REJ exception survived re-establishment** (`reset_lapm()` did not
   clear it), so the first sequence error on a new link went unreported until
   T401.

9. **T401/T403 kept the old rate after a retrain.**  V.42 timers run on the
   transmit bit clock; the resume paths (V.34 11.5, V.90 9.5) kept the link
   but not the rate, so a retrain to a faster rate shortened T401 in real
   time.  `ds_set_line_bit_rate()` now re-expresses them.

The "DTE RX ring overrun" log is now one line per doubling (it was one per
octet from the media thread) and the per-call total is logged at teardown.

## Not done

* **MNP 2-5** is not implemented at all.  V.42 (2002) dropped Annex A, but
  plenty of answering modems still offer MNP when V.42 detection fails; we
  fall back to V.14 (+ES) instead.
* **32-bit FCS** (XID HDLC optional functions) is not offered.  It is the
  remaining defence against noise frames that pass the 16-bit FCS with a
  valid address (1 in 2^16 x 1/128 of noise "frames").
* **Orderly DISC on ATH** (`data_transfer_plan.md`): the bearer just goes;
  the peer learns from carrier loss.
* **SREJ** is not negotiated (optional, 8.5.1 b/c).
* Live qualification: everything above is offline; the slmodemd and RasFinder
  rigs are where (1)-(2) should show as fewer lost lines.
