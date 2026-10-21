# CX93001 V.22bis S1, live receive taps

The first four files are raw G.711 mu-law, 8000 Hz, byte-exact slices of
`live-rx.g711` from calls on 2026-10-21. A CX93001 (`AT+ES=4,4`, `AT+MS=V22B`) dialled 2904
into the CT 117 server (`/root/s10-live1..4`). Each file is 12.9-14.9 s of
its call: our USB1 echo, then the CX's S1 (13.14-13.25 s), its scrambled
ones, and the start of its 2400 bit/s scrambled ones.

| file | live result | SpanDSP before the fix |
|---|---|---|
| call1-rx.ulaw | trained 1200, no CONNECT | 1200 |
| call2-rx.ulaw | trained 2400, CONNECT 2400 MNP | 2400 |
| call3-rx.ulaw | trained 1200, no CONNECT | 1200 |
| call4-rx.ulaw | trained 1200, no CONNECT | 1200 |

Used by `v22bis_s1_test`. See docs/carrier_loss.md, "Why calls 1, 3 and 4
trained at 1200".

`call5-rx.ulaw` is the byte-exact 12.9-14.9 s slice from the first
2026-10-22 re-check (`cx93001-s1-recheck-20261022/call1/live-rx.g711`).
CX setup: `AT\N2`, `AT+MS=V22B`; isolated main `5decc31b` on wired CT 117.
The answerer trained at 1200 and the CX never reported CONNECT. S1 is
present, but the decision eye samples its crossing (roughly 10 degrees
between consecutive outputs); the other T/2 eye still resolves S1. The
original phase-change fix gives 1200; the two-eye search gives 2400. This
fixture establishes receive-side acquisition only, not the peer's response
after our first divergent transmission. Full call evidence and manifest
are retained locally in `reference-data/artifacts/cx93001-s1-recheck-20261022/`.

`call6-rx.ulaw` is 12.9-15.5 s of re-check call 4, with the initial
two-eye candidate (detect S1 on either eye, keep the original decision
eye). Our TX trained at 2400, but RX never acquired the scrambled ones,
V.42 detection timed out, and the CX never reported CONNECT. Selecting
the eye that found S1 during acquisition completes receive training and
delivers bits. `v22bis_s1_test` checks actual received bits on this fixture
and original call 2; checking only TRAINING_SUCCEEDED would miss this
because SpanDSP reports that status from the transmitter.
