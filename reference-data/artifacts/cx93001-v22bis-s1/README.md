# CX93001 V.22bis S1, live receive taps

Raw G.711 mu-law, 8000 Hz, byte-exact slices of `live-rx.g711` from four
calls on 2026-10-21. A CX93001 (`AT+ES=4,4`, `AT+MS=V22B`) dialled 2904
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
