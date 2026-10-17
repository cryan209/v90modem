# Direct Apple pair: delayed E loses upstream B1

`late-e-rx.ulaw` is 68,800 samples (8.6 seconds) cropped at sample 79,200
from the digital answerer's 8 kHz linear RX tap in the 2026-10-17 powered
direct-pair trial `strict-reference-9600up-1`. The linear tap was encoded to
nearest mu-law levels for the replay tool; this is an analogue recording,
not a byte-exact network DS0 capture. Native hardware ran at 9600 Hz.

The peer sends V.34 upstream at 3200 baud, low carrier, 9600 bit/s, GPA,
16-state trellis. B1 is near the beginning, but the live CP-bitstream E
detector triggers about six seconds later (V.90 9.4.2.4-.5). With the
8192-sample T/3 ring the waveform is gone before acquisition. The retained
131072-sample ring acquires the same recording at 100.0% fit:

```sh
./v90_upstream_replay artifacts/apple-direct-v90-b1/late-e-rx.ulaw ulaw 3200 9600 6.2 6.2
```

The handover and history are fixed. Do not use the tool's handover sweep to
compare ring sizes, because that sweep changes the experiment's anchor.
The early-search API is separately tested with a fixed pre-B1 anchor:

```sh
./v90_upstream_replay artifacts/apple-direct-v90-b1/late-e-rx.ulaw ulaw 3200 9600 0.2 0.2 cp-prime
```

That mode asserts that starting the search does not manufacture an E event.
Only a complete B1 template and its out-of-sample validation admit bits;
the early entry suppresses the optional blind/non-reset fallback. The
whole-engine tests supply the actual CRC-valid CP' qualification.

The fixture checks acquisition history, not successful V.90 negotiation or
DTE payload delivery. Full call evidence remains local under
`artifacts/apple-v90-followup-20261017/`.
