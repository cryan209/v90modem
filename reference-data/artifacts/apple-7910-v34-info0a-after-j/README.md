# Apple USB modem -> MultiTech 7910, V.34, INFO0a after our Phase 3 J

First 30 s of the raw device receive tap (`rx.s16`, s16le at 9600 Hz, what
`apple_usb_modem_coupler --rx-replay` consumes) from
`artifacts/apple-v34-v90-20261028/v34-7910-alaw-policy-cap14400` (2026-10-29,
`AT+MS=V34,0,300,14400,300,14400`, 3000 baud).

We sent S, S-bar, PP, TRN and J (V.34 11.3.1.1.7).  The 7910 went silent, never
started Phase 4, and about 3.2 s after our J sent one INFO0a (fill, sync
`01110010`, DPSK on 2400 Hz; V.34 11.3.2.2.2 / 11.5) and fell silent until it hung
up.  The same signature recurred in `v34-7910-final-cap14400`.  Our caller stayed
in PHASE3_WAIT_S: the Tone A Goertzel rejects DPSK at every phase reversal.

Regression row in `tests/fast.list`: the replay must report the peer retrain
and restart Phase 2.
