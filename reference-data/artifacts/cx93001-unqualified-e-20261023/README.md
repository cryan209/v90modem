# CX93001 false E, 23 October 2026

First 24 s of wired CT 117 call `/root/cx-work-20261023-trn8k1`,
CX93001 calling 2904 through Grandstream line 6005. Both files are raw,
byte-exact 8000 Hz mu-law codewords, cropped at the same 192000-sample
boundary. No resampling or transcoding. Server: main 2bb6bb2b plus the
pre-existing working-tree deferred line-echo fit and V.34 changes;
ME_MODE=v90, ME_V90_TRN2D_SYMBOLS=8000, factory +ES, CX AT\N2 and
AT+MS=V90,1,300,33600,300,56000.

The live decoder reported E at TRACE +21143ms with cp_valid=0; independent
DQPSK/GPA decoding finds continuing CRC-valid CP frames from RX-file
20.740313 through at least 23.802188 s. The batch decoder's valid CP
cannot validate the different streaming demodulator's all-ones run.
Pre-change synchronous echo-reference replay also falsely reports E at
+22100ms. Corrected replay accepts batch CP and subsequently streaming CP,
but reports no E or DATA in this prefix. This is a negative regression,
not evidence of later interoperability: replay cannot judge the peer's
response after the first divergent transmission.

V.90 9.4.1.4 permits E instead of CP prime; 9.4.1.6 requires received E
before conditioning for B1. The correction retains the E alternative
when its own bit stream is CRC-qualified. Existing A-law engine-pair
payload tests cover batch CP-prime recovery without a qualified E tap.

SHA-256:

- live-rx.g711: d5d9f4059c739191663f811544a0027eedfaa07e2785903d9a60bb45bbbdcdc1
- live-tx.g711: 13bd2e7f611f28cb393f9bf3829304a1c94b17e5756fb1663561c8628f0faa4c
