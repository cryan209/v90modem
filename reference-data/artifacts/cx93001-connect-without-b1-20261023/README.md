# CONNECT without upstream B1, CX93001, 23 October 2026

First 45 seconds of `cx-ja-B1`, the optimized Ja arm retained under
`reference-data/artifacts/cx-ja-cost-20261023/B1`. Both raw mu-law files
are cropped at the identical 360000-codeword boundary. No resampling,
transcoding, insertion or deletion within the retained interval.

CT 117, CX93001 originating as AudioCodes 6314 and dialling 2904;
ME_MODE=v90, ME_V90_TRN2D_SYMBOLS=8000, factory +ES, CX AT\N2,
ATS7=60, AT+MS=V90,1,300,33600,300,56000. Tested live binary SHA-256:
ac4346df6274a4a2825518c9338f92d60a576e3def4457a2426687931d7396e4.

CP prime is CRC-valid and starts an independent B1 search. Every search
fails (best template fit 61%); after seven windows the receiver reports
that the upstream has no carrier. Error-control detection later falls
back to buffered V.14. The old engine nevertheless emits CONNECT 50666
on its PTY. Corrected engine output must contain no CONNECT, with both
B1-give-up and error-control-fallback evidence present.

V.90 9.4.1.6 requires B1 reception before circuit 109 is turned on.
V.250 +ES fallback relaxes error control, not the physical connection.
The gate keeps the data stack running and permits a delayed CONNECT
when acquisition succeeds. Existing mu-law/A-law engine-pair payload
rows assert initial deferral and subsequent successful delivery.

Read the PTY with tools/replay_dte_check.py, not the engine DATA log:

    ME_MODE=v90 ME_V90_TRN2D_SYMBOLS=8000 python3 tools/replay_dte_check.py \
      --expect unconnected --require-log 'B1 giving up' \
      --require-log 'no error-control peer' --require-log 'CONNECT deferred' \
      -- ./v90_engine_replay <this-dir>/live-rx.g711 ulaw \
      --fast --sync --echo-ref --sip-answer

This is a negative control-plane regression, not a V.90 acquisition or
hardware-payload qualification. It cannot assess peer responses after
our first divergent transmission.

SHA-256:

- live-rx.g711: 1dd0d2a057c7477c04484257e76e420d3ff0a7461f0a646cf6c3d2580b342994
- live-tx.g711: b53a5f9b034d9456d4827ce3585ead31ddc1879895835aa093e675e0686f506b
