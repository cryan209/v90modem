# RasFinder V.32bis startup regression captures

Four byte-exact 17-second prefixes of 8 kHz PCMU receive taps, recorded from
wired vmhost CT 117 to the RasFinder through its AudioCodes MP-114 bearer.
Source paths, sizes and SHA-256 values are in `manifest.json`.

`6311-rx.g711`, `6312-rx.g711` and `6313-rx.g711` come from the original
12-call modulation matrix. The old receiver rejects their S acquisition
before decoding any R1. Its 64-symbol single-gain residual exceeds 0.08,
although the alternating states are stable. A 0.12 acquisition tolerance
recovers the Table 5 offer 0x1660 from every call. Only the first R1 is
asserted: the corrected transmitter then differs from the original call.

`6311-echo-rx.g711` comes from the first correction's 14400-bit/s call.
The receiver acquires our delayed S echo while the answer modem is silent,
trains on it, and misses the real second conditioning signal. The clause
6.1/6.2 acquisition gate recovers R3=0x1000. The shared bulk-delay canceller
also fits the actual 1921-sample echo with about 27 dB ERLE on this prefix.
Only events through R3 are asserted, since our new E then differs from the
transmission whose responses were recorded.

`python3 tools/test_v32bis_rasfinder_acquisition.py` drives the whole engine
with split 80-sample receives. It asserts S/R1 on all three original calls,
and R3 plus a bulk-delay fit on the echo fixture. It does not claim a live
DATA result from replay. Fresh calls validating LAPM/login are retained in
`reference-data/artifacts/v32bis-rf-fixes-20261020/` and described in
`docs/v32bis_compliance_plan.md`.
