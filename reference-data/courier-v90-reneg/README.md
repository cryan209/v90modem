# Courier V.90 renegotiation CP followed by E

`cp-e.g711` is an unchanged 2350-codeword mu-law excerpt from a software
Courier 403 / v90modem closed loop. It is not a physical-hardware recording.
Source: courier-emu `artifacts/analog-v90-reneg-fix/native-mp/call/engine-rx.g711`,
zero-based samples `[394000,396350)`, 8000 codewords/s.

Consumer: `v90_reneg_engine_test.c` (strict batch demodulation regression).
The valid CP has DRN=21, acknowledge=0, and five constellations. Its final
symbol is near excerpt sample 2052; E begins near 2054. E must be decoded on
the same hypothesis as that CRC-valid CP, not inferred from its presence.
V.90 (09/98) sections 8.5.3, 9.4.1.4 and 9.6 permit E to complete the exchange
without a separately decoded CP'. Carrier still requires validated B1.
