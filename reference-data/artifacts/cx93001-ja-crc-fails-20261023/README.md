First 14 s of a live CX93001 V.90 call (CT 117 answerer, 2026-10-23, `runs-b2`).
The CX sends its Ja (the same fixed DIL descriptor in every call) but our
equaliser, conditioned on a late PP start, decodes it with enough errors that
no frame ever passes its CRC; the CX gives up 1.5 s after Ja starts (V.90
9.3.2.4) and retrains. The 384-bit template match in `v90_ja_template_scan()`
recognises the Ja at 281/384 bits and the engine answers with Sd.
