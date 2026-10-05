# Offline PCM payload recovery

`tools/vpcm_record_recover.py` recovers downstream bits from an 8 kHz,
16-bit analogue WAV using previously decoded CP records and measured training
positions. It is an experimental recording tool, separate from the live
modem and HTML event decoder. It requires NumPy and the built local SpanDSP
library; it compiles its C helper and core dependencies afresh on each run.

## Reproduce the QC recordings

The CP files below contain one byte per bit and are preserved with the
2026-10-05 acquisition artifacts. Positions are sample indices in channel L.

```sh
OPENBLAS_NUM_THREADS=1 .venv/bin/python tools/vpcm_record_recover.py \
  gough-lui-v90-v92-modem-sounds/USR-Message-V92QC-bong-bong-bong.wav \
  --cpt artifacts/gough-v92-payload-acquisition-20261005/USR-Message-V92QC-bong-bong-bong-crc0b91.bits \
  --cp artifacts/gough-v92-payload-acquisition-20261005/USR-Message-V92QC-bong-bong-bong-crce681.bits \
  --training-start 172810 --b1-start 194338 \
  --output artifacts/gough-v92-payload-acquisition-20261005/reusable-usr

OPENBLAS_NUM_THREADS=1 .venv/bin/python tools/vpcm_record_recover.py \
  gough-lui-v90-v92-modem-sounds/Motorola-SM56-V92QC-extDILlaserbeam.wav \
  --cpt artifacts/gough-v92-payload-acquisition-20261005/Motorola-SM56-V92QC-extDILlaserbeam-crc6997.bits \
  --cp artifacts/gough-v92-payload-acquisition-20261005/Motorola-SM56-V92QC-extDILlaserbeam-crc3759.bits \
  --training-start 183637 --b1-start 208729 \
  --output artifacts/gough-v92-payload-acquisition-20261005/reusable-motorola
```

CP acquisition and timing search are still separate experiments, not automatic
stages of this CLI. The native Table-14 validator rejects invalid input records.
Both successful calls have V.34 QAM upstream control and PCM downstream;
their V.92/QC names do not establish PCM upstream.

## What qualifies a recovery

V.90 8.6.5 TRN2d and 5.4.5 spectral shaping provide known training magnitudes
and bit constraints. Foreign implementations need not choose our shaper's
signs at every cost tie. An initial sign/channel fit followed by a beam of
native shaper-inverse states reconstructs a training sequence constrained to
ones. The first two training frames are allowed to condition the GPC register.
This estimate alone is not payload evidence.

Inverse filters of 257 and 513 taps are fitted to the first 4092 TRN2d samples.
Neither uses B1d samples in that fit. B1d's first 24 frames calibrate magnitude
gain and offset; the last 24 provide a separate check. The tool requires all
48 B1d frames to decode to ones on both filters before exporting data.
The data frame contains D = drn + 20 bits, unlike TRN2d's D = drn + 8.
USRs' interval-specific codec-output masks differ from the source masks:
slicing uses each interval's output levels and maps their ordinal indices back
to source levels before native demapping. Treating all intervals as one
constellation does not work.

The tool tests two initial differential-polarity hypotheses. The zero seed
leaves exactly bits 0, 18 and 23 wrong on both calls; seed one resolves that
single-bit GPC ambiguity and gives perfect B1d. This is an offline recording
hypothesis. V.90 8.6.1 specifies zero initialization, and the live implementation
and its initialization are unchanged.

Only the prefix where both filters agree and no demapping erasure occurs is
examined for V.42 detection patterns and bit-stuffed HDLC with a valid FCS.
Agreement between filters is a consistency check, not independent proof of
arbitrary user bytes. V.42 detection is also checked by the native public
SpanDSP detector. Application payload extraction is not implemented by this
tool, even if future inputs yield FCS-valid frames.

## Measured results

| Recording | B1d on each filter | Agreeing data bits | Recovered negotiation octets | FCS-valid HDLC frames |
|---|---:|---:|---:|---:|
| Motorola SM56 QC | 1632/1632 ones | 4391 | 0 | 0 |
| USR Message QC | 1584/1584 ones | 60692 | 946 | 0 |

USR's first 40792 data bits are idle marks. At data bit 40792 (25.25534 s),
repeated V.42 answer detection begins: asynchronously framed `E`, eleven
mark bits, asynchronously framed `C`, eleven mark bits. There are 473 complete
pairs. The native detector reports detection success at bit 40876, but never
reports a connected link or delivers application bytes. The exported
`v42-adp.bin` contains `EC` repeated 473 times; its SHA-256 is
`36dff86296d419f8f9f4ab3d3a3608ea31a6f1bbb61df7ca20d30d50b0a571f5`.
Motorola's recovered prefix consists of idle marks followed by a short
uncertain capture-boundary tail. No application bytes have been recovered.

`recovery.json` preserves source/library/helper hashes, the build command,
CP validation, each filter/seed's B1d errors, native detector output and
framing results. `agreeing-data.bits` contains one byte per recovered bit;
candidate streams can contain byte 2 for an erasure. Ignore any stale outputs
from a failed run and require the current manifest's `b1_qualified` result.

Agere has repeated CRC-valid control bodies, but its wire fill layout fails
strict validation and a fill-only repair still leaves b2 = -65. It remains
excluded from payload recovery; no protocol coefficients were altered to
force acceptance. See [the corrected windows](v92_test_call_windows.md).

## Validation

```sh
.venv/bin/python tools/test_vpcm_record_recover.py
```

The framing checks cover known CRC-X25 data, corrupted/stuffed HDLC, erasures,
complete/truncated V.42 detection pairs and permitted mark spacing. Both
positive recordings were rerun through fresh helper builds with warnings
treated as errors. Mutated CP and silent-audio inputs were rejected. Those
results are stored in the acquisition directory's `validation.json`.
