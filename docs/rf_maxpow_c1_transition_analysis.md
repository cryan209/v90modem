# rf-maxpow-c1 raw-tap transition measurement

The change is consistent with increased echo of our transmitted signal. A
peer-only change does not explain the independently predictive TX correlation.
This does **not** establish that echo accounts for all the degradation or locate
its physical source (hybrid, ATA, or another return path).

The analysis uses only decoded PCMU samples from `live-rx.g711` and
`live-tx.g711`: no live receiver output, state, AGC, equalizer, symbol decisions,
or replay. The documentation's independently anchored CPt begins at RX
23.000312 s; its phase-residual degradation is in the 23.375312–23.500312 s
bin. That file-time interval is used instead of assuming a receiver's “symbol
~4000” counter has a shared origin. No independent mapping of that counter was
found in the capture documentation.

## Measurements

| Quantity | Before: RX 23.05–23.25 s | After: RX 23.50–25.20 s |
|---|---:|---:|
| Raw linear PCM RMS | 2529.45 | 2629.55 |
| RX power change | baseline | +0.337 dB (+8.07%) |
| Spectral centroid | 1807.4 Hz | 1924.7 Hz |
| RMS spectral bandwidth | 939.6 Hz | 947.9 Hz |
| Power, 2400–3400 Hz | 1.940 million | 2.313 million |
| Power, 3400–4000 Hz | 0.123 million | 0.196 million |
| Aligned TX RMS | 3772.0 | 8106.0 |
| TX-fit removal, held-out samples | 0.573% | **9.577%** |

RMS is in decoded signed PCM units. Earlier notes used a scale four times
smaller; ratios and dB values agree. Spectrum is Hann-windowed Welch averaging
with 512-sample frames and 50% overlap, without per-frame DC subtraction.
Band powers are in squared PCM units and differ slightly from time-domain
power because of window weighting and finite-window statistics.

After subtracting the TX prediction, the full after-window centroid is
1853.8 Hz and its 2400–3400/3400–4000 Hz powers are 1.915/0.127 million:
much of the added high-frequency content disappears. These full-window
spectra include training data; the stronger evidence is the separate held-out
half (RX 24.35–25.20 s): its centroid moves from 1908.1 to 1840.1 Hz and
9.577% of its power is removed without refitting.

## Alignment and the supplied 187 ms round trip

A normalized raw-waveform cross-correlation searches RX-minus-TX file offsets
80–220 ms on **only the first half** of the after-window. It selects **936
samples / 117 ms**, correlation 0.209947. A centered 129-tap FIR is fitted
on that half and evaluated on the second half; its span is ±8 ms, so it cannot
hide a 70 ms alignment error.

Using the supplied physical RTT of 187 ms, this alignment implies a tap-origin
skew of **−70 ms**:

`file_offset = physical_delay + (TX_origin − RX_origin)`.

This is a conditional reconciliation, not a new measurement of tap origins
or of physical RTT. The RTP traces start almost 3 seconds apart and are
packet-arrival/send records, not a calibration of the PCM dump starts. They
must not be used as that calibration. Nor is SIP signalling RTT a measurement
of the audio echo path.

The TX tap stops the exact constant-magnitude Ri pattern at sample **186266 /
23.283250 s**. Its predicted return is RX **23.400250 s** at the recovered
alignment, inside the documented degraded 125 ms bin. Treating 187 ms as a
file offset instead predicts 23.470250 s, also inside that coarse bin; timing
alone therefore cannot distinguish the hypotheses. The waveform prediction can:

| Post-window reference | Train power removed | Held-out power removed |
|---|---:|---:|
| 117 ms file offset | 13.028% | **9.577%** |
| 187 ms used directly as file offset | 1.622% | **−2.447%** |
| Wrong-delay control: 617 ms | 2.093% | **−4.395%** |

Negative removal means the prediction adds error. Before the transition the
TX is periodic, so identical pre-window fits at different delays are expected;
the pre-window cannot measure the delay or establish absence of all echo.
A shorter after-window (23.42–23.82 s) independently selects the same 117 ms
offset and removes 2.73% held out, but its 1600-sample training half supports
a less stable 129-tap estimate. It is corroborating evidence, not the primary
power estimate.

The added high-frequency power, its suppression by a TX-derived prediction,
and the held-out delay controls favour added local-TX echo over a peer-only
waveform change. A simultaneous peer signal change remains possible; the peer
also changes CPt/SCR material during this interval. The post-fit spectrum is
close to, but not identical to, the pre-window spectrum. The existing notes'
claim of an exact residual step “117 ms later” exceeds the resolution of their
125 ms phase-residual bins; the present timing result is compatibility within
that bin.

## Reproduce and validation

```sh
.venv/bin/python tools/g711_transition_compare.py \
  artifacts/rf-maxpow-c1/live-rx.g711 \
  artifacts/rf-maxpow-c1/live-tx.g711 \
  --rtt-ms 187 --output artifacts/rf-maxpow-c1/transition-analysis
```

The tool writes JSON measurements, frequency-by-frequency PSD CSV, and a
20 ms RMS timeline CSV. Inputs are never changed. Before/after windows,
alignment search, FIR length, and timeline range are command-line options.

An independent synthetic check (NumPy RNG seed 48, 50,000 independent normal
samples in each stream, gain 0.5, delay 936 samples, fit/evaluation window
10,000–30,000) recovers exactly 936 samples and removes 19.46% of held-out
power (expected 20%). With no injected echo, the same fit removes −1.47%
held out despite 1.42% apparent removal in training. This checks the delay
sign and demonstrates why training-only improvements are insufficient.
