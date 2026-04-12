#!/usr/bin/env python3
"""
Spectrogram of the right channel of USR-Message-V92QC in the 9476-13281ms window.
Short-time FFT to see how the carrier evolves over time.
Also check whether this is actually a wideband V.34 signal or something else.
"""
import wave
import struct
import math
import cmath

wav_path = "gough-lui-v90-v92-modem-sounds/USR-Message-V92QC-bong-bong-bong.wav"

with wave.open(wav_path, 'rb') as w:
    nch = w.getnchannels()
    rate = w.getframerate()
    nframes = w.getnframes()
    raw = w.readframes(nframes)

# Extract right channel
all_samples = []
for i in range(nframes):
    off = i * nch * 2 + 1 * 2
    val = struct.unpack_from('<h', raw, off)[0]
    all_samples.append(val)

# Wider window to see context: 7000-15000ms
start_ms = 7000
end_ms = 15000
start_idx = int(start_ms * rate / 1000)
end_idx = min(int(end_ms * rate / 1000), len(all_samples))
samples = all_samples[start_idx:end_idx]
N = len(samples)

print(f"Right channel, {N} samples ({start_ms}-{end_ms} ms)")
print()

# ---- Short-time FFT spectrogram ----
fft_size = 512
hop = 256  # 32ms hop
print("=== Spectrogram (dominant frequency per 64ms window) ===")
print(f"{'Time':>10} {'Freq1':>8} {'Mag1':>8} {'Freq2':>8} {'Mag2':>8} {'Freq3':>8} {'Mag3':>8} {'RMS':>8} {'Profile':>20}")

for frame_start in range(0, N - fft_size, hop):
    segment = samples[frame_start:frame_start + fft_size]
    rms = math.sqrt(sum(s*s for s in segment) / len(segment))

    # Hanning window + FFT
    windowed = [segment[i] * (0.5 - 0.5 * math.cos(2*math.pi*i/(fft_size-1)))
                for i in range(fft_size)]
    fft_r = [0+0j] * fft_size
    for k in range(fft_size // 2 + 1):
        for n in range(fft_size):
            fft_r[k] += windowed[n] * cmath.exp(-2j*math.pi*k*n/fft_size)

    mag = [abs(fft_r[k]) for k in range(fft_size // 2)]
    freq_res = rate / fft_size

    # Top 3 peaks
    peaks = []
    for k in range(2, len(mag) - 2):
        if mag[k] > mag[k-1] and mag[k] > mag[k+1]:
            peaks.append((mag[k], k * freq_res))
    peaks.sort(reverse=True)

    # Characterize: narrowband (single peak) vs wideband
    if peaks:
        top_mag = peaks[0][0]
        total_energy = sum(m*m for m in mag)
        top3_energy = sum(peaks[i][0]**2 for i in range(min(3, len(peaks))))
        concentration = top3_energy / total_energy if total_energy > 0 else 0
        if concentration > 0.5:
            profile = "narrowband"
        elif concentration > 0.2:
            profile = "mixed"
        else:
            profile = "wideband"
    else:
        profile = "silence"

    ms = start_ms + frame_start / rate * 1000
    p = peaks[:3] + [(0, 0)] * (3 - min(3, len(peaks)))
    print(f"{ms:9.0f}ms {p[0][1]:7.0f}Hz {p[0][0]:7.0f} "
          f"{p[1][1]:7.0f}Hz {p[1][0]:7.0f} "
          f"{p[2][1]:7.0f}Hz {p[2][0]:7.0f} "
          f"{rms:7.0f}  {profile}")

# ---- Autocorrelation to find periodicity ----
print()
print("=== Autocorrelation (baud rate detection) per 500ms block ===")
for block_ms in range(int(start_ms), int(end_ms) - 500, 500):
    bs = int((block_ms - start_ms) / 1000 * rate)
    be = min(bs + int(0.5 * rate), N)
    block = samples[bs:be]
    bn = len(block)

    # Compute autocorrelation for lags corresponding to baud rates 2000-4000
    # lag = rate / baud => 2.0 to 4.0 samples
    # But autocorrelation at such small lags isn't useful
    # Instead, look at the envelope autocorrelation

    # Compute envelope via analytic signal (Hilbert approx)
    # Simple: magnitude of complex baseband at a few carrier guesses
    best_peak_lag = 0
    best_peak_val = 0
    best_carrier = 0

    for carrier in [1600, 1646, 1680, 1800, 1829, 1920, 2000]:
        # Mix down
        bb = []
        for i in range(bn):
            t = (bs + i) / rate
            lo = cmath.exp(-2j * math.pi * carrier * ((bs + i + start_idx) / rate))
            bb.append(block[i] * lo)

        # Low-pass (simple average over ~4 samples)
        filt = []
        for i in range(bn):
            s = 0+0j
            c = 0
            for j in range(max(0, i-2), min(bn, i+3)):
                s += bb[j]
                c += 1
            filt.append(s / c)

        # Envelope
        env = [abs(f) for f in filt]
        mean_env = sum(env) / len(env)
        if mean_env < 10:
            continue

        # Autocorrelation of envelope
        max_lag = min(20, bn // 2)
        ac = []
        for lag in range(1, max_lag):
            s = sum((env[i] - mean_env) * (env[i+lag] - mean_env)
                    for i in range(bn - lag))
            ac.append(s / (bn - lag))

        # Find first peak in autocorrelation (indicates symbol period)
        for lag in range(1, len(ac) - 1):
            if ac[lag] > ac[lag-1] and ac[lag] > ac[lag+1] and ac[lag] > best_peak_val:
                best_peak_val = ac[lag]
                best_peak_lag = lag + 1
                best_carrier = carrier

    if best_peak_lag > 0:
        implied_baud = rate / best_peak_lag
        print(f"  {block_ms}ms: best envelope period = {best_peak_lag} samples"
              f" ({implied_baud:.0f} baud) at carrier {best_carrier}Hz")
    else:
        print(f"  {block_ms}ms: no clear periodicity")

# ---- Zero-crossing rate ----
print()
print("=== Zero-crossing rate per 100ms block ===")
for block_ms in range(int(start_ms), int(end_ms), 100):
    bs = int((block_ms - start_ms) / 1000 * rate)
    be = min(bs + int(0.1 * rate), N)
    crossings = 0
    for i in range(bs + 1, be):
        if (samples[i] >= 0) != (samples[i-1] >= 0):
            crossings += 1
    # Zero-crossing rate = 2 * fundamental frequency (roughly)
    est_freq = crossings / (2 * (be - bs) / rate)
    rms = math.sqrt(sum(samples[i]**2 for i in range(bs, be)) / (be - bs))
    print(f"  {block_ms:5d}ms: {crossings:4d} crossings, est freq ~{est_freq:.0f} Hz, RMS={rms:.0f}")
