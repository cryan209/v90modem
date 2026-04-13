#!/usr/bin/env python3
"""
V.90 / V.34 / V.92 Phase 3 upstream demodulator (analogue modem TX).

The analogue (call) modem transmits this Phase 3 sequence on the upstream
channel (V.90 §9.3.2, Figure 5/V.90; waveform definitions in V.90 §8.3):

    S  (128T)            — as defined in §10.1.3.7/V.34
    S̄  (16T)             — as defined in §10.1.3.7/V.34 (inverted polarity)
    [MD]                 — optional marker; as defined in §10.1.3.5/V.34
    S  (128T)            — repeat
    S̄  (16T)             — repeat
    PP (288T = 6×48T)    — phase probing; as defined in §10.1.3.6/V.34
    TRN (≥512T)          — QPSK training; as defined in §10.1.3.8/V.34
                           GPA scrambler (equation 7-2/V.34), 4-point constellation
    J_a (variable)       — DIL descriptor (V.90-specific, §8.3.1/V.90)
    SCR (variable)       — binary ones via GPA, no scrambler re-init (§8.3.5/V.90)

Signal names S, MD, PP, TRN are identical waveforms to V.34 (V.90 §8.3.2–8.3.6
each say "As defined in 10.1.3.x/V.34").  J_a and SCR are V.90 additions.

In V.34-only calls the sequence ends at J/J' (§11.3.1.1/V.34) instead.

GPC scrambler (digital modem TX, V.34 §7): 1 + x^{-5}  + x^{-23}
GPA scrambler (analogue modem TX, V.34 §7): 1 + x^{-18} + x^{-23}

For V.92 upstream Phase 3, the analogue modem transmits:

    Ru / uR             — 2-point pattern (§9.5.2.1.1 / §8.5.5)
    [MD]                — optional marker
    TRN1u (≥2040T)      — GPA-scrambled ±L_U (§8.5.7)
    J_a (variable)      — 24 ones + DIL descriptors (§8.5.4)

V.92 Ja does NOT use the V.34/V.90 4-point J modulation.  It uses the same
2-point modulation as TRN1u, is GPA-scrambled, and is differentially encoded
from the final symbol of the preceding TRN1u (§8.5.4/V.92).

V.90 symbol-rate constraints (§6.2/V.90):
  Mandatory: 3200 baud.  Optional: 3000, 3429.
  Rates 2400, 2743, 2800 are NOT used in V.90 upstream.

Default target: USR-Message-V92QC R channel, 9.476–13.280 s.

Usage:
    python3 tools/v34_phase3_demod.py [options]
    python3 tools/v34_phase3_demod.py --sweep          # try all V.34 candidates
    python3 tools/v34_phase3_demod.py --sym-rate 3200 --carrier 1829
    python3 tools/v34_phase3_demod.py --no-stft        # skip STFT (faster, symbol only)
"""
from __future__ import annotations
import argparse
import sys
import wave
from pathlib import Path

import numpy as np
from scipy import signal as sp

# ---------------------------------------------------------------------------
# V.34 Table 2 — valid (symbol_rate, carrier_hz) pairs
# ---------------------------------------------------------------------------
V34_CARRIERS: list[tuple[int, int]] = [
    (2400, 1600), (2400, 1800),
    (2743, 1646), (2743, 1829),
    (2800, 1680), (2800, 1867),
    (3000, 1800), (3000, 2000),
    (3200, 1829), (3200, 1920),
    (3429, 1959),
]

# V.90 upstream uses only the higher symbol-rate subset (§6.2/V.90).
V90_CARRIERS: list[tuple[int, int]] = [
    (3000, 1800), (3000, 2000),
    (3200, 1829), (3200, 1920),
    (3429, 1959),
]

# Scrambler tap pairs for the ones-check  (V.34 §7)
# Delays are expressed as recurrence taps on the serial bit stream.
GPC_TAPS = (5,  23)   # digital modem TX
GPA_TAPS = (18, 23)   # analogue modem TX

# Upsample target rate before demodulation — gives ~15 sps at 3200 baud
_UPSAMPLE_FS = 48000


# ---------------------------------------------------------------------------
# WAV I/O
# ---------------------------------------------------------------------------
def load_wav_channel(path: Path, channel: int,
                     t0: float, t1: float) -> tuple[np.ndarray, int]:
    """Return float64 PCM samples from one channel of a WAV file."""
    with wave.open(str(path), 'r') as w:
        fs  = w.getframerate()
        nc  = w.getnchannels()
        sw  = w.getsampwidth()
        raw = w.readframes(w.getnframes())
    samp = np.frombuffer(raw, dtype=np.int16 if sw == 2 else np.uint8)
    if nc > 1:
        samp = samp.reshape(-1, nc)[:, channel]
    samp = samp.astype(np.float64)
    return samp[int(t0 * fs): int(t1 * fs)], fs


# ---------------------------------------------------------------------------
# Carrier / symbol-rate estimation
# ---------------------------------------------------------------------------
def carrier_candidates(protocol: str) -> list[tuple[int, int]]:
    if protocol == 'v90':
        return V90_CARRIERS
    return V34_CARRIERS


def estimate_best_carrier(sig: np.ndarray, fs: int,
                          protocol: str = 'auto') -> tuple[int, int]:
    """
    Score each V.34 (sym_rate, carrier) pair by mean PSD energy in the
    band [carrier ± 0.45 × sym_rate].  Returns the best pair.
    """
    nfft  = min(len(sig), 65536)
    win   = np.blackman(nfft)
    spec  = np.abs(np.fft.rfft(sig[:nfft] * win, n=nfft)) ** 2
    freqs = np.fft.rfftfreq(nfft, 1.0 / fs)

    candidates = carrier_candidates(protocol)
    best_score, best = -1.0, candidates[-1]
    for sym_rate, fc in candidates:
        bw   = sym_rate * 0.9
        mask = (freqs >= fc - bw / 2) & (freqs <= fc + bw / 2)
        if not mask.any():
            continue
        score = float(np.mean(spec[mask]))
        if score > best_score:
            best_score, best = score, (sym_rate, fc)
    return best


def phase_occupancy(symbols: np.ndarray, window: int = 128,
                    hop: int = 16) -> tuple[np.ndarray, np.ndarray]:
    times = []
    occ = []
    if len(symbols) < window:
        return np.asarray(times), np.asarray(occ)
    for i in range(0, len(symbols) - window + 1, hop):
        wsy = symbols[i:i + window]
        ph = np.angle(wsy)
        bins = ((ph + np.pi) * (8.0 / (2.0 * np.pi))).astype(int) % 8
        h = np.bincount(bins, minlength=8)
        occ_bins = int(np.sum(h >= max(3, int(0.04 * window))))
        times.append(i + window // 2)
        occ.append(occ_bins)
    return np.asarray(times), np.asarray(occ)


def chunk_bits(bits: str, width: int = 128) -> str:
    return "\n".join(bits[i:i + width] for i in range(0, len(bits), width))


# ---------------------------------------------------------------------------
# Demodulation pipeline (with upsampling for timing accuracy)
# ---------------------------------------------------------------------------
def refine_carrier_frequency(sig: np.ndarray, fs: int,
                              fc_approx: float,
                              search_hz: float = 50.0) -> float:
    """
    Refine carrier frequency estimate using zero-padded FFT peak interpolation.

    Looks within ±search_hz of fc_approx for the exact spectral peak.
    Uses parabolic interpolation on the log-magnitude spectrum for sub-bin
    accuracy.
    """
    nfft = max(65536, 2 ** int(np.ceil(np.log2(len(sig) * 8))))
    win = np.blackman(len(sig))
    spec = np.abs(np.fft.rfft(sig * win, n=nfft))
    freqs = np.fft.rfftfreq(nfft, 1.0 / fs)

    mask = (freqs >= fc_approx - search_hz) & (freqs <= fc_approx + search_hz)
    if not mask.any():
        return fc_approx

    masked_spec = spec[mask]
    masked_freqs = freqs[mask]
    pk_idx = int(np.argmax(masked_spec))

    # Parabolic interpolation on log-magnitude for sub-bin accuracy
    if 0 < pk_idx < len(masked_spec) - 1:
        a = float(np.log(masked_spec[pk_idx - 1] + 1e-30))
        b = float(np.log(masked_spec[pk_idx] + 1e-30))
        c = float(np.log(masked_spec[pk_idx + 1] + 1e-30))
        delta = 0.5 * (a - c) / (a - 2 * b + c + 1e-30)
        df = float(masked_freqs[1] - masked_freqs[0])
        return float(masked_freqs[pk_idx]) + delta * df

    return float(masked_freqs[pk_idx])


def demodulate_to_baseband(sig: np.ndarray, fs: int,
                           sym_rate: int, fc: float,
                           n_sps: int = 8) -> np.ndarray:
    """
    Upsample → bandpass → refine carrier → mix to baseband → lowpass → resample.

    Upsampling to 48 kHz first gives ~15 sps at 3200 baud vs the native
    2.5 sps at 8 kHz, yielding much better timing and phase recovery.

    The carrier frequency is refined using FFT peak interpolation on the
    upsampled signal before downconversion, to avoid residual frequency
    offset that causes constellation rotation.

    Returns complex IQ at sym_rate × n_sps samples/s.
    """
    # 1. Upsample raw signal to _UPSAMPLE_FS (e.g. 8 kHz → 48 kHz)
    if fs != _UPSAMPLE_FS:
        g_up    = np.gcd(_UPSAMPLE_FS, fs)
        sig_up  = sp.resample_poly(sig, _UPSAMPLE_FS // g_up, fs // g_up)
        fs_work = _UPSAMPLE_FS
    else:
        sig_up  = sig
        fs_work = fs

    # 2. Bandpass centred on the carrier (zero-phase to avoid group delay)
    lo  = max(100.0,          fc - sym_rate * 0.55)
    hi  = min(fs_work / 2 - 100, fc + sym_rate * 0.55)
    sos = sp.butter(6, [lo, hi], btype='band', fs=fs_work, output='sos')
    bp  = sp.sosfiltfilt(sos, sig_up)

    # 3. Refine carrier frequency using the bandpassed signal
    fc_refined = refine_carrier_frequency(bp, fs_work, fc, search_hz=80.0)

    # 4. Mix to baseband (complex downconversion) using refined carrier
    t   = np.arange(len(bp)) / fs_work
    iq  = bp * np.exp(-1j * 2 * np.pi * fc_refined * t)

    # 5. Low-pass to ≈ sym_rate / 2 (zero-phase)
    sos2 = sp.butter(8, sym_rate * 0.52, btype='low', fs=fs_work, output='sos')
    iq   = sp.sosfiltfilt(sos2, iq)

    # 6. Resample to exactly n_sps samples per symbol
    target_fs = sym_rate * n_sps
    g         = np.gcd(target_fs, fs_work)
    iq        = sp.resample_poly(iq, target_fs // g, fs_work // g)
    return iq


def find_best_timing_offset(iq: np.ndarray, n_sps: int,
                            cpr_bw: float = 0.01
                            ) -> tuple[np.ndarray, int, float]:
    """
    Try all n_sps sampling phases; return (best_symbols, best_offset, best_evm).
    """
    best_evm    = float('inf')
    best_syms   = None
    best_offset = 0

    for offset in range(n_sps):
        raw  = iq[offset::n_sps]
        raw  = normalize_power(raw)
        syms = recover_carrier_phase(raw, loop_bw=cpr_bw)
        syms = normalize_power(syms)
        evm  = qpsk_evm(syms)
        if evm < best_evm:
            best_evm    = evm
            best_syms   = syms
            best_offset = offset

    return best_syms, best_offset, best_evm


def sample_symbols(iq: np.ndarray, n_sps: int) -> np.ndarray:
    """Decimate IQ to 1 sample/symbol (centre of eye)."""
    return iq[n_sps // 2 :: n_sps]


def recover_carrier_phase(symbols: np.ndarray,
                          loop_bw: float = 0.01) -> np.ndarray:
    """
    Decision-directed QPSK Costas loop.
    Removes residual carrier phase offset / slow drift.

    loop_bw=0.01 tracks up to ~10 Hz residual offset at typical symbol rates
    without overcorrecting on TRN.
    """
    phase = freq = 0.0
    alpha = loop_bw
    beta  = alpha ** 2 / 4.0
    out   = np.zeros(len(symbols), dtype=complex)

    for n, s in enumerate(symbols):
        c        = s * np.exp(-1j * phase)
        out[n]   = c
        decided  = (np.sign(c.real) + 1j * np.sign(c.imag)) * (1 / np.sqrt(2))
        err      = float((c * np.conj(decided)).imag)
        freq    += beta * err
        phase   += freq + alpha * err

    return out


def normalize_power(syms: np.ndarray) -> np.ndarray:
    rms = float(np.sqrt(np.mean(np.abs(syms) ** 2)))
    return syms / (rms + 1e-12)


def qpsk_evm(symbols: np.ndarray) -> float:
    """RMS EVM vs nearest of the 4 QPSK reference points (normalised)."""
    ref  = np.array([1+1j, -1+1j, -1-1j, 1-1j], dtype=complex) / np.sqrt(2)
    errs = [np.abs(s - ref[int(np.argmin(np.abs(ref - s)))]) ** 2
            for s in symbols]
    return float(np.sqrt(np.mean(errs)))


# ---------------------------------------------------------------------------
# STFT-based sub-sequence detector (no symbol recovery needed)
# ---------------------------------------------------------------------------
def stft_detect_phases(sig: np.ndarray, fs: int,
                       sym_rate: int, fc: float,
                       window_t: float = 0.020,
                       hop_t:    float = 0.005) -> dict:
    """
    Detect V.34 Phase 3 sub-sequences without symbol demodulation.

    S-signal signature  : carrier at fc is 10–50× the band average power.
                          The S signal has a strong DC component in baseband
                          which maps to a spectral line at fc in the passband.
    PP-signal signature : 48T autocorrelation of the IQ magnitude envelope.
    TRN signature       : carrier at fc is 1–3× band average (no strong line).

    Works even with a severely distorted phone-line channel.

    Returns a dict with per-frame scores and labels.
    """
    nwin   = int(window_t * fs)
    nhop   = int(hop_t   * fs)
    nfft   = max(nwin, 1024)    # zero-pad for finer frequency resolution
    win    = np.hanning(nwin)

    n_frames = max(1, (len(sig) - nwin) // nhop + 1)
    f_axis   = np.fft.rfftfreq(nfft, 1.0 / fs)
    df       = fs / nfft

    S_score  = np.zeros(n_frames)
    PP_score = np.zeros(n_frames)
    TRN_score= np.zeros(n_frames)
    t_axis   = np.zeros(n_frames)

    # Band: exclude narrow carrier window and edges
    bw_half = sym_rate * 0.5
    f_lo    = max(50.0, fc - bw_half * 1.1)
    f_hi    = min(fs / 2 - 50, fc + bw_half * 1.1)
    band_mask = (f_axis >= f_lo) & (f_axis <= f_hi)

    # Carrier bin
    car_bin = int(round(fc / df))
    car_bw  = max(1, int(round(40 / df)))    # ±40 Hz window

    # Sideband bins for S-signal: fc ± sym_rate/2
    sb_lo_bin = int(round((fc - sym_rate / 2) / df))
    sb_hi_bin = int(round((fc + sym_rate / 2) / df))
    sb_bw     = max(1, int(round(40 / df)))

    # Downsampled IQ for PP ACF (resample to ~4×sym_rate for efficiency)
    # PP ACF is computed in a separate pass on a time-domain IQ envelope
    # Use the full signal → compute envelope magnitude at symbol rate

    # Pre-compute baseband envelope for PP ACF (simple downmix at 8 sps)
    fs_bb    = sym_rate * 4        # 4× oversampled baseband
    g_bb     = np.gcd(fs_bb, fs)
    # Downconvert to complex baseband for the PP ACF detector
    t_full   = np.arange(len(sig)) / fs
    iq_full  = sig.astype(complex) * np.exp(-1j * 2 * np.pi * fc * t_full)
    sos_bb   = sp.butter(4, sym_rate * 0.52, btype='low', fs=fs, output='sos')
    iq_full  = sp.sosfilt(sos_bb, iq_full)
    iq_ds    = sp.resample_poly(iq_full, fs_bb // g_bb, fs // g_bb)
    mag_ds   = np.abs(iq_ds)
    lag_pp   = 4 * 48     # lag = 48 symbols at 4× oversampled

    for i in range(n_frames):
        start = i * nhop
        end   = start + nwin
        if end > len(sig):
            break
        frame     = sig[start:end] * win
        spec      = np.abs(np.fft.rfft(frame, n=nfft)) ** 2
        t_axis[i] = (start + nwin / 2) / fs

        band_pow = spec[band_mask]
        if band_pow.sum() < 1e-30:
            S_score[i] = PP_score[i] = TRN_score[i] = 0.0
            continue
        mean_band = float(band_pow.mean())

        # S score: carrier power / band mean
        lo_c  = max(0, car_bin - car_bw)
        hi_c  = min(len(spec), car_bin + car_bw + 1)
        car_pow = float(spec[lo_c:hi_c].max())
        S_score[i] = car_pow / (mean_band + 1e-30)

        # PP score: ACF at lag 48T using the magnitude envelope
        # Map frame centre time to IQ_ds index
        ds_centre = int(round(t_axis[i] * fs_bb))
        ds_hw     = int(round(window_t * fs_bb / 2))
        ds0       = max(0, ds_centre - ds_hw)
        ds1       = min(len(mag_ds), ds_centre + ds_hw)
        if ds1 - ds0 > lag_pp + 10:
            m    = mag_ds[ds0:ds1]
            m    = m - m.mean()
            norm = float(np.dot(m, m)) + 1e-30
            n_m  = len(m)
            if n_m > lag_pp:
                PP_score[i] = float(
                    np.dot(m[:n_m - lag_pp], m[lag_pp:]) / norm)

        # TRN score: spectral flatness of signal band
        bp = band_pow[band_pow > 0]
        if len(bp) > 1:
            TRN_score[i] = float(
                np.exp(np.mean(np.log(bp + 1e-30))) / (mean_band + 1e-30))

    # Classify frames:
    #   S  → carrier power ratio > 10× (strong DC = S signal in baseband)
    #   PP → ACF at 48T > 0.15 and not S
    #   T  → everything else (TRN / higher-order data)
    labels = []
    for i in range(n_frames):
        if t_axis[i] == 0:
            labels.append('?')
        elif S_score[i] > 10.0:
            labels.append('S')
        elif PP_score[i] > 0.15:
            labels.append('P')
        else:
            labels.append('T')

    return {
        't':          t_axis[:n_frames],
        'S_score':    S_score[:n_frames],
        'PP_score':   PP_score[:n_frames],
        'TRN_score':  TRN_score[:n_frames],
        'label':      labels[:n_frames],
        'f':          f_axis,
        'Sxx':        np.zeros((n_frames, 1)),   # placeholder
    }


def stft_timeline(result: dict, sym_rate: int,
                  min_dur_s: float = 0.015) -> list[tuple[str, float, float]]:
    """
    Collapse per-frame labels into (label, start_s, end_s) runs.
    Runs shorter than min_dur_s are merged into the adjacent run to suppress
    spurious single-frame detections (e.g. one QAM symbol that happens to
    cluster near the real axis and triggers the S carrier threshold).
    """
    t      = result['t']
    labels = result['label']
    if not labels:
        return []

    # Build raw runs
    raw: list[tuple[str, float, float]] = []
    cur = labels[0]
    t0  = float(t[0])
    for i in range(1, len(labels)):
        if labels[i] != cur:
            raw.append((cur, t0, float(t[i - 1])))
            cur = labels[i]
            t0  = float(t[i])
    raw.append((cur, t0, float(t[-1])))

    # Fill gaps: if a short run is flanked by the same label, merge into it.
    # This removes spurious single-frame transitions (e.g. one TRN frame
    # between two PP frames due to window edge effects).
    # We do NOT change the label of a run — only fill gaps between identical labels.
    changed = True
    while changed:
        changed = False
        out = []
        i   = 0
        while i < len(raw):
            lbl, ts, te = raw[i]
            dur = te - ts
            # Check if this is a short run flanked by identical labels
            prev_lbl = out[-1][0] if out else None
            next_lbl = raw[i + 1][0] if i + 1 < len(raw) else None
            if (dur < min_dur_s and
                    prev_lbl is not None and
                    next_lbl is not None and
                    prev_lbl == next_lbl):
                # Fill the gap: extend previous run to include next run
                _, next_ts, next_te = raw[i + 1]
                out[-1] = (prev_lbl, out[-1][1], next_te)
                i += 2
                changed = True
            else:
                out.append((lbl, ts, te))
                i += 1
        raw = out

    return raw


# ---------------------------------------------------------------------------
# Differential dibit extraction (V.34 §9.4)
# ---------------------------------------------------------------------------
def differential_dibits(symbols: np.ndarray) -> np.ndarray:
    """
    Extract differential dibits from complex symbol stream.

    Computes z[n] * conj(z[n-1]), maps the phase difference to quadrant:
        dibit 0:   0° ± 45°   (no phase change)
        dibit 1:  90° ± 45°   (CCW quarter turn)
        dibit 2: 180° ± 45°   (half turn)
        dibit 3: 270° ± 45°   (CW quarter turn)

    Returns array of length len(symbols)-1 with values in {0,1,2,3}.
    """
    conj_prod = symbols[1:] * np.conj(symbols[:-1])
    angles = np.angle(conj_prod)  # in (-π, π]
    # Map to [0, 2π)
    angles = angles % (2 * np.pi)
    # Quadrant boundaries at 45°, 135°, 225°, 315°
    dibits = np.zeros(len(angles), dtype=np.int32)
    dibits[(angles >= np.pi / 4) & (angles < 3 * np.pi / 4)] = 1
    dibits[(angles >= 3 * np.pi / 4) & (angles < 5 * np.pi / 4)] = 2
    dibits[(angles >= 5 * np.pi / 4) & (angles < 7 * np.pi / 4)] = 3
    return dibits


def dibits_to_serial_bits(dibits: np.ndarray) -> np.ndarray:
    """
    Serialise dibit stream to bit pairs: dibit d → (d & 1, (d >> 1) & 1).
    Returns array of length 2 * len(dibits).
    """
    bits = np.empty(len(dibits) * 2, dtype=np.uint8)
    bits[0::2] = dibits & 1
    bits[1::2] = (dibits >> 1) & 1
    return bits


# ---------------------------------------------------------------------------
# Scrambler recurrence check on serial bit stream
# ---------------------------------------------------------------------------
def scrambler_recurrence_score(bits: np.ndarray,
                               taps: tuple[int, int]) -> float:
    """
    Check the scrambler self-synchronising recurrence on a serial bit stream.

    For GPA (taps=(18,23)): b[n] XOR b[n-18] XOR b[n-23] should == 1
    for TRN (scrambled all-ones).

    Works directly on the scrambled (received) bitstream — no descrambler
    initialisation needed.
    """
    t1, t2 = taps
    n0 = max(t1, t2)
    if len(bits) <= n0:
        return 0.0
    b = bits[n0:]
    s = bits[n0 - t1: len(bits) - t1]
    t = bits[n0 - t2: len(bits) - t2]
    return float(np.mean((b ^ s ^ t) == 1))


# ---------------------------------------------------------------------------
# Phase 3 sub-sequence detectors (symbol domain)
# ---------------------------------------------------------------------------
def s_signal_score(symbols: np.ndarray, window: int = 64) -> np.ndarray:
    """
    Per-symbol score for S / S̄ (V.34 §10.1.3.7).

    Phase 3 S is a CONSTANT carrier — the same constellation point repeated
    every baud (differential dibit = 0 for every symbol).  S̄ is the same
    constant carrier phase-shifted 180° from S.  The S→S̄ boundary is a
    single dibit-2 transition, then dibit-0 again for 16T.

    Phase 4 S is different (180° reversals, dibit=2 per baud).

    Score ≈ fraction of dibit-0 (no phase change) in a sliding window.
    A high score means constant carrier → S or S̄.
    """
    n = len(symbols)
    scores = np.zeros(n)
    if n < 3:
        return scores

    dibits = differential_dibits(symbols)
    is_zero = (dibits == 0).astype(np.float64)
    hw = window // 2

    for i in range(hw, n - hw - 1):
        seg = is_zero[max(0, i - hw): min(len(is_zero), i + hw)]
        scores[i] = float(np.mean(seg))

    return scores


def pp_signal_score(symbols: np.ndarray, window: int = 288) -> np.ndarray:
    """
    Per-symbol score for PP (V.34 §10.1.3.6).

    PP is 6 repetitions of a 48-symbol sequence; its normalised
    autocorrelation at lag 48 should be close to 1.

    Score = ACF(mag, 48) / ACF(mag, 0) averaged over a window.
    """
    mag    = np.abs(symbols)
    n      = len(symbols)
    scores = np.zeros(n)
    hw     = window // 2
    hop    = 12

    for i in range(hw, n - hw, hop):
        seg = mag[i - hw : i + hw]
        if len(seg) < window:
            continue
        seg0    = seg - seg.mean()
        acf     = np.correlate(seg0, seg0, mode='full')
        centre  = len(acf) // 2
        if acf[centre] <= 0:
            continue
        score   = float(acf[centre + 48] / acf[centre]) if centre + 48 < len(acf) else 0.0
        i0, i1  = max(0, i - hop // 2), min(n, i + hop // 2)
        scores[i0:i1] = max(0.0, score)

    return scores


def trn_signal_score(symbols: np.ndarray, window: int = 128,
                     hop: int = 16) -> np.ndarray:
    """
    Per-symbol score for TRN using scrambler recurrence on differential dibits.

    TRN = GPA-scrambled all-ones.  The self-synchronising recurrence
    b[n] XOR b[n-18] XOR b[n-23] == 1 holds on the serial bit stream
    derived from differential dibits.  Score ≈ 1.0 for TRN, ≈ 0.5 for random.
    """
    n = len(symbols)
    scores = np.zeros(n)
    if n < 3:
        return scores

    dibits = differential_dibits(symbols)
    # Try both bit orderings and take the best
    serial_a = dibits_to_serial_bits(dibits)
    serial_b = np.empty_like(serial_a)
    serial_b[0::2] = (dibits >> 1) & 1
    serial_b[1::2] = dibits & 1

    hw = window // 2
    for i in range(hw, n - hw, hop):
        d_start = max(0, i - hw)
        d_end = min(len(dibits), i + hw)
        seg_a = serial_a[d_start * 2: d_end * 2]
        seg_b = serial_b[d_start * 2: d_end * 2]
        sc_a = scrambler_recurrence_score(seg_a, GPA_TAPS)
        sc_b = scrambler_recurrence_score(seg_b, GPA_TAPS)
        sc = max(sc_a, sc_b)
        # Also check GPC
        sc_a2 = scrambler_recurrence_score(seg_a, GPC_TAPS)
        sc_b2 = scrambler_recurrence_score(seg_b, GPC_TAPS)
        sc = max(sc, sc_a2, sc_b2)
        i0, i1 = max(0, i - hop // 2), min(n, i + hop // 2)
        scores[i0:i1] = sc

    return scores


# ---------------------------------------------------------------------------
# Ideal TRN generation (V.90 §8.3.6 / V.34 §10.1.3.8)
# ---------------------------------------------------------------------------
def generate_gpa_trn(n_syms: int) -> np.ndarray:
    """
    Generate the ideal V.90/V.34 TRN symbol sequence.

    TRN = GPA scrambler (1 + x^{-5} + x^{-23}, V.34 §7 eq. 7-2) applied to
    an all-ones bit stream, scrambler register initialised to zero (V.34
    §10.1.3.8).  Bits are mapped to the 4-point QPSK constellation:

        (b0, b1) → (1−2·b0 + j·(1−2·b1)) / √2

    This is the analogue modem's TRN in V.90 (§8.3.6) and the call modem's
    TRN in V.34 (§11.3.1.1).

    Returns complex QPSK symbols, length n_syms.
    """
    n_bits  = n_syms * 2
    reg     = np.zeros(23, dtype=np.uint8)   # GPA shift register [b_{n-1}..b_{n-23}]
    bits    = np.empty(n_bits, dtype=np.uint8)
    for i in range(n_bits):
        # GPA: b_out = 1 XOR b_{n-5} XOR b_{n-23}
        b = 1 ^ reg[4] ^ reg[22]
        bits[i] = b
        reg[1:]  = reg[:-1]
        reg[0]   = b
    # Map pairs → QPSK: (b0=0→+1, b0=1→−1), same for b1 → imaginary
    I = (1 - 2 * bits[0::2].astype(np.float64))
    Q = (1 - 2 * bits[1::2].astype(np.float64))
    return (I + 1j * Q) / np.sqrt(2.0)


# ---------------------------------------------------------------------------
# LMS adaptive equalizer trained on known TRN  (V.90 §8.3.6)
# ---------------------------------------------------------------------------
def lms_equalizer_trn(iq_sps: np.ndarray,
                      n_sps: int,
                      trn_start_sym: int,
                      n_trn_syms: int,
                      n_taps: int = 31,
                      mu: float = 0.005,
                      n_rot: int | None = None
                      ) -> tuple[np.ndarray, int, int]:
    """
    Train a fractionally-spaced LMS equalizer on the known TRN region.

    iq_sps          — complex baseband at n_sps samples/symbol (HP-filtered,
                      carrier removed)
    n_sps           — samples per symbol
    trn_start_sym   — symbol index where TRN begins in iq_sps
    n_trn_syms      — number of TRN symbols to train on
    n_taps          — equalizer length in samples (should be odd)
    mu              — LMS step size
    n_rot           — if not None, pre-rotate iq_sps by n_rot × 90° before
                      training (used to resolve QPSK rotational ambiguity)

    Returns:
        eq_syms  — equalized symbols for the full iq_sps (1/sym)
        best_rot — rotational offset (0-3 × 90°) applied before training
        best_off — timing offset (0..n_sps-1) that gave lowest TRN MSE
    """
    ideal_trn = generate_gpa_trn(n_trn_syms + n_taps)   # extra for filter delay

    # Pre-rotate if requested
    if n_rot is not None:
        iq_work = iq_sps * np.exp(1j * np.pi / 2 * n_rot)
    else:
        iq_work = iq_sps

    delay = n_taps // 2   # filter group delay in samples

    best_mse  = float('inf')
    best_rot  = 0
    best_off  = 0
    best_w    = None

    rot_range = range(4) if n_rot is None else [n_rot]

    for rot in rot_range:
        iq_r = iq_work * np.exp(1j * np.pi / 2 * rot) if n_rot is None else iq_work

        for off in range(n_sps):
            # Build the sample stream at the training timing offset
            # We work directly on the oversampled IQ — fractionally-spaced equalizer
            trn_start_samp = trn_start_sym * n_sps + off

            # Initialise equalizer taps centred at delay
            w = np.zeros(n_taps, dtype=complex)
            w[delay] = 1.0 + 0j

            mse_acc = 0.0
            n_mse   = 0

            # Training loop over TRN symbols
            for k in range(n_trn_syms):
                samp_idx = trn_start_samp + k * n_sps
                # Extract n_taps samples centred at samp_idx
                i0 = samp_idx - delay
                i1 = i0 + n_taps
                if i0 < 0 or i1 > len(iq_r):
                    continue
                x   = iq_r[i0:i1]
                y   = np.dot(w.conj(), x)
                d   = ideal_trn[k]                  # desired symbol
                e   = d - y
                step = mu / (1e-6 + float(np.vdot(x, x).real))
                w  += step * e.conj() * x
                mse_acc += float(abs(e) ** 2)
                n_mse   += 1

            mse = mse_acc / max(n_mse, 1)

            if mse < best_mse:
                best_mse = mse
                best_rot = rot
                best_off = off
                # Save best taps
                best_w = w.copy()

    if best_w is None:
        raise RuntimeError("TRN equalizer failed to converge on any timing/rotation hypothesis")

    # Apply equalizer to entire signal with best params
    iq_best = iq_work * np.exp(1j * np.pi / 2 * best_rot) if n_rot is None else iq_work
    n_out   = (len(iq_best) - best_off) // n_sps
    eq_syms = np.zeros(n_out, dtype=complex)
    for k in range(n_out):
        samp_idx = best_off + k * n_sps
        i0 = samp_idx - delay
        i1 = i0 + n_taps
        if i0 < 0:
            # Pad beginning with zeros
            pad = -i0
            x   = np.concatenate([np.zeros(pad, dtype=complex), iq_best[:i1]])
        elif i1 > len(iq_best):
            break
        else:
            x = iq_best[i0:i1]
        eq_syms[k] = np.dot(best_w.conj(), x)

    eq_syms = eq_syms / (np.sqrt(np.mean(np.abs(eq_syms) ** 2)) + 1e-12)
    return eq_syms, best_rot, best_off


# ---------------------------------------------------------------------------
# TRN descrambler check (V.34 §7 / §10.1.3.8)
# ---------------------------------------------------------------------------
def syms_to_bits(symbols: np.ndarray) -> np.ndarray:
    """
    Map QPSK symbols → 2-bit pairs [I1, I2] per symbol.
    I1_n = (Re(s) > 0),  I2_n = (Im(s) > 0).
    Serialised as I1_0, I2_0, I1_1, I2_1, ...
    """
    bits = np.empty(len(symbols) * 2, dtype=np.uint8)
    bits[0::2] = (symbols.real > 0).astype(np.uint8)
    bits[1::2] = (symbols.imag > 0).astype(np.uint8)
    return bits


def gpc_ones_score(bits: np.ndarray, taps: tuple[int, int]) -> float:
    """
    Fraction of positions n ≥ max(taps) where
        bits[n] XOR bits[n−t1] XOR bits[n−t2] == 1.

    For GPC-scrambled all-ones (TRN from call modem) this should be ~1.0.
    Random data gives ~0.5.
    """
    t1, t2 = taps
    n0     = max(t1, t2)
    if len(bits) <= n0:
        return 0.0
    b = bits[n0:]
    s = bits[n0 - t1 : len(bits) - t1]
    t = bits[n0 - t2 : len(bits) - t2]
    return float(np.mean((b ^ s ^ t) == 1))


def check_trn_region(symbols: np.ndarray) -> dict:
    """
    Check a symbol region for TRN using differential dibit extraction.

    Tries all 4 QPSK rotational ambiguities, both bit polarities,
    and both GPC/GPA scramblers.  Uses differential dibits (phase change
    between consecutive symbols) rather than absolute quadrant, which is
    the correct V.34 encoding.

    Also falls back to absolute-quadrant extraction for comparison.
    Returns dict with best score and matched parameters.
    """
    best: dict = {'score': 0.0, 'rotation': 0,
                  'inverted': False, 'scrambler': '?',
                  'method': 'none'}

    # Method 1: Differential dibit extraction (correct for V.34)
    if len(symbols) >= 2:
        dibits = differential_dibits(symbols)
        serial_bits = dibits_to_serial_bits(dibits)
        for inv in (False, True):
            b = (1 - serial_bits) if inv else serial_bits
            for name, taps in (('GPC', GPC_TAPS), ('GPA', GPA_TAPS)):
                sc = scrambler_recurrence_score(b, taps)
                if sc > best['score']:
                    best = {'score': sc, 'rotation': 0,
                            'inverted': inv, 'scrambler': name,
                            'method': 'differential'}
            # Also try swapped bit order: (d>>1)&1 first, then d&1
            b_swap = np.empty_like(serial_bits)
            b_swap[0::2] = (dibits >> 1) & 1
            b_swap[1::2] = dibits & 1
            if inv:
                b_swap = 1 - b_swap
            for name, taps in (('GPC', GPC_TAPS), ('GPA', GPA_TAPS)):
                sc = scrambler_recurrence_score(b_swap, taps)
                if sc > best['score']:
                    best = {'score': sc, 'rotation': 0,
                            'inverted': inv, 'scrambler': name,
                            'method': 'differential-swap'}

    # Method 2: Absolute quadrant with rotation (legacy fallback)
    for rot in range(4):
        s    = symbols * np.exp(1j * np.pi / 2 * rot)
        bits = syms_to_bits(s)
        for inv in (False, True):
            b = (1 - bits) if inv else bits
            for name, taps in (('GPC', GPC_TAPS), ('GPA', GPA_TAPS)):
                sc = gpc_ones_score(b, taps)
                if sc > best['score']:
                    best = {'score': sc, 'rotation': rot,
                            'inverted': inv, 'scrambler': name,
                            'method': 'absolute'}
    return best


# ---------------------------------------------------------------------------
# Human-readable timeline
# ---------------------------------------------------------------------------
LABEL_NAME = {'S': 'S / S̄',       'P': 'PP (6×48T)',
              'T': 'TRN/J_a/SCR', '?': 'unknown'}


def classify_symbols(s_sc: np.ndarray, pp_sc: np.ndarray,
                     s_thresh: float = 0.80,
                     pp_thresh: float = 0.12) -> list[str]:
    """
    Classify each symbol as S (constant carrier), P (probing), or T (TRN/data).

    s_thresh: fraction of dibit-0 in window to qualify as S.  Default 0.80
              (Phase 3 S is constant carrier → nearly all dibit-0).
    """
    labels = []
    for i in range(len(s_sc)):
        if s_sc[i] >= s_thresh:
            labels.append('S')
        elif pp_sc[i] >= pp_thresh:
            labels.append('P')
        else:
            labels.append('T')
    return labels


def runs_from_labels(labels: list[str]) -> list[tuple[str, int]]:
    if not labels:
        return []
    runs: list[tuple[str, int]] = []
    cur, cnt = labels[0], 1
    for c in labels[1:]:
        if c == cur:
            cnt += 1
        else:
            runs.append((cur, cnt))
            cur, cnt = c, 1
    runs.append((cur, cnt))
    return runs


def boolean_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """Return contiguous True runs as half-open index pairs [start, end)."""
    runs: list[tuple[int, int]] = []
    i = 0
    while i < len(mask):
        if not bool(mask[i]):
            i += 1
            continue
        j = i + 1
        while j < len(mask) and bool(mask[j]):
            j += 1
        runs.append((i, j))
        i = j
    return runs


def bridge_short_false_gaps(mask: np.ndarray, max_gap: int = 1) -> np.ndarray:
    """
    Fill short False gaps between True regions.

    This stabilizes occupancy/run detection when a single weak analysis window
    momentarily breaks an otherwise continuous Phase 3 region.
    """
    if len(mask) == 0 or max_gap <= 0:
        return mask.copy()

    out = mask.astype(bool).copy()
    i = 0
    while i < len(out):
        if out[i]:
            i += 1
            continue
        j = i + 1
        while j < len(out) and not out[j]:
            j += 1
        if i > 0 and j < len(out) and out[i - 1] and out[j] and (j - i) <= max_gap:
            out[i:j] = True
        i = j
    return out


def v92_ru_score(symbols: np.ndarray, window: int = 18) -> np.ndarray:
    """
    Score the V.92 Ru/uR 6-symbol two-point pattern.

    Ru  = {same,same,same,flip,flip,flip} differentially
    uR  = complement of Ru.
    """
    n = len(symbols)
    scores = np.zeros(n)
    if n < window + 1:
        return scores

    diffs = np.angle(symbols[1:] * np.conj(symbols[:-1]))
    same = (np.cos(diffs) >= 0.0).astype(np.uint8)
    period = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
    comp = 1 - period
    hw = window // 2

    for i in range(hw, len(same) - hw):
        seg = same[i - hw:i + hw]
        if len(seg) != window:
            continue
        idx = np.arange(window) % 6
        m0 = float(np.mean(seg == period[idx]))
        m1 = float(np.mean(seg == comp[idx]))
        scores[i] = max(m0, m1)
    return scores


def detect_v92_regions(symbols: np.ndarray, sym_rate: int) -> dict:
    """
    Heuristic V.92 Phase 3 region finder.

    Uses phase occupancy as a robust discriminator:
      occupied bins <= 3  -> 2-point Ru/TRN1u-like
      occupied bins >= 6  -> Ja-like transition away from pure 2-point TRN1u
    """
    occ_window = 128
    occ_idx, occ = phase_occupancy(symbols)
    ru_sc = v92_ru_score(symbols)
    out = {
        'occ_idx': occ_idx,
        'occ': occ,
        'ru_score': ru_sc,
        'ru_run': None,
        'phase3_start': None,
        'phase3_source': 'none',
        'two_point_run': None,
        'trn1u_run': None,
        'ja_start': None,
    }
    if len(occ) == 0:
        return out

    thr = 0.75
    mask = bridge_short_false_gaps(ru_sc >= thr, max_gap=6)
    if np.any(mask):
        s0 = int(np.argmax(mask))
        s1 = s0
        while s1 + 1 < len(mask) and mask[s1 + 1]:
            s1 += 1
        out['ru_run'] = (s0, s1)

    low = bridge_short_false_gaps(occ <= 3, max_gap=1)
    low_runs = boolean_runs(low)
    min_low_windows = 6
    chosen_low: tuple[int, int] | None = None

    if out['ru_run'] is not None:
        ru0, ru1 = out['ru_run']
        for run0, run1 in low_runs:
            if (run1 - run0) < min_low_windows:
                continue
            low0 = int(occ_idx[run0] - occ_window // 2)
            low1 = int(occ_idx[run1 - 1] + occ_window // 2)
            if low1 >= ru0 and low0 <= ru1 + occ_window:
                chosen_low = (run0, run1)
                break

    if chosen_low is None:
        for run0, run1 in low_runs:
            if (run1 - run0) >= min_low_windows:
                chosen_low = (run0, run1)
                break

    if chosen_low is None and low_runs:
        chosen_low = max(low_runs, key=lambda run: run[1] - run[0])

    if chosen_low is not None:
        run0, run1 = chosen_low
        two_point_start = max(0, int(occ_idx[run0] - occ_window // 2))
        two_point_end = min(len(symbols) - 1, int(occ_idx[run1 - 1] + occ_window // 2))
        out['two_point_run'] = (two_point_start, two_point_end)
        out['trn1u_run'] = (two_point_start, two_point_end)
        if out['ru_run'] is not None:
            ru0, ru1 = out['ru_run']
            out['phase3_start'] = ru0
            out['phase3_source'] = 'ru_run'
            trn_start = max(two_point_start, ru1 + 1)
            if trn_start <= two_point_end:
                out['trn1u_run'] = (trn_start, two_point_end)
        else:
            out['phase3_start'] = two_point_start
            out['phase3_source'] = 'two_point_run'

        for k in range(run1, len(occ)):
            if occ[k] >= 6:
                out['ja_start'] = int(occ_idx[k])
                break

    return out


def print_timeline(sym_rate: int, runs: list[tuple[str, int]]) -> None:
    T_ms = 1000.0 / sym_rate
    print(f"\n{'Offset':>10}  {'Segment':<14}  {'Symbols':>7}  {'Duration':>10}")
    print(f"{'(ms)':>10}  {'':<14}  {'':>7}  {'(ms)':>10}")
    print('-' * 52)
    t_ms = 0.0
    for ch, cnt in runs:
        dur = cnt * T_ms
        print(f"{t_ms:10.1f}  {LABEL_NAME.get(ch, ch):<14}  {cnt:7d}  {dur:10.1f}")
        t_ms += dur


# ---------------------------------------------------------------------------
# Expected V.34 Phase 3 durations (for comparison)
# ---------------------------------------------------------------------------
def print_expected_durations(sym_rate: int, protocol: str = 'v90') -> None:
    T_ms  = 1000.0 / sym_rate
    if protocol == 'v92':
        print(f"\n  V.92 §9.5.2.1 / V.92 §8.5  analogue-modem Phase 3 expected segments"
              f"  (T = {T_ms:.3f} ms @ {sym_rate} baud):")
        rows = [
            ('Ru',     384,        'V.92 §8.5.5, 2-point'),
            ('uR',      24,        'V.92 §8.5.5, inverted'),
            ('[MD]',   'variable', 'optional marker'),
            ('TRN1u', '≥2040',     'GPA-scrambled ±L_U  (V.92 §8.5.7)'),
            ('J_a',   'variable',  '24 ones + DIL descriptors  (V.92 §8.5.4)'),
        ]
    else:
        print(f"\n  V.90 §9.3.2 / V.90 §8.3  analogue-modem Phase 3 expected segments"
              f"  (T = {T_ms:.3f} ms @ {sym_rate} baud):")
        rows = [
            ('S',      128,        'V.90 §8.3.4 = §10.1.3.7/V.34'),
            ('S̄',      16,         'V.90 §8.3.4 = §10.1.3.7/V.34, inverted polarity'),
            ('[MD]',   'variable', 'optional; length from INFO₁a bits 18:24 × 35 ms  (V.90 §8.3.2)'),
            ('S',      128,        'repeat'),
            ('S̄',      16,         'repeat'),
            ('PP',     288,        '6 × 48T  (V.90 §8.3.3 = §10.1.3.6/V.34)'),
            ('TRN',   '≥512',      'GPA-scrambled, 4-point  (V.90 §8.3.6 = §10.1.3.8/V.34)'),
            ('J_a',   'variable',  'DIL descriptor repetitions, V.90-specific  (V.90 §8.3.1)'),
            ('SCR',   'variable',  'binary ones via GPA, no re-init  (V.90 §8.3.5)'),
        ]
    for label, t_count, note in rows:
        if isinstance(t_count, int):
            dur = f"{t_count * T_ms:.1f} ms"
            sym = f"{t_count}T"
        else:
            dur = ''
            sym = t_count
        print(f"    {label:6s}  {sym:>8}  ({dur:>10})  {note}")


def print_v92_report(sym_rate: int, report: dict) -> None:
    T_ms = 1000.0 / sym_rate
    print(f"\n=== V.92 Phase 3 Heuristic Report ===")
    if report['phase3_start'] is not None:
        print(f"  Phase 3 start:      symbol {report['phase3_start']}  "
              f"({report['phase3_start'] * T_ms:.1f} ms)  "
              f"source={report['phase3_source']}")
    else:
        print("  Phase 3 start:      not found")
    if report['ru_run'] is not None:
        ru0, ru1 = report['ru_run']
        print(f"  Ru/uR candidate: symbols {ru0}–{ru1}  "
              f"({(ru1 - ru0 + 1) * T_ms:.1f} ms)")
    else:
        print("  Ru/uR candidate: not found")
    if report['two_point_run'] is not None:
        t0, t1 = report['two_point_run']
        print(f"  2-point run:      symbols {t0}–{t1}  "
              f"({(t1 - t0) * T_ms:.1f} ms)")
    else:
        print("  2-point run:      not found")
    if report['trn1u_run'] is not None:
        t0, t1 = report['trn1u_run']
        print(f"  TRN1u-like run:   symbols {t0}–{t1}  "
              f"({(t1 - t0) * T_ms:.1f} ms)")
    else:
        print("  TRN1u-like run:  not found")
    if report['ja_start'] is not None:
        print(f"  Ja-like transition begins near symbol {report['ja_start']}  "
              f"({report['ja_start'] * T_ms:.1f} ms)")
    else:
        print("  Ja-like transition: not found")


def bitdump_text(symbols: np.ndarray,
                 wav: Path,
                 t0: float,
                 t1: float,
                 sym_rate: int,
                 fc: float,
                 protocol: str,
                 phase3_start_sym: int | None = None,
                 phase3_start_source: str | None = None,
                 ja_candidate_sym: int | None = None,
                 trn_run: tuple[int, int] | None = None) -> str:
    """
    Emit a validator-friendly text dump with the same label style used by the
    existing Ja tools.
    """
    lines: list[str] = []
    lines.append(f"file={wav}")
    lines.append(
        f"window_s={t0:.6f}-{t1:.6f} protocol={protocol} "
        f"carrier_hz={fc:.3f} sym_rate={sym_rate:.3f}"
    )
    lines.append(f"symbols={len(symbols)}")
    if phase3_start_sym is not None:
        lines.append(
            f"phase3_start_sym={phase3_start_sym} "
            f"phase3_start_ms={(1000.0 * phase3_start_sym / sym_rate):.3f} "
            f"phase3_start_abs_ms={(t0 * 1000.0) + (1000.0 * phase3_start_sym / sym_rate):.3f} "
            f"phase3_start_source={phase3_start_source or 'unknown'}"
        )
    if ja_candidate_sym is not None:
        lines.append(
            f"ja_candidate_sym={ja_candidate_sym} "
            f"ja_candidate_ms={(1000.0 * ja_candidate_sym / sym_rate):.3f}"
        )
    if trn_run is not None:
        trn0, trn1 = trn_run
        lines.append(
            f"trn_run_start_sym={trn0} trn_run_end_sym={trn1} "
            f"trn_run_start_ms={(1000.0 * trn0 / sym_rate):.3f} "
            f"trn_run_end_ms={(1000.0 * trn1 / sym_rate):.3f} "
            f"trn_run_start_abs_ms={(t0 * 1000.0) + (1000.0 * trn0 / sym_rate):.3f} "
            f"trn_run_end_abs_ms={(t0 * 1000.0) + (1000.0 * trn1 / sym_rate):.3f}"
        )

    if len(symbols) >= 2:
        # Differential dibit extraction (V.34 §9.4)
        dibits = differential_dibits(symbols)
        serial_bits = dibits_to_serial_bits(dibits)

        lines.append(f"diff_dibits({len(dibits)}): {''.join(str(int(v)) for v in dibits)}")
        lines.append(f"serial_bits({len(serial_bits)}):")
        lines.append(chunk_bits("".join(str(int(v)) for v in serial_bits), 64))

        # Scrambler recurrence check on the serial bit stream
        gpa_sc = scrambler_recurrence_score(serial_bits, GPA_TAPS)
        gpc_sc = scrambler_recurrence_score(serial_bits, GPC_TAPS)
        lines.append(f"gpa_recurrence_score={gpa_sc:.4f}")
        lines.append(f"gpc_recurrence_score={gpc_sc:.4f}")

        # Legacy absolute-quadrant representations (for comparison)
        states = (((np.angle(symbols) + np.pi) / (np.pi / 2.0)).astype(int)) & 3
        d = (states[1:] - states[:-1]) & 3
        nat_bits = "".join(f"{int(v):02b}" for v in d)
        gray_map = {0: "00", 1: "01", 2: "11", 3: "10"}
        gray_bits = "".join(gray_map[int(v)] for v in d)
        lines.append(f"state_symbols({len(states)}): {''.join(str(int(v)) for v in states)}")
        lines.append(f"diff_symbols({len(d)}): {''.join(str(int(v)) for v in d)}")
        lines.append(f"nat_bits({len(nat_bits)}):")
        lines.append(chunk_bits(nat_bits, 64))
        lines.append(f"gray_bits({len(gray_bits)}):")
        lines.append(chunk_bits(gray_bits, 64))

    return "\n".join(lines) + "\n"


def output_stem(wav: Path, sym_rate: int, fc: float) -> str:
    name = wav.stem.replace(" ", "_")
    return f"p3_{name}_{sym_rate}bd_{int(fc)}Hz"


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description='V.34 Phase 3 upstream demodulator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument('--wav', type=Path,
        default=Path('gough-lui-v90-v92-modem-sounds/'
                     'USR-Message-V92QC-bong-bong-bong.wav'),
        metavar='FILE',
        help='Stereo WAV file (default: USR-Message-V92QC)')
    ap.add_argument('--ch',  type=int, default=1,
        help='WAV channel index: 0=L, 1=R  (default: 1=R = calling modem TX)')
    ap.add_argument('--t0',  type=float, default=9.476, metavar='SEC',
        help='Segment start time in seconds (default: 9.476)')
    ap.add_argument('--t1',  type=float, default=13.280, metavar='SEC',
        help='Segment end time in seconds (default: 13.280)')
    ap.add_argument('--sym-rate', type=int, default=None, metavar='BAUD',
        help='Force V.34 symbol rate (skip auto-detect)')
    ap.add_argument('--carrier',  type=float, default=None, metavar='HZ',
        help='Force carrier frequency in Hz (skip auto-detect)')
    ap.add_argument('--protocol', choices=('auto', 'v34', 'v90', 'v92'),
        default='auto',
        help='Interpretation mode for later analysis stages (default: auto)')
    ap.add_argument('--sweep', action='store_true',
        help='Try every V.34 (sym_rate, carrier) pair and print EVM table')
    ap.add_argument('--no-stft', action='store_true',
        help='Skip STFT-based sub-sequence detection (faster)')
    ap.add_argument('--force-qpsk', action='store_true',
        help='Run the legacy QPSK/LMS/Ja path even when the capture looks V.92-like')
    ap.add_argument('--out-dir', type=Path, default=Path('tools/dumps'),
        help='Output directory for IQ/timeline text files')
    ap.add_argument('--s-thresh',  type=float, default=0.80,
        help='S/S̄ detection threshold (default 0.80, fraction of dibit-0 in window)')
    ap.add_argument('--pp-thresh', type=float, default=0.12,
        help='PP detection threshold (default 0.12)')
    args = ap.parse_args(argv)
    wav_name_upper = args.wav.name.upper()
    hinted_protocol = 'v92' if 'V92' in wav_name_upper else None
    display_protocol = args.protocol if args.protocol != 'auto' else (hinted_protocol or 'v90')

    # ── Load signal ─────────────────────────────────────────────────────────
    print(f"Loading  {args.wav}  ch={args.ch}  "
          f"t={args.t0:.3f}–{args.t1:.3f} s")
    sig, fs = load_wav_channel(args.wav, args.ch, args.t0, args.t1)
    print(f"  {len(sig)} samples @ {fs} Hz  ({len(sig)/fs:.3f} s)  "
          f"RMS = {np.sqrt(np.mean(sig**2)):.1f}")

    # ── Parameter selection ──────────────────────────────────────────────────
    if args.sym_rate and args.carrier:
        sym_rate = args.sym_rate
        fc       = float(args.carrier)
        print(f"  Parameters forced: sym_rate={sym_rate}  fc={fc:.0f} Hz")
    else:
        est_protocol = 'v90' if args.protocol == 'v90' else 'auto'
        sym_rate, fc = estimate_best_carrier(sig, fs, est_protocol)
        if args.sym_rate: sym_rate = args.sym_rate
        if args.carrier:  fc       = float(args.carrier)
        print(f"  Auto-detected:     sym_rate={sym_rate}  fc={fc:.0f} Hz")

    # ── STFT-based sub-sequence detection (default: always run) ─────────────
    if not args.no_stft:
        print(f"\n── STFT-based Phase 3 sub-sequence detection ──")
        print(f"   sym_rate={sym_rate} baud  fc={fc:.0f} Hz")
        stft = stft_detect_phases(sig, fs, sym_rate, fc)
        runs = stft_timeline(stft, sym_rate)

        T_ms = 1000.0 / sym_rate
        print(f"\n  {'Time':>8}  {'Label':>6}  {'S_score':>8}  "
              f"{'PP_score':>8}  {'TRN_flat':>8}")
        print('  ' + '-' * 48)
        for i in range(len(stft['t'])):
            if i % 5 == 0:  # print every 5th frame to reduce output
                print(f"  {stft['t'][i]:8.3f}  {stft['label'][i]:>6}  "
                      f"{stft['S_score'][i]:8.2f}  "
                      f"{stft['PP_score'][i]:8.2f}  "
                      f"{stft['TRN_score'][i]:8.3f}")

        print(f"\n=== STFT Phase 3 Timeline ===")
        print(f"    {'Start':>8}  {'End':>8}  {'Duration':>10}  Segment")
        print('    ' + '-' * 42)
        for lbl, t0r, t1r in runs:
            dur = t1r - t0r
            # Convert to symbols
            n_sym = int(round(dur * sym_rate))
            print(f"    {t0r:8.3f}s  {t1r:8.3f}s  "
                  f"{dur*1000:8.1f} ms  "
                  f"{LABEL_NAME.get(lbl, lbl)}  (~{n_sym} symbols)")

        print_expected_durations(sym_rate, display_protocol)

        # Save STFT result
        args.out_dir.mkdir(parents=True, exist_ok=True)
        stem = output_stem(args.wav, sym_rate, fc)
        stft_path = args.out_dir / f"{stem}.stft.txt"
        with open(stft_path, 'w') as fh:
            fh.write(f"# V.34 Phase 3 STFT sub-sequence scores\n")
            fh.write(f"# sym_rate={sym_rate}  fc={int(fc)}\n")
            fh.write(f"# columns: time_s  S_score  PP_score  TRN_flat  label\n")
            for i in range(len(stft['t'])):
                fh.write(f"{stft['t'][i]:.4f}  "
                         f"{stft['S_score'][i]:.3f}  "
                         f"{stft['PP_score'][i]:.3f}  "
                         f"{stft['TRN_score'][i]:.3f}  "
                         f"{stft['label'][i]}\n")
        print(f"\n  STFT data → {stft_path}")

    # ── Optional EVM sweep ───────────────────────────────────────────────────
    if args.sweep:
        print(f"\n── EVM sweep across all V.34 (sym_rate, carrier) candidates ──")
        print(f"  {'sym_rate':>9}  {'fc_hz':>6}  {'n_sym':>6}  "
              f"{'offset':>6}  {'QPSK EVM':>9}")
        print('  ' + '-' * 42)
        sweep_results = []
        for sr, f in carrier_candidates('v90' if args.protocol == 'v90' else 'auto'):
            try:
                iq_r  = demodulate_to_baseband(sig, fs, sr, float(f))
                syms, offset, evm = find_best_timing_offset(iq_r, 8)
                sweep_results.append((evm, sr, f, syms))
                marker = (' ← best' if sweep_results and
                          evm == min(r[0] for r in sweep_results) else '')
                print(f"  {sr:9d}  {f:6d}  {len(syms):6d}  "
                      f"{offset:6d}  {evm:9.4f}{marker}")
            except Exception as e:
                print(f"  {sr:9d}  {f:6d}  error: {e}")
        if sweep_results:
            best_row = min(sweep_results, key=lambda x: x[0])
            sym_rate, fc = best_row[1], float(best_row[2])
            print(f"\n  Selecting best EVM: sym_rate={sym_rate}  fc={fc:.0f} Hz")

    # ── Demodulate to oversampled baseband ──────────────────────────────────
    N_SPS = 8
    print(f"\nDemodulating: sym_rate={sym_rate} baud  fc={fc:.0f} Hz  "
          f"{N_SPS} sps  (raw → 48kHz upsample)")
    iq_r = demodulate_to_baseband(sig, fs, sym_rate, fc, N_SPS)

    # HP-filter to remove carrier leakage (4.8× power in DC bin otherwise)
    fs_bb  = sym_rate * N_SPS
    sos_hp = sp.butter(4, 150.0, btype='high', fs=fs_bb, output='sos')
    iq_r   = sp.sosfiltfilt(sos_hp, iq_r)

    syms, best_offset, evm = find_best_timing_offset(iq_r, N_SPS)
    dur_sym = len(syms) * 1000.0 / sym_rate
    T_ms    = 1000.0 / sym_rate
    print(f"  Symbols: {len(syms)}  best_offset={best_offset}/{N_SPS}")
    print(f"  QPSK EVM (no equalizer): {evm:.4f}  ({evm*100:.1f}%)")

    occ_idx, occ = phase_occupancy(syms)
    occ_median = float(np.median(occ)) if len(occ) else 8.0
    likely_v92 = (occ_median <= 3.5)
    effective_protocol = args.protocol
    if effective_protocol == 'auto':
        if hinted_protocol is not None:
            effective_protocol = hinted_protocol
        else:
            effective_protocol = 'v92' if likely_v92 else 'v90'
    print(f"  Phase-occupancy median: {occ_median:.1f} bins  "
          f"(auto protocol → {effective_protocol})")

    if effective_protocol == 'v92' and not args.force_qpsk:
        report = detect_v92_regions(syms, sym_rate)
        print_v92_report(sym_rate, report)

        args.out_dir.mkdir(parents=True, exist_ok=True)
        stem = output_stem(args.wav, sym_rate, fc)
        iq_path = args.out_dir / f"{stem}.iq.txt"
        bit_path = args.out_dir / f"{stem}.bits.txt"
        with open(iq_path, 'w') as fh:
            fh.write("# Phase 3 baseband IQ symbols\n")
            fh.write(f"# protocol=v92-ish sym_rate={sym_rate} fc={int(fc)} "
                     f"t0={args.t0} t1={args.t1}\n")
            fh.write("# columns: sym_idx time_ms I Q ru_score\n")
            ru_sc = report['ru_score']
            for i, s in enumerate(syms):
                score = ru_sc[i] if i < len(ru_sc) else 0.0
                fh.write(f"{i:6d} {i*T_ms:9.3f} {s.real:+.6f} {s.imag:+.6f} {score:.3f}\n")
        bit_path.write_text(
            bitdump_text(
                syms,
                args.wav,
                args.t0,
                args.t1,
                sym_rate,
                fc,
                "v92",
                phase3_start_sym=report['phase3_start'],
                phase3_start_source=report['phase3_source'],
                ja_candidate_sym=report['ja_start'],
            ),
            encoding="ascii",
        )
        print(f"\nOutput:")
        print(f"  IQ symbols  → {iq_path}  ({len(syms)} symbols)")
        print(f"  Bit dump    → {bit_path}  "
              f"(labels: dbpsk_bits_inverted, dbpsk_bits, nat_bits, gray_bits)")
        return 0

    # ── LMS equalizer trained on known GPA TRN (V.90 §8.3.6) ────────────────
    # Use STFT timeline to locate TRN start.  Fall back to 135 ms if STFT
    # was skipped or TRN wasn't found.
    trn_start_ms = 135.0    # default: after S(128T) + S̄(16T) + PP(288T)
    if not args.no_stft:
        # Find the first T (TRN) run that is ≥ 512 symbols from STFT result
        for lbl, t0r, t1r in runs:
            if lbl == 'T' and (t1r - t0r) * sym_rate >= 512:
                trn_start_ms = t0r * 1000.0
                break

    trn_start_sym = int(round(trn_start_ms / 1000.0 * sym_rate))
    # Use up to 2000 TRN symbols for training (≈625 ms); avoids J_a boundary
    n_trn_train   = min(2000, len(syms) - trn_start_sym - 100)

    print(f"\n── LMS Equalizer (trained on GPA TRN, V.90 §8.3.6) ──")
    print(f"  TRN starts at symbol {trn_start_sym}  ({trn_start_ms:.0f} ms)")
    print(f"  Training on {n_trn_train} TRN symbols  ({n_trn_train*T_ms:.0f} ms)")
    print(f"  Equalizer: 31 taps, fractionally-spaced @ {N_SPS} sps/sym, μ=0.005")

    eq_syms, lms_rot, lms_off = lms_equalizer_trn(
        iq_r,
        n_sps           = N_SPS,
        trn_start_sym   = trn_start_sym,
        n_trn_syms      = n_trn_train,
        n_taps          = 31,
        mu              = 0.005,
    )

    evm_eq = qpsk_evm(eq_syms)
    print(f"  QPSK EVM after LMS: {evm_eq:.4f}  ({evm_eq*100:.1f}%)")
    print(f"  Best rotation: {lms_rot}×90°   timing offset: {lms_off}/{N_SPS}")

    # Use equalized symbols for all downstream analysis
    syms = eq_syms

    # ── Phase 3 sub-sequence detection ──────────────────────────────────────
    print(f"\nDetecting Phase 3 sub-sequences (dibit-based) …")
    s_sc   = s_signal_score(syms)
    pp_sc  = pp_signal_score(syms)
    trn_sc = trn_signal_score(syms)

    labels   = classify_symbols(s_sc, pp_sc, args.s_thresh, args.pp_thresh)
    runs_sym = runs_from_labels(labels)

    s_count  = labels.count('S')
    pp_count = labels.count('P')
    t_count  = labels.count('T')
    print(f"  S/S̄ symbols: {s_count:5d}  ({s_count  * T_ms:7.1f} ms)")
    print(f"  PP   symbols: {pp_count:5d}  ({pp_count * T_ms:7.1f} ms)")
    print(f"  TRN  symbols: {t_count:5d}  ({t_count  * T_ms:7.1f} ms)")

    # Report TRN scrambler score for the largest T run
    t_run_start, t_run_end = 0, 0
    in_t, ts = False, 0
    for i, lbl in enumerate(labels):
        if lbl == 'T' and not in_t:
            ts, in_t = i, True
        elif lbl != 'T' and in_t:
            if i - ts > t_run_end - t_run_start:
                t_run_start, t_run_end = ts, i
            in_t = False
    if in_t and len(labels) - ts > t_run_end - t_run_start:
        t_run_start, t_run_end = ts, len(labels)
    t_run_len = t_run_end - t_run_start
    if t_run_len >= 48:
        mid = (t_run_start + t_run_end) // 2
        trn_peak = trn_sc[mid] if mid < len(trn_sc) else 0.0
        print(f"  TRN scrambler score in largest T run: {trn_peak:.4f}"
              f"  (≈1.0 = confirmed TRN, ≈0.5 = random)")

    print(f"\n=== V.90 Phase 3 Sub-Sequence Timeline (LMS-equalized) ===")
    print(f"    sym_rate={sym_rate} baud   T={T_ms:.3f} ms   carrier={fc:.0f} Hz")
    print_timeline(sym_rate, runs_sym)

    print_expected_durations(sym_rate, effective_protocol)

    # ── TRN descrambler check on LMS-equalized symbols ───────────────────────
    best_run = (t_run_start, t_run_end)
    trn_len = t_run_len
    print(f"\n=== TRN Descrambler Check (V.90 §8.3.6 / V.34 §7) ===")
    print(f"  Analogue modem TRN uses GPA (1+x^-18+x^-23) per V.90 §8.3 / §6.5")
    print(f"  Largest TRN run: symbols {best_run[0]}–{best_run[1]}"
          f"  ({trn_len} symbols = {trn_len * T_ms:.1f} ms)")

    if trn_len >= 48:
        trn_syms = syms[best_run[0] : best_run[1]]
        r = check_trn_region(trn_syms)
        sc = r['score']
        print(f"  Best:  {r['scrambler']:<3}  method={r['method']}"
              f"  rotation={r['rotation']}×90°  inverted={r['inverted']}")
        print(f"  Score: {sc:.4f}   "
              f"(random ≈ 0.500 | perfect TRN ≈ 0.950)")
        if sc >= 0.80:
            role = ('analogue modem (V.90 §8.3.6)' if r['scrambler'] == 'GPA'
                    else 'digital modem (V.90 §8.4.5)')
            print(f"\n  ✓  TRN CONFIRMED — {r['scrambler']} scrambler, {role}"
                  f",  rotation={r['rotation']}×90°")
        elif sc >= 0.60:
            print(f"\n  ~  Weak TRN match — likely channel residual ISI")
        else:
            print(f"\n  ✗  TRN not confirmed (score ≈ random)")
    else:
        print(f"  Skipped (run too short)")

    # ── Also check on the known-training region directly ────────────────────
    print(f"\n── Direct GPA check on training region (sym {trn_start_sym}"
          f"–{trn_start_sym + n_trn_train}) ──")
    if n_trn_train >= 48:
        trn_direct = syms[trn_start_sym : trn_start_sym + n_trn_train]
        rd = check_trn_region(trn_direct)
        print(f"  {rd['scrambler']:<3}  method={rd['method']}  rot={rd['rotation']}×90°"
              f"  inv={rd['inverted']}  score={rd['score']:.4f}")

    # ── Write output files ───────────────────────────────────────────────────
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stem = output_stem(args.wav, sym_rate, fc)

    # IQ symbol dump  (gnuplot-friendly)
    iq_path = args.out_dir / f"{stem}.iq.txt"
    bit_path = args.out_dir / f"{stem}.bits.txt"
    with open(iq_path, 'w') as fh:
        fh.write(f"# V.34 Phase 3 baseband IQ symbols\n")
        fh.write(f"# sym_rate={sym_rate}  fc={int(fc)}  "
                 f"t0={args.t0}  t1={args.t1}\n")
        fh.write(f"# columns: sym_idx  time_ms  I  Q  s_score  pp_score  trn_score  label\n")
        dibits_out = differential_dibits(syms) if len(syms) >= 2 else np.array([], dtype=np.int32)
        for i, s in enumerate(syms):
            lbl = labels[i]
            dibit_str = str(int(dibits_out[i])) if i < len(dibits_out) else '-'
            fh.write(f"{i:6d}  {i*T_ms:9.3f}  "
                     f"{s.real:+.6f}  {s.imag:+.6f}  "
                     f"{s_sc[i]:.3f}  {pp_sc[i]:.3f}  {trn_sc[i]:.3f}  "
                     f"{lbl}  d{dibit_str}\n")
    bit_path.write_text(
        bitdump_text(
            syms,
            args.wav,
            args.t0,
            args.t1,
            sym_rate,
            fc,
            "v90",
            trn_run=best_run if best_run[1] > best_run[0] else None,
        ),
        encoding="ascii",
    )
    print(f"\nOutput:")
    print(f"  IQ symbols  → {iq_path}  ({len(syms)} symbols)")
    print(f"  Bit dump    → {bit_path}  "
          f"(labels: dbpsk_bits_inverted, dbpsk_bits, nat_bits, gray_bits)")

    # Segment timeline dump
    tl_path = args.out_dir / f"{stem}.timeline.txt"
    with open(tl_path, 'w') as fh:
        fh.write(f"# V.34 Phase 3 segment timeline\n")
        fh.write(f"# sym_rate={sym_rate}  fc={int(fc)}\n")
        fh.write(f"# columns: offset_ms  segment  n_symbols  duration_ms\n")
        t_ms = 0.0
        for ch, cnt in runs_sym:
            dur = cnt * T_ms
            fh.write(f"{t_ms:10.3f}  {LABEL_NAME.get(ch, ch):<14}  "
                     f"{cnt:7d}  {dur:10.3f}\n")
            t_ms += dur
    print(f"  Timeline    → {tl_path}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
