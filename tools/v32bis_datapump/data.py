"""Data-mode TX/RX pipeline for the V.32bis datapump.

Implements the full chain for BER/SER measurement:

    random bits  →  scramble  →  DataModeEncoder  →  pulse-shape
                 →  passband  →  channel impairments
                 →  matched filter  →  carrier phase tracking
                 →  decode_symbols_hard  →  descramble
                 →  BER / SER measurement

The receiver uses a decision-directed carrier phase loop over the
full TCM constellation.  No startup is needed — this models the
data-transfer phase after convergence.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from tools.v32bis_ref.data_mode import (
    DataModeEncoder,
    decode_data_symbols_hard,
    decode_data_symbols_soft,
)
from tools.v32bis_ref.rx_frontend import matched_filter, passband_to_baseband
from tools.v32bis_ref.scrambler import Descrambler, Scrambler, scrambler_tap
from tools.v32bis_ref.tx_passband import PassbandWaveform, baseband_to_passband
from tools.v32bis_ref.tx_waveform import BasebandWaveform, rrc_taps

from .tx import TxConfig


@dataclass(frozen=True)
class DataWaveform:
    """Data-mode transmit waveform and its metadata."""

    tx_bits: list[int]           # original payload bits (pre-scramble)
    scrambled_bits: list[int]    # scrambled bits fed to TCM encoder
    tx_points: list[tuple[int, int]]  # TCM-encoded (I,Q) constellation points
    baseband: BasebandWaveform
    passband: PassbandWaveform
    bit_rate: int
    n_symbols: int


@dataclass(frozen=True)
class DataRecovery:
    """Data-mode receive result."""

    rx_points: list[tuple[float, float]]   # matched-filter output sampled at symbol instants
    decoded_scrambled: list[int]           # decoded bits before descrambling
    decoded_bits: list[int]                # descrambled payload bits
    carrier_phase_error_rad: float         # residual phase after tracking loop
    agc_gain: float                        # block gain applied before decoding
    equalizer_taps: list[complex]          # concatenated feedforward/feedback taps
    equalizer_training_symbols: int        # number of known target symbols used


@dataclass(frozen=True)
class DataResult:
    """Full result returned by :meth:`~tools.v32bis_datapump.V32bisDatapump.run_data`.

    Attributes
    ----------
    bit_rate:
        The data rate used (bps).
    n_symbols:
        Number of data symbols transmitted.
    n_bits:
        Number of payload data bits transmitted.
    bit_errors:
        Number of bit positions where TX and RX differ.
    ber:
        Bit error rate = ``bit_errors / n_bits``.
    symbol_errors:
        Number of symbol groups where at least one bit was wrong.
    ser:
        Symbol error rate = ``symbol_errors / n_symbols``.
    snr_db:
        Channel SNR applied (None if no AWGN added).
    carrier_phase_error_rad:
        Residual carrier phase error after the tracking loop (radians).
    waveform:
        The transmitted :class:`DataWaveform` (available for inspection /
        further impairment tests).
    recovery:
        The received :class:`DataRecovery`.
    """

    bit_rate: int
    n_symbols: int
    n_bits: int
    bit_errors: int
    ber: float
    symbol_errors: int
    ser: float
    snr_db: float | None
    carrier_phase_error_rad: float
    waveform: DataWaveform
    recovery: DataRecovery


# ---------------------------------------------------------------------------
# TX
# ---------------------------------------------------------------------------

def generate_data_waveform(
    config: TxConfig,
    *,
    bit_rate: int,
    n_symbols: int,
    seed: int = 42,
    calling_party: bool = True,
) -> DataWaveform:
    """Generate a data-mode passband waveform.

    Parameters
    ----------
    config:
        Transmitter configuration (carrier, sample rate, pulse shape).
    bit_rate:
        One of 4800, 7200, 9600, 12000, 14400 bps.
    n_symbols:
        Number of data symbols to generate.
    seed:
        RNG seed for the random payload bits.
    calling_party:
        Selects the scrambler polynomial direction per V.32bis §2.2.
    """
    enc = DataModeEncoder(bit_rate)
    bpg = enc.bits_per_group
    n_bits = n_symbols * bpg

    rng = random.Random(seed)
    payload = [rng.randint(0, 1) for _ in range(n_bits)]

    tap = scrambler_tap(calling_party=calling_party, transmit=True)
    scrambled = Scrambler(tap).process_bits(payload)

    tx_points = enc.encode(scrambled)

    taps = rrc_taps(0.5, config.samples_per_symbol, 8)
    upsampled_len = n_symbols * config.samples_per_symbol
    upsampled = [0j] * upsampled_len
    for idx, (i_val, q_val) in enumerate(tx_points):
        upsampled[idx * config.samples_per_symbol] = complex(i_val, q_val)

    bb_samples: list[complex] = [0j] * (upsampled_len + len(taps) - 1)
    for i, sample in enumerate(upsampled):
        if sample == 0j:
            continue
        for j, tap_val in enumerate(taps):
            bb_samples[i + j] += sample * tap_val

    baseband = BasebandWaveform(
        samples=bb_samples,
        taps=taps,
        samples_per_symbol=config.samples_per_symbol,
    )
    passband = baseband_to_passband(
        baseband,
        sample_rate=config.sample_rate,
        carrier_hz=config.carrier_hz,
    )

    return DataWaveform(
        tx_bits=payload,
        scrambled_bits=scrambled,
        tx_points=tx_points,
        baseband=baseband,
        passband=passband,
        bit_rate=bit_rate,
        n_symbols=n_symbols,
    )


# ---------------------------------------------------------------------------
# RX
# ---------------------------------------------------------------------------

def recover_data(
    waveform: PassbandWaveform,
    *,
    matched_filter_taps: list[float],
    n_symbols: int,
    bit_rate: int,
    rx_carrier_hz: float,
    samples_per_symbol: int,
    timing_offset: int = 0,
    phase_gain: float = 0.05,
    calling_party: bool = True,
    enable_equalizer: bool = False,
    training_points: list[tuple[int, int]] | None = None,
    equalizer_feedforward_taps: int = 9,
    equalizer_feedback_taps: int = 4,
    equalizer_step_size: float = 0.0008,
    equalizer_training_symbols: int = 128,
) -> DataRecovery:
    """Demodulate a data-mode passband waveform.

    Uses a decision-directed carrier phase loop: at each symbol the nearest
    constellation point is found and the angle between the received point and
    that nearest point drives a first-order loop filter.

    Parameters
    ----------
    waveform:
        Received passband waveform.
    matched_filter_taps:
        RRC matched-filter taps (same as used at TX).
    n_symbols:
        Expected number of symbols to recover.
    bit_rate:
        One of 4800, 7200, 9600, 12000, 14400 bps.
    rx_carrier_hz:
        Down-conversion carrier frequency (Hz).  A small offset from the TX
        carrier exercises carrier-tracking.
    samples_per_symbol:
        Oversampling ratio (must match the TX).
    timing_offset:
        Integer sample offset for symbol strobe (0 = first sample after
        matched-filter settling).
    phase_gain:
        Loop gain for the carrier phase correction (radians per error unit).
    calling_party:
        Must match the value used at TX to select the correct descrambler.
    """
    # Down-convert and matched-filter
    bb = passband_to_baseband(waveform, carrier_hz=rx_carrier_hz)
    filtered = matched_filter(bb, matched_filter_taps)

    # Build a fast nearest-point lookup for the carrier loop discriminant.
    # We need the full constellation as a list of complex points.
    from tools.v32bis_tcm import _RATE_INFO
    _const_map = _RATE_INFO[bit_rate][1]  # dict: codeword → (I, Q)
    _const_points = [complex(i, q) for (i, q) in _const_map.values()]
    _avg_const_power = sum((p.real * p.real + p.imag * p.imag) for p in _const_points) / len(_const_points)

    # Sample at symbol instants, then apply a block AGC to normalize amplitude.
    start = len(matched_filter_taps) - 1 + timing_offset
    raw_samples: list[complex] = []

    for k in range(n_symbols):
        idx = start + k * samples_per_symbol
        if idx >= len(filtered):
            raw_samples.append(0j)
            continue
        raw_samples.append(filtered[idx])

    observed_power = sum((s.real * s.real + s.imag * s.imag) for s in raw_samples if s != 0j)
    observed_count = sum(1 for s in raw_samples if s != 0j)
    if observed_count > 0 and observed_power > 0.0:
        agc_gain = math.sqrt((_avg_const_power * observed_count) / observed_power)
    else:
        agc_gain = 1.0
    scaled_samples = [sample * agc_gain for sample in raw_samples]

    phase_est = 0.0
    phase_corrected: list[complex] = []
    for raw in scaled_samples:
        corrected = raw * complex(math.cos(-phase_est), math.sin(-phase_est))
        phase_corrected.append(corrected)
        # Decision-directed phase error: imag(corrected × conj(nearest_point)).
        # Find the nearest constellation point and use it as the phase reference.
        if phase_gain > 0.0 and abs(corrected) > 1e-9:
            nearest = min(_const_points,
                          key=lambda p: (corrected.real - p.real) ** 2 + (corrected.imag - p.imag) ** 2)
            mag = abs(nearest)
            if mag > 1e-9:
                phase_error = (corrected * nearest.conjugate()).imag / (mag * mag)
                phase_est += phase_gain * phase_error

    eq_taps: list[complex] = [1.0 + 0.0j]
    eq_training_symbols = 0
    symbol_samples = phase_corrected
    if enable_equalizer and bit_rate != 4800 and phase_corrected:
        if equalizer_feedforward_taps < 1:
            raise ValueError("equalizer_feedforward_taps must be positive")
        if equalizer_feedback_taps < 1:
            raise ValueError("equalizer_feedback_taps must be positive")
        if equalizer_step_size <= 0.0:
            raise ValueError("equalizer_step_size must be positive")

        ff_half = equalizer_feedforward_taps // 2
        padded = [0j] * ff_half + phase_corrected + [0j] * ff_half
        ff_taps = [0j] * equalizer_feedforward_taps
        ff_taps[ff_half] = 1.0 + 0.0j
        fb_taps = [0j] * equalizer_feedback_taps
        past_targets = [0j] * equalizer_feedback_taps
        symbol_samples = []

        if training_points is None:
            known_targets: list[complex] = []
        else:
            known_targets = [complex(i_val, q_val) for i_val, q_val in training_points[:n_symbols]]
        train_count = min(equalizer_training_symbols, len(phase_corrected), len(known_targets))
        eq_training_symbols = train_count

        for index in range(len(phase_corrected)):
            ff_window = padded[index:index + equalizer_feedforward_taps]
            ff_output = sum(tap * sample for tap, sample in zip(ff_taps, ff_window))
            fb_output = sum(tap * sample for tap, sample in zip(fb_taps, past_targets))
            output = ff_output - fb_output
            if index < train_count:
                target = known_targets[index]
            else:
                target = min(
                    _const_points,
                    key=lambda p: (output.real - p.real) ** 2 + (output.imag - p.imag) ** 2,
                )
            error = target - output
            for tap_index in range(equalizer_feedforward_taps):
                ff_taps[tap_index] += equalizer_step_size * error * ff_window[tap_index].conjugate()
            for tap_index in range(equalizer_feedback_taps):
                fb_taps[tap_index] -= equalizer_step_size * error * past_targets[tap_index].conjugate()
            past_targets = [target] + past_targets[:-1]
            symbol_samples.append(output)

        eq_taps = ff_taps + fb_taps

    # Use soft-decision Viterbi decoding for the trellis rates.
    rx_points_float = [(s.real, s.imag) for s in symbol_samples]
    if bit_rate == 4800:
        decoded_scrambled = decode_data_symbols_hard(rx_points_float, bit_rate)
    else:
        decoded_scrambled = decode_data_symbols_soft(rx_points_float, bit_rate)

    # Descramble.
    # In a full duplex link the answer side descrambles what the call side sent
    # (both use tap 17). In this loopback simulation the same-party receiver
    # descrambles its own transmission, so we use the *transmit* tap of the
    # opposite party (which equals the transmit tap of this party — both are
    # tap 17 for V.32bis per §2.2 Table 3).
    rx_tap = scrambler_tap(calling_party=not calling_party, transmit=False)
    bpg = DataModeEncoder(bit_rate).bits_per_group
    n_bits = n_symbols * bpg
    decoded_bits = Descrambler(rx_tap).process_bits(decoded_scrambled[:n_bits])

    return DataRecovery(
        rx_points=rx_points_float,
        decoded_scrambled=decoded_scrambled[:n_bits],
        decoded_bits=decoded_bits,
        carrier_phase_error_rad=phase_est,
        agc_gain=agc_gain,
        equalizer_taps=eq_taps,
        equalizer_training_symbols=eq_training_symbols,
    )


# ---------------------------------------------------------------------------
# BER measurement
# ---------------------------------------------------------------------------

def measure_ber(
    tx_bits: list[int],
    decoded_bits: list[int],
    *,
    bits_per_group: int,
) -> tuple[int, float, int, float]:
    """Compare TX and RX bit streams and return (bit_errors, ber, sym_errors, ser).

    Parameters
    ----------
    tx_bits:
        Original payload bits (pre-scramble, pre-encode).
    decoded_bits:
        Decoded payload bits (post-descramble).
    bits_per_group:
        Number of bits per symbol group (used to compute SER).

    Returns
    -------
    (bit_errors, ber, symbol_errors, ser)
    """
    n = len(tx_bits)
    bit_errors = sum(a != b for a, b in zip(tx_bits, decoded_bits[:n]))
    ber = bit_errors / n if n > 0 else 0.0

    n_symbols = n // bits_per_group
    sym_errors = 0
    for s in range(n_symbols):
        start = s * bits_per_group
        if any(tx_bits[start + k] != decoded_bits[start + k] for k in range(bits_per_group)):
            sym_errors += 1
    ser = sym_errors / n_symbols if n_symbols > 0 else 0.0

    return bit_errors, ber, sym_errors, ser
