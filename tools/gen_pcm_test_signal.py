#!/usr/bin/env python3
"""Generate a known reference PCM test signal for the voice-mode fidelity
harness (tools/voice_pcm_fidelity.py): a slow 300-3000 Hz sweep plus a fixed
1 kHz marker tone, at 8 kHz, encoded straight to raw G.711 A-law octets (no
header) so it can be handed unchanged to a modem's AT+VTX voice-mode stream
or to ME_VOICE_TEST_TX_FILE.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.signal import chirp

sys.path.insert(0, str(Path(__file__).resolve().parent))
from g711_codec import linear_to_alaw  # noqa: E402

SR = 8000


def build_signal(duration: float) -> np.ndarray:
    t = np.arange(int(duration * SR)) / SR
    # 1s silence, marker tone, sweep, marker tone, 1s silence — makes
    # alignment/cross-correlation and "did it clip" checks easy to eyeball.
    marker_len = int(1.0 * SR)
    lead_len = int(1.0 * SR)
    sweep_len = len(t) - 2 * marker_len - 2 * lead_len
    if sweep_len <= 0:
        raise SystemExit("--duration too short for lead/marker/sweep layout")

    lead = np.zeros(lead_len)
    marker_t = np.arange(marker_len) / SR
    marker = 0.5 * np.sin(2 * np.pi * 1000.0 * marker_t)
    sweep_t = np.arange(sweep_len) / SR
    sweep = 0.5 * chirp(sweep_t, f0=300, f1=3000, t1=sweep_t[-1], method="linear")

    sig = np.concatenate([lead, marker, sweep, marker, lead])
    # fade in/out 5ms to avoid a hard step at silence boundaries
    fade = int(0.005 * SR)
    window = np.ones_like(sig)
    window[:fade] = np.linspace(0, 1, fade)
    window[-fade:] = np.linspace(1, 0, fade)
    return sig * window


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=12.0, help="total seconds")
    ap.add_argument("--out", type=Path, required=True, help="output .alaw path (raw G.711 A-law octets)")
    ap.add_argument("--wav-out", type=Path, help="optional 16-bit PCM WAV for human listening")
    opts = ap.parse_args()

    sig = build_signal(opts.duration)
    linear = np.clip(sig * 32767, -32768, 32767).astype(np.int16)
    alaw = linear_to_alaw(linear)

    opts.out.write_bytes(alaw.tobytes())
    print(f"wrote {len(alaw)} A-law octets ({len(alaw) / SR:.2f}s @ {SR}Hz) -> {opts.out}")

    if opts.wav_out:
        import wave
        with wave.open(str(opts.wav_out), "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(SR)
            w.writeframes(linear.tobytes())
        print(f"wrote reference WAV -> {opts.wav_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
