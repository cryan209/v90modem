"""Test-only impairments AFTER analogue reconstruction, before native RX DSP.

Never operates on G.711 octets. Input/output accounting stays one-for-one
at the client's nominal 16 kHz. Clock error samples the reconstructed waveform
at fractional positions in a bounded delay line; it never drops/duplicates a
sample to correct a queue. Positive ppm models a faster receive sampling clock
(apparent received frequencies decrease). This is RX-only clock error, NOT a
coupled TX/RX oscillator model or a production-path resampler. The legacy
adapter uses this DAC only after K56 pump creation (common Phase 2 onwards),
not during V.8bis/V.8. Added delay is therefore NOT an end-to-end network
latency model; every impairment needs a matched clean control.
"""
import functools
import math
import random


@functools.lru_cache(maxsize=4096)
def _weights(phase):
    # 32-tap Blackman-windowed sinc; 4096 fractional phases. These are
    # simulator choices, not K56flex/ITU datapump constants.
    frac = phase / 4096.0
    values = []
    for offset in range(-15, 17):
        x = offset - frac
        window = .42 + .5*math.cos(math.pi*x/16) + .08*math.cos(2*math.pi*x/16)
        values.append(math.sin(math.pi*x)/(math.pi*x)*window if x else 1.0)
    scale = sum(values)
    return tuple(x/scale for x in values)


class AnalogueRxChannel:
    def __init__(self, *, gain_db=0.0, noise_dbfs=None, clock_ppm=0.0,
                 delay_samples=0, seed=1):
        if not all(math.isfinite(x) for x in (gain_db, clock_ppm)):
            raise ValueError("gain and clock must be finite")
        if noise_dbfs is not None and not math.isfinite(noise_dbfs):
            raise ValueError("noise must be finite")
        if abs(clock_ppm) > 200 or not -40 <= gain_db <= 6:
            raise ValueError("expected clock within +/-200 ppm and gain -40..6 dB")
        if noise_dbfs is not None and not -120 <= noise_dbfs <= 0:
            raise ValueError("expected noise -120..0 dBFS RMS")
        if not isinstance(delay_samples, int) or not 0 <= delay_samples <= 4096:
            raise ValueError("expected delay 0..4096 samples at 16 kHz")
        if clock_ppm and delay_samples < 32:
            raise ValueError("clock error requires at least 32 samples of causal delay")
        self.gain_db, self.noise_dbfs, self.clock_ppm = gain_db, noise_dbfs, clock_ppm
        self.delay = delay_samples
        self.gain = 10**(gain_db/20)
        self.noise_rms = 0 if noise_dbfs is None else 32768*10**(noise_dbfs/20)
        self.step = 1/(1 + clock_ppm*1e-6)
        self.rng = random.Random(seed)
        self.seed = seed
        self.buffer, self.base = [], 0
        self.input_count = self.output_count = self.clipped = 0
        self.signal_energy = self.noise_energy = 0.0

    def _sample(self, index):
        if index < 0:
            return 0
        if not self.base <= index < self.input_count:
            raise ValueError("RX clock delay exhausted; increase simulator delay")
        return self.buffer[index-self.base]

    def process(self, samples):
        self.buffer.extend(samples)
        self.input_count += len(samples)
        result = []
        for i in range(len(samples)):
            position = (self.output_count+i)*self.step - self.delay
            center = math.floor(position)
            phase = round((position-center)*4096)
            if phase == 4096:
                center += 1
                phase = 0
            if phase:
                value = sum(w*self._sample(center+k) for k, w in zip(range(-15, 17), _weights(phase)))
            else:
                value = self._sample(center)
            signal = value*self.gain
            noise = self.rng.gauss(0, self.noise_rms) if self.noise_rms else 0.0
            self.signal_energy += signal*signal
            self.noise_energy += noise*noise
            value = round(signal + noise)
            self.clipped += int(value < -32768 or value > 32767)
            result.append(max(-32768, min(32767, value)))
        self.output_count += len(samples)
        keep_from = max(0, math.floor(self.output_count*self.step-self.delay)-16)
        discard = min(len(self.buffer), max(0, keep_from-self.base))
        if discard:
            del self.buffer[:discard]
            self.base += discard
        return result

    def statistics(self):
        n = max(1, self.output_count)
        return dict(boundary="post-DAC 16 kHz linear samples, downstream RX only",
                    gain_db=self.gain_db, noise_dbfs_rms=self.noise_dbfs,
                    clock_ppm=self.clock_ppm, delay_samples_16k=self.delay, seed=self.seed,
                    input_samples=self.input_count, output_samples=self.output_count,
                    source_clock_drift_samples=self.output_count*(self.step-1),
                    signal_rms=math.sqrt(self.signal_energy/n),
                    noise_rms=math.sqrt(self.noise_energy/n), clipped_samples=self.clipped)


class ImpairedDAC:
    """Decorate the existing reconstruction object; DS0 and upstream untouched."""
    def __init__(self, dac, channel):
        self.dac, self.channel = dac, channel
        self.tap_count = dac.tap_count

    def process(self, samples):
        return self.channel.process(self.dac.process(samples))
