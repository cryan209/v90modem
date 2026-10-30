#!/usr/bin/env python3
"""Numerical and accounting checks for the test-only analogue RX channel."""
import math
import unittest
from analogue_rx_channel import AnalogueRxChannel, ImpairedDAC


class ChannelTests(unittest.TestCase):
    def test_identity_and_delay(self):
        samples = list(range(-1000, 1000))
        self.assertEqual(AnalogueRxChannel().process(samples), samples)
        c = AnalogueRxChannel(delay_samples=32)
        self.assertEqual(c.process(samples), [0]*32 + samples[:-32])
        self.assertEqual(c.input_count, c.output_count)

    def test_gain(self):
        c = AnalogueRxChannel(gain_db=-6)
        self.assertEqual(c.process([10000, -10000, 0]), [5012, -5012, 0])

    def test_noise_calibration(self):
        c = AnalogueRxChannel(noise_dbfs=-60, seed=7)
        result = c.process([0]*100000)
        expected = 32768*10**(-60/20)
        self.assertLess(abs(sum(result)/len(result)), expected*.01)
        self.assertAlmostEqual(c.statistics()['noise_rms']/expected, 1, delta=.01)
        self.assertEqual(c.clipped, 0)

    def test_chunk_invariance(self):
        samples = [round(7000*math.sin(2*math.pi*2800*n/16000)) for n in range(8192)]
        for ppm in (-50, 0, 50):
            a = AnalogueRxChannel(clock_ppm=ppm, delay_samples=256, noise_dbfs=-70)
            b = AnalogueRxChannel(clock_ppm=ppm, delay_samples=256, noise_dbfs=-70)
            expected = a.process(samples)
            actual = []
            for start in range(0, len(samples), 128):
                actual += b.process(samples[start:start+128])
            self.assertEqual(expected, actual)
            self.assertEqual(b.input_count, b.output_count)
            self.assertLess(len(b.buffer), 320)

    def test_clock_waveform(self):
        # Compare to the continuous tone at the requested ADC clock positions,
        # rather than accepting a mapper/receiver loopback as evidence.
        freq, amp, n = 3000, 8000, 20000
        source = [round(amp*math.sin(2*math.pi*freq*i/16000)) for i in range(n)]
        for ppm in (-50, 50):
            c = AnalogueRxChannel(clock_ppm=ppm, delay_samples=256)
            output = c.process(source)
            error = sum((output[i]-amp*math.sin(2*math.pi*freq*(i*c.step-256)/16000))**2
                        for i in range(512, n))
            self.assertLess(math.sqrt(error/(n-512)), 1)
            self.assertEqual(len(output), n)
            self.assertAlmostEqual(c.statistics()['source_clock_drift_samples'], n*(c.step-1))

    def test_clipping_and_validation(self):
        c = AnalogueRxChannel(gain_db=6)
        self.assertEqual(c.process([30000, -30000]), [32767, -32768])
        self.assertEqual(c.clipped, 2)
        for kwargs in ({'clock_ppm': float('nan')}, {'gain_db': float('inf')},
                       {'noise_dbfs': 1}, {'clock_ppm': 50}, {'delay_samples': -1}):
            with self.assertRaises(ValueError):
                AnalogueRxChannel(**kwargs)

    def test_dac_decorator(self):
        class DAC:
            tap_count = 129
            def process(self, samples):
                return [sample for sample in samples for _ in range(2)]
        dac = ImpairedDAC(DAC(), AnalogueRxChannel())
        self.assertEqual(dac.tap_count, 129)
        self.assertEqual(dac.process([1, 2]), [1, 1, 2, 2])


if __name__ == '__main__':
    unittest.main()
