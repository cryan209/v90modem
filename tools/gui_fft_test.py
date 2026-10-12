#!/usr/bin/env python3
"""Compare the native display FFT against its original direct DFT."""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).with_name('modem_gui_native.swift').read_text()
helper = source[source.index('func spectrumMagnitudes('):source.index('let rxColor')]
checks = r'''
for size in [128, 256] {
    for count in [0, 1, 64, 128, 256, 1600] {
        // Include DC, Nyquist, an on-grid tone and an off-grid tone.
        var samples: [Double] = []
        for i in 0..<count {
            let phase = 2*Double.pi*Double(i)/Double(size)
            let baseline = 1000.0 + (i % 2 == 0 ? 2000.0 : -2000.0)
            let onGrid = 9000*cos(phase*13)
            let offGrid = 3000*sin(phase*17.3)
            samples.append(baseline + onGrid + offGrid)
        }
        let actual = spectrumMagnitudes(samples, size: size)
        for k in 0...size/2 {
            var re = 0.0, im = 0.0
            for (i,v) in samples.enumerated() {
                let window = 0.5-0.5*cos(2*Double.pi*Double(i)/Double(max(1,count-1)))
                let angle = 2*Double.pi*Double(k)*Double(i)/Double(size)
                re += v*window*cos(angle); im -= v*window*sin(angle)
            }
            let expected = hypot(re,im)
            precondition(abs(actual[k]-expected) <= 1e-6 + expected*1e-10,
                         "Mismatch: size \(size), count \(count), bin \(k)")
        }
    }
}
print("GUI FFT matches direct DFT for all bins, including DC/Nyquist and long snapshots")
'''
with tempfile.TemporaryDirectory(prefix='modem-gui-fft-test-') as directory:
    root = Path(directory)
    script = root/'test.swift'
    script.write_text('import Foundation\n' + helper + checks)
    subprocess.run(['swiftc', '-module-cache-path', str(root/'cache'),
                    str(script), '-o', str(root/'test')], check=True)
    subprocess.run([str(root/'test')], check=True)
