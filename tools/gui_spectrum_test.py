#!/usr/bin/env python3
"""Check fresh-sample histories, averaging, phase timing and native layout."""
from pathlib import Path
import subprocess
import tempfile
import sys

source = Path(__file__).with_name('modem_gui_native.swift').read_text()
source = source[:source.index('final class Console')]
checks = r'''
let app = NSApplication.shared
app.appearance = NSAppearance(named:.darkAqua)
let plot = SpectrumPlot(frame:NSRect(x:0,y:0,width:550,height:100))
plot.peakHold = true
let tone = (0..<512).map { 16000*cos(2*Double.pi*Double($0)*13/256) }
let quiet = tone.map { $0/4 }
plot.update([tone,quiet],counts:[512,512])
let original = plot.amplitudes[0][13]
plot.averaging = true
plot.update([quiet,tone],counts:[512,512])
precondition(plot.amplitudes[0][13] == original, "Duplicate telemetry changed averaging")
plot.update([quiet,tone],counts:[1024,1024])
let expected = original*sqrt(0.8+0.2/16)
precondition(abs(plot.amplitudes[0][13]-expected)<1e-10)
precondition(plot.peaks[0][13] == original, "Peak was lost on a quieter frame")
plot.clearPeaks()
precondition(plot.peaks == plot.amplitudes)
plot.reset()
precondition(plot.seen == [-1,-1] && plot.peaks == [[],[]])
plot.averaging = false
plot.update([tone,quiet],counts:[512,512])
plot.hover = NSPoint(x:240,y:40)
let waterfall = Waterfall(frame:NSRect(x:0,y:0,width:550,height:100))
waterfall.update(tone,count:8000,phase:"Training · S")
waterfall.update(quiet,count:8000,phase:"Connected")
precondition(waterfall.held == 1 && waterfall.markers[0] == "Training · S")
waterfall.update(quiet,count:16000,phase:"Connected")
precondition(waterfall.stamps[1] == 2 && waterfall.markers[1] == "Connected")
waterfall.sampleRate = 9600
waterfall.update(quiet,count:28800,phase:"Connected")
precondition(waterfall.stamps[2] == 3, "Raw device waterfall used the engine clock")
waterfall.sampleRate = 8000
for i in 4...140 { waterfall.update(tone,count:i*8000,phase:"Connected") }
precondition(waterfall.held == 128 && waterfall.stamps[(waterfall.next-1+128)%128] == 140)
waterfall.fftSize = 512; waterfall.reset()
precondition(waterfall.held == 0 && waterfall.levels[0].count == 257)
for i in 1...40 { waterfall.update(tone,count:i*1200,phase:i<20 ? "Training · S" : "Connected") }
let root = NSView(frame:NSRect(x:0,y:0,width:570,height:230))
root.addSubview(plot); plot.frame.origin = NSPoint(x:10,y:120)
root.addSubview(waterfall); waterfall.frame.origin = NSPoint(x:10,y:10)
let bitmap = root.bitmapImageRepForCachingDisplay(in:root.bounds)!
root.cacheDisplay(in:root.bounds,to:bitmap)
let png = bitmap.representation(using:.png,properties:[:])!
try png.write(to:URL(fileURLWithPath:CommandLine.arguments[1]))
print("Spectrum history, power averaging, peak reset, waterfall clock and wrap checks passed")
'''
with tempfile.TemporaryDirectory(prefix='modem-gui-spectrum-test-') as directory:
    root = Path(directory)
    script = root/'test.swift'
    script.write_text(source + checks)
    subprocess.run(['swiftc', '-module-cache-path', str(root/'cache'),
                    str(script), '-o', str(root/'test')], check=True)
    preview = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else root/'preview.png'
    subprocess.run([str(root/'test'), str(preview)], check=True)
