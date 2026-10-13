// Native passive modem console. AppKit owns rendering; the modem's control
// loop publishes bounded snapshots. Signal data never passes through this UI.
import AppKit
import AVFoundation

// Display-only radix-2 FFT. Folding windowed samples modulo size preserves
// the old spectrum's frequency grid even when a snapshot exceeds that size.
func spectrumMagnitudes(_ samples: [Double], size: Int) -> [Double] {
    precondition(size >= 2 && size & (size-1) == 0)
    var re = [Double](repeating: 0, count: size)
    var im = re
    for (i, value) in samples.enumerated() {
        let window = 0.5-0.5*cos(2*Double.pi*Double(i)/Double(max(1,samples.count-1)))
        re[i % size] += value*window
    }
    var j = 0
    for i in 1..<size {
        var bit = size >> 1
        while j & bit != 0 { j ^= bit; bit >>= 1 }
        j ^= bit
        if i < j { re.swapAt(i,j); im.swapAt(i,j) }
    }
    var length = 2
    while length <= size {
        let half = length/2
        for k in 0..<half {
            let angle = -2*Double.pi*Double(k)/Double(length)
            let wr = cos(angle), wi = sin(angle)
            for start in stride(from: 0, to: size, by: length) {
                let a = start+k, b = a+half
                let tr = wr*re[b]-wi*im[b], ti = wr*im[b]+wi*re[b]
                re[b] = re[a]-tr; im[b] = im[a]-ti
                re[a] += tr; im[a] += ti
            }
        }
        length *= 2
    }
    return (0...size/2).map { hypot(re[$0],im[$0]) }
}

// One-sided peak-amplitude dBFS; a full-scale bin-centred sine reads 0 dBFS.
func spectrumAmplitudes(_ samples: [Double], size: Int) -> [Double] {
    let signal = Array(samples.suffix(size))
    guard signal.count == size else { return [] }
    let weight = Double(size-1)/2
    return spectrumMagnitudes(signal, size: size).enumerated().map {
        $0.element * ($0.offset == 0 || $0.offset == size/2 ? 1 : 2) / weight / 32768
    }
}
func spectrumDB(_ amplitude: Double) -> Double {
    max(-90,min(0,20*log10(max(1e-12,amplitude))))
}

let rxColor = NSColor.systemTeal
let txColor = NSColor.systemOrange

final class SignalPlot: NSView {
    enum Kind { case waveform, constellation, eye, recoveredEye, histogram }
    var kind: Kind = .waveform
    var samples: [Double] = []
    var points: [[[Double]]] = [[], []]
    var recovered: [[Double]] = []
    var beforeEqualizer = false
    var eyeMessage = "Waiting for recovered samples"
    var color = rxColor
    var pcm = true
    var baud = 3200.0
    var carrier = 1800.0
    var phase = 0.0
    override var isFlipped: Bool { true }
    override func draw(_ dirtyRect: NSRect) {
        NSColor(calibratedWhite: 0.055, alpha: 1).setFill()
        bounds.fill()
        let w = bounds.width, h = bounds.height
        NSColor(calibratedWhite: 0.18, alpha: 1).setStroke()
        let grid = NSBezierPath()
        grid.move(to:NSPoint(x:w/2,y:0)); grid.line(to:NSPoint(x:w/2,y:h))
        grid.move(to:NSPoint(x:0,y:h/2)); grid.line(to:NSPoint(x:w,y:h/2))
        grid.stroke()
        if kind == .recoveredEye {
            let attrs:[NSAttributedString.Key:Any] = [.font:NSFont.systemFont(ofSize:10),.foregroundColor:NSColor.secondaryLabelColor]
            guard recovered.count >= 5 else { (eyeMessage as NSString).draw(at:NSPoint(x:10,y:h/2-5),withAttributes:attrs); return }
            let offset = beforeEqualizer ? 0 : 2
            let scale = max(1e-6,recovered.flatMap { [$0[offset],$0[offset+1]] }.map { abs($0) }.max() ?? 1)
            // Actual T/2 measurements only. Straight joins are a visual aid,
            // not oversampled measurements or an invented pulse shape.
            for channel in 0..<2 {
                let mid = channel == 0 ? h*0.25 : h*0.75
                let ink = channel == 0 ? rxColor : txColor
                for start in 0..<(recovered.count-4) where recovered[start][4] == 0 {
                    let segment = Array(recovered[start...start+4])
                    guard segment.enumerated().allSatisfy({ Int($0.element[4]) == $0.offset%2 }) else { continue }
                    let line = NSBezierPath()
                    for j in 0..<5 {
                        let p = NSPoint(x:Double(j)*w/4,y:mid-segment[j][offset+channel]/scale*h*0.20)
                        j == 0 ? line.move(to:p) : line.line(to:p)
                        ink.withAlphaComponent(j%2 == 0 ? 0.5 : 0.2).setFill()
                        NSBezierPath(ovalIn:NSRect(x:p.x-1,y:p.y-1,width:2,height:2)).fill()
                    }
                    ink.withAlphaComponent(0.12).setStroke(); line.stroke()
                }
                ((channel == 0 ? "I" : "Q") as NSString).draw(at:NSPoint(x:3,y:mid-10),withAttributes:attrs)
            }
            ("0" as NSString).draw(at:NSPoint(x:3,y:h-13),withAttributes:attrs)
            ("T" as NSString).draw(at:NSPoint(x:w/2,y:h-13),withAttributes:attrs)
            ("2T · dots are measured T/2 samples" as NSString).draw(at:NSPoint(x:max(w/2+12,w-195),y:h-13),withAttributes:attrs)
            return
        }
        if kind == .histogram {
            var bins = [Double](repeating:0,count:64)
            for sample in samples { bins[min(63,max(0,Int((sample+32768)/1024)))] += 1 }
            let peak = max(1,bins.max() ?? 1)
            color.setFill()
            for (i,v) in bins.enumerated() { NSRect(x:Double(i)*w/64,y:h-v/peak*h*0.9,width:max(1,w/64-1),height:v/peak*h*0.9).fill() }
            return
        }
        if kind == .constellation {
            let scale = max(1, points.flatMap { $0 }.flatMap { $0 }.map { abs($0) }.max() ?? 1)*1.15
            for (dir, values) in points.enumerated() {
                (dir == 0 ? rxColor : txColor).setFill()
                for p in values where p.count == 2 {
                    let radius = dir == 0 ? 1.5 : 2.0
                    NSBezierPath(ovalIn: NSRect(x: w/2+p[0]/scale*min(w,h)/2-radius,
                        y: h/2-p[1]/scale*min(w,h)/2-radius, width: radius*2, height: radius*2)).fill()
                }
            }
            return
        }
        guard !samples.isEmpty else { return }
        let path = NSBezierPath()
        path.lineWidth = 1
        if kind == .waveform {
            color.setStroke()
            for (i, value) in samples.enumerated() {
                let p = NSPoint(x: Double(i)*w/Double(max(1, samples.count-1)), y: h/2-value*h/65536)
                i == 0 ? path.move(to: p) : path.line(to: p)
            }
        } else {
            guard baud >= 300 && baud <= 8000 && carrier >= 0 && carrier <= 4000 && phase >= 0 && phase <= 1 else { return }
            var y = samples.enumerated().map { pcm ? $0.element : $0.element*cos(2*Double.pi*carrier*Double($0.offset)/8000) }
            if !pcm {
                let mixed = y
                y = mixed.indices.map { i in
                    let lo = max(0,i-2), hi = min(mixed.count,i+3)
                    return mixed[lo..<hi].reduce(0,+)/Double(hi-lo)
                }
            }
            let scale = max(1, y.map { abs($0) }.max() ?? 1)
            // Eightfold display-only windowed sinc reconstruction makes the
            // DS0's one-sample-per-symbol PCM ladder visible between samples.
            var previous = 0.0
            for sub in 0..<max(0,(min(y.count,64)-1)*8) {
                let t = Double(sub)/8
                let at = Int(t)
                var value = 0.0, weights = 0.0
                for i in max(0,at-7)...min(y.count-1,at+8) {
                    let d = t-Double(i)
                    let sinc = abs(d)<0.00001 ? 1 : sin(Double.pi*d)/(Double.pi*d)
                    let window = abs(d)<8 ? (0.5+0.5*cos(Double.pi*d/8)) : 0
                    value += y[i]*sinc*window; weights += sinc*window
                }
                if abs(weights)>0.00001 { value /= weights }
                let x = ((t*(pcm ? 8000 : baud)/8000+phase).truncatingRemainder(dividingBy: 2))/2
                let p = NSPoint(x: x*w, y: h/2-value/scale*h*0.40)
                if sub == 0 || x < previous { path.move(to:p) } else { path.line(to:p) }
                previous = x
            }
            rxColor.withAlphaComponent(0.22).setStroke()
        }
        path.stroke()
    }
}

// Spectrum histories advance only on fresh sample counters, never on redraw.
final class SpectrumPlot: NSView {
    var size = 256, averaging = false, peakHold = false
    var sampleRate = 8000.0
    var amplitudes: [[Double]] = [[],[]], peaks: [[Double]] = [[],[]]
    var seen = [-1,-1], carriers = [0.0,0.0]
    var hover: NSPoint?
    var tracking: NSTrackingArea?
    override var isFlipped: Bool { true }
    func reset() { amplitudes = [[],[]]; peaks = [[],[]]; seen = [-1,-1]; needsDisplay = true }
    func clearPeaks() { peaks = amplitudes; needsDisplay = true }
    func update(_ audio: [[Double]], counts: [Int]) {
        for dir in 0..<2 where counts[dir] > seen[dir] {
            let next = spectrumAmplitudes(audio[dir], size:size)
            guard !next.isEmpty else { continue }
            seen[dir] = counts[dir]
            if averaging && amplitudes[dir].count == next.count {
                // Exponential average of power, not logarithmic levels.
                amplitudes[dir] = zip(amplitudes[dir],next).map { sqrt(0.8*$0*$0+0.2*$1*$1) }
            } else { amplitudes[dir] = next }
            if peaks[dir].count != next.count { peaks[dir] = next }
            else { peaks[dir] = zip(peaks[dir],next).map { max($0,$1) } }
        }
        needsDisplay = true
    }
    override func updateTrackingAreas() {
        super.updateTrackingAreas()
        if let tracking = tracking { removeTrackingArea(tracking) }
        tracking = NSTrackingArea(rect:.zero,options:[.mouseMoved,.mouseEnteredAndExited,.activeInKeyWindow,.inVisibleRect],owner:self,userInfo:nil)
        addTrackingArea(tracking!)
    }
    override func mouseMoved(with event:NSEvent) { hover = convert(event.locationInWindow,from:nil); needsDisplay = true }
    override func mouseExited(with event:NSEvent) { hover = nil; needsDisplay = true }
    override func draw(_ dirtyRect:NSRect) {
        NSColor(calibratedWhite:0.055,alpha:1).setFill(); bounds.fill()
        let left = 34.0, width = max(1,bounds.width-left-4), height = max(1,bounds.height-28)
        let attrs: [NSAttributedString.Key:Any] = [.font:NSFont.systemFont(ofSize:9),.foregroundColor:NSColor.secondaryLabelColor]
        for level in stride(from:0,through:-90,by:-30) {
            let y = -Double(level)/90*height
            NSColor.darkGray.setStroke(); let line = NSBezierPath()
            line.move(to:NSPoint(x:left,y:y)); line.line(to:NSPoint(x:left+width,y:y)); line.stroke()
            ("\(level)" as NSString).draw(at:NSPoint(x:1,y:max(0,y-9)),withAttributes:attrs)
        }
        for dir in 0..<2 {
            let color = dir == 0 ? rxColor : txColor
            for held in [false,true] where !held || peakHold {
                let values = held ? peaks[dir] : amplitudes[dir]
                guard values.count > 1 else { continue }
                let line = NSBezierPath(); line.lineWidth = held ? 0.7 : 1.3
                for (i,v) in values.enumerated() {
                    let point = NSPoint(x:left+Double(i)*width/Double(size/2),y:-spectrumDB(v)/90*height)
                    i == 0 ? line.move(to:point) : line.line(to:point)
                }
                if held { line.setLineDash([3,3],count:2,phase:0) }
                color.withAlphaComponent(held ? 0.45 : 1).setStroke(); line.stroke()
            }
            if carriers[dir] > 0 && carriers[dir] < sampleRate/2 {
                color.withAlphaComponent(0.4).setStroke(); let marker = NSBezierPath()
                let x = left+carriers[dir]/(sampleRate/2)*width
                marker.move(to:NSPoint(x:x,y:0)); marker.line(to:NSPoint(x:x,y:height)); marker.stroke()
            }
        }
        ("0 Hz" as NSString).draw(at:NSPoint(x:left,y:height+1),withAttributes:attrs)
        (String(format:"%g kHz",sampleRate/4000) as NSString).draw(at:NSPoint(x:left+width/2-14,y:height+1),withAttributes:attrs)
        (String(format:"%g kHz · dBFS",sampleRate/2000) as NSString).draw(at:NSPoint(x:left+width-65,y:height+1),withAttributes:attrs)
        if let point = hover {
            let bin = min(size/2,max(0,Int(((point.x-left)/width*Double(size/2)).rounded())))
            let frequency = Double(bin)*sampleRate/Double(size)
            NSColor.white.withAlphaComponent(0.4).setStroke(); let cursor = NSBezierPath()
            let x = left+Double(bin)*width/Double(size/2)
            cursor.move(to:NSPoint(x:x,y:0)); cursor.line(to:NSPoint(x:x,y:height)); cursor.stroke()
            let values = (0..<2).map { dir -> String in
                guard amplitudes[dir].indices.contains(bin) else { return "—" }
                return String(format:"%.1f dBFS",spectrumDB(amplitudes[dir][bin]))
            }
            (String(format:"%.1f Hz · RX %@ · TX %@",frequency,values[0],values[1]) as NSString).draw(at:NSPoint(x:left,y:height+14),withAttributes:attrs)
        } else {
            ("RX teal · TX orange · hover for frequency and level" as NSString).draw(at:NSPoint(x:left,y:height+14),withAttributes:attrs)
        }
    }
}

// Display-only spectrum history: 128 rows × 129 bins, overwritten in memory.
// Fixed dBFS colour scale preserves level changes between successive rows.
final class Waterfall: NSView {
    static let rows = 128
    var fftSize = 256
    var sampleRate = 8000.0
    var bins: Int { fftSize/2+1 }
    var levels = [[Double]](repeating:[Double](repeating:-90,count:129),count:rows)
    var next = 0, held = 0, seen = -1
    var carrier = 0.0
    var stamps = [Double](repeating:0,count:rows)
    var markers = [String](repeating:"",count:rows)
    var lastPhase = ""
    override var isFlipped: Bool { true }
    func reset() { next = 0; held = 0; seen = -1; lastPhase = ""; levels = Array(repeating:Array(repeating:-90,count:bins),count:Self.rows); needsDisplay = true }
    func update(_ samples:[Double], count:Int, phase:String) {
        guard count > seen, samples.count >= fftSize else { return }
        seen = count
        let signal = Array(samples.suffix(fftSize))
        let amplitudes = spectrumAmplitudes(signal,size:fftSize)
        for k in 0..<bins { levels[next][k] = spectrumDB(amplitudes[k]) }
        stamps[next] = Double(count)/sampleRate
        markers[next] = phase != lastPhase ? phase : ""
        lastPhase = phase
        next = (next+1)%Self.rows; held = min(Self.rows,held+1); needsDisplay = true
    }
    override func draw(_ dirtyRect:NSRect) {
        NSColor(calibratedWhite:0.055,alpha:1).setFill(); bounds.fill()
        let plotHeight = max(1,bounds.height-28), rowHeight = plotHeight/Double(Self.rows)
        let binWidth = bounds.width/Double(bins)
        for row in 0..<held {
            let source = (next-1-row+Self.rows)%Self.rows
            for bin in 0..<bins {
                let strength = (levels[source][bin]+90)/90
                NSColor(calibratedHue:0.67-strength*0.67,saturation:0.9,brightness:0.08+0.92*strength,alpha:1).setFill()
                NSRect(x:Double(bin)*binWidth,y:Double(row)*rowHeight,width:binWidth+0.5,height:rowHeight+0.5).fill()
            }
        }
        let markerAttrs: [NSAttributedString.Key:Any] = [.font:NSFont.systemFont(ofSize:9),.foregroundColor:NSColor.white,.backgroundColor:NSColor.black.withAlphaComponent(0.7)]
        var previousLabelY = -12.0
        for row in 0..<held {
            let source = (next-1-row+Self.rows)%Self.rows
            if !markers[source].isEmpty {
                let y = Double(row)*rowHeight
                NSColor.white.withAlphaComponent(0.5).setStroke(); let line = NSBezierPath()
                line.move(to:NSPoint(x:0,y:y)); line.line(to:NSPoint(x:bounds.width,y:y)); line.stroke()
                if y-previousLabelY >= 12 && y <= plotHeight-11 {
                    (String(format:"%.2fs · %@",stamps[source],markers[source]) as NSString).draw(at:NSPoint(x:2,y:y),withAttributes:markerAttrs)
                    previousLabelY = y
                }
            }
        }
        if carrier > 0 && carrier < sampleRate/2 {
            NSColor.systemYellow.withAlphaComponent(0.6).setStroke()
            let marker = NSBezierPath(); let x = carrier/(sampleRate/2)*bounds.width
            marker.move(to:NSPoint(x:x,y:0)); marker.line(to:NSPoint(x:x,y:plotHeight)); marker.stroke()
        }
        let attrs:[NSAttributedString.Key:Any] = [.font:NSFont.systemFont(ofSize:9),.foregroundColor:NSColor.secondaryLabelColor]
        ("0 Hz" as NSString).draw(at:NSPoint(x:2,y:plotHeight+2),withAttributes:attrs)
        (String(format:"%g kHz",sampleRate/4000) as NSString).draw(at:NSPoint(x:bounds.width/2-15,y:plotHeight+2),withAttributes:attrs)
        (String(format:"%g kHz",sampleRate/2000) as NSString).draw(at:NSPoint(x:bounds.width-40,y:plotHeight+2),withAttributes:attrs)
        if held > 0 {
            let newest = (next-1+Self.rows)%Self.rows, oldest = (next-held+Self.rows)%Self.rows
            (String(format:"%.2f–%.2f s · sample clock · gaps omitted",stamps[oldest],stamps[newest]) as NSString).draw(at:NSPoint(x:2,y:plotHeight+14),withAttributes:attrs)
        }
    }
}

final class Console {
    let scroll = NSScrollView()
    let text = NSTextView()
    var seen = 0
    init(height: CGFloat) {
        text.isEditable = false
        text.font = .monospacedSystemFont(ofSize: 11, weight: .regular)
        text.textColor = .labelColor
        text.backgroundColor = NSColor(calibratedWhite: 0.08, alpha: 1)
        text.isVerticallyResizable = true
        text.isHorizontallyResizable = false
        text.textContainer?.widthTracksTextView = true
        scroll.documentView = text
        scroll.hasVerticalScroller = false
        scroll.translatesAutoresizingMaskIntoConstraints = false
        scroll.heightAnchor.constraint(equalToConstant: height).isActive = true
    }
    func replace(_ value: String) {
        let bounded = String(value.suffix(24000)).split(separator:"\n",omittingEmptySubsequences:false).suffix(max(2,Int(scroll.frame.height/14))).joined(separator:"\n")
        if text.string != bounded { text.string = bounded }
    }
    func append(_ value: String) {
        replace(text.string+value)
        text.scrollToEndOfDocument(nil)
    }
    func update(_ entries: [[Any]], binary: Bool) {
        for entry in entries where entry.count == 2 {
            guard let id = entry[0] as? Int, id > seen,
                  let b64 = entry[1] as? String, let data = Data(base64Encoded:b64) else { continue }
            if id > seen+1 { append("\n[Older console output discarded]\n") }
            if binary {
                append(data.map { b in
                    if b == 10 { return "\n" }; if b == 13 { return "" }
                    return b>=32 && b<127 ? String(UnicodeScalar(b)) : String(format:"\\x%02x", b)
                }.joined())
            } else { append(String(decoding:data,as:UTF8.self)) }
            seen = id
        }
    }
}

final class App: NSObject, NSApplicationDelegate, NSWindowDelegate {
    let base: URL
    let session = URLSession(configuration: .ephemeral)
    var window: NSWindow!
    let status = NSTextField(labelWithString:"Starting modem…")
    let diagnostics = NSTabView()
    var appleTab: NSTabViewItem?
    let usbPicker = NSPopUpButton(), audioPicker = NSPopUpButton(), capturePicker = NSPopUpButton(), autoAnswer = NSPopUpButton()
    let onHookMonitor = NSButton(checkboxWithTitle:"On-hook monitor",target:nil,action:nil)
    let watchRings = NSButton(checkboxWithTitle:"Watch rings",target:nil,action:nil)
    let rawSource = NSButton(checkboxWithTitle:"Device audio",target:nil,action:nil)
    let captureAudio = NSButton(checkboxWithTitle:"Record RX/TX",target:nil,action:nil)
    let lineHealth = NSTextField(labelWithString:"Line: unchecked"), audioHealth = NSTextField(labelWithString:"Audio: waiting")
    let capturePath = NSTextField(labelWithString:"")
    var appleUSBIDs = [""], appleAudioIDs = [""], appleCaptureIDs: [String] = []
    var appleInventoryKey = "", appleCaptureKey = ""
    var audioRate = 8000.0
    let hardwareLabel = NSTextField(labelWithString:"")
    let error = NSTextField(labelWithString:"")
    let rx = SignalPlot(), tx = SignalPlot(), constellation = SignalPlot(), eye = SignalPlot()
    let at = Console(height:64), serial = Console(height:64), training = Console(height:28), log = Console(height:130)
    let rxWire = Console(height:54), txWire = Console(height:54)
    let histogram = SignalPlot()
    let spectrum = SpectrumPlot()
    let fftSizeControl = NSPopUpButton()
    let averageControl = NSButton(checkboxWithTitle:"Average",target:nil,action:nil)
    let peakControl = NSButton(checkboxWithTitle:"Peak hold",target:nil,action:nil)
    let rxWaterfall = Waterfall(), txWaterfall = Waterfall()
    let carrierLabel = NSTextField(labelWithString:"Carrier: waiting")
    let listen = NSPopUpButton()
    let audioEngine = AVAudioEngine(), player = AVAudioPlayerNode()
    var audioSeen = [0,0], audioEpoch = -1, audioScheduled = 0, audioDirection = 0
    let atInput = NSTextField(), dataInput = NSTextField()
    let loopbackMode = NSPopUpButton()
    let loopbackStatus = NSTextField(labelWithString:"")
    var loopbackRow: NSStackView!, startLoop: NSButton!, pattern: NSButton!
    var loopbackProfiles: [String] = []
    let eyeSource = NSPopUpButton()
    var eyeCountSeen = -1, eyeUpdated = Date.distantPast
    let eyeDir = NSPopUpButton(), dataMode = NSPopUpButton()
    let atPath = NSTextField(labelWithString:""), dataPath = NSTextField(labelWithString:"")
    var sendData: NSButton!
    var timer: Timer?
    var loading = false, frozen = false
    var snapshot: [String:Any] = [:]
    var epoch = -1, eventSeen = 0
    var history: [String] = []
    init(_ url: URL) { base = url; super.init() }

    func label(_ text: String) -> NSTextField {
        let v = NSTextField(wrappingLabelWithString:text)
        v.font = .systemFont(ofSize:11)
        v.textColor = .secondaryLabelColor
        return v
    }
    func row(_ views: [NSView]) -> NSStackView {
        let v = NSStackView(views:views); v.orientation = .horizontal; v.spacing = 8
        v.alignment = .centerY
        return v
    }
    func button(_ title: String, _ action: Selector) -> NSButton {
        NSButton(title:title, target:self, action:action)
    }
    func panel(_ title: String, _ children: [NSView]) -> NSBox {
        let box = NSBox(); box.title = title; box.contentViewMargins = NSSize(width:6,height:6)
        let stack = NSStackView(views:children)
        stack.orientation = .vertical; stack.alignment = .leading; stack.spacing = 4
        stack.translatesAutoresizingMaskIntoConstraints = false
        box.contentView = NSView()
        box.contentView!.addSubview(stack)
        NSLayoutConstraint.activate([stack.leadingAnchor.constraint(equalTo:box.contentView!.leadingAnchor),
            stack.trailingAnchor.constraint(equalTo:box.contentView!.trailingAnchor),
            stack.topAnchor.constraint(equalTo:box.contentView!.topAnchor),
            stack.bottomAnchor.constraint(equalTo:box.contentView!.bottomAnchor)])
        for child in children { child.widthAnchor.constraint(equalTo:stack.widthAnchor).isActive = true }
        return box
    }
    func pair(_ left: NSView, _ right: NSView) -> NSStackView {
        let stack = row([left,right]); stack.alignment = .top
        stack.distribution = .fillEqually
        return stack
    }
    func applicationDidFinishLaunching(_ notification: Notification) {
        NSApp.appearance = NSAppearance(named:.darkAqua)
        window = NSWindow(contentRect:NSRect(x:0,y:0,width:1200,height:810),
            styleMask:[.titled,.closable,.miniaturizable,.resizable], backing:.buffered, defer:false)
        window.title = "Modem · Live line"
        window.minSize = NSSize(width:1000,height:770)
        window.delegate = self
        let root = NSStackView(); root.orientation = .vertical; root.alignment = .leading; root.spacing = 6
        root.edgeInsets = NSEdgeInsets(top:8,left:10,bottom:8,right:10)
        root.translatesAutoresizingMaskIntoConstraints = false
        let content = NSView(); window.contentView = content; content.addSubview(root)
        NSLayoutConstraint.activate([root.leadingAnchor.constraint(equalTo:content.leadingAnchor),root.trailingAnchor.constraint(equalTo:content.trailingAnchor),root.topAnchor.constraint(equalTo:content.topAnchor),root.bottomAnchor.constraint(lessThanOrEqualTo:content.bottomAnchor)])
        for plot in [rx,tx,constellation,eye] { plot.heightAnchor.constraint(equalToConstant:72).isActive = true }
        for plot in [constellation,eye] { plot.constraints.filter { $0.firstAttribute == .height }.forEach { $0.constant = 100 } }
        tx.color = txColor; constellation.kind = .constellation; eye.kind = .eye
        status.font = .systemFont(ofSize:14,weight:.semibold)
        error.textColor = .systemRed
        eyeDir.addItems(withTitles:["RX","TX"])
        eyeSource.addItems(withTitles:["Recovered · equalizer","Recovered · input","Line estimate"])
        dataMode.addItems(withTitles:["UTF-8 + CRLF","Hex bytes"])
        atInput.placeholderString = "ATDnumber or AT command"; atInput.target = self; atInput.action = #selector(sendAT)
        dataInput.placeholderString = "Serial payload"; dataInput.target = self; dataInput.action = #selector(sendSerial)
        sendData = button("Send data",#selector(sendSerial))
        histogram.kind = .histogram
        fftSizeControl.addItems(withTitles:["128 · 62.5 Hz", "256 · 31.25 Hz", "512 · 15.625 Hz"])
        fftSizeControl.selectItem(at:1)
        for control in [fftSizeControl,averageControl,peakControl] as [NSControl] { control.target = self; control.action = #selector(spectrumOptions) }
        for plot in [histogram,spectrum] as [NSView] { plot.heightAnchor.constraint(equalToConstant:100).isActive = true }
        listen.addItems(withTitles:["Audio off","Listen RX","Listen TX"])
        audioEngine.attach(player)
        audioEngine.connect(player,to:audioEngine.mainMixerNode,format:AVAudioFormat(standardFormatWithSampleRate:8000,channels:1))
        player.volume = 0.25
        @discardableResult func tab(_ title:String,_ view:NSView) -> NSTabViewItem {
            let item = NSTabViewItem(identifier:title); item.label = title
            let container = NSView(); view.translatesAutoresizingMaskIntoConstraints = false
            container.addSubview(view)
            NSLayoutConstraint.activate([view.leadingAnchor.constraint(equalTo:container.leadingAnchor,constant:4),view.trailingAnchor.constraint(equalTo:container.trailingAnchor,constant:-4),view.topAnchor.constraint(equalTo:container.topAnchor,constant:4)])
            item.view = container; diagnostics.addTabViewItem(item)
            return item
        }
        tab("Signal",pair(panel("Received constellation · teal samples / orange decisions",[constellation]),panel("Eye · I/Q at recovered timing, or explicit line estimate",[eye])))
        tab("Audio",pair(panel("RX amplitude histogram · −32768 to +32767",[histogram]),panel("RX/TX spectrum · −90 to 0 dBFS",[spectrum])))
        for plot in [rxWaterfall,txWaterfall] { plot.heightAnchor.constraint(equalToConstant:100).isActive = true }
        tab("Waterfall",pair(panel("RX · newest at top · −90 to 0 dBFS",[rxWaterfall]),panel("TX · newest at top · −90 to 0 dBFS",[txWaterfall])))
        tab("Process log",log.scroll)
        for picker in [usbPicker,audioPicker,capturePicker] { picker.widthAnchor.constraint(equalToConstant:210).isActive = true }
        autoAnswer.addItems(withTitles:["Auto-answer off"] + (1...9).map { "After \($0) ring\($0 == 1 ? "" : "s")" })
        autoAnswer.target = self; autoAnswer.action = #selector(setAutoAnswer)
        for (control,action) in [(onHookMonitor,#selector(setOnHookMonitor)),(watchRings,#selector(setWatchRings)),(captureAudio,#selector(setCaptureAudio)),(rawSource,#selector(spectrumOptions))] {
            control.target = self; control.action = action
        }
        rawSource.state = .on
        for field in [lineHealth,audioHealth,capturePath] { field.font = .systemFont(ofSize:10); field.lineBreakMode = .byTruncatingMiddle }
        appleTab = tab("Apple hardware",panel("Codec / line controls · raw audio and saved AT settings",[
            row([label("USB"),usbPicker,label("Audio"),audioPicker,button("Refresh",#selector(refreshApple)),button("Apply",#selector(applyApple))]),
            row([button("Check line",#selector(probeApple)),onHookMonitor,rawSource,watchRings,autoAnswer,button("Save AT settings",#selector(saveAppleSettings))]),
            row([captureAudio,capturePicker,button("Replay RX",#selector(replayApple)),button("Stop replay",#selector(stopAppleReplay))]),
            lineHealth,audioHealth,capturePath]))
        diagnostics.removeTabViewItem(appleTab!)
        diagnostics.heightAnchor.constraint(equalToConstant:205).isActive = true
        startLoop = button("Start loopback",#selector(startLoopback))
        pattern = button("256-byte pattern",#selector(sendPattern))
        loopbackRow = row([label("Local test"),loopbackMode,startLoop,pattern,loopbackStatus]); loopbackRow.isHidden = true
        let items: [NSView] = [
            row([status,listen,button("Freeze",#selector(freeze))]),hardwareLabel,loopbackRow,error,
            pair(panel("RX line · latest 512 samples",[rx]),panel("TX line · latest 512 samples",[tx])),
            row([eyeDir,eyeSource,carrierLabel]),
            row([label("FFT"),fftSizeControl,averageControl,peakControl,button("Clear peaks",#selector(clearSpectrumPeaks))]),
            diagnostics,
            pair(panel("RX wire · repeated bytes collapsed",[rxWire.scroll]),panel("TX wire · repeated bytes collapsed",[txWire.scroll])),
            pair(panel("AT control",[row([button("Answer",#selector(answer)),button("Hang up",#selector(hangup)),button("Info",#selector(info))]),at.scroll,row([atInput,button("Send AT",#selector(sendAT))])]),
                 panel("Serial data",[serial.scroll,row([dataInput,dataMode,sendData])])),
            training.scroll]
        for view in items { root.addArrangedSubview(view); view.widthAnchor.constraint(equalTo:root.widthAnchor,constant:-20).isActive = true }
        at.scroll.toolTip = "Separate AT PTY; latest output. No history is saved to disk."
        rxWire.scroll.toolTip = "Framed/compressed line bits before modulation scrambling, packed LSB first. Latest 256 byte runs."
        for path in [atPath,dataPath] { path.font = .monospacedSystemFont(ofSize:10,weight:.regular); path.lineBreakMode = .byTruncatingMiddle }
        if let screen = NSScreen.main { window.setFrame(NSRect(x:screen.visibleFrame.midX-600,y:screen.visibleFrame.midY-405,width:min(1200,screen.visibleFrame.width),height:min(810,screen.visibleFrame.height)),display:true) }
        window.center(); window.makeKeyAndOrderFront(nil); NSApp.activate(ignoringOtherApps:true)
        timer = Timer.scheduledTimer(withTimeInterval:0.15,repeats:true) { [weak self] _ in self?.poll() }
        poll()
    }
    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }
    func applicationWillTerminate(_ notification: Notification) { timer?.invalidate() }
    @objc func freeze(_ sender:NSButton) { frozen.toggle(); sender.title = frozen ? "Resume plots" : "Freeze plots" }
    @objc func spectrumOptions() {
        spectrum.size = [128,256,512][max(0,fftSizeControl.indexOfSelectedItem)]
        spectrum.averaging = averageControl.state == .on
        spectrum.peakHold = peakControl.state == .on
        spectrum.reset()
        for plot in [rxWaterfall,txWaterfall] { plot.fftSize = spectrum.size; plot.reset() }
    }
    @objc func clearSpectrumPeaks() { spectrum.clearPeaks() }
    @objc func refreshApple() { post("apple",["action":"refresh"]) }
    @objc func applyApple() {
        post("apple",["action":"configure","usb_address":appleUSBIDs[max(0,usbPicker.indexOfSelectedItem)],"audio_uid":appleAudioIDs[max(0,audioPicker.indexOfSelectedItem)]])
    }
    @objc func probeApple() { post("apple",["action":"probe"]) }
    @objc func setOnHookMonitor() { post("apple",["action":"monitor","enabled":onHookMonitor.state == .on]) }
    @objc func setWatchRings() { post("apple",["action":"watch","enabled":watchRings.state == .on]) }
    @objc func setAutoAnswer() { post("apple",["action":"auto_answer","rings":max(0,autoAnswer.indexOfSelectedItem)]) }
    @objc func saveAppleSettings() { post("apple",["action":"save_settings"]) }
    @objc func setCaptureAudio() { post("apple",["action":"capture","enabled":captureAudio.state == .on]) }
    @objc func replayApple() {
        guard appleCaptureIDs.indices.contains(capturePicker.indexOfSelectedItem) else { return }
        post("apple",["action":"replay","id":appleCaptureIDs[capturePicker.indexOfSelectedItem]])
    }
    @objc func stopAppleReplay() { post("apple",["action":"stop"]) }
    func renderApple(_ s:[String:Any], _ hardware:[String:Any]) {
        if let tab = appleTab, !diagnostics.tabViewItems.contains(where: { $0 === tab }) {
            diagnostics.addTabViewItem(tab); diagnostics.selectTabViewItem(tab)
        }
        let devices = hardware["devices"] as? [String:[[String:String]]] ?? [:]
        let usb = devices["usb"] ?? [], audio = devices["audio"] ?? []
        let selectedUSB = hardware["usb_address"] as? String ?? "Automatic", selectedAudio = hardware["audio_uid"] as? String ?? "Automatic"
        let key = (try? JSONSerialization.data(withJSONObject:devices,options:[.sortedKeys])).map { String(decoding:$0,as:UTF8.self) } ?? ""
        let inventory = key+selectedUSB+selectedAudio
        if inventory != appleInventoryKey {
            appleInventoryKey = inventory
            appleUSBIDs = [""]+usb.compactMap { $0["id"] }; appleAudioIDs = [""]+audio.compactMap { $0["id"] }
            usbPicker.removeAllItems(); audioPicker.removeAllItems()
            usbPicker.addItems(withTitles:["Automatic"]+usb.map { $0["label"] ?? $0["id"] ?? "USB" })
            audioPicker.addItems(withTitles:["Automatic"]+audio.map { ($0["label"] ?? "Audio")+" · "+($0["id"] ?? "") })
            if let i = appleUSBIDs.firstIndex(of:selectedUSB) { usbPicker.selectItem(at:i) }
            if let i = appleAudioIDs.firstIndex(of:selectedAudio) { audioPicker.selectItem(at:i) }
        }
        let captures = hardware["captures"] as? [[String:Any]] ?? []
        let ids = captures.compactMap { $0["id"] as? String }
        if ids.joined() != appleCaptureKey {
            let previous = appleCaptureIDs.indices.contains(capturePicker.indexOfSelectedItem) ? appleCaptureIDs[capturePicker.indexOfSelectedItem] : ""
            appleCaptureKey = ids.joined(); appleCaptureIDs = ids
            capturePicker.removeAllItems(); capturePicker.addItems(withTitles:captures.map { $0["label"] as? String ?? "Capture" })
            if let i = ids.firstIndex(of:previous) { capturePicker.selectItem(at:i) }
        }
        captureAudio.state = (hardware["capture"] as? Bool ?? false) ? .on : .off
        autoAnswer.selectItem(at:min(9,max(0,hardware["auto_answer"] as? Int ?? 0)))
        let health = s["line_health"] as? [String:Any] ?? [:]
        onHookMonitor.state = (health["monitoring"] as? Bool ?? false) ? .on : .off
        watchRings.state = (health["watching"] as? Bool ?? false) ? .on : .off
        let hook = health["hook_register"] as? Int ?? -1, sense = health["sense"] as? Int ?? -1
        let hookText = hook < 0 ? "unchecked" : hook & 1 == 1 ? "off-hook" : "on-hook"
        let senseText = sense < 0 ? "unchecked" : sense == 0 ? "no telephone pair" : String(format:"0x%02X · uncalibrated",sense)
        lineHealth.stringValue = "USB \((health["usb_open"] as? Bool ?? false) ? "open" : "not opened") · Hook \(hookText) · Line sense \(senseText) · \(health["rings"] as? Int ?? 0) ring bursts"
        let raw = s["hardware_audio"] as? [String:Any] ?? [:], metrics = raw["metrics"] as? [[String:Any]] ?? []
        let levels = metrics.enumerated().map { dir,m -> String in
            String(format:"%@ DC %.0f · RMS %.0f · clipped %d",dir == 0 ? "RX" : "TX",m["dc"] as? Double ?? 0,m["rms"] as? Double ?? 0,m["clipped"] as? Int ?? 0)
        }.joined(separator:" | ")
        audioHealth.stringValue = "\(levels) · Render errors \(raw["render_errors"] as? Int ?? 0) · TX empty \(raw["tx_empty_samples"] as? Int ?? 0) · Ring overruns \(raw["tx_ring_overruns"] as? Int ?? 0)"
        capturePath.stringValue = (hardware["replay"] as? Bool ?? false) ? "Offline RX replay · no USB line" : (hardware["capture_path"] as? String ?? "").isEmpty ? "Recording off · Device audio changes waveform, spectrum and listening; eye stays on the engine tap" : "Recording: \(hardware["capture_path"] as? String ?? "")"
    }
    @objc func answer() { send("at",Data("ATA\r".utf8)) }
    @objc func hangup() { send("at",Data("ATH\r".utf8)) }
    @objc func modulation() { send("at",Data("AT+MS?\r".utf8)) }
    @objc func info() { send("at",Data("ATI\r".utf8)) }
    @objc func sendAT() { send("at",Data((atInput.stringValue+"\r").utf8)); atInput.stringValue = "" }
    @objc func sendSerial() {
        let value = dataInput.stringValue
        var data = Data()
        if dataMode.indexOfSelectedItem == 1 {
            let hex = value.filter { !$0.isWhitespace }
            guard hex.count % 2 == 0 else { error.stringValue = "Hex requires pairs of digits"; return }
            var i = hex.startIndex
            while i < hex.endIndex {
                let end = hex.index(i,offsetBy:2)
                guard let byte = UInt8(hex[i..<end],radix:16) else { error.stringValue = "Invalid hex byte"; return }
                data.append(byte); i = end
            }
        } else { data = Data((value+"\r\n").utf8) }
        send("data",data); dataInput.stringValue = ""
    }
    @objc func startLoopback() {
        guard loopbackProfiles.indices.contains(loopbackMode.indexOfSelectedItem) else { return }
        post("loopback",["profile":loopbackProfiles[loopbackMode.indexOfSelectedItem]])
    }
    @objc func sendPattern() { send("data",Data((0..<256).map { UInt8($0) })) }
    func send(_ port:String, _ data:Data) {
        guard data.count <= 8000 else { error.stringValue = "Send at most 8,000 bytes at once"; return }
        post("send",["port":port,"data":data.base64EncodedString()])
    }
    func post(_ endpoint:String,_ payload:[String:Any]) {
        var req = URLRequest(url:URL(string:endpoint,relativeTo:base)!.absoluteURL)
        req.httpMethod = "POST"
        req.setValue("application/json",forHTTPHeaderField:"Content-Type")
        let components = URLComponents(url:base,resolvingAgainstBaseURL:true)!
        req.setValue("http://127.0.0.1:\(components.port!)",forHTTPHeaderField:"Origin")
        req.httpBody = try? JSONSerialization.data(withJSONObject:payload)
        session.dataTask(with:req) { [weak self] data,response,err in
            DispatchQueue.main.async {
                guard let self = self else { return }
                if let err = err { self.error.stringValue = err.localizedDescription }
                else if (response as? HTTPURLResponse)?.statusCode != 200 {
                    let value = data.flatMap { try? JSONSerialization.jsonObject(with:$0) } as? [String:Any]
                    self.error.stringValue = value?["error"] as? String ?? "Send failed"
                } else { self.error.stringValue = "" }
            }
        }.resume()
    }
    // Listening only: 300 ms prebuffer, bounded to 800 ms. Never touches DSP.
    func resetAudio() {
        player.stop(); audioScheduled = 0
    }
    func monitorAudio(_ s:[String:Any]) {
        let rate = s["audio_rate"] as? Double ?? 8000
        if rate != audioRate {
            resetAudio(); audioEngine.stop(); audioEngine.disconnectNodeOutput(player)
            audioEngine.connect(player,to:audioEngine.mainMixerNode,format:AVAudioFormat(standardFormatWithSampleRate:rate,channels:1))
            audioRate = rate; audioSeen = [0,0]
        }
        let current = s["epoch"] as? Int ?? 0, direction = listen.indexOfSelectedItem
        if current != audioEpoch || direction != audioDirection {
            audioEpoch = current; audioDirection = direction; audioSeen = [0,0]; resetAudio()
        }
        guard direction > 0 else { return }
        let d = direction-1, frames = s["listen"] as? [[String:Any]] ?? []
        guard frames.indices.contains(d) else { return }
        let frame = frames[d], count = frame["count"] as? Int ?? 0
        let hex = Array(frame["hex"] as? String ?? ""), n = hex.count/4
        let delta = max(0,count-audioSeen[d])
        guard delta > 0 else { return }
        // A missed ring window or excess latency starts a fresh bounded listen.
        if audioSeen[d] > 0 && delta > n { resetAudio() }
        let fresh = min(n,delta)
        // Completion callbacks count whole buffers, including already played
        // prefixes. Use the output clock to measure the actual pending tail.
        var pending = audioScheduled
        if player.isPlaying, let render = player.lastRenderTime,
           let played = player.playerTime(forNodeTime:render) {
            pending = max(0,audioScheduled-Int(played.sampleTime))
        }
        if pending+fresh > Int(rate*0.8) { resetAudio() }
        do { if !audioEngine.isRunning { try audioEngine.start() } }
        catch { self.error.stringValue = "Audio: \(error.localizedDescription)"; return }
        guard let format = AVAudioFormat(standardFormatWithSampleRate:rate,channels:1),
              let buffer = AVAudioPCMBuffer(pcmFormat:format,frameCapacity:AVAudioFrameCount(fresh)),
              let channel = buffer.floatChannelData?[0] else { return }
        buffer.frameLength = AVAudioFrameCount(fresh)
        for i in 0..<fresh {
            let at = (n-fresh+i)*4
            let lo = UInt16(String(hex[at..<at+2]),radix:16) ?? 0
            let hi = UInt16(String(hex[at+2..<at+4]),radix:16) ?? 0
            channel[i] = Float(Int16(bitPattern:lo | hi<<8))/32768
        }
        audioSeen[d] = count; audioScheduled += fresh
        player.scheduleBuffer(buffer)
        if !player.isPlaying && audioScheduled >= Int(rate*0.3) { player.play() }
    }
    func poll() {
        guard !loading else { return }; loading = true
        var req = URLRequest(url:URL(string:"state",relativeTo:base)!.absoluteURL)
        req.timeoutInterval = 2
        session.dataTask(with:req) { [weak self] data,_,err in
            let value = data.flatMap { try? JSONSerialization.jsonObject(with:$0) } as? [String:Any]
            DispatchQueue.main.async {
                guard let self = self else { return }; self.loading = false
                if let value = value { self.snapshot = value; self.render(value) }
                else { self.error.stringValue = "GUI connection lost: \(err?.localizedDescription ?? "invalid snapshot")" }
            }
        }.resume()
    }
    func render(_ s:[String:Any]) {
        let states = ["Idle","Dialling","V.8 negotiation","Training","Connected","Hanging up"]
        let mods = ["—","V.91","V.90","V.34","V.22bis","x2","V.32bis","Clear channel","V.21","Bell 103","V.23"]
        let state = s["state"] as? Int ?? 0, mod = s["modulation"] as? Int ?? 0
        let dataReady = (s["data_ready"] as? Int ?? 0)>0
        let call = (s["age"] as? Double ?? 9)>2 ? "Telemetry stale" : states.indices.contains(state) ? (state == 4 && !dataReady ? "Carrier up · negotiating link" : states[state]) : "Starting"
        let mode = (s["v92"] as? Int ?? 0)>0 ? "V.92" : mods.indices.contains(mod) ? mods[mod] : "—"
        let transport = s["hardware"] != nil ? "Analogue" : (s["law"] as? Int ?? 0)>0 ? "PCMA" : "PCMU"
        status.stringValue = "\(call)  ·  \(mode)  ·  \(transport)  |  RX \(s["rx_signal"] as? String ?? "—")  |  TX \(s["tx_signal"] as? String ?? "—")"
        dataInput.isEnabled = dataReady; sendData.isEnabled = dataReady
        let local = s["loopback"] as? [String:Any]
        loopbackRow.isHidden = local == nil
        if let local = local {
            let profiles = local["profiles"] as? [[String]] ?? []
            if loopbackProfiles.isEmpty {
                loopbackProfiles = profiles.compactMap { $0.first }
                loopbackMode.addItems(withTitles:profiles.compactMap { $0.last })
                if let i = loopbackProfiles.firstIndex(of:"v22") { loopbackMode.selectItem(at:i) }
            }
            let busy = local["busy"] as? Bool ?? false
            startLoop.isEnabled = !busy && state == 0 && (local["peer_state"] as? Int ?? 0) == 0 && (local["ready"] as? Bool ?? false)
            loopbackMode.isEnabled = startLoop.isEnabled
            pattern.isEnabled = dataReady && !busy && (local["pattern_ready"] as? Bool ?? false)
            loopbackStatus.stringValue = dataReady && !pattern.isEnabled ? "Waiting for both carriers to settle" : dataReady ? "Echoed \(local["echoed"] as? Int ?? 0) bytes" : local["status"] as? String ?? ""
        }
        let hardware = s["hardware"] as? [String:Any]
        hardwareLabel.isHidden = hardware == nil
        if let hardware = hardware {
            renderApple(s,hardware)
            window.title = "Modem · Apple USB analogue line"
            hardwareLabel.stringValue = "Apple USB · \(s["hardware_status"] as? String ?? "Starting") · USB \(hardware["usb_address"] as? String ?? "Automatic") · Audio \(hardware["audio_uid"] as? String ?? "Automatic") · ATDnumber to dial; ATA waits for a ring"
        }
        let streams = s["streams"] as? [String:[[Any]]] ?? [:]
        at.update(streams["at"] ?? [],binary:false); serial.update(streams["data"] ?? [],binary:true); log.update(streams["log"] ?? [],binary:false)
        let paths = s["ports"] as? [String:String] ?? [:]
        atPath.stringValue = paths["at"] ?? ""; dataPath.stringValue = paths["data"] ?? ""
        let currentEpoch = s["epoch"] as? Int ?? 0
        if currentEpoch != epoch { epoch = currentEpoch; eventSeen = 0; history = []; rxWaterfall.reset(); txWaterfall.reset(); spectrum.reset(); eyeCountSeen = -1; eyeUpdated = .distantPast }
        let events = s["events"] as? [String] ?? [], count = s["event_count"] as? Int ?? 0
        for (i,event) in events.enumerated() where count-events.count+i >= eventSeen { history.append(event) }
        eventSeen = count; history = Array(history.suffix(120)); training.replace(history.joined(separator:"\n"))
        if let exit = s["exit"] as? Int { error.stringValue = "Modem exited: \(exit)" }
        else if error.stringValue.hasPrefix("Modem exited:") { error.stringValue = "" }
        var display = s
        let useRaw = hardware != nil && rawSource.state == .on
        if useRaw, let raw = s["hardware_audio"] as? [String:Any] {
            display["audio"] = raw["audio"]; display["listen"] = raw["listen"]; display["audio_rate"] = raw["rate"]
        }
        display["epoch"] = (s["epoch"] as? Int ?? 0)*2+(useRaw ? 1 : 0)
        monitorAudio(display)
        guard !frozen else { return }
        let audio = display["audio"] as? [[Double]] ?? [[],[]]
        if audio.count == 2 { rx.samples = audio[0]; tx.samples = audio[1]; eye.samples = (s["audio"] as? [[Double]] ?? [[],[]])[max(0,eyeDir.indexOfSelectedItem)]; histogram.samples = audio[0] }
        let rate = display["audio_rate"] as? Double ?? 8000
        if spectrum.sampleRate != rate {
            spectrum.sampleRate = rate; spectrum.reset()
            for waterfall in [rxWaterfall,txWaterfall] { waterfall.sampleRate = rate; waterfall.reset() }
        }
        let listening = display["listen"] as? [[String:Any]] ?? []
        if audio.count == 2 && listening.count == 2 {
            rxWaterfall.carrier = s["rx_carrier"] as? Double ?? 0
            txWaterfall.carrier = s["tx_carrier"] as? Double ?? 0
            let counts = listening.map { $0["count"] as? Int ?? 0 }
            spectrum.carriers = [rxWaterfall.carrier,txWaterfall.carrier]
            spectrum.update(audio,counts:counts)
            let phase = states.indices.contains(state) ? states[state] : "Unknown"
            rxWaterfall.update(audio[0],count:counts[0],phase:phase+" · "+(s["rx_signal"] as? String ?? "—"))
            txWaterfall.update(audio[1],count:counts[1],phase:phase+" · "+(s["tx_signal"] as? String ?? "—"))
        }
        constellation.kind = mod >= 8 ? .recoveredEye : .constellation
        constellation.eyeMessage = "\(mods[min(mod, mods.count - 1)]) uses frequency shifts · no QAM constellation"
        constellation.points = s["iq"] as? [[[Double]]] ?? [[],[]]
        eye.pcm = (s[eyeDir.indexOfSelectedItem == 0 ? "rx_pcm" : "tx_pcm"] as? Int ?? 0)>0
        eye.baud = Double(s[eyeDir.indexOfSelectedItem == 0 ? "rx_baud" : "tx_baud"] as? Int ?? 3200); if eye.baud < 300 { eye.baud = 3200 }
        eye.carrier = s[eyeDir.indexOfSelectedItem == 0 ? "rx_carrier" : "tx_carrier"] as? Double ?? 0
        carrierLabel.stringValue = eye.pcm ? "PCM eye · 8,000 samples/s" : eye.carrier > 0 ? String(format:"%@ · %@ %.1f Hz · %g baud",eye.pcm ? "PCM eye" : "QAM eye",eyeDir.indexOfSelectedItem == 0 ? "RX recovered carrier" : "TX nominal carrier",eye.carrier,eye.baud) : "RX carrier: waiting for QAM receiver"
        if mod >= 8 { carrierLabel.stringValue = "\(mods[min(mod, mods.count - 1)]) FSK · RX \(Int(s["rx_baud"] as? Double ?? 0)) baud · RX centre \(Int(s["rx_carrier"] as? Double ?? 0)) Hz" }
        eye.phase = 0
        let eyeCount = s["eye_count"] as? Int ?? 0
        if eyeCount != eyeCountSeen { eyeCountSeen = eyeCount; if eyeCount > 0 { eyeUpdated = Date() } }
        eye.kind = eyeSource.indexOfSelectedItem == 2 ? .eye : .recoveredEye
        eye.beforeEqualizer = eyeSource.indexOfSelectedItem == 1
        let available = eyeDir.indexOfSelectedItem == 0 && !eye.pcm && Date().timeIntervalSince(eyeUpdated)<1 && (s["age"] as? Double ?? 9)<1
        eye.recovered = available ? (s["eye"] as? [[Double]] ?? []).filter { $0.count == 5 } : []
        eye.eyeMessage = eyeDir.indexOfSelectedItem == 1 ? "TX has no receiver clock · select Line estimate" : eye.pcm ? "PCM recovery tap unavailable · select Line estimate" : "Waiting for live recovered T/2 samples"
        if mod >= 8 { eye.kind = .recoveredEye; eye.recovered = []; eye.eyeMessage = "FSK · use waveform, spectrum or waterfall" }
        for plot in [rx,tx,constellation,eye,histogram,spectrum] as [NSView] { plot.needsDisplay = true }
        let values = s["wire"] as? [[String:Any]] ?? []
        for (i,console) in [rxWire,txWire].enumerated() where values.indices.contains(i) {
            let count = values[i]["count"] as? Int ?? 0
            let runs = values[i]["runs"] as? [[Int]] ?? []
            var lines: [String] = [], row: [String] = []
            for run in runs where run.count == 2 {
                if run[1] >= 4 {
                    if !row.isEmpty { lines.append(row.joined(separator:" ")); row = [] }
                    lines.append(String(format:"%02X ×%d",run[0],run[1]))
                } else {
                    for _ in 0..<run[1] { row.append(String(format:"%02X",run[0])); if row.count == 16 { lines.append(row.joined(separator:" ")); row = [] } }
                }
            }
            if !row.isEmpty { lines.append(row.joined(separator:" ")) }
            console.replace(count == 0 ? "No line bytes yet" : "\(count) bytes total\n"+lines.joined(separator:"\n"))

        }
    }
}

guard CommandLine.arguments.count == 2, let url = URL(string:CommandLine.arguments[1]), url.host == "127.0.0.1" else {
    fputs("Usage: modem_gui_native <local GUI URL>\n",stderr); exit(2)
}
let app = NSApplication.shared
app.setActivationPolicy(.regular)
let delegate = App(url)
app.delegate = delegate
app.run()
