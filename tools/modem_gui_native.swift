// Native passive modem console. AppKit owns rendering; the modem's control
// loop publishes bounded snapshots. Signal data never passes through this UI.
import AppKit
import AVFoundation

let rxColor = NSColor.systemTeal
let txColor = NSColor.systemOrange

final class SignalPlot: NSView {
    enum Kind { case waveform, constellation, eye, histogram, spectrum }
    var kind: Kind = .waveform
    var samples: [Double] = []
    var points: [[[Double]]] = [[], []]
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
        if kind == .histogram || kind == .spectrum {
            var bins = [Double](repeating:0,count:64)
            if kind == .histogram {
                for sample in samples { bins[min(63,max(0,Int((sample+32768)/1024)))] += 1 }
            } else if !samples.isEmpty {
                for k in 0..<64 {
                    var re = 0.0, im = 0.0
                    for (i,v) in samples.enumerated() {
                        let win = 0.5-0.5*cos(2*Double.pi*Double(i)/Double(max(1,samples.count-1)))
                        let angle = 2*Double.pi*Double(k)*Double(i)/128
                        re += v*win*cos(angle); im -= v*win*sin(angle)
                    }
                    bins[k] = sqrt(re*re+im*im)
                }
            }
            let peak = max(1,bins.max() ?? 1)
            color.setFill()
            for (i,v) in bins.enumerated() { NSRect(x:Double(i)*w/64,y:h-v/peak*h*0.9,width:max(1,w/64-1),height:v/peak*h*0.9).fill() }
            if kind == .spectrum && carrier > 0 {
                NSColor.systemYellow.setStroke(); let marker = NSBezierPath()
                marker.move(to:NSPoint(x:carrier/4000*w,y:0)); marker.line(to:NSPoint(x:carrier/4000*w,y:h)); marker.stroke()
            }
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
    let error = NSTextField(labelWithString:"")
    let rx = SignalPlot(), tx = SignalPlot(), constellation = SignalPlot(), eye = SignalPlot()
    let at = Console(height:64), serial = Console(height:64), training = Console(height:28), log = Console(height:130)
    let rxWire = Console(height:54), txWire = Console(height:54)
    let histogram = SignalPlot(), spectrum = SignalPlot()
    let carrierLabel = NSTextField(labelWithString:"Carrier: waiting")
    let listen = NSPopUpButton()
    let audioEngine = AVAudioEngine(), player = AVAudioPlayerNode()
    var audioSeen = [0,0], audioEpoch = -1, queued = 0, audioGeneration = 0
    let atInput = NSTextField(), dataInput = NSTextField()
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
        window = NSWindow(contentRect:NSRect(x:0,y:0,width:1200,height:760),
            styleMask:[.titled,.closable,.miniaturizable,.resizable], backing:.buffered, defer:false)
        window.title = "Modem · Live line"
        window.minSize = NSSize(width:1000,height:720)
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
        dataMode.addItems(withTitles:["UTF-8 + CRLF","Hex bytes"])
        atInput.placeholderString = "ATDnumber or AT command"; atInput.target = self; atInput.action = #selector(sendAT)
        dataInput.placeholderString = "Serial payload"; dataInput.target = self; dataInput.action = #selector(sendSerial)
        sendData = button("Send data",#selector(sendSerial))
        histogram.kind = .histogram; spectrum.kind = .spectrum
        for plot in [histogram,spectrum] { plot.heightAnchor.constraint(equalToConstant:100).isActive = true }
        listen.addItems(withTitles:["Audio off","Listen RX","Listen TX"])
        audioEngine.attach(player)
        audioEngine.connect(player,to:audioEngine.mainMixerNode,format:AVAudioFormat(standardFormatWithSampleRate:8000,channels:1))
        player.volume = 0.25
        let diagnostics = NSTabView()
        func tab(_ title:String,_ view:NSView) { let item = NSTabViewItem(identifier:title); item.label = title; item.view = view; diagnostics.addTabViewItem(item) }
        tab("Signal",pair(panel("Received constellation · teal samples / orange decisions",[constellation]),panel("Eye · follows selected direction",[eye])))
        tab("Audio",pair(panel("RX amplitude histogram · −32768 to +32767",[histogram]),panel("RX spectrum · 0 to 4000 Hz",[spectrum])))
        tab("Process log",log.scroll)
        diagnostics.heightAnchor.constraint(equalToConstant:160).isActive = true
        let items: [NSView] = [
            row([status,listen,button("Freeze",#selector(freeze))]),error,
            pair(panel("RX line · 64 ms",[rx]),panel("TX line · 64 ms",[tx])),
            row([eyeDir,carrierLabel,label("Carrier demodulation → I/Q points. TCM uses the same QAM signal.")]),
            diagnostics,
            pair(panel("RX wire · repeated bytes collapsed",[rxWire.scroll]),panel("TX wire · repeated bytes collapsed",[txWire.scroll])),
            pair(panel("AT control",[row([button("Answer",#selector(answer)),button("Hang up",#selector(hangup)),button("Info",#selector(info))]),at.scroll,row([atInput,button("Send AT",#selector(sendAT))])]),
                 panel("Serial data",[serial.scroll,row([dataInput,dataMode,sendData])])),
            training.scroll]
        for view in items { root.addArrangedSubview(view); view.widthAnchor.constraint(equalTo:root.widthAnchor,constant:-20).isActive = true }
        at.scroll.toolTip = "Separate AT PTY; latest output. No history is saved to disk."
        rxWire.scroll.toolTip = "Framed/compressed line bits before modulation scrambling, packed LSB first. Latest 256 byte runs."
        for path in [atPath,dataPath] { path.font = .monospacedSystemFont(ofSize:10,weight:.regular); path.lineBreakMode = .byTruncatingMiddle }
        if let screen = NSScreen.main { window.setFrame(NSRect(x:screen.visibleFrame.midX-600,y:screen.visibleFrame.midY-380,width:min(1200,screen.visibleFrame.width),height:min(760,screen.visibleFrame.height)),display:true) }
        window.center(); window.makeKeyAndOrderFront(nil); NSApp.activate(ignoringOtherApps:true)
        timer = Timer.scheduledTimer(withTimeInterval:0.15,repeats:true) { [weak self] _ in self?.poll() }
        poll()
    }
    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }
    func applicationWillTerminate(_ notification: Notification) { timer?.invalidate() }
    @objc func freeze(_ sender:NSButton) { frozen.toggle(); sender.title = frozen ? "Resume plots" : "Freeze plots" }
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
    func send(_ port:String, _ data:Data) {
        guard data.count <= 8000 else { error.stringValue = "Send at most 8,000 bytes at once"; return }
        var req = URLRequest(url:URL(string:"send",relativeTo:base)!.absoluteURL)
        req.httpMethod = "POST"
        req.setValue("application/json",forHTTPHeaderField:"Content-Type")
        let components = URLComponents(url:base,resolvingAgainstBaseURL:true)!
        req.setValue("http://127.0.0.1:\(components.port!)",forHTTPHeaderField:"Origin")
        req.httpBody = try? JSONSerialization.data(withJSONObject:["port":port,"data":data.base64EncodedString()])
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
    func monitorAudio(_ s:[String:Any]) {
        let current = s["epoch"] as? Int ?? 0
        if current != audioEpoch { audioEpoch = current; audioSeen = [0,0]; player.stop(); queued = 0; audioGeneration += 1 }
        let frames = s["listen"] as? [[String:Any]] ?? []
        for (d,frame) in frames.enumerated() where d < 2 {
            let count = frame["count"] as? Int ?? 0, hex = Array(frame["hex"] as? String ?? "")
            let n = hex.count/4, fresh = min(n,max(0,count-audioSeen[d])); audioSeen[d] = count
            guard listen.indexOfSelectedItem == d+1, fresh > 0, queued < 3 else { continue }
            do { if !audioEngine.isRunning { try audioEngine.start() } } catch { self.error.stringValue = "Audio: \(error.localizedDescription)"; continue }
            guard let format = AVAudioFormat(standardFormatWithSampleRate:8000,channels:1), let buffer = AVAudioPCMBuffer(pcmFormat:format,frameCapacity:AVAudioFrameCount(fresh)), let channel = buffer.floatChannelData?[0] else { continue }
            buffer.frameLength = AVAudioFrameCount(fresh)
            for i in 0..<fresh {
                let at = (n-fresh+i)*4
                let lo = UInt16(String(hex[at..<at+2]),radix:16) ?? 0, hi = UInt16(String(hex[at+2..<at+4]),radix:16) ?? 0
                channel[i] = Float(Int16(bitPattern:lo | hi<<8))/32768
            }
            queued += 1
            let generation = audioGeneration
            player.scheduleBuffer(buffer,completionCallbackType:.dataPlayedBack) { [weak self] _ in DispatchQueue.main.async { if let self = self, generation == self.audioGeneration { self.queued = max(0,self.queued-1) } } }
            if !player.isPlaying { player.play() }
        }
        if listen.indexOfSelectedItem == 0 { player.stop(); queued = 0; audioGeneration += 1 }
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
        let mods = ["—","V.91","V.90","V.34","V.22bis","x2","V.32bis","Clear channel"]
        let state = s["state"] as? Int ?? 0, mod = s["modulation"] as? Int ?? 0
        let dataReady = (s["data_ready"] as? Int ?? 0)>0
        let call = (s["age"] as? Double ?? 9)>2 ? "Telemetry stale" : states.indices.contains(state) ? (state == 4 && !dataReady ? "Carrier up · negotiating link" : states[state]) : "Starting"
        let mode = (s["v92"] as? Int ?? 0)>0 ? "V.92" : mods.indices.contains(mod) ? mods[mod] : "—"
        status.stringValue = "\(call)  ·  \(mode)  ·  \((s["law"] as? Int ?? 0)>0 ? "PCMA" : "PCMU")  |  RX \(s["rx_signal"] as? String ?? "—")  |  TX \(s["tx_signal"] as? String ?? "—")"
        dataInput.isEnabled = dataReady; sendData.isEnabled = dataReady
        let streams = s["streams"] as? [String:[[Any]]] ?? [:]
        at.update(streams["at"] ?? [],binary:false); serial.update(streams["data"] ?? [],binary:true); log.update(streams["log"] ?? [],binary:false)
        let paths = s["ports"] as? [String:String] ?? [:]
        atPath.stringValue = paths["at"] ?? ""; dataPath.stringValue = paths["data"] ?? ""
        let currentEpoch = s["epoch"] as? Int ?? 0
        if currentEpoch != epoch { epoch = currentEpoch; eventSeen = 0; history = [] }
        let events = s["events"] as? [String] ?? [], count = s["event_count"] as? Int ?? 0
        for (i,event) in events.enumerated() where count-events.count+i >= eventSeen { history.append(event) }
        eventSeen = count; history = Array(history.suffix(120)); training.replace(history.joined(separator:"\n"))
        if let exit = s["exit"] as? Int { error.stringValue = "Modem exited: \(exit)" }
        monitorAudio(s)
        guard !frozen else { return }
        let audio = s["audio"] as? [[Double]] ?? [[],[]]
        if audio.count == 2 { rx.samples = audio[0]; tx.samples = audio[1]; eye.samples = audio[max(0,eyeDir.indexOfSelectedItem)]; histogram.samples = audio[0]; spectrum.samples = audio[0] }
        constellation.points = s["iq"] as? [[[Double]]] ?? [[],[]]
        eye.pcm = (s[eyeDir.indexOfSelectedItem == 0 ? "rx_pcm" : "tx_pcm"] as? Int ?? 0)>0
        eye.baud = Double(s[eyeDir.indexOfSelectedItem == 0 ? "rx_baud" : "tx_baud"] as? Int ?? 3200); if eye.baud < 300 { eye.baud = 3200 }
        eye.carrier = s[eyeDir.indexOfSelectedItem == 0 ? "rx_carrier" : "tx_carrier"] as? Double ?? 0; spectrum.carrier = s["rx_carrier"] as? Double ?? 0
        carrierLabel.stringValue = eye.pcm ? "PCM eye · 8,000 samples/s" : eye.carrier > 0 ? String(format:"%@ · %@ %.1f Hz · %g baud",eye.pcm ? "PCM eye" : "QAM eye",eyeDir.indexOfSelectedItem == 0 ? "RX recovered carrier" : "TX nominal carrier",eye.carrier,eye.baud) : "RX carrier: waiting for QAM receiver"
        eye.phase = 0
        for plot in [rx,tx,constellation,eye,histogram,spectrum] { plot.needsDisplay = true }
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
