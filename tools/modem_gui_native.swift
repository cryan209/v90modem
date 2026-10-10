// Native passive modem console. AppKit owns rendering; the modem's control
// loop publishes bounded snapshots. Signal data never passes through this UI.
import AppKit

let rxColor = NSColor.systemTeal
let txColor = NSColor.systemOrange

final class SignalPlot: NSView {
    enum Kind { case waveform, constellation, eye }
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
        for i in 1..<8 { grid.move(to: NSPoint(x: w*Double(i)/8, y: 0)); grid.line(to: NSPoint(x: w*Double(i)/8, y: h)) }
        for i in 1..<4 { grid.move(to: NSPoint(x: 0, y: h*Double(i)/4)); grid.line(to: NSPoint(x: w, y: h*Double(i)/4)) }
        grid.stroke()
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
            for sub in 0..<max(0,(y.count-1)*8) {
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
        scroll.hasVerticalScroller = true
        scroll.translatesAutoresizingMaskIntoConstraints = false
        scroll.heightAnchor.constraint(equalToConstant: height).isActive = true
    }
    func replace(_ value: String) {
        let bounded = String(value.suffix(24000))
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
    let at = Console(height:140), serial = Console(height:140), training = Console(height:120), log = Console(height:160)
    let rxWire = Console(height:130), txWire = Console(height:130)
    let atInput = NSTextField(), dataInput = NSTextField()
    let eyeMode = NSPopUpButton(), eyeDir = NSPopUpButton(), wireMode = NSPopUpButton(), dataMode = NSPopUpButton()
    let baud = NSTextField(string:"3200"), carrier = NSTextField(string:"1800"), phase = NSTextField(string:"0")
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
        let box = NSBox(); box.title = title; box.contentViewMargins = NSSize(width:12,height:12)
        let stack = NSStackView(views:children)
        stack.orientation = .vertical; stack.alignment = .leading; stack.spacing = 8
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
        window = NSWindow(contentRect:NSRect(x:0,y:0,width:1380,height:900),
            styleMask:[.titled,.closable,.miniaturizable,.resizable], backing:.buffered, defer:false)
        window.title = "Modem · Live line"
        window.minSize = NSSize(width:1050,height:650)
        window.delegate = self
        let scroll = NSScrollView(); scroll.hasVerticalScroller = true
        let root = NSStackView(); root.orientation = .vertical; root.alignment = .leading; root.spacing = 12
        root.edgeInsets = NSEdgeInsets(top:18,left:18,bottom:18,right:18)
        root.translatesAutoresizingMaskIntoConstraints = false
        scroll.documentView = root
        window.contentView = scroll
        root.widthAnchor.constraint(equalTo:scroll.contentView.widthAnchor).isActive = true
        for plot in [rx,tx,constellation,eye] { plot.heightAnchor.constraint(equalToConstant:165).isActive = true }
        tx.color = txColor; constellation.kind = .constellation; eye.kind = .eye
        status.font = .systemFont(ofSize:14,weight:.semibold)
        error.textColor = .systemRed
        eyeMode.addItems(withTitles:["PCM · reconstructed DS0","QAM / TCM · mixed I"])
        eyeDir.addItems(withTitles:["RX","TX"])
        wireMode.addItems(withTitles:["Datapump line bits · packed octets","G.711 DS0 · exact codewords"])
        dataMode.addItems(withTitles:["UTF-8 + CRLF","Hex bytes"])
        for input in [baud,carrier,phase] { input.widthAnchor.constraint(equalToConstant:65).isActive = true }
        atInput.placeholderString = "ATDnumber or AT command"; atInput.target = self; atInput.action = #selector(sendAT)
        dataInput.placeholderString = "Serial payload"; dataInput.target = self; dataInput.action = #selector(sendSerial)
        sendData = button("Send data",#selector(sendSerial))
        let items: [NSView] = [
            row([status,button("Freeze plots",#selector(freeze))]),
            label("Signal → datapump wire → serial data. AT control stays separate. All histories are bounded in memory."), error,
            pair(panel("RX waveform",[rx,label("64 ms · 8,000 samples/s · fixed ±32768 scale · expanded G.711")]),
                 panel("TX waveform",[tx,label("64 ms · exact transmitted G.711 levels")])),
            pair(panel("Receive QAM / TCM constellation",[constellation,label("Teal: receiver report. Orange: reported decisions, including V.34 trellis traceback. Waiting receivers show no points.")]),
                 panel("Line-derived eye",[row([eyeMode,eyeDir]),
                    row([label("Baud"),baud,label("Carrier Hz"),carrier,label("Phase T"),phase]),eye,
                    label("Two symbol periods. Manual timing; display-only sinc reconstruction and QAM mixing/smoothing. Not the receiver's recovered eye.")])),
            panel("Live wire bytes",[wireMode,label("Line bits after V.14/V.42 framing/compression, before modulation scrambling/mapping. Packed LSB first from call start, not DTE characters or aligned LAPM frames. DS0 view shows exact G.711 codes, including training."),
                pair(panel("RX",[rxWire.scroll]),panel("TX",[txWire.scroll]))]),
            pair(panel("AT control interface",[atPath,row([button("Answer",#selector(answer)),button("Hang up",#selector(hangup)),button("Modulation",#selector(modulation)),button("Info",#selector(info))]),at.scroll,row([atInput,button("Send AT",#selector(sendAT))])]),
                 panel("Serial data interface",[dataPath,serial.scroll,row([dataInput,dataMode,sendData]),label("DTE payload after deframing/decompression. Nonprintable bytes appear as \\xNN.")])),
            panel("Dialling & carrier acquisition",[training.scroll]),
            panel("Modem process log · latest output",[log.scroll])]
        for view in items { root.addArrangedSubview(view); view.widthAnchor.constraint(equalTo:root.widthAnchor,constant:-36).isActive = true }
        for path in [atPath,dataPath] { path.font = .monospacedSystemFont(ofSize:10,weight:.regular); path.lineBreakMode = .byTruncatingMiddle }
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
        guard !frozen else { return }
        let audio = s["audio"] as? [[Double]] ?? [[],[]]
        if audio.count == 2 { rx.samples = audio[0]; tx.samples = audio[1]; eye.samples = audio[max(0,eyeDir.indexOfSelectedItem)] }
        constellation.points = s["iq"] as? [[[Double]]] ?? [[],[]]
        eye.pcm = eyeMode.indexOfSelectedItem == 0
        eye.baud = Double(baud.stringValue) ?? 3200; eye.carrier = Double(carrier.stringValue) ?? 1800; eye.phase = Double(phase.stringValue) ?? 0
        for plot in [rx,tx,constellation,eye] { plot.needsDisplay = true }
        let values = s[wireMode.indexOfSelectedItem == 0 ? "wire" : "pcm"] as? [[String:Any]] ?? []
        for (i,console) in [rxWire,txWire].enumerated() where values.indices.contains(i) {
            let count = values[i]["count"] as? Int ?? 0, hex = values[i]["hex"] as? String ?? ""
            let chars = Array(hex), n = chars.count/2
            var lines: [String] = []
            for start in stride(from:0,to:n,by:16) {
                lines.append((start..<min(n,start+16)).map { String(chars[2*$0...2*$0+1]) }.joined(separator:" "))
            }
            console.replace(count == 0 ? "No bytes observed" : "\(count) octets observed · latest \(n)\n"+lines.joined(separator:"\n"))
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
