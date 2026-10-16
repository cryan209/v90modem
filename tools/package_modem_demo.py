#!/usr/bin/env python3
"""Package the browser modem as one offline HTML file, including its WASM.
The modular source remains in web/modem-lab/dist; no third-party bundler needed.
"""
import base64
import html as html_escape
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DIST = ROOT / 'web/modem-lab/dist'


def main():
    html = (DIST/'index.html').read_text()
    engine = (DIST/'modem-engine.mjs').read_text().replace('export default createModem;', '')
    pair = (DIST/'engine-pair.mjs').read_text().replace("import createModem from './modem-engine.mjs';", '')
    pair = pair.replace('export function ', 'function ').replace('export async function ', 'async function ')
    binary = base64.b64encode((DIST/'modem-engine.wasm').read_bytes()).decode()
    pair = pair.replace('const options=()=>({', "const wasmBytes=Uint8Array.from(atob('"+binary+"'),c=>c.charCodeAt(0));\n  const options=()=>({wasmBinary:wasmBytes,locateFile:name=>name,")
    worker = (DIST/'worker.mjs').read_text().replace("import {runPair} from './engine-pair.mjs';", '')
    encoded = base64.b64encode((engine+'\n'+pair+'\n'+worker).encode()).decode()
    fsk = (DIST/'fsk.mjs').read_text().replace('export const ', 'const ').replace('export function ', 'function ')
    app = (DIST/'app.mjs').read_text().replace("import {transmit,receive,wav,RATE} from './fsk.mjs';", '')
    app = app.replace("new Worker('./worker.mjs',{type:'module'})", "new Worker(offlineWorkerUrl,{type:'module'})")
    setup = "const offlineWorkerUrl=URL.createObjectURL(new Blob([new TextDecoder().decode(Uint8Array.from(atob(document.getElementById('modem-worker').textContent.trim()),c=>c.charCodeAt(0)))],{type:'text/javascript'}));\nwindow.addEventListener('pagehide',()=>URL.revokeObjectURL(offlineWorkerUrl));\n"
    script = fsk+'\n'+setup+app
    assert '</script' not in script.lower()
    html = html.replace('<link rel="stylesheet" href="style.css">', '<style>'+(DIST/'style.css').read_text()+'</style>')
    license_text=html_escape.escape((DIST/'COPYING.spandsp').read_text())
    html=html.replace('<a href="NOTICE.txt">Source &amp; license</a>', '<a href="#license">Source &amp; license</a>')
    html=html.replace('</main>', '<details id="license" class="license"><summary>Source and SpanDSP LGPL 2.1 license</summary><p>DSP source and rebuild instructions: <a href="https://github.com/cryan209/v90modem">cryan209/v90modem</a> (spandsp-master, web/modem-lab, and tools/build_modem_wasm.py).</p><pre>'+license_text+'</pre></details></main>')
    html = html.replace('<script type="module" src="app.mjs"></script>', '<script id="modem-worker" type="text/plain">'+encoded+'</script><script type="module">'+script+'</script>')
    target=ROOT/'artifacts/modem-lab.html'
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(html)
    print(target)


if __name__ == '__main__':
    main()
