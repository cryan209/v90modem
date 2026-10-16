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
    pair = pair.replace("import {negotiationRecords} from './negotiation.mjs';", '')
    negotiation=(DIST/'negotiation.mjs').read_text().replace('export const ', 'const ').replace('export function ', 'function ')
    pair = pair.replace('export function ', 'function ').replace('export async function ', 'async function ')
    binary = base64.b64encode((DIST/'modem-engine.wasm').read_bytes()).decode()
    assert pair.count('const options=(side)=>({') == 1, 'WASM factory changed; update offline packager'
    pair = "const wasmBytes=Uint8Array.from(atob('"+binary+"'),c=>c.charCodeAt(0));\n"+pair
    pair = pair.replace('const options=(side)=>({', 'const options=(side)=>({wasmBinary:wasmBytes,locateFile:name=>name,')
    pair = pair.replace('createModem({print:', 'createModem({wasmBinary:wasmBytes,locateFile:name=>name,print:')
    worker = (DIST/'worker.mjs').read_text().replace("import {runPair,runReliability} from './engine-pair.mjs';", '')
    encoded = base64.b64encode((engine+'\n'+negotiation+'\n'+pair+'\n'+worker).encode()).decode()
    fsk = (DIST/'fsk.mjs').read_text().replace('export const ', 'const ').replace('export function ', 'function ')
    app = (DIST/'app.mjs').read_text().replace("import {transmit,receive,wav,RATE} from './fsk.mjs';", '')
    app = app.replace("import {inspectorText} from './negotiation.mjs';", '')
    app = app.replace("new Worker('./worker.mjs',{type:'module'})", "createOfflineModemRunner()")
    setup = "const offlineWorkerUrl=URL.createObjectURL(new Blob([new TextDecoder().decode(Uint8Array.from(atob(document.getElementById('modem-worker').textContent.trim()),c=>c.charCodeAt(0)))],{type:'text/javascript'}));\nwindow.addEventListener('pagehide',()=>URL.revokeObjectURL(offlineWorkerUrl));\n"
    setup += """
// file:// has an opaque origin in some browsers, which can reject blob workers.
// The identical engine can run here, yielding between batches and honouring Stop.
class InlineModemRunner {
  constructor(){this.cancelled=false;}
  terminate(){this.cancelled=true;}
  postMessage(data){
    const reliability=data.task==='reliability';
    const progress=p=>{if(!this.cancelled)this.onmessage?.({data:{type:reliability?'reliability-progress':'progress',...p}});};
    const promise=reliability?runReliability(data.settings,progress,()=>this.cancelled)
      :runPair(data.mode,data.text,progress,()=>this.cancelled,data.startup);
    promise.then(result=>{
      if(!this.cancelled)this.onmessage?.({data:{type:reliability?'reliability-result':'result',...result}});
    }).catch(error=>{
      if(!this.cancelled)this.onmessage?.({data:{type:'error',message:error.message}});
    });
  }
}
function createOfflineModemRunner(){
  return location.protocol==='file:' ? new InlineModemRunner() : new Worker(offlineWorkerUrl,{type:'module'});
}
"""
    script = engine+'\n'+negotiation+'\n'+pair+'\n'+fsk+'\n'+setup+app
    assert '</script' not in script.lower()
    html = html.replace('<link rel="stylesheet" href="style.css">', '<style>'+(DIST/'style.css').read_text()+'</style>')
    license_text=html_escape.escape((DIST/'COPYING.spandsp').read_text())
    html=html.replace('<a href="NOTICE.txt">Source &amp; license</a>', '<a href="#license">Source &amp; license</a>')
    html=html.replace('</main>', '<details id="license" class="license"><summary>Source and SpanDSP LGPL 2.1 license</summary><p>DSP source and rebuild instructions: <a href="https://github.com/cryan209/v90modem">cryan209/v90modem</a> (spandsp-master, web/modem-lab, and tools/build_modem_wasm.py).</p><pre>'+license_text+'</pre></details></main>')
    html = html.replace('<script type="module" src="app.mjs"></script>', '<script id="modem-worker" type="text/plain">'+encoded+'</script><script type="module">'+script+'</script>')
    # Keep both existing single-file entry points on the same engine build.
    for target in (ROOT/'artifacts/modem-lab.html', DIST/'standalone.html'):
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(html)
        print(target)


if __name__ == '__main__':
    main()
