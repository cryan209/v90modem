// Exercise the generated HTML's actual file:// runner, without an HTTP server.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const html=await fs.readFile(new URL('../../artifacts/modem-lab.html',import.meta.url),'utf8');
assert.equal(await fs.readFile(new URL('./dist/standalone.html',import.meta.url),'utf8'),html,'Standalone HTML contains a different engine build');
const embedded=html.match(/<script id="modem-worker" type="text\/plain">([\s\S]*?)<\/script>/)[1];
const source=html.match(/<script type="module">([\s\S]*?)<\/script>/)[1];
const canvas=new Proxy({}, {get:()=>()=>{}}),elements=new Map();
globalThis.document={getElementById(id){if(!elements.has(id))elements.set(id,{value:id==='message'?'Hello from the browser!':id==='v34Rate'?'0':'0',textContent:id==='modem-worker'?embedded:'',open:false,disabled:false,checked:id==='startupV8'||id==='inspectFollow',addEventListener(){},setAttribute(){},classList:{add(){},remove(){},toggle(){}},getContext:()=>canvas});return elements.get(id);},querySelectorAll:()=>[]};
globalThis.window={addEventListener(){}};
globalThis.cancelAnimationFrame=()=>{};
globalThis.location={protocol:'file:'};
globalThis.Worker=class{constructor(){throw new Error('file:// must not construct a Worker');}};
const folder=await fs.mkdtemp(path.join(os.tmpdir(),'modem-offline-'));
try{
 const entry=path.join(folder,'offline.mjs');
 await fs.writeFile(entry,source+'\nexport {createOfflineModemRunner,InlineModemRunner,runPair,select,updateStartup};\n');
 const {createOfflineModemRunner,InlineModemRunner,runPair,select,updateStartup}=await import(pathToFileURL(entry));
 select('34');assert.equal(document.getElementById('v34Rate').disabled,false);assert.equal(document.getElementById('v34Rate').value,'0');assert.equal(document.getElementById('startupV8').disabled,true);assert.equal(document.getElementById('startupV8').checked,true);assert.equal(document.getElementById('startupBis').disabled,false);
 select('x2');assert.equal(document.getElementById('startupV8').disabled,true);assert.equal(document.getElementById('startupV8').checked,false);assert.equal(document.getElementById('startupBis').disabled,true);assert.equal(document.getElementById('showConstellation').disabled,true);
 select('flex');assert.equal(document.getElementById('dial').textContent,'▶ Encode & send');
 select('32');assert.equal(document.getElementById('startupV8').disabled,false);document.getElementById('startupV8').checked=false;updateStartup();assert.equal(document.getElementById('startupBis').disabled,true);
 select('v21');assert.equal(document.getElementById('startupV8').disabled,true);assert.equal(document.getElementById('startupBis').disabled,true);
 console.log('PASS: startup controls gate unsupported combinations');
 for(const [mode,startup] of [['x2',{}],['flex',{}],[32,{v8:false,v8bis:false}],[34,{v8:true,v8bis:true,v34MaxRate:0}],[90,{v8:true,v8bis:true}]]){
  const runner=createOfflineModemRunner();assert.ok(runner instanceof InlineModemRunner);
  const result=await new Promise((resolve,reject)=>{runner.onmessage=({data})=>{if(data.type==='error')reject(new Error(data.message));if(data.type==='result')resolve(data);};runner.postMessage({mode,startup,text:'Offline file test — hello'});});
  assert.ok(result.passed);assert.equal(result.received,'Offline file test — hello');assert.equal(result.reply,['x2','flex'].includes(mode)?'':'ACK: Offline file test — hello');
  if(mode===34){assert.equal(result.txRate,33600);assert.equal(result.rxRate,33600);}
  runner.terminate();console.log(`PASS: packaged file:// ${['x2','flex'].includes(mode)?mode+' downstream PCM':'V.'+mode}, no Worker, verified ${['x2','flex'].includes(mode)?'downstream':'bidirectional'} payload`);
 }
 let cancelled=false;
 await assert.rejects(runPair(32,'cancel',()=>{cancelled=true;},()=>cancelled),/Call cancelled/);
 console.log('PASS: cooperative cancellation');
}finally{await fs.rm(folder,{recursive:true,force:true});}
