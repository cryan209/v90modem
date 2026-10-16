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
const canvas=new Proxy({}, {get:()=>()=>{}}),elements=new Map();let download;
globalThis.document={getElementById(id){if(!elements.has(id))elements.set(id,{value:id==='message'?'Hello from the browser!':id==='v92Startup'?'full':'0',textContent:id==='modem-worker'?embedded:'',open:false,disabled:false,checked:id==='startupV8'||id==='inspectFollow',listeners:{},addEventListener(type,handler){this.listeners[type]=handler;},setAttribute(){},classList:{add(){},remove(){},toggle(){}},getContext:()=>canvas});return elements.get(id);},querySelectorAll:()=>[],createElement(){return {click(){download=this;}};}};
globalThis.window={addEventListener(){}};
globalThis.cancelAnimationFrame=()=>{};
globalThis.location={protocol:'file:'};
globalThis.Worker=class{constructor(){throw new Error('file:// must not construct a Worker');}};
const folder=await fs.mkdtemp(path.join(os.tmpdir(),'modem-offline-'));
try{
 const entry=path.join(folder,'offline.mjs');
 await fs.writeFile(entry,source+'\nexport {createOfflineModemRunner,InlineModemRunner,runPair,select,updateStartup,drawStats,renderReliability,updateReliabilityControls,stop,startReliability};\nexport function setTestResult(value){result=value;} export function getTestReliability(){return reliabilityResult;}\n');
 const {createOfflineModemRunner,InlineModemRunner,runPair,select,updateStartup,drawStats,setTestResult,renderReliability,updateReliabilityControls,stop,startReliability,getTestReliability}=await import(pathToFileURL(entry));
 select('34');assert.equal(document.getElementById('v34Rate').disabled,false);assert.equal(document.getElementById('v34Rate').value,'0');assert.equal(document.getElementById('startupV8').disabled,true);assert.equal(document.getElementById('startupV8').checked,true);assert.equal(document.getElementById('startupBis').disabled,false);
 select('92');assert.equal(document.getElementById('v92Control').hidden,false);assert.equal(document.getElementById('v92Startup').disabled,false);assert.equal(document.getElementById('lineNoise').disabled,true);assert.equal(document.getElementById('lineDelay').disabled,false);
 select('34');assert.equal(document.getElementById('lineNoise').disabled,false);
 select('x2');assert.equal(document.getElementById('startupV8').disabled,true);assert.equal(document.getElementById('startupV8').checked,false);assert.equal(document.getElementById('startupBis').disabled,true);assert.equal(document.getElementById('showConstellation').disabled,true);
 select('flex');assert.equal(document.getElementById('dial').textContent,'▶ Encode & send');
 select('32');assert.equal(document.getElementById('startupV8').disabled,false);document.getElementById('startupV8').checked=false;updateStartup();assert.equal(document.getElementById('startupBis').disabled,true);
 for(const mode of ['v22','v22bis','v32bis']){select(mode);assert.equal(document.getElementById('startupV8').disabled,false);assert.equal(document.getElementById('lineNoise').disabled,false);assert.equal(document.getElementById('reliabilityPanel').hidden,true);}
 for(const mode of ['v23','bell212']){select(mode);assert.equal(document.getElementById('startupV8').disabled,true);assert.equal(document.getElementById('startupV8').checked,false);assert.equal(document.getElementById('startupBis').disabled,true);}
 select('v23');assert.equal(document.getElementById('showConstellation').disabled,true);
 select('91');assert.equal(document.getElementById('startupV8').checked,true);assert.equal(document.getElementById('startupV8').disabled,true);assert.equal(document.getElementById('lineNoise').disabled,true);assert.equal(document.getElementById('lineDelay').disabled,false);assert.equal(document.getElementById('showConstellation').disabled,true);
 select('v21');assert.equal(document.getElementById('startupV8').disabled,true);assert.equal(document.getElementById('startupBis').disabled,true);
 console.log('PASS: startup controls gate unsupported combinations');
 for(const [mode,startup] of [['v22',{v8:true}],['v22bis',{v8:true}],['v32bis',{v8:true}],['v23',{v8:false}],['bell212',{v8:false}],[91,{v8:true}],['x2',{}],['flex',{}],[32,{v8:false,v8bis:false}],[34,{v8:true,v8bis:true,v34MaxRate:0}],[90,{v8:true,v8bis:true,line:{delayMs:80}}],[92,{v8:true,v92Startup:'compare',line:{delayMs:20}}]]){
  const runner=createOfflineModemRunner();assert.ok(runner instanceof InlineModemRunner);
  const result=await new Promise((resolve,reject)=>{runner.onmessage=({data})=>{if(data.type==='error')reject(new Error(data.message));if(data.type==='result')resolve(data);};runner.postMessage({mode,startup,text:'Offline file test — hello'});});
  assert.ok(result.passed);assert.equal(result.received,'Offline file test — hello');assert.equal(result.reply,['x2','flex'].includes(mode)?'':'ACK: Offline file test — hello');
  if(mode===92){
   select('92');setTestResult(result);drawStats(0);assert.equal(document.getElementById('statBytes').textContent,'0');assert.equal(document.getElementById('quickComparison').hidden,true);
   drawStats(result.stats.verifiedAtSeconds);assert.equal(document.getElementById('statBytes').textContent,result.stats.verifiedBytes.toLocaleString());assert.equal(document.getElementById('quickComparison').hidden,false);
   assert.equal(result.modulation,'V92');assert.equal(result.txRate,48000);assert.equal(result.rxRate,56000);assert.ok(result.comparison.full.passed);assert.ok(result.comparison.savedSeconds>0);}
  if(mode===90){assert.equal(result.txRate,31200);assert.equal(result.rxRate,56000);assert.ok(result.events.filter(e=>e.rates).every(e=>e.rates.caller.tx===31200&&e.rates.answerer.rx===31200));select('90');setTestResult(result);drawStats(result.stats.verifiedAtSeconds);assert.equal(document.getElementById('statRates').textContent,'31,200 / 56,000 bit/s');assert.equal(document.getElementById('statRateLabel').textContent,'Caller upload / download');}
  const extra={v22:[1200,1200,'V22'],v22bis:[2400,2400,'V22B'],v32bis:[14400,14400,'V32B'],v23:[75,1200,'V23C'],bell212:[1200,1200,'B212'],91:[52000,52000,'V91']}[mode];
  if(extra){assert.deepEqual([result.txRate,result.rxRate,result.modulation],extra);select(String(mode));setTestResult(result);drawStats(result.stats.verifiedAtSeconds);assert.equal(document.getElementById('statRates').textContent,`${extra[0].toLocaleString()} / ${extra[1].toLocaleString()} bit/s`);}
  if(mode==='v23')assert.equal(document.getElementById('statRateLabel').textContent,'Caller upload / download');
  if(mode===34){assert.equal(result.txRate,33600);assert.equal(result.rxRate,33600);}
  runner.terminate();console.log(`PASS: packaged file:// ${['x2','flex'].includes(mode)?mode+' downstream PCM':result.modulation.replace(/^V/,'V.')}, no Worker, verified ${['x2','flex'].includes(mode)?'downstream':'bidirectional'} payload`);
 }
 for(const [id,value] of Object.entries({reliabilityScope:'all',reliabilityRepeats:'1',reliabilityDuration:'30',reliabilityV92:'quick'}))document.getElementById(id).value=value;
 select('34');document.getElementById('lineNoise').value='30';updateReliabilityControls();
 assert.equal(document.getElementById('reliabilityStart').disabled,true);
 document.getElementById('reliabilityScope').value='current';updateReliabilityControls();assert.equal(document.getElementById('reliabilityStart').disabled,false);
 document.getElementById('lineNoise').value='0';
 const reliabilityRunner=createOfflineModemRunner();let completed=0;
 const reliability=await new Promise((resolve,reject)=>{
  reliabilityRunner.onmessage=({data})=>{
   if(data.type==='error')reject(new Error(data.message));
   if(data.type==='reliability-progress'){assert.equal(data.summary.calls.length,data.summary.completedCalls);completed=Math.max(completed,data.summary.completedCalls);}
   if(data.type==='reliability-result')resolve(data);
  };
  reliabilityRunner.postMessage({task:'reliability',settings:{repeats:1,durationSeconds:30}});
 });
 assert.equal(completed,3);assert.equal(reliability.calls.filter(c=>c.passed).length,3);renderReliability(reliability);
 for(const mode of [34,90,92]){assert.equal(document.getElementById('reliability'+mode+'Passed').textContent,'1/1');assert.equal(document.getElementById('reliability'+mode+'Mismatches').textContent,'0');}
 assert.equal(document.getElementById('reliabilityExport').disabled,false);assert.equal(document.getElementById('reliabilityFailures').hidden,true);
 document.getElementById('reliabilityExport').listeners.click();assert.equal(download.download,'modem-reliability-results.json');
 const exported=await (await fetch(download.href)).json();assert.equal(exported.completedCalls,3);assert.deepEqual(exported.calls,reliability.calls);URL.revokeObjectURL(download.href);
 stop();assert.equal(document.getElementById('reliability34Passed').textContent,'1/1');
 reliabilityRunner.terminate();
 select('34');document.getElementById('reliabilityScope').value='current';document.getElementById('reliabilityRepeats').value='2';
 await startReliability();
 const deadline=Date.now()+30000;
 while(!getTestReliability()?.completedCalls&&Date.now()<deadline)await new Promise(resolve=>setTimeout(resolve,20));
 assert.equal(getTestReliability()?.completedCalls,1);stop();
 assert.equal(getTestReliability().status,'stopped');assert.equal(getTestReliability().activeCallUngraded,true);assert.equal(getTestReliability().calls.length,1);
 assert.equal(document.getElementById('status').textContent,'Reliability test stopped');assert.equal(document.getElementById('reliability34Completed').textContent,'1/2');
 document.getElementById('reliabilityExport').listeners.click();
 const stoppedExport=await (await fetch(download.href)).json();assert.equal(stoppedExport.status,'stopped');assert.equal(stoppedExport.calls.length,1);URL.revokeObjectURL(download.href);
 console.log('PASS: packaged reliability dispatch, three sustained calls, completed-only results, rendered table, JSON exports, stop during the next call and channel gating');
 let cancelled=false;
 await assert.rejects(runPair(32,'cancel',()=>{cancelled=true;},()=>cancelled),/Call cancelled/);
 console.log('PASS: cooperative cancellation');
}finally{await fs.rm(folder,{recursive:true,force:true});}
