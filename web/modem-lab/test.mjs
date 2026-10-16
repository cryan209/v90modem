import createModem from './dist/modem-engine.mjs';
import assert from 'node:assert/strict';
import {transmit,receive,wav} from './dist/fsk.mjs';
import {hdlcFrames,inspectorText,negotiationRecords} from './dist/negotiation.mjs';
import {runPair,validateLine,delayLine,checkPcmRates} from './dist/engine-pair.mjs';
const examples=['A','\0\0\0',String.fromCharCode(...Array.from({length:95},(_,i)=>i+32)),'Kia ora — hello 🌏','x'.repeat(240)];
for(const profile of ['v21','bell103'])for(const text of examples)for(const snr of [99,24,12]){
 const tx=transmit(text,profile,snr),rx=receive(tx.samples,profile);
 assert.equal(rx.text,text,`${profile} at ${snr} dB`);
 // Fractional-symbol prefix: acquisition must not know the TX sample clock.
 const shifted=new Float32Array(tx.samples.length+13);shifted.set(tx.samples,13);
 assert.equal(receive(shifted,profile).text,text,`${profile} offset acquisition`);
}
const sample=transmit('WAV').samples,header=new DataView(wav(sample));
assert.equal(header.getUint32(24,true),8000);assert.equal(header.getUint32(40,true),sample.length*2);
const stereo=new DataView(wav(new Float32Array([.5,-.5]),new Float32Array([-.25,.25])));
assert.equal(stereo.getUint16(22,true),2);assert.equal(stereo.getUint32(40,true),8);
assert.equal(stereo.getInt16(44,true),16384);assert.equal(stereo.getInt16(46,true),-8192);
console.log('PASS: FSK sample-only decoding, both tone pairs, clean/noisy, offset acquisition, UTF-8, 240 bytes, WAV header');
for(const mode of [32,90])for(const text of ['Hello from the browser!','Kia ora — hello 🌏','0123456789'.repeat(24)]){
 const result=await runPair(mode,text);
 assert.ok(result.passed,`Mode ${mode} failed training or payload`);
 assert.equal(result.received,text);assert.equal(result.reply,'ACK: '+text);
 assert.equal(result.g711.callerTx.length,result.left.length*2);
 assert.equal(result.g711.answererTx.length,result.right.length*2);
 assert.equal(result.droppedProtocol,0,'Protocol log capture was truncated');
 const frames=result.negotiation.frames;
 for(const name of ['XID','SABME','UA'])assert.ok(frames.some(f=>f.valid&&f.type===name),`Missing CRC-valid ${name}`);
 const xid=frames.find(f=>f.valid&&f.type==='XID'&&f.side==='caller'&&f.direction==='TX');
 assert.ok(xid.fields.some(s=>s==='V.42bis codewords P1: 1024'));
 const original=result.wireBits.callerTx,index=xid.wireStart+16;
 const corrupt=original.slice(0,index)+(original[index]==='1'?'0':'1')+original.slice(index+1);
 assert.ok(!hdlcFrames(corrupt,result.bitClock,'callerTx').some(f=>f.wireStart===xid.wireStart&&f.valid),'Corrupt HDLC frame accepted');
 assert.ok(!inspectorText(result,0,'simple').includes('XID · FCS valid'),'Future events leaked into timeline');
 assert.ok(inspectorText(result,60,'raw').includes('captured bits so far'));
 if(mode===90){const simple=inspectorText(result,60);for(const field of ['INFO0d','INFO0a','INFO1a','parsed Ja DIL descriptor','sending DIL'])assert.ok(simple.includes(field),`Missing ${field}`);}
 assert.ok(result.constellations.some(f=>f.points.length>0),'No measured QAM symbols');
 if(mode===32){assert.equal(result.txRate,9600);assert.equal(result.rxRate,9600);}
 else{assert.equal(result.txRate,31200);assert.equal(result.rxRate,56000);}
 console.log(`PASS: V.${mode}, ${new TextEncoder().encode(text).length} bytes, both directions, ${result.left.length/8000}s, measured constellation`);
}

for(const [mode,startup] of [[34,{v8:true,v8bis:false}],[34,{v8:true,v8bis:true}],[32,{v8:false,v8bis:false}],[32,{v8:true,v8bis:true}],[90,{v8:true,v8bis:true}]]){
 const r=await runPair(mode,'Startup test — '+('0123456789'.repeat(22)),()=>{},()=>false,startup);
 assert.ok(r.passed,`V.${mode} startup failed`);assert.equal(r.droppedProtocol,0);
 assert.ok(r.negotiation.frames.some(f=>f.valid&&f.type==='XID'));
 if(mode===34){assert.equal(r.txRate,33600);assert.equal(r.rxRate,33600);assert.ok(r.protocol.some(e=>/selected V34|mod=V34|modulation=V34/.test(e.line)),'Did not actually select V.34');}
 if(startup.v8bis)for(const side of ['caller','answerer'])assert.ok(r.protocol.some(e=>e.side===side&&/V\.8bis complete/.test(e.line)),'V.8bis did not complete');
 if(startup.v8===false){assert.ok(!r.protocol.some(e=>/[<>](CM|JM):/.test(e.line)));assert.ok(r.events.some(e=>e.message?.includes('V.25')));}
 console.log(`PASS: V.${mode}, startup ${JSON.stringify(startup)}, payload, LAPM, rates ${r.txRate}/${r.rxRate}`);
}
await assert.rejects(runPair(90,'bad',()=>{},()=>false,{v8:false,v8bis:false}),/require V.8/);

for(const v8bis of [false,true]){
 const r=await runPair(34,'Stable V.34 — '+('0123456789'.repeat(22)),()=>{},()=>false,{v8:true,v8bis,v34MaxRate:24000});
 assert.ok(r.passed);assert.equal(r.txRate,24000);assert.equal(r.rxRate,24000);
 assert.ok(!r.protocol.some(e=>/V\.34.*(?:asking for .*retraining|training failed|restarting Phase 2)/.test(e.line)),'Stable V.34 retrained');
 console.log(`PASS: V.34 24000 ceiling, V.8bis=${v8bis}, long payload, no retrains`);
}

// Varied sustained traffic catches errors hidden by short/repetitive messages.
let payloadSeed=1234;
const sustained=Array.from({length:12000},()=>{payloadSeed=(Math.imul(payloadSeed,1664525)+1013904223)>>>0;return String.fromCharCode(32+payloadSeed%95);}).join('');
for(const v8bis of [false,true])for(const ceiling of [24000,0]){
 const r=await runPair(34,sustained,()=>{},()=>false,{v8:true,v8bis,v34MaxRate:ceiling});
 assert.ok(r.passed);assert.equal(r.txRate,ceiling||33600);assert.equal(r.rxRate,ceiling||33600);
 assert.ok(!r.protocol.some(e=>/V\.34.*(?:asking for .*retraining|training failed|restarting Phase 2)/.test(e.line)));
 assert.equal(r.negotiation.frames.filter(f=>!f.valid).length,0);
 console.log(`PASS: V.34 sustained 12000-byte transfer, V.8bis=${v8bis}, ceiling=${ceiling||'automatic'}, ${r.txRate}/${r.rxRate}, no retrains or invalid HDLC`);
}

// Proprietary PCM demonstrations must really decode, without claiming startup.
for(const mode of ['x2','flex'])for(const text of [...examples,sustained]){
 const r=await runPair(mode,text);
 assert.equal(r.kind,'pcm');assert.ok(r.passed);assert.equal(r.received,text);
 assert.equal(r.reply,'');assert.equal(r.txRate,56000);assert.equal(r.rxRate,0);
 assert.equal(r.pcm.frameSamples,mode==='x2'?6:8);
 assert.equal(r.pcm.frameBits,mode==='x2'?42:56);
 assert.equal(r.g711.serverTx.length,r.pcm.samples*2);
 assert.equal(r.negotiation.frames.length,0);assert.equal(r.protocol.length,0);
 assert.ok(!r.events.some(e=>/Connected|V\.8|LAPM/.test(e.message||'')));
 assert.match(inspectorText(r,60),/no analogue equalizer/);
 assert.ok(inspectorText(r,0,'raw').includes('(no codewords yet)'));
 assert.ok(r.left.some(x=>x!==0));assert.ok(r.right.every(x=>x===0));
 console.log(`PASS: ${mode} downstream PCM, ${new TextEncoder().encode(text).length} bytes, independent decode, no fabricated connection`);
}
for(const mode of ['x2','flex']){
 let cancelled=false;
 await assert.rejects(runPair(mode,'cancel',()=>{cancelled=true;},()=>cancelled),/Call cancelled/);
}

// A damaged DS0 must affect the independent decoder, rather than returning
// text from the transmitter or an out-of-band application copy.
for(const [mode,id] of [['x2',2],['flex',56]]){
 const r=await runPair(mode,'Independent PCM receiver test');
 const wire=Uint8Array.from(r.g711.serverTx.match(/../g),h=>parseInt(h,16));
 wire.fill(0,0,r.pcm.frameSamples);
 const m=await createModem({print:()=>{},printErr:()=>{}}),input=m._malloc(wire.length),output=m._malloc(256);
 try{
  m.HEAPU8.set(wire,input);const n=m._browser_pcm_decode(id,input,wire.length,output,256);
  const data=n>0?m.HEAPU8.slice(output,output+n):new Uint8Array();
  const size=data[0]|(data[1]<<8);
  const decoded=new TextDecoder().decode(data.slice(2,2+size));
  assert.ok(n<0||decoded!=='Independent PCM receiver test');
  assert.equal(m._browser_pcm_encode(999,input,1,output,256),-1);
  assert.equal(m._browser_pcm_decode(id,input,wire.length-1,output,256),-1);
  console.log(`PASS: ${mode} detects damaged codewords and rejects invalid frames`);
 }finally{m._free(input);m._free(output);}
}

// Preserve the DS0 exactly while adding propagation across callback boundaries.
const delayed=delayLine(23),sourceCodewords=Uint8Array.from({length:480},(_,i)=>i%256),receivedCodewords=[];
for(let off=0;off<480;off+=160){const block=delayed(sourceCodewords.slice(off,off+160));assert.equal(block.length,160);receivedCodewords.push(...block);}
assert.deepEqual(receivedCodewords,[...new Uint8Array(23).fill(255),...sourceCodewords.slice(0,457)]);
for(const bad of [{noiseDb:NaN},{lossDb:-1},{clockPpm:101},{delayMs:1.5},{seed:-1}])assert.throws(()=>validateLine(34,bad));
assert.throws(()=>validateLine(92,{noiseDb:30}),/analogue modem modes only/);
assert.throws(()=>validateLine('x2',{delayMs:20}),/fixed byte-exact/);

const statText='Measured transfer — 0123456789';
for(const [mode,line] of [[34,{}],[34,{noiseDb:30}],[34,{lossDb:6}],[34,{clockPpm:10}],[90,{delayMs:20}],[92,{delayMs:20}]]){
 const r=await runPair(mode,statText,()=>{},()=>false,{v8:true,line});
 assert.ok(r.passed,`V.${mode} impaired line ${JSON.stringify(line)}`);
 assert.equal(r.received,statText);assert.equal(r.reply,'ACK: '+statText);
 const s=r.stats,n=new TextEncoder().encode(statText).length;
 assert.equal(s.verifiedBytes,n*2+5);assert.equal(s.callerBytes,n);assert.equal(s.answererBytes,n+5);
 assert.ok(s.connectSeconds>0);assert.ok(s.transferSeconds>0);
 assert.ok(Math.abs(s.usefulBitsPerSecond-s.verifiedBytes*8/s.transferSeconds)<1e-6);
 assert.equal(s.clippedSamples,0);
 assert.equal(r.waveform,line.noiseDb||line.lossDb||line.clockPpm?'received':'transmitted');
 if(line.noiseDb===30){assert.ok(r.txRate<33600);assert.ok(r.rxRate<33600);}
 if(mode===92){assert.equal(r.modulation,'V92');assert.equal(r.txRate,48000);assert.equal(r.rxRate,56000);}
 console.log(`PASS: V.${mode}, line ${JSON.stringify(line)}, verified payload, measured rates ${r.txRate}/${r.rxRate} and statistics`);
}
for(const v8bis of [false,true]){
 const r=await runPair(92,sustained,()=>{},()=>false,{v8:true,v8bis,v92Startup:'compare'});
 assert.ok(r.passed);assert.equal(r.modulation,'V92');assert.equal(r.txRate,48000);assert.equal(r.rxRate,56000);
 assert.ok(r.comparison.full.passed);assert.ok(r.comparison.quick.passed);
 assert.ok(r.comparison.savedSeconds>0);
 assert.ok(r.protocol.some(e=>/short Phase 1 complete/.test(e.line)));
 assert.ok(r.protocol.some(e=>/short Phase 2 \(digital\)/.test(e.line)));
 assert.ok(r.protocol.some(e=>/short Phase 2 \(analogue\)/.test(e.line)));
 assert.ok(!r.protocol.some(e=>/short Phase 2 abandoned/.test(e.line)));
 assert.equal(r.stats.invalidHdlcFrames,0);
 console.log(`PASS: V.92 full/Quick Connect comparison, V.8bis=${v8bis}, 12000-byte bidirectional payload, saving ${r.comparison.savedSeconds.toFixed(2)} s`);
}

const failedLine=await runPair(34,'Severe line test',()=>{},()=>false,{v8:true,line:{noiseDb:16,lossDb:24}});
assert.equal(failedLine.passed,false);assert.equal(failedLine.stats.verifiedBytes,0);
assert.equal(failedLine.stats.connectSeconds,null);assert.equal(failedLine.stats.usefulBitsPerSecond,null);
console.log('PASS: failed channel makes no connection, verified-byte or throughput claim');

assert.throws(()=>checkPcmRates('V90',{tx:56000,rx:56000},{tx:56000,rx:56000}),/inconsistent/);
assert.throws(()=>checkPcmRates('V90',{tx:0,rx:56000},{tx:56000,rx:31200}),/inconsistent/);
assert.throws(()=>checkPcmRates('V90',{tx:31200,rx:56000},{tx:56000,rx:28800}),/inconsistent/);
checkPcmRates('V90',{tx:31200,rx:56000},{tx:56000,rx:31200});
for(const v8bis of [false,true]){
 const r=await runPair(90,'V90 delayed report',()=>{},()=>false,{v8:true,v8bis,line:{delayMs:80}});
 assert.ok(r.passed);assert.equal(r.txRate,31200);assert.equal(r.rxRate,56000);
 const connections=r.events.filter(e=>e.rates);assert.ok(connections.length);
 for(const e of connections){
  assert.equal(e.rates.caller.tx,31200);assert.equal(e.rates.caller.rx,56000);
  assert.equal(e.rates.answerer.tx,56000);assert.equal(e.rates.answerer.rx,31200);
  assert.match(e.message,/upload 31200 \/ download 56000/);
 }
 console.log(`PASS: V.90 at 80-ms delay, V.8bis=${v8bis}, event and endpoint reports agree at 31200/56000`);
}


// Real engine paths: UTF-8, the full UI payload limit, and legacy startup.
const addedModes=[['v22',1200,1200,'V22'],['v22bis',2400,2400,'V22B'],['v32bis',14400,14400,'V32B'],['v23',75,1200,'V23C'],['bell212',1200,1200,'B212'],[91,52000,52000,'V91']];
for(const [mode,tx,rx,carrier] of addedModes){
 for(const text of ['Kia ora — hello 🌏',String.fromCharCode(...Array.from({length:240},(_,i)=>33+(i*37)%90))]){
  const legacy=['v23','bell212'].includes(mode);
  const r=await runPair(mode,text,()=>{},()=>false,{v8:!legacy,v8bis:false});
  assert.ok(r.passed,`${mode} failed: ${r.logs.slice(-10).join('\n')}`);assert.equal(r.received,text);assert.equal(r.reply,'ACK: '+text);
  assert.equal(r.modulation,carrier);assert.equal(r.txRate,tx);assert.equal(r.rxRate,rx);assert.equal(r.droppedProtocol,0);
  const connects=r.events.filter(e=>e.rates);assert.ok(connects.length);
  for(const e of connects){assert.deepEqual(e.rates.caller,{tx,rx});assert.deepEqual(e.rates.answerer,{tx:rx,rx:tx});}
  if(mode==='v23'){assert.equal(r.stats.invalidHdlcFrames,null);assert.ok(!r.logs.some(l=>l.includes('V.42 LAPM connected')));}
  else assert.ok(r.negotiation.frames.some(f=>f.valid&&f.type==='SABME'));
 }
 console.log(`PASS: ${carrier} real-engine UTF-8 and 240-byte payloads, both directions, negotiated ${tx}/${rx}`);
}
for(const mode of ['v22','v22bis','v32bis']){
 const r=await runPair(mode,'Legacy startup',()=>{},()=>false,{v8:false,v8bis:false});
 assert.ok(r.passed,`${mode} legacy startup failed`);
 console.log(`PASS: ${mode} V.25 startup without V.8`);
}
for(const mode of ['v23','bell212'])await assert.rejects(runPair(mode,'invalid',()=>{},()=>false,{v8:true}),/legacy startup/);
assert.throws(()=>checkPcmRates('V23C',{tx:1200,rx:1200},{tx:1200,rx:1200}),/inconsistent/);
assert.throws(()=>checkPcmRates('V91',{tx:65000,rx:65000},{tx:65000,rx:65000}),/inconsistent/);
console.log('PASS: legacy startup gating and asymmetric/digital directional rate validation');
