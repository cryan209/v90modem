import createModem from './dist/modem-engine.mjs';
import assert from 'node:assert/strict';
import {transmit,receive,wav} from './dist/fsk.mjs';
import {hdlcFrames,inspectorText,negotiationRecords} from './dist/negotiation.mjs';
import {runPair} from './dist/engine-pair.mjs';
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
