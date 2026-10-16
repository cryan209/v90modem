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
 if(mode===34){assert.ok(r.txRate>0&&r.txRate<=33600);assert.ok(r.protocol.some(e=>/selected V34|mod=V34|modulation=V34/.test(e.line)),'Did not actually select V.34');}
 if(startup.v8bis)for(const side of ['caller','answerer'])assert.ok(r.protocol.some(e=>e.side===side&&/V\.8bis complete/.test(e.line)),'V.8bis did not complete');
 if(startup.v8===false){assert.ok(!r.protocol.some(e=>/[<>](CM|JM):/.test(e.line)));assert.ok(r.events.some(e=>e.message?.includes('V.25')));}
 console.log(`PASS: V.${mode}, startup ${JSON.stringify(startup)}, payload, LAPM, rates ${r.txRate}/${r.rxRate}`);
}
await assert.rejects(runPair(90,'bad',()=>{},()=>false,{v8:false,v8bis:false}),/require V.8/);
