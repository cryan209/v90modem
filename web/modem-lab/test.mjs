import assert from 'node:assert/strict';
import {transmit,receive,wav} from './dist/fsk.mjs';
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
 assert.ok(result.constellations.some(f=>f.points.length>0),'No measured QAM symbols');
 if(mode===32){assert.equal(result.txRate,9600);assert.equal(result.rxRate,9600);}
 else{assert.equal(result.txRate,31200);assert.equal(result.rxRate,56000);}
 console.log(`PASS: V.${mode}, ${new TextEncoder().encode(text).length} bytes, both directions, ${result.left.length/8000}s, measured constellation`);
}
