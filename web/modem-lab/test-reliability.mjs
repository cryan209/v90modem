import assert from 'node:assert/strict';
import {runReliability} from './dist/engine-pair.mjs';

for(const settings of [{modes:[32]},{modes:[90,90]},{repeats:0},{durationSeconds:45},{startup:{v8:false}},{startup:{line:{noiseDb:30}}}]){
 let started=false;
 await assert.rejects(runReliability(settings,()=>{started=true;}));assert.equal(started,false);
}
console.log('PASS: reject invalid runs and unfair cross-mode channel settings before starting');

const progress=[];
const result=await runReliability({repeats:2,durationSeconds:30},p=>{
 assert.equal(p.summary.calls.length,p.summary.completedCalls);
 assert.ok(p.summary.calls.every(c=>c.failureReason!==undefined));
 progress.push(p.summary.completedCalls);
});
assert.equal(result.status,'complete');assert.equal(result.totalCalls,6);assert.equal(result.completedCalls,6);
assert.deepEqual(result.calls.map(c=>c.mode),[34,90,92,90,92,34]);
for(const c of result.calls){
 assert.ok(c.passed,JSON.stringify(c));assert.ok(c.connected);assert.equal(c.disconnected,false);assert.equal(c.mismatchBytes,0);
 assert.equal(c.trafficSeconds,30);assert.ok(c.simulationSeconds>=c.connectSeconds+30);
 assert.equal(c.verifiedBytes,c.sentBytes);assert.equal(c.receivedBytes,c.sentBytes);
 assert.equal(c.callerSentBytes,c.answererReceivedBytes);assert.equal(c.answererSentBytes,c.callerReceivedBytes);
 assert.ok(c.callerReceivedBytes>50000);assert.ok(c.answererReceivedBytes>50000);
 assert.equal(c.usefulBitsPerSecond,c.bytesDuringTraffic*8/30);
 assert.equal(c.clippedSamples,0);
 if(c.mode===90){assert.equal(c.txRate,31200);assert.equal(c.rxRate,56000);}
 if(c.mode===92){assert.equal(c.txRate,48000);assert.equal(c.rxRate,56000);}
}
for(const row of result.rows){assert.equal(row.completed,2);assert.equal(row.connected,2);assert.equal(row.passed,2);assert.equal(row.failed,0);assert.equal(row.disconnects,0);}
assert.ok(progress.includes(0)&&progress.includes(6));
console.log('PASS: six fresh calls, full 30-second bidirectional traffic, independent byte checks, drain accounting and comparison totals');

const long=await runReliability({modes:[92],repeats:1,durationSeconds:60,startup:{v92Startup:'full',line:{delayMs:80}}});
const call=long.calls[0];assert.ok(call.passed,JSON.stringify(call));assert.equal(call.trafficSeconds,60);
assert.ok(call.verifiedBytes>600000);assert.equal(call.callerSentBytes,call.answererReceivedBytes);
console.log('PASS: sustained 60-second V.92 full-startup transfer at 80-ms delay exceeds 600000 verified bytes');

const noisy=await runReliability({modes:[34],repeats:1,durationSeconds:30,startup:{line:{noiseDb:30}}});
assert.equal(noisy.completedCalls,1);assert.equal(noisy.calls[0].requestedTrafficSeconds,30);
if(noisy.calls[0].passed){assert.equal(noisy.calls[0].sentBytes,noisy.calls[0].receivedBytes);assert.equal(noisy.calls[0].mismatchBytes,0);}
console.log(`PASS: noisy V.34 call finishes and reports its actual outcome (passed=${noisy.calls[0].passed}, retrains=${noisy.calls[0].retrainRequests})`);

const failed=await runReliability({modes:[34],repeats:1,durationSeconds:30,startup:{line:{noiseDb:16,lossDb:24}}});
assert.equal(failed.completedCalls,1);assert.equal(failed.rows[0].passed,0);assert.equal(failed.rows[0].connected,0);
assert.equal(failed.calls[0].verifiedBytes,0);assert.equal(failed.calls[0].failureReason,'No connection');
assert.equal(failed.calls[0].usefulBitsPerSecond,null);
console.log('PASS: failed training is a completed failure with no invented throughput or verification');

let cancel=false,last;
await assert.rejects(runReliability({repeats:2},p=>{last=p.summary;if(last.completedCalls===1)cancel=true;},()=>cancel),/Call cancelled/);
assert.equal(last.completedCalls,1);assert.equal(last.calls.length,1);
console.log('PASS: cancellation retains the finished call and does not grade the next one');
