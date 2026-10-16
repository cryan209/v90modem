import createModem from './modem-engine.mjs';
import {negotiationRecords} from './negotiation.mjs';
export function expand(code){const u=(~code)&255;const t=(((u&15)<<3)+132)<<((u>>4)&7);return ((u&128)?132-t:t-132)/32768;}
export async function runPair(mode,text,onProgress=()=>{},shouldStop=()=>false,startup={v8:true,v8bis:false}){
  const line=validateLine(mode,startup.line);
  if(Number(mode)===92 && startup.v92Startup==='compare'){
    const full=await runPair(92,text,p=>onProgress({...p,state:'Full startup · '+p.state}),shouldStop,{...startup,v92Startup:'full'});
    const quick=await runPair(92,text,p=>onProgress({...p,state:'Quick Connect · '+p.state}),shouldStop,{...startup,v92Startup:'quick'});
    quick.comparison={full:{...full.stats,passed:full.passed,txRate:full.txRate,rxRate:full.rxRate},quick:{...quick.stats,passed:quick.passed,txRate:quick.txRate,rxRate:quick.rxRate},
      savedSeconds:full.passed&&quick.passed?full.stats.connectSeconds-quick.stats.connectSeconds:null};
    return quick;
  }

  if(['x2','flex'].includes(String(mode)))return runPcmDemo(String(mode),text,onProgress,shouldStop);
  if(![32,34,90,92].includes(Number(mode)))throw new Error('Unsupported modem mode');
  if(startup.v8===false&&Number(mode)!==32)throw new Error('V.34, V.90 and V.92 require V.8 in this demo');
  if(startup.v8bis&&startup.v8===false)throw new Error('V.8bis requires V.8 in this demo');
  const startupFlags=(startup.v8===false?0:1)|(startup.v8bis?2:0)|(startup.v92Startup==='quick'?4:0);
  const logs=[],protocol=[],bitClock=[];let simTime=0,droppedProtocol=0;
  const options=(side)=>({print:line=>capture(side,line),printErr:line=>capture(side,line)});
  function capture(side,line){if(protocol.length<100000){protocol.push({time:simTime,side,line});logs.push(line);}else droppedProtocol++;}
  const a=await createModem(options('caller')),b=await createModem(options('answerer'));
  if(a.HEAPU8.buffer===b.HEAPU8.buffer)throw new Error('Modem instances share memory');
  const frame=value=>{const body=new TextEncoder().encode(value),out=new Uint8Array(body.length+2);out[0]=body.length&255;out[1]=body.length>>8;out.set(body,2);return out;};
  const unframe=(m)=>{const n=m._browser_received_n(),data=m.HEAPU8.slice(m._browser_received(),m._browser_received()+n);if(n<2)return {text:'',complete:false};const size=data[0]|(data[1]<<8);if(size>65533)return {text:'',complete:false};return {text:new TextDecoder().decode(data.slice(2,Math.min(n,size+2))),complete:n>=size+2};};
  const bytes=frame(text);if(bytes.length>60002)throw new Error('Message exceeds 60000 UTF-8 bytes');let replies=null;
  const pa=a._malloc(bytes.length);let pb=0;
  a.HEAPU8.set(bytes,pa);
  a._browser_init(Number(mode),1,startupFlags,startup.v34MaxRate||0);b._browser_init(Number(mode),0,startupFlags,startup.v34MaxRate||0);
  for(const [m,seed] of [[a,line.seed],[b,(line.seed+1)>>>0]]){
    if(m._browser_line_config(line.noiseDb,line.lossDb,line.clockPpm,seed)!==0)throw new Error('Unsupported analogue line settings');
  }
  const toCaller=delayLine(line.delayMs*8),toAnswerer=delayLine(line.delayMs*8);
  const analogueActive=[32,34].includes(Number(mode))&&(line.noiseDb||line.lossDb||line.clockPpm);
  const ai=a._browser_input(),bi=b._browser_input(),ao=a._browser_output(),bo=b._browser_output();
  a.HEAPU8.fill(255,ai,ai+160);b.HEAPU8.fill(255,bi,bi+160);
  const callerCodewords=new Uint8Array(480000),answererCodewords=new Uint8Array(480000);
  const left=[],right=[],events=[],constellations=[];let at=0,bt=0,doneAt=0,connectedAt=null,lastState='',lastRx='',lastReply='';
  try{
    for(let block=0;block<3000;block++){
      if(shouldStop())throw new Error('Call cancelled');
      simTime=(block+1)*.02;
      const sa=a._browser_step(),sb=b._browser_step();
      const x=a.HEAPU8.slice(ao,ao+160),y=b.HEAPU8.slice(bo,bo+160);
      a.HEAPU8.set(toCaller(y),ai);b.HEAPU8.set(toAnswerer(x),bi);
      callerCodewords.set(x,block*160);answererCodewords.set(y,block*160);
      if(analogueActive){
        left.push(Float32Array.from(new Int16Array(b.HEAPU8.buffer,b._browser_line_samples(),160),v=>v/32768));
        right.push(Float32Array.from(new Int16Array(a.HEAPU8.buffer,a._browser_line_samples(),160),v=>v/32768));
      }else{left.push(Float32Array.from(x,expand));right.push(Float32Array.from(y,expand));}
      if(block%10===0){const raw=b.UTF8ToString(b._browser_monitor());if(raw){const monitor=JSON.parse('{'+raw+'}');if(monitor.iq[0].length)constellations.push({time:(block+1)*.02,points:monitor.iq[0]});}}
      bitClock.push({time:simTime,callerTx:a._browser_bit_count(1),callerRx:a._browser_bit_count(0),answererTx:b._browser_bit_count(1),answererRx:b._browser_bit_count(0)});
      const time=(block+1)*.02, ar=a._browser_rate(),br=b._browser_rate();
      const startupName=side=>startup.v8===false?'V.25':startup.v8bis&&!protocol.some(e=>e.side===side&&/V\.8bis complete/.test(e.line))?'V.8bis':'V.8';
      const states=side=>['idle','dialing',startupName(side),'training','data','hangup'];
      const callerRates={tx:a._browser_tx_rate(),rx:a._browser_rx_rate()},answererRates={tx:b._browser_tx_rate(),rx:b._browser_rx_rate()};
      const carrier=a.UTF8ToString(a._browser_modulation());
      if(ar&&br)checkPcmRates(carrier,callerRates,answererRates);
      const state=`${sa}/${sb}/${ar}/${br}/${callerRates.tx}/${callerRates.rx}/${answererRates.tx}/${answererRates.rx}/${startupName('caller')}/${startupName('answerer')}`;
      if(state!==lastState){events.push({time,...(ar&&br?{rates:{caller:callerRates,answerer:answererRates},carrier}:{}),message:ar&&br?(['V90','V92'].includes(carrier)?`Connected · ${carrier.replace('V','V.')} · caller upload ${callerRates.tx} / download ${callerRates.rx} bit/s`:`Connected · caller TX ${callerRates.tx} / RX ${callerRates.rx} bit/s`):`Training · caller ${states('caller')[sa]} / answerer ${states('answerer')[sb]}`});lastState=state;}
      if(ar && br){
        if(connectedAt===null)connectedAt=time;
        if(at<bytes.length)at+=a._browser_send(pa+at,bytes.length-at);
        if(replies && bt<replies.length)bt+=b._browser_send(pb+bt,replies.length-bt);
      }
      const remote=unframe(b),local=unframe(a),rx=remote.text,reply=local.text;
      if(remote.complete && !replies){replies=frame('ACK: '+rx);pb=b._malloc(replies.length);b.HEAPU8.set(replies,pb);}
      if(rx!==lastRx||reply!==lastReply){events.push({time,received:rx,reply});lastRx=rx;lastReply=reply;}
      if(rx===text && reply==='ACK: '+text && at===bytes.length && replies && bt===replies.length && !doneAt){doneAt=time;events.push({time,message:'Payload verified in both directions'});}
      if(doneAt && time>doneAt+1)break;
      if(block%50===0){onProgress({time,state:events.at(-1)?.message||'Training'});await new Promise(resolve=>setTimeout(resolve,0));}
    }
    const length=left.length*160,l=new Float32Array(length),r=new Float32Array(length);
    left.forEach((x,i)=>l.set(x,i*160));right.forEach((x,i)=>r.set(x,i*160));
    const read=(m,dir)=>{const ptr=m._browser_read_bits(dir);if(!ptr)throw new Error('Bit capture allocation failed');const bits=m.UTF8ToString(ptr);m._free(ptr);return bits;};
    const wireBits={callerTx:read(a,1),callerRx:read(a,0),answererTx:read(b,1),answererRx:read(b,0)};
    const hex=bytes=>Array.from(bytes,b=>b.toString(16).padStart(2,'0')).join('');
    const g711={callerTx:hex(callerCodewords.subarray(0,length)),answererTx:hex(answererCodewords.subarray(0,length))};
    const negotiation=negotiationRecords({protocol,bitClock,wireBits});
    const payloadBytes=bytes.length-2,replyBytes=new TextEncoder().encode('ACK: '+text).length;
    const stats={connectSeconds:connectedAt,verifiedAtSeconds:doneAt||null,transferSeconds:doneAt?doneAt-connectedAt:null,
      verifiedBytes:doneAt?payloadBytes+replyBytes:0,callerBytes:doneAt?payloadBytes:0,answererBytes:doneAt?replyBytes:0,
      usefulBitsPerSecond:doneAt&&doneAt>connectedAt?(payloadBytes+replyBytes)*8/(doneAt-connectedAt):null,
      retrainRequests:protocol.filter(e=>/\[ME\].*(?:asking for.*retrain|retrain requested|requesting.*retrain)/i.test(e.line)).length,
      invalidHdlcFrames:negotiation.frames.filter(f=>!f.valid).length,
      clippedSamples:a._browser_line_clipped()+b._browser_line_clipped(),simulationSeconds:length/8000};
    return {stats,line,waveform:analogueActive?'received':'transmitted',modulation:a.UTF8ToString(a._browser_modulation()),left:l,right:r,events,startup,protocol,bitClock,wireBits,g711,negotiation,droppedProtocol,constellations,received:lastRx,reply:lastReply,passed:!!doneAt,logs,
      rate:a._browser_rate(),txRate:a._browser_tx_rate(),rxRate:a._browser_rx_rate()};
  }finally{a._free(pa);b._free(pb);a._browser_destroy();b._browser_destroy();}
}

// Two isolated instances: the receiver receives only DS0 codewords and a
// fixed public profile. This is a downstream payload demonstration, not startup.
async function runPcmDemo(mode,text,onProgress,shouldStop){
 const id=mode==='x2'?2:56,name=mode==='x2'?'x2':'K56flex';
 const bytes=new TextEncoder().encode(text);
 if(bytes.length>60000)throw new Error('Message exceeds 60000 UTF-8 bytes');
 const packet=new Uint8Array(bytes.length+2);packet[0]=bytes.length&255;packet[1]=bytes.length>>8;packet.set(bytes,2);
 onProgress({time:0,state:'Encoding downstream PCM payload'});
 await new Promise(resolve=>setTimeout(resolve,0));if(shouldStop())throw new Error('Call cancelled');
 const a=await createModem({print:()=>{},printErr:()=>{}}),b=await createModem({print:()=>{},printErr:()=>{}});
 if(a.HEAPU8.buffer===b.HEAPU8.buffer)throw new Error('PCM instances share memory');
 const capacity=Math.ceil(packet.length*8/42)*8+16;
 const src=a._malloc(packet.length),tx=a._malloc(capacity),rx=b._malloc(capacity),decoded=b._malloc(packet.length+16);
 try{
  a.HEAPU8.set(packet,src);const samples=a._browser_pcm_encode(id,src,packet.length,tx,capacity);
  if(samples<0)throw new Error(name+' encoder rejected the payload');
  const octets=a.HEAPU8.slice(tx,tx+samples);b.HEAPU8.set(octets,rx);
  const n=b._browser_pcm_decode(id,rx,samples,decoded,packet.length+16);
  if(n<2)throw new Error(name+' decoder rejected the received codewords');
  const data=b.HEAPU8.slice(decoded,decoded+n),size=data[0]|(data[1]<<8);
  if(size>n-2)throw new Error('Corrupt PCM message length');
  const received=new TextDecoder().decode(data.slice(2,2+size));
  const prefix=2400,length=prefix+samples+4000,left=new Float32Array(length),right=new Float32Array(length);
  left.set(Float32Array.from(octets,expand),prefix);
  const end=(prefix+samples)/8000,passed=received===text;
  return {kind:'pcm',mode,profile:mode==='x2'?'Short-record allocation 15; B=37, MD=5; GPC':'µ-law, 56000 bit/s, no impairment report',
   left,right,events:[{time:0,message:name+' downstream PCM · fixed profile · no dial-up negotiation'},
    {time:.3,message:'Encoding '+bytes.length+' UTF-8 bytes at 56,000 bit/s'},
    {time:end,received},{time:end,message:passed?'Payload verified':'Payload mismatch'}],
   received,reply:'',passed,rate:56000,txRate:56000,rxRate:0,constellations:[],protocol:[],droppedProtocol:0,
   negotiation:{records:[],frames:[]},wireBits:{},bitClock:[],g711:{serverTx:Array.from(octets,x=>x.toString(16).padStart(2,'0')).join('')},
   stats:{connectSeconds:null,verifiedAtSeconds:end,transferSeconds:samples/8000,verifiedBytes:passed?bytes.length:0,callerBytes:passed?bytes.length:0,answererBytes:0,
    usefulBitsPerSecond:passed?bytes.length*8/(samples/8000):null,retrainRequests:0,invalidHdlcFrames:null,clippedSamples:0,simulationSeconds:length/8000},
   pcm:{samples,frameSamples:mode==='x2'?6:8,frameBits:mode==='x2'?42:56,sourceBytes:bytes.length,decodedBytes:size},logs:[]};
 }finally{a._free(src);a._free(tx);b._free(rx);b._free(decoded);}
}

// Lab-only propagation: retain every codeword and every sample clock tick.
// Leading idle is the initial condition of the delayed line, not a stream edit.
export function delayLine(samples){
 const ring=new Uint8Array(samples);ring.fill(255);let position=0;
 return frame=>{
  if(!samples)return frame;
  const out=new Uint8Array(frame.length);
  for(let i=0;i<frame.length;i++){out[i]=ring[position];ring[position]=frame[i];position=(position+1)%samples;}
  return out;
 };
}
export function validateLine(mode,input={}){
 const line={noiseDb:0,lossDb:0,clockPpm:0,delayMs:0,seed:1234,...input};
 for(const key of ['noiseDb','lossDb','clockPpm','delayMs','seed'])if(!Number.isFinite(line[key]))throw new Error('Invalid line setting: '+key);
 if((line.noiseDb!==0&&(line.noiseDb<10||line.noiseDb>80))||line.lossDb<0||line.lossDb>30||Math.abs(line.clockPpm)>100
   ||!Number.isInteger(line.delayMs)||line.delayMs<0||line.delayMs>200||!Number.isInteger(line.seed)||line.seed<0||line.seed>4294967295)throw new Error('Line settings outside the supported range');
 if(![32,34].includes(Number(mode))&&(line.noiseDb||line.lossDb||line.clockPpm))throw new Error('Analogue noise, attenuation and clock drift are supported for V.32/V.34 only');
 if(['x2','flex'].includes(String(mode))&&line.delayMs)throw new Error('PCM payload demos use a fixed byte-exact line');
 return line;
}

// V.90 §§5/6 and V.250 6.4.3: report actual directional rates. Never
// repair an impossible report by clamping or copying the other direction.
export function checkPcmRates(carrier,caller,answerer){
 if(!['V90','V92'].includes(carrier))return;
 const values=[caller.tx,caller.rx,answerer.tx,answerer.rx];
 if(values.some(n=>!Number.isInteger(n)||n<=0)||caller.tx!==answerer.rx||caller.rx!==answerer.tx
    ||(carrier==='V90'&&(caller.tx>33600||caller.rx>56000)))
  throw new Error(`${carrier} directional rate report is inconsistent: caller TX/RX ${caller.tx}/${caller.rx}, answerer TX/RX ${answerer.tx}/${answerer.rx}`);
}

// Fresh calls with sustained, independently checked binary streams. Only
// completed calls are scored; audio and per-bit captures are not retained.
export async function runReliability(settings={},onProgress=()=>{},shouldStop=()=>false){
 const modes=settings.modes||[34,90,92],repeats=settings.repeats??3,durationSeconds=settings.durationSeconds??30;
 if(!Array.isArray(modes)||!modes.length||new Set(modes).size!==modes.length||modes.some(m=>![34,90,92].includes(m)))throw new Error('Reliability supports V.34, V.90 and V.92');
 if(!Number.isInteger(repeats)||repeats<1||repeats>5||![30,60].includes(durationSeconds))throw new Error('Choose 1–5 calls and 30 or 60 seconds of traffic');
 const startup={v8:true,v8bis:false,v34MaxRate:0,v92Startup:'quick',...settings.startup};
 if(startup.v8===false||!['full','quick'].includes(startup.v92Startup))throw new Error('Reliability requires V.8 and Full or Quick Connect startup');
 const line=validateLine(modes[0],startup.line);
 for(const mode of modes)validateLine(mode,line); // No silent removal of unsupported impairments.
 const calls=[],total=modes.length*repeats;
 const snapshot=()=>({kind:'reliability',status:'running',modes:[...modes],repeats,durationSeconds,startup:{...startup,line},totalCalls:total,completedCalls:calls.length,calls:[...calls],rows:reliabilityRows(modes,calls)});
 for(let attempt=0;attempt<repeats;attempt++)for(let offset=0;offset<modes.length;offset++){
  if(shouldStop())throw new Error('Call cancelled');
  const mode=modes[(offset+attempt)%modes.length];
  const update=p=>onProgress({...p,mode,attempt:attempt+1,callNumber:calls.length+1,totalCalls:total,summary:snapshot()});
  update({time:0,state:'Training'});
  const call=await reliabilityCall(mode,attempt,durationSeconds,startup,line,update,shouldStop);
  calls.push(call);onProgress({time:call.simulationSeconds,state:'Call completed',mode,attempt:attempt+1,callNumber:calls.length,totalCalls:total,summary:snapshot()});
 }
 return {...snapshot(),status:'complete'};
}
function reliabilityRows(modes,calls){
 return modes.map(mode=>{
  const group=calls.filter(c=>c.mode===mode),connected=group.filter(c=>c.connected),good=group.filter(c=>c.passed);
  return {mode,completed:group.length,connected:connected.length,passed:good.length,failed:group.length-good.length,
   verifiedBytes:group.reduce((n,c)=>n+c.verifiedBytes,0),mismatchBytes:group.reduce((n,c)=>n+c.mismatchBytes,0),
   retrainRequests:group.reduce((n,c)=>n+c.retrainRequests,0),disconnects:group.filter(c=>c.disconnected).length,
   meanConnectSeconds:connected.length?connected.reduce((n,c)=>n+c.connectSeconds,0)/connected.length:null,
   meanUsefulBitsPerSecond:good.length?good.reduce((n,c)=>n+c.usefulBitsPerSecond,0)/good.length:null,
   rates:[...new Set(connected.map(c=>`${c.txRate}/${c.rxRate}`))]};
 });
}
function reliabilityPattern(seed){
 let state=seed>>>0;
 return ()=>{state=(Math.imul(state,1664525)+1013904223)>>>0;return state>>>24;};
}
async function reliabilityCall(mode,attempt,duration,startup,line,onProgress,shouldStop){
 let retrainRequests=0;
 const capture=text=>{if(/\[ME\].*(?:asking for.*retrain|retrain requested|requesting.*retrain)/i.test(text))retrainRequests++;};
 const a=await createModem({print:capture,printErr:capture}),b=await createModem({print:capture,printErr:capture});
 let pa=0,pb=0,initializedA=false,initializedB=false,time=0,connectedAt=null,sendUntil=null,trafficBytes=0,trafficCaptured=false,disconnected=false,error=null,txRate=0,rxRate=0,clippedSamples=0,modulation='';
 const seed=(0x12345678+attempt)>>>0;
 const streams=[{tx:a,rx:b,next:reliabilityPattern(seed),expected:reliabilityPattern(seed)},
                {tx:b,rx:a,next:reliabilityPattern(seed^0x9e3779b9),expected:reliabilityPattern(seed^0x9e3779b9)}]
  .map(s=>({...s,buffer:new Uint8Array(4096),offset:4096,accepted:0,received:0,mismatches:0}));
 try{
  if(a.HEAPU8.buffer===b.HEAPU8.buffer)throw new Error('Reliability endpoints share memory');
  const flags=1|(startup.v8bis?2:0)|(startup.v92Startup==='quick'?4:0)|8;
  a._browser_init(mode,1,flags,startup.v34MaxRate||0);initializedA=true;
  b._browser_init(mode,0,flags,startup.v34MaxRate||0);initializedB=true;
  for(const [m,seedValue] of [[a,line.seed],[b,(line.seed+1)>>>0]])if(m._browser_line_config(line.noiseDb,line.lossDb,line.clockPpm,seedValue))throw new Error('Invalid reliability channel');
  pa=a._malloc(4096);pb=b._malloc(4096);streams[0].pointer=pa;streams[1].pointer=pb;
  const ai=a._browser_input(),bi=b._browser_input(),ao=a._browser_output(),bo=b._browser_output();
  const toCaller=delayLine(line.delayMs*8),toAnswerer=delayLine(line.delayMs*8);
  a.HEAPU8.fill(255,ai,ai+160);b.HEAPU8.fill(255,bi,bi+160);
  // 60 s to connect, selected sustained interval, then 15 s to drain the
  // bounded in-flight window. A final partial interval never counts as a pass.
  for(let block=0;block<(60+duration+15)*50;block++){
   if(shouldStop())throw new Error('Call cancelled');
   time=(block+1)/50;
   const sa=a._browser_step(),sb=b._browser_step();
   a.HEAPU8.set(toCaller(b.HEAPU8.slice(bo,bo+160)),ai);
   b.HEAPU8.set(toAnswerer(a.HEAPU8.slice(ao,ao+160)),bi);
   const ar=a._browser_rate(),br=b._browser_rate();
   if(ar&&br){
    const caller={tx:a._browser_tx_rate(),rx:a._browser_rx_rate()},answerer={tx:b._browser_tx_rate(),rx:b._browser_rx_rate()};
    modulation=a.UTF8ToString(a._browser_modulation());checkPcmRates(modulation,caller,answerer);
    txRate=caller.tx;rxRate=caller.rx;
    if(connectedAt===null){connectedAt=time;sendUntil=(block+1+duration*50)/50;}
   }
   for(const s of streams){
    const count=s.rx._browser_received_n(),data=s.rx.HEAPU8.subarray(s.rx._browser_received(),s.rx._browser_received()+count);
    for(const byte of data){if(byte!==s.expected())s.mismatches++;s.received++;}
    s.rx._browser_clear_received();
   }
   if(connectedAt!==null&&(sa===0||sb===0||sa===5||sb===5||!ar||!br)){disconnected=true;break;}
   if(connectedAt===null&&time>=60)break;
   if(connectedAt!==null&&time<sendUntil){
    for(const [index,s] of streams.entries()){
     if(s.offset===4096){for(let i=0;i<4096;i++)s.buffer[i]=s.next();s.tx.HEAPU8.set(s.buffer,s.pointer);s.offset=0;}
     // Keep at most one second of data in flight, so a low-rate healthy
     // connection can drain without mistaking queued bytes for corruption.
     const window=Math.max(512,Math.min(8192,(index===0?txRate:rxRate)/8));
     const available=Math.min(4096-s.offset,Math.max(0,Math.floor(window-(s.accepted-s.received))));
     if(available){const n=s.tx._browser_send(s.pointer+s.offset,available);if(n<0||n>available)throw new Error('Invalid DTE acceptance count');s.offset+=n;s.accepted+=n;}
    }
   }
   if(sendUntil!==null&&time>=sendUntil){
    if(!trafficCaptured){trafficBytes=streams[0].received+streams[1].received;trafficCaptured=true;}
    if(streams.every(s=>s.received===s.accepted))break;
    if(time>=sendUntil+15){error='Payload did not drain within 15 seconds';break;}
   }
   if(block%50===0){onProgress({time,state:connectedAt===null?'Training':time<sendUntil?'Sustained transfer':'Draining queued data'});await new Promise(resolve=>setTimeout(resolve,0));}
  }
 }catch(e){if(shouldStop()||e.message==='Call cancelled')throw e;error=e.message;}
 finally{clippedSamples=(initializedA?a._browser_line_clipped():0)+(initializedB?b._browser_line_clipped():0);if(pa)a._free(pa);if(pb)b._free(pb);if(initializedA)a._browser_destroy();if(initializedB)b._browser_destroy();}
 const mismatchBytes=streams.reduce((n,s)=>n+s.mismatches,0),receivedBytes=streams.reduce((n,s)=>n+s.received,0);
 const finishedInterval=sendUntil!==null&&time>=sendUntil;
 const passed=connectedAt!==null&&finishedInterval&&!disconnected&&!error&&!mismatchBytes&&streams.every(s=>s.accepted>0&&s.received===s.accepted)&&modulation===`V${mode}`;
 return {mode,attempt:attempt+1,connected:connectedAt!==null,connectSeconds:connectedAt,txRate,rxRate,modulation,passed,disconnected,error,
  clippedSamples,requestedTrafficSeconds:duration,trafficSeconds:connectedAt===null?0:Math.min(duration,time-connectedAt),simulationSeconds:time,
  verifiedBytes:!mismatchBytes?receivedBytes:0,receivedBytes,mismatchBytes,sentBytes:streams.reduce((n,s)=>n+s.accepted,0),
  callerSentBytes:streams[0].accepted,answererSentBytes:streams[1].accepted,callerReceivedBytes:streams[1].received,answererReceivedBytes:streams[0].received,
  bytesDuringTraffic:trafficBytes,usefulBitsPerSecond:finishedInterval?trafficBytes*8/duration:null,retrainRequests,
  failureReason:passed?null:error||(!connectedAt?'No connection':disconnected?'Disconnected':mismatchBytes?'Decoded bytes did not match':!finishedInterval?'Incomplete traffic interval':modulation!==`V${mode}`?'Connected using a fallback modulation':'Incomplete payload')};
}
