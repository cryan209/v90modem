import createModem from './modem-engine.mjs';
import {negotiationRecords} from './negotiation.mjs';
export function expand(code){const u=(~code)&255;const t=(((u&15)<<3)+132)<<((u>>4)&7);return ((u&128)?132-t:t-132)/32768;}
export async function runPair(mode,text,onProgress=()=>{},shouldStop=()=>false,startup={v8:true,v8bis:false}){
  if(['x2','flex'].includes(String(mode)))return runPcmDemo(String(mode),text,onProgress,shouldStop);
  if(![32,34,90].includes(Number(mode)))throw new Error('Unsupported modem mode');
  if(startup.v8===false&&Number(mode)!==32)throw new Error('V.34 and V.90 require V.8 in this demo');
  if(startup.v8bis&&startup.v8===false)throw new Error('V.8bis requires V.8 in this demo');
  const startupFlags=(startup.v8===false?0:1)|(startup.v8bis?2:0);
  const logs=[],protocol=[],bitClock=[];let simTime=0,droppedProtocol=0;
  const options=(side)=>({print:line=>capture(side,line),printErr:line=>capture(side,line)});
  function capture(side,line){if(protocol.length<100000){protocol.push({time:simTime,side,line});logs.push(line);}else droppedProtocol++;}
  const a=await createModem(options('caller')),b=await createModem(options('answerer'));
  if(a.HEAPU8.buffer===b.HEAPU8.buffer)throw new Error('Modem instances share memory');
  const frame=value=>{const body=new TextEncoder().encode(value),out=new Uint8Array(body.length+2);out[0]=body.length&255;out[1]=body.length>>8;out.set(body,2);return out;};
  const unframe=(m)=>{const n=m._browser_received_n(),data=m.HEAPU8.slice(m._browser_received(),m._browser_received()+n);if(n<2)return {text:'',complete:false};const size=data[0]|(data[1]<<8);if(size>65533)throw new Error('Corrupt message length');return {text:new TextDecoder().decode(data.slice(2,Math.min(n,size+2))),complete:n>=size+2};};
  const bytes=frame(text);if(bytes.length>60002)throw new Error('Message exceeds 60000 UTF-8 bytes');let replies=null;
  const pa=a._malloc(bytes.length),pb=b._malloc(bytes.length+5);
  a.HEAPU8.set(bytes,pa);
  a._browser_init(Number(mode),1,startupFlags,startup.v34MaxRate||0);b._browser_init(Number(mode),0,startupFlags,startup.v34MaxRate||0);
  const ai=a._browser_input(),bi=b._browser_input(),ao=a._browser_output(),bo=b._browser_output();
  a.HEAPU8.fill(255,ai,ai+160);b.HEAPU8.fill(255,bi,bi+160);
  const callerCodewords=new Uint8Array(480000),answererCodewords=new Uint8Array(480000);
  const left=[],right=[],events=[],constellations=[];let at=0,bt=0,doneAt=0,lastState='',lastRx='',lastReply='';
  try{
    for(let block=0;block<3000;block++){
      if(shouldStop())throw new Error('Call cancelled');
      simTime=(block+1)*.02;
      const sa=a._browser_step(),sb=b._browser_step();
      const x=a.HEAPU8.slice(ao,ao+160),y=b.HEAPU8.slice(bo,bo+160);
      a.HEAPU8.set(y,ai);b.HEAPU8.set(x,bi);
      callerCodewords.set(x,block*160);answererCodewords.set(y,block*160);
      left.push(Float32Array.from(x,expand));right.push(Float32Array.from(y,expand));
      if(block%10===0){const raw=b.UTF8ToString(b._browser_monitor());if(raw){const monitor=JSON.parse('{'+raw+'}');if(monitor.iq[0].length)constellations.push({time:(block+1)*.02,points:monitor.iq[0]});}}
      bitClock.push({time:simTime,callerTx:a._browser_bit_count(1),callerRx:a._browser_bit_count(0),answererTx:b._browser_bit_count(1),answererRx:b._browser_bit_count(0)});
      const time=(block+1)*.02, ar=a._browser_rate(),br=b._browser_rate();
      const startupName=side=>startup.v8===false?'V.25':startup.v8bis&&!protocol.some(e=>e.side===side&&/V\.8bis complete/.test(e.line))?'V.8bis':'V.8';
      const states=side=>['idle','dialing',startupName(side),'training','data','hangup'];
      const state=`${sa}/${sb}/${ar}/${br}/${startupName('caller')}/${startupName('answerer')}`;
      if(state!==lastState){events.push({time,message:ar&&br?`Connected · caller TX ${a._browser_tx_rate()} / RX ${a._browser_rx_rate()} bit/s`:`Training · caller ${states('caller')[sa]} / answerer ${states('answerer')[sb]}`});lastState=state;}
      if(ar && br){
        if(at<bytes.length)at+=a._browser_send(pa+at,bytes.length-at);
        if(replies && bt<replies.length)bt+=b._browser_send(pb+bt,replies.length-bt);
      }
      const remote=unframe(b),local=unframe(a),rx=remote.text,reply=local.text;
      if(remote.complete && !replies){replies=frame('ACK: '+rx);b.HEAPU8.set(replies,pb);}
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
    return {left:l,right:r,events,startup,protocol,bitClock,wireBits,g711,negotiation,droppedProtocol,constellations,received:lastRx,reply:lastReply,passed:!!doneAt,logs,
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
   pcm:{samples,frameSamples:mode==='x2'?6:8,frameBits:mode==='x2'?42:56,sourceBytes:bytes.length,decodedBytes:size},logs:[]};
 }finally{a._free(src);a._free(tx);b._free(rx);b._free(decoded);}
}
