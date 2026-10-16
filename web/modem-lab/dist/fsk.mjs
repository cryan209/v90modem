// ITU-T V.21 §2: 300 bit/s, channel 1 mark 980 Hz / space 1180 Hz.
// Bell 103 uses 1270/1070 Hz for the originating channel.
// V.14-style asynchronous 8N1 demonstration, not a V.25 startup emulator.
export const RATE = 8000;
export const profiles = {
  v21: {name:'V.21', mark:980, space:1180},
  bell103: {name:'Bell 103', mark:1270, space:1070},
};
export function transmit(text, profile='v21', snr=99) {
  const bytes=new TextEncoder().encode(text);
  const bits=Array(24).fill(1);
  for(const byte of bytes){ bits.push(0); for(let b=0;b<8;b++)bits.push((byte>>b)&1); bits.push(1); }
  bits.push(...Array(12).fill(1));
  const samples=new Float32Array(Math.ceil(bits.length*RATE/300));
  let phase=0, random=0x12345678;
  const noise=snr>=99?0:(0.22/Math.sqrt(2))*10**(-snr/20)*Math.sqrt(3);
  const p=profiles[profile];
  for(let i=0;i<samples.length;i++){
    const bit=bits[Math.min(bits.length-1,Math.floor(i*300/RATE))];
    phase+=2*Math.PI*(bit?p.mark:p.space)/RATE;
    random=(Math.imul(random,1664525)+1013904223)>>>0;
    samples[i]=0.22*Math.sin(phase)+noise*(random/2147483648-1);
  }
  return {samples,bits,bytes};
}
// Least-squares sinusoid fit: compares tones without assuming oscillator phase.
function energy(samples,start,end,hz){
  let xc=0,xs=0,cc=0,ss=0,cs=0;
  for(let i=Math.max(0,Math.ceil(start));i<Math.min(samples.length,Math.floor(end));i++){
    const a=2*Math.PI*hz*i/RATE,c=Math.cos(a),s=Math.sin(a);
    xc+=samples[i]*c;xs+=samples[i]*s;cc+=c*c;ss+=s*s;cs+=c*s;
  }
  const det=cc*ss-cs*cs;
  return det>1e-8?(ss*xc*xc+cc*xs*xs-2*cs*xc*xs)/det:0;
}
export function receive(samples,profile='v21'){
  const p=profiles[profile], symbol=RATE/300, bytes=[],frames=[];
  const score=(s,e)=>energy(samples,s,e,p.mark)-energy(samples,s,e,p.space);
  let previous=score(0,symbol), end=Math.ceil(symbol)+1;
  while(end+10*symbol<samples.length){
    const current=score(end-symbol,end);
    if(previous>0 && current<=0){
      const edge=end-symbol/2;
      let byte=0;
      for(let b=0;b<8;b++) if(score(edge+(b+1)*symbol,edge+(b+2)*symbol)>0) byte|=1<<b;
      const stop=score(edge+9*symbol,edge+10*symbol)>0;
      frames.push({sample:Math.round(edge),byte,stop});
      if(stop) bytes.push(byte);
      end=Math.round(edge+10*symbol);previous=1;
    }else{previous=current;end++;}
  }
  return {text:new TextDecoder().decode(new Uint8Array(bytes)),bytes,frames};
}
export function wav(samples,right=null){
  const channels=right?2:1;
  const buffer=new ArrayBuffer(44+samples.length*2*channels),v=new DataView(buffer);
  const word=(offset,s)=>{for(let i=0;i<s.length;i++)v.setUint8(offset+i,s.charCodeAt(i));};
  word(0,'RIFF');v.setUint32(4,36+samples.length*2*channels,true);word(8,'WAVE');word(12,'fmt ');
  v.setUint32(16,16,true);v.setUint16(20,1,true);v.setUint16(22,channels,true);
  v.setUint32(24,RATE,true);v.setUint32(28,RATE*2*channels,true);v.setUint16(32,2*channels,true);v.setUint16(34,16,true);
  word(36,'data');v.setUint32(40,samples.length*2*channels,true);
  for(let i=0;i<samples.length;i++)for(let ch=0;ch<channels;ch++){const s=ch?right[i]:samples[i];v.setInt16(44+(i*channels+ch)*2,Math.round(Math.max(-1,Math.min(1,s))*32767),true);}
  return buffer;
}
