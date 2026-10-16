// Passive inspection. V.42 §§7–8 HDLC framing and Table 11 XID parameters.
// Wire bits are already demodulated/descrambled by the real modem engine.
export const protocolHex=bytes=>Array.from(bytes,b=>b.toString(16).padStart(2,'0')).join(' ');
export const octetBits=bytes=>Array.from(bytes,b=>Array.from({length:8},(_,i)=>(b>>i)&1).join('')).join(' ');
export function hdlcFrames(bits,clock,key){
  const frames=[];let previous=-1;
  for(let end=bits.indexOf('01111110');end>=0;end=bits.indexOf('01111110',end+1)){
    if(previous>=0 && end>previous+8){
      const stuffed=bits.slice(previous+8,end);let plain='',ones=0,aborted=false;
      for(const bit of stuffed){if(bit==='1'){if(++ones>5){aborted=true;break;}plain+='1';}else{if(ones!==5)plain+='0';ones=0;}}
      if(!aborted && plain.length>=32 && plain.length%8===0){
        const bytes=Uint8Array.from({length:plain.length/8},(_,i)=>parseInt(plain.slice(i*8,i*8+8).split('').reverse().join(''),2));
        let crc=65535;for(const byte of bytes){crc^=byte;for(let i=0;i<8;i++)crc=(crc>>>1)^((crc&1)?0x8408:0);}
        const c=bytes[1],u=c&0xef,names={0xaf:'XID',0x6f:'SABME',0x63:'UA',0x43:'DISC',0x0f:'DM',0x87:'FRMR'};
        const type=(c&1)===0?'I':(c&3)===1?['RR','RNR','REJ','SREJ'][(c>>2)&3]:names[u]||'U';
        const time=clock.find(t=>t[key]>=end+8)?.time||clock.at(-1)?.time||0;
        const fields=[];
        if(type==='XID' && bytes[2]===0x82){
          let at=3;const limit=bytes.length-2;
          while(at+3<=limit){const group=bytes[at++],len=bytes[at++]*256+bytes[at++],finish=Math.min(at+len,limit);
            while(at+2<=finish){const id=bytes[at++],size=bytes[at++];if(at+size>finish)break;const value=bytes.slice(at,at+size);at+=size;const number=Array.from(value).reduce((n,b)=>n*256+b,0);
              const standard={3:'HDLC optional functions',5:'TX N401 (bits)',6:'RX N401 (bits)',7:'TX window k',8:'RX window k'};
              const compression={0:'Parameter set ID',1:'V.42bis directions P0',2:'V.42bis codewords P1',3:'V.42bis max string P2'};
              fields.push(`${group===0x80?standard[id]||'PI '+id:group===0xf0?compression[id]||'private PI '+id:'group '+group+' PI '+id}: ${id===3&&group===0x80?protocolHex(value):id===0&&group===0xf0?String.fromCharCode(...value):number}`);
            }at=finish;
          }
        }
        frames.push({time,side:key.startsWith('caller')?'caller':'answerer',direction:key.endsWith('Tx')?'TX':'RX',stage:'lapm',title:`${type} · ${crc===0xf0b8?'FCS valid':'FCS INVALID'}`,type,valid:crc===0xf0b8,hex:protocolHex(bytes),bits:plain,wireStart:previous+8,wireEnd:end,fields});
      }
    }
    previous=end;
  }
  return frames;
}
export function protocolStage(line){
 if(/V\.8|[<>](CM|JM|CJ|CI):|V8 result/.test(line))return 'v8';
 if(/INFO[01]/.test(line))return 'info';
 if(/Ja|Jd|DIL|CP[at]|TRN[12]d/.test(line))return 'pcm';
 if(/LAPM|V\.42|XID|SABME|\bUA\b/.test(line))return 'lapm';
 return 'other';
}
export const cleanProtocolLine=line=>line.replace(/^(?:FLOW|WARNING|DEBUG|ERROR)\s+\[[^\]]+\]\s?/,'').replace(/^\[(?:ME|V90|V34|TRACE[^\]]*)\]\s*/,'');
export function negotiationRecords(result){
 const records=[],seen=new Map();
 const add=r=>{const key=r.side+'|'+r.title+'|'+r.hex+'|'+r.fields.join('|');if(seen.has(key)){seen.get(key).repeats++;seen.get(key).repeatTimes.push(r.time);return;}r.repeats=1;r.repeatTimes=[r.time];records.push(r);seen.set(key,r);};
 const logs=result.protocol||[],contexts={};
 for(const e of logs){const line=cleanProtocolLine(e.line);e.stage=/^\s{2,}/.test(line)?contexts[e.side]||'other':protocolStage(e.line);if(!/^\s{2,}/.test(line))contexts[e.side]=e.stage;}
 for(let i=0;i<logs.length;i++){
   const e=logs[i];if(/^\[TRACE/.test(e.line))continue;const line=cleanProtocolLine(e.line),stage=e.stage||protocolStage(e.line);let title='',fields=[],hex='';
   const menu=line.match(/([<>])(CM|JM|CJ|CI):\s*((?:[0-9a-f]{2}\s*)+)/i);
   if(menu){title=`${menu[1]==='<'?'TX':'RX'} ${menu[2]} · ${menu[2]==='CM'?'calling menu':menu[2]==='JM'?'joint menu':menu[2]==='CJ'?'menu acknowledgement':'call indication'}`;hex=menu[3].trim();}
   else if(/V\.8 peer (offer|summary):/.test(line) && !/protocol=None/.test(line))title=line;
   else if(/Rx INFO0a V\.92 flags:/.test(line)){title='RX INFO0a · peer is the analogue modem';fields=[line];}
   else if(/^(Tx|Rx) INFO[01][acd]?(?: \(|:)/.test(line)){
     title=line;
     for(let j=i+1;j<Math.min(logs.length,i+25);j++){
       if(logs[j].side!==e.side)continue;
       const next=cleanProtocolLine(logs[j].line);
       if(/^\s{2,}/.test(next)||/^Tx INFO0d V\.92 flags/.test(next))fields.push(next.trim());else break;
     }
   }
   else if(/parsed Ja DIL descriptor|Ja confirmed by CRC-valid|Phase 3: sending DIL|analogue DIL measured|V\.42 (detection succeeded|XID negotiated|LAPM connected)|strict RX event: valid INFO1a/.test(line))title=line;
   if(title.startsWith('Tx INFO0d'))title+=' · digital modem capabilities';
   if(title.startsWith('V.8 peer summary:')&&title.includes('PCM=V.90/V.92 analogue'))fields.push('The peer advertises the analogue PCM role.');
   if(title.startsWith('V.8 peer summary:')&&title.includes('PCM=V.90/V.92 digital'))fields.push('The peer advertises the digital PCM role.');
   if(stage==='info'){const evidence=fields.join(' '),up=evidence.match(/upstream(?: rate)?(?:_| )code\s*=?\s*(\d+)/),down=evidence.match(/downstream(?: rate)?(?:_| )code\s*=?\s*(\d+)/);const rates=[2400,2743,2800,3000,3200,3429];if(up&&rates[Number(up[1])])fields.push('Upstream symbol rate: '+rates[Number(up[1])]+' baud');if(down&&Number(down[1])===6)fields.push('Downstream selection: PCM');}
   if(title)add({...e,stage,title,fields,hex,source:'engine log'});
 }
 const rawFrames=Object.entries(result.wireBits||{}).flatMap(([key,bits])=>hdlcFrames(bits,result.bitClock||[],key));
 for(const f of rawFrames)if(f.valid && !['I','RR'].includes(f.type))add({...f,title:f.direction+' '+f.title,source:'wire decode'});
 return {records:records.sort((a,b)=>a.time-b.time),frames:rawFrames};
}
export function inspectorText(result,time,view='simple',side='all',stage='all'){
 const allowed=r=>r.time<=time && (side==='all'||r.side===side) && (stage==='all'||r.stage===stage);
 const stamp=r=>`${r.time.toFixed(2).padStart(5)} s  ${r.side.toUpperCase()}`;
 if(view==='simple')return (result.negotiation?.records||[]).filter(allowed).map(r=>`${stamp(r)}  ${r.title}${r.repeatTimes.filter(t=>t<=time).length>1?' [repeated]':''}${r.fields.length?'\n    '+r.fields.join('\n    '):''}${r.hex?'\n    '+r.hex:''}`).join('\n\n')||'No decoded negotiation records yet.';
 if(view==='verbose'){
   const entries=(result.protocol||[]).map(e=>({...e,stage:e.stage||protocolStage(e.line)})).filter(allowed);
   const visible=entries.slice(-600);
   return `${entries.length>600?`Showing latest 600 of ${entries.length} log lines. Export JSON for the complete capture.\n\n`:''}`+visible.map(r=>`${stamp(r)}  ${cleanProtocolLine(r.line)}`).join('\n')||'No engine logs yet.';
 }
 const parts=['RAW COVERAGE: V.8 menu octets; packed INFO1 frames where logged; demodulated data-layer TX/RX bits including LAPM. The complete G.711 DS0 is also captured. A full demodulated startup bitstream is not available; Ja has decoded fields, and DIL is PCM training. Octet bits below are LSB first; packed INFO bytes retain engine layout.'];
 if(stage==='all'||stage==='v8'||stage==='info'){
   const raw=(result.protocol||[]).filter(e=>e.time<=time&&(side==='all'||e.side===side));
   for(const e of raw){const l=cleanProtocolLine(e.line),menu=l.match(/[<>](CM|JM|CJ|CI):\s*((?:[0-9a-f]{2}\s*)+)/i),info=l.match(/raw frame \((\d+) bits, \d+ bytes\):\s*(.*)/i);
     if(menu&&(stage==='all'||stage==='v8')){const bytes=menu[2].trim().split(/\s+/).map(h=>parseInt(h,16));parts.push(`${stamp(e)} ${menu[1]} OCTETS\n${protocolHex(bytes)}\nLSB-first octets: ${octetBits(bytes)}`);}
     if(info&&(stage==='all'||stage==='info'))parts.push(`${stamp(e)} PACKED INFO (engine label retained)\n${l}`);
   }
 }
 if(stage==='all'||stage==='pcm')for(const [key,hex] of Object.entries(result.g711||{})){
   if(side!=='all'&&!key.startsWith(side))continue;
   const count=Math.min(hex.length/2,Math.floor(time*8000));
   parts.push(`${key} · exact µ-law DS0 · ${count} codewords so far · latest 64 shown\n${hex.slice(Math.max(0,count-64)*2,count*2).match(/.{1,2}/g)?.join(' ')||'(no codewords yet)'}\nDIL is PCM training codewords, not application bits. Export includes the complete DS0.`);
 }
 if(stage==='all'||stage==='lapm')for(const [key,bits] of Object.entries(result.wireBits||{})){
   const endpoint=key.startsWith('caller')?'caller':'answerer';if(side!=='all'&&side!==endpoint)continue;
   const count=result.bitClock?.filter(c=>c.time<=time).at(-1)?.[key]||0;
   parts.push(`${key} · ${count} captured bits so far · latest 512 shown, unmodified transmission order\n${bits.slice(Math.max(0,count-512),count).match(/.{1,64}/g)?.join('\n')||'(no data-layer bits yet)'}`);
 }
 if(stage==='all'||stage==='lapm')for(const f of (result.negotiation?.frames||[]).filter(allowed).slice(-24))parts.push(`${stamp(f)} ${f.direction} ${f.title}\n${f.hex}\nDe-stuffed LSB-first octets: ${f.bits.match(/.{1,8}/g)?.join(' ')}`);
 return parts.join('\n\n');
}
