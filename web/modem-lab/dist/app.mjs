import {transmit,receive,wav,RATE} from './fsk.mjs';
const $=id=>document.getElementById(id);
let mode='v21',context,source,worker,result,started=0,animation=0,downloadUrl,playing=false,elapsed=0,plotType='spectrum';
const descriptions={
 v21:['Two tones, every bit.','V.21 switches between 980 Hz for a 1 and 1180 Hz for a 0. This demo sends eight data bits between a start bit and a stop bit, 300 times a second.'],
 bell103:['The sound of 300 bits a second.','Bell 103’s originating channel uses 1270 Hz for a 1 and 1070 Hz for a 0. A separate JavaScript receiver recovers your message from the waveform.'],
 '32':['A conversation before the conversation.','Two instances of the real modem engine negotiate V.32 at up to 9,600 bit/s, train their receivers, and exchange your message and an acknowledgement over G.711.'],
 '90':['A digital end. An analogue end.','V.90 sends PCM downstream and QAM upstream. The calling modem runs the analogue role; the answering modem runs the digital role. Both train and recover real payload bytes.'],
};
const isFsk=()=>mode==='v21'||mode==='bell103';
function updateCount(){const n=new TextEncoder().encode($('message').value).length;$('byteCount').textContent=n;$('dial').disabled=n===0||n>240||playing;}
function select(next){stop();mode=next;document.querySelectorAll('.mode').forEach(b=>{b.classList.toggle('active',b.dataset.mode===mode);b.setAttribute('aria-pressed',String(b.dataset.mode===mode));});
 $('modeTitle').textContent=descriptions[mode][0];$('modeDescription').textContent=descriptions[mode][1];
 $('engineBadge').textContent=isFsk()?'JAVASCRIPT · LIVE DSP':'WEBASSEMBLY · REAL ENGINE';
 $('showConstellation').disabled=isFsk();setPlot(isFsk()?'spectrum':'constellation');
 $('noise').disabled=!isFsk();$('noiseHint').textContent=isFsk()?'Noise is added to the FSK signal before the receiver sees it.':'The advanced modes use a clean, byte-exact G.711 virtual line.';
 $('received').textContent='';$('verification').textContent='AWAITING SIGNAL';$('status').textContent='Ready to dial';$('events').textContent='The receiver listens to the generated samples.\nNo prerecorded audio. No text shortcut.';
 $('clock').textContent='00.00 s';$('duration').textContent='—';result=null;$('download').disabled=true;draw(0);
}
function stop(){worker?.terminate();worker=null;if(source){source.onended=null;try{source.stop();}catch{}source=null;}
 playing=false;cancelAnimationFrame(animation);$('stop').disabled=true;$('lamp').classList.remove('on');document.querySelectorAll('.mode').forEach(b=>b.disabled=false);$('message').disabled=false;updateCount();
 if(result){$('status').textContent='Stopped';}
}
function showEvents(t){
 const visible=result.events.filter(e=>e.time<=t);
 $('events').textContent=visible.filter(e=>e.message).map(e=>`${e.time.toFixed(2).padStart(5)} s  ${e.message}`).join('\n');
 const rx=visible.filter(e=>e.received!==undefined).at(-1);
 $('received').textContent=rx?.received||'';
 if(rx?.reply)$('received').textContent+='\n\n← '+rx.reply;
 const verified=visible.some(e=>e.message==='Payload verified in both directions'||e.message==='Payload verified');
 $('verification').textContent=verified?'PAYLOAD VERIFIED':rx?'RECEIVING':'AWAITING SIGNAL';
 const status=visible.filter(e=>e.message).at(-1);if(status)$('status').textContent=status.message;
 $('events').scrollTop=$('events').scrollHeight;
}
function plot(canvas,samples,pos,spectrum=false){
 const c=canvas.getContext('2d'),w=canvas.width,h=canvas.height;c.clearRect(0,0,w,h);
 c.strokeStyle='#293b2c';c.lineWidth=1;
 for(let i=0;i<=8;i++){c.beginPath();c.moveTo(i*w/8,0);c.lineTo(i*w/8,h);c.stroke();}
 for(let i=0;i<=4;i++){c.beginPath();c.moveTo(0,i*h/4);c.lineTo(w,i*h/4);c.stroke();}
 if(!samples)return;
 c.strokeStyle='#c8ee9a';c.lineWidth=2;c.beginPath();
 if(!spectrum){for(let i=0;i<160;i++){const v=samples[Math.max(0,Math.min(samples.length-160,pos-80))+i]||0;const x=i*w/159,y=h/2-v*h*1.5;if(i)c.lineTo(x,y);else c.moveTo(x,y);}}
 else{for(let k=0;k<=128;k++){let re=0,im=0;for(let n=0;n<256;n++){const sample=samples[Math.max(0,Math.min(samples.length-1,pos+n-128))]||0;const window=.5-.5*Math.cos(2*Math.PI*n/255);re+=sample*window*Math.cos(2*Math.PI*k*n/256);im-=sample*window*Math.sin(2*Math.PI*k*n/256);}const db=20*Math.log10(Math.hypot(re,im)/128+1e-6);const y=h-(Math.max(-80,db)+80)/80*h;if(k)c.lineTo(k*w/128,y);else c.moveTo(0,y);}}
 c.stroke();
}
function setPlot(type){plotType=type;$('showSpectrum').classList.toggle('selected',type==='spectrum');$('showConstellation').classList.toggle('selected',type==='constellation');$('plotLabel').textContent=type==='spectrum'?'FREQUENCY SPECTRUM':'ANSWERER · RECEIVED QAM SYMBOLS';$('spectrum').setAttribute('aria-label',type==='spectrum'?'Frequency spectrum of the current audio':'Measured received QAM constellation');$('plotScale').textContent=type==='spectrum'?'0 — 4,000 Hz':'I / Q · AUTO SCALE';draw(0);}
function plotConstellation(t){
 const canvas=$('spectrum'),c=canvas.getContext('2d'),w=canvas.width,h=canvas.height;c.clearRect(0,0,w,h);
 c.strokeStyle='#293b2c';c.beginPath();c.moveTo(w/2,0);c.lineTo(w/2,h);c.moveTo(0,h/2);c.lineTo(w,h/2);c.stroke();
 const frame=result?.constellations?.filter(f=>f.time<=t).at(-1);
 if(!frame){c.fillStyle='#83978b';c.font='16px monospace';c.fillText('Waiting for QAM symbols',20,h/2+6);return;}
 const points=frame.points,scale=Math.max(1,...points.flatMap(p=>p.map(Math.abs)))*1.1;
 const unit=Math.min(w,h)*.46/scale;c.fillStyle='#c8ee9aaa';
 for(const [re,im] of points){c.beginPath();c.arc(w/2+re*unit,h/2-im*unit,2.1,0,Math.PI*2);c.fill();}
}
function draw(t){const pos=Math.round(t*RATE);plot($('scope'),result?.left,pos);if(plotType==='spectrum')plot($('spectrum'),result?.left,pos,true);else plotConstellation(t);$('clock').textContent=t.toFixed(2).padStart(5,'0')+' s';}
function tick(){if(!playing)return;elapsed=Math.min(context.currentTime-started,result.left.length/RATE);draw(elapsed);showEvents(elapsed);animation=requestAnimationFrame(tick);}
async function play(){
 const buffer=context.createBuffer(result.right?2:1,result.left.length,RATE);buffer.copyToChannel(result.left,0);if(result.right)buffer.copyToChannel(result.right,1);
 source=context.createBufferSource();source.buffer=buffer;const gain=context.createGain();gain.gain.value=.65;source.connect(gain).connect(context.destination);
 source.onended=()=>{elapsed=result.left.length/RATE;draw(elapsed);showEvents(elapsed);stop();$('status').textContent=result.passed?'Complete · payload verified':result.error||'Run ended without a verified payload';};
 started=context.currentTime;source.start();$('lamp').classList.add('on');$('status').textContent='Sending carrier';tick();
 $('duration').textContent=(result.left.length/RATE).toFixed(2)+' s';$('download').disabled=false;
}
async function dial(){
 stop();const text=$('message').value;if(!text||new TextEncoder().encode(text).length>240)return;
 try{context??=new AudioContext();await context.resume();playing=true;updateCount();$('stop').disabled=false;$('message').disabled=true;document.querySelectorAll('.mode').forEach(b=>b.disabled=true);
 $('received').textContent='';$('verification').textContent='AWAITING SIGNAL';$('events').textContent='';$('download').disabled=true;
 if(isFsk()){
   const snrs=[99,24,12,0],tx=transmit(text,mode,snrs[Number($('noise').value)]),rx=receive(tx.samples,mode);
   const passed=rx.text===text;const events=[{time:0,message:'Idle carrier · acquiring 300 bit/s FSK'}];
   rx.frames.filter(f=>f.stop).forEach((f,i)=>events.push({time:(f.sample+10*RATE/300)/RATE,received:new TextDecoder().decode(new Uint8Array(rx.bytes.slice(0,i+1)))}));
   events.push({time:tx.samples.length/RATE-.05,message:passed?'Payload verified':'Payload mismatch · try a cleaner line'});
   result={left:tx.samples,events,passed};await play();
 }else{
   $('status').textContent='Loading the WebAssembly modems…';
   worker=new Worker('./worker.mjs',{type:'module'});
   worker.onerror=e=>fail(e.message||'Could not load the modem worker.');
   worker.onmessage=async({data})=>{if(data.type==='progress'){$('status').textContent=`Generating call · ${data.time.toFixed(1)} s simulated`;}else if(data.type==='error'){fail(data.message);}else if(data.type==='result'){worker.terminate();worker=null;result=data;await play();}};
   worker.postMessage({mode,text});
 }
 }catch(e){fail(e.message);}
}
function fail(message){stop();$('status').textContent='Could not run: '+message;$('events').textContent='Serve this folder over HTTP and build the WASM assets first.\n'+message;}
$('showSpectrum').addEventListener('click',()=>{setPlot('spectrum');draw(elapsed);});$('showConstellation').addEventListener('click',()=>{setPlot('constellation');draw(elapsed);});
$('dial').addEventListener('click',dial);$('stop').addEventListener('click',stop);$('message').addEventListener('input',updateCount);
$('noise').addEventListener('input',()=>{$('noiseValue').textContent=['Clean','24 dB SNR','12 dB SNR','0 dB SNR'][$('noise').value];});
document.querySelectorAll('.mode').forEach(b=>b.addEventListener('click',()=>select(b.dataset.mode)));
$('download').addEventListener('click',()=>{if(!result)return;if(downloadUrl)URL.revokeObjectURL(downloadUrl);downloadUrl=URL.createObjectURL(new Blob([wav(result.left,result.right)],{type:'audio/wav'}));const a=document.createElement('a');a.href=downloadUrl;a.download=`modem-${mode}.wav`;a.click();});
window.addEventListener('pagehide',()=>{stop();context?.close();if(downloadUrl)URL.revokeObjectURL(downloadUrl);});
updateCount();draw(0);
