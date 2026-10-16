let reliabilityResult=null,reliabilityRunning=false,reliabilityUrl=null,reliabilityToken=0;
import {transmit,receive,wav,RATE} from './fsk.mjs';
import {inspectorText} from './negotiation.mjs';
const $=id=>document.getElementById(id);
let mode='v21',context,source,worker,result,started=0,animation=0,downloadUrl,playing=false,elapsed=0,lastInspect=-1,captureUrl,plotType='spectrum';
const descriptions={
 v22:['Phase shifts at 1,200 bits a second.','Two real V.22 engines use differential phase modulation at 600 symbols per second. They train, then exchange your text and an acknowledgement at 1,200 bit/s in both directions.'],
 v22bis:['Four bits in every symbol.','V.22bis uses 16-QAM at 600 symbols per second for 2,400 bit/s in each direction. Two real engines negotiate and independently recover your text over LAPM.'],
 v32bis:['The faster V.32 family.','Two real V.32bis engines train with echo cancellation and negotiate up to 14,400 bit/s. The original V.32 preset remains limited to 9,600 bit/s.'],
 v23:['A terminal and a page of text.','V.23 mode 2 sends FSK on two unequal channels: the caller uploads at 75 bit/s and downloads at 1,200 bit/s. Real engines exchange asynchronous characters without V.8 or LAPM. Longer messages take time on the slow channel.'],
 bell212:['The Bell answer tone.','Bell 212A starts with a 2,225 Hz answer tone, then carries 1,200 bit/s in each direction using differential phase modulation. Two real engines exchange your message over LAPM without V.8.'],
 '91':['A digital bearer in both directions.','Two real V.91 engines negotiate symmetric PCM over a synchronous, byte-exact G.711 bearer, then verify your text over LAPM. This is a digital-to-digital experiment; audible playback does not model an analogue telephone loop.'],
 '92':['PCM in both directions.','Two real V.92 engines negotiate PCM upstream and downstream, then verify your text over LAPM. Compare full startup with the engine’s Quick Connect procedures on the same virtual line.'],
 x2:['The first wave of PCM modems.','The real x2 mapper sends your text as six-sample PCM frames at 56,000 bit/s. A separate decoder recovers it from exact µ-law codewords using a fixed profile. Downstream payload only: no dial-up training or upstream connection.'],
 flex:['Eight samples, fifty-six bits.','The real K56flex mapper sends your text as eight-sample PCM frames at 56,000 bit/s. A separate decoder recovers it from exact µ-law codewords using a fixed profile. Downstream payload only: no dial-up training or upstream connection.'],
 v21:['Two tones, every bit.','V.21 switches between 980 Hz for a 1 and 1180 Hz for a 0. This demo sends eight data bits between a start bit and a stop bit, 300 times a second.'],
 bell103:['The sound of 300 bits a second.','Bell 103’s originating channel uses 1270 Hz for a 1 and 1070 Hz for a 0. A separate JavaScript receiver recovers your message from the waveform.'],
 '32':['A conversation before the conversation.','Two instances of the real modem engine negotiate V.32 at up to 9,600 bit/s, train their receivers, and exchange your message and an acknowledgement over G.711.'],
 '34':['An analogue conversation, both ways.','Two complete V.34 engines probe the line, negotiate their symbol and bit rates, train, and exchange your text using QAM in both directions. Automatic rate selection reaches 33,600 bit/s on this virtual line.'],
 '90':['A digital end. An analogue end.','V.90 sends PCM downstream and QAM upstream. The calling modem runs the analogue role; the answering modem runs the digital role. Both train and recover real payload bytes.'],
};
const isPcm=()=>['x2','flex'].includes(mode);
const isFsk=()=>mode==='v21'||mode==='bell103';
const isAnalogue=()=>['32','34','v22','v22bis','v32bis','v23','bell212'].includes(mode);
const isLegacy=()=>['v23','bell212'].includes(mode);
const optionalV8=()=>['32','v22','v22bis','v32bis'].includes(mode);
const hasConstellation=()=>!isFsk()&&!isPcm()&&mode!=='v23'&&mode!=='91';
function updateStartup(){
 updateReliabilityControls();
 const fsk=isFsk();if(!optionalV8())$('startupV8').checked=true;
 if(isPcm()||isLegacy()){$('startupV8').checked=false;$('startupBis').checked=false;}
 $('startupV8').disabled=isPcm()||isLegacy()||fsk||!optionalV8()||playing;
 if(!$('startupV8').checked)$('startupBis').checked=false;
 $('startupBis').disabled=isPcm()||isLegacy()||fsk||!$('startupV8').checked||playing;
 $('v92Control').hidden=mode!=='92';$('v92Startup').disabled=mode!=='92'||playing;
 $('lineControls').hidden=fsk||isPcm();
 const analogue=isAnalogue();
 for(const id of ['lineNoise','lineLoss','lineClock'])$(id).disabled=!analogue||playing;
 $('lineDelay').disabled=fsk||isPcm()||playing;
 $('lineHint').textContent=mode==='bell212'?'Bell 212A passes on the clean 0 ms line. Its LAPM connection currently fails with 20 ms added delay. Noise, attenuation and clock drift affect only the receive waveform.':analogue?'Noise, attenuation and clock drift affect the analogue receive waveform. The G.711 bearer stays byte-exact.':'PCM modes preserve codewords. Only propagation delay is available here.';
 $('v34RateControl').hidden=mode!=='34';$('v34Rate').disabled=mode!=='34'||playing;
 $('startupHint').textContent=isLegacy()?'Legacy startup; V.8 and V.8bis are not used.':isPcm()?'Fixed downstream profile; V.8/V.8bis and line training are not run.':fsk?'FSK presets begin with an idle carrier.':optionalV8()?($('startupV8').checked?'V.8 negotiates the modulation; V.8bis adds an earlier capabilities exchange.':'V.25 answer tone and legacy startup; no V.8 menus.'):'V.8 is required for this mode; optionally prepend V.8bis.';
}
function updateCount(){const n=new TextEncoder().encode($('message').value).length;$('byteCount').textContent=n;$('dial').disabled=n===0||n>240||playing;}
function select(next){stop();mode=next;document.querySelectorAll('.mode').forEach(b=>{b.classList.toggle('active',b.dataset.mode===mode);b.setAttribute('aria-pressed',String(b.dataset.mode===mode));});
 $('modeTitle').textContent=descriptions[mode][0];$('modeDescription').textContent=descriptions[mode][1];
 updateStartup();$('engineBadge').textContent=isFsk()?'JAVASCRIPT · LIVE DSP':isPcm()?'WEBASSEMBLY · PCM PAYLOAD':'WEBASSEMBLY · REAL ENGINE';
 $('showConstellation').disabled=!hasConstellation();setPlot(hasConstellation()?'constellation':'spectrum');
 $('inspectSide').disabled=isPcm();$('inspectStage').disabled=isPcm();
 $('inspectorFeatures').textContent=isPcm()?'Fixed profile · PCM codewords':mode==='v23'?'Legacy startup · asynchronous characters':isLegacy()?'Bell answer tone · LAPM':['v22','v22bis','v32bis','32'].includes(mode)?'Startup · training · LAPM':mode==='91'?'V.8 · DIL · PCM · LAPM':'V.8 · INFO · Ja / DIL · LAPM';
 $('dial').textContent=isPcm()?'▶ Encode & send':'▶ Dial & send';
 $('trafficLabel').textContent=isPcm()?'SERVER → PCM RECEIVER':'CALLER → ANSWERER';
 $('scopeLabel').textContent=isPcm()?'SERVER PCM WAVEFORM · 8 kHz':'CALLER WAVEFORM · 8 kHz';
 $('noise').disabled=!isFsk();$('noiseHint').textContent=isFsk()?'Noise is added to the FSK signal before the receiver sees it.':isPcm()?'The payload demo uses exact codewords and a fixed shared profile.':'Open Line impairments to change the simulated channel.';
 $('received').textContent='';$('verification').textContent='AWAITING SIGNAL';$('status').textContent='Ready to dial';$('events').textContent='The receiver listens to the generated samples.\nNo prerecorded audio. No text shortcut.';
 $('transferStats').hidden=true;$('quickComparison').hidden=true;
 elapsed=0;lastInspect=-1;$('exportCapture').disabled=true;$('inspectOutput').textContent='No capture yet.';
 $('clock').textContent='00.00 s';$('duration').textContent='—';result=null;$('download').disabled=true;draw(0);
}
function stop(){
 reliabilityToken++;
 if(reliabilityRunning){
  reliabilityRunning=false;$('status').textContent='Reliability test stopped';
  if(reliabilityResult)reliabilityResult={...reliabilityResult,status:'stopped',activeCallUngraded:true};
  $('reliabilityStatus').textContent=`Stopped · ${reliabilityResult?.completedCalls||0} calls completed. The active call was not graded.`;
 }
 worker?.terminate();worker=null;if(source){source.onended=null;try{source.stop();}catch{}source=null;}
 playing=false;cancelAnimationFrame(animation);$('stop').disabled=true;$('lamp').classList.remove('on');document.querySelectorAll('.mode').forEach(b=>b.disabled=false);$('message').disabled=false;updateCount();updateStartup();
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
function drawInspector(t,force=false){
 if(!$('inspector').open)return;
 if($('inspectFollow').checked)$('inspectTime').value=String(t);else t=Number($('inspectTime').value);
 $('inspectTimeValue').textContent=t.toFixed(2)+' s';
 if(!force&&Math.abs(t-lastInspect)<.15)return;lastInspect=t;
 $('inspectOutput').textContent=result?.protocol?inspectorText(result,t,$('inspectView').value,$('inspectSide').value,$('inspectStage').value):isFsk()?'This FSK demo sends an idle carrier and asynchronous characters. It has no V.8, INFO, Ja/DIL, or LAPM negotiation.':'No capture yet.';
 $('inspectHint').textContent=isPcm()?'Downstream payload only. The fixed profile is shared with an independent receiver; no handshake or upstream is simulated.':`Endpoint names identify the observing modem; “peer” means the other modem. Times are sample-clock frame timestamps (20 ms resolution).${$('inspectStage').value==='pcm'?' Ja describes the requested learning sequence; DIL is the PCM learning waveform sent in response.':''}${result?.droppedProtocol?' Log limit reached: '+result.droppedProtocol+' lines omitted.':''}`;
 $('inspectOutput').scrollTop=$('inspectOutput').scrollHeight;
}
function drawStats(t){
 const s=result?.stats;if(!s){$('transferStats').hidden=true;return;}
 $('transferStats').hidden=false;
 const connected=s.connectSeconds!==null&&t>=s.connectSeconds,verified=result.passed&&s.verifiedAtSeconds!==null&&t>=s.verifiedAtSeconds;
 $('statConnect').textContent=connected?s.connectSeconds.toFixed(2)+' s':result.kind==='pcm'||isFsk()?'Not applicable':t>=(s.simulationSeconds||Infinity)-.02?'Not established':'Waiting';
 $('statRateLabel').textContent=['V90','V92','V23C'].includes(result.modulation)?'Caller upload / download':'Caller TX / RX';
 $('statRates').textContent=connected?`${result.txRate.toLocaleString()} / ${result.rxRate.toLocaleString()} bit/s`:result.kind==='pcm'?'56,000 downstream':isFsk()?'300 bit/s':'—';
 $('statBytes').textContent=verified?s.verifiedBytes.toLocaleString():'0';
 $('statThroughput').textContent=verified&&s.usefulBitsPerSecond!==null?Math.round(s.usefulBitsPerSecond).toLocaleString()+' bit/s':'—';
 $('statRetrains').textContent=String((result.protocol||[]).filter(e=>e.time<=t&&/\[ME\].*(?:asking for.*retrain|retrain requested|requesting.*retrain)/i.test(e.line)).length);
 $('statCrc').textContent=s.invalidHdlcFrames===null?'Not applicable':String((result.negotiation?.frames||[]).filter(f=>f.time<=t&&!f.valid).length);
 $('statHint').textContent=result.kind==='pcm'?'Downstream payload only. Throughput includes the two-byte length and final frame padding.':isFsk()?'Application bytes verified from audio. Throughput includes the idle carrier.':`Negotiated ${connected?(result.modulation||'carrier'):'carrier pending'}. Simulation clock; useful throughput combines both directions and includes application framing and acknowledgement time. Retrain requests and CRC failures count captured events, not inferred bit errors.`;
 const c=result.comparison,comparisonReady=c&&t>=(s.verifiedAtSeconds??s.simulationSeconds);$('quickComparison').hidden=!comparisonReady;
 if(comparisonReady)$('quickComparison').textContent=c.full.passed&&c.quick.passed?`Full startup ${c.full.connectSeconds.toFixed(2)} s · Quick Connect ${c.quick.connectSeconds.toFixed(2)} s · ${Math.abs(c.savedSeconds).toFixed(2)} s ${c.savedSeconds>=0?'saved':'longer'}. Two separate simulated calls; no stored real-line profile.`:'One comparison call did not verify its payload; no time saving is claimed.';
}
function draw(t){drawStats(t);drawInspector(t);const pos=Math.round(t*RATE);plot($('scope'),result?.left,pos);if(plotType==='spectrum')plot($('spectrum'),result?.left,pos,true);else plotConstellation(t);$('clock').textContent=t.toFixed(2).padStart(5,'0')+' s';}
function tick(){if(!playing)return;elapsed=Math.min(context.currentTime-started,result.left.length/RATE);draw(elapsed);showEvents(elapsed);animation=requestAnimationFrame(tick);}
async function play(){
 $('scopeLabel').textContent=result.waveform==='received'?'CALLER → ANSWERER · RECEIVED WAVEFORM · 8 kHz':isPcm()?'SERVER PCM WAVEFORM · 8 kHz':'CALLER WAVEFORM · 8 kHz';
 const buffer=context.createBuffer(result.right?2:1,result.left.length,RATE);buffer.copyToChannel(result.left,0);if(result.right)buffer.copyToChannel(result.right,1);
 source=context.createBufferSource();source.buffer=buffer;const gain=context.createGain();gain.gain.value=.65;source.connect(gain).connect(context.destination);
 source.onended=()=>{elapsed=result.left.length/RATE;draw(elapsed);showEvents(elapsed);stop();$('status').textContent=result.passed?'Complete · payload verified':result.error||'Run ended without a verified payload';};
 started=context.currentTime;source.start();$('lamp').classList.add('on');$('status').textContent='Sending carrier';tick();
 $('duration').textContent=(result.left.length/RATE).toFixed(2)+' s';$('download').disabled=false;$('exportCapture').disabled=!result.protocol;$('inspectTime').max=String(result.left.length/RATE);drawInspector(elapsed,true);
}
async function dial(){
 stop();const text=$('message').value;if(!text||new TextEncoder().encode(text).length>240)return;
 try{context??=new AudioContext();await context.resume();result=null;$('transferStats').hidden=true;$('inspectFollow').checked=true;$('inspectTime').value='0';$('inspectTime').max='0';$('inspectOutput').textContent='Generating a new capture…';playing=true;updateStartup();elapsed=0;lastInspect=-1;$('exportCapture').disabled=true;updateCount();$('stop').disabled=false;$('message').disabled=true;document.querySelectorAll('.mode').forEach(b=>b.disabled=true);
 $('received').textContent='';$('verification').textContent='AWAITING SIGNAL';$('events').textContent='';$('download').disabled=true;
 if(isFsk()){
   const snrs=[99,24,12,0],tx=transmit(text,mode,snrs[Number($('noise').value)]),rx=receive(tx.samples,mode);
   const passed=rx.text===text,n=new TextEncoder().encode(text).length;const events=[{time:0,message:'Idle carrier · acquiring 300 bit/s FSK'}];
   rx.frames.filter(f=>f.stop).forEach((f,i)=>events.push({time:(f.sample+10*RATE/300)/RATE,received:new TextDecoder().decode(new Uint8Array(rx.bytes.slice(0,i+1)))}));
   events.push({time:tx.samples.length/RATE-.05,message:passed?'Payload verified':'Payload mismatch · try a cleaner line'});
   result={left:tx.samples,events,passed,stats:{connectSeconds:null,verifiedAtSeconds:tx.samples.length/RATE-.05,verifiedBytes:passed?n:0,usefulBitsPerSecond:passed?n*8/(tx.samples.length/RATE):null,retrainRequests:0,invalidHdlcFrames:null}};await play();
 }else{
   $('status').textContent='Loading the WebAssembly modems…';
   worker=new Worker('./worker.mjs',{type:'module'});
   worker.onerror=e=>fail(e.message||'Could not load the modem worker.');
   worker.onmessage=async({data})=>{if(data.type==='progress'){$('status').textContent=`${data.state||'Generating call'} · ${data.time.toFixed(1)} s simulated`;}else if(data.type==='error'){fail(data.message);}else if(data.type==='result'){worker.terminate();worker=null;result=data;await play();}};
   worker.postMessage({mode,text,startup:{v8:$('startupV8').checked,v8bis:$('startupBis').checked,v34MaxRate:mode==='34'?Number($('v34Rate').value):0,v92Startup:$('v92Startup').value,line:{noiseDb:isAnalogue()?Number($('lineNoise').value):0,lossDb:isAnalogue()?Number($('lineLoss').value):0,clockPpm:isAnalogue()?Number($('lineClock').value):0,delayMs:isPcm()?0:Number($('lineDelay').value)}}});
 }
 }catch(e){fail(e.message);}
}
function updateReliabilityControls(){
 const supported=['34','90','92'].includes(mode);
 $('reliabilityPanel').hidden=!supported;
 for(const id of ['reliabilityScope','reliabilityRepeats','reliabilityDuration','reliabilityV92'])$(id).disabled=playing;
 const incompatible=$('reliabilityScope').value==='all'&&mode==='34'&&['lineNoise','lineLoss','lineClock'].some(id=>Number($(id).value)!==0);
 $('reliabilityStart').disabled=!supported||playing||incompatible;
 $('reliabilityHint').textContent=incompatible?'For a fair three-mode comparison, turn noise off, set attenuation to 0 dB and use synchronous clocks. Or test the current modem only.':'Uses the selected line settings, V.8bis and V.34 ceiling. Each call starts fresh; V.92 uses the startup selected here. Stop keeps completed results.';
 $('reliabilityExport').disabled=!reliabilityResult?.completedCalls;
}
function renderReliability(summary){
 reliabilityResult=summary;$('reliabilityTable').hidden=false;
 const failures=summary.calls.filter(c=>!c.passed);$('reliabilityFailures').hidden=!failures.length;$('reliabilityFailures').textContent=failures.map(c=>`V.${c.mode} call ${c.attempt}: ${c.failureReason}`).join('\n');
 for(const mode of [34,90,92]){
  const row=summary.rows.find(r=>r.mode===mode);$('reliabilityRow'+mode).hidden=!row;if(!row)continue;
  const values={Completed:`${row.completed}/${summary.repeats}`,Connected:`${row.connected}/${row.completed}`,Passed:`${row.passed}/${row.completed}`,
   Bytes:row.verifiedBytes.toLocaleString(),Mismatches:row.mismatchBytes.toLocaleString(),Retrains:String(row.retrainRequests),Drops:String(row.disconnects),
   Connect:row.meanConnectSeconds===null?'—':row.meanConnectSeconds.toFixed(2)+' s',Throughput:row.meanUsefulBitsPerSecond===null?'—':Math.round(row.meanUsefulBitsPerSecond).toLocaleString()+' bit/s',Rates:row.rates.join(', ')||'—'};
  for(const [key,value] of Object.entries(values))$('reliability'+mode+key).textContent=value;
 }
 $('reliabilityExport').disabled=!summary.completedCalls;
}
async function startReliability(){
 stop();result=null;$('transferStats').hidden=true;$('quickComparison').hidden=true;
 const settings={modes:$('reliabilityScope').value==='all'?[34,90,92]:[Number(mode)],repeats:Number($('reliabilityRepeats').value),durationSeconds:Number($('reliabilityDuration').value),
  startup:{v8:true,v8bis:$('startupBis').checked,v34MaxRate:Number($('v34Rate').value),v92Startup:$('reliabilityV92').value,
   line:{noiseDb:mode==='34'?Number($('lineNoise').value):0,lossDb:mode==='34'?Number($('lineLoss').value):0,clockPpm:mode==='34'?Number($('lineClock').value):0,delayMs:Number($('lineDelay').value)}}};
 reliabilityResult=null;reliabilityRunning=true;playing=true;updateStartup();updateCount();$('message').disabled=true;$('stop').disabled=false;
 $('reliabilityTable').hidden=true;$('reliabilityFailures').hidden=true;$('reliabilityStatus').textContent='Starting fresh modem instances…';$('status').textContent='Reliability test running';
 $('download').disabled=true;$('exportCapture').disabled=true;document.querySelectorAll('.mode').forEach(b=>b.disabled=true);
 const token=reliabilityToken;
 try{
  worker=new Worker('./worker.mjs',{type:'module'});
  worker.onerror=e=>{if(token===reliabilityToken){stop();$('reliabilityStatus').textContent='Test stopped: '+(e.message||'Worker failed');}};
  worker.onmessage=({data})=>{
   if(token!==reliabilityToken)return;
   if(data.type==='reliability-progress'){
    renderReliability(data.summary);
    $('reliabilityStatus').textContent=`V.${data.mode} · call ${data.callNumber}/${data.totalCalls} · ${data.state} · ${data.time.toFixed(1)} s simulated`;
   }else if(data.type==='reliability-result'){
    reliabilityRunning=false;stop();renderReliability(data);
    const passed=data.calls.filter(c=>c.passed).length;
    $('reliabilityStatus').textContent=`Complete · ${data.completedCalls}/${data.totalCalls} calls finished · ${passed} passed`;
    $('status').textContent='Reliability test complete';
   }else if(data.type==='error'){stop();$('reliabilityStatus').textContent='Test stopped: '+data.message;}
  };
  worker.postMessage({task:'reliability',settings});
 }catch(e){stop();$('reliabilityStatus').textContent='Test stopped: '+e.message;}
}
$('reliabilityStart').addEventListener('click',startReliability);
for(const id of ['reliabilityScope','lineNoise','lineLoss','lineClock'])$(id).addEventListener('change',updateReliabilityControls);
$('reliabilityExport').addEventListener('click',()=>{
 if(!reliabilityResult?.completedCalls)return;if(reliabilityUrl)URL.revokeObjectURL(reliabilityUrl);
 reliabilityUrl=URL.createObjectURL(new Blob([JSON.stringify(reliabilityResult,null,2)],{type:'application/json'}));
 const a=document.createElement('a');a.href=reliabilityUrl;a.download='modem-reliability-results.json';a.click();
});
function fail(message){stop();$('status').textContent='Could not run: '+message;$('events').textContent='The run stopped before a verified result.\n'+message;}
$('inspectTime').addEventListener('input',()=>{$('inspectFollow').checked=false;drawInspector(elapsed,true);});$('inspectFollow').addEventListener('change',()=>drawInspector(elapsed,true));
$('inspector').addEventListener('toggle',()=>drawInspector(elapsed,true));for(const id of ['inspectView','inspectSide','inspectStage'])$(id).addEventListener('change',()=>drawInspector(elapsed,true));
$('exportCapture').addEventListener('click',()=>{if(!result?.protocol)return;if(captureUrl)URL.revokeObjectURL(captureUrl);const {left,right,constellations,logs,...capture}=result;capture.coverage=result.kind==='pcm'?'Downstream payload only: fixed profile, exact server PCM codewords, independent decoded UTF-8 text; no dial-up handshake or upstream.':'Engine protocol logs, V.8 menu octets, logged packed INFO frames, full demodulated data-layer TX/RX bits, decoded HDLC frames, and exact G.711 DS0 codewords (hex). Not a complete demodulated startup bitstream.';captureUrl=URL.createObjectURL(new Blob([JSON.stringify(capture,null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=captureUrl;a.download=`modem-${mode}-negotiation.json`;a.click();});
$('showSpectrum').addEventListener('click',()=>{setPlot('spectrum');draw(elapsed);});$('showConstellation').addEventListener('click',()=>{setPlot('constellation');draw(elapsed);});
$('startupV8').addEventListener('change',updateStartup);$('startupBis').addEventListener('change',updateStartup);
$('dial').addEventListener('click',dial);$('stop').addEventListener('click',stop);$('message').addEventListener('input',updateCount);
$('noise').addEventListener('input',()=>{$('noiseValue').textContent=['Clean','24 dB SNR','12 dB SNR','0 dB SNR'][$('noise').value];});
document.querySelectorAll('.mode').forEach(b=>b.addEventListener('click',()=>select(b.dataset.mode)));
$('download').addEventListener('click',()=>{if(!result)return;if(downloadUrl)URL.revokeObjectURL(downloadUrl);downloadUrl=URL.createObjectURL(new Blob([wav(result.left,result.right)],{type:'audio/wav'}));const a=document.createElement('a');a.href=downloadUrl;a.download=`modem-${mode}.wav`;a.click();});
window.addEventListener('pagehide',()=>{stop();if(reliabilityUrl)URL.revokeObjectURL(reliabilityUrl);context?.close();if(downloadUrl)URL.revokeObjectURL(downloadUrl);if(captureUrl)URL.revokeObjectURL(captureUrl);});
updateCount();updateStartup();draw(0);
