#!/usr/bin/env python3
"""Clock an original Courier endpoint against the actual v90modem engine.

Byte-exact DS0 feedback, real engine PTY and original I-modem firmware.
Checks both CONNECT reports, bearer law and complete bidirectional DTE text.
A-law uses the native fixed PCMU startup alphabet without transcoding.
The NET3 test network releases a priming ring to establish its data link.
Use --fast for offline clocked runs; no recording supplies peer responses.
"""
import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import select
import struct
import subprocess
import sys
import tempfile
import time
ROOT = Path(__file__).resolve().parents[1].parent / "courier-emu"
sys.path.insert(0, str(ROOT))
os.environ.setdefault('COURIER_LINE_FRAME_MS', '20')


class EnginePeer:
    def __init__(self, output, engine, fast=False):
        self.output, self.engine, self.fast = output, engine, fast
        self.process = None
        self.samples = 0
        self.frames = 0
        self.sent_non_ff = self.received_non_ff = 0
        self.error = None
        self.log = self.rx = self.tx = None

    def start(self):
        if self.process is not None: return
        self.log = (self.output/'engine.log').open('wb')
        self.rx = (self.output/'engine-rx.g711').open('wb')
        self.tx = (self.output/'engine-tx.g711').open('wb')
        env = os.environ.copy(); env['ME_MODE'] = 'x2-symm'; env['ME_DATA_FRAMING']=os.environ.get('ME_DATA_FRAMING','lapm')
        self.environment = {k:v for k,v in env.items() if k.startswith('ME_')}
        self.engine_sha256 = hashlib.sha256(self.engine.read_bytes()).hexdigest()
        self.pty='/private/tmp/'+self.output.name+'-pty'
        self.dte=None; self.dte_text=bytearray(); self.payload_sent=False
        self.process = subprocess.Popen([str(self.engine), self.pty]+(['--alaw'] if os.environ.get('X2_SYM_LAW')=='alaw' else [])+(['--call'] if os.environ.get('X2_SYM_CALL')=='1' else []),
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log, env=env)
        self.origin = time.monotonic()

    def exchange(self, octets):
        self.start()
        if not octets: return b''
        assert len(octets) <= 4096
        if not self.fast:
            delay = self.origin + self.samples/8000 - time.monotonic()
            if delay > 0: time.sleep(delay)
        self.rx.write(octets)
        self.process.stdin.write(struct.pack('<H',len(octets))+octets)
        self.process.stdin.flush()
        reply = bytearray()
        while len(reply) < len(octets):
            if not select.select([self.process.stdout],[],[],30)[0]:
                raise TimeoutError('engine frame')
            chunk = self.process.stdout.read(len(octets)-len(reply))
            if not chunk: raise EOFError('engine closed')
            reply.extend(chunk)
        if self.dte is None:
            self.dte=os.open(self.pty,os.O_RDWR|os.O_NONBLOCK|os.O_NOCTTY)
        while True:
            try:
                chunk=os.read(self.dte,4096)
                if not chunk:break
                self.dte_text.extend(chunk)
            except BlockingIOError:break
        if b'CONNECT' in self.dte_text and not self.payload_sent:
            os.write(self.dte,b'ENGINE-X2-SYMMETRIC\r\n');self.payload_sent=True
        self.tx.write(reply)
        self.samples += len(octets); self.frames += 1
        self.sent_non_ff += sum(x!=255 for x in octets)
        self.received_non_ff += sum(x!=255 for x in reply)
        return bytes(reply)

    def stop(self):
        if self.process:
            if not self.process.stdin.closed: self.process.stdin.close()
            try: self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait()
            for f in (self.log,self.rx,self.tx): f.close()
            if self.dte is not None:os.close(self.dte);self.dte=None
            (self.output/'engine-dte.bin').write_bytes(self.dte_text)

    def status(self):
        log_path=self.output/'engine.log'
        transcript=log_path.read_text() if log_path.exists() else ''
        return {'frames':self.frames,'octets_sent':self.samples,'octets_received':self.samples,
                'sent_non_ff':self.sent_non_ff,'received_non_ff':self.received_non_ff,
                'error':self.error,'wall_clock_paced':not self.fast,
                'environment':getattr(self,'environment',{}),
                'engine_sha256':getattr(self,'engine_sha256',None),
                'stages':re.findall(r'X2 stage=(\w+)',transcript),
                'accepted_markers':re.findall(r'X2 accepted marker=([0-9a-f]+)',transcript),
                'engine_exit':self.process.returncode if self.process else None, 'dte_text':getattr(self,'dte_text',b'').decode('latin1'), 'payload_sent':getattr(self,'payload_sent',False)}


def imodem(args):
    from tools import probe_imodem_pair as pair
    peer=EnginePeer(args.output,args.engine,args.fast)
    pair.G711Peer=lambda path,listen:peer
    if args.law=='alaw':
        from courier_emu.bri import audio_bearer, LAW_A
        pair.audio_bearer=lambda capability:audio_bearer(capability,LAW_A)
    saved=sys.argv
    sys.argv=['closed-loop','--worker','answer' if args.engine_call else 'originate','--socket','unused',
        '--result',str(args.output/'imodem-result.json'),'--protocol','x2',
        '--instructions',str(args.instructions),'--settings',args.imodem_settings,
        '--nvram',str(args.nvram),'--trace-negotiation','--answer-send' if args.engine_call else '--originate-send','IMODEM-X2-SYMMETRIC\r\n','--establish',args.establish]+(['--prime-originate'] if args.prime_originate else [])
    original_machine=pair.IsdnMachine
    trace={'addresses':['039f','03e2','f6a0','f6a1','f6ba','fef0','fef1','ffdc','ffde','036d','03ed','039a','03cf','03db','ffd9','0322','02b2'],
           'samples':[],'writes':[],'gate_captures':[],'gate_instructions':[]}
    seen=None;next_at=0;dumped=False;prime_cleared=False
    def machine(*a,**kw):
        prior=kw['serial_pump']
        def pump(current):
            nonlocal seen,next_at,dumped,prime_cleared
            if args.prime_originate and not prime_cleared and current.instructions>=25_000_000:
                from courier_emu.bri import RELEASE, q931_message
                bri=current.bri
                if bri.call_reference is not None:
                    bri._send_layer3(bri._call_link_tei(),q931_message(RELEASE,bri.call_reference,True))
                    prime_cleared=True
            prior(current)
            core=current.mailbox.core
            if core is None:return
            if core is not seen:
                core.set_data_trace_range(0x039f,0x039f) if args.classifier else core.set_data_trace_range(0xf6a0,0xf6c0)
                core.set_pc_capture(0xde14 if args.classifier else 0x9596,[int(a,16) for a in trace['addresses']])
                core.set_pc_trace_range(0xddf6,0xde1b) if args.classifier else core.set_pc_trace_range(0x958f,0x965c)
                core.set_data_event_limit(2000)
                core.trace_data_writes(clear=True)
                seen=core
            if current.instructions<next_at:return
            next_at=current.instructions+250000
            trace['gate_instructions'].extend(core.pc_trace())
            core.clear_pc_trace()
            trace['gate_captures'].extend(core.pc_captures())
            core.clear_pc_captures()
            trace['writes'].extend(core.data_events())
            core.trace_data_writes(clear=True)
            trace['samples'].append({'instructions':current.instructions,
                'engine_samples':peer.samples,'state':core.state(),
                'cells':{a:core.data(int(a,16)) for a in trace['addresses']}})
            if not dumped and current.instructions>80000000:
                (args.output/'native-program.bin').write_bytes(struct.pack('<65536H',*(core.program(i) for i in range(65536))))
                dumped=True
        kw['serial_pump']=pump
        return original_machine(*a,**kw)
    pair.IsdnMachine=machine
    try:
        options=pair.arguments();pair.run_side(options)
    finally:
        sys.argv=saved;peer.stop();pair.IsdnMachine=original_machine
        (args.output/'native-control.json').write_text(json.dumps(trace,indent=2)+'\n')
    native=json.loads((args.output/'imodem-result.json').read_text())
    engine=peer.status()
    engine_text=engine['dte_text']
    native_text=native.get('serial_a','')
    checks={
        'engine_connect_64000':'CONNECT 64000' in engine_text,
        'native_connect_64000_x2':'CONNECT 64000/ARQ/x2/LAPM' in native_text,
        'native_to_engine':'IMODEM-X2-SYMMETRIC\r\n' in engine_text,
        'engine_to_native':'ENGINE-X2-SYMMETRIC\r\n' in native_text,
        'native_8n1':native.get('dte_framing')=='8N1',
        'engine_exit_ok':engine['engine_exit']==0,
        'native_no_error':native.get('error') is None and native['mailbox'].get('error') is None,
        'native_no_pcm_underruns':native['mailbox']['pcm']['rx_empty_frames']==0,
    }
    if not args.engine_call:
        expected='90 90 a3' if args.law=='alaw' else '90 90 a2'
        checks['native_requested_bearer']=(native['bri'].get('outbound_setup') or {}).get('bearer_capability')==expected
    report={'type':'imodem','law':args.law,'engine':engine,
        'nvram':str(args.nvram),'nvram_sha256':hashlib.sha256(args.nvram.read_bytes()).hexdigest(),
        'settings':options.settings,'instructions':args.instructions,'checks':checks}
    (args.output/'call.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'law':args.law,'checks':checks,'passed':all(checks.values())},indent=2))
    if not all(checks.values()):raise SystemExit('symmetric native interoperability check failed')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('type',choices=['imodem']);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--engine',type=Path,default=ROOT.parent/'v90modem/v90_engine_peer')
    p.add_argument('--nvram',type=Path,default=ROOT/'artifacts/imodem-pair-x2-full-rate-routed-20261002/nvram-230400-switch2.sav')
    p.add_argument('--instructions',type=int,default=300_000_000);p.add_argument('--fast',action='store_true')
    p.add_argument('--classifier',action='store_true',help='trace the native PCM classifier instead of the INFO0 gate')
    p.add_argument('--imodem-settings',default='S54=0S58=48&A3&B1Q0',
                   help='disable I-modem server (2), symmetric (8), and V.90 (32); retain constellation option 16')
    p.add_argument('--require-marker',action='store_true',help='fail unless the live engine accepts the supported 4d x2 marker')
    p.add_argument('--law',choices=['ulaw','alaw'],default='ulaw')
    p.add_argument('--establish',choices=['terminal','network'],default='terminal')
    p.add_argument('--prime-originate',action='store_true')
    p.add_argument('--engine-call',action='store_true')
    args=p.parse_args();os.environ['X2_SYM_LAW']=args.law;os.environ['X2_SYM_CALL']='1' if args.engine_call else '0';args.output=args.output.resolve()
    if args.law=='alaw':
        args.imodem_settings=args.imodem_settings.replace('S58=48','S58=52')
        # Ie030002 uses PCMU control codewords even when its Q.931
        # SETUP requests A3. Select that native alphabet, no transcoding.
        os.environ['ME_X2_CONTROL_LAW']='ulaw'
        if not args.engine_call:
            args.establish='network';args.prime_originate=True
    args.output.mkdir(parents=True,exist_ok=False)
    # A derivative sealed test profile, never a write to the source NVRAM.
    from courier_emu import imodem_config as config
    source=args.nvram.read_bytes()
    profile=bytearray(config.set_dte_framing(source,0))
    protocol=4 if args.law=='alaw' else 2
    for pos in range(0,len(profile),config.SECTOR_SIZE):
        sector=bytes(profile[pos:pos+config.SECTOR_SIZE])
        if any(config.page_is_sealed(sector[p:p+config.PAGE_SIZE])
               for p in range(0,len(sector),config.PAGE_SIZE)):
            profile[pos:pos+config.SECTOR_SIZE]=config.seal(config.set_switch_protocol(sector,protocol))
    args.nvram=args.output/'native-profile-8n1.sav'
    args.nvram.write_bytes(profile)
    (args.output/'profile.json').write_text(json.dumps({'source_sha256':hashlib.sha256(source).hexdigest(),
        'switch_protocol':protocol,'dte_framing':'8N1','no_firmware_modification':True},indent=2)+'\n')
    imodem(args)
    if args.require_marker:
        result=json.loads((args.output/'call.json').read_text())
        if '4d' not in result['engine']['accepted_markers']:
            raise SystemExit('live peer did not negotiate the supported x2 marker')
if __name__=='__main__':main()
