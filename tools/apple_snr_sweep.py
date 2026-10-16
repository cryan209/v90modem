#!/usr/bin/env python3
"""Controlled local analogue-loop tone/gain sweep; Si3056 §§5.29–5.30.

Keeps 12 dB analog-loop TX headroom. Each invocation restores ARX/registers
and hook state. Saved failures remain evidence, never successful measurements.
"""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--quiet-pbx',required=True)
    p.add_argument('--device',choices=['0:13','0:14']);p.add_argument('--check',choices=['low','high','gain9'])
    args=p.parse_args();args.out=args.out.resolve();args.out.mkdir(parents=True,exist_ok=False)
    for addr,uid,extension in [('0:13','145430','6004'),('0:14',':000000:','6005')]:
        if args.device and addr!=args.device:continue
        points=[(.25,0)] if args.check=='low' else [(1,0)] if args.check=='high' else [(1,9)] if args.check=='gain9' else [(.25,0),(.5,0),(1,0),(1,3),(1,6),(1,9),(1,12)]
        for level,gain in points:
            for attempt in range(2):
                d=args.out/f"{addr.replace(':','-')}-level{level}-gain{gain}-try{attempt}";d.mkdir()
                env=dict(os.environ,APPLE_MODEM_ADDR=addr,APPLE_MODEM_AUDIO_UID=uid,
                    APPLE_MODEM_RX_TAP=str(d/'rx.s16'),APPLE_MODEM_TX_TAP=str(d/'tx.s16'))
                body=f'Channel: PJSIP/{extension}\nCallerid: Apple SNR Probe <6002>\nMaxRetries: 0\nWaitTime: 12\nApplication: Wait\nData: 12\nArchive: no\n'
                remote="import os,tempfile\nbody="+repr(body)+"\nfd,path=tempfile.mkstemp(prefix='apple-snr-',suffix='.call',dir='/var/spool/asterisk')\nwith os.fdopen(fd,'w') as f:f.write(body)\nos.chmod(path,0o644);os.replace(path,'/var/spool/asterisk/outgoing/'+os.path.basename(path))\n"
                subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=6',args.quiet_pbx,'python3 -'],input=remote,text=True,check=True,timeout=12)
                time.sleep(2)
                with (d/'log.txt').open('w') as log:
                    proc=subprocess.Popen([str(ROOT/'apple_usb_modem_coupler'),'--rate','10286','--loopback-test','analog',
                        '--loopback-pattern','tone','--loopback-tone-level',str(level),'--loopback-rx-gain',str(gain),
                        '--line-events',str(d/'events.jsonl')],env=env,stdout=subprocess.PIPE,stderr=log,text=True)
                    try: stdout,_=proc.communicate(timeout=25)
                    except subprocess.TimeoutExpired:
                        proc.send_signal(signal.SIGINT);stdout,_=proc.communicate(timeout=8)
                        (d/'timeout.txt').write_text(stdout);raise
                r=json.loads(stdout);r.update(device=addr,exit_code=proc.returncode)
                (d/'result.json').write_text(json.dumps(r,indent=2)+'\n')
                status=json.loads(subprocess.check_output([str(ROOT/'apple_usb_modem_probe'),'--line-status'],env=env,text=True))
                restored=subprocess.check_output([str(ROOT/'apple_usb_modem_probe'),'--regs','0f'],env=env,text=True)
                (d/'after.json').write_text(json.dumps(status,indent=2)+'\n'+restored)
                assert r.get('restored')==1 and not (status['hook_register']&1)
                print(d.name,'exit',proc.returncode,'restored',r['restored'],'reason',r['reason'],flush=True)
                if proc.returncode==0:break
                assert not status['overcurrent'] and not status['receive_overload']
                assert r['reason']=='Test register did not read back' and attempt==0


if __name__=='__main__':main()
