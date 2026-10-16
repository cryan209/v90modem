#!/usr/bin/env python3
"""Bounded standalone Si3056 §5.30 native-rate probes, no engine resampling."""
import argparse,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--quiet-pbx',required=True)
    p.add_argument('--device', choices=['0:13','0:14'])
    p.add_argument('--rates', type=int, nargs='+', choices=[8000,9600,10286], default=[8000,10286,9600])
    args=p.parse_args();args.out=args.out.resolve();args.out.mkdir(parents=True,exist_ok=False)
    for address,uid,extension in [('0:13','145430','6004'),('0:14',':000000:','6005')]:
        if args.device and address != args.device: continue
        for rate in args.rates:
            for mode in ['digital','isolation','analog']:
                d=args.out/(address.replace(':','-')+f'-{rate}-{mode}');d.mkdir()
                env=dict(os.environ,APPLE_MODEM_ADDR=address,APPLE_MODEM_AUDIO_UID=uid,
                    APPLE_MODEM_RX_TAP=str(d/'rx.s16'),APPLE_MODEM_TX_TAP=str(d/'tx.s16'))
                if mode=='analog':
                    body=f'Channel: PJSIP/{extension}\nCallerid: Apple Rate Probe <6002>\nMaxRetries: 0\nWaitTime: 12\nApplication: Wait\nData: 12\nArchive: no\n'
                    remote="import os,tempfile\nbody="+repr(body)+"\nfd,path=tempfile.mkstemp(prefix='apple-rate-',suffix='.call',dir='/var/spool/asterisk')\nwith os.fdopen(fd,'w') as f:f.write(body)\nos.chmod(path,0o644);os.replace(path,'/var/spool/asterisk/outgoing/'+os.path.basename(path))\n"
                    subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=6',args.quiet_pbx,'python3 -'],input=remote,text=True,check=True,timeout=12)
                    time.sleep(2)
                with (d/'log.txt').open('w') as log:
                    run=subprocess.run([str(ROOT/'apple_usb_modem_coupler'),'--rate',str(rate),'--loopback-test',mode,
                        '--loopback-pattern','prbs','--line-events',str(d/'events.jsonl')],
                        env=env,stdout=subprocess.PIPE,stderr=log,text=True,timeout=25)
                try:value=json.loads(run.stdout)
                except json.JSONDecodeError:
                    (d/'stdout.txt').write_text(run.stdout);raise
                value.update(device=address,exit_code=run.returncode)
                (d/'result.json').write_text(json.dumps(value,indent=2)+'\n')
                status=subprocess.check_output([str(ROOT/'apple_usb_modem_probe'),'--line-status'],env=env,text=True)
                (d/'after-line-status.json').write_text(status)
                print(d.name,json.dumps(value),flush=True)
                assert value.get('restored')==1 and value.get('rate_hz')==rate
                assert not (json.loads(status)['hook_register']&1)
                assert run.returncode==0, 'Probe failed; retain evidence and stop.'


if __name__=='__main__':main()
