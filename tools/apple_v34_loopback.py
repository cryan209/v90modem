#!/usr/bin/env python3
"""V.34 §§10.1/11 training and PRBS payload through one Apple hardware leg.

The opposite direction is the normal software G.711 harness bearer. This is
not a two-line hardware interoperability test. No decoder state crosses the
socket: only 8000 Hz s16le samples, paced by actual CoreAudio receive samples.
"""
import argparse
import json
import os
import re
from pathlib import Path
import signal
import socket
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['digital','isolation','analog'],default='digital')
    p.add_argument('--address',required=True);p.add_argument('--uid',required=True)
    p.add_argument('--baud',type=int,default=2400);p.add_argument('--bps',type=int,default=9600)
    p.add_argument('--direction',choices=['caller','answer'],default='caller',help='Which transmitter crosses the hardware leg')
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--quiet-pbx',help='Optional SSH host for a bounded quiet incoming lab call')
    p.add_argument('--extension',help='Lab extension for that incoming call')
    args=p.parse_args()
    if bool(args.quiet_pbx)!=bool(args.extension):p.error('--quiet-pbx and --extension belong together')
    args.out=args.out.resolve();args.out.mkdir(parents=True,exist_ok=False)
    env=dict(os.environ,APPLE_MODEM_ADDR=args.address,APPLE_MODEM_AUDIO_UID=args.uid,
        APPLE_MODEM_RX_TAP=str(args.out/'rx.s16'),APPLE_MODEM_TX_TAP=str(args.out/'tx.s16'))
    if args.quiet_pbx:
        body=f'Channel: PJSIP/{args.extension}\nCallerid: Apple V34 Loopback <6002>\nMaxRetries: 0\nWaitTime: 12\nApplication: Wait\nData: 75\nArchive: no\n'
        remote="import os,tempfile\nbody="+repr(body)+"\nfd,path=tempfile.mkstemp(prefix='apple-v34-',suffix='.call',dir='/var/spool/asterisk')\nwith os.fdopen(fd,'w') as f:f.write(body)\nos.chmod(path,0o644);os.replace(path,'/var/spool/asterisk/outgoing/'+os.path.basename(path))\n"
        subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=6',args.quiet_pbx,'python3 -'],input=remote,text=True,check=True,timeout=12)
        time.sleep(2)
    a,b=socket.socketpair();peer=hardware=None;timed_out=False
    with (args.out/'hardware.log').open('w') as hwlog,(args.out/'modem.log').open('w') as modemlog:
        try:
            hardware=subprocess.Popen([str(ROOT/'apple_usb_modem_coupler'),'--rate','8000','--loopback-test',args.mode,
                '--loopback-peer-fd',str(a.fileno()),'--line-events',str(args.out/'events.jsonl')],
                env=env,pass_fds=[a.fileno()],stdout=subprocess.PIPE,stderr=hwlog,text=True)
            peer_env=dict(os.environ,V34_DUPLEX_HARDWARE_FD=str(b.fileno()),V34_DUPLEX_HARDWARE_DIRECTION=args.direction)
            peer=subprocess.Popen([str(ROOT/'v34_duplex_test'),str(args.baud),str(args.bps)],env=peer_env,
                pass_fds=[b.fileno()],stdout=modemlog,stderr=modemlog)
            a.close();b.close()
            try:peer.wait(timeout=75)
            except subprocess.TimeoutExpired:timed_out=True;peer.terminate();peer.wait(timeout=5)
            try:output,_=hardware.communicate(timeout=12)
            except subprocess.TimeoutExpired:
                timed_out=True;hardware.send_signal(signal.SIGINT);output,_=hardware.communicate(timeout=10)
        finally:
            a.close();b.close()
            for child in [peer,hardware]:
                if child and child.poll() is None:child.send_signal(signal.SIGINT);child.wait(timeout=10)
    try:transport=json.loads(output)
    except (ValueError,UnboundLocalError):transport={}
    status=subprocess.run([str(ROOT/'apple_usb_modem_probe'),'--line-status'],env=env,capture_output=True,text=True,timeout=10)
    try:line=json.loads(status.stdout)
    except ValueError:line={}
    modem_text=(args.out/'modem.log').read_text()
    match=re.search(r'trained=(\d+)/(\d+) rx_bits=(\d+)/(\d+) errors=(\d+)/(\d+) time=([\d.]+)s',modem_text)
    modem={} if not match else dict(trained=[int(match[1]),int(match[2])],rx_bits=[int(match[3]),int(match[4])],bit_errors=[int(match[5]),int(match[6])],duration_s=float(match[7]))
    payload_ok=modem.get('trained')==[1,1] and min(modem.get('rx_bits',[0]))>=16000 and modem.get('bit_errors')==[0,0]
    passed=payload_ok and not timed_out and peer.returncode==0 and hardware.returncode==0 and bool(transport.get('restored')) and line.get('hook_register',1)&1==0
    result=dict(passed=passed,mode=args.mode,address=args.address,baud=args.baud,bps=args.bps,direction=args.direction,
        modem=modem,modem_exit=peer.returncode,hardware_exit=hardware.returncode,timed_out=timed_out,transport=transport,line_after=line,
        topology='one hardware waveform leg; opposite direction is software G.711')
    (args.out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    print((args.out/'modem.log').read_text()[-1800:])
    return 0 if passed else 1

if __name__=='__main__':raise SystemExit(main())
