#!/usr/bin/env python3
"""Local SIP registrar/auth/listener lifecycle check; no physical ATA required."""
import hashlib
import json
import os
from pathlib import Path
import re
import selectors
import struct
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request
import urllib.error


def wait(fn):
    end=time.monotonic()+15
    while time.monotonic()<end:
        value=fn()
        if value: return value
        time.sleep(.1)
    raise AssertionError('ATA GUI timeout')


def main():
    repo=Path(__file__).resolve().parents[1]
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sip:
        sip.bind(('127.0.0.1',0)); port=sip.getsockname()[1]
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as udp, tempfile.TemporaryDirectory(prefix='gui-ata-') as temp:
        udp.bind(('127.0.0.1',0)); udp.settimeout(3); local=udp.getsockname()[1]
        env=dict(os.environ,BROWSER='true',ME_MODE='v22')
        env.pop('ATA_TEST_PASSWORD',None); env.pop('ATA_TEST_USER',None)
        with (Path(temp)/'log').open('wb') as log:
            child=subprocess.Popen([sys.executable,str(repo/'tools/modem_gui.py'),'--web',str(repo/'sip_v90_modem'),'--ata-server','--bind-addr','127.0.0.1','--local-port',str(port),'--rtp-port','44000','--auto-answer','0'],env=env,stdout=subprocess.PIPE,stderr=log)
            try:
                with selectors.DefaultSelector() as sel:
                    sel.register(child.stdout,selectors.EVENT_READ); assert sel.select(20)
                    url=child.stdout.readline().decode().strip().removeprefix('Modem GUI: ')
                assert url.startswith('http://127.0.0.1:'), (Path(temp)/'log').read_text()
                origin='/'.join(url.split('/')[:3])
                def state():
                    with urllib.request.urlopen(url+'state',timeout=2) as r: return json.load(r)
                def action(name,**values):
                    req=urllib.request.Request(url+'ata',json.dumps(dict(action=name,**values)).encode(),headers={'Origin':origin,'Content-Type':'application/json'})
                    with urllib.request.urlopen(req,timeout=10) as r: assert r.status==200
                def ready(): return wait(lambda: s if (s:=state()).get('interfaces_ready') and s.get('ata_registration') is not None else None)
                ready()
                cseq=0
                def request(method='REGISTER',auth='',expires=300,contact=True):
                    nonlocal cseq
                    cseq+=1; uri=f'sip:127.0.0.1:{port}'
                    body=''
                    if method=='INVITE': body=f'v=0\r\no=ata 1 1 IN IP4 127.0.0.1\r\ns=test\r\nc=IN IP4 127.0.0.1\r\nt=0 0\r\nm=audio 44010 RTP/AVP 0\r\na=rtpmap:0 PCMU/8000\r\n'
                    message=f'{method} {uri} SIP/2.0\r\nVia: SIP/2.0/UDP 127.0.0.1:{local};branch=z9hG4bK-test{cseq}\r\nFrom: <sip:ata@127.0.0.1>;tag=test\r\nTo: <sip:ata@127.0.0.1>\r\nCall-ID: test{cseq}@localhost\r\nCSeq: {cseq} {method}\r\nMax-Forwards: 70\r\n'+(f'Contact: <sip:ata@127.0.0.1:{local}>\r\n' if contact else '')+f'Expires: {expires}\r\n'+auth+('Content-Type: application/sdp\r\n' if body else '')+f'Content-Length: {len(body)}\r\n\r\n'+body
                    udp.sendto(message.encode(),('127.0.0.1',port))
                    end=time.monotonic()+5
                    while time.monotonic()<end:
                        data=udp.recv(16384).decode()
                        if f'CSeq: {cseq} {method}' in data and not data.startswith('SIP/2.0 100'): return data
                    raise AssertionError('No SIP response')
                assert request().startswith('SIP/2.0 200')
                wait(lambda: state()['ata_registration']['expires']>0)
                query=request(contact=False);assert query.startswith('SIP/2.0 200') and 'Contact:' in query,query
                assert request(expires=0).startswith('SIP/2.0 200')
                wait(lambda: state()['ata_registration']['expires']==0)
                action('stop'); assert not state()['ata_server']['running']; assert child.poll() is None
                try: action('start',address='0.0.0.0',sip_port=port,rtp_port=44000,rings=0,username='ata',password='secret')
                except urllib.error.HTTPError as e: assert e.code==400
                else: raise AssertionError('Wildcard advertised address accepted')
                action('start',address='127.0.0.1',sip_port=port,rtp_port=44000,rings=0,username='ata',password='secret');ready()
                assert state()['ata_server']['authenticated']
                assert 'secret' not in json.dumps(state())
                challenge=request(); assert challenge.startswith('SIP/2.0 401'),challenge
                realm=re.search(r'realm="([^"]+)"',challenge)[1];nonce=re.search(r'nonce="([^"]+)"',challenge)[1]
                def auth(method,password='secret',value=nonce):
                    md5=lambda x:hashlib.md5(x.encode()).hexdigest()
                    uri=f'sip:127.0.0.1:{port}'
                    response=md5(md5(f'ata:{realm}:{password}')+':'+value+':'+md5(method+':'+uri))
                    return f'Authorization: Digest username="ata", realm="{realm}", nonce="{value}", uri="{uri}", response="{response}", algorithm=MD5\r\n'
                assert request(auth=auth('REGISTER','wrong')).startswith('SIP/2.0 401')
                assert request(auth=auth('REGISTER',value='stale')).startswith('SIP/2.0 401')
                assert request(auth=auth('REGISTER')).startswith('SIP/2.0 200')
                assert request('INVITE').startswith('SIP/2.0 401')
                response=request('INVITE',auth('INVITE')); assert response.startswith('SIP/2.0 180'),response
                action('stop'); assert not state()['ata_server']['running']
                action('start',address='127.0.0.1',sip_port=port,rtp_port=44000,rings=1,username='ata',password='secret');ready()
                challenge=request(); nonce=re.search(r'nonce="([^"]+)"',challenge)[1]
                # Rebind the default parameter to this fresh process's nonce.
                response=request('INVITE',auth('INVITE',value=nonce))
                while not response.startswith('SIP/2.0 200'):
                    response=udp.recv(16384).decode()
                to=re.search(r'^To: (.+)$',response,re.M)[1].strip()
                ack=f'ACK sip:127.0.0.1:{port} SIP/2.0\r\nVia: SIP/2.0/UDP 127.0.0.1:{local};branch=z9hG4bK-ack\r\nFrom: <sip:ata@127.0.0.1>;tag=test\r\nTo: {to}\r\nCall-ID: test{cseq}@localhost\r\nCSeq: {cseq} ACK\r\nMax-Forwards: 70\r\nContent-Length: 0\r\n\r\n'
                udp.sendto(ack.encode(),('127.0.0.1',port))
                media_port=int(re.search(r'm=audio (\d+)',response)[1])
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as rtp:
                    rtp.bind(('127.0.0.1',44010))
                    for seq in range(30):
                        rtp.sendto(struct.pack('!BBHII',0x80,0,seq,seq*160,12345)+b'\xff'*160,('127.0.0.1',media_port));time.sleep(.02)
                wait(lambda: state().get('ata_call',{}).get('rx_packets',0)>0)
                media=state()['ata_call'];assert media['codec']=='PCMU/8000' and media['peer'] and media['tx_packets']>0,media
                action('stop');assert not state()['ata_server']['running']
                print('ATA GUI: registration/unregister, digest challenge/rejection, authenticated incoming call, negotiated G.711/RTP statistics, validation and stop/restart passed')
            except Exception:
                print((Path(temp)/'log').read_text(),file=sys.stderr);raise
            finally:
                child.terminate()
                try: child.wait(timeout=10)
                except subprocess.TimeoutExpired: child.kill(); child.wait()

if __name__=='__main__':main()
