#!/usr/bin/env python3
"""Four-channel Diva modem test menu. No shell is exposed to callers."""
import argparse, array, errno, fcntl, hashlib, json, logging, os, pty, select, signal, socket, subprocess, tempfile, pathlib, pwd, termios, threading, time
from stream_test import run as stream_run
LOG=logging.getLogger('bri-test')
class Disconnected(Exception): pass
class Port:
 def __init__(self, fd): self.fd=fd; self.buf=bytearray(); self.cd_supported=True
 def alive(self):
  if not self.cd_supported:return
  bits=array.array('i',[0])
  try:fcntl.ioctl(self.fd,termios.TIOCMGET,bits,True)
  except OSError as e:
   if e.errno in (errno.ENOTTY,errno.EINVAL):self.cd_supported=False;return
   raise
  if not bits[0]&termios.TIOCM_CAR:raise Disconnected()
 def send(self,data):
  if isinstance(data,str):data=data.encode()
  while data:
   _,w,_=select.select([], [self.fd], [], 1)
   if w:
    try:n=os.write(self.fd,data);data=data[n:]
    except BlockingIOError:pass
 def read(self,n=4096,timeout=1,carrier=True):
  if carrier:self.alive()
  if self.buf:
   data=bytes(self.buf[:n]);del self.buf[:n];return data
  if not select.select([self.fd],[],[],timeout)[0]:return b''
  try:return os.read(self.fd,n)
  except BlockingIOError:return b''
 def line(self,timeout=300,echo=True):
  out=bytearray();end=time.monotonic()+timeout
  while time.monotonic()<end:
   data=self.read(1)
   if not data:continue
   c=data[0]
   if c==13:
    if echo:self.send('\r\n')
    return out.decode(errors='replace')
   if c==10:
    if not out:continue
    if echo:self.send('\r\n')
    return out.decode(errors='replace')
   if c in (8,127):
    if out:out.pop();self.send('\b \b' if echo else '')
   elif 32<=c<=126 and len(out)<512:
    out.append(c)
    if echo:self.send(bytes([c]))
  raise Disconnected('idle timeout')
 def prompt(self,s):self.send(s);return self.line()

def payload(size):
 pattern=b'0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz\r\n'
 return (pattern*((size+len(pattern)-1)//len(pattern)))[:size]

def bridge(port,target):
 protocol=target['protocol'];host=target['host'];number=int(target.get('port',22 if protocol=='ssh' else 23))
 if protocol=='ssh':
  cmd=['/usr/bin/ssh','-tt','-o','StrictHostKeyChecking=ask','-o','UserKnownHostsFile=/var/lib/bri-test/known_hosts','-o','IdentityAgent=none','-o','PubkeyAuthentication=no','-p',str(number)]
  user=target.get('user')
  cmd+=['--',f'{user}@{host}' if user else host]
 elif protocol=='telnet':cmd=['/usr/bin/telnet','--',host,str(number)]
 else:raise ValueError('unknown target protocol')
 port.send('\r\nOpening target. Ctrl-] returns to menu.\r\n')
 master,slave=pty.openpty();proc=subprocess.Popen(cmd,stdin=slave,stdout=slave,stderr=slave,start_new_session=True);os.close(slave)
 try:
  while proc.poll() is None:
   port.alive();ready,_,_=select.select([port.fd,master],[],[],1)
   if master in ready:
    try:data=os.read(master,4096)
    except OSError:break
    if not data:break
    port.send(data)
   if port.fd in ready:
    data=port.read()
    if b'\x1d' in data:break
    if data:os.write(master,data)
 finally:
  if proc.poll() is None:
   os.killpg(proc.pid,signal.SIGTERM)
   try:proc.wait(timeout=3)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
  os.close(master)
 port.send('\r\nReturned to test menu.\r\n')

def file_transfer(port,protocol,direction,size=65536):
 """Let lrzsz own the modem descriptor exclusively during a transfer."""
 options={'z':['--zmodem'],'y':['--ymodem'],'x':['--xmodem','--with-crc']}
 if protocol not in options:raise ValueError('invalid transfer protocol')
 with tempfile.TemporaryDirectory(prefix='bri-transfer-') as directory:
  target=pathlib.Path(directory)
  if direction=='upload':
   cmd=['/usr/bin/rz','--restricted','--binary','--quiet','--timeout','100']+options[protocol]
   if protocol=='x':cmd+=['upload.bin']
  else:
   (target/'test-payload.bin').write_bytes(payload(size))
   cmd=['/usr/bin/sz','--restricted','--binary','--quiet','--timeout','100']+[o for o in options[protocol] if o!='--with-crc']+['test-payload.bin']
  kwargs={}
  if os.geteuid()==0:
   account=pwd.getpwnam('nobody');os.chown(directory,account.pw_uid,account.pw_gid)
   if direction=='download':os.chmod(target/'test-payload.bin',0o644)
   kwargs={'user':account.pw_uid,'group':account.pw_gid,'extra_groups':[]}
  # prlimit bounds received file sizes without preexec_fn in threaded workers.
  cmd=['/usr/bin/prlimit','--fsize=67108864','--',*cmd]
  port.buf.clear();termios.tcflush(port.fd,termios.TCIFLUSH)
  port.send(f'\r\n{protocol.upper()}MODEM {direction}: start your terminal transfer now.\r\n')
  flags=fcntl.fcntl(port.fd,fcntl.F_GETFL);settings=termios.tcgetattr(port.fd)
  fcntl.fcntl(port.fd,fcntl.F_SETFL,flags&~os.O_NONBLOCK)
  start=time.monotonic();proc=None
  try:
   proc=subprocess.Popen(cmd,stdin=port.fd,stdout=port.fd,stderr=subprocess.DEVNULL,cwd=directory,start_new_session=True,**kwargs)
   while proc.poll() is None:
    port.alive()
    if time.monotonic()-start>900:raise TimeoutError('transfer timeout')
    time.sleep(.2)
   elapsed=max(time.monotonic()-start,.001);result=proc.returncode
  finally:
   if proc and proc.poll() is None:
    os.killpg(proc.pid,signal.SIGTERM)
    try:proc.wait(timeout=2)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
   termios.tcsetattr(port.fd,termios.TCSANOW,settings)
   fcntl.fcntl(port.fd,fcntl.F_SETFL,flags)
  port.send(f'\r\nTransfer {"complete" if result==0 else "failed/cancelled"} in {elapsed:.2f}s.\r\n')
  if result==0:
   total=0
   for file in sorted(target.iterdir()):
    if file.is_symlink() or not file.is_file():continue
    digest=hashlib.sha256();count=0
    with file.open('rb') as stream:
     while chunk:=stream.read(65536):digest.update(chunk);count+=len(chunk)
    total+=count
    name=''.join(c if 32<=ord(c)<127 else '?' for c in file.name)
    port.send(f'{name}: {count} bytes\r\nSHA256 {digest.hexdigest()}\r\n')
   port.send(f'{total} bytes / {elapsed:.2f}s = {total/elapsed:.0f} bytes/s\r\n')
   LOG.info('%s protocol=%s bytes=%s seconds=%.2f',direction,protocol,total,elapsed)
  if direction=='upload':port.send('Test upload files discarded.\r\n')



def timed_stream(port,channel):
 kind=port.prompt('Test download/upload/duplex: ').strip().lower()
 try:seconds=int(port.prompt('Seconds [610], maximum 3600: ').strip() or '610')
 except ValueError:port.send('Invalid duration.\r\n');return
 if kind not in ('download','upload','duplex') or not 1<=seconds<=3600:
  port.send('Invalid test.\r\n');return
 port.send('STREAM READY\r\n')
 stats=stream_run(port.fd,'answerer',kind,seconds,port.alive,
                  lambda stats: LOG.info('%s stream progress %s',channel['device'],json.dumps(stats)))
 stats['device']=channel['device']
 LOG.info('%s stream result %s',channel['device'],json.dumps(stats))
 port.send('\r\nSTREAM RESULT '+json.dumps(stats)+'\r\n')

def ppp_session(port,channel):
 if not pathlib.Path('/usr/sbin/pppd').exists():
  port.send('PPP unavailable: install pppd.\r\n');return
 # Dedicated lab IP pair per tty; no default route or LAN proxy ARP.
 unit=int(channel['device'].split('ttyds')[-1])
 local=f'10.254.{90+unit}.1';remote=f'10.254.{90+unit}.2'
 port.send(f'PPP READY {local} {remote}\r\n')
 flags=fcntl.fcntl(port.fd,fcntl.F_GETFL);settings=termios.tcgetattr(port.fd)
 fcntl.fcntl(port.fd,fcntl.F_SETFL,flags&~os.O_NONBLOCK)
 proc=None
 try:
  proc=subprocess.Popen(['/usr/sbin/pppd','notty','nodetach','local','noauth',
       'nodefaultroute','noproxyarp','noipv6','noccp','novj','nopersist',
       'lcp-echo-interval','10','lcp-echo-failure','6','maxconnect','1800',
       f'{local}:{remote}'],stdin=port.fd,stdout=port.fd,stderr=subprocess.DEVNULL)
  while proc.poll() is None:port.alive();time.sleep(.2)
  LOG.info('PPP %s exit=%s',channel['device'],proc.returncode)
 finally:
  if proc and proc.poll() is None:
   proc.terminate()
   try:proc.wait(timeout=5)
   except subprocess.TimeoutExpired:proc.kill();proc.wait()
  termios.tcsetattr(port.fd,termios.TCSANOW,settings)
  fcntl.fcntl(port.fd,fcntl.F_SETFL,flags)
 # A PPP caller must hang up after teardown; do not mix menu bytes into HDLC.
 raise Disconnected('PPP ended')

def menu(port,targets,channel):
 port.send('\r\nBRI MODEM TEST LAB\r\n')
 while True:
  port.send('\r\n1 Text echo\r\n2 Upload test\r\n3 Download test\r\n')
  port.send('4 Telnet to a host\r\n5 SSH to a host\r\n')
  port.send('6 Timed binary stream\r\n7 PPP lab link\r\n')
  port.send('Q Hang up\r\n')
  choice=port.prompt('Select: ').strip().lower()
  if choice=='q':return
  if choice=='6':timed_stream(port,channel);continue
  if choice=='7':ppp_session(port,channel);continue
  if choice=='1':
   port.send('Echo test: each line is returned. /menu exits.\r\n')
   while True:
    line=port.line()
    if line=='/menu':break
    port.send('ECHO: '+line+'\r\n')
  elif choice in ('2','3'):
   protocol=port.prompt('Protocol ZMODEM/YMODEM/XMODEM [Z]: ').strip().lower() or 'z'
   protocol=protocol[0]
   if protocol not in ('z','y','x'):port.send('Unknown protocol.\r\n');continue
   size=65536
   if choice=='3':
    try:size=int(port.prompt('Byte count [65536], maximum 1048576: ').strip() or '65536')
    except ValueError:port.send('Invalid count.\r\n');continue
    if not 1<=size<=1048576:port.send('Count out of range.\r\n');continue
   try:file_transfer(port,protocol,'upload' if choice=='2' else 'download',size)
   except TimeoutError:port.send('\r\nTransfer timed out.\r\n')
  elif choice in ('4','5'):
   host=port.prompt('Hostname or IP: ').strip()
   if not host or host.startswith('-') or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.-:' for c in host):
    port.send('Invalid host.\r\n');continue
   try:socket.getaddrinfo(host,None)
   except socket.gaierror:port.send('Host could not be resolved.\r\n');continue
   protocol='ssh' if choice=='5' else 'telnet'
   try:number=int(port.prompt(f'Port [{22 if protocol=="ssh" else 23}]: ').strip() or ('22' if protocol=='ssh' else '23'))
   except ValueError:port.send('Invalid port.\r\n');continue
   if not 1<=number<=65535:port.send('Invalid port.\r\n');continue
   target={'host':host,'port':number,'protocol':protocol}
   if protocol=='ssh':
    user=port.prompt('SSH username: ').strip()
    if not user or user.startswith('-') or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-' for c in user):
     port.send('Invalid username.\r\n');continue
    target['user']=user
   bridge(port,target)
  else:port.send('Unknown option.\r\n')

def open_port(path):
 fd=os.open(path,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
 a=termios.tcgetattr(fd);a[0]=0;a[1]=0;a[2]=termios.CS8|termios.CREAD|termios.CLOCAL|termios.HUPCL|termios.CRTSCTS;a[3]=0;a[4]=a[5]=termios.B115200;a[6][termios.VMIN]=0;a[6][termios.VTIME]=0
 termios.tcsetattr(fd,termios.TCSANOW,a);return Port(fd)

def command(port,cmd):
 port.send(cmd+'\r');data=bytearray();end=time.monotonic()+5
 while time.monotonic()<end:
  data.extend(port.read(timeout=.2,carrier=False))
  if b'ERROR' in data:raise RuntimeError(f'{cmd}: ERROR')
  if b'OK' in data:return
 raise RuntimeError(f'{cmd}: no OK')

def worker(channel,targets):
 path=channel['device']
 while True:
  port=None
  try:
   port=open_port(path)
   for cmd in ('AT&F14',f'AT+iQ=a{channel["controller"]}',f'AT+iA{channel["number"]}','AT+MS=V90,1',r'AT\N3%C1',r'ATE0V1S0=1\V1#CID=14'):
    command(port,cmd)
   LOG.info('%s ready controller=%s number=%s',path,channel['controller'],channel['number'])
   data=bytearray()
   while True:
    data.extend(port.read(timeout=1,carrier=False))
    if b'CONNECT ' in data:
     idx=data.index(b'CONNECT ');stop=data.find(b'\n',idx)
     if stop<0:continue
     LOG.info('%s %s',path,bytes(data[idx:stop]).decode(errors='replace').strip())
     port.buf.extend(data[stop+1:]);break
    if len(data)>8192:del data[:-4096]
   try:menu(port,targets,channel)
   except Disconnected:pass
   LOG.info('%s session ended',path)
  except Exception:LOG.exception('%s worker restarting',path);time.sleep(2)
  finally:
   if port:os.close(port.fd)
  time.sleep(.5)

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--config',required=True);args=ap.parse_args()
 cfg=json.load(open(args.config));logging.basicConfig(level=logging.INFO,format='%(asctime)s %(message)s')
 threads=[]
 for channel in cfg['channels']:
  t=threading.Thread(target=worker,args=(channel,cfg.get('targets',[])),daemon=True);t.start();threads.append(t)
 for t in threads:t.join()
if __name__=='__main__':main()
