#!/usr/bin/env python3
"""half_sip_call.py: put one of D-Modem's interop harnesses (half_<peer>,
dsp/test in the D-Modem build) on a live SIP call, as the caller.

A harness reads its line's audio on fd 3 and writes its own on fd 4 (16 bit
samples, one out for each in) and prints its DTE pty ("created <name>:
<pty>"); D-Modem's interop_call joins two of them through netsim.  This puts
the line on an RTP stream instead: it INVITEs <number> at the SIP server as
<user> (password = user, the rig's test accounts), PCMU only, and runs the
harness in real time on a local 20 ms clock, silence on its line until the
call is answered.

The DTE gets ATZ, the --at commands one by one (each must say OK), then a
bare ATD once the call is answered, as interop_call does.  After CONNECT the
data check is PPP, because the rig's v90ras answers each line into pppd:
the async-HDLC frames from the far end must pass their FCS; LCP must open
in both directions (their multilink options refused, the rest acked as
sent, ours empty), then IPCP (taking the address the server offers); and
--echoes LCP Echo-Requests and then as many ICMP pings to the server's
address, each with a random payload, must come back intact.  No
pppd is needed on this host.

The bearer is passed through untouched: G.711 to and from 16 bit by the
standard u-law tables only, no gain.  The far end's frames go into a small
queue that the local clock drains; a frame is dropped only if the queue
exceeds --max-queue (clock drift), and every drop is counted and reported.

  half_sip_call.py --harness ./half_courier --at 'ATS34=128S56=224S58=33' \\
      --number 2950 --out /root/live/c1
Prints one result line; exit status 0 when the check passes.
"""
import argparse, audioop, hashlib, os, random, re, select, socket, struct
import subprocess, sys, time, uuid

ap = argparse.ArgumentParser()
ap.add_argument('--harness', required=True)
ap.add_argument('--harness-dir', default='.', help='cwd for the harness')
ap.add_argument('--at', default='', help="';'-separated setup commands")
ap.add_argument('--number', default='2950')
ap.add_argument('--server', default='192.168.88.122')
ap.add_argument('--user', default='2903')
ap.add_argument('--local-ip', default=None)
ap.add_argument('--sip-port', type=int, default=5190)
ap.add_argument('--rtp-port', type=int, default=31000)
ap.add_argument('--out', required=True)
ap.add_argument('--connect-timeout', type=float, default=90)
ap.add_argument('--echoes', type=int, default=40)
ap.add_argument('--echo-bytes', type=int, default=200)
ap.add_argument('--max-queue', type=int, default=8)
ap.add_argument('--hold', type=float, default=0, help='extra seconds of call after the check')
ap.add_argument('--answer', action='store_true',
                help='REGISTER as --user and answer one call (ATA) instead of calling')
ap.add_argument('--wait', type=float, default=120, help='--answer: seconds to wait for the call')
ap.add_argument('--check', choices=('ppp', 'lines'), default=None,
                help='data check: ppp (default when calling) or lines (default when answering)')
ap.add_argument('--lines', type=int, default=40, help="lines: this many each way")
ap.add_argument('--line-interval', type=float, default=0.5)
args = ap.parse_args()
os.makedirs(args.out, exist_ok=True)
logf = open(os.path.join(args.out, 'bridge.log'), 'w')
T0 = time.monotonic()


def log(msg):
    logf.write('%8.3f %s\n' % (time.monotonic() - T0, msg))
    logf.flush()


def local_ip():
    if args.local_ip:
        return args.local_ip
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect((args.server, 5060))
    ip = s.getsockname()[0]
    s.close()
    return ip


LIP = local_ip()

# ---------------------------------------------------------------- SIP
sip = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sip.bind(('0.0.0.0', args.sip_port))
rtp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rtp.bind(('0.0.0.0', args.rtp_port))
rtp.setblocking(False)
call_id = uuid.uuid4().hex + '@' + LIP
tag = uuid.uuid4().hex[:10]
cseq = 1
ruri = 'sip:%s@%s' % (args.number, args.server)
sdp = ('v=0\r\no=- 1 1 IN IP4 %s\r\ns=-\r\nc=IN IP4 %s\r\nt=0 0\r\n'
       'm=audio %d RTP/AVP 0\r\na=rtpmap:0 PCMU/8000\r\na=ptime:20\r\na=sendrecv\r\n'
       % (LIP, LIP, args.rtp_port))


def sip_send(text):
    sip.sendto(text.encode(), (args.server, 5060))
    log('SIP > ' + text.split('\r\n', 1)[0])


def invite(auth=None, branch=None):
    h = ['INVITE %s SIP/2.0' % ruri,
         'Via: SIP/2.0/UDP %s:%d;branch=z9hG4bK%s;rport' % (LIP, args.sip_port, branch or uuid.uuid4().hex[:12]),
         'Max-Forwards: 70',
         'From: <sip:%s@%s>;tag=%s' % (args.user, args.server, tag),
         'To: <%s>' % ruri,
         'Call-ID: ' + call_id,
         'CSeq: %d INVITE' % cseq,
         'Contact: <sip:%s@%s:%d>' % (args.user, LIP, args.sip_port),
         'Content-Type: application/sdp',
         'User-Agent: half_sip_call']
    if auth:
        h.append(auth)
    h.append('Content-Length: %d' % len(sdp))
    sip_send('\r\n'.join(h) + '\r\n\r\n' + sdp)


def parse(msg):
    head, _, body = msg.partition('\r\n\r\n')
    lines = head.split('\r\n')
    hdr = {}
    for l in lines[1:]:
        k, _, v = l.partition(':')
        hdr.setdefault(k.strip().lower(), []).append(v.strip())
    return lines[0], hdr, body


def digest(chal, method, uri):
    p = dict(re.findall(r'(\w+)="?([^",]*)"?', chal))
    ha1 = hashlib.md5(('%s:%s:%s' % (args.user, p['realm'], args.user)).encode()).hexdigest()
    ha2 = hashlib.md5(('%s:%s' % (method, uri)).encode()).hexdigest()
    if 'qop' in p:
        cn = uuid.uuid4().hex[:8]
        r = hashlib.md5(('%s:%s:00000001:%s:auth:%s' % (ha1, p['nonce'], cn, ha2)).encode()).hexdigest()
        return ('username="%s", realm="%s", nonce="%s", uri="%s", response="%s", algorithm=MD5, '
                'qop=auth, nc=00000001, cnonce="%s"%s' % (args.user, p['realm'], p['nonce'], uri, r, cn,
                                                         ', opaque="%s"' % p['opaque'] if 'opaque' in p else ''))
    r = hashlib.md5(('%s:%s:%s' % (ha1, p['nonce'], ha2)).encode()).hexdigest()
    return 'username="%s", realm="%s", nonce="%s", uri="%s", response="%s", algorithm=MD5' % (
        args.user, p['realm'], p['nonce'], uri, r)


dialog = {}   # to (with tag), route, remote target, rtp address


def ack_for(hdr, status_line):
    to = hdr['to'][0]
    target = ruri
    if status_line.split()[1] == '200' and 'contact' in hdr:
        m = re.search(r'<([^>]+)>', hdr['contact'][0])
        target = m.group(1) if m else ruri
    h = ['ACK %s SIP/2.0' % target,
         'Via: SIP/2.0/UDP %s:%d;branch=z9hG4bK%s;rport' % (LIP, args.sip_port, uuid.uuid4().hex[:12]
                                                            if status_line.split()[1] == '200' else
                                                            hdr['via'][0].split('branch=z9hG4bK')[1].split(';')[0]),
         'Max-Forwards: 70',
         'From: <sip:%s@%s>;tag=%s' % (args.user, args.server, tag),
         'To: ' + to, 'Call-ID: ' + call_id, 'CSeq: %d ACK' % cseq]
    for rr in reversed(hdr.get('record-route', [])):
        h.append('Route: ' + rr)
    h.append('Content-Length: 0')
    sip_send('\r\n'.join(h) + '\r\n\r\n')


def bye():
    global cseq
    if not dialog:
        return
    cseq += 1
    h = ['BYE %s SIP/2.0' % dialog['target'],
         'Via: SIP/2.0/UDP %s:%d;branch=z9hG4bK%s;rport' % (LIP, args.sip_port, uuid.uuid4().hex[:12]),
         'Max-Forwards: 70',
         'From: ' + dialog.get('local', '<sip:%s@%s>;tag=%s' % (args.user, args.server, tag)),
         'To: ' + dialog['to'], 'Call-ID: ' + dialog.get('call_id', call_id), 'CSeq: %d BYE' % cseq]
    for rr in dialog['route']:
        h.append('Route: ' + rr)
    h.append('Content-Length: 0')
    sip_send('\r\n'.join(h) + '\r\n\r\n')


def reply_ok(hdr, first, code='200 OK', to=None, body=None):
    h = ['SIP/2.0 ' + code] + ['Via: ' + v for v in hdr['via']] + [
        'From: ' + hdr['from'][0], 'To: ' + (to or hdr['to'][0]), 'Call-ID: ' + hdr['call-id'][0],
        'CSeq: ' + hdr['cseq'][0]]
    if body:
        h += ['Contact: <sip:%s@%s:%d>' % (args.user, LIP, args.sip_port),
              'Content-Type: application/sdp', 'Content-Length: %d' % len(body)]
        sip_send('\r\n'.join(h) + '\r\n\r\n' + body)
    else:
        h.append('Content-Length: 0')
        sip_send('\r\n'.join(h) + '\r\n\r\n')


reg = dict(cseq=1, call_id=uuid.uuid4().hex + '@' + LIP, tag=uuid.uuid4().hex[:10], ok=False)


def register(expires=300, auth=None):
    uri = 'sip:%s' % args.server
    h = ['REGISTER %s SIP/2.0' % uri,
         'Via: SIP/2.0/UDP %s:%d;branch=z9hG4bK%s;rport' % (LIP, args.sip_port, uuid.uuid4().hex[:12]),
         'Max-Forwards: 70',
         'From: <sip:%s@%s>;tag=%s' % (args.user, args.server, reg['tag']),
         'To: <sip:%s@%s>' % (args.user, args.server),
         'Call-ID: ' + reg['call_id'], 'CSeq: %d REGISTER' % reg['cseq'],
         'Contact: <sip:%s@%s:%d>' % (args.user, LIP, args.sip_port),
         'Expires: %d' % expires, 'User-Agent: half_sip_call']
    if auth:
        h.append(auth)
    h.append('Content-Length: 0')
    reg['expires'] = expires
    sip_send('\r\n'.join(h) + '\r\n\r\n')
    reg['cseq'] += 1


# ---------------------------------------------------------------- harness
r3, w3 = os.pipe()   # line into the harness (its fd 3)
r4, w4 = os.pipe()   # its line out (its fd 4)
herr = open(os.path.join(args.out, 'harness.log'), 'wb')


def child_fds():
    # subprocess closes every fd but pass_fds after this runs, so 3 and 4
    # are made here and kept with close_fds=False; our own ends are closed
    os.dup2(r3, 3)
    os.dup2(w4, 4)
    for fd in (r3, w4, w3, r4):
        if fd > 4:
            os.close(fd)


proc = subprocess.Popen([args.harness], cwd=args.harness_dir, preexec_fn=child_fds,
                        close_fds=False, stdout=subprocess.PIPE, stderr=herr)
os.close(r3); os.close(w4)
pty_name = None
buf = b''
while pty_name is None:
    line = proc.stdout.readline()
    if not line:
        sys.exit('harness ended before its pty: see harness.log')
    herr.write(line); herr.flush()
    m = re.search(rb'created \w+: (\S+)', line)
    if m:
        pty_name = m.group(1).decode()
log('harness pty ' + pty_name)
# keep draining its stdout into the log
os.set_blocking(proc.stdout.fileno(), False)
dte = os.open(pty_name, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
# the harness has made the pty raw already
tx_tap = open(os.path.join(args.out, 'line-tx.g711'), 'wb')   # harness -> network
rx_tap = open(os.path.join(args.out, 'line-rx.g711'), 'wb')   # network -> harness
dte_rx = open(os.path.join(args.out, 'dte-rx.bin'), 'wb')

# ---------------------------------------------------------------- state
queue = []
stats = dict(rtp_in=0, dropped=0, underrun=0, underrun_mid=0, late_ticks=0)
seq = random.randrange(65536); ts = random.randrange(1 << 32); ssrc = random.randrange(1 << 32)
call_state = 'idle'   # idle, inviting, up, ended
remote_rtp = None
dte_in = bytearray()
SILENCE = b'\xff' * 160


def tick():
    """one 20 ms frame through the harness"""
    global seq, ts
    if call_state == 'up' and queue:
        frame = queue.pop(0)
        stats['started'] = True
    else:
        if call_state == 'up':
            stats['underrun'] += 1
            if stats.get('started'):
                stats['underrun_mid'] += 1
                log('underrun (queue empty) mid-call')
        frame = SILENCE
    rx_tap.write(frame) if call_state == 'up' else None
    os.write(w3, audioop.ulaw2lin(frame, 2))
    need = 320; out = b''
    while len(out) < need:
        chunk = os.read(r4, need - len(out))
        if not chunk:
            raise EOFError('harness line closed')
        out += chunk
    u = audioop.lin2ulaw(out, 2)
    if call_state == 'up' and remote_rtp:
        tx_tap.write(u)
        rtp.sendto(struct.pack('!BBHII', 0x80, 0, seq, ts, ssrc) + u, remote_rtp)
        seq = (seq + 1) & 0xffff
    ts = (ts + 160) & 0xffffffff


def service(timeout):
    """SIP, RTP, the DTE and the harness's stdout, until the next tick"""
    global call_state, remote_rtp, cseq
    end = time.monotonic() + timeout
    first = True
    while True:
        left = end - time.monotonic()
        # poll at least once: a tick that runs late (catching up) must
        # still take the frames that have arrived, or it feeds silence
        if left <= 0 and not first:
            return
        first = False
        rd, _, _ = select.select([sip, rtp, dte, proc.stdout], [], [], max(0.0, left))
        if rtp in rd:
            while True:
                try:
                    pkt, addr = rtp.recvfrom(2048)
                except BlockingIOError:
                    break
                if len(pkt) < 12 or (pkt[1] & 0x7f) != 0:
                    continue
                cc = pkt[0] & 0x0f
                payload = pkt[12 + 4 * cc:]
                if len(payload) != 160:
                    log('RTP payload of %d octets' % len(payload))
                stats['rtp_in'] += 1
                if call_state == 'up':
                    queue.append(payload)
                    if len(queue) > args.max_queue:
                        queue.pop(0); stats['dropped'] += 1
        if sip in rd:
            msg = sip.recv(65535).decode(errors='replace')
            first, hdr, body = parse(msg)
            log('SIP < ' + first)
            if first.startswith('SIP/2.0'):
                code = int(first.split()[1])
                if 'REGISTER' in hdr.get('cseq', [''])[0]:
                    if code in (401, 407):
                        name = 'www-authenticate' if code == 401 else 'proxy-authenticate'
                        register(reg['expires'], ('Authorization' if code == 401 else 'Proxy-Authorization')
                                 + ': Digest ' + digest(hdr[name][0], 'REGISTER', 'sip:%s' % args.server))
                    elif code == 200:
                        reg['ok'] = reg['expires'] > 0
                        log('registered' if reg['ok'] else 'unregistered')
                    else:
                        log('REGISTER failed: ' + first)
                elif 'INVITE' in hdr.get('cseq', [''])[0]:
                    if code in (401, 407):
                        ack_for(hdr, first)
                        name = 'www-authenticate' if code == 401 else 'proxy-authenticate'
                        cseq += 1
                        invite(('Authorization' if code == 401 else 'Proxy-Authorization')
                               + ': Digest ' + digest(hdr[name][0], 'INVITE', ruri))
                    elif code == 200:
                        ack_for(hdr, first)
                        m = re.search(r'c=IN IP4 (\S+)', body); p = re.search(r'm=audio (\d+)', body)
                        remote_rtp = (m.group(1), int(p.group(1)))
                        tgt = re.search(r'<([^>]+)>', hdr['contact'][0]).group(1)
                        dialog.update(to=hdr['to'][0], target=tgt,
                                      route=list(reversed(hdr.get('record-route', []))))
                        if call_state != 'up':
                            call_state = 'up'
                            log('answered, media to %s:%d' % remote_rtp)
                    elif code >= 300:
                        ack_for(hdr, first)
                        call_state = 'ended'
                        log('call failed: ' + first)
            else:
                method = first.split()[0]
                if method == 'INVITE' and args.answer:
                    if call_state in ('idle', 'up') and (not dialog or dialog.get('call_id') == hdr['call-id'][0]):
                        if not dialog:
                            m = re.search(r'c=IN IP4 (\S+)', body); p = re.search(r'm=audio (\d+)', body)
                            remote_rtp = (m.group(1), int(p.group(1)))
                            tgt = re.search(r'<([^>]+)>', hdr['contact'][0]).group(1)
                            dialog.update(call_id=hdr['call-id'][0], to=hdr['from'][0],
                                          local=hdr['to'][0] + ';tag=' + tag, target=tgt,
                                          route=hdr.get('record-route', []))
                            reply_ok(hdr, first, '100 Trying')
                            log('INVITE from %s, media to %s:%d' % (hdr['from'][0], remote_rtp[0], remote_rtp[1]))
                        reply_ok(hdr, first, '200 OK', to=dialog['local'], body=sdp)
                        call_state = 'up'
                    else:
                        reply_ok(hdr, first, '486 Busy Here')
                elif method == 'CANCEL':
                    reply_ok(hdr, first)
                    call_state = 'ended'
                elif method in ('BYE', 'OPTIONS', 'NOTIFY', 'UPDATE', 'INFO'):
                    reply_ok(hdr, first)
                if method == 'BYE':
                    call_state = 'ended'
                    log('far end hung up')
        if dte in rd:
            try:
                d = os.read(dte, 4096)
            except BlockingIOError:
                d = b''
            except OSError:
                d = b''
            if d:
                dte_in.extend(d); dte_rx.write(d); dte_rx.flush()
        if proc.stdout in rd:
            try:
                d = proc.stdout.read()
            except Exception:
                d = None
            if d:
                herr.write(d); herr.flush()


# ---------------------------------------------------------------- the clock
next_t = time.monotonic()


def run_for(seconds, until=None):
    global next_t
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if until and until():
            return True
        now = time.monotonic()
        if now - next_t > 0.04:
            stats['late_ticks'] += 1
            log('tick %.0f ms late' % (1000 * (now - next_t)))
        tick()
        next_t += 0.020
        service(max(0.0, next_t - time.monotonic()))
    return until() if until else True


def dte_send(s):
    os.write(dte, s.encode() if isinstance(s, str) else s)


def wait_text(pattern, seconds):
    start = len(dte_in)
    rx = re.compile(pattern)
    ok = run_for(seconds, lambda: rx.search(bytes(dte_in[start:])) is not None)
    return ok, bytes(dte_in[start:])


def finish(result, ok):
    global call_state
    if call_state == 'up':
        bye()
        call_state = 'bye'
        run_for(1.0)
    if reg['ok']:
        register(0)
        run_for(1.0)
    try:
        proc.terminate(); proc.wait(5)
    except Exception:
        proc.kill()
    s = ('%s | rtp_in=%d dropped=%d underruns=%d (mid-call %d) late_ticks(>40ms)=%d' %
         (result, stats['rtp_in'], stats['dropped'], stats['underrun'], stats['underrun_mid'],
          stats['late_ticks']))
    log('RESULT ' + s)
    print(s)
    sys.exit(0 if ok else 1)


run_for(0.5)
for c in ['ATZ'] + [c for c in args.at.split(';') if c]:
    dte_send(c + '\r')
    ok, text = wait_text(rb'OK\r|ERROR', 5)
    log('DTE %s -> %r' % (c, text))
    if not ok or b'ERROR' in text:
        finish('setup %s failed: %r' % (c, text), False)

if args.answer:
    register()
    if not run_for(10, lambda: reg['ok']):
        finish('not registered as %s' % args.user, False)
    print('registered as %s' % args.user, flush=True)
    if not run_for(args.wait, lambda: call_state in ('up', 'ended')) or call_state != 'up':
        finish('no call (%s)' % call_state, False)
    t_ans = time.monotonic()
    dte_send('ATA\r')
else:
    invite()
    call_state = 'inviting'
    if not run_for(30, lambda: call_state in ('up', 'ended')) or call_state != 'up':
        finish('call not answered (%s)' % call_state, False)
    t_ans = time.monotonic()
    dte_send('ATD\r')
ok, text = wait_text(rb'CONNECT[^\r]*\r|NO CARRIER', args.connect_timeout)
m = re.search(rb'CONNECT[^\r]*', text)
if not m:
    finish('no CONNECT: %r' % text[-200:], False)
connect = m.group(0).decode()
t_conn = time.monotonic() - t_ans
log('%s after %.1f s' % (connect, t_conn))
check = args.check or ('lines' if args.answer else 'ppp')
if check == 'lines':
    # each side sends --lines numbered lines; ours are 'srv'/'cli' by role
    mine, theirs = ('srv', 'cli') if args.answer else ('cli', 'srv')
    start = m.end()
    pat = re.compile((r'%s (\d{5}) the quick brown fox jumps over the lazy dog 0123456789\r\n' % theirs).encode())
    for i in range(args.lines):
        if call_state != 'up':
            break
        dte_send('%s %05d the quick brown fox jumps over the lazy dog 0123456789\r\n' % (mine, i))
        run_for(args.line_interval)
    got = lambda: len(set(pat.findall(bytes(dte_in[start:]))))
    run_for(15 + args.hold, lambda: got() >= args.lines or call_state != 'up')
    n = got()
    carrier = b'NO CARRIER' in bytes(dte_in[start:])
    ok = n == args.lines and not carrier
    finish('%s %s after %.1f s; %d of %d %r lines exact; %s; call %s'
           % ('PASS' if ok else 'FAIL', connect, t_conn, n, args.lines, theirs,
              'NO CARRIER' if carrier else 'carrier held', call_state), ok)

# ---------------------------------------------------------------- PPP


def fcs16(data):
    f = 0xffff
    for b in data:
        f ^= b
        for _ in range(8):
            f = (f >> 1) ^ 0x8408 if f & 1 else f >> 1
    return f


def frame(proto, payload):
    body = b'\xff\x03' + struct.pack('!H', proto) + payload
    f = fcs16(body) ^ 0xffff
    body += struct.pack('<H', f)
    out = bytearray(b'\x7e')
    for b in body:
        if b < 0x20 or b in (0x7d, 0x7e):
            out += bytes([0x7d, b ^ 0x20])
        else:
            out.append(b)
    out.append(0x7e)
    return bytes(out)


ppp = dict(good=0, bad=0, lcp_acked_theirs=False, lcp_ours_acked=False, echo_ok=0, echo_bad=0,
           ipcp_acked_theirs=False, ipcp_ours_acked=False, ping_ok=0, ping_bad=0, terminated=False)
pos = [m.end()]   # parse the DTE stream from just after CONNECT
cur = bytearray(); esc = [False]
frames = []
our_ip = [b'\0\0\0\0']; their_ip = [None]


def pull_frames():
    data = bytes(dte_in[pos[0]:]); pos[0] = len(dte_in)
    for b in data:
        if b == 0x7e:
            if len(cur) >= 4:
                if fcs16(cur) == 0xf0b8:
                    ppp['good'] += 1; frames.append(bytes(cur[:-2]))
                elif ppp['good']:
                    ppp['bad'] += 1   # the text before the first flag is not a frame
            cur.clear(); esc[0] = False
        elif b == 0x7d:
            esc[0] = True
        else:
            cur.append(b ^ 0x20 if esc[0] else b); esc[0] = False


def proto_of(f):
    if f[:2] == b'\xff\x03':
        f = f[2:]
    if f[0] & 1:
        return f[0], f[1:]
    return struct.unpack('!H', f[:2])[0], f[2:]


def options(body):
    out = []; i = 0
    while i + 2 <= len(body):
        t, ln = body[i], body[i + 1]
        if ln < 2:
            break
        out.append((t, body[i:i + ln])); i += ln
    return out


def cp(proto, code, ident, data=b''):
    dte_send(frame(proto, bytes([code, ident]) + struct.pack('!H', 4 + len(data)) + data))


def csum(b):
    if len(b) & 1:
        b += b'\0'
    s = sum(struct.unpack('!%dH' % (len(b) // 2), b))
    while s >> 16:
        s = (s & 0xffff) + (s >> 16)
    return (~s) & 0xffff


echo_wait = {}
ping_wait = {}
# RFC 1990 multilink (MRRU 17, short sequence 18, endpoint discriminator 19)
# is refused, so the server's pppd runs this as a plain link
MP_OPTS = (17, 18, 19)


def handle_frames():
    while frames:
        p, d = proto_of(frames.pop(0))
        if p == 0x0021:
            if len(d) >= 28 and d[9] == 1 and d[20] == 0:      # ICMP echo reply
                ident, sq = struct.unpack('!HH', d[24:28])
                want = ping_wait.pop(sq, None)
                if want is not None:
                    if d[28:] == want:
                        ppp['ping_ok'] += 1
                    else:
                        ppp['ping_bad'] += 1
            continue
        if p not in (0xc021, 0x8021) or len(d) < 4:
            if p not in (0xc021, 0x8021):
                # anything else (IPV6CP, CCP ...): Protocol-Reject
                cp(0xc021, 8, 99, struct.pack('!H', p) + d[:32])
            continue
        code, ident = d[0], d[1]
        ln = struct.unpack('!H', d[2:4])[0]
        body = d[4:ln]
        if p == 0xc021:
            if code == 1:      # Configure-Request
                rej = b''.join(o for t, o in options(body) if t in MP_OPTS)
                if rej:
                    cp(0xc021, 4, ident, rej)
                    log('LCP: rejected their multilink options')
                else:
                    cp(0xc021, 2, ident, body)
                    if not ppp['lcp_acked_theirs']:
                        log('LCP: acked their Configure-Request (%d option octets)' % len(body))
                    ppp['lcp_acked_theirs'] = True
            elif code == 2 and ident == 1:
                ppp['lcp_ours_acked'] = True
            elif code == 9:    # Echo-Request
                cp(0xc021, 10, ident, b'\0\0\0\0' + body[4:])
            elif code == 10 and ident in echo_wait:
                if body[4:] == echo_wait.pop(ident):
                    ppp['echo_ok'] += 1
                else:
                    ppp['echo_bad'] += 1
            elif code == 5:    # Terminate-Request
                cp(0xc021, 6, ident)
                ppp['terminated'] = True
                log('LCP: Terminate-Request from the far end')
        else:                  # IPCP
            if code == 1:
                opts = options(body)
                addr = [o for t, o in opts if t == 3]
                other = b''.join(o for t, o in opts if t != 3)
                if other:
                    cp(0x8021, 4, ident, other)
                else:
                    if addr:
                        their_ip[0] = addr[0][2:6]
                    cp(0x8021, 2, ident, body)
                    ppp['ipcp_acked_theirs'] = True
            elif code == 3:    # Nak: take the address offered
                for t, o in options(body):
                    if t == 3:
                        our_ip[0] = o[2:6]
                cp(0x8021, 1, 1, bytes([3, 6]) + our_ip[0])
            elif code == 2:
                ppp['ipcp_ours_acked'] = True
            elif code == 5:
                cp(0x8021, 6, ident)


def lcp_step():
    pull_frames(); handle_frames()
    return False


def negotiate(what, req, done, seconds=30):
    t0 = time.monotonic(); last = 0
    while time.monotonic() - t0 < seconds and not done() and call_state == 'up' and not ppp['terminated']:
        if time.monotonic() - last > 3:
            req(); last = time.monotonic()
        run_for(0.1); lcp_step()
    return done()


lcp_open = negotiate('LCP', lambda: cp(0xc021, 1, 1),
                     lambda: ppp['lcp_acked_theirs'] and ppp['lcp_ours_acked'])
log('LCP open %s, frames good %d bad %d' % (lcp_open, ppp['good'], ppp['bad']))
ipcp_open = lcp_open and negotiate('IPCP', lambda: cp(0x8021, 1, 1, bytes([3, 6]) + our_ip[0]),
                                   lambda: ppp['ipcp_acked_theirs'] and ppp['ipcp_ours_acked'])
if ipcp_open:
    log('IPCP open: us %s, them %s' % (socket.inet_ntoa(our_ip[0]), socket.inet_ntoa(their_ip[0] or b'\0' * 4)))


def exchange(n, send, waiting):
    for i in range(n):
        send(i)
        t = time.monotonic()
        while waiting(i) and time.monotonic() - t < 8 and call_state == 'up':
            run_for(0.05); lcp_step()
        if call_state != 'up':
            return


def send_echo(i):
    ident = 2 + (i % 250); data = os.urandom(args.echo_bytes)
    echo_wait.clear(); echo_wait[ident] = data
    cp(0xc021, 9, ident, b'\0\0\0\0' + data)


def send_ping(i):
    data = os.urandom(args.echo_bytes)
    ping_wait.clear(); ping_wait[i] = data
    icmp = struct.pack('!BBHHH', 8, 0, 0, 0x5650, i) + data
    icmp = icmp[:2] + struct.pack('!H', csum(icmp)) + icmp[4:]
    ip = struct.pack('!BBHHHBBH4s4s', 0x45, 0, 20 + len(icmp), i, 0, 64, 1, 0, our_ip[0], their_ip[0])
    ip = ip[:10] + struct.pack('!H', csum(ip)) + ip[12:]
    dte_send(frame(0x0021, ip + icmp))


if lcp_open:
    exchange(args.echoes, send_echo, lambda i: bool(echo_wait))
    n_echo_lost = args.echoes - ppp['echo_ok'] - ppp['echo_bad']
if ipcp_open and their_ip[0]:
    exchange(args.echoes, send_ping, lambda i: bool(ping_wait))
if args.hold:
    run_for(args.hold, lambda: (lcp_step(), call_state != 'up')[1])
ok = (lcp_open and ipcp_open and ppp['echo_ok'] == args.echoes and ppp['ping_ok'] == args.echoes
      and ppp['bad'] == 0)
finish('%s %s after %.1f s; LCP %s, IPCP %s; LCP echoes %d/%d, pings %d/%d (%d B each); '
       'PPP frames good %d bad %d; call %s'
       % ('PASS' if ok else 'FAIL', connect, t_conn, 'open' if lcp_open else 'NOT open',
          'open' if ipcp_open else 'NOT open', ppp['echo_ok'], args.echoes, ppp['ping_ok'],
          args.echoes, args.echo_bytes, ppp['good'], ppp['bad'], call_state), ok)
