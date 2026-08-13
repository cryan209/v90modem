#!/usr/bin/env python3
"""G.711 byte-exactness probe for the Asterisk path.

Registers as a PJSIP endpoint, dials the Echo() test extension, streams a
known u-law codeword pattern, and diffs the returned payload against what was
sent. Any transcode, gain, PLC, or jitter-buffer resampling in the path shows
up as a byte difference; sample insertion/deletion shows up as an offset.

The pattern deliberately includes the codewords V.90 Phase 3 actually uses:
the full 0..255 sweep (every u-law level, so a law translation cannot hide in
a quiet region) and an Sd-like {+W,+0,+W,-W,-0,-W} six-slot cycle.

Needs a live PBX and a dialplan target that answers and echoes, e.g.

    exten => 9099,1,Answer()
    exten => 9099,n,Echo()
    exten => 9099,n,Hangup()

with no MixMonitor or other audiohook on it -- the point is to measure the
path, so anything in the path is part of what is being measured.

--law alaw offers PCMA instead. Against a ulaw-only endpoint that is expected
to fail with `488 Not Acceptable Here`, which is the useful result: it proves
the endpoint refuses the lossy path rather than merely preferring the clean
one.
"""
import argparse
import hashlib
import random
import re
import socket
import struct
import sys
import time

PTIME_MS = 20
SAMPLES_PER_PKT = 160

# Filled in from the command line by main().
SERVER = None
PORT = 5060
USER = None
PASSWD = None
TARGET = None
NPKTS = None


def md5(s):
    return hashlib.md5(s.encode()).hexdigest()


def build_pattern(npkts):
    """Deterministic u-law payload: 0..255 sweep, Sd-like cycle, then PRNG."""
    rng = random.Random(0x905A)
    out = bytearray()
    sweep = bytes(range(256))
    sd = bytes([0xE0, 0xFF, 0xE0, 0x60, 0x7F, 0x60])  # +W +0 +W -W -0 -W
    while len(out) < npkts * SAMPLES_PER_PKT:
        out += sweep
        out += sd * 40
        out += bytes(rng.randrange(256) for _ in range(320))
    return bytes(out[: npkts * SAMPLES_PER_PKT])


class SipUac:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(5)
        self.sock.connect((socket.gethostbyname(SERVER), PORT))
        self.local_ip, self.local_port = self.sock.getsockname()
        self.call_id = "%s@probe" % md5(str(time.time()))[:16]
        self.ftag = md5("f%f" % time.time())[:12]
        self.branch_n = 0
        self.cseq = 1

    def branch(self):
        self.branch_n += 1
        return "z9hG4bK-%s-%d" % (md5(self.call_id)[:10], self.branch_n)

    def send(self, msg):
        self.sock.send(msg.encode())

    def recv(self, want=None, timeout=6):
        end = time.time() + timeout
        while time.time() < end:
            self.sock.settimeout(max(0.2, end - time.time()))
            try:
                data = self.sock.recv(65535).decode(errors="replace")
            except socket.timeout:
                break
            code = data.split("\r\n")[0]
            if want is None or want in code:
                return data
            if " 100 " in code or " 180 " in code or " 183 " in code:
                continue
            return data
        return None

    def auth_header(self, resp, method, uri):
        m = re.search(r"WWW-Authenticate:\s*Digest\s*(.*)", resp, re.I)
        if not m:
            m = re.search(r"Proxy-Authenticate:\s*Digest\s*(.*)", resp, re.I)
        params = dict(re.findall(r'(\w+)="?([^",]+)"?', m.group(1)))
        realm, nonce = params["realm"], params["nonce"]
        ha1 = md5("%s:%s:%s" % (USER, realm, PASSWD))
        ha2 = md5("%s:%s" % (method, uri))
        rsp = md5("%s:%s:%s" % (ha1, nonce, ha2))
        return ('Authorization: Digest username="%s", realm="%s", nonce="%s", '
                'uri="%s", response="%s", algorithm=MD5\r\n'
                % (USER, realm, nonce, uri, rsp))


def sdp(ip, port, pt, name):
    return ("v=0\r\no=- 0 0 IN IP4 %s\r\ns=probe\r\nc=IN IP4 %s\r\nt=0 0\r\n"
            "m=audio %d RTP/AVP %d\r\na=rtpmap:%d %s/8000\r\na=ptime:20\r\n"
            "a=sendrecv\r\n" % (ip, ip, port, pt, pt, name))


def run(pt, name):
    uac = SipUac()
    rtp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rtp.bind(("0.0.0.0", 0))
    rtp_port = rtp.getsockname()[1]
    ip = uac.local_ip

    uri = "sip:%s" % SERVER
    to = "sip:%s@%s" % (TARGET, SERVER)
    contact = "<sip:%s@%s:%d>" % (USER, ip, uac.local_port)

    def invite(auth=""):
        body = sdp(ip, rtp_port, pt, name)
        return (
            "INVITE %s SIP/2.0\r\n"
            "Via: SIP/2.0/UDP %s:%d;branch=%s;rport\r\n"
            "From: <sip:%s@%s>;tag=%s\r\n"
            "To: <%s>\r\n"
            "Call-ID: %s\r\n"
            "CSeq: %d INVITE\r\n"
            "Contact: %s\r\n"
            "Max-Forwards: 70\r\n"
            "%s"
            "Content-Type: application/sdp\r\n"
            "Content-Length: %d\r\n\r\n%s"
            % (to, ip, uac.local_port, uac.branch(), USER, SERVER, uac.ftag,
               to, uac.call_id, uac.cseq, contact, auth, len(body), body))

    uac.send(invite())
    resp = uac.recv()
    if resp and (" 401 " in resp.split("\r\n")[0] or " 407 " in resp.split("\r\n")[0]):
        uac.cseq += 1
        uac.send(invite(uac.auth_header(resp, "INVITE", to)))
        resp = uac.recv(want=" 200 ")
    if not resp or " 200 " not in resp.split("\r\n")[0]:
        print("FAIL: no 200 OK.\n%s" % (resp or "(timeout)")[:600])
        return 1

    m = re.search(r"m=audio (\d+)", resp)
    c = re.search(r"c=IN IP4 ([\d.]+)", resp)
    remote = (c.group(1), int(m.group(1)))
    fmts = re.search(r"m=audio \d+ RTP/AVP ([\d ]+)", resp).group(1).split()
    print("negotiated: m=audio payload types %s -> %s:%d" % (fmts, remote[0], remote[1]))
    if fmts[0] != str(pt):
        print("WARNING: first payload type is %s, not %d (%s)" % (fmts[0], pt, name))

    rtag = re.search(r"To:.*?;tag=([^\r\n;]+)", resp).group(1)
    ack = ("ACK %s SIP/2.0\r\nVia: SIP/2.0/UDP %s:%d;branch=%s;rport\r\n"
           "From: <sip:%s@%s>;tag=%s\r\nTo: <%s>;tag=%s\r\nCall-ID: %s\r\n"
           "CSeq: %d ACK\r\nMax-Forwards: 70\r\nContent-Length: 0\r\n\r\n"
           % (to, ip, uac.local_port, uac.branch(), USER, SERVER, uac.ftag,
              to, rtag, uac.call_id, uac.cseq))
    uac.send(ack)

    payload = build_pattern(NPKTS)
    ssrc = 0x5A905A90
    seq0 = 1000
    ts0 = 160000
    rtp.settimeout(0.05)
    received = bytearray()

    for i in range(NPKTS):
        hdr = struct.pack("!BBHII", 0x80, pt, (seq0 + i) & 0xFFFF,
                          (ts0 + i * SAMPLES_PER_PKT) & 0xFFFFFFFF, ssrc)
        chunk = payload[i * SAMPLES_PER_PKT:(i + 1) * SAMPLES_PER_PKT]
        rtp.sendto(hdr + chunk, remote)
        deadline = time.time() + PTIME_MS / 1000.0
        while time.time() < deadline:
            try:
                pkt, _ = rtp.recvfrom(2048)
            except socket.timeout:
                continue
            if len(pkt) > 12 and (pkt[1] & 0x7F) == pt:
                received += pkt[12:]

    drain = time.time() + 0.5
    while time.time() < drain:
        try:
            pkt, _ = rtp.recvfrom(2048)
        except socket.timeout:
            continue
        if len(pkt) > 12 and (pkt[1] & 0x7F) == pt:
            received += pkt[12:]

    bye = ("BYE %s SIP/2.0\r\nVia: SIP/2.0/UDP %s:%d;branch=%s;rport\r\n"
           "From: <sip:%s@%s>;tag=%s\r\nTo: <%s>;tag=%s\r\nCall-ID: %s\r\n"
           "CSeq: %d BYE\r\nMax-Forwards: 70\r\nContent-Length: 0\r\n\r\n"
           % (to, ip, uac.local_port, uac.branch(), USER, SERVER, uac.ftag,
              to, rtag, uac.call_id, uac.cseq + 1))
    uac.send(bye)

    sent = payload
    print("sent %d bytes, received %d bytes" % (len(sent), len(received)))
    if not received:
        print("FAIL: no RTP returned")
        return 1

    # Echo() starts returning mid-stream; find the alignment offset.
    best = None
    probe = bytes(received[400:1200])
    idx = sent.find(probe)
    if idx >= 0:
        best = (idx - 400, len(probe))
    if best is None:
        for off in range(0, min(len(received), 4000)):
            seg = bytes(received[off:off + 800])
            j = sent.find(seg)
            if j >= 0:
                best = (j - off, 800)
                break
    if best is None:
        print("FAIL: returned audio does not align to any position in the sent "
              "pattern - the payload was altered, not merely delayed.")
        diffs = sum(1 for a, b in zip(sent, received) if a != b)
        print("  naive byte diff over %d bytes: %d differing (%.1f%%)"
              % (min(len(sent), len(received)), diffs,
                 100.0 * diffs / max(1, min(len(sent), len(received)))))
        return 1

    off = best[0]
    print("alignment: received[0] == sent[%d]" % off)
    a = sent[off:]
    b = bytes(received[:len(a)])
    n = min(len(a), len(b))
    a, b = a[:n], b[:n]
    diffs = [i for i in range(n) if a[i] != b[i]]
    print("compared %d bytes (%.2f s of audio)" % (n, n / 8000.0))
    if not diffs:
        print("RESULT: BYTE-EXACT. Every u-law codeword returned unchanged.")
        return 0
    print("RESULT: %d of %d bytes differ (%.3f%%)" % (len(diffs), n, 100.0 * len(diffs) / n))
    for i in diffs[:12]:
        print("  offset %6d: sent 0x%02X  got 0x%02X" % (off + i, a[i], b[i]))
    runs = []
    start = prev = diffs[0]
    for i in diffs[1:]:
        if i != prev + 1:
            runs.append((start, prev))
            start = i
        prev = i
    runs.append((start, prev))
    print("  %d differing run(s); longest %d bytes"
          % (len(runs), max(e - s + 1 for s, e in runs)))
    return 1


def main():
    global SERVER, PORT, USER, PASSWD, TARGET, NPKTS
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--server", default="asterisk.net.cryan.nz")
    ap.add_argument("--port", type=int, default=5060)
    ap.add_argument("--user", default="6001")
    ap.add_argument("--password", default="6001")
    ap.add_argument("--target", default="9099",
                    help="extension running Answer()+Echo()")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--law", choices=("ulaw", "alaw"), default="ulaw")
    ap.add_argument("--expect-488", action="store_true",
                    help="invert the status: pass when the endpoint refuses "
                         "the offer, which is what a law-pinned endpoint "
                         "should do for the law it does not accept")
    args = ap.parse_args()

    SERVER, PORT = args.server, args.port
    USER, PASSWD = args.user, args.password
    TARGET = args.target
    NPKTS = max(1, int(args.seconds * 1000 / PTIME_MS))

    pt, name = (0, "PCMU") if args.law == "ulaw" else (8, "PCMA")
    print("probing %s@%s:%d as %s with %s, %.1f s"
          % (TARGET, SERVER, PORT, USER, name, args.seconds))
    rc = run(pt, name)

    if args.expect_488:
        if rc != 0:
            print("EXPECTED REFUSAL: the endpoint did not accept %s." % name)
            return 0
        print("UNEXPECTED: %s was accepted and echoed. The endpoint is not "
              "pinned to one law, so a bridge can still transcode." % name)
        return 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
