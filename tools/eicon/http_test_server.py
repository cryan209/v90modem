#!/usr/bin/env python3
"""Temporary HTTP integrity endpoint bound only to a PPP peer address."""
import argparse
import errno
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import time

MAX_BYTES = 16 * 1024 * 1024


def payload(size):
    return hashlib.shake_256(b'eicon-ppp-http-v1').digest(size)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        try:
            size = int(self.path.removeprefix('/download/'))
            if not self.path.startswith('/download/') or not 1 <= size <= MAX_BYTES:
                raise ValueError()
        except ValueError:
            self.send_error(400); return
        data = payload(size)
        self.send_response(200)
        self.send_header('Content-Type', 'application/octet-stream')
        self.send_header('Content-Length', str(size))
        self.send_header('X-SHA256', hashlib.sha256(data).hexdigest())
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if self.path != '/upload' or not 1 <= size <= MAX_BYTES:
                raise ValueError()
        except ValueError:
            self.send_error(400); return
        self.connection.settimeout(900)
        digest = hashlib.sha256(); count = 0
        while count < size:
            block = self.rfile.read(min(65536, size-count))
            if not block:
                self.send_error(400, 'Incomplete upload'); return
            digest.update(block); count += len(block)
        response = json.dumps(dict(bytes=count, sha256=digest.hexdigest())).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(response)))
        self.end_headers()
        self.wfile.write(response)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bind', required=True)
    ap.add_argument('--port', type=int, default=19890)
    args = ap.parse_args()
    # Wait for IPCP to create this address. Never listen on the LAN wildcard.
    end = time.monotonic()+600
    while True:
        try:
            server = ThreadingHTTPServer((args.bind, args.port), Handler)
            break
        except OSError as exc:
            if exc.errno != errno.EADDRNOTAVAIL or time.monotonic() > end:
                raise
            time.sleep(.2)
    print('HTTP READY', args.bind, args.port, flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
