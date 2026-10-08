"""Exercise asymmetric and simultaneous stream integrity without modem hardware."""
import os
import socket
import threading
from stream_test import run

for kind in ('download', 'upload', 'duplex'):
    left, right = socket.socketpair()
    left.setblocking(False); right.setblocking(False)
    results = {}; errors = []
    def endpoint(role, sock):
        try: results[role] = run(sock.fileno(), role, kind, 1)
        except Exception as exc: errors.append(exc)
    threads = [threading.Thread(target=endpoint, args=(role, sock))
               for role, sock in (('caller', left), ('answerer', right))]
    for thread in threads: thread.start()
    for thread in threads: thread.join(10)
    assert all(not thread.is_alive() for thread in threads), 'deadlock'
    assert not errors, errors
    a, b = results['caller'], results['answerer']
    assert a['ok'] and b['ok']
    assert a['tx_bytes'] == b['rx_bytes'] and b['tx_bytes'] == a['rx_bytes']
    assert a['rx_bytes'] if kind != 'upload' else b['rx_bytes']
    left.close(); right.close()
    print(kind, 'PASS')

# A CRC-valid but wrong payload must fail independently of transport framing.
import zlib
from stream_test import HEADER, BLOCK
left, right = socket.socketpair()
left.setblocking(False)
bad = bytes(BLOCK)
right.sendall(HEADER.pack(b'EIC1', 0, BLOCK, zlib.crc32(bad)) + bad +
              HEADER.pack(b'EIC1', 1, 0, 0) + b'MENU')
result = run(left.fileno(), 'caller', 'download', 1)
assert not result['ok'] and result['errors'] == 1
assert left.recv(4) == b'MENU', 'binary receiver consumed following menu bytes'
left.close(); right.close()
print('corruption detection and menu boundary PASS')

# Exercise the answerer's actual menu handoff and result framing.
import json
from bri_test_answerer import timed_stream
left, right = socket.socketpair()
left.setblocking(False); right.setblocking(False)
class TestPort:
    fd = right.fileno()
    answers = iter(('duplex', '1'))
    def prompt(self, text): return next(self.answers)
    def send(self, data):
        if isinstance(data, str): data = data.encode()
        right.sendall(data)
    def alive(self): pass
errors = []
def answer():
    try: timed_stream(TestPort(), {'device': '/dev/ttyds1'})
    except Exception as exc: errors.append(exc)
thread = threading.Thread(target=answer)
thread.start()
import select
ready = bytearray()
while not ready.endswith(b'\r\n'):
    assert select.select([left], [], [], 5)[0]
    ready.extend(left.recv(1))
assert ready == b'STREAM READY\r\n'
result = run(left.fileno(), 'caller', 'duplex', 1)
thread.join(5)
assert not thread.is_alive() and not errors, errors
text = left.recv(4096)
peer = json.loads(text.split(b'STREAM RESULT ')[1].strip())
assert peer['device'] == '/dev/ttyds1' and peer['ok'] and result['ok']
assert peer['rx_bytes'] == result['tx_bytes'] and peer['tx_bytes'] == result['rx_bytes']
left.close(); right.close()
print('answerer menu handoff PASS')
