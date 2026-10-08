"""Bounded full-duplex incompressible DTE test, shared by caller and answerer.

Wire frames: EIC1, sequence u32, length u32, CRC32 u32, payload. A zero
length ends each direction. Payload is SHAKE256(direction + sequence).
No modem DSP or sample accounting is involved.
"""
import hashlib
import os
import select
import struct
import time
import zlib

HEADER = struct.Struct('!4sIII')
BLOCK = 1024


def payload(direction, sequence):
    return hashlib.shake_256(direction.encode() + struct.pack('!I', sequence)).digest(BLOCK)


def run(fd, role, kind, seconds, alive=lambda: None, progress=lambda stats: None):
    if role not in ('caller', 'answerer') or kind not in ('download', 'upload', 'duplex'):
        raise ValueError('invalid role or test')
    if not 1 <= seconds <= 3600:
        raise ValueError('seconds must be 1..3600')
    tx_direction = 'upload' if role == 'caller' else 'download'
    rx_direction = 'download' if role == 'caller' else 'upload'
    enabled = kind == 'duplex' or kind == tx_direction
    tx_seq = rx_seq = 0
    pending = bytearray()
    incoming = bytearray()
    tx_done = rx_done = False
    start = time.monotonic()
    last_progress = start
    last_rx = None
    stats = dict(kind=kind, requested_seconds=seconds, tx_bytes=0, rx_bytes=0,
                 frames=0, errors=0, first_rx_seconds=None, last_rx_seconds=None, max_rx_gap_seconds=0)
    while not (tx_done and rx_done):
        now = time.monotonic()
        if now-start > seconds+90:
            raise TimeoutError('stream completion timeout')
        alive()
        if not pending and not tx_done:
            data = payload(tx_direction, tx_seq) if enabled and now-start < seconds else b''
            pending.extend(HEADER.pack(b'EIC1', tx_seq, len(data), zlib.crc32(data)) + data)
            ending = not data
        readers, writers, _ = select.select([fd] if not rx_done else [],
                                           [fd] if pending else [], [], .2)
        if writers:
            try:
                count = os.write(fd, pending)
            except BlockingIOError:
                count = 0
            del pending[:count]
            if not pending:
                if ending:
                    tx_done = True
                else:
                    stats['tx_bytes'] += BLOCK
                    tx_seq += 1
        if readers:
            try:
                needed = HEADER.size-len(incoming) if len(incoming) < HEADER.size else HEADER.unpack_from(incoming)[2]+HEADER.size-len(incoming)
                data = os.read(fd, needed)
            except BlockingIOError:
                data = None
            if data == b'':
                raise EOFError('stream closed')
            if data:
                incoming.extend(data)
            while len(incoming) >= HEADER.size:
                magic, sequence, length, crc = HEADER.unpack_from(incoming)
                if magic != b'EIC1' or length not in (0, BLOCK):
                    raise ValueError('invalid stream frame header')
                if len(incoming) < HEADER.size+length:
                    break
                body = bytes(incoming[HEADER.size:HEADER.size+length])
                del incoming[:HEADER.size+length]
                if sequence != rx_seq or zlib.crc32(body) != crc:
                    stats['errors'] += 1
                if not length:
                    rx_done = True
                    if incoming:
                        raise ValueError('data after end frame')
                    break
                if body != payload(rx_direction, sequence):
                    stats['errors'] += 1
                stats['rx_bytes'] += length
                stats['frames'] += 1
                elapsed = time.monotonic()-start
                if stats['first_rx_seconds'] is None:
                    stats['first_rx_seconds'] = elapsed
                if last_rx is not None:
                    stats['max_rx_gap_seconds'] = round(max(stats['max_rx_gap_seconds'], elapsed-last_rx), 3)
                last_rx = elapsed
                stats['last_rx_seconds'] = elapsed
                rx_seq = sequence+1
        if now-last_progress >= 30:
            progress(dict(stats, elapsed_seconds=round(now-start, 3)))
            last_progress = now
    stats['elapsed_seconds'] = round(time.monotonic()-start, 3)
    stats['rx_active_seconds'] = round((stats['last_rx_seconds'] or 0) -
                                      (stats['first_rx_seconds'] or 0), 3)
    stats['tx_bytes_per_second'] = round(stats['tx_bytes']/max(stats['elapsed_seconds'], .001), 1)
    stats['rx_bytes_per_second'] = round(stats['rx_bytes']/max(stats['elapsed_seconds'], .001), 1)
    stats['ok'] = stats['errors'] == 0
    return stats
