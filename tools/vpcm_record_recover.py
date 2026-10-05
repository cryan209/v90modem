#!/usr/bin/env python3
"""Experimental CP-constrained recovery from an 8 kHz analogue recording.

V.90 8.6.5 training and 5.4.5 shaping constrain the channel estimate. Strict
Table-14 CP records and a measured TRN2d/B1d position are required inputs.
B1d (8.6.1) is graded on the recorded audio; no B1d samples train the inverse
filter. Two filter lengths must both decode all 48 B1d frames before their
agreeing, erasure-free data prefix is examined for V.42 detection or HDLC FCS.
This tool never changes live G.711 processing or claims application payload
from a constellation fit, idle marks, or V.42 detection bytes.
"""
import argparse
import hashlib
import json
import os
import pathlib
import subprocess
import wave

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
CORE = ['v90.c', 'vpcm_cp.c', 'v90_dil_measure.c', 'v92_phase4_decode.c', 'v91.c']


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(command):
    p = subprocess.run(list(map(str, command)), capture_output=True, text=True, check=False)
    if p.returncode:
        raise RuntimeError(f"{' '.join(map(str, command))}: {p.stdout}{p.stderr}")
    return p.stdout


def build(out):
    library = ROOT / 'spandsp-master/src/.libs/libspandsp.a'
    if not library.exists():
        raise RuntimeError('Build the local SpanDSP library with make spandsp first.')
    try:
        flags = subprocess.check_output(['pkg-config', '--cflags', 'libtiff-4'], text=True).split()
    except (OSError, subprocess.CalledProcessError):
        flags = ['-I/opt/homebrew/opt/libtiff/include'] if pathlib.Path('/opt/homebrew/opt/libtiff/include').exists() else []
    binary = out / 'vpcm_record_probe'
    command = [os.environ.get('CC', 'cc'), '-Wall', '-Wextra', '-Werror',
               f'-I{ROOT}', f'-I{ROOT / "spandsp-master/src"}', *flags,
               ROOT / 'tools/vpcm_record_probe.c', *(ROOT / p for p in CORE),
               library, '-lm', '-o', binary]
    run(command)
    return binary, list(map(str, command))


def optimize_signs(y, h, x, passes=8):
    """Fit signs without assuming a foreign shaper uses our cost tie breaks."""
    error = x - np.convolve(y, h, 'same')
    radius = len(h) // 2
    for _ in range(passes):
        changes = 0
        for i in range(radius, len(y) - radius):
            delta = -2 * y[i] * h
            sl = slice(i - radius, i + radius + 1)
            if 2 * np.dot(error[sl], delta) > np.dot(delta, delta):
                y[i] = -y[i]
                error[sl] -= delta
                changes += 1
        if not changes:
            break
    return y


def inverse(x, y, taps, ridge, margin=0):
    half = taps // 2
    matrix = np.lib.stride_tricks.sliding_window_view(np.pad(x, (half, half)), taps)
    train = matrix[margin:len(y) - margin]
    wanted = y[margin:len(y) - margin]
    regularizer = ridge * np.mean(train * train) * len(train)
    coeff = np.linalg.solve(train.T @ train + np.eye(taps) * regularizer, train.T @ wanted)
    return matrix @ coeff


def crc16_octets(values):
    crc = 0xffff
    for value in values:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


def hdlc_frames(bits):
    b = list(bits)
    flag = [0, 1, 1, 1, 1, 1, 1, 0]
    flags = [i for i in range(len(b) - 7) if b[i:i + 8] == flag]
    frames = []
    for a, z in zip(flags, flags[1:]):
        dest, ones, valid = [], 0, True
        for value in b[a + 8:z]:
            if value > 1:
                valid = False
                break
            if ones == 5:
                if value:
                    valid = False
                    break
                ones = 0
                continue
            dest.append(value)
            ones = ones + 1 if value else 0
        if valid and len(dest) >= 24 and len(dest) % 8 == 0:
            octets = bytes(sum(dest[i + k] << k for k in range(8))
                           for i in range(0, len(dest), 8))
            frames.append(dict(bit=a, octets=len(octets),
                               fcs_ok=crc16_octets(octets) == 0xf0b8,
                               hex=octets.hex()))
    return dict(flags=len(flags), frames=frames,
                fcs_valid_frames=sum(f['fcs_ok'] for f in frames))


def detection_octets(bits):
    """Extract repeated complete ADPs with 8–16 mark bits between characters."""
    b = list(bits)
    e = [0, 1, 0, 1, 0, 0, 0, 1, 0, 1]
    c = [0, 1, 1, 0, 0, 0, 0, 1, 0, 1]
    pairs, positions = 0, []
    i = 0
    while i + 36 <= len(b):
        if b[i:i + 10] != e:
            i += 1
            continue
        j = i + 10
        while j < len(b) and b[j] == 1:
            j += 1
        if not 8 <= j - i - 10 <= 16 or b[j:j + 10] != c:
            i += 1
            continue
        k = j + 10
        while k < len(b) and b[k] == 1:
            k += 1
        if 8 <= k - j - 10 <= 16:
            pairs += 1
            positions.append(i)
            i = k
        else:
            i += 1
    return b'EC' * pairs, positions


def recover(args):
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    binary, build_command = build(out)
    with wave.open(str(args.input), 'rb') as w:
        if w.getframerate() != 8000 or w.getsampwidth() != 2:
            raise ValueError('Expected an 8 kHz 16-bit PCM WAV.')
        channel = 0 if args.channel == 'L' else 1
        if channel >= w.getnchannels():
            raise ValueError('Requested channel is absent.')
        x = np.frombuffer(w.readframes(w.getnframes()), '<i2').reshape(-1, w.getnchannels())[:, channel].astype(float)
    if not 0 <= args.training_start < args.b1_start < len(x) - 288:
        raise ValueError('Training/B1d positions fall outside the recording.')
    if args.training_start + 4096 >= args.b1_start:
        raise ValueError('The training window must precede B1d without overlap.')
    reference = out / 'trn2d.reference.s16'
    cpt_validation = run([binary, 'reference', args.cpt, reference])
    b1_reference = out / 'b1d.reference.s16'
    cp_validation = run([binary, 'reference', args.cp, b1_reference, 'b1'])
    levels = [[] for _ in range(6)]
    for line in run([binary, 'reference', args.cp]).splitlines():
        if line.startswith('slot='):
            fields = dict(item.split('=') for item in line.split())
            levels[int(fields['slot'])].append((int(fields['level']), int(fields['codec_level'])))
    raw = args.cp.read_bytes()
    drn = sum(raw[20 + i] << i for i in range(5))
    D = drn + 20
    data_rate = D * 8000 / 6
    target = x[args.training_start:]
    ref = np.fromfile(reference, '<i2').astype(float)
    y = ref[:4096].copy()
    if np.std(target[:4096]) < 1:
        raise ValueError('The training window contains no usable signal.')
    h = np.zeros(17)
    h[8] = np.std(target[:4096]) / np.std(y)
    for _ in range(15):
        y = optimize_signs(y, h, target[:4096])
        a = np.lib.stride_tricks.sliding_window_view(y, 17)[:, ::-1]
        h = np.linalg.lstsq(a, target[8:4088], rcond=None)[0]
    preliminary = inverse(target, y, 129, .001)
    beam_input = out / 'training-input.f64'
    preliminary[:4092].astype('<f8').tofile(beam_input)
    training = out / 'training-constrained.s16'
    training_summary = run([binary, 'training', args.cpt, reference, beam_input, training])
    y = np.fromfile(training, '<i2').astype(float)
    expected = np.fromfile(b1_reference, '<i2').astype(float)[:288]
    codec_expected = expected.copy()
    for i in range(288):
        pairs = np.array(levels[i % 6])
        k = np.argmin(abs(pairs[:, 0] - abs(expected[i])))
        codec_expected[i] = np.copysign(pairs[k, 1], expected[i])
    branches, accepted = [], []
    for taps in [257, 513]:
        eq = inverse(target, y, taps, 1e-6, margin=64)[args.b1_start - args.training_start:]
        gain, offset = np.linalg.lstsq(np.column_stack([abs(codec_expected[:144]), np.ones(144)]), abs(eq[:144]), rcond=None)[0]
        normalized = np.copysign(np.maximum(0, (abs(eq) - offset) / gain), eq)
        selected = np.empty(len(eq), dtype='<i2')
        for slot in range(6):
            pairs = np.array(levels[slot])
            values = normalized[slot::6]
            indices = np.argmin(abs(abs(values)[:, None] - pairs[:, 1]), axis=1)
            selected[slot::6] = np.copysign(pairs[indices, 0], values).astype('<i2')
        stream = out / f'levels-{taps}.s16'
        selected.tofile(stream)
        branch_accepted = None
        for seed in [0, 1]:
            bp = out / f'candidate-{taps}-seed{seed}.bits'
            summary = run([binary, 'demap', args.cp, stream, bp, seed])
            bits = bp.read_bytes()
            errors = [i for i, bit in enumerate(bits[:48 * D]) if bit != 1]
            row = dict(taps=taps, seed=seed, b1_bits=48 * D,
                       b1_error_bits=errors, gain=float(gain), offset=float(offset),
                       b1_magnitude_correlation=float(np.corrcoef(abs(eq[:288]), abs(codec_expected))[0, 1]),
                       demap=summary, candidate_file=bp.name)
            branches.append(row)
            if not errors:
                branch_accepted = bits[48 * D:]
        if branch_accepted is not None:
            accepted.append(branch_accepted)
    result = dict(input=str(args.input.resolve()), input_sha256=digest(args.input),
                  channel=args.channel, cpt_sha256=digest(args.cpt), cp_sha256=digest(args.cp),
                  training_start_sample=args.training_start, b1_start_sample=args.b1_start,
                  data_start_sample=args.b1_start + 288, data_rate_bps=data_rate,
                  cpt_validation=cpt_validation, cp_validation=cp_validation,
                  training=training_summary, branches=branches, b1_qualified=len(accepted) == 2,
                  application_bytes=0, build_command=build_command,
                  helper_sha256=digest(binary), spandsp_sha256=digest(ROOT / 'spandsp-master/src/.libs/libspandsp.a'),
                  source_hashes={p: digest(ROOT / p) for p in ['tools/vpcm_record_probe.c', 'tools/vpcm_record_recover.py', *CORE]},
                  limitation='Detection octets are modem negotiation; no application bytes are inferred.')
    if len(accepted) == 2:
        a, b = accepted
        n = min(len(a), len(b))
        end = next((i for i in range(n) if a[i] != b[i] or a[i] > 1), n)
        prefix = a[:end]
        data = out / 'agreeing-data.bits'
        data.write_bytes(prefix)
        octets, positions = detection_octets(prefix)
        (out / 'v42-adp.bin').write_bytes(octets)
        result.update(agreeing_data_bits=end, hdlc=hdlc_frames(prefix),
                      adp_pairs=len(octets) // 2, detection_octets=len(octets), adp_bit_positions=positions)
        p = subprocess.run([str(binary), 'v42', str(data), str(round(data_rate))], capture_output=True, text=True)
        result['native_v42'] = dict(exit=p.returncode, output=p.stdout + p.stderr)
    (out / 'recovery.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=pathlib.Path)
    parser.add_argument('--channel', choices=['L', 'R'], default='L')
    parser.add_argument('--cpt', type=pathlib.Path, required=True, help='Strict Table-14 CPt, one byte per bit')
    parser.add_argument('--cp', type=pathlib.Path, required=True, help='Strict final CP, one byte per bit')
    parser.add_argument('--training-start', type=int, required=True, help='First mapped TRN2d sample')
    parser.add_argument('--b1-start', type=int, required=True, help='First B1d sample')
    parser.add_argument('--output', type=pathlib.Path, required=True)
    args = parser.parse_args()
    try:
        result = recover(args)
    except (RuntimeError, ValueError, OSError, np.linalg.LinAlgError) as error:
        parser.exit(1, f'{error}\n')
    print(json.dumps({key: result.get(key) for key in ['b1_qualified', 'agreeing_data_bits', 'adp_pairs', 'detection_octets', 'application_bytes']}))


if __name__ == '__main__':
    main()
