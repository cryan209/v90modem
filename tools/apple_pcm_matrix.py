#!/usr/bin/env python3
"""Native Apple USB modem bench: exhaustive PCM excitation and RX modelling.

Requires numpy/scipy and the local apple_usb_modem_audio/probe/coupler tools.
The two modems must already be connected by the user's powered direct loop.
This measures the complete DAC/line/ADC path, not an individual converter.
No modem engine, echo cancellation, AGC or live resampler is used.
"""
import argparse
import concurrent.futures
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np
from scipy.linalg import lstsq
from scipy.optimize import minimize_scalar
from scipy.signal import butter, correlate, resample_poly, sosfilt
from numpy.lib.stride_tricks import sliding_window_view

RATES = [7200, 8000, 8229, 8400, 9000, 9600, 10286]
ROOT = Path(__file__).resolve().parent.parent
UNITS = {'A': ('1:2', '1130000'), 'B': ('0:3', '000000')}


def g711_levels(law):
    c = np.arange(256, dtype=np.int32)
    if law == 'ulaw':
        u = c ^ 255
        t = ((u & 15)*8 + 132) << ((u >> 4) & 7)
        return np.where(u & 128, 132-t, t-132)
    a = c ^ 0x55
    t = (a & 15)*16
    seg = (a >> 4) & 7
    t = np.where(seg == 0, t+8, np.where(seg == 1, t+264, (t+264) << (seg-1).clip(0)))
    return np.where(a & 128, t, -t)


def power_limited_alphabet(law, magnitudes):
    """Maximise the minimum spacing with uniform RMS <= V.90 Table 15's 2540.

    For a proposed spacing, the earliest legal ascending points minimise
    power. The first positive point is at least half a spacing from zero,
    keeping the two opposite signs just as far apart as adjacent magnitudes.
    This is a bench alphabet, not CP's six-slot mapper/power-shaping logic.
    """
    levels = g711_levels(law)
    positive = np.unique(levels[levels > 0])
    lo, hi, best = 0.0, 65536.0, None
    for _ in range(40):
        gap = (lo+hi)/2
        selected = []
        threshold = gap/2
        for v in positive:
            if v >= threshold:
                selected.append(int(v)); threshold = v+gap
                if len(selected) == magnitudes:
                    break
        if len(selected) == magnitudes and np.mean(np.square(selected)) <= 2540**2:
            best = selected; lo = gap
        else:
            hi = gap
    if best is None:
        raise ValueError(f'no {magnitudes}-magnitude alphabet in {law}')
    return np.array([np.flatnonzero(levels == sign*v)[0] for sign in [-1, 1] for v in best])


def generate(out, rates, profile='levels'):
    out.mkdir(parents=True, exist_ok=True)
    for rate in rates:
        existing_manifest = out/f'{rate}-signal.json'
        if existing_manifest.exists() and json.loads(existing_manifest.read_text()).get('profile', 'levels') != profile:
            raise ValueError(f'{out}: existing waveform profile differs; use a new output directory')
        rng = np.random.default_rng(20261017)
        pieces, sections = [], []
        code_files = {}
        count = 0

        def add(name, samples, **meta):
            nonlocal count
            samples = np.clip(np.rint(samples), -32768, 32767).astype('<i2')
            pieces.append(samples)
            sections.append(dict(name=name, start=count, count=len(samples), **meta))
            count += len(samples)

        add('quiet', np.zeros(rate))
        add('sync', rng.choice([-6000, 6000], size=rate//2))
        add('gap', np.zeros(rate//4))
        add('linear_train', rng.integers(-8000, 8001, size=rate*2))
        tone_levels = [8192, 24000, 32000] if profile != 'levels' else [64, 128, 256, 512, 1024, 2048, 4096, 8192, 12000, 16000, 24000, 32000]
        for level in tone_levels:
            t = np.arange(round(rate*.25))/rate
            add('tone', level*np.sin(2*np.pi*1000*t), peak=level, hz=1000)
        # Each of all 65536 signed input values occurs twice, independently
        # shuffled. A DC staircase would not measure this AC-coupled path.
        for repeat in range(2 if profile == 'levels' else 0):
            add('pcm16', rng.permutation(np.arange(-32768, 32768)), repeat=repeat)
        for law in ['ulaw', 'alaw']:
            sizes = [4, 8, 12, 16, 24, 32, 48, 64] if profile == 'sweep' else [None]
            for size in sizes:
                name = f'{law}-{2*size}levels' if size else law
                alphabet = np.arange(256)
                if profile == 'sweep':
                    alphabet = power_limited_alphabet(law, size)
                elif profile == 'subset':
                    levels = g711_levels(law)
                    targets = np.array([396, 780, 1180, 1564, 1980, 2492, 3004, 3516])
                    targets = np.concatenate([-targets, targets])
                    alphabet = np.array([np.argmin(abs(levels-v)) for v in targets])
                codes = np.concatenate([rng.permutation(alphabet) for _ in range(16384//len(alphabet))])
                code_files[name] = codes.astype('u1').tobytes()
                # OFFLINE signal construction, never a G.711 bearer path.
                v = resample_poly(g711_levels(law)[codes].astype(float), rate, 8000)
                add(name, v, law=law, symbol_rate=8000, symbols=len(codes),
                    alphabet=[int(c) for c in alphabet],
                    tx_symbol_rms=float(np.sqrt(np.mean(g711_levels(law)[alphabet].astype(float)**2))),
                    construction_clips=int(np.sum((v < -32768) | (v > 32767))))
        add('tail', np.zeros(rate))
        x = np.concatenate(pieces)
        path = out/f'{rate}-tx.s16'
        if path.exists() and path.read_bytes() != x.tobytes():
            raise ValueError(f'{path}: refusing to overwrite an existing, different reference waveform')
        for law, data in code_files.items():
            target = out/f'{rate}-{law}.codes'
            if target.exists() and target.read_bytes() != data:
                raise ValueError(f'{target}: refusing to replace a different codeword reference')
        for law, data in code_files.items():
            (out/f'{rate}-{law}.codes').write_bytes(data)
        path.write_bytes(x.tobytes())
        (out/f'{rate}-zero.s16').write_bytes(np.zeros(len(x)+rate, dtype='<i2').tobytes())
        manifest = dict(rate=rate, bits=16, channels=1, profile=profile, samples=len(x),
                        sha256=hashlib.sha256(path.read_bytes()).hexdigest(), sections=sections)
        (out/f'{rate}-signal.json').write_text(json.dumps(manifest, indent=2)+'\n')


def capture(out, rates, profile='levels'):
    generate(out, rates, profile)
    records = []

    def usb(tag, args, tool='apple_usb_modem_probe'):
        p = subprocess.run([str(ROOT/tool), *args], env=dict(os.environ, APPLE_MODEM_ADDR=UNITS[tag][0]),
                           text=True, capture_output=True, timeout=15)
        records.append(dict(unit=tag, args=args, rc=p.returncode, stdout=p.stdout, stderr=p.stderr))
        return p

    def status():
        for tag in UNITS:
            p = usb(tag, ['--line-status'])
            s = json.loads(p.stdout)
            if p.returncode or s['overcurrent'] or not s['hardware_protection_enabled'] or not s['hook_register'] & 1:
                raise RuntimeError(f'{tag}: line protection/hook check failed: {s}')

    def audio(tag, rate, source, dest):
        p = subprocess.run([str(ROOT/'apple_usb_modem_audio'), 'play', str(rate), str(source), str(dest)],
                           env=dict(os.environ, APPLE_MODEM_AUDIO_UID=UNITS[tag][1]),
                           text=True, capture_output=True, timeout=65)
        dest.with_suffix('.log').write_text(p.stdout+p.stderr)
        if p.returncode or 'render errors 0' not in p.stdout or 'CUT SHORT' in p.stdout:
            raise RuntimeError(f'{tag} audio failed: {p.stdout}{p.stderr}')
        return dict(unit=tag, rc=p.returncode, stdout=p.stdout, stderr=p.stderr)

    try:
        for tag in UNITS:
            usb(tag, ['--hook', 'on'], 'apple_usb_modem_coupler')
        status()
        for rate in rates:
            for src in UNITS:
                dst = 'B' if src == 'A' else 'A'
                prefix = out/f'{rate}-{src}to{dst}'
                if prefix.with_suffix('.json').exists():
                    continue
                with concurrent.futures.ThreadPoolExecutor(2) as pool:
                    peer = pool.submit(audio, dst, rate, out/f'{rate}-zero.s16', Path(str(prefix)+'-rx.s16'))
                    time.sleep(.25)
                    local = audio(src, rate, out/f'{rate}-tx.s16', Path(str(prefix)+'-local.s16'))
                    remote = peer.result()
                status()
                prefix.with_suffix('.json').write_text(json.dumps(dict(rate=rate, src=src, dst=dst,
                    audio_binary_sha256=hashlib.sha256((ROOT/'apple_usb_modem_audio').read_bytes()).hexdigest(),
                    local=local, peer=remote), indent=2)+'\n')
                print(f'{rate} Hz {src}->{dst}: complete', flush=True)
    finally:
        for tag in UNITS:
            usb(tag, ['--hook', 'off'], 'apple_usb_modem_coupler')
            usb(tag, ['--monitor', 'off'])
            usb(tag, ['--line-status'])
        (out/f'usb-{time.time_ns()}.json').write_text(json.dumps(records, indent=2)+'\n')


def db_ratio(signal, error):
    return float(10*np.log10(max(np.mean(signal*signal), 1e-20)/max(np.mean(error*error), 1e-20)))


def tone_fit(z, rate):
    t = np.arange(len(z))/rate

    def fit(freq):
        q = np.column_stack([np.sin(2*np.pi*freq*t), np.cos(2*np.pi*freq*t), np.ones(len(t))])
        coef = np.linalg.lstsq(q, z, rcond=None)[0]
        return q@coef, coef

    opt = minimize_scalar(lambda f: np.mean((z-fit(f)[0])**2), bounds=(995, 1005), method='bounded')
    estimated, coef = fit(opt.x)
    return float(opt.x), estimated, coef


def clock_align(y, ratio):
    """Offline, bandlimited clock registration; never used on live audio.

    A sub-ppm clock difference becomes a large broadband modelling residual
    over a 30-second capture. Estimate it from independent high-level tones,
    then retain both the estimate and unregistered observations in the report.
    """
    result = np.empty(len(y))
    offsets = np.arange(-64, 65)
    window = np.kaiser(len(offsets), 10)
    for start in range(0, len(y), 4096):
        pos = np.arange(start, min(start+4096, len(y)))*ratio
        base = np.floor(pos).astype(int)
        weights = np.sinc(pos[:, None]-base[:, None]-offsets)*window
        weights /= np.sum(weights, axis=1)[:, None]
        result[start:start+len(pos)] = np.sum(y[np.clip(base[:, None]+offsets, 0, len(y)-1)]*weights, axis=1)
    return result


def analyze(out, rates):
    """Fit channel memory before judging levels; score held-out observations."""
    rows = []
    for rate in rates:
        m = json.loads((out/f'{rate}-signal.json').read_text())
        x = np.fromfile(out/f'{rate}-tx.s16', dtype='<i2').astype(float)
        sections = m['sections']
        sync = next(s for s in sections if s['name'] == 'sync')
        train = next(s for s in sections if s['name'] == 'linear_train')
        nt = 129
        w = sliding_window_view(x, nt)
        for src, dst in [('A', 'B'), ('B', 'A')]:
            prefix = out/f'{rate}-{src}to{dst}'
            if not prefix.with_suffix('.json').exists():
                continue
            raw = np.fromfile(Path(str(prefix)+'-rx.s16'), dtype='<i2').astype(float)
            s0 = sync['start']
            reference = x[s0:s0+sync['count']]
            c = correlate(raw[:6*rate], reference, mode='valid', method='fft')
            lag = int(np.argmax(abs(c))) - s0
            if lag < 0 or lag+len(x) > len(raw):
                raise RuntimeError(f'{prefix}: incomplete aligned capture, lag={lag}')
            y = raw[lag:lag+len(x)]
            frequencies = []
            for s in sections:
                if s['name'] == 'tone' and s['peak'] >= 8192:
                    a = s['start']+round(rate*.05)
                    b = s['start']+s['count']-round(rate*.03)
                    frequencies.append(tone_fit(y[a:b], rate)[0])
            observed_hz = float(np.median(frequencies))
            ratio = 1000/observed_hz
            registered = clock_align(y, ratio)
            unregistered = y
            y = registered
            # Train on the first second, validate on the next second.
            lo, hi = train['start']+nt, train['start']+rate-nt
            h = lstsq(w[lo:hi], y[lo+nt//2:hi+nt//2], lapack_driver='gelsy')[0]
            predicted = correlate(x, h, mode='same', method='fft')
            v = slice(train['start']+rate+nt, train['start']+2*rate-nt)
            quiet = y[rate//4:3*rate//4]
            sos = butter(5, [300, 3400], btype='bandpass', fs=rate, output='sos')
            band_y = sosfilt(sos, y)
            band_error = sosfilt(sos, y-predicted)
            result = dict(rate=rate, direction=f'{src}->{dst}', lag_samples=lag,
                measured_clock_difference_ppm=(observed_hz/1000-1)*1e6,
                clock_registration='Offline 129-tap windowed-sinc, ratio measured on separate 1 kHz tone sections.',
                quiet_rms=float(np.std(quiet)), linear_validation_sndr_db=db_ratio(y[v], y[v]-predicted[v]),
                voiceband_quiet_rms=float(np.std(sosfilt(sos, quiet)[rate//10:])),
                voiceband_linear_validation_sndr_db=db_ratio(band_y[v], band_error[v]),
                tones=[], pcm16=[], g711={})
            np.save(Path(str(prefix)+'-channel.npy'), h)
            for s in sections:
                a, b = s['start'], s['start']+s['count']
                if s['name'] == 'tone':
                    z = y[a+round(rate*.05):b-round(rate*.03)]
                    frequency, estimated, coef = tone_fit(z, rate)
                    ac = estimated-coef[2]
                    result['tones'].append(dict(peak=s['peak'], frequency_hz=frequency,
                        gain=float(np.hypot(*coef[:2])/s['peak']), sndr_db=db_ratio(ac, z-estimated),
                        rail_fraction=float(np.mean(abs(unregistered[a+round(rate*.05):b-round(rate*.03)]) >= 32760)),
                        rx_peak=float(max(abs(unregistered[a:b])))))
                elif s['name'] == 'pcm16':
                    # A separate fit at full range distinguishes level-dependent
                    # distortion from the small-signal channel. Fit the first
                    # half and score the second: no in-sample residual score.
                    lo, mid, hi = a+nt, a+s['count']//2, b-nt
                    hh = lstsq(w[lo:mid], y[lo+nt//2:mid+nt//2], lapack_driver='gelsy')[0]
                    yp = w[mid:hi]@hh
                    actual = y[mid+nt//2:hi+nt//2]
                    result['pcm16'].append(dict(repeat=s['repeat'], distinct_tx_values=int(len(np.unique(x[a:b]))),
                        heldout_sndr_db=db_ratio(actual, actual-yp),
                        rx_rail_fraction=float(np.mean(abs(unregistered[a:b]) >= 32760)),
                        raw_min=int(min(unregistered[a:b])), raw_max=int(max(unregistered[a:b])),
                        raw_min_count=int(np.sum(unregistered[a:b] == min(unregistered[a:b]))),
                        raw_max_count=int(np.sum(unregistered[a:b] == max(unregistered[a:b]))),
                        registration_overshoot_fraction=float(np.mean(abs(y[a:b]) > 32767)),
                        small_signal_model_sndr_db=db_ratio(y[a+nt:b-nt], y[a+nt:b-nt]-predicted[a+nt:b-nt])))
                    # Every native TX value is retained, with its context-aware
                    # prediction and measured RX. Two observations/code cannot
                    # establish an individual code's noise distribution.
                    table = np.column_stack([x[a:b], unregistered[a:b], y[a:b], predicted[a:b]])
                    np.save(Path(str(prefix)+f'-pcm16-{s["repeat"]}.npy'), table)
                elif s.get('law', s['name']) in ['ulaw', 'alaw']:
                    tag = s['name']
                    law = s.get('law', tag)
                    z = resample_poly(y[a:b], 8000, rate)
                    codes = np.frombuffer((out/f'{rate}-{tag}.codes').read_bytes(), dtype='u1')
                    target = g711_levels(law)[codes].astype(float)
                    taps = 81
                    design = sliding_window_view(z, taps)/10000
                    count = min(len(design), len(target)-taps//2)
                    design = design[:count]
                    truth = target[taps//2:taps//2+count]
                    midpoint = count//2
                    xx, tt = design[:midpoint], truth[:midpoint]/10000
                    gram = xx.T@xx
                    ridge = 1e-4*np.trace(gram)/taps
                    eq = np.linalg.solve(gram+ridge*np.eye(taps), xx.T@tt)
                    estimates = design[midpoint:]@eq*10000
                    truth = truth[midpoint:]
                    held_codes = codes[taps//2+midpoint:taps//2+count]
                    levels = np.unique(g711_levels(law)[np.unique(codes)])
                    ii = np.clip(np.searchsorted(levels, estimates), 1, len(levels)-1)
                    chosen = np.where(abs(levels[ii]-estimates)<abs(levels[ii-1]-estimates), levels[ii], levels[ii-1])
                    per_level = []
                    for code, level in enumerate(g711_levels(law)):
                        take = held_codes == code
                        if not np.any(take):
                            continue
                        per_level.append(dict(code=code, tx_level=int(level), count=int(sum(take)),
                            estimate_mean=float(np.mean(estimates[take])), estimate_std=float(np.std(estimates[take])),
                            correct_fraction=float(np.mean(chosen[take] == level))))
                    with Path(str(prefix)+f'-{tag}-levels.csv').open('w', newline='') as f:
                        writer = csv.DictWriter(f, fieldnames=list(per_level[0]))
                        writer.writeheader(); writer.writerows(per_level)
                    result['g711'][tag] = dict(law=law, symbol_rate=8000, equalizer_taps=taps,
                        construction_clips=s['construction_clips'], heldout_symbols=len(truth),
                        heldout_sndr_db=db_ratio(truth, truth-estimates),
                        symbol_correct_fraction=float(np.mean(chosen == truth)),
                        alphabet_levels=len(levels), tx_symbol_rms=float(np.sqrt(np.mean(target**2))),
                        note='Offline supervised linear equalizer with the known transmit alphabet; not a negotiated rate.')
                    # Fit the null-preserving PR4 target independently. Adding
                    # the TRUE predecessor back is an oracle noise bound, not
                    # a realizable decoder; the C trellis below has no oracle.
                    pcm16k = resample_poly(y[a:b], 16000, rate)
                    pr_taps = 129
                    pr_design = sliding_window_view(pcm16k, pr_taps)[::2]/10000
                    center = (pr_taps-1)//4
                    npr = min(len(pr_design), len(target)-center)
                    pr_design = pr_design[:npr]
                    indices = np.arange(center, center+npr)
                    pr_truth = target[indices]-target[indices-2]
                    cut = npr//2
                    pr_h = lstsq(pr_design[:cut], pr_truth[:cut]/10000, lapack_driver='gelsy')[0]
                    pr_est = pr_design[cut:]@pr_h*10000
                    pr_error = pr_est-pr_truth[cut:]
                    reference_indices = indices[cut:]
                    level_rows = []
                    for code, level in enumerate(g711_levels(law)):
                        e = pr_error[codes[reference_indices] == code]
                        if not len(e):
                            continue
                        level_rows.append(dict(code=code, tx_level=int(level), count=len(e),
                            oracle_bias=float(np.mean(e)), oracle_std=float(np.std(e))))
                    with Path(str(prefix)+f'-{tag}-pr4-noise.csv').open('w', newline='') as f:
                        writer = csv.DictWriter(f, fieldnames=list(level_rows[0]))
                        writer.writeheader(); writer.writerows(level_rows)
                    chosen_levels = []
                    last_sigma = 0.0
                    for lr in sorted((lr for lr in level_rows if lr['tx_level'] > 0), key=lambda lr: lr['tx_level']):
                        level = lr['tx_level']
                        sigma = lr['oracle_std']+abs(lr['oracle_bias'])
                        previous = chosen_levels[-1] if chosen_levels else 0
                        if level-previous < 6*max(sigma, last_sigma):
                            continue
                        if np.mean(np.square(chosen_levels+[level])) > 2540**2:
                            continue
                        chosen_levels.append(level)
                        last_sigma = sigma
                    result['g711'][tag]['pr4_oracle'] = dict(
                        heldout_sndr_db=db_ratio(pr_truth[cut:], pr_error),
                        rms_error=float(np.sqrt(np.mean(pr_error**2))),
                        six_sigma_positive_levels=chosen_levels,
                        uniform_rms=float(np.sqrt(np.mean(np.square(chosen_levels)))) if chosen_levels else 0,
                        note='Optimistic model with true predecessor; six-sigma spacing and uniform RMS <=2540 (V.90 Table 15). Validate any subset on a new capture before claiming a usable constellation.')
                    # The actual PR4 decoder gets independent known-symbol
                    # training, then scores only unseen symbols. Candidates
                    # are the fixed, known alphabet, never the next true symbol.
                    pr4_tool = ROOT/'apple_pcm_pr4'
                    if pr4_tool.exists():
                        pcm16k_path = Path(str(prefix)+f'-{tag}-registered-16k.s16')
                        pcm16k_path.write_bytes(np.clip(np.rint(pcm16k), -32768, 32767).astype('<i2').tobytes())
                        level_path = out/f'{law}-levels.s16'
                        level_path.write_bytes(g711_levels(law).astype('<i2').tobytes())
                        proc = subprocess.run([str(pr4_tool), str(pcm16k_path), str(out/f'{rate}-{tag}.codes'), str(level_path)],
                                              capture_output=True, text=True, timeout=90, check=True)
                        pr4 = json.loads(proc.stdout)
                        Path(str(prefix)+f'-{tag}-pr4.json').write_text(json.dumps(pr4, indent=2)+'\n')
                        result['g711'][tag]['pr4'] = {k:val for k,val in pr4.items() if k != 'levels'}
            Path(str(prefix)+'-analysis.json').write_text(json.dumps(result, indent=2)+'\n')
            rows.append(result)
            print(rate, src+'->'+dst, 'linear %.1f dB'%result['linear_validation_sndr_db'],
                  'G711', {law:round(r['symbol_correct_fraction']*100, 1) for law, r in result['g711'].items()}, flush=True)
    (out/'summary.json').write_text(json.dumps(rows, indent=2)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['generate', 'capture', 'analyze'])
    p.add_argument('output', type=Path)
    p.add_argument('--rates', type=int, nargs='+', default=RATES, choices=RATES)
    p.add_argument('--profile', choices=['levels', 'subset', 'sweep'], default='levels')
    a = p.parse_args()
    if a.action == 'analyze':
        analyze(a.output.resolve(), a.rates)
    else:
        {'generate': generate, 'capture': capture}[a.action](a.output.resolve(), a.rates, a.profile)


if __name__ == '__main__':
    main()
