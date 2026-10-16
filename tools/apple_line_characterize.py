#!/usr/bin/env python3
"""Offline noise and response diagnostics for retained Si3056 §5.30 probes.

Quiet noise is measured independently of the known signal. Active residual
also contains model error and distortion; neither quantity is a modem BER.
No audio samples or live DSP settings are modified.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy import signal
from apple_loopback_spectrum import CASES, ROOT, fit, db
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

FS = 8000


def psd(x):
    return signal.welch(x, FS, nperseg=1024, noverlap=512, detrend='constant')


def band_power(f, p, lo=300, hi=3400):
    return float(np.sum(p[(f >= lo) & (f <= hi)]) * (f[1] - f[0]))


def characterize(name, out):
    directory = ROOT / CASES[name][0]
    meta = json.loads((directory / 'result.json').read_text())
    tx = np.fromfile(directory / 'tx.s16', dtype='<i2').astype(float)
    rx = np.fromfile(directory / 'rx.s16', dtype='<i2').astype(float)
    lag = meta['lag_samples']
    # Exclude startup and at least 256 samples of settling after the probe.
    quiet_windows = [(2000, 7000), (22000, 23500)]
    quiet = []
    for a, b in quiet_windows:
        assert b <= len(rx)
        assert np.all(tx[a-lag-256:b-lag+256] == 0)
        f, p = psd(rx[a:b])
        quiet.append(dict(start=a, end=b, dc=float(rx[a:b].mean()),
                          rms_ac=float(rx[a:b].std()),
                          band_rms=float(np.sqrt(band_power(f, p)))))
    noise = np.mean([psd(rx[a:b])[1] for a, b in quiet_windows], axis=0)
    ix = np.arange(8256, 19700)
    train = np.arange(5800)
    test = np.arange(6500, len(ix))
    target = rx[ix + lag]
    f, active = psd(target[test])
    rows = {}
    # Keep at least a 188-sample gap even with the longest training windows.
    for taps in [65, 257, 513]:
        half = taps // 2
        features = tx[ix[:, None] + np.arange(-half, half+1)]
        score, predicted, coeff, bias = fit(features, target, train, test)
        _, residual = psd((target-predicted)[test])
        rows[str(taps)] = dict(score, bias=float(bias),
            residual_band_rms=float(np.sqrt(band_power(f, residual))),
            active_band_residual_snr_db=float(db(band_power(f, active) / band_power(f, residual))))
    # Frequency response comes from the training-only FIR, avoiding Welch
    # window bias. Centered taps are evaluated with their actual offsets.
    hz, h = signal.freqz(coeff[::-1], worN=4096, fs=FS)
    h *= np.exp(2j*np.pi*hz/FS*half)
    gain = 20*np.log10(np.maximum(np.abs(h), 1e-15))
    ref = float(np.interp(1000, hz, gain))
    relative = gain-ref
    band = (hz >= 300) & (hz <= 3400)
    phase = np.unwrap(np.angle(h))
    slope, intercept = np.polyfit(hz[band], phase[band], 1)
    phase_error = (phase-(slope*hz+intercept))*180/np.pi
    cutoffs = {}
    for attenuation in [3, 6, 10, 20]:
        hits = np.flatnonzero((hz > 1000) & (relative <= -attenuation))
        cutoffs[str(attenuation)] = float(hz[hits[0]]) if len(hits) else None
    result = dict(name=name, source=str(directory), quiet=quiet,
        quiet_band_rms=float(np.sqrt(band_power(f, noise))),
        active_band_rms=float(np.sqrt(band_power(f, active))),
        quiet_referenced_band_snr_db=float(db(band_power(f, active)/band_power(f, noise))),
        forward_models=rows, relative_cutoff_hz=cutoffs,
        passband_ripple_db=float(np.ptp(relative[band])),
        phase_delay_us=float(-slope/(2*np.pi)*1e6),
        phase_error_p5_p95_degrees=[float(v) for v in np.percentile(phase_error[band], [5,95])],
        relative_gain_db={str(v):float(np.interp(v,hz,relative)) for v in [100,300,1000,2000,3000,3200,3400,3500,3600,3800,3900]})
    np.savetxt(out/(name+'-response.csv'), np.column_stack([hz,gain,relative,-relative,phase_error]),
        delimiter=',', header='frequency_hz,gain_db,relative_to_1khz_db,ideal_inverse_relative_db,phase_error_degrees',comments='')
    return result, (hz, relative, phase_error, f, active, noise, residual)


def self_check():
    rng = np.random.default_rng(3056)
    x = rng.normal(size=120000)
    f, p = psd(x)
    expected = np.var(x) * (3400-300) / (FS/2)
    assert abs(band_power(f,p)/expected-1) < .04
    # A known two-tap response checks centered-FIR orientation and phase.
    coefficients = np.array([.2, .7, .1])
    hz, h = signal.freqz(coefficients[::-1], worN=1024, fs=FS)
    h *= np.exp(2j*np.pi*hz/FS)
    direct = sum(c*np.exp(2j*np.pi*hz/FS*offset)
                 for c,offset in zip(coefficients,[-1,0,1]))
    assert np.max(np.abs(h-direct)) < 1e-12
    print('Synthetic checks passed: band noise power and centered FIR response.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--self-check', action='store_true')
    args=parser.parse_args()
    if args.self_check:
        self_check()
        if args.out is None: return
    if args.out is None: parser.error('--out is required unless using --self-check')
    args.out.mkdir(parents=True,exist_ok=True)
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    summaries=[]
    for name in ['isolation-13','analog-13','isolation-14','analog-14']:
        row,(hz,response,phase,f,active,noise,residual)=characterize(name,args.out)
        summaries.append(row)
        axes[0,0].plot(hz,response,label=name)
        axes[0,1].plot(hz,-response,label=name)
        if name.startswith('analog'):
            axes[1,0].plot(f,db(noise),label=name+' quiet')
            axes[1,0].plot(f,db(residual),ls='--',label=name+' active residual')
        axes[1,1].plot(hz,phase,label=name)
        print(json.dumps(row),flush=True)
    axes[0,0].set(title='Response relative to 1 kHz',ylabel='Gain (dB)',ylim=(-35,2))
    axes[0,1].set(title='Ideal inverse magnitude: noise boost rises sharply',ylabel='Required boost (dB)',ylim=(-2,35))
    axes[1,0].set(title='Quiet noise versus 513-tap active residual',ylabel='Raw sample units squared / Hz (dB)',ylim=(-40,0))
    axes[1,1].set(title='Phase departure from passband linear delay',ylabel='Degrees',ylim=(-30,30))
    for ax in axes.flat:
        ax.set(xlim=(100,3950),xlabel='Frequency (Hz)');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Apple local loopback: independent quiet noise and measured filter response')
    fig.savefig(args.out/'noise-and-response.png',dpi=160)
    (args.out/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n')


if __name__ == '__main__':
    main()
