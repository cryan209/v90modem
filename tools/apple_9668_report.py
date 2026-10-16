#!/usr/bin/env python3
"""Recognize repeated 9668 playback; raw correlation is not a decoder BER."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy import signal
from g711_codec import ulaw_to_linear
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path);args=parser.parse_args();d=args.directory;fs=10286
    assert 'WriteFormat: ulaw' in (d/'pbx-channel.txt').read_text()
    assert 'WriteTranscode: No' in (d/'pbx-channel.txt').read_text()
    source=ulaw_to_linear(np.frombuffer((d/'usr-dil.ulaw').read_bytes(),dtype=np.uint8)).astype(float)
    y=np.fromfile(d/'rx.s16',dtype='<i2').astype(float)
    reference=signal.resample_poly(source,5143,4000);reference-=reference.mean();centered=y-y.mean()
    correlation=signal.correlate(centered,reference,mode='valid',method='fft')
    energy=np.r_[0,np.cumsum(centered**2)]
    norm=np.sqrt(np.maximum((energy[len(reference):]-energy[:-len(reference)])*sum(reference**2),1))
    correlation/=norm
    peaks,_=signal.find_peaks(abs(correlation),distance=int(2*fs),height=.12)
    rows=[]
    for start in peaks:
        part=y[start:start+len(reference)]
        rows.append(dict(start_s=float(start/fs),raw_correlation=float(correlation[start]),
            rx_rms_ac=float(part.std()),rx_peak=float(abs(part).max()),rail_fraction=float(np.mean(abs(part)>=32760))))
    summary=dict(native_rate_hz=fs,source_rate_hz=8000,source_duration_s=len(source)/8000,
        source_peak=float(abs(source).max()),source_distinct_levels=len(np.unique(source)),
        complete_matches_in_receive_capture=len(rows),bursts=rows,
        note='First playback may be in dial.s16; no capture gap is spliced. Raw correlation is before channel equalization.')
    (d/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    fig,axes=plt.subplots(2,1,figsize=(11,7),layout='constrained')
    f,t,z=signal.stft(y/32768,fs,nperseg=1024,noverlap=768)
    axes[0].pcolormesh(t,f,20*np.log10(np.maximum(abs(z),1e-7)),vmin=-90,vmax=-10,cmap='magma',shading='auto')
    for row in rows:axes[0].axvline(row['start_s'],color='cyan',alpha=.5,lw=.8)
    axes[0].set(ylim=(0,5100),xlabel='Receive capture time (s)',ylabel='Frequency (Hz)',title='Seven complete DIL playbacks, then call-progress tone')
    axes[1].plot(np.arange(len(source))/8000,source/32768,color='#4c72b0',lw=.6,label='PBX source, decoded mu-law')
    start=peaks[0];part=y[start:start+len(reference)]
    axes[1].plot(np.arange(len(part))/fs,part/32768,color='#c44e52',alpha=.7,lw=.5,label='First complete received playback')
    axes[1].set(xlabel='Time within playback (s)',ylabel='Sample / digital full scale',title='Actual source and received waveform — no equalization');axes[1].legend();axes[1].grid(alpha=.2)
    fig.suptitle('Live Apple modem 0:13 call to 9668 — native 10286 samples/s, mu-law without PBX transcoding')
    fig.savefig(d/'call-dil.png',dpi=160)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
