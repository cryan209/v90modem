#!/usr/bin/env python3
"""Measure G.711 levels in captured Apple hardware loopbacks (G.711 §3).

Fixed 65-tap least-squares equalizer trains on one block and scores a separate
block. No training samples contribute to reported symbol/bit errors. Mu-law's
positive/negative zero are canonicalized by the encoder. This is full-alphabet
8 ksymbol/s PCM testing, not a V.90 negotiation or achievable-rate estimate.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from g711_codec import linear_to_ulaw, linear_to_alaw


def report(directory):
    meta=json.loads((directory/'result.json').read_text())
    if not (directory/'tx.s16').exists() or not (directory/'rx.s16').exists():
        return dict(device=meta['device'],mode=meta['mode'],law=meta['pattern'],restored=bool(meta['restored']),reason=meta['reason'],test_samples=0)
    tx=np.fromfile(directory/'tx.s16',dtype='<i2').astype(float)
    rx=np.fromfile(directory/'rx.s16',dtype='<i2').astype(float)
    lag=meta['lag_samples']
    # Warmup is one second, PRBS occupies [1,2.5) seconds. Leave filter margins.
    indices=np.arange(8256,min(19800,len(tx)-33,len(rx)-lag-33))
    indices=indices[indices+lag>=33]
    if len(indices)<8000:
        return dict(device=meta['device'],mode=meta['mode'],law=meta['pattern'],restored=bool(meta['restored']),reason='Insufficient aligned steady-state capture',test_samples=0)
    target=tx[indices]
    features=np.stack([rx[indices+lag+k] for k in range(-32,33)],axis=1)
    train=np.arange(0,4096); test=np.arange(5000,len(indices))
    mean=features[train].mean(axis=0); scale=max(1,features[train].std())
    design=np.column_stack([(features-mean)/scale,np.ones(len(indices))])
    matrix=design[train]; penalty=np.eye(matrix.shape[1])*1e-4;penalty[-1,-1]=0
    weights=np.linalg.solve(matrix.T@matrix+penalty,matrix.T@target[train])
    predicted=design[test]@weights; expected=target[test]
    # Analog test intentionally reduces TX by 12 dB. Restore its scale for
    # decisions against the same original G.711 alphabet; no gain fit on test.
    factor=1/meta.get('tx_scale',0.25 if meta['mode']=='analog' else 1)
    encode=linear_to_alaw if meta['pattern']=='alaw' else linear_to_ulaw
    reference=encode(np.rint(expected*factor)); decoded=encode(np.rint(predicted*factor))
    bits=np.unpackbits(np.bitwise_xor(reference,decoded)).sum()
    error=predicted-expected; power=np.mean(expected**2); mse=np.mean(error**2)
    raw=rx[indices[test]+lag]
    transport=[]
    if meta['mode']=='digital':
        lookup={tuple(tx[i:i+3]):i for i in range(8000,min(20000,len(tx)-3))}
        counts=Counter()
        for j in range(11000,min(19000,len(rx)-3)):
            source=lookup.get(tuple(rx[j:j+3]))
            if source is not None:counts[j-source]+=1
        transport=[dict(lag_samples=int(k),three_sample_matches=int(v)) for k,v in counts.most_common(4)]
    return dict(device=meta['device'],mode=meta['mode'],law=meta['pattern'],
        candidate_transport_lags=transport,
        restored=bool(meta['restored']),train_samples=len(train),test_samples=len(test),taps=65,
        source_linear_levels=int(len(np.unique(tx[8000:20000]))),
        sample_errors=int(meta['sample_errors']),raw_correlation=meta['correlation'],
        rx_dc=float(np.mean(raw)),rx_min=float(raw.min()),rx_max=float(raw.max()),
        rx_rail_samples=int(np.count_nonzero(np.abs(raw)>=32760)),
        heldout_rmse=float(np.sqrt(mse)),heldout_snr_db=float(10*np.log10(power/max(mse,1e-20))),
        symbol_errors=int(np.count_nonzero(decoded!=reference)),symbol_error_rate=float(np.mean(decoded!=reference)),
        bit_errors=int(bits),bit_error_rate=float(bits/(8*len(test))),
        residual_p95=float(np.percentile(np.abs(error),95)))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    args=parser.parse_args();results=[]
    for path in sorted(args.directory.glob('*/result.json')):
        results.append(report(path.parent))
    print(json.dumps(results,indent=2))

if __name__=='__main__':main()
