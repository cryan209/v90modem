#!/usr/bin/env python3
"""Recover repeated K56flex parameter records from an analogue 8 kHz WAV.

Offline experiment: NumPy FIR fitting and joint six-sample probe decisions.
Exports candidate bits separately from independently CRC-checked records.
Does not infer a payload rate, upstream impairment report, or user data.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import wave

# Bound BLAS parallelism before importing NumPy for repeatable offline runs.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import numpy as np


def crc16(words):
    crc = 0xffff
    for word in words:
        for bit in range(16):
            crc = (crc >> 1) ^ (0x8408 if (crc ^ (word >> bit)) & 1 else 0)
    return crc


def records(bits):
    """Draft 0.23 4.12: marker, ten zero-separated words, residue zero."""
    found = []
    for marker in range(16, len(bits) - 172):
        if bits[marker] != 1 or bits[marker + 1] != 0 or not all(bits[marker - 16:marker + 1]):
            continue
        words = []
        for i in range(10):
            z = marker + 1 + 17*i
            if bits[z] != 0:
                break
            words.append(sum(bits[z + 1 + b] << b for b in range(16)))
        if len(words) == 10 and crc16(words) == 0:
            found.append({"bit_marker": marker, "raw_words": [f"{w:04x}" for w in words[:9]],
                          "received_crc": f"{words[9]:04x}", "crc_residue": 0})
    return found


def correlate(audio, reference, begin=0):
    ref = reference - reference.mean()
    size = len(ref)
    if len(audio) < size or not np.dot(ref, ref):
        return None
    nfft = 1 << (len(audio) + size - 1).bit_length()
    corr = np.fft.irfft(np.fft.rfft(audio, nfft)*np.fft.rfft(ref[::-1], nfft), nfft)[size - 1:len(audio)]
    sums = np.r_[0., np.cumsum(audio)]
    powers = np.r_[0., np.cumsum(audio*audio)]
    var = powers[size:] - powers[:-size] - (sums[size:] - sums[:-size])**2/size
    scores = corr/np.sqrt(np.maximum(var, 1)*np.dot(ref, ref))
    if begin >= len(scores):
        return None
    scores[:max(0, begin)] = 0
    peak = int(np.argmax(scores))
    return {"sample": peak, "seconds": peak/8000, "correlation": float(scores[peak])}


def recover(path, output, channel="L", law=0):
    root = Path(__file__).resolve().parents[1]
    with wave.open(str(path), "rb") as wav:
        if wav.getframerate() != 8000 or wav.getsampwidth() != 2 or wav.getnchannels() not in (1, 2):
            raise ValueError("requires mono/stereo 8 kHz 16-bit PCM WAV")
        audio = np.frombuffer(wav.readframes(wav.getnframes()), dtype="<i2").reshape(-1, wav.getnchannels())
        audio = audio[:, 0 if channel == "L" or wav.getnchannels() == 1 else 1].astype(float)
    output.mkdir(parents=True, exist_ok=True)
    sources = [root/n for n in ["tools/k56flex_record_probe.c", "k56flex_probe.c", "k56flex.c"]]
    report = {"input": str(path.resolve()), "input_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
              "channel": channel, "law": ["ulaw", "alaw"][law], "payload_verified": False,
              "source_sha256": {str(f.relative_to(root)): hashlib.sha256(f.read_bytes()).hexdigest()
                                for f in sources + [Path(__file__).resolve()]},
              "trials": [], "repeated_records": []}
    with tempfile.TemporaryDirectory(prefix="k56flex-record-") as tmp:
        helper = Path(tmp)/"probe"
        subprocess.run(["cc", "-O2", "-I"+str(root), *map(str, sources), "-lm", "-o", str(helper)], check=True)
        def reference(stage, length):
            return np.frombuffer(subprocess.check_output([str(helper), "reference", str(stage), str(law), str(length)]), dtype=np.int16).astype(float)
        report["p1_anchor"] = correlate(audio, reference(2, 1024))
        p1 = report["p1_anchor"]
        if p1 and p1["correlation"] >= .65:
            # Exclude the initial P3: PT_A follows all three probe stretches.
            report["pt_anchor"] = correlate(audio, reference(4, 4096), p1["sample"] + (1364 + 704)*12)
        pt = report.get("pt_anchor")
        if pt and pt["correlation"] >= .65:
            ref = reference(4, 8000)
            for taps in (257, 513):
                start, half = pt["sample"], taps//2
                if start < half or start + 8000 + half >= len(audio):
                    continue
                matrix = np.lib.stride_tricks.sliding_window_view(audio[start-half:], taps)
                coeff = np.linalg.lstsq(matrix[:6000], ref[:6000], rcond=1e-5)[0]
                equalized = matrix @ coeff
                error = equalized[6000:8000] - ref[6000:8000]
                snr = float(10*np.log10(np.mean(ref[6000:8000]**2)/max(np.mean(error*error), 1e-20)))
                stream = output/f"equalized-{taps}.f32"
                equalized.astype(np.float32).tofile(stream)
                for pacing in (0, 1):
                    bits_path = output/f"candidate-{taps}-pacing{pacing}.bits"
                    subprocess.run([str(helper), "decode", str(stream), str(law), str(pacing), str(bits_path)], check=True, stdout=subprocess.DEVNULL)
                    found = records(list(bits_path.read_bytes()))
                    report["trials"].append({"taps": taps, "pacing_assumed": pacing, "held_out_training_snr_db": snr,
                                             "candidate_bits_file": bits_path.name, "records": found})
            # Require exact raw record agreement in both independent FIR fits,
            # with at least two occurrences in each. Retain pacing as a hypothesis.
            for pacing in (0, 1):
                counts = []
                for taps in (257, 513):
                    count = {}
                    for trial in report["trials"]:
                        if (trial["taps"], trial["pacing_assumed"]) == (taps, pacing):
                            for rec in trial["records"]:
                                key = tuple(rec["raw_words"])
                                count[key] = count.get(key, 0) + 1
                    counts.append(count)
                for key in sorted(set(counts[0]) & set(counts[1])):
                    if min(c[key] for c in counts) >= 2:
                        report["repeated_records"].append({"raw_words": list(key), "pacing_assumed": pacing,
                                                           "occurrences_by_taps": dict(zip(("257", "513"), (c[key] for c in counts)))})
    (output/"records.json").write_text(json.dumps(report, indent=2)+"\n")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wav", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--channel", choices=("L", "R"), default="L")
    parser.add_argument("--law", choices=("ulaw", "alaw"), default="ulaw")
    args = parser.parse_args()
    result = recover(args.wav, args.output, args.channel, int(args.law == "alaw"))
    print(json.dumps({"repeated_records": result["repeated_records"], "payload_verified": False}, indent=2))
