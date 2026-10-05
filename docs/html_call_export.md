# Offline call HTML export

Build and export an 8 kHz recording:

```sh
make vpcm_decode
./vpcm_decode --visualize-html call.html recording.wav
./vpcm_decode --law ulaw --visualize-html call.html live-rx.g711
```

`--visualize-html` takes an output filename. It enables the chronological
call log and defaults to the implemented call decoders. Explicit decoder flags
restrict that selection, for example `--v34 --visualize-html call.html`.
The output embeds its data and JavaScript and needs no external libraries.

The viewer supports individual stereo channels, a merged overlay, and a shared
spectrogram with separate tone overlays. Tone toggles, spectral lift, contrast,
and tone lift affect display only. Click an event row to move the cursor; hover
over the chart to inspect spectral bins and tracked tones.

## Evidence and timing

Each event shows its exact input sample offset as well as milliseconds.
Expandable **Raw evidence** preserves the decoder's original detail fields,
including CRC results, soft-lock scores, candidate status and recovery source
where available. Structured details are a presentation of those fields.

Diagnostic rows show receiver candidates and inferred channel roles. They are
not transmitted signals and are excluded from the chart's event markers.
The stereo role/U_INFO diagnostic records the inferred side, the resolved
U_INFO, and which channel supplied it. These are decoder inferences, not
hardware identity or proof that a peer accepted a signal.

Stereo WAV channels share the recording's time origin. Two separate live RX
and TX taps do not acquire a shared origin from having equal file lengths;
comparison requires independently measured alignment. All positions are in the
input recording, not the transmitter's engine clock.

## Shared decode results

As of 2026-10-05, text call logs and HTML retain the same per-channel event
lists. Event generation borrows the Phase 1/2 and V.34 results already produced
by Stage A and resolved across stereo channels. This preserves later recovered
phase timing and MP fields and avoids independent Phase 2 probes in the event
emitter and an additional call-log decode for HTML. The Stage A context owns
Phase 1/2 allocations; retained call logs own their copied event strings.

This does not unify every later signal scan into one decoder. Jd/Ja/DIL/CP
collectors still run their own existing analyses. Nor does exporting an event
make its evidence stronger: inspect the source, CRC and candidate fields before
using it to diagnose a live call.

Coverage includes the existing V.8/V.8bis, V.34, and V.90/V.91/V.92 offline
decoders, plus the proprietary receive evidence below. V.32bis session
decoding is not implemented here.

## x2 and K56flex receive evidence

```sh
./vpcm_decode --x2 --visualize-html x2.html recording.wav
./vpcm_decode --k56flex --visualize-html flex.html recording.wav
```

Both paths also run with `--all` and the default HTML/call-log decoder
selection. Explicit protocol selections are respected. `--k56flex` includes
the generic V.8bis signal and message scan; a generic CL or MS is not itself
proof of K56flex.

x2 reuses the analogue peer's 1200 Hz INFO0/marker receiver and the
3200-baud/high-carrier Courier MP receiver. A separate receive-only 2400 Hz
INFO0 scan requires two identical CRC-valid 17-bit digital bodies. INFO0
framing is shared with ordinary V.34 (10/1996), 10.1.2.3.3/Table 14, so
INFO0 alone is only an x2-compatible diagnostic. A supported CRC-valid
directional marker promotes the analogue INFO0 to x2 evidence. MP requires
two matching protected records with valid CRCs; unprotected tail bits are
reported without assigning meaning. E requires twenty consecutive ones on an
MP-qualified timing/phase hypothesis. Captures starting after INFO0 can still
recover MP/E. The supported marker is the existing high-carrier index-four
profile. Other carrier/rate profiles and downstream payload are not decoded.
The analogue INFO0 API retains the low sixteen body bits; the digital scan
retains all seventeen. No CONNECT or upstream user-data claim follows from E.

K56flex uses the existing linear-audio client front end to acquire the P1
probe. Training stages inferred by its shadow model are diagnostic rows;
the parameter-record event requires the existing framing/checksum parser to
succeed. Record rates are fields decoded from that record, not measured
throughput. The upstream report waveform remains unrecovered: the model uses
report `8880`, extension `00`, and these assumptions appear in event evidence.
No payload bits are exported or labelled verified.

Proprietary event offsets are **detection positions**, including receive-filter
latency. They are not reconstructed first samples of the signal. K56flex P1
acquisition additionally supplies its measured `p1_anchor_sample`. Receiver
failure is placed at the last processed input sample and the report keeps any
previously decoded records.

`make legacy_pcm_decode_test && ./legacy_pcm_decode_test` checks silence
rejection, generated INFO0/marker frames in both x2 carrier directions, the
preserved Courier MP/E capture with a leading offset, and complete generated
K56flex training/parameter recordings in both laws. The K56flex test records a
server dialogue first and gives only that audio to the offline receiver.
Passing it establishes self-consistency, not hardware interop. The bundled
x2 recording yields peer INFO0/marker; the K56flex recording acquires P1 but
does not yield a valid parameter record with this model.

## Validation of the export update

The update was checked by rebuilding `vpcm_decode`, exporting real mono G.711
and stereo WAV captures, comparing original event sets and audio envelopes,
and checking text/HTML event counts. Embedded JavaScript, data-array lengths,
all stereo views, cursor motion, tone toggles, sliders and event-row clicks
were exercised with a mocked DOM in Node. The existing Phase 2 decoder and
V.92 procedure evaluator tests passed. These checks do not verify browser
layout or hardware interoperability.

The existing `k56flex_client_test` noisy A-law priming case currently fails
three assertions in one scenario (failure in PRIME, one priming frame, no
data bits). It is recorded in `docs/gap_analysis_2026-10.md`; it also failed
on this macOS/ARM validation run. The new offline tests pass, and none of
the existing K56flex receiver or channel implementations were changed.

## Gough Lui corpus check, 2026-10-05

All 50 bundled WAV calls (27 V.34 and 23 V.90/V.92/proprietary) were scanned
with both `--x2` and `--k56flex`, exporting HTML for every call. The initial
run mislabeled shared INFO0 on 27 non-x2 recordings. The compatibility-only
classification and marker qualification above correct that error. The
corrected run has qualified x2 events only on `x2-42667.wav` and K56flex P1
acquisition only on `k56flex-48000.wav`. This is a negative-control result for
this corpus, not a universal classifier guarantee.

The x2 recording yields analogue INFO0 `21ff` and marker `4d`; no MP/E was
recovered. The K56flex recording acquires P1 on Caller RX, but its model
accumulates 4280 mismatches in 9890 checked blocks and fails at about 18.985 s
without a valid parameter record. Its fixed-stage timeline is not sufficient
to claim those stages from the waveform.

Reports, logs, hashes, process results and the corpus matrix are preserved in
`artifacts/gough-legacy-decode-20261005-qualified/report.md`. All processes,
JavaScript syntax/data checks, per-channel text/HTML event-count checks and
mocked viewer-control execution for all 50 reports passed. Browser layout
remains unverified.


## Deeper V.90 payload audit, 2026-10-05

A separate `--v90 --phase12 --call-log` audit covered the 21 other bundled
V.90/V.92/vendor recordings: 19 completed the full pass, and two whose deep
searches exceeded 240 seconds completed a bounded Jd-only pass. The latter do
not establish a negative CP/MP/payload result. Ten calls yield Jd capability
evidence, four yield CRC-valid CPt, and two yield CRC-valid MP. No validated
B1/B1d boundary or user payload was recovered. The Motorola SM56 MP is repeated
semantic recovery with CRC-field corrections, and is kept separate from the
observed CRC-valid records.

The audit exposed a replay crash on three calls: `v34_seed_rx_mp` selects the
live CP stage, but the offline E/B1 replay has no CP callback. The replay now
selects its existing E watcher after seeding (V.90 9.4.1.6), without changing
the live receiver. All three failing recordings complete after the fix, and
the legacy PCM and V.34 Phase-2 decoder regression tests pass.

`VPCM_V90_POST_MP_LEVELS_CSV=<file>` optionally exports actual post-MP equalizer
levels for independent B1d checks. It does not add an inferred boundary or
payload event to HTML. Swann's tested single-constellation B1d hypotheses fail
a held-out fit; this is not proof its waveform lacks data. Full logs, binary
hashes, coverage, CRC evidence and the independent fit are retained in
`artifacts/gough-v90-payload-20261005/report.md`.
