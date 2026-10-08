# Spec causality audit of the Eicon failures

Audited against local ITU-T V.34 (02/1998) and V.90 (09/1998), including
visual inspection of V.34 Figures 10–12. This distinguishes a conformance
defect from evidence that the defect caused a particular captured failure.

## Conclusion

Four V.34 violations have now been corrected: nonlinear projection in the
precoder feedback, retained precoder history at B1, J(16) classified as J(4),
and missing 16-point Phase 4 energy normalization. The latter two were
established against an actual Eicon J request and measured TX waveform.
The corrected native PCMA call now trains at 31200 upstream / 21600
downstream and passes ten exact echoes with card receive aborted/CRC 0/0.

Separately, the V.90 payload failures were traced to loss and long arrival
stalls after packets enter the Mac's WireGuard tunnel. The same-source LAN
build passes V.90 echoes in both G.711 laws: ten PCMU and thirty PCMA lines.
See `eicon_rtp_causality_20261008.md` and `v34_tx_eicon_audit.md` for captures
and the staged hardware comparisons. This remains tested interop, rather
than certification of all modes and rates.

## Checks relevant to the observed failures

| Requirement | Implementation / evidence | Causal assessment |
|---|---|---|
| V.34 10.1.3.3/.9: clockwise differential MP, selected constellation, scrambler and CRC | Actual caller PCMU TX tap independently yields 247 CRC-valid 4-point MP frames | CRC-valid four-point frames do not satisfy a peer requesting 16-point; the J classification defect was subsequently established |
| V.34 11.4.1.1.2/.3: finish MP, send MP', wait for peer MP'/E, finish MP', then E | Replay switches to MP'; independent tap finds 234 acknowledged frames; peer repeatedly sends acknowledgement zero | Missing local acknowledgement is not the explanation; extending timeout does not establish peer acceptance |
| V.34 Table 20: MP trellis/nonlinear/shaping fields select the remote transmitter | `mp_apply_parameters()` applies peer fields to TX and local fields to RX | Direction is implemented correctly |
| V.34 11.4.1.1.3: negotiate directional rates from both maxima and common mask | `v34_negotiate_mp_rates()` intersects masks/maxima and applies symmetric-rate restriction | No direction reversal found; 4800 requested by Eicon does not mean our receiver requests 4800 |
| V.90 Table 16 / 9.4.2.4: digital MP selects analogue upstream encoder and highest common rate | Analogue path uses MP trellis, nonlinear selection, expanded shaping, coefficients and intersection with CP mask | Captured selections agree with the handover; both 31200 and capped 21600 fail |
| V.34 7 / V.90 5.3: role-specific scrambler | Native caller GPC; V.90 analogue B1/data GPA, explicitly set at handover | No inherited downstream/GPC scrambler found in V.90 payload entry |
| V.34 8.1/.2, 9.6.3/Table 12, 10.1.3.1: B1 length, state reset, inversion pattern and new superframe | Full P mapping-frame B1; reset mapper/filter states; final-superframe inversion phase; rollover to frame zero | Earlier native delay-line omission corrected; V.90 reset path already had it |
| V.34 9.6.2/Figure 7: Q14 complex coefficients, Q7 feedback rounding and quantization, ties towards zero | Added exhaustive independent boundary/tie checks of production functions; pass | No rounding violation found in the checked domain |
| V.34 9.6.3.1/.2, Figure 9/10: labels and 16-state trellis wiring/delay | Added independent Figure-9 labels and all 256 Figure-10 transitions; pass | No 16-state transition/label error found; full shell-mapper waveform interop is not proved |
| V.34 9.7, 10.1.3 note: project x(n), retain raw feedback, compensate average power | Existing corrected tests evaluate equations 9-33–35 and native handover; V.90 enables projection and compensation | Native old violation demonstrable; V.90 remaining corruption not explained by it |
| V.90 9.4.2.5: B1 follows the completed 20-bit E without a callback-block gap | Handover armed while CP'/E is transmitted and taken at the exact modulator symbol | The prior block-boundary silence defect is already corrected |

## Watchdog discrepancy, not the observed upstream cause

V.34 11.4.2.1.2 specifies the caller's E deadline as 2500 ms + 2 round-trip
delays after J', or 30 seconds when the peer CME bit is set. The current
duplex receive watchdog uses 2500 ms + 3 assumed RTDs measured from the first
accepted MP for both roles. Its origin is later and its caller allowance
larger. Eicon INFO0 in the replay says it is not a CME modem.

This is a conformance discrepancy, but it cannot explain premature timeout
on the observed native call: we already allow more time, and emit many
MP' frames while the peer keeps transmitting unacknowledged MP. It also
does not run in the established V.90 data path. No timing knob was changed.

## All-zero retry offers require measurement, not a fabricated capability

V.34 10.1.2.3.4/Table 15 explicitly defines zero projected rate as an
unusable symbol rate. Thus all-zero INFO1c alone is not proof of a spec
violation. The recorded-schedule replay's final retry reports 13 probe
blocks and in-band SNR of approximately -2.1 to -2.8 dB on otherwise enabled
rows. The builder emits zero in response to that measurement.

It remains necessary to determine whether those probe windows measure the
peer's intended L2 correctly. Advertising a usable rate regardless of the
probe would conceal that question. The earlier batch report's description
of a separate recovery problem should be read with this qualification.

## Validation and limits

### Fixed-playout delivery check and corrected CRC alignment

`tools/eicon_compare_received_audio.py` re-examines the paired direct
fixed-playout capture using the independently established card-minus-local
offset of 2343 samples. From local TX file seconds **18.000000 through
36.397500**, all **147180 decoded linear samples are exactly equal** at
the card and at our transmitter. The only 46 codeword differences in this
continuous interval are PCMU 0x7f -> 0xff: negative zero changed to positive
zero, with no waveform change. This is stronger than the earlier rounded
correlation or 99.9–99.95% codeword equality at four selected windows.

The remainder of the comparison is not equal: the first waveform mismatch
is at local TX 36.397500 seconds. A fixed-offset comparison beyond that
point does not establish whether the trace has lost records, playout has
shifted, or individual samples changed. No whole-call exactness is claimed.
The failed echo and card CRC reports belong to this paired capture; their
individual event origins have not yet been mapped onto the sample interval.

The reproducible result is saved in
`artifacts/eicon-direct-fixed-audio-20261008-r3/received-audio-comparison.json`.
Thus bearer distortion is excluded for this exact interval, while our
generated waveform/data stream remains suspect. This does not establish
that the known-working card is wrong.

`make v34_phase4_16pt_test && ./v34_phase4_16pt_test` passes after adding
spec-based Figure-9/Figure-10 and rounding checks. The checks compare
production operations directly with independently transcribed spec rules;
they do not depend on this project's matching receiver. Existing B1 and
nonlinear seam cases also pass. `git diff --check` passes.

Subsequent record-order alignment locates the first card CRC indication just
after the exact interval ends: altered audio starts at local TX 36.397500,
and the preceding Audio1 block ends at TX-equivalent 36.399625. That capture
therefore does not demonstrate bad upstream frames during intact delivery.
The later simultaneous RTP/card captures and LAN trials resolve this part
of the earlier uncertainty. Native V.34's separate Phase 4 failure remained
on the LAN, leading to the J(16) and power corrections described above.
