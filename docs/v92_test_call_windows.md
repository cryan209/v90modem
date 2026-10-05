# V.92 test-call recovery windows

The 2026-10-05 payload investigation supersedes the earlier spectrogram-only
window assignments. Quick Connect and V.92 capability do not establish that
PCM upstream was selected. The three QC recordings contain recoverable V.34
QAM upstream training and repeated V.90 Table-14 CP records. Motorola and USR
also yield downstream B1d and post-training bits using those parameters.

Positions below use the stereo WAV's shared 8000 Hz sample clock. R is the
analogue/upstream recording; L is the digital/downstream recording.

| Call | R: repeated CRC-valid CPt begins | L: mapped TRN2d begins | L: B1d begins | L: post-B1d begins |
|---|---:|---:|---:|---:|
| Agere SV92 QC | 15.3625 s | unresolved | unresolved | unresolved |
| Motorola SM56 V92 QC | 22.37125 s | 183637 / 22.954625 s | 208729 / 26.091125 s | 209017 / 26.127125 s |
| USR Message V92 QC | 21.08156 s | 172810 / 21.60125 s | 194338 / 24.29225 s | 194626 / 24.32825 s |

The R-channel QAM receiver uses 3200 baud / 1828.5714 Hz for Motorola and USR,
and 3429 baud / 1959.4286 Hz for Agere. Motorola and USR CP records pass the
native strict validator. Agere's CRC-valid records fail its fill check;
repairing only that hypothesis still leaves coefficient b2 = -65 outside the
accepted range. Agere parameters are therefore not admitted to payload recovery.

Two independent inverse-filter lengths recover all 48 B1d frames without
errors for Motorola and USR. Their agreeing post-B1d prefixes contain 4391 and
60692 bits respectively. Motorola's prefix is predominantly idle marks. USR
contains idle marks followed by 473 complete V.42 EC answer-detection pairs,
confirmed by the native V.42 detector. Neither prefix yields a valid HDLC FCS
or application payload. See [the recovery workflow](offline_pcm_payload_recovery.md).

## Retracted PCM-upstream assignments

The earlier R-channel intervals (Agere 9.9–14.4 s, Motorola 13.9–18.7 s,
USR 13.7–18.3 s) were labelled TRN2u from their appearance. They were never
validated by a training sequence or control CRC. A dedicated PAM hypothesis
sweep recovered no valid CPu/SUVu there; the later QAM evidence means those
intervals must not be treated as established PCM-upstream fixtures. Earlier
claims of connected data after 15.3/23.3/21.7 s were likewise unverified.

The failed sweep remains in artifacts/gough-v92-recovery-20261005/.
The positive control and downstream recovery evidence is in
artifacts/gough-v92-payload-acquisition-20261005/. These results establish
recovered modem negotiation, not a V.92 PCM-upstream data connection.
