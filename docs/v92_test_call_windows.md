# V.92 Test-Call Phase 4 Windows

These windows were identified from separate-channel spectrograms of the
checked-in stereo calls.  The calls connected successfully, so the long
low-level broadband region on the analogue/upstream (right) channel is the
TRN2u-bearing Phase 4 interval; the structured comb that follows is the
SUVu/CPu/E2u and transition region, followed by connected data.

| Call | Upstream channel | TRN2u-bearing interval | Phase 4 control/transition | Connected data |
|---|---:|---:|---:|---:|
| Agere SV92 QC | R | about 9.9–14.4 s | about 14.4–15.3 s | after about 15.3 s |
| Motorola SM56 V92 QC | R | about 13.9–18.7 s | about 18.7–23.3 s | after about 23.3 s |
| USR Message V92 QC | R | about 13.7–18.3 s | about 18.3–21.7 s | after about 21.7 s |

The left-channel broadband blocks are the digital/downstream direction and
must not be used as TRN2u input.  Earlier corpus sweeps mixed these regions and
also used the high-level control transition to estimate `L_U` for Motorola and
USR, producing incorrect windows and amplitudes.

Representative strict replay commands are:

```sh
./v92_trn2u_replay gough-lui-v90-v92-modem-sounds/Agere-SV92-QC.wav \
  --analog-wav --channel R --start 79200 --max 36000 --points 8 --lu 96

./v92_trn2u_replay gough-lui-v90-v92-modem-sounds/Motorola-SM56-V92QC-extDILlaserbeam.wav \
  --analog-wav --channel R --start 111200 --max 38400 --points 8 --lu 75

./v92_trn2u_replay gough-lui-v90-v92-modem-sounds/USR-Message-V92QC-bong-bong-bong.wav \
  --analog-wav --channel R --start 109600 --max 36800 --points 8 --lu 75
```

The point count still needs confirmation from Jp bit 48; both 4- and 8-point
hypotheses should be retained until Jp is decoded.  Even with the corrected
windows, the critically sampled 8 kHz recordings do not yet yield a sustained
descrambled-one run.  This is now a symbol-recovery problem, not uncertainty
about which part of the call contains Phase 4.
