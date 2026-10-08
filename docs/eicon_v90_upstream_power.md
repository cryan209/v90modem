# V.90 upstream power probe correction, 8 October 2026

The live Eicon calls in `eicon_live_20261008.md` establish upstream CRC
errors at the foreign receiver. They do not identify the faulty encoder
operation. This change addresses one independently reproducible calibration
defect, not the entire hardware interoperability problem.

## Normative contract and defect

V.90 8.5.1 and 9.4.2.5 use V.34 B1 and the negotiated data-mode modulation.
V.34 (02/1998) 9.6.2, Figure 7 and Table 11 compute the precoder feedback
from unprojected x(n). Clause 9.7, equations 9-33 through 9-35, subsequently
projects x(n), using its average energy. The note in 10.1.3 requires power
compensation for the effects of both precoding and nonlinear encoding.

The V.90 transmitter follows this ordering (`nl_x_warp=true`). Its
normalization probe was reset by `v34_seed_tx_data()` to the legacy ordering
(`nl_x_warp=false`), which applies a different nonlinear operation to p(n).
With nonlinear encoding and nonzero precoder coefficients, the probe thus
measured a different x(n) sequence from the live transmitter. The fix copies
the live ordering into the probe before generating any mapping frames.

The Eicon's captured 7910 Type-1 MP selects nonlinear encoding, expanded
shaping and Q14 coefficients `(56,-6), (-32,2), (38,-13)`; see
`artifacts/eicon-v90-asterisk-after-clearmode-7910-r1/server.log`.
At N=5 with these coefficients, the old probe measured average energy
10.7160196 instead of 10.7189458 and produced scale 1.262977 instead of
1.26279899. This is a small correction, about 0.0012 dB of amplitude scale;
it is not sufficient evidence to attribute the observed CRC errors to it.

The subsequent native V.34 audit removes the legacy feedback operation
entirely and enables the correct native output projection. That correction
subsumes the probe-mode workaround described above. See
`v34_tx_eicon_audit.md` for its independent regressions and hardware limits.

## Regression and verification

`v34_data_test` adds 18 cases: N=5/9/13, nonlinear encoding off/on, and
zero, captured Eicon, and stronger legal precoder coefficients. The reference
mapper disables nonlinear feedback, and an independent energy-moment
expansion evaluates the clause 9.7 projection. This checks both the raw
average energy and the compensated scale rather than asking our receiver
whether it agrees with our transmitter.
Every raw mapping frame is also compared with the linear reference to verify
that calibration has not advanced the live B1 state or altered feedback.

The captured-coefficient case fails before the fix and passes afterward.
All 420 mapper cases pass. `vpcm_loopback_test --all-tests`, `v34_mp_test`,
`v34_phase4_16pt_test`, `v90_analogue_tx_test` and `v90_analogue_rx_test`
pass. The modem was rebuilt against the updated SpanDSP library.

`make test` stops at three assertions in `k56flex_client_test`, for its
A-law 32k RC-loop/noise case. That binary links only the K56flex sources and
libm; neither changed source participates in it. The full suite is therefore
not reported as passing.

## Hardware follow-up

`artifacts/eicon-v90-power-probe-20261008-u1/` is a sandbox socket-bind
failure, not a modem outcome.

The permitted PCMU call, `artifacts/eicon-v90-power-probe-20261008-u2/`,
reaches the analogue B1/data handover at +17028 ms but expires the B1d
deadline at +23565 ms, retrains, and never supplies the menu. No checked
echo lines pass. This call does not establish a hardware improvement.

The PCMA call, `artifacts/eicon-v90-power-probe-20261008-a1/`, expires
the Ja/Sd Phase 3 deadline at +10957 ms, retrains, and also never supplies
the menu. It never exercises the changed B1/data calibration path, so its
failure cannot grade this correction. Neither hardware trial is a pass.

The original SIP echo harness was `/private/tmp/eicon_v90_echo_probe.py`;
the captures retain TX/RX DS0, IO scheduling, server log, PTY output and
summary. No Cisco, Asterisk or Eicon configuration was changed.
