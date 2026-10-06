# V.34 half-duplex clause 12 audit, 6 October 2026

Normative source: `ITU Docs/T-REC-V.34-199610-S!!PDF-E.pdf`, clauses
10.2.3.3, 10.2.4 and 12; printed pages 58-64, including the corrected
Figure 27 on printed page 62. This audit covers modem procedures; T.30
Annex F and the fax service-class command interfaces are separate layers.

## Changes made against the specification

| Requirement | Implementation and verification |
| --- | --- |
| 12.6.3.1/.2: four control-channel symbols of scrambled ones before primary entry | Both roles finish that tail without pulling DTE bits, then the source enters 12.5.1 and the recipient conditions for 12.5.2. The transition is made at a control-symbol boundary, including within a media block. |
| 12.5.3.1: 35 ms scrambled-one primary tail | A control request detaches user input while retaining mapper, scrambler, carrier and pulse-shaper state for 280 generated DS0 samples. No receiver/sample counter is rewound. |
| 12.6.1.1/.2/.4 and 12.6.2.1/.2: normal return to control | Source sends 70 ms silence, 24T Sh, 8T Sh-bar, ALT, E and data. Its final 16T of ALT is counted after detection of the returned Sh/Sh-bar. Recipient detects the complete Sh/Sh-bar sequence and responds with its own Sh/Sh-bar, ALT and E. Retains the previous MPh parameters. |
| Corrected Figure 27: both E sequences before control user transmission resumes | Resynchronization sends idle marks until peer E has been received; it does not consume DTE data during that wait. |
| 12.6.1.3 / 12.6.2.3 and corrected Figure 26: recipient requests modulation negotiation | `v34_half_duplex_request_parameters()` queues a new primary ceiling during a recipient page. After complete Sh/Sh-bar detection it sends PPh/ALT and waits for the source PPh, then both exchange MPh/E. Tests require 9600 -> 7200 at both ends, verified returned control and second primary payload, and no AC fallback. |
| 12.4.3.1: foreign Tone A/B instead of PPh in initial control startup | Raw-sample tone qualification switches the source to B/A and INFOh reception, then repeats 12.3.1. Restricted to initial startup before peer PPh; does not turn normal page resync into a primary retrain. The peer fixture returns to the real Phase 2 tone/INFOh generator; the source must detect the G.711 waveform. |
| 12.7.1/.2: primary retrain, either initiator and either source role | `v34_start_retrain()` has a separate HDX path: clamp, exact 560-sample silence, fresh A/B ranging without INFO0, probe, INFOh, primary training and control restart. The unprompted peer detects >50 ms of the foreign tone on raw DS0 samples. Tests assert preserved sample clocks and require only the source to send L1/L2. |
| 12.2.1 / 12.2.2: source/recipient Phase 2 mirror | The source sends the probe and receives INFOh; the recipient sends the first reversal and INFOh, regardless of call direction. Retrain uses the same mirror. A calling recipient qualifies returned Tone A separately from old primary data and L2 bin energy. |
| 12.4.1.2: source MPh follows detected peer PPh | Removed the old unconditional ALT-to-MPh fallback at 120T when PPh was absent. Loss now leads to the specified recovery. |
| 12.4.3.2-.4, 12.4.4.1-.3, 12.6.1.5/.6, 12.6.2.4/.5: three-second recovery bounds | Separate acquisition/MPh/E watchdog states use generated samples. The resynchronization acquisition and E deadlines retain the local Sh/Sh-bar origin. Control recovery does not restart Phase 2 or discard the trained primary profile. |
| 10.2.4.1 and 12.8.1/.2: AC control retrain | AC is alternating antipodal points. Sustained AC exceeding 100 ms makes the receiver respond with PPh. Initiator answers PPh with PPh/ALT; both exchange MPh/E and resume control data. Simultaneous initiators become responders upon AC detection. A response latch prevents repeatedly restarting PPh while the peer continues AC. |
| Table 23 / 12.4.1.3/.4 and 12.4.2.4/.5: usable MPh parameters | CRC-valid offers requesting unsupported 2400 bit/s control transmission, or no enabled primary rate below the common ceiling, are rejected. The watchdog then recovers rather than treating CRC validity as parameter validity. |
| Retain enabled primary-rate ceiling on recovery | B1 rebuilds working mapper parameters, including a profile maximum. MPh uses a separately retained configured ceiling so a later control retrain cannot silently increase the rate offer. |

## Acquisition bugs exposed by the conformant turn-off

The four control tail symbols exposed an S/S-bar false detection in the
following silence. Primary receive-filter ringing supplied S-like votes, and
numerical phase changes could satisfy the junction condition before the real
128T S interval. HDX primary resynchronization now establishes the minimum
65 ms silent interval and low-energy run, clears tail-derived S votes and
junction history, and requires energy in both symbols of the actual junction.
This is scoped to clause 12 primary resynchronization.

Sh/Sh-bar detection correlates the complete 24T/8T pattern on both T/2 eye
phases and qualifies the normalized correlation. The weaker crossing-phase
match caused one 3429/u-law row to miss E; the complete-pattern qualification
now selects a usable eye. No ideal symbols are injected into the receiver.

A restart also exposed inherited eye votes/pending flips from the previous
page. Fresh HDX startup clears those measurements. Its initial S acquisition
uses a 64-symbol eye window, fitting inside 12.3's 128T signal rather than
waiting for the previous 256-symbol window. Full-duplex acquisition is unchanged.

Primary retrain exposed an initiating recipient hearing old primary data
before the source responded. A coarse tone latch could reverse its tone before
the far end had even detected the retrain. Retrain now qualifies the peer tone
on raw samples; ordinary calling-recipient post-L2 qualification also requires
a dominant 2400 Hz bin so the probe itself cannot release INFOh early.

The 64-symbol HDX eye window previously used a signal-energy threshold scaled
to the unrelated 256-symbol duplex window. Repeated Phase 3 after startup tone
recovery exposed that mismatch. Both accumulation and qualification now use the
same window length; duplex window behavior is unchanged.

## Automated evidence

- `make v34-hdx-primary-test`: the existing 38 rate/law/startup/restart rows.
- `make v34-hdx-turnaround-test`: 27 rows. All six symbol rates in both laws,
  with call and answer modem as source, plus 3200/21600 in both laws and a
  3200/21600 A-law row with 80-sample blocks. Each grades the first primary
  interval, returned bidirectional control data, then the second primary
  interval. Primary alignment must start at source bit zero.
- `make v34-hdx-recovery-test`: 17 rows. Four-second loss of one direction
  during PPh, MPh or E, both laws and both source roles, an 80-sample variant,
  and four CRC-valid invalid-MPh tests (unsupported CC rate and unusable
  primary mask). Each requires a real AC recovery and recovered PRBS data.
  E loss is injected at receive-sample boundaries because MPh and E can both
  arrive within one media block. Recovery may lose in-flight user bits;
  alignment offsets are reported, not represented as lossless delivery.

- `make v34-hdx-parameters-test`: 25 rows, all six symbol rates, both laws
  and both source roles, plus an 80-sample row. Requires a directly observed
  source PPh without AC, 7200 bit/s negotiated from the new recipient ceiling,
  clean returned control and a clean second primary interval.
- `make v34-hdx-retrain-test`: 56 rows: 48 primary retrains (six rates,
  both laws, both source roles and either initiator), six explicit control
  retrains (either or simultaneous initiators in both laws), and two 80-sample
  rows. Primary tests verify the pre-retrain payload, actual silent/clamped
  transition, automatic peer response, source-only probe, recovered control,
  and a new zero-offset primary payload. In-flight bits before the responder
  detects the tone/AC are excluded from the new exchange, not claimed lostlessly.
- `make v34-hdx-startup-recovery-test`: 25 rows, all six symbol rates and
  both laws/source roles plus an 80-sample row. A recipient fixture returns to
  Tone A/B and sends INFOh; the source must take the actual tone-recovery path
  and then carry independent bidirectional control payload.

`V34_HDX_BLOCK_SAMPLES` is now actually read by the harness. The older parser
used a mismatched variable name, so the previously named 80-sample rows had
silently used 160 samples. All three original targets are rerun with the corrected
parser as well as the added targets (188 rows total).

The new targets are invoked by `make test`. Standalone targeted runs are
necessary when the full suite stops at the unrelated full-duplex failures
recorded in `t30_annex_f_v34_fax.md`. Shared regression checks include the PCM
loopback suite, 3200/21600 full-duplex in both laws, and both fax-class suites.
These are offline/G.711 loopbacks, not new Canon hardware interoperability.

Control retrain PPh detection (10.2.4.5/12.8) uses a bounded, normalized
16-symbol window at each T/2 eye phase, requiring four further matching
windows. The previous exponential history included arbitrary old data/AC
and missed the responder's entire 32T PPh after a delayed request. Initial
12.4 startup keeps its established detector: changing that acquisition time
also shifted primary training and broke a mirrored 3200-baud retrain row.
The 212 existing HDX rows pass with the recovery-only detector.

## Remaining conformance and interoperability work

- Parameter-change API currently changes the primary rate ceiling. Changes to
  other MPh options and control rate need their own API and interoperability tests.
- `make v34-hdx-channel-test` covers 36 deterministic impaired-channel rows:
  primary payload with 20/80/200 ms one-way propagation in both laws, clean
  and uniform analogue noise of peak 32 PCM units; primary/control retrain
  initiated by either end or both simultaneously with 80/200 ms asymmetric
  propagation and peak-8 noise, both source call roles and both laws.
  Noise is added before G.711 encoding, and encoded samples are delayed
  without inserting/dropping samples. This does not cover clock drift,
  echoes, frequency offsets, or a measured telephone-line impairment model.
- Delayed page turnarounds remain open: at 80/200 ms asymmetric delay,
  ordinary Sh/E return produces unaligned source control data; parameter
  changes can stall after the source receives MPh while the recipient has
  not acquired the source PPh. Reproduce with `V34_HDX_PRIMARY=1
  V34_HDX_TURNAROUND=1 V34_HDX_DELAY_MS=80 V34_HDX_REVERSE_DELAY_MS=200
  V34_HDX_NOISE_PEAK=8 ./v34_hdx_test 3200 9600 ulaw 30`; add
  `V34_HDX_PARAMETERS=1` for the parameter-change case.

- Phase 2 recovery beyond the newly covered startup tone fallback and primary
  retrain needs a separate clause-by-clause audit against foreign waveforms.
  INFOh now governs the source symbol rate, carrier, requested power reduction,
  pre-emphasis and TRN constellation (Table 22/12.3.1), independent of call role.
  Reserved symbol-rate and pre-emphasis indices cannot release Phase 3.
  `make v34-hdx-infoh-test` verifies low-carrier selection against the source's
  high-carrier default: all six symbol rates, both laws and both source call
  roles (24 rows, error-free primary payload). Different symbol-rate profiles,
  nonzero power/pre-emphasis and 16-point TRN still need waveform tests.
- 10.2.4's optional 2400 bit/s control channel remains unimplemented and is
  not advertised. Unsupported remote requests are rejected explicitly.
- Primary rates 31.2 and 33.6 kbit/s remain unverified, with known loopback
  errors; existing supported payload rows extend through 28.8 kbit/s.
- Annex F's 40-one/70 ms handshake, ECM image transport and complete T.30
  page exchange remain separate integration work. A modem-level channel
  request does not by itself implement that fax handshake.
- Circuit-105 requests currently come from the explicit mode API. Automatic
  carrier-level circuit-109 behavior and real circuit/DTE handshakes need
  separate measurement against 6.6.2 and the clause 12 procedures.

This is a tested completion of normal page turnarounds and substantial control
recovery, not a claim of complete V.34 or Super G3 conformance.
