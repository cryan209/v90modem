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
| 12.6.1.3: a PPh response requests modulation negotiation | Source can switch from the Sh/ALT branch to PPh/ALT/MPh/E when the recipient requests parameter negotiation. The new branch has no dedicated adversarial waveform test yet. |
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

The new targets are invoked by `make test`. Standalone targeted runs are
necessary when the full suite stops at the unrelated full-duplex failures
recorded in `t30_annex_f_v34_fax.md`. Shared regression checks include the PCM
loopback suite, 3200/21600 full-duplex in both laws, and both fax-class suites.
These are offline/G.711 loopbacks, not new Canon hardware interoperability.

## Remaining conformance and interoperability work

- 12.4.3.1: foreign Tone A/B recovery during initial control startup is not
  implemented here. Do not describe the timeout/AC work as all of 12.4.3.
- 12.6.2.3 parameter-change initiation and its complete two-peer test, and
  noise/delay/simultaneous-AC adversarial tests beyond the current loss cases.
- 12.7 primary retrain needs a clause-12-specific audit and waveform tests;
  the shared `v34_start_retrain()` implements 11.5 and is not evidence that
  the half-duplex primary procedure is correct.
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
