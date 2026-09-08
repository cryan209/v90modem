# V.92 specification and amendment audit

Checked 2026-09-08 against the local sources, in precedence order:

- [V.92 (11/2000)](../ITU%20Docs/T-REC-V.92-200011-I!!PDF-E.pdf).
- [Amendment 1 (07/2001)](../ITU%20Docs/T-REC-V.92-200107-I!Amd1!PDF-E.pdf).
- [Amendment 2 (03/2002)](../ITU%20Docs/T-REC-V.92-200203-I!Amd2!PDF-E.pdf).
- [Corrigendum 1 (07/2003)](../ITU%20Docs/T-REC-V.92-200307-I!Cor1!PDF-E.pdf).

Corrigendum 1 explicitly supersedes Amendment 1's 9.10.1 and Figure 20.
The CRC cross-reference was checked against V.34 (10/1996), 10.1.2.3.2,
including Figure 14. This is a source-level and modeled-waveform audit, not
an assertion of hardware interoperability or complete V.92 conformance.

## Implemented corrections

### SCR: Amendment 1 item 5, replacement 8.6.6

The digital transmitter sent absolute GPC scrambler output in SCR. The
amended procedure requires differential encoding, preserving the final
Jp-prime sign and continuing the existing scrambler. `V90_TX_SCR` in
`v90.c` now does that. U_INFO, G.711 codewords and the six-symbol termination
boundary retain their existing definitions.

`test_spec_scr()` in `v92_startup_test.c` observes transmitted Jp/Jp-prime
signs, reconstructs the scrambler history independently, and grades the next
96 SCR codewords on both laws. It fails on the old absolute-sign transmitter.
The analogue startup receiver currently does not grade SCR's scrambled-one
content; coupled startup alone could not reveal this defect.

### Control CRCs: base 8.5.1, 8.7 and 8.8, V.34 10.1.2.3.2

`v92_cp_rx.c` and `v92_phase4_decode.c` previously passed sync and start bits
through the CRC generator. The referenced V.34 procedure excludes them.
Both codecs now cover only the information words; omitted optional CPd
sections still preserve the 17-bit word cadence. This corrects analogue
CPt/CPu/CPus/SUVu transmission and digital reception, and digital CPd/SUVd
transmission and analogue reception. Existing V.90 CP, V.92 Ja, Jd and Jp
CRC code already excludes framing bits and was left unchanged.

The independent test uses a left-shifting 0x1021 polynomial register and
reflects the remainder for transmission, rather than calling the production
CRC helper. Coverage includes both acknowledgements, 2/4/8-point short-frame
alignment, CPt and CPu with one through six constellation sets and separate
codec masks, mandatory CPd, and all eight CPd optional-section combinations.
The former encoder and decoder agreed with each other and passed round-trip
tests despite being wrong on the wire. The old CRC fails the new oracle.

### SUVd reserved bits: Amendment 1 item 4, replacement Table 31

The digital encoder emits zero in bits 19:31. The analogue decoder now
reports nonzero reserved bits diagnostically without treating them as a
reason to reject a CRC-valid frame: the table says the receiver does not
interpret them. These bits still participate in the CRC. The regression
first corrupts one without updating CRC (rejected), then supplies its correct
CRC (accepted, with `reserved_ok=false`). Other message decoders still need
a separate review of their reserved-field acceptance rules.

## Amendment coverage and remaining work

| Source | Requirement | Implementation evidence or remaining gap |
| --- | --- | --- |
| Amd.1 item 1, Table 18 | PCM INFO1a filter capacities; 276-symbol MD units; reserved ones in 40:49 | `prepare_v90_info1a()` selects Table 18 after mutual capability, emits the baseline filter capabilities and reserved ones. `modem_engine.c` converts MD with 276. Startup tests cover the selection and MD receiver gating. Analogue MD generation remains zero-only. |
| Amd.1 item 2, Table 19 | V.34 upstream during short Phase 2 uses 35 ms MD units and -512 in 40:49 | The short-phase analyzer is offline. No complete live short-start analogue/digital session is established; do not apply Table 18's 276-symbol unit indiscriminately to Table 19. |
| Amd.1 item 3, 8.7.6 | TRN2u resets scrambler except second TRN2u of a silent renegotiation; context-specific differential seed | `v92_trn2u_tx_start()` covers initial/retrain entry from E1u. There is no implemented complete silent-renegotiation controller. Its second TRN2u must preserve scrambler state and seed the differential encoder from E2u; calling the existing reset helper there would be wrong. |
| Amd.1 item 4, Table 31 | Silent-period request at bit 32, CPu acknowledgement at 33, reserved receive behavior | Codec supports both fields; reserved acceptance fixed here. Digital initial startup requests no silence. Analogue controller explicitly rejects a silence request; renegotiation remains absent. |
| Amd.1 item 5, 8.6.6 | Differential SCR with Jp-prime continuity | Fixed and independently waveform-tested here. |
| Amd.1 item 6, 9.11 | drn=0 cleardown in CPt/CPu/CPus/CPd, acknowledgement and role-dependent waits, no silence request | Codecs can represent zero, but the analogue Phase-4 controller rejects zero as an unsupported profile. No complete protocol cleardown path with the specified waits is implemented. |
| Amd.1 item 7, Table 32 | MH sequence fields, denial reasons and reserved codes | No live MH codec/controller found. SpanDSP AT-command names are not evidence of modem-on-hold signaling. |
| Amd.1 items 8/9, 9.10.1/Figure 20 | MH transition timing | Superseded by Cor.1; do not implement the earlier unconditional optional-Tone-RT rule. |
| Amd.2, new 9.10.3 | Suspend and resume error correction; preserve original V.42 C/R roles; no new XID or link establishment | No live on-hold integration or dedicated suspension interface in `data_stack.h`. Reusing call reset or restarting LAPM would violate this requirement. |
| Cor.1 item 1, 9.7.1.2 | Digital retrain response: qualified Tone A, silence, Tone B; distinguish MH signals from a reversal | Complete V.92 retrain/MH discrimination is not established. Existing V.34 recovery does not prove this V.92 procedure. |
| Cor.1 item 2, 9.10.1 | Initiator sends silence then Tone RT; skip RT only if peer RT was detected during silence; finish each MH sequence | Open with the MH controller. Keep this condition explicit in future transitions. |
| Cor.1 item 3, 9.10.2.1 | Physical-role reversal preserves negotiated link-layer state | Open with Amd.2's V.42 suspension/resumption work. |
| Cor.1 item 4, Figure 20 | Corrected MH request/acknowledgement timing | Open with the MH waveform and procedure tests. |

## Verification for this change

- `./v92_startup_test --spec-only`: passes the independent CRC, reserved-bit
  and SCR tests. Each actual wire defect was reproduced before its fix.
- `./v92_startup_test --core-only`: all six core pairs pass (both laws,
  zero/measured DIL, first-CPd erasure), including payload checks.
- `./vpcm_loopback_test --all-tests`: passes.
- `./v92_proc_eval_test`: passes the offline clause 9.2 evaluation suite.
- Full `./v92_startup_test`: still fails in the 48 kHz PCMU zero-DIL audio
  case at the analogue Phase-4 DATA assertion. This was already reproduced
  before these changes and remains unresolved. The source/bitstream fixes
  do not establish analogue audio convergence.

The immediate physical-bearer gap remains the adaptive analogue receiver.
The next protocol audit should cover 9.6 failure/recovery and 9.7 retrain,
then add 9.8/9.9 renegotiation and parameter exchange before connecting
9.11 cleardown or 9.10 modem-on-hold to a live session. The modem-on-hold
work must include the link layer from the outset, per Amd.2 and Cor.1.

## Follow-up: 16 kHz analogue baseline

The analogue default and audio startup matrix now use 16 kHz, with direct
8-to-16 kHz network-DAC reconstruction. The reconstruction/chunk checks pass;
the PCMU zero-DIL audio startup still fails at the analogue Phase-4 DATA
assertion. The historical 48 kHz result above is retained as the audit's
baseline evidence, not the current configured audio rate.
