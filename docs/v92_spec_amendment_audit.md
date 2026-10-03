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
| Amd.1 item 7, Table 32 | MH sequence fields, denial reasons and reserved codes | `v92_mh.c` codec + framer, graded by an independent CRC oracle in `v92_mh_test` (every signal x info, every single-bit error, undefined signal ignored, reserved info flagged not rejected). Not yet wired to the INFO-modulation line signals. |
| Amd.1 items 8/9, 9.10.1/Figure 20 | MH transition timing | Superseded by Cor.1; do not implement the earlier unconditional optional-Tone-RT rule. |
| Amd.2, new 9.10.3 | Suspend and resume error correction; preserve original V.42 C/R roles; no new XID or link establishment | `v42_suspend()`/`v42_resume()` in vendored SpanDSP, `ds_suspend_link()`/`ds_resume_link()` in `data_stack.h`. Timer frozen in ms (rescaled to the new rate), cut-off frame discarded both ends, immediate RR/RNR P=1 checkpoint if I-frames are outstanding. `v42_link_test`: 30 s clocked hold with noise, resume 9600->4800 and 4800->28800, byte-exact, one XID, worst 296 ms to data; the same test with suspend removed disconnects. Not yet called by the engine. |
| Cor.1 item 1, 9.7.1.2 | Digital retrain response: qualified Tone A, silence, Tone B; distinguish MH signals from a reversal | Complete V.92 retrain/MH discrimination is not established. Existing V.34 recovery does not prove this V.92 procedure. |
| Cor.1 item 2, 9.10.1 | Initiator sends silence then Tone RT; skip RT only if peer RT was detected during silence; finish each MH sequence | `v92_mh_ctrl_t`: 70 ms silence, RT >= 50 ms (20 after an MH), skip only on peer RT during the silence, every transition deferred to a sequence boundary. Tested: Cor.1 skip, and every MH run in every scenario is a multiple of 40 bits. |
| Cor.1 item 3, 9.10.2.1 | Physical-role reversal preserves negotiated link-layer state | Satisfied by construction if the engine resumes rather than re-initialises: the C/R addresses are fixed at `ds_init_v42*()`, and `ds_resume_link()` does not touch them. Documented in `data_stack.h`. |
| Cor.1 item 4, Figure 20 | Corrected MH request/acknowledgement timing | Procedure-level: Figures 20-24, 9.10.1.1's 2 s + round-trip timeout, responder retrain discrimination, T1, and on-hold exit to Phase 1 run two controllers against each other over a delayed 600 bit/s line in `v92_mh_test` (in `make test`). Waveform level and engine integration still open. |

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

## Follow-up: modem-on-hold controller (2026-10-03)

`v92_mh.c` implements 8.9.2 (Table 32 per Amd.1, Tables 33/34) and the
9.10.1/9.10.2 transactions per Cor.1, as a controller with no audio: it takes
detector states (peer RT, silence, ANSam, Tone B reversal, QC/CM) and MH
bits, and reports what to transmit and an action FIFO (`SUSPEND_LINK`,
`ON_HOLD`, `PHASE1_ANSWER`, `PHASE1_CALL`, `RETRAIN`, `DISCONNECT`).

One interpretation is fixed in one place: Table 32/33's 4-bit entries are
read as numbers with the lower-numbered bit least significant, the reading
under which Table 33 is monotonic and every signal code is odd.  No foreign
MH capture exists to confirm it.

Still open: modulating/demodulating MH on the INFO channel and RT in the
engine, connecting the controller's `SUSPEND_LINK` to `ds_suspend_link()` and
Phase 1-4 completion after a hold to `ds_resume_link()`,
the null-CM/JM cleardown from on-hold (flagged as `null_cm`), and 9.11's
drn=0 cleardown.

## Follow-up: engine integration (2026-10-03)

`modem_engine.c` drives the controller through `v92_mh_line` when
`ME_V92_MH=1`, on the digital side of a call whose INFO0s were mutually
V.92.  Receive: every codeword goes to the MH demodulator first; once a
transaction starts, MH owns the line.  Transmit: MH supplies the codewords
while engaged.  Actions: `SUSPEND_LINK` -> `ds_suspend_link()`;
`PHASE1_ANSWER`/`PHASE1_CALL` and the on-hold state -> V.8 restarted in the
same SIP call, with `data_stack_start_online()` resuming the suspended link;
`RETRAIN` -> `restart_v90_phase2_locked()`; `DISCONNECT` -> hangup.

Not established: any of it against a real V.92 modem.  Known weakness: a
Tone RT that is a retrain is recognised only after the peer's reversal, so
the Phase 2 restart that follows may miss that reversal.  Short Phase 1
(QC) after a hold is not used; reconnection runs full V.8.
