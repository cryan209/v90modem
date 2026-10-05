# V.120 conformance audit

Date: 2026-10-05. Scope: the asynchronous, default-LLI, UI-only circuit-mode
profile in `clear_channel.c`, `clear_channel.h`, `clear_channel_test.c`, and
its engine/PTY integration. This is an implementation audit, not an ISDN
hardware interoperability result. No protocol implementation was changed.

## Sources and method

- ITU-T V.120 (10/1996), `ITU Docs/T-REC-V.120-199610-I!!PDF-E.pdf`.
- Corrigendum 1 (05/1999), `ITU Docs/T-REC-V.120-199905-I!Cor1!PDF-E.pdf`.
- Direct source trace and a standalone probe compiled from current
  `clear_channel.c`, `at_ms.c`, and `spandsp-master/src/at_interpreter.c`.
  SpanDSP's static library supplies HDLC and interpreter dependencies.
  The probe constructs FCS-valid frames independently and feeds their bits
  through the public DS0 receiver. Its source and reproduction command are
  in `v250_command_conformance_audit.md`.

Q.922 is a normative dependency of clauses 4 and 6.2.1. This audit does not
claim a complete Q.922/HDLC conformance review. The local bearer is SIP/G.711
with byte-exact PCM; it has no Q.931 ISDN call setup or low-layer compatibility
negotiation. Both ends must agree on this restricted profile beforehand.

## Confirmed defects

| ID | Clause | Source and observed behaviour | Consequence / correction |
|---|---|---|---|
| V120-1 | 6.2.2.3, Table 4; 4.1 | `cc_v120_frame_header()` sets C/R from the calling role. The answerer emits `0a 01 03 83`. Table 4 is symmetric: commands use C/R=0 in both directions, responses use 1. UI data is a command. | Both ends must send `08 01 03 83` for default-LLI UI data. Future response frames must explicitly use C/R=1. Remove the role-based command convention and update the test which currently asserts the wrong answerer header. |
| V120-2 | 3.1, 3.1.2.1 | `v120_frame()` walks a chain of CS extension octets. Header `03 00 80` followed by `AB` is accepted and delivers `AB`, with zero bad frames. A missing CS (`H=03`, end of frame) is also accepted without a bad-frame count. | The header has at most H plus one CS octet. With H.E=0, require one available CS octet with CS.E=1; reject before delivering any payload otherwise. Do not swallow arbitrary payload looking for E=1. |
| V120-3 | 6.2.2.1, Table 3; 6.3.2 | RX checks EA but never decodes or filters LLI. A frame for LLI 257 (`08 03`) delivers `AB` into the LLI-256 PTY. | Route only the configured logical link. Do not merge another link or management/signalling LLI into DTE data. Keep unsupported-LLI statistics separate from corrupt frames. |
| V120-4 | 3.1.2, 3.2.3, 3.2.4.1 | RX discards even a well-formed CS octet and TX never consults RR. The current receive-variant test uses CS=`80`, which includes RR=0, and merely expects its removal. | A CS-capable UI profile must maintain DR/SR/RR and honour receive-ready flow control. Alternatively explicitly limit the supported profile to CS disabled by prior agreement; accepting and ignoring an active peer's flow-control state is not equivalent to supporting it. |
| V120-5 | 3.2.1.1; 7.2.1, 7.2.2(5) | RX increments `rx_breaks` for every BR-marked frame, without delivering a break to the DTE. There is no outbound break callback or BR frame generation. | Preserve break ordering after the frame's queued characters; represent break start/end in the byte/PTY API. A counter alone does not implement TE2 break delivery. |

## Correct behaviour and bounded omissions

| Clause | Assessment |
|---|---|
| 2.4 | Restricted 56k uses the first seven transmitted bits and forces the remaining bit to one. The reverse path ignores it. The code's DS0 MSB-first packing is consistent with this rule. |
| 3.1.1, Figure 4; 3.1.1.5, Table 2 | `83` is a valid async H octet: E=1, BR=0, reserved/error bits zero, B=F=1. B/F segmentation concerns synchronous HDLC. The async RX currently accepts `82` and delivers its bytes; treat this as permissive handling of an out-of-profile input, not proof that synchronous reassembly is implemented. |
| 3.2.1.1; 7.2.1 | Transporting PTY bytes without start/stop bits, in order, in variable-sized batches is appropriate for the fixed 8-bit asynchronous profile. Serial parity/framing variants are not implemented by the byte-only interface. |
| 3.1.3 | Initial/idle HDLC flags are appropriate for this B-channel-style profile. |
| 3.2.2 | TX has 256 data octets plus one H octet: 257 information octets, excluding address/control/FCS. This is not itself an overflow defect. The header's assertion that N201 is 260 is not established by V.120; N201 depends on Q.922 and agreed parameters. RX permits a larger frame than TX; an agreed receive limit needs explicit enforcement. |
| 3.2.3 | CS use is optional. Its initialization/change/flow-control rules apply when that facility is selected. A missing initial CS is not an unconditional defect in a CS-disabled profile. |
| 4.1; 3.2.1 | UI-only service is permitted. Acknowledged multiframe service is recommended for data integrity but its absence does not invalidate an explicitly agreed UI-only profile. I/SABME/RR frames are currently discarded. |
| 4.2.2 | XID link verification is optional for applications not requiring it. Our preset peers omit it. Add XID command/response and TM20/NM20 for peers that require verification; this is an interoperability extension, not an unconditional mandatory handshake. |
| 4.2.3 | A future verification implementation must handle the XID/SABME mode collision, rather than dropping all non-UI traffic. |
| 6.2.2.1, Table 3; 6.3.2 | LLI 256 is the default. Multiple LLIs and their establishment procedures are outside the advertised single-link profile. |
| 6.3.1; 6.3.2.4.5 | Q.931 bearer/LLC signalling is absent on SIP. Corrigendum 1 changes LLC user-rate/modem-type coding, not H/CS or UI data framing; it is relevant if ISDN signalling is added. |

## Fix order and required evidence

1. Correct symmetric C/R, enforce the two-octet header bound, and filter LLI.
   Add independent-wire assertions for commands in both roles, FCS-valid
   malformed CS cases, and another LLI. Existing self-loopback passes cannot
   establish these conventions.
2. Decide and document the supported CS profile. If enabled, add initial CS,
   ordered state updates, RR pause/resume, and control-only frames permitted
   while flow-controlled. Test that a paused peer receives no user data.
3. Add ordered break start/end delivery, then optional XID verification.
4. Audit Q.922 FCS/control/P-F/maximum-frame rules and confirm the profile
   against a real terminal adaptor or a conforming independent implementation.

Do not widen the async path into synchronous HDLC merely by accepting B/F
combinations. That requires a separate mode with encapsulated inner HDLC
frames, reassembly, and the relevant error/idle handling of 3.2.1.2 and 7.3.
