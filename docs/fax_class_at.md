# Fax service classes over the AT interface (T.31 class 1, T.32 class 2.0)

## What was wrong

`data_interface.c` ran a bare `at_init()`. SpanDSP's AT interpreter parses the
whole `+F` command set out of its generated dictionary, so the *capability*
commands looked fine — `AT+FCLASS=?` answered `0,1,1.0`, `AT+FCLASS=1` was
accepted, `AT+FTM=?` listed the modulations. That made the interface look like
it supported fax.

It did not. Every class 1 *action* command answered `ERROR`:

    AT+FTS=8   ERROR
    AT+FRS=1   ERROR
    AT+FTM=96  ERROR
    AT+FTH=3   ERROR

`process_class1_cmd()` (`at_interpreter.c`) parses the argument and then
dispatches to `s->class1_handler`. Nothing registered one, and nothing supplied
the V.21/V.27ter/V.29/V.17 fax datapumps such a handler would have to drive.

There is a second, spec-correct `ERROR` that masks this: T.31 8.3 requires the
class 1 action commands to fail while the phone is on hook. A probe that never
takes the modem off hook cannot tell the two apart — which is why the test
below dials before it asserts anything.

## What it does now

The AT interpreter is SpanDSP's T.31 fax modem (`t31_state_t`), whose own
`at_state_t` carries the Hayes set. T.31 registers the class 1 handler and owns
the fax datapumps, so the action commands work; `t31_at_rx()` replaces
`at_interpreter()`, and the modem-control callbacks (dial, answer, hangup) pass
through T.31 unchanged.

`AT+FCLASS=0` leaves the data path exactly as before: T.31 is then only an AT
parser, and the engine owns the audio. A non-zero FCLASS makes
`di_fax_active()` true, and the engine hands the call's audio to the fax
datapumps instead of running V.8 and the V.34/V.90 startup — T.30 negotiation
happens in the DTE, above `+FTM`/`+FRM`/`+FTH`/`+FRH`. The interception sits at
the top of `me_rx_audio()`/`me_tx_audio()` and `me_rx_g711()`/`me_tx_g711()`,
converting G.711 to the linear PCM the fax modems take.

`di_on_connected()` also changes for a fax call: T.31 reports `OK` and sits in
off-hook command mode waiting for the action commands, rather than the
`CONNECT <rate>` a data call reports.

One fix in the vendored SpanDSP: `at_cmd_plus_GCAP()` answered only the
`AT+GCAP?` form, so the bare `AT+GCAP` V.250 6.1.9 defines — the form fax
software probes with — got a bare `OK`, which reads as "no fax support".

## Coverage

`fax_class_test` (in `make test`) drives the real PTY a DTE sees. It asserts
the capability replies, that the action commands are `ERROR` on hook, and then
dials, calls `di_on_connected()`, and requires that `+FTS`/`+FRS` complete,
that `+FTM=96` and `+FTH=3` report `CONNECT`, and that each **puts a carrier on
the wire** (measured RMS, ~4550 for V.29 9600 and ~3220 for V.21 flags). A
command that is accepted and drives nothing would pass a result-code check
alone.

The test runs a pump thread feeding 20 ms frames both ways, because T.31's
state machine advances on samples, not wall time: without it `+FTS`, `+FRS` and
the `CONNECT` after `+FTM` never complete, which looks like rejection and is
not.

# Class 2.0 (T.32)

Class 1 and class 2.0 are the two divisions of labour between DTE and DCE.
Class 1 puts T.30 in the DTE, which drives every HDLC frame and image carrier
itself. **Class 2.0 puts T.30 in the modem**: the DTE declares its capabilities
(`+FCC`), reads what was negotiated (`+FCS`), and hands over or takes back page
image data (`+FDT`/`+FDR`). That is a whole fax terminal, not a command set, so
`fax_class2.c` is a T.32 command layer over SpanDSP's `fax_state_t` — a
complete T.30 terminal with the same fax datapumps under it.

SpanDSP's own `+F` class 2 command handlers are `TODO` stubs, so none of this
comes from the library's AT layer; only the `+FCLASS` list is SpanDSP's, and it
now advertises `0,1,1.0,2.0`. While class 2.0 is selected the DTE's AT lines go
to `fax_class2.c`; a line it does not claim goes on to the T.31 interpreter,
which keeps ownership of the S registers, echo and result-code settings.

## Image data

T.32 carries compressed T.4/T.6 image data across the DTE link, and SpanDSP's
T.30 sends and receives TIFF files. SpanDSP's own T.4 codec bridges the two,
so nothing here touches libtiff:

- `+FDT` — the DTE's compressed page goes through `t4_rx`, which writes the
  TIFF T.30 transmits. That also recovers the row count, which the DTE never
  states, and rejects a page the DTE garbled rather than putting it on the
  line.
- `+FDR` — the received TIFF page goes back out through `t4_tx` in the
  negotiated format, so the DTE gets a stream a fax machine would have sent.

### +FBO, phase C bit order (8.5.3.4)

The octets T.4 hands us are transmitted LSB first — `t4_tx_get_bit()` takes bit
0 of each octet first, and T.31's class 1 path does the same with the DTE's own
octets and no reversal at all. So the stream this module carries by default is
the one a class 1 DTE sees, which is `+FBO`'s **direct** order: `+FBO=0` changes
nothing, and the reversed setting flips each phase C octet on its way across
the DTE link. The reversal is applied *before* DLE stuffing on the way out and
*after* unstuffing on the way in, because T.32 3.2's stuffing is about the
octets that appear on the link.

`+FBO` has two halves: the phase C image data, and the negotiation frame data a
DTE reads with `+FNR`. 0 is direct in both and 3 is reversed in both; of the two
mixed values this takes 1 as "phase C reversed", the reading that puts the phase
C bit in the low position. The negotiation half is **not** applied: the one
negotiation report that carries frame octets, `+FNF:`, renders them as hex text
rather than as octets on the link, and which order that hex should be in is a
convention there is no spec text to hand to settle. It is reported in the
frame's own order. So 0/1 behave as 2/3 do.

### +FNR, negotiation message reporting (8.5.1.11)

`AT+FNR=<rpr>,<tpr>,<idr>,<nsr>`, four flags, default all zero. This used to be
parsed as a single value, so the `AT+FNR=1,1,1,1` fax software actually sends
answered `ERROR`.

The reports come from a `t30_set_real_time_frame_handler()`, which is the only
place the negotiation messages exist *as messages* — by Phase B they have
become state.

- **rpr** — `+FIS:` decoded from the received DIS/DTC: what the far end says it
  can do. Only the fields T.32 has a subparameter for are read; the ones this
  DCE cannot act on (BF, JP) are reported as zero rather than guessed at.
- **tpr** — `+FCS:` from the DCS we send.
- **idr** — `+FCI:`/`+FTI:` with the remote's identification. Reported from
  Phase B rather than from the CSI/TSI frame, because the frame handler runs
  *before* T.30 decodes the identification.
- **nsr** — `+FNF:` with the FIF of a received NSF/NSC/NSS frame, in hex.

`+FCS:` is reported exactly once either way: `tpr` moves it to the DCS instant
rather than adding a second one. With `tpr` clear the Phase B report still goes
out, because a transmit session has no other moment at which the DTE learns
what was negotiated (see the `+FDT` deviation above).

## Implemented

Actions: `+FDT` (8.3.3), `+FDR` (8.3.4), `+FKS` (8.3.5), `+FIP` (8.3.6).
Parameters: `+FCC` (8.5.1.1), `+FIS` (8.5.1.2), `+FCS` (8.5.1.3),
`+FLI`/`+FPI` (8.5.1.5), `+FCR` (8.5.1.9), `+FPS` (8.5.2.2), `+FHS` (8.5.2.7),
`+FNR` (8.5.1.11), `+FBS` (8.5.3.2), `+FBO` (8.5.3.4), `+FMI`/`+FMM`/`+FMR`,
and the accepted-and-stored set `+FCQ +FIE +FCT +FMS +FEA +FFC +FAA +FRY`.
Reports: `+FCS:` for the negotiated session, `+FIS:` for the far end's
capabilities, `+FCI:`/`+FTI:` for its identification, `+FNF:` for its
non-standard frames, `+FPS:` per page, `+FHS:` at the end of the session.

An omitted subparameter keeps its current value, which is what `,` means in a
T.32 list — `AT+FIS=,5` changes BR alone.

## Deviations, and why

**`+FDT` takes the page before it reports `+FCS`.** T.32 8.3.3 has the DCE
complete Phase B, report the negotiated session parameters, and then take the
page. SpanDSP's T.30 decides what to do at the DIS it receives and needs the
document to exist by then, so a `+FDT` here answers `CONNECT` and takes the
page first. The DTE still gets its `+FCS` report — after the data rather than
before it. In the ordinary sequence (`+FDT` straight after `ATD`, while the
call is still being set up) the spool is complete long before Phase B, and
where the report lands is the only difference the DTE can see. A DTE that
adapts its image format to the reported `+FCS` will not see it in time.

**`+FHS` mapping is coarse.** T.30's completion codes are finer grained than
T.32's two-digit status. The ones with a clear counterpart are mapped; the rest
fall back on the "unspecified phase B/C/D error" codes, choosing the phase from
where the failure was. Inventing a precise code for an error T.32 does not
enumerate would be worse than reporting the phase honestly.

**`+FBS` reports `0,0`.** There is no separate DTE buffer to report a size for;
`t4_rx` consumes the stream as it arrives.

## Coverage

`fax_class2_test` (in `make test`) runs a whole session each way against a
plain SpanDSP fax terminal, so nothing is graded by the code under test. It
drives the class 2.0 side exactly as a DTE does — AT lines in, DLE-stuffed
image data in and out — and **compares the page that arrives against the page
that was sent, raster by raster**. Both directions pass with zero differing
rows.

It then runs both sessions again at `+FBO=1`, reversing on the DTE side, and
requires the pages to arrive intact — which they can only do if the DCE
reverses in both directions and in the right place relative to the stuffing.
On the transmit side that round trip is its own control (an ignored `+FBO`
would put a reversed page on the line and the far end would reject it); on the
receive side it is not, since the test would be undoing a reversal that never
happened, so the two captured DTE streams are compared directly and must be
bit reverses of each other, and must differ.

That comparison is the point. The interesting failures here all report `OK`: a
page that negotiates and transfers as garbage, or a stream handed to the DTE in
a format it did not ask for, would pass a result-code check.

`test_fnr` runs the same session shape with reporting off, fully on, and then
**against a far end configured the other way** — T.6 and ECM disabled. Our own
`+FIS` defaults are `DF=3, EC=2`, so that last one is what separates decoding
the received DIS from echoing our own parameters back; both would look right
otherwise. The `+FNF:` check uses an NSF the far end was given, so the hex is
compared against a known frame.

`fax_class_test` covers the other half — that class 2.0 is *reachable* through
the real PTY the DTE sees (`AT+FCLASS=2.0`, a `+FCC?` only this module answers,
an `ATE0` that must still reach T.31, and the way back to class 0), which
`fax_class2_test` bypasses by calling the module directly.

## Not done

- Class 2 (the pre-standard `AT+FCLASS=2`) — only 2.0 is offered. The two are
  not compatible, and 2.0 is the ITU-T one.
- `+FBO`'s negotiation-frame half — see above.
- Polling (`+FSP`, `+FLP`, `+FPI` beyond storing the ID), sub-addressing,
  passwords and non-standard frames (`+FSA`, `+FPW`, `+FNS`, `+FPA`) — parsed
  where they are simple parameters, but nothing acts on them.
- `+FDD`/`+FIT`/`+FLO`/`+FPR` — these are T.31 DTE-link parameters and stay
  with the T.31 interpreter.
- T.38 — `t31_init()` is given no T.38 packet handler and the context is put in
  audio mode.
- No hardware interop: as with everything else here, the automated tests cover
  offline and loopback paths only. A real fax session against a far-end fax
  machine is unverified — and for class 2.0 that includes the `+FDT` ordering
  deviation above, which no loopback against our own sequencing can expose.
