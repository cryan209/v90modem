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

## Reading the specifications

`ITU Docs/` has T.31, T.32 (and its Amd 1), T.30, T.4 and T.6. Reading T.32
against the code it had already produced found four things wrong, which is
worth recording: the `+FHS` codes were assigned across the wrong phases and
directions, `tpr=0` must *suppress* the `+FCS:` report rather than leave it
unconditional, `+FPS:` carries four line counts this had reduced to one, and
`+FDT` must end in `ERROR` when the remote rejects the page. It also confirmed
two readings that had been taken on internal evidence alone — `+FBO`'s value
map and its stuffing order — and turned a "not done because the convention is
unsettled" into a "not done, and Note 3 says not to".

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

Table 27/T.32 settles the four values, and the reading taken here was right:
`+FBO=1` is reversed phase C with direct phase B/D, `+FBO=2` the other way
round. Note 2 to 8.5.3.4 confirms the placement too — "transparency mechanisms
(e.g. for `<DLE>` characters) shall be applied to the data **after reversal**,
i.e. as the data will be transmitted on the DTE-DCE link".

The phase B/D half governs T.30 control messages as they appear in the
`+FHT:`/`+FHR:` reports of 8.6, which `+FBU` enables — so `+FBO=2` and `+FBO=3`
reverse the octets in those reports and nothing else. Note 3 is explicit that
`+FBO` does **not** affect the `+FNC`, `+FNF` or `+FNS` responses, so those
stay in the frame's own order.

### Multi-page documents (8.3.3.3, 8.3.3.7)

T.32 8.3.3.3 has the DTE issue a `+FDT` for every page, and 8.3.3.7's
`<DLE><ppm>` at the end of each one says whether another follows. So the pages
of a document **accumulate into a single multi-page TIFF**, which is exactly
what T.30 needs in order to send MPS between pages and EOP after the last: it
takes the post page message from the page count, and this way the count is
right. A `+FDT` while a document is open adds to it rather than starting a new
file.

The document is not transmitted until the DTE ends a page with EOP, and T.30 is
not started while one is still being handed over — started on half a document
it sends the first page and an EOP after it. If the call is answered mid-way,
the `+FDT` that completes the document is what starts the session; `+FCT`
bounds how long a DTE may leave one half done, and a document that is never
finished is discarded when the call ends.

`<DLE>;` (EOM, "another document, renegotiate in Phase B") is accepted and
keeps the document open like MPS. Whether T.30 then sends EOM or MPS follows
the **actual** page formats — `t4_tx_next_page_has_different_format()` — rather
than the DTE's code, so an EOM between two pages of the same format goes out as
MPS.

A non-final `+FDT` is answered `OK` as soon as the page is taken, since nothing
has been transmitted yet; only the final one waits for the far end and reports
its verdict. That is the same ordering deviation as below, not a new one.

### The post page response is held for the DTE (8.3.4.3, 8.4.3, 8.5.2.2)

A receiving DCE does not answer the far end's post page message on its own. It
works out the response, reports the page to the DTE, and **holds** it; the
DTE's next `+FDR` releases it, carrying whatever the DTE left in `+FPS`. That
is how a DTE refuses a page it did not like (`+FPS=2`, RTN, retrain requested)
or asks for a procedure interrupt (`+FPS=4`/`5`) after seeing it.

So a receiving session needs **one more `+FDR` than it has pages**, and the last
one is what Table 17 completes with `+FHS:` and `OK`.

**A second change to the vendored SpanDSP**, which sends the response the moment
the page ends. `t30_set_post_page_response_hold()` makes it stop there instead,
and `t30_release_post_page_response(s, ppr)` sends it with the DTE's verdict.
A repeat of the post page message while the response is held — which is what
the far end does when nobody answers — is ignored rather than assessed as a new
page. Both are guarded by a flag nothing sets unless a class 2.0 DTE is
driving, so ordinary sessions are untouched.

**The wait is bounded by `+FCT`** (8.5.2.6), which is what T.32 already
provides for it: "how long the DCE will wait for a command after having
transmitted all available Phase C data", 1 s units, default and mandatory value
30 s. On expiry the receive side does what 8.5.2.6 asks — sends a T.30 DCN to
the far end and executes an implied orderly abort — and reports `+FHS:02`,
Table 20's "call aborted, from +FKS or `<CAN>`", which is what an implied abort
is. `+FCT=0` disables the timeout; the Recommendation does not say what 0 means,
and that is the reading T.31's `+FIT` uses for the same value.

Sending the DCN needed a third small SpanDSP addition, `t30_disconnect()`: the
library can be told the far end hung up, but had no way to hang up *politely*.
`+FKS` uses it too — 8.3.5 asks for "a DCN message at the next opportunity",
and it previously just dropped the line.

**T.30's own timers are usually tighter than `+FCT`.** The far end repeats its
post page message after a few seconds and gives up after a few repeats, well
inside the default 30 s. So on a real call a dawdling DTE loses it to T.30
before `+FCT` fires; the timeout is the DTE-facing bound, not the only one.

`+FCT` bounds the transmit side as well, which 8.5.2.6 covers in the same
breath: "For transmission (`+FDT`), when this timeout is reached, the DCE shall
properly terminate any Phase C data transfer in progress, then execute an
implied `+FKS` orderly abort command." The wait it bounds there is a `+FDT`
that has been opened — `CONNECT` given — and is not being fed. Without it the
module sits in data mode for the rest of the call, reading everything the DTE
types, including any command it tries to escape with, as image data. Every
block the DTE hands over restarts the timer, so a page that takes longer than
`+FCT` to cross a slow DTE link is not aborted in the middle of a transfer that
is going fine.

### Procedure interrupts: +FIE, +FVO (8.5.2.1, 8.4.4.2, 8.3.3.8, 8.3.4.8)

There are two of them and they go opposite ways.

**A transmitting DTE asks** with 8.3.3.8's `<DLE><pri>`, embedded in the phase C
stream before the page terminator. T.30 carries the request in the post page
message, which becomes a PRI-Q, and the remote grants by answering PIN or PIP.

**A receiving DTE asks** by setting `+FPS` to 4 (PIN) or 5 (PIP) before the
`+FDR` that releases the held post page response, which makes that response a
PIN or PIP.

`+FIE=1` enables all of it. With `+FIE=0` (the default) a remote's requests are
ignored and not reported, and the PRI-Q codes in `+FET:` are replaced by their
non-PRI equivalents, as 8.5.2.1 requires. With `+FIE=1` a received PRI-Q is
reported in `+FET:` and `+FPS` is adjusted to 4 or 5, so the DTE can leave it in
place to accept or overwrite it to refuse.

`+FVO` (8.4.4.2) is reported when the interrupt has been negotiated. The
session is suspended, the DCE stays off-hook and `+FCLASS` is unchanged; the
DTE decides whether to resume or hang up.

While implementing this, 8.3.3.7 turned out to specify the **page terminator**
too, and this had it wrong: a transmitted page ends with `<DLE><ppm>` — `,` for
MPS, `;` for EOM, `.` for EOP — not `<DLE><ETX>`, which is the *receive*
direction's terminator (8.3.4.4). All three are accepted now, and `<DLE><ETX>`
is kept as a lenient synonym for EOP because that is what a stream-oriented DTE
tends to send. Table 15 also shows a `+FDT` completing with `OK` or `ERROR` and
nothing else, so the `+FPS:` report this used to emit there is gone — 8.4.3
scopes that report to `+FDR`, and the page status goes into the `+FPS`
parameter, which `AT+FPS?` reads.

**One change to the vendored SpanDSP.** Its receiver has the interrupt case as
an empty `if (s->remote_interrupts_allowed) { }` TODO, so a receiving station
could never answer PIP or PIN. The post page response now honours
`local_interrupt_pending` — PIP in place of MCF, PIN in place of RTN, which is
the same page verdict with the request attached. It is guarded by a flag that
is false unless a DTE explicitly asks, so ordinary sessions are untouched, and
it is what makes both directions testable: the far end in the test grants an
interrupt because of it. Removing it fails four checks.

### +FNS, non-standard frames (8.5.1.6, 8.4.2.4)

`AT+FNS="<hex octets>"` loads the FIF of a non-standard frame, up to 90 octets.
Which frame carries it is T.30's decision, not the DTE's — NSF goes with a DIS,
NSS with a DCS, NSC with a DTC — so the one string is offered for all three and
whichever frame gets sent takes it.

The octet values need no transformation. 8.5.1.6 says each octet is sent LSB
first, so `"D8A2"` is the bit pattern `0001101101000101` on the line, which is
what HDLC does with the octets `0xD8, 0xA2`. 8.4.2.4 reports received frames in
the same convention, and 8.6's `+FHR:` example agrees.

Per 8.5.1.6: spaces between octets are ignored, a repeated `+FNS` **appends**,
`+FNS=""` resets to the null string, and `+FNS=?` reports the capacity. A
half octet or a non-hex character is refused.

Received non-standard frames are 8.4.2.4's three separate reports — `+FNF:` for
an NSF, `+FNC:` for an NSC, `+FNS:` for an NSS — with the FIF in hex,
**separated by spaces**. Earlier versions reported all three as `+FNF:` with the
octets run together; both were wrong.

#### The FCF low bit

T.30 uses the low bit of the FCF two different ways, and getting it wrong is
silent. In DIS/DTC, CSI/CIG and NSF/NSC it is a real distinction and the values
must be matched exactly. In DCS, TSI and NSS it is a don't-care that SpanDSP
sets from whether a DIS has been received — so a TSI arrives as `0x42` or
`0x43` — and those must be matched with it masked off, which is what T.30's own
dispatcher does.

Matching TSI and NSS exactly meant every TSI from a *calling* station was
reported as a `+FCI:` instead of a `+FTI:`, and a received NSS was never
reported at all. The polling test had hidden the first of those by asserting
`+FCI:` **or** `+FTI:`; it is exact now, and there is a test for each.

### Error correction mode (T.30 Annex A, +FIS/+FCS EC)

ECM is what any modern peer negotiates, so this is the ordinary case rather
than a variation on it. `+FIS`'s and `+FCS`'s EC subparameter is 0 for off, 1
for 64-octet ECM frames and 2 for 256-octet ones; `+FCC` offers 2 and that is
also the default `+FIS`.

Two things about it are not shared with a plain session.

**8.4.3's line counts have to be latched at the end of the page.** They come
from SpanDSP's T.4 receiver, and in an ECM session T.30 has released that
receiver by the time the DTE collects the page --
`t30_get_transfer_statistics()` then reports zeros, because the operation it
reads from has finished. The page itself arrives intact, so this fails in the
one way that is easy to miss: every page of every error-corrected fax was
reported to the DTE as `+FPS:1,0,0,0,0`, a good status with nothing received.
The counts are now taken in the phase D handler, where the page has just ended
and the receiver that counted it is still alive, and held until the `+FDR`
that collects the page. Only one page is ever outstanding, because 8.3.4.3's
held post page response stops T.30 starting the next one.

**+FCS's EC is read off the DCS, not echoed from +FIS.** T.30 Table 2 bit 28
of the DCS is the frame size -- 0 is 256 octets, 1 is 64 -- and note 42 lets a
transmitter ignore a request for 64, so what the DTE offered is not what was
agreed. The DCS passes through the same real-time frame handler `+FNR` and
`+FBU` use, in whichever direction it goes, and the bit is latched there.
Mask the FCF with `0xFE` as everywhere else: a DCS's low bit is a don't-care.

**EC=1, T.30 A.3.2's other frame size, did not exist at all.** SpanDSP never
set DIS bit 7, never set DCS bit 28, and chose the size with a comment saying
so -- "Always use 256 octets per ECM frame, whatever the other end says it is
capable of". A DTE could ask for `EC=1` and get 256-octet frames with nothing
on the wire to say otherwise, in either role.

Three fixes in the vendored tree, and the split between them is A.3.1's: the
*transmitting* terminal chooses the frame size, and the *receiving* terminal
expresses a preference in bit 7 of its DIS/DTC. So a receiving DCE can only
ask, and a transmitting one decides. New `t30_set_ecm_frame_size()` carries
the preference; `build_dis_or_dtc()` puts it in bit 7; `set_from_dis()`
honours the far end's bit 7, or this terminal's own preference where the far
end expresses none; and `build_dcs()` sets bit 28 to say which size is in use.
256 stays the default in both roles, because Table 2 note 42 lets a
transmitter ignore the request and a receiver must handle 256 regardless.

And one that was simply inverted: `set_from_dcs()` read bit 28 as
`? 256 : 64`, against Table 2's `0 = 256 octets, 1 = 64 octets`, so a terminal
that received a DCS and then transmitted used the frame size the DCS had not
asked for.

### +FBU, HDLC frame reporting (8.5.1.10, 8.6)

`+FBU=1` reports every T.30 phase B and phase D frame, in both directions, as
`+FHT:` (sent) and `+FHR:` (received) followed by the octets in hex. It comes
off the same real-time frame handler `+FNR` uses.

8.6 requires the HDLC flags and FCS to be dropped, which is already the frame
SpanDSP hands over, and requires the report to precede the response derived
from it — so a received DIS reports as `+FHR: FF 13 80 ...`, the form of 8.6's
own worked example, and lands before the `+FIS:` it produced.

**ECM phase C data frames are excluded**, as 8.6 requires. They reach the same
handler — SpanDSP sends them through `send_frame()` like any other — and are
identified the way SpanDSP's own SSL-fax handler identifies them: a non-final
frame carrying `T4_FCD` or `T4_RCP`.

8.6 also says command echo must be off while `+FBU` is set: the reports are
unsolicited and interleave with whatever the DTE is typing. `ATE` is left
alone; the echo is suppressed only while `+FBU` is on.

### +FNR, negotiation message reporting (8.5.1.11)

`AT+FNR=<rpr>,<tpr>,<idr>,<nsr>`, four flags, default all zero. This used to be
parsed as a single value, so the `AT+FNR=1,1,1,1` fax software actually sends
answered `ERROR`.

The reports come from a `t30_set_real_time_frame_handler()`, which is the only
place the negotiation messages exist *as messages* — by Phase B they have
become state.

- **rpr** — `+FIS:` decoded from a received DIS, `+FTC:` from a DTC: what the
  far end says it can do. Only the fields T.32 has a subparameter for are read;
  the ones this DCE cannot act on (BF, JP) are reported as zero rather than
  guessed at.
- **tpr** — `+FCS:`, from the DCS we send in a transmit session (Table 13) and
  at the `+FDR` in a receive session (Table 16).
- **idr** — `+FCI:`/`+FTI:` with the remote's identification. Reported from
  Phase B rather than from the CSI/TSI frame, because the frame handler runs
  *before* T.30 decodes the identification.
- **nsr** — `+FNF:` with the FIF of a received NSF/NSC/NSS frame, in hex.

Table 22 is explicit that `tpr=0` **suppresses** the `+FCS:` report while still
loading the `+FCS` parameter, so `AT+FCS?` reads it either way. Note 1 to that
table spells out the consequence the DTE then lives with: without the report it
must send the format T.30 mandates (normal resolution, A4 length, 1728 wide,
1-D) or enable `+FFC` conversion.

Table 13 gates the `+FCS:` of a `+FDT` on the *first* subparameter rather than
the second, which contradicts both Table 22 and Table 16. Table 22 is the
definitive per-switch description and Table 16 agrees with it, so `tpr` is what
this implements.

## Implemented

Actions: `+FDT` (8.3.3), `+FDR` (8.3.4), `+FKS` (8.3.5), `+FIP` (8.3.6).
Parameters: `+FCC` (8.5.1.1), `+FIS` (8.5.1.2), `+FCS` (8.5.1.3),
`+FLI`/`+FPI` (8.5.1.5), `+FCR` (8.5.1.9), `+FPS` (8.5.2.2), `+FHS` (8.5.2.7),
`+FNS` (8.5.1.6), `+FIE` (8.5.2.1), `+FCT` (8.5.2.6), `+FNR` (8.5.1.11), `+FBU` (8.5.1.10), `+FLP` (8.5.1.7), `+FSP` (8.5.1.8), `+FAP` (8.5.1.12),
`+FSA`/`+FPA`/`+FPW` (8.5.1.13), `+FBS` (8.5.3.2), `+FBO` (8.5.3.4), `+FMI`/`+FMM`/`+FMR`,
and the accepted-and-stored set `+FCQ +FIE +FCT +FMS +FEA +FFC +FAA +FRY`.
Reports: `+FCS:` for the negotiated session, `+FIS:`/`+FTC:` for the far end's
capabilities, `+FCI:`/`+FTI:`/`+FPI:` for its identification, `+FNF:` for its
non-standard frames (and `+FNC:`/`+FNS:` for the other two kinds), `+FVO` for
a negotiated procedure interrupt,
`+FHT:`/`+FHR:` for the HDLC frames themselves, `+FPO`
for its offer to be polled, `+FPS:<ppr>,<lc>,
<blc>,<cblc>,<lbc>` and `+FET:<ppm>` per page, `+FHS:` at the end of the
session. 8.3.3.4's rule that `+FDT` ends in `ERROR` when the remote rejected
the page (RTN or PIN) rather than `OK` is implemented.

An omitted subparameter keeps its current value, which is what `,` means in a
T.32 list — `AT+FIS=,5` changes BR alone.

## Polling (8.5.1.7, 8.5.1.8, 8.4.2.2)

**Being polled — `+FLP`.** `+FLP=1` offers the spooled `+FDT` page for polling;
T.30's DIS bit 9 carries the offer, and SpanDSP sets that bit from having a
document to send, so what `+FLP` gates is whether the document is offered at
all. Only the answering side's DIS carries the bit, so a *calling* DCE hands
its page over regardless — there it is an ordinary send, not an offer to be
polled. The DCE resets `+FLP` to 0 once a polled document has gone.

**Polling — `+FSP`.** `+FSP=1` says the DTE can receive a polled document. A
received DIS with bit 9 set then raises the `+FPO` response, and the DTE
answers it with `+FDR` (to poll) or `+FDT` (if it would rather send). 8.5.1.8's
note makes `+FCR=0` act as `+FSP=0`, which this follows: with no receive
capability there is nothing to poll into. The DCE resets `+FSP` to 0 once a
polled document has arrived.

`+FSA`, `+FPA`, `+FPW` and `+FPI` supply the sub-address, selective polling
address, password and polling ID those sessions carry, and `+FAP` controls
whether the corresponding received frames are reported.

`+FLP=0` and a DTC that arrives anyway is specified as an orderly disconnect
with `+FHS:23`, and that is implemented — but it is **not** covered by the
tests, because a conformant far end reads DIS bit 9, sees no offer and gives up
without ever sending a DTC. That is the point of the bit, and it is what the
`+FLP=0` test asserts instead.

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

**`+FHS` mapping is approximate.** T.30's completion codes are finer grained
than T.32's status. The mapping now follows Table 20/T.32, which is organised
by phase *and direction* — 40-4F is transmit phase C and 90-9F is receive phase
C, so a code is only right alongside the direction it happened in. Codes with
no counterpart fall back on the "unspecified" code for their phase and
direction rather than on an invented value.

**`+FBS` reports `0,0`.** There is no separate DTE buffer to report a size for;
`t4_rx` consumes the stream as it arrives.

**No `<DC2>` handshake.** 8.3.4 has the DTE send `<DC2>` after the `+FDR`
`CONNECT` to start the data flowing. This DCE starts sending immediately. A DTE
that sends `<DC2>` is unharmed — it is discarded — but a DTE relying on the
pause to get ready does not get it.

**A retrain requested with `+FPS=2` is sent, but nothing follows it up.** The
RTN reaches the far end, and what it does about it is its business; this DCE
does not re-request the page.

**`+FVO` on a transmit-side interrupt does not accompany the `+FDT` result.**
Table 15 pairs them, but a `+FDT` issued before the call has already completed
by the time the far end grants anything — the ordering deviation above. The
`+FVO` still reaches the DTE, on its own.

## Reading the specifications

`ITU Docs/` has T.31, T.32 (and its Amd 1), T.30, T.4 and T.6. Reading T.32
against the code it had already produced found four things wrong, which is
worth recording: the `+FHS` codes were assigned across the wrong phases and
directions, `tpr=0` must *suppress* the `+FCS:` report rather than leave it
unconditional, `+FPS:` carries four line counts this had reduced to one, and
`+FDT` must end in `ERROR` when the remote rejects the page. It also confirmed
two readings that had been taken on internal evidence alone — `+FBO`'s value
map and its stuffing order — and turned a "not done because the convention is
unsettled" into a "not done, and Note 3 says not to".

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

`test_transmit` and `test_receive` each run twice more, at `EC=2` and `EC=1`,
so the page comparison and the line counts are checked in error correction
mode as well as without it -- every transfer test used to be `EC=0`, which is why the zeroed
line counts above survived a green suite. Removing the latch fails exactly the
two ECM receive rows and nothing else, which is what makes them a test of it.
Each also asserts `+FCS`'s EC field, against a far end that is ECM-capable in
every arm, so it separates reporting the DCS from echoing our own offer back.

The `EC=1` rows are graded **on the wire, from the far end's own frame
handler**: the DCS's bit 28, and the payload length of the `T4_FCD` frames
that carry the page. The bit alone would not do -- a DCS that says 64 and then
sends 256-octet frames passes a bit check and fails against a real peer -- and
the frame lengths alone would not either, since the receiver reassembles the
page from frames that carry their own lengths, so a peer that ignored bit 28
entirely would still deliver an intact page. The two directions test different
halves and are separately confirmed to bite: withholding DIS bit 7 fails the
three receive assertions and nothing else, because transmitting is driven by
our own preference; withholding the `set_from_dis()` change fails all six.

`test_fnr` runs the same session shape with reporting off, fully on, and then
**against a far end configured the other way** — T.6 and ECM disabled. Our own
`+FIS` defaults are `DF=3, EC=2`, so that last one is what separates decoding
the received DIS from echoing our own parameters back; both would look right
otherwise. The `+FNF:` check uses an NSF the far end was given, so the hex is
compared against a known frame.

`test_post_page_hold` takes a page, overwrites the good verdict the DCE
reported with `+FPS=2`, and requires **RTN** on the wire at the far end. There
is deliberately no "and nothing was sent before the release" check beside it: one
was written and measured, it reads the same with the hold removed, so it cannot
fail. The RTN assertion is what carries it — without the hold an MCF goes out at
the end of the page and the DTE's RTN never does — and that was confirmed by
removing the hold and watching it fail.

`test_held_response_timeout` sets `+FCT=2`, takes a page and then has the DTE
say nothing at all, requiring a DCN at the far end, `+FHS:02` at the DTE and
the outstanding command to complete. Disabling the expiry fails all three.

`test_transmit_timeout` opens a `+FDT` and stalls it, both after part of a page
and after none of it — two cases, because the timer is armed at `CONNECT` and
restarted per block, and the partial-feed case alone does not exercise the
first. It requires the transfer to be terminated, a DCN to go out, `+FHS:02`,
and plain AT commands to be understood again rather than swallowed as image
data. The DCN is asserted on **our own transmit path**, through the `+FBU`
frame report, which is generated inside `send_frame()`: a stalled `+FDT` means
T.30 never had a document, so the call collapses on its own account within the
first second and the DCN lands after the far end has stopped listening. That is
the scenario, not the implementation, and the weaker claim is the honest one.

`test_multipage_transmit` runs at `EC=0`, `EC=2` and `EC=1`, and the ECM arms
also check the DCS's frame size and the `T4_FCD` lengths across both pages --
T.30 A.1 rebuilds the ECM block per page, so a size carried over from a stale
partial page would show there. It sends two pages, the first ended with MPS
and the second with EOP, and requires the far end to hold **both, in order,
each the one that was sent** — plus a check that the two pages differ, so "both intact"
means something. It runs twice, with the call answered after the document and
again **between** the pages. The second needs a realistic gap to be worth
anything: at 3 s it passes with the deferred start removed, because T.30 has
not yet given up on having no document; at 10 s it does not, which is what
makes it a test of the deferral rather than of nothing.

`test_multipage_receive` is the other direction, and is where the class 2.0
bookkeeping and T.30's ECM buffering meet: one `+FDR` per page, with the post
page response held between them, then a last `+FDR` that takes no page and
ends the session. It runs at all three EC settings and checks each page's
image, its `+FET:` (0 = MPS after the first, 2 = EOP after the last) and its
line counts.

**The two pages are deliberately different lengths**, 80 rows and 72. The
counts are latched when a page ends and held until the DTE collects it, and
with pages of equal length a latch that was never updated would report the
right number for the second page as well -- the test would pass on a modem
that only ever measured page one. Both failure modes are confirmed to bite:
reading the statistics when the DTE asks, as the code did before, fails page
one of every arm and page two of the ECM ones; latching only the first page
fails page two of all three.

`test_page_rejected_rtn` is T.32 Annex II.8, "receive one page with line
errors and retransmission", mid-document: the DTE writes `+FPS=2` before the
`+FDR` that releases the response, so RTN goes on the wire, the far end
retrains and sends the same page again, and that repeat has to reach the DTE
as a page of its own before the document carries on. The far end has to be
told it may retransmit (`t30_set_retransmit_capable()`); without that SpanDSP
moves on to the next page instead and there is nothing to test.

**The ECM arm of it asserts the opposite, and that is the point.** T.30
Table 5, note 2 to RTN: *"RTN is not applicable to the optional T.4 error
correction mode."* ECM repairs a page frame by frame -- the receiver answers a
partial page with MCF or PPR and there is no signal for "bad page, retrain" --
so a DTE's `+FPS=2` cannot be honoured, MCF goes out and page two follows. On
the wire that is indistinguishable from the held response being ignored, which
*is* a real bug in the non-ECM case, so the two are pinned separately: with
the DTE's verdict ignored, the non-ECM arm fails six checks and the ECM arm
fails none.

`test_transmit_page_rejected` is the other half, Annex II.7. The far end
refuses page one with RTN -- through the same held-response API the DCE uses
for its own DTE -- and the checks are II.7's own steps: RTN on the wire,
`AT+FPS?` reading back 2 (Table 23, and nothing else covers that mapping in
the transmit direction), a `+FCS:` from the renegotiation that II.7 calls "RTN
forces back to Phase B", and the far end ending up with the rejected page, the
repeat, and page two after it. A DCE that skipped the bad page would leave two
pages, the wrong one missing, and still report a good session. The `+FPS?`
read has to wait for the frame: the far end's handler sees RTN as it goes out,
and this side still has to demodulate it -- read too early it says 1, which is
what a fixed pump did.

Both run twice: the rejection **mid-document**, at MPS, and at **EOP**, the
last page of the document. The two are not the same case, and everything
below was needed to make the second work at all -- before it, a page refused
at the end of a document ended the call with `+FHS:20` and the DTE got no
second chance.

**Three fixes, each of which fails the tests on its own.**

*The transmitter gave up.* T.30 Figure 5-2c has a transmitter whose page is
refused ask "CAPABLE RE-XMIT?", and go back to phase B and repeat the page if
it is, or send DCN if it is not. SpanDSP defaults to not, which is
conformant -- but a class 2.0 DCE is capable: the page it sent is still in the
document the DTE handed over, which is exactly what II.7 has the DTE hand over
a second time. `t30_set_retransmit_capable()` is now set. Note it changes
nothing mid-document, where both settings already retransmit; it is the EOP
branch of the figure that differs.

*The receiver would not take the page back.* Figure 5-2d sends a receiver
which responded RTN or RTP to **B**, "go to beginning of phase B", with no
distinction between EOP and MPS -- but `process_state_iii_q()` had no case for
DCS at all, so the transmitter's fresh training landed in the default arm and
the call was torn down as an unexpected frame. That is the "unexpected
message" a rejected page used to produce, and it is a receiver-side bug that
this DCE would have hit against any far end that retransmits.

*And the document was closed before the verdict.* `assess_copy_quality()`
released the T.4 receiver as soon as the post page message was EOP -- before
the copy quality was decided, and long before the DTE releases a held
response. The repeat then reopened the output file, truncating the pages
already received and restarting their numbering, so the page reached the DCE
and could not be handed to the DTE (`CONNECT` followed by `ERROR`). The
teardown is now deferred to when the response is settled, and skipped
altogether when that response is RTN or RTP, since the document is not over.
A visible consequence: a far end now files the copy it refused at the end of a
document, as it already did mid-document.

**The page that goes again is the DTE's, handed over with a second `+FDT`.**
That is II.7's own model, and it needed T.30 to stop after the RTN rather than
repeat a page it already had. `t30_set_retransmit_hold()` does that: the
"CAPABLE RE-XMIT?" branch of Figure 5-2c sets a pending flag and sends
nothing, and `t30_resume_retransmission()` starts the document that is set
*now*, from its first page. On this side the refusal marks the session, the
next `+FDT` opens a fresh document file -- a new one each time, since T.30
reopens by name and is still holding the old one -- and `spool_close()`
releases the retransmission when the DTE finishes it.

Mid-document the DTE supplies the rest of the document from the refused page
onwards, not just the one page, because the procedure goes back to phase B and
the document it is given starts at page one. That falls out of T.30 rather
than being a choice here: phase B is the beginning of a document.

**The EOP test hands over a different image the second time**, and that is
what makes it a test of provenance rather than of retransmission: a DCE that
repeated its own copy would put the first page on the line again and pass
every other check. Removing the hold fails exactly that one assertion and
nothing else. T.32 does not require the two pages to match -- the DTE simply
sends a page.

`test_rejected_page_never_resent` is the other side of holding a procedure for
a DTE: if the page never comes, 8.5.2.6's `+FCT` ends the call with a DCN and
`+FHS:02`, exactly as it does for a `+FDT` that is opened and not fed.
Without the timer being armed on the refusal, the far end sits in phase B
after its RTN until its own T.30 timers give up and the DTE is told nothing;
that is two failing checks.

**What it still does not match**: II.7 has the `+FDT` itself complete with
ERROR once the far end's verdict arrives. Here it has already completed, for
the reason in *Deviations* -- the `+FDT` completes when the page is spooled,
which on this DCE can be before the call is even answered. The DTE learns the
verdict from `+FPS` instead.

`test_unfinished_document_discarded` hands over a page that says another
follows, drops the call, and makes a second one — deliberately without leaving
class 2.0 in between, since that clears everything and would hide the leak.
The second call must carry its own page and only its own page.

`test_transmit_timeout_not_tripped` feeds a page in four blocks over 2.4 s
against a `+FCT` of one second, with no gap longer than the timeout, and
requires the transfer to still be open. Without the per-block restart it is
aborted; that half of the mechanism had no test until this one, and the
restart was measured to be uncovered before it was written.

`test_procedure_interrupt` checks both directions **on the wire**, from the far
end's own frame handler, because both are a substitution inside a frame that
gets sent either way — a session that ignored the request entirely still
completes and still reports `OK`. It covers `<DLE><pri>` turning the post page
message into a PRI-Q, the far end granting with PIP, the `+FVO` that follows,
`+FIE=0` suppressing it, the `+FET:` substitution in both settings, and a
receive-side `+FPS=5` turning the post page response into a PIP.

`test_fns` checks the parameter's own rules and then that the octets **reach
the far end** — captured by a real-time frame handler on the peer as the frame
arrives, because T.30 frees its received-frame store in `release_resources()`
immediately after the phase E handler, so `t30_get_rx_nss()` after a session is
always empty. That is a fact about SpanDSP's lifetimes, not about the frame; a
test that read it there would have reported a working `+FNS` as broken.

`test_fbu` checks the reports against what T.30 must have exchanged — a
received DIS as `FF 13 80 ...`, the DCS we send, the report landing before the
`+FIS:` it produced, and `+FBO=2` reversing all of it to `FF C8 01 ...`. The
ECM exclusion is checked on a session that **actually runs ECM**, since the
absence of an FCD report in a session that has no ECM would prove nothing;
that check was confirmed to bite by removing the exclusion and watching it
fail.

`test_poll_remote` and `test_be_polled` run polled sessions both ways — this
DCE polling a far end that has a document, and a far end polling a document
this DCE offered — and compare the page in each. A polled transfer that reports
`+FPO` and then delivers nothing would pass a result-code check.

`fax_class_test` covers the other half — that class 2.0 is *reachable* through
the real PTY the DTE sees (`AT+FCLASS=2.0`, a `+FCC?` only this module answers,
an `ATE0` that must still reach T.31, and the way back to class 0), which
`fax_class2_test` bypasses by calling the module directly.

## Not done

- Class 2 (the pre-standard `AT+FCLASS=2`) — only 2.0 is offered. The two are
  not compatible, and 2.0 is the ITU-T one.
- Multi-*document* transmit: `<DLE>;` keeps one document open rather than
  ending it and renegotiating, as above.
- `+FDD`/`+FIT`/`+FLO`/`+FPR` — these are T.31 DTE-link parameters and stay
  with the T.31 interpreter.
- T.38 — `t31_init()` is given no T.38 packet handler and the context is put in
  audio mode.
- No hardware interop: as with everything else here, the automated tests cover
  offline and loopback paths only. A real fax session against a far-end fax
  machine is unverified — and for class 2.0 that includes the `+FDT` ordering
  deviation above, which no loopback against our own sequencing can expose.
