# Fax service class over the AT interface (T.31)

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

## Not done

- Class 2/2.0 (`AT+FCLASS=2`, `=2.0`) — the `+F` service-class-2 commands are
  parsed but their handlers in SpanDSP are `TODO` stubs, and `AT+FCLASS=2.0`
  correctly answers `ERROR` against the advertised `0,1,1.0`.
- T.38 — `t31_init()` is given no T.38 packet handler and the context is put in
  audio mode.
- No hardware interop: as with everything else here, the automated tests cover
  offline and loopback paths only. A real fax session against a far-end fax
  machine is unverified.
