# Bell 212A

`AT+MS=B212` (alias `B212A`) or `--mode b212` selects Bell 212A: V.22's
1200 bit/s DPSK (carriers 1200/2400 Hz, the same scrambler), answered with
the Bell 2225 Hz tone instead of V.25's 2100 Hz, and with no guard tone. There
is no V.8, so both ends must be set to it out of band. The connect report and
ATI6 say `B212`, and the engine's modulation is `V22BIS`.

## Handshake

| | Answerer | Caller |
|---|---|---|
| 1 | 75 ms silence, then 2225 Hz at the data level (-14 dBm0) | silence; engine tone detector listens for 2225 Hz |
| 2 | | tone held 150 ms: start the datapump, sending scrambled ones (low channel) |
| 3 | 270 ms of the caller's scrambled ones: drop the tone, send scrambled ones (V.22 timing) | |
| 4 | | the tone detector sees the tone end: the datapump trains on the answerer's scrambled ones from the nominal 2400 Hz carrier |
| 5 | CONNECT 1200 | CONNECT 1200, once the tone has been gone for 200 ms |

Most of this is SpanDSP's V.22bis datapump with
`V22BIS_BELL212A_COMPATIBILITY_MODE`. That option was declared upstream but
did nothing; it is now implemented in `v22bis_tx.c` / `v22bis_rx.c`:

- **Answerer:** in the U11 stage it sends the tone instead of unscrambled ones.
  Its receiver ends that stage on the caller's scrambled ones, exactly as it
  ends U11 in V.22.
- **Caller:** it starts in S11 and skips V.22's unscrambled-ones detection. It
  waits for `v22bis_bell212a_answer_tone_ended()`, which the engine calls after
  two clear 20 ms blocks. Carrier tracking is off while it waits.

## Why the caller does not demodulate the tone

Seen through a V.22 receiver, 2225 Hz is a -105 degree step every symbol.
The nearest step is binary 11's 270 degrees, so a plain V.22 caller *can* take
it for unscrambled ones. Whether it does depends on its carrier loop pulling
25 Hz onto the tone. `bell212a_test`, with a bearer delay added, showed what
that costs:

- **Carrier drag.** The caller's carrier finished at 2378.6 Hz instead of 2400,
  and answer-to-call data was garbage (about 45% errors) at 200 and 400 ms each
  way. Over local SIP, whose fixed jitter buffer adds 200 ms each way, the GUI
  echo failed every time.
- **Early exit.** A dibit statistic on the tone (6 changes in 16 symbols) still
  fired early while symbol timing settled.

The fix measures the tone's power directly (the engine's Goertzel qualifier,
shared with V.21) and tells the datapump when the tone ends. After that fix:

- Both ends are error-free at 0, 100, 200 and 400 ms each way.
- The GUI echo passed three runs out of three.

## Bell 103 on the same line

- **Bell 103 caller vs our B212 answerer.** The answerer also listens to its
  low band for Bell 103's 1070/1270 Hz, using the V.21 circuit-109
  qualification (`docs/v21.md`). If that qualifies, the answerer falls back to
  Bell 103, as a 103/212A answer modem does, and reports `B103`. Until those
  tones are gone it will not take a V.22 training on them. A constant tone
  descrambles to all ones, which looks exactly like scrambled ones.
- **Our B212 caller vs a Bell 103 answerer.** The caller hears 2225 Hz, which
  is a 103 answerer's mark, and sends scrambled ones. The tone never ends, so
  the caller never connects, and S7 ends the call. Falling back to 103 on the
  calling side is not implemented.

## Tests

- `bell212a_test`, the datapump alone:
  - The tone is pure 2225 Hz at -14 dBm0. Plain V.22's USB1 has 1.5% of its
    power there.
  - A 212A pair is error-free both ways at four bearer delays. A plain V.22
    pair runs as the baseline.
  - A 212A answerer works against plain V.22 and V.22bis 2400 callers with no
    delay.
- `engine_pair_test`:
  - B212 pairs in both laws.
  - B212 caller vs B103 answerer never connects.
  - B103 caller vs B212 answerer falls back to Bell 103.
- `tools/gui_loopback_test.py b212`: byte-exact echo over local SIP.

## Not done

- **Billing delay.** V.25 and Bell both wait about 2 s before the answer tone.
  This answerer waits 75 ms, as SpanDSP's V.22 does.
- **Delayed bearer, mixed callers.** A plain V.22/V.22bis caller (not 212A
  mode) against our 212A answerer still relies on the tone looking like USB1,
  so it is only verified with no delay.
- **Real modems.** None of this has met a real 212A modem.
