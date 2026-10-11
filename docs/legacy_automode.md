# Legacy automode: V.21, Bell 103, Bell 212A and V.23 peers

With `+MS` automode on (the default, V.250 6.4.1), a call that started with
V.8 also listens for the V.8-less modems that V.25 call establishment still
reaches. It moves to the matching datapump when one of them identifies itself.
`AT+MS=<carrier>,0` (automode off) or `ME_LEGACY_AUTOMODE=0` withdraws this,
and `+MS` bounds that exclude a peer's rates leave it out.

Code: `legacy_automode_rx_locked()` in `modem_engine.c`, next to the V.32bis
Annex A watch.

## What is listened for

V.25 3.10 iii) has a duplex call modem put its starting signal (binary 1) on
the line once the answer tone has ended, and an answer modem sends its own
after the tone. A Bell 103 originate modem keys on energy in its
2025-2225 Hz receive band, which includes 2100 Hz. So each peer gives itself
away with a steady mark:

| Heard by | Signal | Peer | Then |
|---|---|---|---|
| answer modem | 980 Hz | V.21 call modem (channel 1 mark) | V.21, channel 2 |
| answer modem | 1270 Hz | Bell 103 originate | Bell 103 answer |
| answer modem | 390 Hz | V.23 terminal (backward channel) | V.23, we send 1200 |
| answer modem | 1300 Hz for 1 s | V.23 modem sending 1200 | V.23, we send 75 |
| call modem | 1650 Hz | V.21 answer modem (channel 2 mark) | V.21, channel 1 |
| call modem | 2225 Hz | Bell 103 or 212A answer modem | Bell 212A; after 2.5 s with the tone still on, Bell 103 |
| call modem | 1300 Hz | V.23 host sending 1200 | V.23, we send 75 |

**What counts as a signal.** "Steady" means one 50 Hz bin holds at least 85%
of the received power, in every 20 ms block, for 400 ms. That is the V.21 and
V.23 circuit-109 time.
- V.8's CI, CM and JM sit on the same V.21 channels but are FSK data, so any
  block that holds a zero bit fails the test.
- Once the answer modem has received a CM, it stops looking.
- 1300 Hz at the answer modem needs a full second: V.25 2.1's calling tone is
  also 1300 Hz, but comes in 0.5-0.7 s bursts.

**When the answer modem hears it.**
- A Bell 103 caller is usually heard during our V.8 answer tone.
- A V.21 or V.23 caller waits for that tone to end. After a V.8 failure with
  no CM, the answer modem moves to V.22bis USB1 (V.32bis A.2.2), and the
  caller's mark is caught there. The test sees this at about 6.8 s.

**Bell 212A versus Bell 103.** Both answer with 2225 Hz, and only the answer
modem knows which it is. The call modem tries 212A first: it sends scrambled
ones, and a 212A answer modem drops its tone after 270 ms
(`docs/bell212a.md`). A Bell 103 answer modem keeps it, because it is its
mark. After 2.5 s the call modem sends the 1270 Hz Bell 103 originate mark
instead, which a 103/212A answer modem also accepts.

## Changes to the fixed modes that came with this

- **Mark-only carrier qualification.** Circuit 109 for V.21, Bell 103 and
  V.23 now qualifies on the receive channel's mark alone. V.21 8.2.1.1 and
  V.23 8.3.1.1 define the response time with "a frequency corresponding to
  the characteristic frequency of binary 1". Qualifying on mark and space let
  a 2100 Hz answer tone pass as carrier: 2100 Hz is V.23's forward space, so
  our V.23 call modem "connected" to V.8's ANSam.
- **Calling V.21/V.23 wait for the answer tone.** A fixed V.21 or V.23 call
  modem now holds its transmitter silent while an answer tone is on the line
  (V.25 3.10). It starts once 2100 Hz has been heard for 200 ms and then been
  absent for 80 ms, or once the far end's mark qualifies with no answer tone
  (a fixed answer modem sends none). Bell 103 does not hold.
- **The Bell 212A caller also takes V.22 USB1.** Its tone detector now accepts
  USB1 (2250 Hz) as well as 2225 Hz. A V.22 answer modem behaves exactly like
  a 212A one from the call modem's side, so a fixed B212 caller now reaches
  our automode answerer. That takes about 13 s, because the V.8 answer tone
  runs to its timeout first.

## Tests (`tests/fast.list`)

Each case is two whole engines over G.711.

- **Answer side:** our default offer against fixed V.21, Bell 103, V.23 and
  Bell 212A callers.
- **Call side:** our default offer against fixed V.21, Bell 212A, Bell 103
  (the fallback) and V.23 answerers.
- **Automode off:** a V.34-only answer modem must not fall back.

Each case is graded on the modulation, the CONNECT rate, the payload both
ways and the engine log line that names the signal.

## Not done

- **Real modems:** none of these has met a real V.21, 103, 212A or V.23 modem.
- **V.32bis training phase:** a legacy caller that only starts after our
  USB1 phase is over (Ta, 3 s, when V.32bis is offered) is not listened for
  during V.32bis training.
- **Answer-side Bell 212A:** we send no 2225 Hz in automode. A 212A caller is
  answered through V.22's USB1, which it accepts as its answer tone.
