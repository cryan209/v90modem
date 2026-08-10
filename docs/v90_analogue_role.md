# Taking the analogue side of a V.90 call

This software has always been the **digital** modem: it injects PCM codewords
downstream and consumes a constellation the peer chose. The analogue role is
the other half — the side with a D/A → loop → A/D hop, which is therefore the
side that measures the line and decides the constellation (§5.4.3).

Wanting it is not academic. Every receive test in this tree is fed by our own
transmitter, so a wrong-but-self-consistent assumption is invisible; and the
analogue side of the loopback harness is the grader for the entire digital-side
suite, so if a convention is wrong in shared `vpcm_*` code, both ends agree and
the peer still fails. Running our analogue side against a real digital modem
breaks that circularity.

## Status: V.8 only

`ME_V90_ROLE=analogue` makes V.8 offer the analogue side correctly. Nothing
past V.8 exists, and the engine says so rather than pretending:

```text
[ME] V.90 role: ANALOGUE (opt-in; Phase 3 analogue TX is not implemented — V.8 only)
[ME] V.8 negotiated V.90 with this end as the ANALOGUE modem, but Phase 3 has
     no analogue transmitter (Sr/TRN1r/Jr, Ja+DIL descriptor) — falling back to V.34
```

The fallback is deliberate. Everything after the V.8 result handler —
`start_v34_training()` and the Phase 3 state machine — drives the *digital*
transmitter. Continuing into V.90 there would put a digital Sd/TRN1d/Jd on the
wire from a modem that had just told the peer it was analogue, while the peer
waited for Sr/TRN1r/Jr and a Ja carrying our DIL descriptor. A working V.34
call beats a V.90 one built on a contradiction.

Default is unchanged: without the environment variable the engine is the
digital modem exactly as before.

## What V.8 had to change (§Table 5)

| field | digital role | analogue role |
|---|---|---|
| `pstn_access` | `V8_PSTN_ACCESS_DCE_ON_DIGITAL` | `0` |
| `pcm_modem_availability` | `V8_PSTN_PCM_MODEM_V90_V92_DIGITAL` | `V8_PSTN_PCM_MODEM_V90_V92_ANALOGUE` |
| V.92 QC/QCA octet | `0x45` / `0x47` | omitted |

`pstn_access` matters as much as the availability bit: it says the DCE sits on
the digital network. Leaving it set while advertising analogue availability
tells a peer two contradictory things about where we are. The V.92 octet
encodes the *digital* modem's capabilities and has no analogue equivalent wired
up, so an analogue-role offer omits it rather than guessing.

## What is already built

The measurement half of the analogue role is done and tested, because it turned
out to be separable from the signalling:

- **Impairment measurement** — [`../v90_dil_measure.c`](../v90_dil_measure.c).
  Given the DIL descriptor we sent, measure what came back: per-Ucode,
  per-data-frame-interval level, spread, and the RBS slot mask.
- **Constellation and rate** — Ci per interval under a measured noise margin
  and §8.5.2's Table 15 power cap, then `drn` and bit rate.
  [`v90_constellation_selection.md`](v90_constellation_selection.md).
- **Descriptors to send** — [`../v90_dil_presets.c`](../v90_dil_presets.c),
  with `v90_dil_desc_validate()` to catch a descriptor that would measure
  nothing in an interval.

So the analogue side already knows what to ask for and what to do with the
answer. What it cannot do is conduct the conversation.

## What is left

Roughly in dependency order:

1. **Phase 2 (§9.2)** — INFO0a/INFO1a instead of INFO0d/INFO1d. INFO1a is where
   U<sub>INFO</sub>, the DIL descriptor length and the symbol rates go, so this
   is also where a chosen preset gets transmitted.
2. **Phase 3 transmit (§9.3.2)** — Sr, S̄r, TRN1r, Jr, and Ja carrying the DIL
   descriptor. The digital-side equivalents in `v90.c` are the model; none of
   the analogue ones exist.
3. **Phase 3 receive** — largely done. The offline decoders read a real digital
   modem's Sd, S̄d, TRN1d and Jd (`make eicon-rx-test`); what is missing is
   driving them from the live engine rather than from a file.
4. **Phase 4 (§9.4)** — build a CP from the measurement, which is the one part
   already written, and run B1/E.
5. **Data mode** — receive the mapped downstream instead of transmitting it.

Step 3's near-completion is the reason to expect this to work: the hardest part
of the analogue role, reading a foreign downstream, is the part with test
coverage against a modem that actually connects.

## Testing it without a second modem

`tools/eicon_adsp_sip.py` runs the Eicon card's own shipped V.90 firmware as a
SIP endpoint, answer-only. `sip_modem.c` can originate (ATD via
`pjsua_call_make_call`), so an analogue-role build can dial the card directly —
no Asterisk in the path, no hardware. That gives a real digital peer whose
internal state is readable per-sample (`TrnProgress`, `Rstatus`, eye words) via
its `adsp.csv`, which is a far better failure signal than the Courier's silence.
