# V.70 DSVD (digital simultaneous voice and data)

ITU-T V.70 is a profile. It puts a **V.76** multiplexer (HDLC, V.42 LAPM-based) under a
**V.75** control entity, adds a speech coder (**G.729 Annex A**) and a data channel, and
hands the multiplexed bit stream to a V.34 or V.32bis datapump. This tree has all of it
except V.8bis entry and an engine hook.

| Layer | Files | Spec |
|---|---|---|
| Terminal (SCF, voice cadence, DTE, tunnelling) | `v70.c/.h` | V.70 (08/96) + Cor.1 |
| Control entity (channels, capabilities, break, SAR, audio header) | `v75.c/.h` | V.75 (08/96) + Cor.1 |
| H.245 messages <-> aligned PER | `v75_h245.c`, `per.c/.h`, `h245_schema.c/.h` (generated) | H.245 (03/2022), X.691 |
| Multiplexer | `v76.c/.h` | V.76 (08/96) + Cor.1 |
| G.729A adapter (optional) | `v70_g729a.c/.h` | G.729 Annex A |

## Verified

`make test` runs `v76_test` (1397 checks), `v75_test` (318), `v70_test` (468) and
`v70_v34_test`. The last runs two complete terminals over **two real V.34 datapumps and a
G.711 round trip** (3200 baud / 21600 bit/s, u-law, A-law and with suspend/resume): DSVD
is active 0.1 s after data mode, and 20 s later each side has moved ~2000 voice frames and
20000 DTE octets exactly, in order, with no acknowledgement timeouts.

- **FCS** is checked against an independent implementation of the 5.1.6 polynomial division
  (and the Cor.1 text), not the reflected-register shortcut the module uses.
- **H.245 PER** is checked against `asn1tools`, an independent X.691 implementation:
  `make h245-per-oracle` fuzzes 27000+ values over 11 types (spec-driven random walks of the
  DSVD subset), byte for byte and value for value, in both directions. 13 golden vectors from
  the same oracle are committed (`v75_h245_golden.h`) so `make test` needs no Python.
- **G.729A**: `make v70-g729a-test G729A_SRC=...` -- encode is bit-exact against the ITU's
  `*.BIT`, decode against `*.PST`, with the speech carried through the whole DSVD stack
  (ALGTHM, SPEECH 37.5 s, PITCH, LSP).
- **Suspend/resume** (V.76 Annex A) takes worst-case voice latency from 288 ms to 4.8 ms
  when data frames are 1000 octets.
- **Silence frees the line** (V.70 5.4): in 6 s at 28.8 kbit/s the DTE moves 11776 octets
  while the voice channel talks, 16896 with SID-only silence, 17408 with none.

## Defects the tests found, worth remembering

1. **A refill that queued no bits wedged the transmitter.** After a max-length real-time frame
   the data frame resumes with *no* resume flag (A.3 b-ii), so `tx_refill` pushed nothing and
   `bq_n` went negative. The fix is to loop until bits exist.
2. **A flag-lookalike check on pushed bits is wrong.** A data octet with a stuffed zero
   (`0 11111 [0] 11`) reads as `0 1^7` once the stuffed zero is dropped. Only the *raw* run of
   ones distinguishes data from a suspend/resume flag.
3. **FCS length ambiguity (found on real speech, 1 frame in 65536).** 5.1.6 says try every
   supported FCS length, but with 8- and 16-bit both supported about one 8-bit-FCS frame in
   65536 also validates as a 16-bit-FCS frame one octet shorter, and was delivered truncated.
   7.1.2.1 fixes a DLC's FCS length at its SABME, so an established DLC now uses its own length
   only. `v76_test` builds such a colliding frame and fails without the fix.
4. **PER range > 64K**: the 2-bit length is *not* octet-aligned; only the value octets that
   follow are. (Aligning before the length produced 311 of 1300 oracle mismatches.)
5. `asn1tools` emits an **empty open type for a NULL extension**; X.691 10.1.3 replaces an
   empty complete encoding with one `00` octet, which is what `per.c` does. The fuzzer excludes
   that one construct rather than trusting either side blindly.

## Where H.245 (2022) disagrees with V.75 (1996), and what was chosen

H.245 wins. `V75Parameters.audioHeaderPresent` is a BOOLEAN there (V.75 Annex A says NULL).
`V76ModeParameters` is a two-way CHOICE (suspend/resume with/without address) carried in a
`RequestMode` ModeElement, not a block of mux parameters. `OpenLogicalChannelAck` carries no
V.76 parameters in its reverse direction. `CloseLogicalChannel.reason` is a non-optional
extension addition; a V.75-era sender cannot include it, so the codec treats it as optional.
Two defects in the published ASN.1 text (an `OPTIONAL` on a CHOICE alternative, an undefined
`DataCapability`) are patched by name in `tools/h245/extract_asn1.py`.

## The H.245 subset and why pruning is safe

`tools/h245/prune.py` lists what a DSVD terminal speaks. A dropped CHOICE alternative is still
*counted* (its index and the width of the index field are unchanged); it simply has no schema,
so the encoder refuses it and the decoder reports it as unsupported. Extension additions that
are unknown or pruned are skipped on decode (they are length-prefixed). Not mapped: the `t84`
and `nlpid` data applications (X.263 over a network layer) and everything non-DSVD.

Regenerating: `make h245-schema` (needs `pdftotext`; `H245_PYTHON` with `asn1tools`).

## G.729 licensing

The ITU G.729 package in `ITU Docs/` is "Copyright (c) AT&T, France Telecom, NTT, Universite
de Sherbrooke. All rights reserved" with no open-source terms, so its source is **not**
copied into the tree. `v70_g729a.c` is an adapter that compiles against a source tree you
extract (`G729A_SRC`). The reference code keeps its state in globals (one encoder and one
decoder per process) and does not survive a second `Init_*` in the same process.

## Not done

- **V.8bis**: V.70 6.1 enters DSVD through it. The caller says when the modem has trained
  (`v70_start()`) and which role it holds. `k56flex_v8bis.c` is a different V.8bis profile.
- **Engine integration**: nothing in `modem_engine.c` links these files (they are not in
  `SRCS`). The bit interface (`v70_tx_get_bit` / `v70_rx_put_bit`) is the same shape as
  `ds_tx_get_bit` / `ds_rx_put_bit`, so `data_stack` is the natural host, but a voice path
  (and what the SIP side does with it) has to be decided first.
- **m-SREJ** (V.76 Figure 7's span-list encoding) and a G.729 Annex B VAD/CNG.
- **Not exercised against a foreign DSVD terminal.** Everything above is against independent
  implementations of each *layer's* rules, never against another modem.
