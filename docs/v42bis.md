# V.42bis stream compression

Enable the modem data path with `ME_DATA_FRAMING=lapm` and
`ME_DATA_COMPRESSION=v42bis` (the default compression selection).
The default offer is P0=3, P1=1024, P2=32. `ds_init_v42_ex()` permits
explicit offers and P0=0 for plain LAPM. The bare V.42 library defaults to
no compression; applications must attach codecs before opting in.

The implementation follows ITU-T V.42bis (01/1990):

- 5.1 / Annex A: negotiate supported directions and smaller limits. All
  two-octet P1 sizes from 512 through 65535 are supported; P2 is 6 through
  250. Dictionary storage is allocated for the agreed P1 and codeword widths
  extend from 9 through 16 bits, including nonpowers of two.
- 6.2–6.5 / 7.3–7.5 / 8: maintain the shared tree dictionary in transparent
  and compressed modes, recover leaf entries, exclude the last added entry
  from string matching, bound string lengths, and emit/consume STEPUP.
- 7.8 / 9: automatically choose transparent or compressed mode using a
  compressibility monitor; ECM, ETM, escape cycling and EID maintain state
  across arbitrary transfer boundaries. Applications may also force a mode.
- 7.8.3: `v42bis_compress_reset()` sends outstanding data, ETM when needed,
  then RESET using the old escape value before initializing the encoder.
  Reset policy is implementation-defined; no periodic reset is required.
- 7.9: compressed flush sends the partial match, postpones its dictionary
  update until the next character, and emits FLUSH only for residual bits.
  Transparent flush preserves the accumulated match length, including across
  repeated one-octet flushes. Idle flushes emit no additional wire bytes.
- 5.6: `v42bis_restart()` implements C-INIT without sending RESET, discarding
  pending data and initializing both directions while retaining parameters,
  callbacks and compression policy. LAPM establishment invokes C-INIT even
  for SABME without a fresh XID. A physical retrain that retains LAPM retains
  dictionary state.
- 5.8: invalid commands, missing dictionary entries, C1 references, oversized
  strings and STEPUP beyond the negotiated width latch C-ERROR. Further
  decoding/flush fails until C-INIT; the modem reports link error and stops.

Compression runs once before LAPM buffering; retries send the same encoded
information frame. Only accepted information bytes enter the decoder.
`v42bis_release()` frees dictionaries for caller-owned contexts;
`v42bis_free()` also frees the context. Release is idempotent. Release an
initialized caller-owned context before passing it to `v42bis_init()` again.

## Validation

`make v42bis_test data_stack_test v42_link_test` builds the native tests.
`v42bis_test` exercises dictionary limits, RESET, repeated flushes, one-octet
receive fragments and 2000 deterministic malformed streams. The integration
suite covers both directions, directional refusal, retries, maximum P1, fresh
sessions and a peer SABME followed by a fresh stream without another XID.

Run independent bidirectional checks with:

```sh
python3 tools/v42bis_interop.py --reference ../modem-dsp-emu
```

The reference is needed only for testing. Production has no Python dependency.
The 79 checks cover 16-bit codewords, dictionary reuse, all supported string
limits, automatic and forced mode transitions, RESET, C-INIT recovery and
aligned/fragmented/idle flushes. Native UBSan and Linux ASan/UBSan also pass.

Hardware application interoperability is a separate measurement. The earlier
RasFinder capture negotiated P0=3/P1=1024/P2=32 but delivered opaque bytes with
no initial ECM; interpreting them as a fresh compressed stream referenced an
absent dictionary entry. These codec changes do not establish that capture's
missing state or prove a readable hardware application exchange. See
`t30_annex_f_v34_fax.md` for the preserved captures and investigation.

The subsequent isolated hardware call `rf-v42bis-complete-20260930` again
negotiated P0=3/P1=1024/P2=32 and connected LAPM at 19200 bit/s. It delivered
21 opaque application bytes, and physical retraining resumed the existing
data link. A readable banner/application roundtrip remains unverified.
Artifacts are preserved in `/tmp/v44-port/rf-v42bis-complete-20260930`.
