# V.92 37333-bit/s trellis regression

Consumer: `v92_b1u_lock_test.c::test_live_soft_fixture()`.

A compact receiver-only slice of the wired slmodemd call investigated on
5 November 2026; see `docs/v92_upstream_trellis.md`. This is a new regression
fixture, not a retained evidence directory.

- `37333-soft-q20.s32le`: 5000 twelve-symbol frames, 60000 samples. Each is
  a signed little-endian int32 divided by 2^20, in the actual decoder's
  normalised G*u units. Values were rounded from the live double input.
- `37333-source-lsb.bin`: the corresponding 280000 expected source bits,
  packed least-significant bit first in each byte (35000 bytes). These are
  pre-LAPM bits after GPA descrambling, not application payload bytes.

The slice starts at native frame 18 of B1u: source offset 1008 bits, native
symbol offset 216. `dense_cpd()` in the test states the captured 36-point
CPd. The known first 18 B1u frames, encoded with its precoder, initialise
GPA, differential and trellis state; the test does not supply payload bits
to the receiver. The live equaliser has already removed the channel and
precoder, so the receive CPd has coefficients disabled.

The short frame-local receiver made 13637 bit errors over this 5000-frame
interval before quantisation. The streaming receiver must decode every
emitted bit correctly, with only its fixed four terminal frames left
pending. Input chunks 1/37/160/320, delayed refit-window association,
replay/live boundaries, and per-call reset are checked against this slice.
No artificial samples are appended to flush the pending traceback.

Provenance: the disposable `probe3-37333` input dump and native transmitter
bit hook in `artifacts/v92-upstream-investigation/`. Production fixture
checks do not read that expiring directory. The native input was reduced
to bit 0 and GPA-descrambled (V.92 6.3); source alignment was confirmed
against high-entropy receiver anchors. See the topic note for controls,
binary identities and the larger unquantised comparison.
