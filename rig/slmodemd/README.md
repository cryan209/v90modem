# slmodemd rig patches

Patches applied to the SmartLink slmodemd on the private interop rig (container
`d-modem`, sources under `/src/slmodemd/`). Not upstreamed. See `rig/README.md`
for the d-modem side.

## `v92-pcm-upstream.patch`

Makes the DSP blob accept a peer's V.92 PCM-upstream offer instead of refusing it.

### Why it is needed

The blob implements PCM upstream — `V34SetINFO1aBits` contains both
`V.34 Upstream is selected (info1)...` and `PCM Upstream is selected (info1)...`.
But `V34GiveINFO1dBits` gates it:

```
isPCM = local_cap && remote_cap && (peer INFO1d Table 17 bit 70)
if (!isPCM) return 0
p = *(v34obj + 0xac3c)                  /* = m->dp_runtime */
if ((signed char)p[2] < 0) return 0     /* accept PCM upstream */
log("we got PCM upstream under V.92Lite ..."); InitiateRetrain(90); return 1
```

`dp_runtime` byte 2 is a flags bitfield:

| bit | meaning | written by |
|-----|---------|------------|
| 4 | V.92 mode | `vpcm_create` (set when requested DP is 92) |
| 6 | `DSPINFO[8] & 1` | `dp_runtime_create` |
| 7 | PCM upstream permitted | `dp_runtime_create` — **cleared unconditionally** |

`dp_runtime_create` clears bit 7 on both branches of its `DSPINFO[0xc]` test, and
nothing anywhere in the linked binary ever sets it. So the PCM-upstream path is
complete but unreachable. This patch sets the bit after `dp_runtime_create`.

### Use

Off unless `SLM_V92_PCM_UPSTREAM` is set in slmodemd's environment. It enables code
that has plausibly never executed in this build, so keep it opt-in and always A/B it
against an unpatched run.

```sh
docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz \
  -e SLM_V92_PCM_UPSTREAM=1 -e DM_RESAMPLER=sinc \
  d-modem sh -c "/src/slmodemd/slmodemd -d9 -e /src/d-modem > /tmp/slm.log 2>&1"
```

Confirm with `grep "PCM upstream force-enabled" /tmp/slm.log` (expect
`dp_runtime flags[2] = 0x90` — bit 7 plus bit 4 for V.92).

The digital side must also advertise PCM upstream, or `isPCM` is never true and the
gate is not reached: run `sip_v90_modem` with `ME_V92_PCM_UPSTREAM=1`, and dial with
`AT+MS=92,0,300,56000`.

### Apply / roll back

```sh
docker exec d-modem sh -c "cd /src/slmodemd && patch -p2 < /tmp/v92-pcm-upstream.patch && make"
# rollback:
docker exec d-modem sh -c "cd /src/slmodemd && cp modem.c.bak-prev92up modem.c && make"
# or just restore the binary: cp slmodemd.bak-prev92up slmodemd
```

## TRN2d reference capture (`tools/smartlink_trn2d_reference_wrap.c`)

Settles which of sign / level / timing the peer's Phase-4 Error Energy plateau
(~300-380 vs ~110-146 during DIL) is made of, in the peer's own units.

SmartLink trains Phase 4 data-aided: `V90Phase4Demodulator::trn2dKnownDemod()`
calls its own digital-side Phase 4 generator and returns the reference sample
its equalizer expects for the current TRN2d symbol — a symbol-by-symbol
prediction of *our* transmitter. Any legal-but-unpredicted choice we make
(prime suspect: the §5.4.5.6 shaper-metric input convention, see
`ME_V90_SHAPER_METRIC` below) becomes a permanent reference-error floor the
equalizer cannot adapt away. The wrap hooks that method and streams
interleaved int16 pairs `[received, reference]` to
`/tmp/smartlink-trn2d-pairs.s16` for the whole call (every Phase-4 attempt,
concatenated; `TRN2D_REF_SYMBOLS` caps the count, default 200000).
`(received - reference)^2` reproduces Error Energy in the peer's units;
the received stream symbol-aligns the capture against our `live-tx.g711`.

### Relink

```sh
# copy the wrap source in
docker cp tools/smartlink_trn2d_reference_wrap.c d-modem:/src/slmodemd/

# inside the container: alias the original at its .text offset, compile the
# hook (slmodemd is 32-bit i386), and relink with the hook object FIRST plus
# --allow-multiple-definition so the hook's definition wins.
# Verify the offset first if dsplibs.o ever changes:
#   objdump -t dsplibs.o | grep trn2dKnownDemod    # expect .text 0x25de0
docker exec d-modem sh -c "cd /src/slmodemd && \
  objcopy --add-symbol \
    _ZN20V90Phase4Demodulator15trn2dKnownDemodEs_original=.text:0x25de0,global,function \
    dsplibs.o dsplibs_trnref.o && \
  gcc -m32 -O2 -c smartlink_trn2d_reference_wrap.c -o trn2d_wrap.o"
# then relink slmodemd substituting 'trn2d_wrap.o dsplibs_trnref.o' for
# 'dsplibs.o' in the final link line, with -Wl,--allow-multiple-definition,
# and keep the result as slmodemd_trnref (leave the stock binary alone).
```

### Run

Rig side (note the deployed d-modem binary may still default to zoh — the env
overrides it either way; rebuild per `rig/README.md` when convenient):

```sh
docker exec d-modem sh -c "pkill slmodemd; pkill socat" ; sleep 1
docker exec -d -e SIP_LOGIN=6000:6000@asterisk.net.cryan.nz \
  -e DM_RESAMPLER=sinc -e DM_RS_HEADROOM=0.25 \
  d-modem sh -c "/src/slmodemd/slmodemd_trnref -d9 -e /src/d-modem > /tmp/slm.log 2>&1"
docker exec -d d-modem sh -c "socat TCP-LISTEN:5556,reuseaddr,fork /dev/ttySL0"
# ALWAYS verify the AT bridge before trusting a run:
(printf 'AT\r'; sleep 3) | nc -w 6 tower 5556    # expect OK
```

Server side (Mac) — baseline call, then the A/B call adds
`ME_V90_SHAPER_METRIC=transmit`:

```sh
ME_V8_ANSWER_TONE=ansam_pr SIP_FORCE_PCMU=1 \
ME_TRAINING_TIMEOUT_MS=300000 \
ME_V90_J_LOOKAHEAD_BITS=3000 ME_V90_JD_RESYNC_SYMBOLS=24000 ME_V90_SD_DELAY_MS=750 \
VPCM_G711_TAP_DIR=artifacts/v90-hardware/$(date -u +%Y%m%dT%H%M%SZ)-trn2d_ref \
./sip_v90_modem --sip-server asterisk.net.cryan.nz --username 6001 --password 6001 \
  --pty /tmp/v90modem
```

`ME_TRAINING_TIMEOUT_MS=300000` stops our own 60 s fallback from capping
Phase-4 attempts at ~3 per call. Confirm hook liveness with
`docker exec d-modem grep TRN2REF /tmp/slm.log`, then retrieve:

```sh
docker cp d-modem:/tmp/smartlink-trn2d-pairs.s16 .
docker cp d-modem:/tmp/slm.log .
```

### Read-out

Diff the dumped reference stream against our `live-tx.g711` TRN2d window
(both fully known, aligned via the received stream):

- **signs**: reference sign-flipped on ~31% of symbols → SmartLink predicts
  the strict §5.4.5.6 transmitted-levels metric; re-run the A/B call with
  `ME_V90_SHAPER_METRIC=transmit` (offline this flips exactly 30.7% of signs
  and zero Ucodes for the call-13 constellation) and cross-check against the
  `smartlink_shaper_probe` oracle.
- **levels**: a scalar or per-Ucode offset → pad/resampler-gain workstream,
  not the shaper.
- **timing**: frame misalignment shows immediately in the pair stream.
