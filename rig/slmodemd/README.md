# slmodemd rig patches

Patches applied to the SmartLink slmodemd on `tower.net.cryan.nz` (container
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
