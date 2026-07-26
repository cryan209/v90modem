# V.90 Hardware Interoperability Procedure

## Required bearer

- analogue modem connected to an FXS gateway
- SIP server, gateway, and modem server on the same LAN for the first tests
- PCMU or PCMA with no transcoding
- VAD/CNG, PLC, AGC, noise suppression, and gateway echo cancellation disabled
- raw RTP capture when the gateway or PBX can provide it

## One bounded attempt

```bash
./tools/v90_hardware_interop.py \
  --label "modem-chipset_gateway-model_pcmu" \
  --duration 180 \
  -- \
  --sip-server asterisk.example \
  --username 6001 \
  --password 'secret' \
  --pty /tmp/v90modem
```

During the run, place one call from the analogue modem. After connection, send
a deterministic payload through the PTY. Keep the generated directory intact;
`manifest.json` hashes the log and raw G.711 taps, while `summary.json` records
the first failed protocol layer when DATA is not reached.

## Evidence to record

- analogue modem manufacturer, model, firmware, and chipset
- FXS gateway model, firmware, port settings, and PCM law
- PBX version and codec configuration
- negotiated downstream and upstream rates shown by the analogue modem
- whether CPt, CP, MP, MP-prime, E, B1d, and DATA were observed
- first failure point from `summary.json`
- RTP loss, reordering, duplicate packets, and timestamp discontinuities
- downstream and upstream byte/bit error counts from the deterministic payload
- for USRobotics Courier tests, run the post-call diagnostic bundle before
  resetting the analogue modem:
  `./.venv/bin/python tools/cx_at.py --dev /dev/cu.usbserial-21210 usrdiag`
- for a Courier failure that reaches V.90 Phase 3/4 and then retrains, also run
  the verbose hidden-Y diagnostic bundle once:
  `./.venv/bin/python tools/cx_at.py --dev /dev/cu.usbserial-21210 usrdeepdiag`
- when the Courier itself is originating the analogue call, use its hidden Y4
  call-progress dial form for one repro run:
  `./.venv/bin/python tools/cx_at.py --dev /dev/cu.usbserial-21210 usry4dial 6001 --wait 120`

## Progression

1. V.8 and V.90 selection.
2. CRC-valid INFO0a/INFO1a.
3. S/Jd/DIL Phase 3 completion.
4. Strict CPt and TRN2d.
5. Data-mode CP, MP/MP-prime, E, and B1d.
6. Downstream unframed payload transfer.
7. Upstream V.34 payload transfer.
8. Simultaneous full-duplex transfer.
9. Retrain and rate renegotiation.

Do not promote a combination to interoperable until two consecutive fresh
calls reach the same negotiated rate and transfer the deterministic payload
without unexplained errors.

## Results matrix

| Modem/chipset | Gateway | Law | V.8 | Phase 3 | CPt/CP | E/B1d | DATA | Rates | Evidence |
|---|---|---:|---:|---:|---:|---:|---:|---|---|
| _pending_ | _pending_ | - | - | - | - | - | - | - | - |
