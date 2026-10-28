# Opt-in Apple → Eicon caller profiles

These are configuration presets, not new DSP defaults. They were checked with
an Apple A1082 on the **verified working 6004 telephone lead**, dialling Eicon
7900/7910. The independent serial modem on 6005 reports NO DIALTONE; no preset
repairs that line path. Do not assume every peer supports direct LAPM.

## Recommended starting point: V.32bis, 9600, error control required

[`apple-eicon-v32bis-9600.at`](apple-eicon-v32bis-9600.at):

```text
AT+MS=V32B,0,300,9600,300,9600
AT+ES=2,3,5
```

Enter these as separate commands in the Apple GUI's **AT/control console**, not
the data console, while idle. Then dial `ATD7910#` or `ATD7900#`. The `#`
terminates the ATA's digit collection. **The GUI saves active settings before
ATD/ATA** so a fresh backend retains them; dialling can therefore persist this
preset in the GUI's stored profile. Record your existing `AT+MS?` / `AT+ES?`
values first and restore them afterward if this is only a trial. Keep native
9600-Hz audio and the normal automatic clock correction used in these checks.
The `.at` file itself is also accepted by the backend's `APPLE_GUI_PROFILE`
loader (the GUI launcher normally manages its own profile path).

`+ES=2,3,5` is the V.250 Table 20 policy: the caller initiates LAPM without
ODP/ADP detection, requires LAPM rather than falling back to raw data, and the
answerer also requires LAPM. Only **outgoing** calls were tested here. V.42
§8.10 permits establishment without the detection phase. No timer, gain,
echo-canceller, line-protection or audio-rate default is changed.

Observed short data checks, 28 October 2026:

| Destination | V.32bis 9600 / LAPM / V.42bis |
|---|---|
| 7910 | **2/2** fresh calls, three exact PPP LCP echoes each, zero bad PPP FCS |
| 7900 | **2/3** attempts passed three exact text echoes; one failed physical training |

The failed 7900 repeat overlapped the first five seconds of hardware-free
regression checks; a subsequent quiet repeat passed. That is a confound, not
proof of its cause and not grounds to omit the failed attempt. These are short
checks, not sustained-transfer, PPP/IP, or general hardware qualification.
9600 is four times the previous checked 2400 carrier rate, **not** a measured
fourfold application-throughput increase.

## Lower-speed option

[`apple-eicon-v22bis.at`](apple-eicon-v22bis.at) uses `AT+MS=V22B,0` with the
same required/direct-LAPM policy. One fresh 7900 call passed three exact text
echoes with LAPM/V.42bis, rather than the earlier buffered V.14 fallback.
This is not a two-fresh-call qualification for that preset.

To restore the normal error-control policy without changing the selected
carrier, issue `AT+ES=3,0,2`. `AT&F` resets **all** AT settings to factory values;
it is not an undo limited to this preset. Stored settings remain stored until
replaced deliberately.

## Validation and remaining work

`python3 tools/apple_eicon_profile_check.py` loads both files through the real
backend with impossible USB selection **0:0**, queries them, changes settings,
checks `ATZ0` restore and `AT&F`, and verifies the files remain unchanged. It
never dials or answers. It is included in `make gui-apple-test`.

V.34 can transfer an exact echo at 21600, but subsequent data stalls/retrains;
it has not passed the three-echo check. Lowering its ceiling, native 9000-Hz
audio, and disabling automatic clock correction did not establish a better
profile. No such experimental setting is shipped here.

Full observations and retained local evidence paths:
[`docs/apple_usb_modem_sm56.md`](../docs/apple_usb_modem_sm56.md), “Eicon reliability
profiles: direct LAPM and a checked 9600 ceiling”.
