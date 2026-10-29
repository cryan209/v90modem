# User guide

This guide takes you from a source checkout to a modem call and explains how to
attach an application. For current interoperability limits, see the
[status table](../readme.md#status-26-october-2026). A supported modulation is
not a promise that every hardware peer or VoIP provider will work.

## 1. Choose the connection you need

| Goal | Setup |
|---|---|
| Explore without a SIP account or hardware | [Local GUI loopback](#local-demo-no-sip-account-required) |
| Answer an ordinary analogue modem | Run this program on SIP; connect the hardware modem to an FXS gateway/ATA routed to your SIP account |
| Dial a remote modem service | Run this program on SIP; use `ATD<number>` on its PTY |
| Attach a BBS, PPP or other data application | First establish a working call, then use the [application interface](#6-attach-an-application) |
| Send or receive fax | Use a fax application and the [fax AT interface](fax_class_at.md), not a terminal data session |

The normal topology is:

```text
hardware modem -- telephone cable -- FXS gateway/ATA -- PBX -- SIP/RTP -- sip_v90_modem -- PTY -- application
```

No sound card or USB modem is needed on the SIP host. The PTY emulates a serial
modem's command/data port; it does **not** itself supply a login prompt, BBS,
PPP server or Internet connection.

For V.90/V.92, the digital side must reach the analogue modem's D/A converter
through a byte-exact G.711 path. A generic Internet voice service, an extra
analogue conversion, or audio processing can prevent PCM operation. V.34 is a
useful comparison but is also sensitive to VoIP impairments. See
[the line requirements](../readme.md#what-the-line-must-provide).

## 2. Build

Run commands from the repository root. Keep the bundled `pjproject/` and
`spandsp-master/` trees: their local modifications are part of the modem.
Do not replace them with distribution or Homebrew modem libraries.

### macOS

Install Apple's command-line tools (if not already installed) and Homebrew.
Then install the build dependencies and an optional terminal program:

```sh
xcode-select --install
brew install autoconf automake libtool pkg-config libtiff jpeg-turbo openssl@3 picocom
make sip_v90_modem
```

The makefile discovers Homebrew's prefix. The supported local PJSIP build
selects its own architecture suffix; `ARCH_SUFFIX` is not needed for it.

### Debian / Ubuntu

```sh
sudo apt update
sudo apt install build-essential autoconf automake libtool pkg-config \
  libasound2-dev libssl-dev libopus-dev libtiff-dev libjpeg-dev uuid-dev \
  libavformat-dev libavcodec-dev libswscale-dev libavutil-dev libv4l-dev \
  python3 picocom
make sip_v90_modem
```

These are development packages for the build's system dependencies, not
substitutes for the vendored SpanDSP and PJSIP.

### Check the build

```sh
./sip_v90_modem --help
make test-fast
```

`make sip_v90_modem` builds the application; plain `make` also builds the test
binaries. `make test-fast` builds and runs the quick test tier. These tests are
offline checks, not proof of hardware interoperability. Failure logs are kept
under `test-logs/`; see [testing](../readme.md#testing) for the longer suites.

Python 3 is needed for the GUI launcher and test tools. The GUI uses the
standard library. The optional scientific analysis tools have extra packages:

```sh
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements-tools.txt
```

### Local demo (no SIP account required)

```sh
./sip_v90_modem --gui-loopback
# Browser alternative (also on Linux):
./sip_v90_modem --gui-web --loopback
```

Choose a loopback profile in the GUI to explore training and data. This does
not test a PBX, ATA or real telephone line. See [GUI mode](gui_mode.md) for
controls, the local ATA test server and other backends.

## 3. Start a SIP line

You need an account on a PBX or SIP service that routes modem calls and permits
unprocessed G.711. Configure the PBX/gateway for PCMU or PCMA end to end and
disable VAD/comfort noise, echo cancellation, gain control and noise
suppression. For initial diagnosis, use a wired LAN with a known gateway.

In terminal 1:

```sh
./sip_v90_modem --sip-server pbx.example.net \
  --username 6001 --password 'replace-with-your-password' \
  --pty-link /tmp/v90modem
```

Replace all example account details. Leave this process running and watch its
SIP registration and call logs. The local SIP port defaults to UDP 5060; if
another program owns it, choose an unused port with `--local-port`.
`--rtp-port` selects the media base port; if explicitly configuring it, use an
even port and allow the adjacent RTCP port too. SIP signalling and RTP media
must both be reachable through host firewalls and the PBX/network.

`--bind-addr` selects a local interface address, not a general NAT solution.
Start on a LAN before diagnosing remote/NAT paths. A registration error does
not necessarily terminate the process: check the log, not just its existence.

**Credentials:** `--password` can appear in shell history and process listings.
Use a dedicated test account, avoid shared hosts, and redact commands/logs
before sharing them. Captures can contain the actual transferred data.

Without `--sip-server`, the program listens in peer-to-peer mode rather than
registering with a PBX. For a guided local ATA setup, use the
[GUI ATA test server](gui_mode.md#ata-test-server). `--no-register` keeps
outbound authentication but does not register a new Contact; it is not the
normal choice when you want incoming calls routed to this process.

## 4. Open the AT console

In terminal 2:

```sh
picocom --baud 115200 --flow n /tmp/v90modem
```

The baud value is a terminal-program setting, not the negotiated modem speed:
a PTY has no physical serial baud rate. Disable local echo and software flow
control, use 8N1, and send a carriage return with Enter. In picocom, exit with
Ctrl-A then Ctrl-X. Minicom can also be used; disable its modem initialisation
and flow-control settings if they interfere with the session.

Enter each line below separately, waiting for its response:

```text
AT
ATI3
AT+MS?
AT+MR=1;+ER=1;+DR=1
```

`AT` should return `OK`; `ATI3` identifies the build, `+MS?` reads the current
modulation offer, and the last line enables rate/error-control/compression
reports before `CONNECT`. `AT$` lists the built-in help pages.

Use only one reader per PTY. A terminal and a data application opening the
same port compete for bytes; they do not each receive a copy.

## 5. Make a call

### Dial out

```text
ATD6002
```

This dials `sip:6002@pbx.example.net` using the configured server. Use a dialled
number or extension, not a full SIP URI in the AT command. While dialling, a
key can abort the attempt; otherwise the factory S7 timeout is 120 seconds.

After `CONNECT`, the combined PTY carries remote payload, **not AT commands**.
Text you type is sent to the peer. A blank screen may simply mean the peer is
waiting for your application to speak first.

### Answer an incoming call

Route another modem's call to your SIP account. By default the program reports
`RING` and answers after two rings. To answer manually:

```text
ATS0=0
```

Wait for `RING`, then send `ATA`. Use `ATS0=2` to restore automatic answering.
The other modem still needs a data application attached at its end.

### Escape, inspect and hang up

On the combined console:

1. Stop sending data for at least one second (the factory guard time).
2. Type `+++` **without Enter**.
3. Send nothing for at least another second; wait for `OK`.
4. Send `ATI6` for link diagnostics, `ATI11` for training detail, or `ATH` to
   hang up. If still connected, `ATO` returns to payload mode.

Guard timing uses S12 and the escape character uses S2. Typing `ATH` directly
in payload mode sends those letters to the remote application instead of
hanging up. Closing the terminal is not a reliable substitute: a PTY has no
physical DTR signal. Stop the modem process with Ctrl-C in terminal 1 when
finished; that ends its active call.

### Change modulation for the next call

In command state, before dialling or answering:

```text
AT+MS=V34
AT+MS?
```

Use `AT+MS=V90` to return to the normal PCM offer with fallback, or
`AT+MS=V92` followed by `AT+PIG=0` to offer V.92 PCM upstream. Changes do not
alter a call in progress. `ATZ`/`AT&F` restore the power-on offer (subject to
stored profiles for `ATZ`). Supported carriers, automode behaviour, and the
limits of rate-bound enforcement are in [the AT reference](at_commands.md#modulation-atms).

## 6. Attach an application

For a classic serial-modem application, give it `/tmp/v90modem` and let it
manage AT commands and the transition at `CONNECT`. Stop picocom first. The
application must handle the remote service's protocol; modem negotiation does
not automatically negotiate PPP, log in, or start a shell.

For an application that only wants payload, use split consoles instead:

```sh
./sip_v90_modem --sip-server pbx.example.net \
  --username 6001 --password 'replace-with-your-password' \
  --control-link /tmp/v90-control --data-link /tmp/v90-data
```

- Open `/tmp/v90-control` in a terminal for all AT commands and call events.
  It stays in command mode during a call, so no `+++` is needed.
- Attach the payload application to `/tmp/v90-data`. It receives data only,
  not `RING`, `CONNECT`, or `NO CARRIER`. Arrange its call lifecycle through
  the control console; do not expect the data port to run modem chat scripts.
- Configure a byte-oriented application to use raw terminal mode without
  local echo, CR/LF translation or XON/XOFF processing.
- Do not combine `--pty-link` with the split-console options. Both
  `--control-link` and `--data-link` must be supplied together.

For PPP transfer examples and measured results, see
[Eicon soaks and PPP](eicon_soak_ppp.md). Those scripts target the project rig,
not an arbitrary provider. Multi-line services have per-line ports and links:
see [multi-line operation](multi_line.md).

## 7. Save a working configuration

Create a private directory and start the modem with a profile path:

```sh
mkdir -p "$HOME/.config/v90modem"
# Add this option to your usual launch command:
# --profile "$HOME/.config/v90modem/profile.conf"
```

Set the desired AT parameters, then send:

```text
AT&W0
AT&Y0
AT&V
```

`&W0` stores profile 0; `&Y0` chooses it at power-on. Without `--profile`,
stored settings disappear when the process exits. The file stores AT settings
and dial strings, not SIP credentials or a complete service launch command.
See [stored profiles](at_stored_profiles.md) for restore and error behaviour.

## Troubleshooting

| Symptom | Check next |
|---|---|
| Build cannot find TIFF, JPEG, OpenSSL or another library | Install the development packages above; confirm `pkg-config` can see them. Do not work around it by switching to system SpanDSP/PJSIP. |
| Behaviour did not change after a header edit | Run `make clean`, then rebuild; stale objects can hide header changes. |
| PTY path missing or cannot be opened | Check that the modem is still running, that its startup log reports a console, and that the link's directory is writable. Use separate link paths for separate instances. |
| `AT` gives no visible `OK` | Confirm the correct PTY, Enter/CR, echo/flow settings, and no competing reader. If already connected, escape first or use the control port. |
| Registration fails or incoming calls never ring | Check account credentials, registrar address, PBX routing, local port conflicts and firewall reachability. Inspect the SIP log. |
| SIP call answers but training times out | Verify bidirectional RTP, PCMU/PCMA agreement, no transcoding/audio processing, and the far-end modem role. SIP answer is not modem `CONNECT`. |
| `CONNECT` but no prompt or useful data | Check what application is attached at each end, combined versus split console use, and the negotiated error control/compression (`ATI6`). |
| Repeated retrains, corruption or V.34 fallback | Check the bearer and timing before changing DSP settings. Compare a V.34 call and preserve diagnostics; `CONNECT` alone is not a payload test. |
| Log says `falling back to linear PCM bridge` | The PJSIP build lacks G.711 passthrough. Rebuild the bundled tree with `pj_config_site.h`; the linear bridge is not suitable for V.90. |

After a failed call, collect `ATI3`, `ATI6`, `ATI11` and `ATY11`, the exact
launch command (redacted), peer model/firmware, selected law, and PBX/gateway
path. `--verbose` adds engine logging. For raw G.711 taps and a bounded,
recorded call, follow [README diagnostics](../readme.md#diagnostics) and
[hardware interoperability](v90_hardware_interop.md). Check our transmit tap
before attributing a failure to the peer. Treat recordings as sensitive.

### Keeping logs bounded

The test runner retains two output segments per test: `test-logs/NNNN.log`
(newest) and `NNNN.log.1` (previous). Each is limited to **4 MiB**, so captured
stdout/stderr uses at most **8 MiB per test**. Older output is discarded;
read `.log.1` before `.log` for chronological order. Exit statuses, timeouts
and failure scratch directories are still preserved. Increase the capture
window when needed with `tools/run_tests.py --log-segment-mib 16 ...`.

For a long-running server, use the same bounded capture instead of an unlimited
shell redirect (substitute your normal arguments):

```bash
set -o pipefail
./sip_v90_modem ... 2>&1 | python3 tools/rotate_log.py modem.log
```

This also retains two 4 MiB segments, `modem.log.1` then `modem.log`. Use
`--segment-mib 16` to increase each segment. **The command overwrites these
paths on startup.** Output goes to the files, not the terminal; use
`tail -F modem.log` in another terminal to follow rotation. With a pipeline,
`$!` identifies the logger, not the modem; use your normal process supervisor
to stop/manage the server.

These limits apply only to captured stdout/stderr, not G.711/PCM taps, packet
captures, or separate diagnostic files opened by a test or the modem. Disable
those captures when not needed. Routine live runs should also leave
`ME_V34_SPAN_FLOW_LOG`, `VPCM_ME_VERBOSE` and `--verbose` off; SpanDSP FLOW
traces are synchronous and can interfere with real-time audio, not just fill
disk. `V34_DUPLEX_LOG` enables similarly verbose offline training traces.

Optional `VPCM_G711_TAP_DIR` and `ME_G711_CAPTURE` recordings now use bounded
background-writer queues. Check each raw file's `.status.json` sidecar before
treating it as complete: overflow or disk errors stop that capture at a prefix,
without changing live audio. These queues do not limit raw-file disk size.
See [background captures](performance.md#background-g711-captures) for the
completion checks and remaining synchronous diagnostic paths.

For callback stalls and CPU investigation, start with the compact per-call
`[PERF]` summaries described in [performance](performance.md). They are
available without enabling verbose traces.

Most environment variables are diagnostic or experimental, not normal user
configuration. Prefer AT commands and documented launch options; consult the
[classified environment inventory](env_knobs.md) before using a tuning knob.
