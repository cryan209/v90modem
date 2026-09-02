# Running the vendor hsfmodem driver under Linux, for a usbmon capture

The point is the one thing neither source reading nor probing the device could
give us: **the bulk-OUT byte sequence the vendor driver uses to make this part
transmit.**

**This now works.**  The stack loads, binds, dials and streams under Debian 7,
and the captures are in `artifacts/hsf-usbmon/`.  Findings are written up in
`docs/hsf_usb_daa.md` ("The vendor driver, captured on real hardware").

## Recipe

Host: `tower`, x86_64, Unraid 6.12 -- qemu + KVM, **no SLIRP**, so networking
is a tap on `br0`.  Everything lives in `/mnt/user/domains/hsfcap/wheezy`.

**Use kernel 3.2, not 3.13.**  On 3.13 the shim's `OSSCHED` (8 pointers, 64
bytes, embedded in the blob's own structures so it cannot be enlarged) is too
small for the `>=3.8` `kthread_work` path, and the workaround in
`osservices-ossched.patch` gets as far as an oops in `hsfengine1901_` during
probe.  Pre-3.8 avoids that path entirely and **needs no patch at all**.

1. **Rootfs**: `mkroot.sh` then `mkroot2.sh`, both run inside a `debian:wheezy`
   container over a bind mount.  They install kernel 3.2, headers,
   `build-essential`, `usbutils`, `tcpdump`, sshd and a DHCP client, and set up
   serial autologin.  Debian 7's archive is still served by
   `archive.debian.org`, so **apt works** -- unlike trusty, whose archives are
   gone.  `Acquire::Check-Valid-Until false` and the `wheezy/updates` security
   line are both required (without security, `libc6-dev` is uninstallable).
2. **Disk**: `docker export` the container into an ext4 image.  A modern
   `mkfs.ext4` writes features 3.2 cannot read -- create it with
   `-O ^metadata_csum,^64bit,^metadata_csum_seed,^orphan_file`, or the guest
   fails to mount root (`unsupported optional features (2000)`) and then, once
   that is fixed, drops to a maintenance shell because wheezy's `e2fsck`
   rejects `orphan_file` as `FEATURE_C12`.
3. **Boot** with `run-wheezy.sh`: qemu loads `vmlinuz`/`initrd` straight off
   the host, so there is no bootloader and no installer.  USB passthrough is by
   vid:pid so a replug re-attaches.
4. **Build in the guest** -- there is a compiler in it now, so no container and
   no cross-build:

       make -C modules all IMPORTED_ARCH=x86_64

   `IMPORTED_ARCH` is still needed (`uname -i` returns "unknown"); `-DRETPOLINE`
   is **not** -- that was only ever trusty's vermagic.  The `GPL/hda` sub-build
   fails on `sound/driver.h`; the makefile ignores it and it is irrelevant to a
   USB device.
5. **Load order**: `hsfosspec, hsfengine, hsfserial, hsfsoar, hsfusbcd2`.
   Without `hsfsoar`, `hsfusbcd2` fails on `GetSOARLibInterface`.
6. **Userspace**: `make install IMPORTED_ARCH=x86_64`, then
   `hsfconfig --auto --country=NEW_ZEALAND`.  Without it the driver logs
   `NVM_Open: cannot create instance ... err=-2` and `hsfdcpd returned -2` on
   every probe.  **hsfconfig unloads the modules when it finishes** -- reload
   them before capturing.

## Capturing

* `capture-full.sh` -- the definitive one.  Starts `tcpdump` **before** the
  driver loads, so the capture carries enumeration, interface claiming, the
  whole `CD2_CONTROL_SCRIPT` bring-up and the bulk datapump on a live dial.
* `capture-at.sh` -- same but via the usbmon *text* interface.  Note it
  **truncates payloads at 32 bytes**, which loses the tail of every script body
  and all of the bulk audio.  Prefer the pcap.
* `capture-run.sh` -- driver load plus a USB reset, no call.

Decode with `tools/hsf_pcap.py` (pcap, full payloads) or `tools/hsf_usbmon.py`
(text).  `hsf_pcap.py --bulk-out` writes the transmit stream as a raw file.

The device's bootloader answers EP0 for only ~3 s after it enumerates, so if a
firmware upload is ever needed the driver must already be loaded when the part
appears; `usbreset.c` in the guest resets it through qemu to re-open that
window.  In practice the part came up already holding firmware and no
`CD2_UPLOAD_FIRMWARE` was ever issued.

## Guest access

`ssh hsfcap` (see `~/.ssh/config`).  **OpenSSH 6.0 predates ed25519**, so the
guest needs an RSA key and the client needs
`PubkeyAcceptedAlgorithms +ssh-rsa` / `HostkeyAlgorithms +ssh-rsa`.

## Making the vendor driver narrate its own data path

`modules/osusb.c` is GPL source and the blob reaches the bus only through it,
so it can be made to log exactly what the blob asks for -- which usbmon cannot
show, because usbmon records what crossed the wire and not the host-side
structure that produced it.

`_DEBUG` alone is **not** enough: `dbg` is defined as a no-op elsewhere and
`osusb.c`'s `#ifndef dbg` guard therefore never fires.  Force it, near the top
of `osusb.c` (just before `#define Working TRUE`):

    #undef dbg
    #define dbg(format, arg...) printk(KERN_DEBUG "hsfdbg: " format "\n" , ## arg)

then `make -C modules all IMPORTED_ARCH=x86_64` and reload.  `dmesg | grep
hsfdbg` then carries every `OsUsbMakeDataReceiveRequest` /
`...TransmitRequest` / `...ControlRequest` with its size and buffer.

Gotchas: `rmmod` fails with "in use" if a tty is still open or the interface is
bound (`fuser -k /dev/ttySHSF0`, then unbind under
`/sys/bus/usb/drivers/hsfusbcd2/`), and once the refcount is stuck only a guest
restart clears it.  If `insmod` says "File exists" the OLD module is still
loaded and you are reading stale behaviour.
