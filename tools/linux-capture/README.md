# Running the vendor hsfmodem driver under Linux, for a usbmon capture

The point is the one thing neither source reading nor probing the device could
give us: **the bulk-OUT byte sequence the vendor driver uses to make this part
transmit.**  Every host-side layer has now been reproduced from static analysis
and the transmit FIFO still never drains, so the remaining move is to watch the
real driver drive real hardware.

## Recipe (verified as far as "driver loads, binds, and creates ttys")

Host used: `tower`, x86_64, Unraid 6.12 -- qemu + KVM present, **no SLIRP**, so
networking is a tap on `br0`.

1. **Guest**: Ubuntu 14.04 cloud image (kernel 3.13).  cloud-init seed built on
   macOS with `hdiutil makehybrid -iso -default-volume-name cidata`.
   **trusty's archives are GONE from old-releases**, so apt inside the guest is
   dead -- do not plan on installing anything there.
2. **USB passthrough** by vid:pid, so a replug re-attaches:
   `-device qemu-xhci,id=xhci -device usb-host,bus=xhci.0,vendorid=0x0572,productid=0x1300`
   **Replug the device AFTER the guest is up.**  This part's bootloader answers
   EP0 for only ~3 s after it enumerates, so a device attached at qemu start
   has gone silent before the guest enumerates it.
3. **Build the modules off-guest** (no compiler in the guest): copy the guest's
   `/usr/src/linux-headers-*` out and build in a `gcc:4.9` container -- the
   kernel was built with gcc 4.8 and a modern gcc cannot compile 3.13:

       docker run --rm -e KCPPFLAGS=-DRETPOLINE -v $PWD:/w -w /w gcc:4.9 sh -c \
         'cp -a usr/src/linux-headers-* /usr/src/; cd hsfmodem-linux &&
          make -C modules all IMPORTED_ARCH=x86_64 \
               CNXT_KERNELSRC=/usr/src/linux-headers-3.13.0-170-generic'

   * `IMPORTED_ARCH=x86_64` is required: `uname -i` returns "unknown" in a
     container and the build then looks for `makeflags-unknown.mak`.
   * `KCPPFLAGS=-DRETPOLINE` is required: without it vermagic lacks
     `retpoline ` and insmod refuses the module.
   * the `GPL/hda` sub-build fails on `sound/driver.h`; the makefile ignores it
     and it is irrelevant to a USB device.
4. **Load order**: `hsfosspec, hsfengine, hsfserial, hsfsoar, hsfusbcd2`.
   Without `hsfsoar`, `hsfusbcd2` fails on `GetSOARLibInterface`.
   `/dev/ttySHSF0-7` appear once `hsfserial` is in.

## The one source fix needed

`osservices-ossched.patch`, against `modules/osservices.c`.

The shim's `OSSCHED` is `8*sizeof(void*)` = 64 bytes and **the blob embeds it
in its own structures, so it cannot be enlarged** -- that is exactly what the
`sizeof()` guard in `OsInit` protects.  On 3.13 the `>=3.8` path wraps a
`struct kthread_work` plus two pointers = 72 bytes, so the module refuses to
load with `OSSCHED too small (64 < 72)`.  The patch stores a POINTER to a
separately allocated `kwork_data`, so the blob's storage holds 8 bytes and its
layout is untouched.

## Open: it oopses in the blob during USB probe

    RIP  hsfengine1901_+0x16/0xc0 [hsfengine]
         usb_probe_device -> driver_probe_device -> generic_probe

Two leads, and the second subsumes the first:

* The firmware download had already failed once ("Firmware download failed")
  because the driver loaded ~17 minutes after the device enumerated, far past
  the ~3 s bootloader window.  Probing a device in that state may simply be
  unsupported.
* **Use a kernel older than 3.8.**  That avoids the `kthread_work` path
  entirely, so this patch is not needed at all and the shim runs the code the
  vendor actually shipped and tested.  Debian 7 (kernel 3.2) is the obvious
  target: `archive.debian.org` still serves it, and kernel headers can be
  fetched as a `.deb` and `dpkg -x`'d without working apt.

## Then

`tools/hsf_capture.sh` runs inside the guest, one action per file.  Note it was
written for the CONTROL plane and says the datapump underrunning "does not
matter" -- for the transmit question the datapump is the whole point, so add a
capture of a live connection carrying bulk traffic.  Decode with
`tools/hsf_usbmon.py`.
