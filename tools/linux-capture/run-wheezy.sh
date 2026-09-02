#!/bin/sh
# Boot the Debian 7 (kernel 3.2) capture guest.  No bootloader: qemu loads the
# kernel and initrd straight off the host, rootfs is a bare ext4 image on vda.
# USB passthrough is by vid:pid so a replug re-attaches -- and it MUST be
# replugged after the guest is up (the bootloader answers EP0 for only ~3 s).
cd /mnt/user/domains/hsfcap/wheezy
exec qemu-system-x86_64 \
  -enable-kvm -m 2048 -smp 2 -nographic \
  -kernel vmlinuz -initrd initrd \
  -append "root=/dev/vda rw console=ttyS0,115200" \
  -drive file=wheezy.img,if=virtio,format=raw \
  -netdev tap,id=n0,ifname=tap0,script=no,downscript=no \
  -device virtio-net-pci,netdev=n0,mac=52:54:00:a5:fc:02 \
  -device qemu-xhci,id=xhci \
  -device usb-host,bus=xhci.0,vendorid=0x0572,productid=0x1300 \
  -serial file:console-wheezy.log -monitor telnet:127.0.0.1:4444,server,nowait \
  "$@"
