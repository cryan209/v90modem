#!/bin/bash
# Build a Debian 7 (wheezy, kernel 3.2) rootfs for the HSF usbmon capture guest.
# Runs INSIDE a debian:wheezy container; see tools/linux-capture/README.md.
set -e
cat > /etc/apt/sources.list <<EOF
deb http://archive.debian.org/debian wheezy main contrib non-free
deb http://archive.debian.org/debian-security wheezy/updates main contrib non-free
EOF
echo 'Acquire::Check-Valid-Until "false";' > /etc/apt/apt.conf.d/99no-check
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --force-yes linux-image-amd64 linux-headers-amd64 \
    build-essential usbutils module-init-tools procps net-tools \
    udev initramfs-tools less file
echo "root:root" | chpasswd
sed -i "/ttyS0/d" /etc/inittab
echo "T0:2345:respawn:/sbin/getty -a root -L ttyS0 115200 vt100" >> /etc/inittab
echo ttyS0 >> /etc/securetty
echo "/dev/vda / ext4 defaults 0 1" > /etc/fstab
echo hsfcap > /etc/hostname
ls -la /boot/
