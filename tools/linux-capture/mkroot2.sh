#!/bin/bash
# Second pass over the wheezy rootfs: sshd + DHCP, so the guest can be driven
# over the LAN instead of through a write-only serial log.
set -e
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --force-yes --no-install-recommends openssh-server isc-dhcp-client iproute
cat > /etc/network/interfaces <<EOF
auto lo
iface lo inet loopback
auto eth0
iface eth0 inet dhcp
EOF
mkdir -p /root/.ssh && chmod 700 /root/.ssh
cat > /root/.ssh/authorized_keys <<EOF
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIBtSBjwLvp5f8pchKqX+0gK18Naq9ABKlisisjVGfwcZ scottcryan@Scotts-MacBook-Air-10.local
EOF
chmod 600 /root/.ssh/authorized_keys
sed -i 's/^PermitRootLogin.*/PermitRootLogin yes/' /etc/ssh/sshd_config
cat > /etc/rc.local <<EOF
#!/bin/sh -e
modprobe usbmon || true
mount -t debugfs none /sys/kernel/debug 2>/dev/null || true
echo "GUEST IP: \$(ifconfig eth0 | grep -o 'inet addr:[0-9.]*' | cut -d: -f2)" > /dev/ttyS0
exit 0
EOF
chmod +x /etc/rc.local
echo done
