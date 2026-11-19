#!/bin/sh
export PATH=/sbin:/bin:/usr/sbin:/usr/bin

mount -t proc proc /proc 2>/dev/null
mount -t sysfs sysfs /sys 2>/dev/null
mount -t devtmpfs devtmpfs /dev 2>/dev/null
mkdir -p /dev/pts /run /tmp /sys/kernel/config
mount -t devpts devpts /dev/pts 2>/dev/null
mount -t configfs configfs /sys/kernel/config 2>/dev/null

G=/sys/kernel/config/usb_gadget/g1
mkdir -p "$G"
echo 0x1d6b > "$G/idVendor"
echo 0x0104 > "$G/idProduct"
echo 0x0200 > "$G/bcdUSB"
echo 0x0100 > "$G/bcdDevice"
mkdir -p "$G/strings/0x409"
echo A51ALPINE > "$G/strings/0x409/serialnumber"
echo Samsung > "$G/strings/0x409/manufacturer"
echo "A51 Alpine ACM" > "$G/strings/0x409/product"
mkdir -p "$G/configs/c.1/strings/0x409"
echo ACM > "$G/configs/c.1/strings/0x409/configuration"
mkdir -p "$G/functions/acm.gs0"
ln -s "$G/functions/acm.gs0" "$G/configs/c.1/acm.gs0" 2>/dev/null
mkdir -p "$G/functions/ecm.usb0"
echo 02:1d:6b:00:00:01 > "$G/functions/ecm.usb0/dev_addr"
echo 02:1d:6b:00:00:02 > "$G/functions/ecm.usb0/host_addr"
ln -s "$G/functions/ecm.usb0" "$G/configs/c.1/ecm.usb0" 2>/dev/null

UDC=
i=0
while [ -z "$UDC" ] && [ $i -lt 50 ]; do
	for u in /sys/class/udc/*; do
		[ -e "$u" ] || continue
		UDC=${u##*/}
		break
	done
	[ -n "$UDC" ] || sleep 0.2
	i=$((i + 1))
done

if [ -z "$UDC" ]; then
	echo "alpine-acm: no UDC" > /dev/kmsg
	exec /bin/sh -i
fi

echo "$UDC" > "$G/UDC" || {
	echo "alpine-acm: UDC bind failed" > /dev/kmsg
	exec /bin/sh -i
}

i=0
while [ ! -c /dev/ttyGS0 ] && [ $i -lt 50 ]; do
	sleep 0.2
	i=$((i + 1))
done

stty -F /dev/ttyGS0 115200 raw -echo 2>/dev/null

i=0
while [ ! -e /sys/class/net/usb0 ] && [ $i -lt 50 ]; do
	sleep 0.2
	i=$((i + 1))
done
[ -e /sys/class/net/usb0 ] && ip link set usb0 up 2>/dev/null

# Prefer the full Alpine rootfs on the microSD ext4 partition.
mkdir -p /rootfs
ROOTDEV=/dev/mmcblk0p1
i=0
while [ ! -b "$ROOTDEV" ] && [ $i -lt 75 ]; do
	sleep 0.2
	i=$((i + 1))
done
if [ -b "$ROOTDEV" ] && mount -t ext4 -o rw,noatime "$ROOTDEV" /rootfs; then
	echo "alpine-root: switch_root $ROOTDEV" > /dev/kmsg
	exec /bin/busybox switch_root /rootfs /sbin/init
fi
echo "alpine-root: rootfs unavailable; initramfs fallback" > /dev/kmsg

if [ -e /sys/class/net/usb0 ]; then
	ip link set usb0 up
	ip addr add 192.168.42.1/24 dev usb0 2>/dev/null
	ip route add 192.168.42.0/24 dev usb0 2>/dev/null
	if ! ip route add default via 192.168.42.2 dev usb0 2>/dev/null; then
		echo "alpine-net: default route failed" > /dev/kmsg
	fi
	mkdir -p /etc
	echo "nameserver 1.1.1.1" > /etc/resolv.conf
	echo "alpine-acm: usb0 192.168.42.1/24 via 192.168.42.2" > /dev/kmsg
	if ping -c 1 -W 2 1.1.1.1 >/dev/null 2>&1; then
		echo "alpine-net: internet reachable" > /dev/kmsg
	else
		echo "alpine-net: internet NOT reachable" > /dev/kmsg
	fi
	# Local test shell; host: nc 192.168.42.1 2323
	nc -lk -s 192.168.42.1 -p 2323 -e /bin/sh >/dev/null 2>&1 &
else
	echo "alpine-acm: usb0 not found" > /dev/kmsg
fi

{
	echo
	echo "A51 Alpine ACM shell"
	echo "host console: screen /dev/ttyACM0 115200"
	echo "host network: usb0 192.168.42.2/24, device 192.168.42.1"
	echo
} > /dev/ttyGS0

exec /bin/sh -i < /dev/ttyGS0 > /dev/ttyGS0 2>&1
