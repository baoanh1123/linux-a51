#!/bin/sh
#
# Record one line per boot for the exynos9610-wlbt stability test, then reboot
# for the next sample.  Install it as a systemd oneshot unit so the run needs no
# ssh session - ssh over the USB link drops while the phone reboots, and polling
# it from the host distorts the very timing being measured.
#
#   /etc/systemd/system/wlan-boottest.service:
#	[Unit]
#	Description=WLAN stability boot test
#	After=NetworkManager.service
#	Wants=NetworkManager.service
#	[Service]
#	Type=oneshot
#	ExecStart=/root/wlan-boottest.sh
#	[Install]
#	WantedBy=multi-user.target
#
#   /root/wlan-boottest.runs   number of boots to collect
#   /root/wlan-boottest.count  written by this script
#   /root/wlan-boottest.log    the results
#
# The NFS root is exported from the host, so the log can be read from there
# (/srv/a51/rootfs/root/wlan-boottest.log, root-only directory).
#
# To put a wpa_supplicant in front of the join instead of NetworkManager: the
# unit must be Type=simple with wpa_supplicant in the foreground.  With
# Type=oneshot and "wpa_supplicant -B" systemd kills the daemonised supplicant
# with the unit's cgroup as soon as ExecStart returns, which looks exactly like
# "no join happened, so no crash" in the results.
#
L=/root/wlan-boottest.log
C=/root/wlan-boottest.count
N=$(cat /root/wlan-boottest.runs 2>/dev/null); N=${N:-0}
n=$(cat $C 2>/dev/null); n=${n:-0}
n=$((n+1))
echo $n > $C
sleep 70
wd=$(journalctl -k -b | grep -c "peer watchdog")
st=$(nmcli -t device status 2>/dev/null | grep wlan0 | cut -d: -f3)
ip=$(ip -br a show wlan0 2>/dev/null | grep -o "10\.9\.9\.[0-9]*" | head -1)
lat=$(ping -c2 -W2 -I wlan0 10.9.9.1 2>/dev/null | tail -1)
{
  echo "=== boot $n $(date -Is) watchdog=$wd state=$st ip=${ip:-none}"
  echo "    ping: ${lat:-failed}"
  journalctl -k -b | grep "exynos9610-wlbt" | sed "s/.*wifibt: //" |
    grep -vE "Call trace|Modules linked|^\s+\w+\+0x|fwlog" |
    grep -E "privacy|connected to|port authorized|peer watchdog|WLAN start|handshake|recovery|restart|refused|timeout"
} >> $L
if [ $n -lt $N ]; then sync; reboot; fi
