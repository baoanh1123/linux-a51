#!/usr/bin/env python3
"""Turn a captured SCSC logring dump into the FAPI frames it contains.

The vendor WLAN driver pushes a copy of every control frame it sends and every
one it receives into the SCSC logring as a binary record, and the running
Android build exposes that ring:

    /sys/kernel/debug/scsc/ring0/samsg      # blocks, returns only new records
    /sys/kernel/debug/scsc/ring0/samlog     # whole ring snapshot (heavy)
    /sys/kernel/debug/scsc/ring0/stat

Capturing one connection:

    adb shell 'mount -t debugfs none /sys/kernel/debug'      # if not mounted
    nohup timeout 45 adb exec-out 'cat /sys/kernel/debug/scsc/ring0/samsg' \\
          > /tmp/vendor.txt &
    sleep 3
    adb shell 'cmd wifi connect-network <ssid> open'         # or wpa2 <pass>

Records look like

    <7>[ 1399.897718] [c3] [S] [bin_wifi_ctrl_tx] :: HEX[72/72]: 1820...

and the hex is printed with every 4-byte group reversed, the way
print_hex_dump() shows it, so it has to be un-reversed before it is a frame.
A frame is the FAPI signal: a 10-byte fapi_vif_signal_header (id, receiver_pid,
sender_pid, fw_reference, vif), the signal struct, then any appended payload.

Usage:
    vendor-samlog.py <capture> [tag] [id,id,...]

    tag   bin_wifi_ctrl_tx | bin_wifi_ctrl_rx | bin_wifi_data_tx
    ids   restrict to these signal ids, hex, e.g. 200b,2004,2018
"""

import re
import sys

LINE = re.compile(r'^<\d>\[\s*([0-9.]+)\].*?\[(bin_wifi_ctrl_tx|bin_wifi_ctrl_rx'
                  r'|bin_wifi_data_tx)\] :: HEX\[(\d+)/(\d+)\]: ([0-9a-f]*)')


def dehex(h):
    """Undo the 4-byte group reversal the ring reader applies."""
    b = bytearray()
    n = len(h) - (len(h) % 8)
    for i in range(0, n, 8):
        b += bytes.fromhex(h[i:i + 8])[::-1]
    if len(h) % 8:
        b += bytes.fromhex(h[n:])
    return bytes(b)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    path = sys.argv[1]
    tagf = sys.argv[2] if len(sys.argv) > 2 else None
    want = None
    if len(sys.argv) > 3:
        want = {int(x, 16) for x in sys.argv[3].split(',')}

    for line in open(path, errors='replace'):
        m = LINE.match(line)
        if not m:
            continue
        if tagf and tagf not in m.group(2):
            continue
        data = dehex(m.group(5))
        if len(data) < 2:
            continue
        sig = int.from_bytes(data[:2], 'little')
        if want and sig not in want:
            continue
        # header: id u16, receiver_pid u16, sender_pid u16, fw_reference u16,
        # vif u16 - so the signal struct starts at 10
        print("%12s %#06x len=%u %s" % (m.group(1), sig, len(data),
                                        data.hex()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
