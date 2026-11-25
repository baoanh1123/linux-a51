#!/usr/bin/env python3
"""Decode the peer's own log ring (MXLOG phase 5) from a debugfs peek dump.

The firmware writes its log into a ring the host never reads (nothing in the
mainline driver drains the HIP log stream), so the ring keeps the last few
hundred words it wrote.  Reading it back is the only way to hear the peer
explain itself:

    echo "35dd88 4096" > /sys/kernel/debug/exynos9610-wlbt/peek
    cat /sys/kernel/debug/exynos9610-wlbt/peek > ring.txt
    exynos9610-mxlog.py ring.txt

The offset comes from the "stream log buffer +0x..." line of
/sys/kernel/debug/exynos9610-wlbt/dump.

Record layout (drivers/misc/samsung/scsc/mxlog.c, mxlog_phase5_message_handler):

    | sync|lvl|len | tstamp | offset | arg1 | arg2 | ...

  * the first word is 0x9669 << 16 with the level and the number of following
    words packed below it, so (w >> 8) & 0xff is the level and w & 0xff counts
    tstamp + offset + args;
  * offset is a byte offset into common/log-strings.bin, the firmware's format
    string table, which Samsung's vendor partition does not ship - so the format
    string is unavailable here and the record is printed as its offset plus raw
    arguments.  Offsets are stable for a given firmware, so two runs can be
    compared even without the text.

Usage:
    exynos9610-mxlog.py <peek-dump> [--from <addr>]

With --from, the dump is walked from that address instead of from wherever the
first sync word appears, which matters only when the ring wrapped.
"""

import argparse
import re
import sys

SYNC = 0x9669
WORD = re.compile(r'\b[0-9a-f]{8}\b')

LEVELS = {0: 'I', 1: 'D', 2: 'W', 3: 'E', 4: 'F'}


def words_from(path):
    """The peek file prints "addr word  addr word ...", four pairs per line."""
    words = []
    for line in open(path):
        fields = line.split()
        if len(fields) < 8:
            continue
        for i in range(1, len(fields), 2):
            words.append(int(fields[i], 16))
    return words


def decode(words, start=0):
    out = []
    i = start
    n = len(words)
    while i < n:
        w = words[i]
        if (w >> 16) != SYNC:
            i += 1
            continue
        level = (w >> 8) & 0xff
        length = w & 0xff
        if length < 2 or i + length >= n + 1:
            i += 1
            continue
        body = words[i + 1:i + 1 + length]
        if len(body) < 2:
            break
        stamp, offset = body[0], body[1]
        args = body[2:]
        out.append((stamp, level, offset, args))
        i += 1 + length
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dump')
    ap.add_argument('--from', dest='start', type=lambda x: int(x, 16),
                    default=None)
    a = ap.parse_args()

    words = words_from(a.dump)
    if not words:
        sys.exit('no words in %s' % a.dump)

    start = a.start if a.start is not None else 0
    records = decode(words, start)
    if not records and a.start is None:
        # the ring may not start on a record boundary; try every word
        for off in range(len(words)):
            records = decode(words, off)
            if len(records) > 3:
                start = off
                break

    print('%d record(s) from word %d' % (len(records), start))
    base = None
    for stamp, level, offset, args in records:
        if base is None:
            base = stamp
        argstr = ' '.join('%08x' % x for x in args)
        print('%s %+6d fmt@%08x %s' %
              (LEVELS.get(level, '?'), (stamp - base) & 0xffffffff,
               offset, argstr))


if __name__ == '__main__':
    main()
