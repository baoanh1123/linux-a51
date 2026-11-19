#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""
Verify ramoops pattern records written by exynos9611-ramoops-console-init.c.

Usage:
    exynos9611-ramoops-pattern-verify.py console <console-ramoops-0.raw>
    exynos9611-ramoops-pattern-verify.py pmsg <pmsg-ramoops-0.raw>

The console mode checks the fixed markers and the four CONSOLEPAT records; the
pmsg mode checks the 480 KiB stream written to /dev/pmsg0. Every mismatch is
reported with the offset, the expected and observed bytes and the changed bits,
so a systematic write/data-integrity fault can be told apart from stale data.
"""

import sys
from collections import Counter

FIRST_CHAR = 0x20
CHARS = 95

PATTERN_COUNT = 4
PATTERN_DATA = 880
PMSG_PATTERN_BYTES = 480 * 1024

MARKERS = [
    b"RAMOOPS-CONSOLE-ONLY-BEGIN-9C21",
    b"RAMOOPS-CONSOLE-ONLY-ASCII-000: 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz",
    b"RAMOOPS-CONSOLE-ONLY-ASCII-001: abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ9876543210",
    b"RAMOOPS-CONSOLE-ONLY-END-9C21",
]


def console_pattern(index):
    return bytes(FIRST_CHAR + (i * 37 + index * 53) % CHARS
                 for i in range(PATTERN_DATA))


def pmsg_pattern():
    return bytes(FIRST_CHAR + (pos * 37 + (pos // 1024) * 53) % CHARS
                 for pos in range(PMSG_PATTERN_BYTES))


def report_mismatches(name, expected, actual, limit=40):
    if len(expected) != len(actual):
        print(f"{name}: length mismatch expected {len(expected)} "
              f"got {len(actual)}")
    bad = [i for i in range(min(len(expected), len(actual)))
           if expected[i] != actual[i]]
    total = min(len(expected), len(actual))
    print(f"{name}: {total} bytes checked, {len(bad)} differ "
          f"({len(bad) / total * 100:.3f}%)" if total else f"{name}: no data")
    cleared = Counter()
    setbits = Counter()
    for i in bad[:limit]:
        e = expected[i]
        a = actual[i]
        cleared.update(bit for bit in range(8) if e & (1 << bit) and not a & (1 << bit))
        setbits.update(bit for bit in range(8) if a & (1 << bit) and not e & (1 << bit))
        print(f"  off={i:7d} exp={chr(e)!r}({e:02x}) got={chr(a)!r}({a:02x}) "
              f"cleared={e & ~a & 0xff:02x} set={a & ~e & 0xff:02x}")
    if len(bad) > limit:
        print(f"  ... {len(bad) - limit} more")
    for i in bad[limit:]:
        e = expected[i]
        a = actual[i]
        cleared.update(bit for bit in range(8) if e & (1 << bit) and not a & (1 << bit))
        setbits.update(bit for bit in range(8) if a & (1 << bit) and not e & (1 << bit))
    print(f"  cleared bit histogram: {sorted(cleared.items())}")
    print(f"  set bit histogram:     {sorted(setbits.items())}")
    blocks = Counter(i // 1024 for i in bad)
    print(f"  {len(blocks)} of {max(1, (total + 1023) // 1024)} 1 KiB blocks "
          f"affected")
    return bad


def verify_console(data):
    ok = True
    for marker in MARKERS:
        idx = data.find(marker)
        if idx < 0:
            ok = False
            print(f"marker missing: {marker.decode()!r}")
            continue
        print(f"marker ok at {idx}: {marker.decode()!r}")
    for index in range(PATTERN_COUNT):
        header = f"CONSOLEPAT-{index}-LEN{PATTERN_DATA}: ".encode()
        idx = data.find(header)
        if idx < 0:
            ok = False
            print(f"record header missing: {header.decode()!r}")
            continue
        start = idx + len(header)
        report_mismatches(f"CONSOLEPAT-{index}", console_pattern(index),
                          data[start:start + PATTERN_DATA])
    return ok


def verify_pmsg(data):
    expected = pmsg_pattern()
    report_mismatches("pmsg", expected, data)
    return True


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in ("console", "pmsg"):
        print(__doc__)
        return 2

    with open(sys.argv[2], "rb") as handle:
        data = handle.read()

    if sys.argv[1] == "console":
        verify_console(data)
    else:
        verify_pmsg(data)

    return 0


if __name__ == "__main__":
    sys.exit(main())
