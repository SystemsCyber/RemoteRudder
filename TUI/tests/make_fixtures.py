#!/usr/bin/env python3
"""
Regenerate test fixtures from the full capture logs.

The full logs are ~100 MB and are not committed. This carves small slices that
preserve the characteristics the tests depend on, keeping only arbitration IDs
the HMI actually decodes.

The trolling slice needs care: `combined_filtered.log` runs at planing speed
for its first ~100k frames, so slicing from the start gives you a second copy
of the fast regime. This script scans for the window with the most sub-1.6 mph
samples instead of assuming an offset.

Usage:
    python3 tests/make_fixtures.py [--logs ../logs] [--out fixtures]
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

from can.io.canutils import CanutilsLogReader

DECODED_IDS = {
    0x18F01D21,  # steering
    0x19F10D13,  # rudder
    0x09F8021C,  # GPS COG/SOG
    0x09F8011C,  # GPS position
    0x18FEE81C,  # vehicle direction (compass, pitch)
    0x0CF00400,  # engine
    0x09F112F8,  # vessel heading
    0x18FF50E0,  # autopilot status
}

COG_SOG_ID = 0x09F8021C
SLOW_THRESHOLD_MPH = 1.6


def sog_mph(msg) -> float | None:
    if msg.arbitration_id != COG_SOG_ID or len(msg.data) < 6:
        return None
    try:
        return struct.unpack("<H", msg.data[4:6])[0] / 100.0 * 2.236936
    except struct.error:
        return None


def carve(src: Path, dst: Path, want: int, skip: int = 0) -> int:
    """Write `want` decoded frames from `src` to `dst`, after skipping `skip`."""
    written = 0
    skipped = 0
    lines = []
    for msg in CanutilsLogReader(str(src)):
        if msg.arbitration_id not in DECODED_IDS:
            continue
        if skipped < skip:
            skipped += 1
            continue
        lines.append(
            f"({msg.timestamp:.6f}) can0 "
            f"{msg.arbitration_id:08X}#{msg.data.hex().upper()}\n"
        )
        written += 1
        if written >= want:
            break
    dst.write_text("".join(lines))
    return written


def find_slow_window(src: Path, window: int = 5000, step: int = 500) -> int:
    """Return the decoded-frame offset of the window richest in slow samples."""
    idx = 0
    slow = []
    for msg in CanutilsLogReader(str(src)):
        if msg.arbitration_id not in DECODED_IDS:
            continue
        v = sog_mph(msg)
        if v is not None and v < SLOW_THRESHOLD_MPH:
            slow.append(idx)
        idx += 1

    if not slow:
        return 0

    best_start, best_count = 0, 0
    for start in range(0, max(idx - window, 1), step):
        count = sum(1 for i in slow if start <= i < start + window)
        if count > best_count:
            best_start, best_count = start, count
    print(f"    slow window at frame {best_start} ({best_count} slow samples)")
    return best_start


def describe(path: Path) -> None:
    ids = set()
    speeds = []
    n = 0
    for msg in CanutilsLogReader(str(path)):
        ids.add(msg.arbitration_id)
        v = sog_mph(msg)
        if v is not None and v < 200:
            speeds.append(v)
        n += 1
    line = f"    {path.name}: {n} frames, {len(ids)} unique IDs"
    if speeds:
        below = sum(1 for s in speeds if s < SLOW_THRESHOLD_MPH)
        line += (
            f", SOG {min(speeds):.2f}-{max(speeds):.2f} mph, "
            f"{below}/{len(speeds)} below {SLOW_THRESHOLD_MPH}"
        )
    line += f", autopilot_status={'yes' if 0x18FF50E0 in ids else 'NO'}"
    print(line)


def main() -> int:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", default=str(here.parent / "logs"))
    ap.add_argument("--out", default=str(here / "fixtures"))
    args = ap.parse_args()

    logs = Path(args.logs)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    fast_src = logs / "candump-2025-08-06_16442_horsetooth_firstConstants.log"
    slow_src = logs / "combined_filtered.log"

    missing = [p for p in (fast_src, slow_src) if not p.exists()]
    if missing:
        print("Missing source logs:", *[f"  {p}" for p in missing], sep="\n")
        print("\nThe full captures are not committed. Point --logs at them.")
        return 1

    print("Carving fixtures...")

    print("  underway_fast.log")
    carve(fast_src, out / "underway_fast.log", want=4000, skip=2000)
    describe(out / "underway_fast.log")

    print("  trolling_slow.log")
    offset = find_slow_window(slow_src)
    carve(slow_src, out / "trolling_slow.log", want=5000, skip=offset)
    describe(out / "trolling_slow.log")

    print("  smoke.log")
    carve(fast_src, out / "smoke.log", want=400)
    describe(out / "smoke.log")

    total = sum(p.stat().st_size for p in out.glob("*.log"))
    print(f"\nTotal fixture size: {total/1024:.0f} KB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
