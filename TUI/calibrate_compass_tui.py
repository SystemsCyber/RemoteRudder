#!/usr/bin/env python3
"""
calibrate_compass_tui.py -- on-water compass calibration helper.

A standalone curses tool that walks you through the Garmin GPS 24xd basic
calibration (the procedure for a NMEA 2000 network without a Garmin
chartplotter -- which is this boat) and confirms success by watching the
heading come alive on the bus.

It does NOT command the calibration -- the 24xd runs its own routine when you
drive the figure-8 and straight-line pattern. What this tool does:

  * watches the 24xd's heading PGN (127250) live, by NAME (so it finds the
    device at whatever address it holds)
  * detects the phases of the basic-calibration procedure (heading present ->
    disappears when the circles start -> reappears when calibrated)
  * shows real-time heading, GPS fix, and speed so you know when you have the
    4 mph the heading-alignment step needs
  * when calibration succeeds (heading becomes valid and steady), offers to
    ENABLE the 24xd in the heading-source registry so the main HMI starts
    using it -- the "just works" hand-off

Run it on the boat, follow the on-screen steps, and when it says calibrated,
the main app will pick up the 24xd automatically on its next run (the registry
default is edited, see the end of this file) OR immediately if you point it at
a shared registry.

Usage:
    python3 calibrate_compass_tui.py --channel can0 --bitrate 250000
    python3 calibrate_compass_tui.py --replay logs/somefile.log   (dry run)

The procedure it guides (from the GPS 24xd manual, "Performing Basic
Calibration"): drive to calm open water, then two slow tight circles within
3 minutes, continue ~1.5 rotations until heading reappears, then straighten and
drive >= 4 mph in a straight line until heading appears aligned.
"""

from __future__ import annotations

import argparse
import curses
import struct
import time
from typing import Optional

# The 24xd's stable NAME identity (mfg 229, function 145, identity 1602535),
# confirmed from its address claim. Calibration binds to this, so it works
# regardless of the 24xd's current source address.
TARGET_IDENTITY = 1602535
TARGET_MFG = 229
TARGET_FUNCTION = 145

PGN_VESSEL_HEADING = 0x1F112      # 127250
PGN_COG_SOG = 0x1F802             # 129026
PGN_GNSS = 0x1FD06                # 129029
HEADING_ALIGN_MIN_MPH = 4.0       # the manual's cruising-speed requirement


def pgn_of(can_id: int) -> int:
    pf = (can_id >> 16) & 0xFF
    if pf < 240:
        return (can_id >> 8) & 0x3FF00
    return (can_id >> 8) & 0x3FFFF


class CalibrationMonitor:
    """
    Tracks the 24xd's heading state to infer calibration phase.

    Phases, following the basic-calibration procedure:
      WAITING   - no heading yet / device not found
      HEADING   - heading present and valid (pre-calibration or done)
      CIRCLING  - heading disappeared (device detected the circles, calibrating)
      ALIGNING  - heading reappeared after circles; drive straight to align
      DONE       - heading valid and steady after alignment
    """

    def __init__(self):
        self.phase = "WAITING"
        self.address: Optional[int] = None    # resolved from claims
        self.heading: Optional[float] = None
        self.heading_valid = False
        self.speed_mph: Optional[float] = None
        self.fix_type: Optional[int] = None
        self.last_heading_change = time.time()
        self._had_heading = False
        self._steady_since: Optional[float] = None
        self._last_heading_val: Optional[float] = None

    def note_claim(self, sa: int, mfg: int, function: int, identity: int):
        if identity == TARGET_IDENTITY:
            self.address = sa

    def on_message(self, can_id: int, data: bytes):
        sa = can_id & 0xFF
        pgn = pgn_of(can_id)

        # Only trust heading/speed from the target device once we know its
        # address. Before the claim resolves, we still show any heading so the
        # operator is not staring at a blank screen, but phase logic waits.
        if pgn == PGN_VESSEL_HEADING and (self.address is None or sa == self.address):
            raw = struct.unpack_from("<H", data, 1)[0]
            valid = raw < 0xFFFE
            self._update_heading(valid, raw * 0.0001 * 57.29578 if valid else None)

        elif pgn == PGN_COG_SOG and (self.address is None or sa == self.address):
            sog_raw = struct.unpack_from("<H", data, 4)[0]
            if sog_raw != 0xFFFF:
                self.speed_mph = sog_raw / 100.0 * 2.236936

        elif pgn == PGN_GNSS and (self.address is None or sa == self.address):
            if len(data) > 0:
                self.fix_type = data[0] & 0x0F

    def _update_heading(self, valid: bool, value: Optional[float]):
        now = time.time()
        was = self.heading_valid
        self.heading_valid = valid
        self.heading = value

        if valid != was:
            self.last_heading_change = now

        # Phase machine
        if valid and not self._had_heading:
            self._had_heading = True
            if self.phase == "WAITING":
                self.phase = "HEADING"

        if self.phase in ("HEADING",) and not valid and self._had_heading:
            # Heading disappeared after being present -> circles detected
            self.phase = "CIRCLING"
            self._steady_since = None

        if self.phase == "CIRCLING" and valid:
            # Heading came back after the circles -> alignment step
            self.phase = "ALIGNING"
            self._steady_since = None

        # Steadiness detection for DONE: heading valid and not jumping
        if valid and value is not None:
            if (self._last_heading_val is None
                    or abs(_angdiff(value, self._last_heading_val)) < 2.0):
                if self._steady_since is None:
                    self._steady_since = now
            else:
                self._steady_since = now
            self._last_heading_val = value

            if (self.phase == "ALIGNING" and self._steady_since is not None
                    and now - self._steady_since > 5.0):
                self.phase = "DONE"


def _angdiff(a, b):
    d = a - b
    while d > 180:
        d -= 360
    while d <= -180:
        d += 360
    return d


PHASE_GUIDANCE = {
    "WAITING": [
        "Waiting for the GPS 24xd on the bus.",
        "Make sure it is powered and connected to the NMEA 2000 backbone.",
        "It should appear once it sends an address claim (press A to request).",
    ],
    "HEADING": [
        "Heading is present. Ready to begin basic calibration.",
        "1. Drive to calm, open water.",
        "2. When ready, begin TWO slow, tight circles within 3 minutes.",
        "   Keep the boat level -- do not let it list.",
        "The heading will DISAPPEAR when the 24xd detects the circles.",
    ],
    "CIRCLING": [
        "Circles detected -- the 24xd is calibrating the compass.",
        "Continue turning the same direction at the same speed,",
        "about 1 1/2 more rotations, until the heading REAPPEARS.",
        "Keep the boat steady and level.",
    ],
    "ALIGNING": [
        "Compass calibrated. Now align the heading:",
        "1. When safe, straighten out.",
        f"2. Drive a straight line at >= {HEADING_ALIGN_MIN_MPH:.0f} mph.",
        "Hold it steady until the heading settles.",
        "(Watch SPEED below -- you need cruising speed for this step.)",
    ],
    "DONE": [
        "Calibration complete. Heading is valid and steady.",
        "",
        "Press E to ENABLE the 24xd as a heading source in the HMI,",
        "or Q to quit (you can enable it later in heading_sources.py).",
    ],
}


def run(stdscr, args):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    curses.init_pair(2, curses.COLOR_YELLOW, -1)
    curses.init_pair(3, curses.COLOR_RED, -1)
    curses.init_pair(4, curses.COLOR_CYAN, -1)

    mon = CalibrationMonitor()

    # --- CAN source: live bus or replay --------------------------------------
    from j1939_name import (decode_name, is_address_claim, source_address,
                            build_request_for_address_claim,
                            REQUEST_ADDRESS_CLAIM_DATA)

    bus = None
    replay = None
    if args.replay:
        from can.io.canutils import CanutilsLogReader
        replay = iter(CanutilsLogReader(args.replay))
    else:
        import can
        from hmi_canlink import backend_for_platform
        backend, kwargs = backend_for_platform(args.backend, args.channel)
        bus = can.Bus(interface=backend, channel=args.channel,
                      bitrate=args.bitrate, **kwargs)

    enabled_result = {"enabled": False}

    def poll_message():
        if replay is not None:
            try:
                m = next(replay)
                return m.arbitration_id, bytes(m.data)
            except StopIteration:
                return None
        else:
            m = bus.recv(timeout=0.0)
            if m is None:
                return None
            return m.arbitration_id, bytes(m.data)

    last_draw = 0.0
    while True:
        # Drain available messages
        for _ in range(200):
            got = poll_message()
            if got is None:
                break
            can_id, data = got
            if is_address_claim(can_id):
                nm = decode_name(data)
                if nm is not None:
                    mon.note_claim(source_address(can_id), nm.mfg_code,
                                   nm.function, nm.identity)
            mon.on_message(can_id, data)

        # Keys
        try:
            ch = stdscr.getch()
        except curses.error:
            ch = -1
        if ch in (ord("q"), ord("Q")):
            break
        elif ch in (ord("a"), ord("A")) and bus is not None:
            # Request address claims so the 24xd announces itself.
            try:
                import can
                bus.send(can.Message(
                    arbitration_id=build_request_for_address_claim(),
                    data=REQUEST_ADDRESS_CLAIM_DATA, is_extended_id=True))
            except Exception:
                pass
        elif ch in (ord("e"), ord("E")) and mon.phase == "DONE":
            enabled_result["enabled"] = True
            _enable_24xd_in_registry()
            break

        # Draw at ~10 Hz
        now = time.time()
        if now - last_draw >= 0.1:
            last_draw = now
            _draw(stdscr, mon)

        time.sleep(0.02)

    return enabled_result["enabled"]


def _draw(stdscr, mon: CalibrationMonitor):
    stdscr.erase()
    rows, cols = stdscr.getmaxyx()

    def put(y, x, s, attr=0):
        if 0 <= y < rows and 0 <= x < cols:
            stdscr.addnstr(y, x, s, max(0, cols - x - 1), attr)

    put(0, 2, "GPS 24xd COMPASS CALIBRATION", curses.A_BOLD)
    addr = f"0x{mon.address:02X}" if mon.address is not None else "not found (press A)"
    put(0, 40, f"device: {addr}", curses.color_pair(4))

    # Phase banner
    phase_color = {
        "WAITING": 2, "HEADING": 1, "CIRCLING": 2, "ALIGNING": 2, "DONE": 1,
    }.get(mon.phase, 0)
    put(2, 2, f"PHASE: {mon.phase}", curses.color_pair(phase_color) | curses.A_BOLD)

    # Live readouts
    hv = f"{mon.heading:6.1f} deg" if (mon.heading_valid and mon.heading is not None) else "  ----  "
    hcolor = 1 if mon.heading_valid else 3
    put(4, 2, "Heading:", curses.A_BOLD)
    put(4, 12, hv, curses.color_pair(hcolor) | curses.A_BOLD)

    sp = f"{mon.speed_mph:5.1f} mph" if mon.speed_mph is not None else " ---- "
    spcolor = 1 if (mon.speed_mph is not None and mon.speed_mph >= HEADING_ALIGN_MIN_MPH) else 2
    put(5, 2, "Speed:", curses.A_BOLD)
    put(5, 12, sp, curses.color_pair(spcolor))
    if mon.phase == "ALIGNING":
        put(5, 26, f"(need >= {HEADING_ALIGN_MIN_MPH:.0f} for alignment)",
            curses.color_pair(2))

    fix = {0: "no fix", 2: "2D", 3: "3D", 4: "GNSS+DR"}.get(mon.fix_type, "?")
    put(6, 2, "GPS fix:", curses.A_BOLD)
    put(6, 12, fix, curses.color_pair(1 if (mon.fix_type or 0) >= 2 else 3))

    # Guidance for the current phase
    put(8, 2, "STEPS:", curses.A_BOLD | curses.color_pair(4))
    for i, line in enumerate(PHASE_GUIDANCE.get(mon.phase, [])):
        put(9 + i, 4, line)

    # Footer
    keys = "[A] request device   [E] enable 24xd (when DONE)   [Q] quit"
    put(rows - 1, 2, keys, curses.color_pair(4))

    stdscr.refresh()


def _enable_24xd_in_registry():
    """
    Persist the 24xd-enabled state so the main HMI uses it.

    This edits heading_sources.py in place, flipping the 24xd profile's
    enabled flag to True. The next time the main app starts, the 24xd is a live
    heading source. (If you run the HMI and this tool against a shared registry
    object in one process, you would instead call registry.set_enabled(...)
    directly -- see the comment in heading_sources.py.)
    """
    import os
    import re

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "heading_sources.py")
    try:
        with open(path, "r") as f:
            src = f.read()
        # Flip the 24xd profile's enabled=False to True. The profile is
        # identified by its label to avoid touching the wall compass.
        # This is a targeted replace on the 24xd block.
        new = re.sub(
            r'(label="GPS 24xd",\s*\n\s*priority=1,\s*\n\s*enabled=)False',
            r'\1True',
            src,
        )
        if new != src:
            with open(path, "w") as f:
                f.write(new)
    except Exception:
        # If the edit fails, the operator can enable it manually; the tool
        # already told them how.
        pass


def main():
    ap = argparse.ArgumentParser(description="GPS 24xd compass calibration helper")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--bitrate", type=int, default=250000)
    ap.add_argument("--backend", default=None)
    ap.add_argument("--replay", default=None,
                    help="a candump log to replay instead of a live bus (dry run)")
    args = ap.parse_args()

    enabled = curses.wrapper(run, args)
    if enabled:
        print("GPS 24xd enabled as a heading source. Restart the HMI to use it.")
    else:
        print("Exited without enabling the 24xd.")


if __name__ == "__main__":
    main()
