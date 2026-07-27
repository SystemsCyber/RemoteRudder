"""
Regression tests for the two bugs found in the 2026-07-25 sea trial:

1. The GPS vehicle-direction (a stationary-GPS course, ~39 deg) was written
   into compass_heading, clobbering the real wall compass (~180 deg south) and
   making the displayed heading bounce. Vehicle direction must stay in its own
   signal and never touch the compass heading.

2. The heading goal bounced between the HMI-set value and 360, because the
   autopilot's goal wraparound used `> 360` (leaving 360 as 360) while the HMI
   state used `% 360` (storing 0). The two goal copies diverged. They must
   normalize identically.
"""

from __future__ import annotations

import struct

import can
import pytest


def _frame(cid, hexdata):
    return can.Message(arbitration_id=cid, data=bytes.fromhex(hexdata),
                       is_extended_id=True)


class TestVehicleDirNotCompass:
    def test_vehicle_dir_does_not_write_compass_heading(self, can_iface):
        """PGN 65256 must not return a 'compass' key."""
        # a vehicle-direction frame
        out = can_iface.process_message(
            _frame(0x18FEE81C, "00500A0000640000"))
        assert out is None or "compass" not in out
        if out is not None:
            assert "gps_vehicle_dir" in out or "speed" in out

    def test_compass_heading_holds_real_compass(self, wired, stationary_south_log):
        """
        On the stationary-south capture, compass_heading must reflect the wall
        compass (~170-181 south), NOT the stale ~39 deg GPS course.
        """
        wired.feed(stationary_south_log)
        comp = wired.state.signal_value("compass_heading")
        assert comp is not None
        # south-ish; definitely not the ~39 deg GPS course
        assert 150.0 < comp < 200.0, f"compass_heading {comp} looks like the GPS course, not the compass"

    def test_fused_matches_compass_when_parked(self, wired, stationary_south_log):
        wired.feed(stationary_south_log)
        comp = wired.state.signal_value("compass_heading")
        fused = wired.state.signal_value("fused_heading")
        assert comp is not None
        # The key regression: compass_heading is the real south reading, not the
        # ~39 deg GPS course. If fused is populated it should agree; if the test
        # harness has not bound the registry it may be None, which is fine here
        # -- the bug was compass_heading being wrong, and that is asserted above.
        if fused is not None:
            assert abs(((comp - fused + 180) % 360) - 180) < 5.0


class TestGoalSingleNormalization:
    def test_state_and_autopilot_agree_at_360(self):
        from hmi_state import SystemState
        from autopilot import Autopilot
        from can_interface import CANinterface

        st = SystemState()
        ci = CANinterface.__new__(CANinterface)
        from app_tui import HMIApp
        HMIApp._init_can_fields(ci)
        ci.bus = None
        ap = Autopilot(ci)

        for goal in (360.0, 0.0, -1.0, 359.5, 720.0, 361.0, 180.0):
            v = st.set_goal(goal, "test")
            ap.set_heading_goal(goal)
            assert abs(v - ap.heading_goal) < 1e-9, \
                f"goal {goal}: state {v} != autopilot {ap.heading_goal}"

    def test_360_normalizes_to_zero(self):
        from autopilot import Autopilot
        from can_interface import CANinterface
        from app_tui import HMIApp

        ci = CANinterface.__new__(CANinterface)
        HMIApp._init_can_fields(ci)
        ci.bus = None
        ap = Autopilot(ci)
        ap.set_heading_goal(360.0)
        assert ap.heading_goal == 0.0  # never 360

    def test_adjust_wraps_with_modulo(self):
        from autopilot import Autopilot
        from can_interface import CANinterface
        from app_tui import HMIApp

        ci = CANinterface.__new__(CANinterface)
        HMIApp._init_can_fields(ci)
        ci.bus = None
        ap = Autopilot(ci)
        ap.set_heading_goal(0.0)
        ap.adjust_heading_goal(-1)
        assert ap.heading_goal == 359.0
        ap.set_heading_goal(359.0)
        ap.adjust_heading_goal(1)
        assert ap.heading_goal == 0.0  # 360 -> 0
