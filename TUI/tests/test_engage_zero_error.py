"""
Engaging the autopilot must start from ZERO error: the goal is seeded to the
current, accurate heading, and the controller's current_heading is pinned to the
same value, so the first PID cycle sees error = 0 and the boat holds its present
course. Engage must also refuse if no FRESH heading is available (so the goal is
never seeded from a stale/inaccurate heading).
"""

from __future__ import annotations

import time
from unittest import mock

import pytest


class _App:
    """Minimal stand-in wiring state + bridge + autopilot like app_tui."""

    def __init__(self):
        from hmi_state import SystemState
        from hmi_heading import HeadingMonitor
        from hmi_bridge import Bridge
        from can_interface import CANinterface
        from autopilot import Autopilot
        from app_tui import HMIApp

        self.state = SystemState()
        self.monitor = HeadingMonitor(self.state)
        self.bridge = Bridge(self.state, self.monitor)
        self.bridge.use_ekf = True
        self.can = CANinterface.__new__(CANinterface)
        HMIApp._init_can_fields(self.can)
        self.can.bus = None
        self.autopilot = Autopilot(self.can)
        self.tui = None
        self.bridge.heading_registry.note_claim(0xF8, 229, 140, 1039212)
        # bind the real handle_command / set_goal from HMIApp
        self.handle_command = HMIApp.handle_command.__get__(self)
        self.set_goal = HMIApp.set_goal.__get__(self)

    def feed_heading(self, deg, n=30):
        for _ in range(n):
            self.bridge.on_decoded({"compass_heading": deg, "heading_sa": 0xF8,
                                    "heading_value": deg})
            self.state.set_signal("compass_heading", deg)
            self.bridge.step_ekf()
            self.bridge.derive_fused()
            time.sleep(0.002)


@pytest.fixture
def app():
    return _App()


def test_engage_starts_from_zero_error(app):
    app.feed_heading(179.4)
    app.state.cog_lock = True
    with mock.patch.object(app.monitor, "ok_to_engage", return_value=(True, "ok")):
        app.handle_command("autopilot_enable", source="test")
    ap = app.autopilot
    assert ap.autopilot_engaged
    assert abs(ap.heading_error) < 0.001, "engage did not start from zero error"
    assert abs(ap.heading_goal - ap.current_heading) < 0.001


def test_engage_seeds_goal_to_current_heading(app):
    app.feed_heading(88.0)
    app.state.cog_lock = True
    with mock.patch.object(app.monitor, "ok_to_engage", return_value=(True, "ok")):
        app.handle_command("autopilot_enable", source="test")
    # goal should be the heading we were holding, not a stale value
    assert abs(((app.autopilot.heading_goal - 88.0 + 180) % 360) - 180) < 1.0


def test_engage_clears_integral(app):
    app.feed_heading(179.4)
    app.autopilot._integral = 500.0  # pretend windup from before
    app.state.cog_lock = True
    with mock.patch.object(app.monitor, "ok_to_engage", return_value=(True, "ok")):
        app.handle_command("autopilot_enable", source="test")
    assert app.autopilot._integral == 0.0, "engage did not clear integral windup"


def test_engage_refuses_stale_heading(app):
    # a stale fused heading must not be used to seed the goal
    app.state.set_signal("fused_heading", 100.0)
    sig = app.state.get_signal("fused_heading")
    sig.last_rx = time.time() - 10  # make it stale
    app.state.cog_lock = True
    with mock.patch.object(app.monitor, "ok_to_engage", return_value=(True, "ok")):
        app.handle_command("autopilot_enable", source="test")
    assert not app.autopilot.autopilot_engaged, \
        "engaged from a stale heading -- goal would not be zero-error"
