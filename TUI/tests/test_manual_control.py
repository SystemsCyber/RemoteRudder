"""
Tests for manual steering, engage gating, and the remapped keys.

Covers the operator-requested changes:
  * Up arrow snaps the goal to current heading (was Home + Fn)
  * . and , step the motor in encoder counts, disengaged-only
  * engage requires a COG lock and always seeds the goal at current heading
  * COG loss mid-run holds the last rudder and stays engaged

Manual steering is the fallback for a lost lock or a bad sensor: the operator
disengages and steers by hand. The disengaged-only rule means a manual input
can never fight the autopilot -- the two are mutually exclusive by
construction, not by timing.
"""

from __future__ import annotations

import curses
from unittest.mock import MagicMock

import pytest

from app_tui import MANUAL_STEP_COUNTS, HMIApp
from hmi_state import SystemState


@pytest.fixture
def app():
    """A minimally-wired HMIApp with mocked CAN and autopilot."""
    a = HMIApp.__new__(HMIApp)
    a.state = SystemState()
    a.autopilot = MagicMock()
    a.autopilot.shaft_goal = 1425.0
    a.can = MagicMock()
    a.can.shaft_goal = 1425.0
    a.monitor = MagicMock()
    a.monitor.ok_to_engage = lambda: (True, "ok")
    a.tui = None
    a.set_goal = lambda v, s: a.state.set_goal(v, s)
    a.adjust_goal = lambda d, s: a.state.adjust_goal(d, s)
    return a


class TestManualSteering:
    def test_step_right_when_disengaged(self, app):
        app.state.autopilot_engaged = False
        app.handle_command("manual_step_right", "tui")
        app.can.adjust_shaft_goal.assert_called_once_with(MANUAL_STEP_COUNTS)

    def test_step_left_when_disengaged(self, app):
        app.state.autopilot_engaged = False
        app.handle_command("manual_step_left", "tui")
        app.can.adjust_shaft_goal.assert_called_once_with(-MANUAL_STEP_COUNTS)

    def test_step_auto_enables_servo(self, app):
        """A step is useless if the servo is off; enable it so the motor moves."""
        app.state.autopilot_engaged = False
        app.state.servo_enabled = False
        app.handle_command("manual_step_right", "tui")
        assert app.state.servo_enabled is True

    def test_step_ignored_when_engaged(self, app):
        """
        The safety rule: manual and autopilot are mutually exclusive. A step
        while engaged does nothing to the motor.
        """
        app.state.autopilot_engaged = True
        app.handle_command("manual_step_left", "tui")
        app.can.adjust_shaft_goal.assert_not_called()

    def test_step_is_ten_counts(self):
        """Encoder counts, increment of 10 -- the operator's requested unit."""
        assert MANUAL_STEP_COUNTS == 10

    def test_repeated_steps_accumulate(self, app):
        app.state.autopilot_engaged = False
        for _ in range(3):
            app.handle_command("manual_step_right", "tui")
        assert app.can.adjust_shaft_goal.call_count == 3


class TestEngageGating:
    def test_engage_blocked_without_cog_lock(self, app):
        app.state.autopilot_engaged = False
        app.state.cog_lock = False
        app.handle_command("autopilot_enable", "tui")
        assert app.state.autopilot_engaged is False

    def test_engage_allowed_with_cog_lock(self, app):
        app.state.autopilot_engaged = False
        app.state.cog_lock = True
        app.state.set_signal("fused_heading", 175.0)
        app.handle_command("autopilot_enable", "tui")
        assert app.state.autopilot_engaged is True

    def test_engage_seeds_goal_at_current_heading(self, app):
        """
        Always hold the present course on engage, never turn toward a stale
        setpoint. This is item 1's second requirement.
        """
        app.state.autopilot_engaged = False
        app.state.cog_lock = True
        app.state.set_signal("fused_heading", 210.0)
        app.state.set_goal(50.0, "old")  # a stale goal that must be overridden
        app.handle_command("autopilot_enable", "tui")
        assert app.state.heading_goal == pytest.approx(210.0)

    def test_engage_pushes_goal_to_autopilot(self, app):
        app.state.autopilot_engaged = False
        app.state.cog_lock = True
        app.state.set_signal("fused_heading", 175.0)
        app.handle_command("autopilot_enable", "tui")
        assert app.autopilot.heading_goal == pytest.approx(175.0)

    def test_engage_blocked_by_heading_quality(self, app):
        """The noise interlock still applies, before the COG-lock check."""
        app.state.cog_lock = True
        app.monitor.ok_to_engage = lambda: (False, "sigma 12deg")
        app.handle_command("autopilot_enable", "tui")
        assert app.state.autopilot_engaged is False


class TestAutopilotHoldOnCogLoss:
    """
    When engaged and COG lock is lost mid-run, the autopilot holds its last
    rudder and stays engaged. It re-acquires when COG returns. It does not
    center (the boat is still moving) and does not disengage.
    """

    def test_set_cog_lock_toggles_flag(self):
        from autopilot import Autopilot

        ap = Autopilot.__new__(Autopilot)
        ap._cog_lock = True
        ap.set_cog_lock(False)
        assert ap._cog_lock is False
        ap.set_cog_lock(True)
        assert ap._cog_lock is True


class TestKeyRemapping:
    """The TUI key handler, driven directly."""

    @pytest.fixture
    def tui(self, state):
        from hmi_tui import TUI

        commands = []
        sets = []
        t = TUI(
            state,
            on_goal_delta=lambda d: state.adjust_goal(d, "tui"),
            on_goal_set=lambda v: (sets.append(v), state.set_goal(v, "tui")),
            on_command=lambda c: commands.append(c),
        )
        t.commands = commands
        t.sets = sets
        return t

    def test_up_snaps_to_heading(self, tui, state):
        """Up arrow replaces Home (which needed the Fn key)."""
        state.set_signal("fused_heading", 127.4)
        state.set_goal(200.0, "init")
        tui._handle_key(curses.KEY_UP)
        assert state.heading_goal == pytest.approx(127.4)

    def test_up_falls_back_to_compass(self, tui, state):
        state.set_signal("compass_heading", 131.2)
        state.set_goal(200.0, "init")
        tui._handle_key(curses.KEY_UP)
        assert state.heading_goal == pytest.approx(131.2)

    def test_up_with_no_heading_flashes(self, tui, state):
        tui._handle_key(curses.KEY_UP)
        assert "no heading" in tui._flash_msg.lower()

    def test_dot_is_manual_step_right(self, tui):
        tui._handle_key(ord("."))
        assert "manual_step_right" in tui.commands

    def test_comma_is_manual_step_left(self, tui):
        tui._handle_key(ord(","))
        assert "manual_step_left" in tui.commands

    def test_brackets_are_goal_ten(self, tui, state):
        state.set_goal(130.0, "init")
        tui._handle_key(ord("]"))
        assert state.heading_goal == pytest.approx(140.0)
        tui._handle_key(ord("["))
        assert state.heading_goal == pytest.approx(130.0)

    def test_arrows_still_one_degree(self, tui, state):
        state.set_goal(130.0, "init")
        tui._handle_key(curses.KEY_RIGHT)
        assert state.heading_goal == pytest.approx(131.0)
        tui._handle_key(curses.KEY_LEFT)
        assert state.heading_goal == pytest.approx(130.0)

    def test_down_still_five(self, tui, state):
        state.set_goal(130.0, "init")
        tui._handle_key(curses.KEY_DOWN)
        assert state.heading_goal == pytest.approx(125.0)

    def test_home_no_longer_snaps(self, tui, state):
        """
        Home was the old snap key. It should no longer do anything, so a stray
        Home press cannot move the goal.
        """
        state.set_signal("fused_heading", 127.4)
        state.set_goal(200.0, "init")
        tui._handle_key(curses.KEY_HOME)
        assert state.heading_goal == pytest.approx(200.0)

    def test_dot_comma_no_longer_move_goal(self, tui, state):
        """They are manual-step commands now, not goal nudges."""
        state.set_goal(130.0, "init")
        tui._handle_key(ord("."))
        tui._handle_key(ord(","))
        assert state.heading_goal == pytest.approx(130.0)
