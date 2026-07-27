"""
Tests for the autopilot PID, focused on the operator's requirements:

  * The integral is a TRUE accumulator: on a stationary boat chasing a goal it
    cannot reach, the integral winds up over time and the shaft command LIMITS
    OUT (pins at its travel stop). The old code reset the integral to 0 every
    cycle, so it never wound up -- that was the bug.
  * Asking for more turn accumulates more steering in that direction.
  * The integral is clamped so overshoot is bounded when the error clears.
  * Disengaging resets the integral (no inherited windup).
"""

from __future__ import annotations

import pytest


@pytest.fixture
def autopilot():
    from can_interface import CANinterface
    from autopilot import Autopilot
    from app_tui import HMIApp
    ci = CANinterface.__new__(CANinterface)
    HMIApp._init_can_fields(ci)
    ci.bus = None
    ap = Autopilot(ci)
    ap.shaft_center = 1425
    return ap


def _spin(ap, error, cycles, dt=0.1):
    """Run the PID `cycles` times at a fixed error and simulated dt."""
    last = None
    for _ in range(cycles):
        ap.heading_error = error
        last = ap.compute_rudder_command()
        ap._last_time -= dt  # advance the simulated clock
    return last


class TestIntegralAccumulates:
    def test_integral_grows_with_persistent_error(self, autopilot):
        """The core fix: a persistent error makes the integral WIND UP."""
        i0 = autopilot._integral
        _spin(autopilot, error=5.0, cycles=20)
        assert autopilot._integral > i0 + 5, \
            "integral did not accumulate on a persistent error"

    def test_integral_not_reset_each_cycle(self, autopilot):
        """Regression: the old code set self._integral = 0 every call."""
        _spin(autopilot, error=3.0, cycles=10)
        first = autopilot._integral
        _spin(autopilot, error=3.0, cycles=10)
        assert autopilot._integral > first, \
            "integral was reset between cycles (the old bug)"

    def test_more_error_more_steering(self, autopilot):
        """More turn requested -> more accumulated steering in that direction."""
        _spin(autopilot, error=10.0, cycles=20)
        right_integral = autopilot._integral
        assert right_integral > 0  # positive error -> positive integral
        # opposite direction accumulates the other way
        autopilot._integral = 0.0
        _spin(autopilot, error=-10.0, cycles=20)
        assert autopilot._integral < 0


class TestShaftLimitsOut:
    def test_stationary_unreachable_goal_limits_shaft(self, autopilot):
        """
        Stationary boat, big persistent error it cannot satisfy: the shaft
        command must pin at its travel limit (limit out).
        """
        cmd = _spin(autopilot, error=90.0, cycles=40)
        assert cmd >= autopilot.shaft_out_max - 1, \
            f"shaft did not limit out (cmd={cmd}, max={autopilot.shaft_out_max})"

    def test_negative_error_limits_low(self, autopilot):
        cmd = _spin(autopilot, error=-90.0, cycles=40)
        assert cmd <= autopilot.shaft_out_min + 1, \
            f"shaft did not limit out low (cmd={cmd})"

    def test_output_clamped_to_shaft_travel(self, autopilot):
        """The output never exceeds the physical shaft range."""
        for err in (200.0, -200.0, 45.0, -45.0):
            autopilot._integral = 0.0
            cmd = _spin(autopilot, error=err, cycles=30)
            assert autopilot.shaft_out_min <= cmd <= autopilot.shaft_out_max


class TestIntegralClamp:
    def test_integral_is_bounded(self, autopilot):
        """A long persistent error must not wind the integral to infinity."""
        _spin(autopilot, error=90.0, cycles=500)
        assert abs(autopilot._integral) <= autopilot._integral_max + 1, \
            "integral exceeded its clamp -- overshoot would be unbounded"

    def test_clamp_bounds_residual_authority(self, autopilot):
        """Ki * integral is bounded by the configured authority."""
        _spin(autopilot, error=90.0, cycles=500)
        authority = abs(autopilot.Ki * autopilot._integral)
        assert authority <= autopilot.integral_authority_max + 1


class TestIntegralReset:
    def test_reset_integral_clears(self, autopilot):
        _spin(autopilot, error=10.0, cycles=20)
        assert autopilot._integral != 0
        autopilot.reset_integral()
        assert autopilot._integral == 0.0


class TestPidPanelAndTuning:
    """The TUI PID panel shows live PID state and the tuning keys adjust gains."""

    def _render_pid(self, cols=155, selected="Kp"):
        import os, pty, curses, struct, fcntl, termios
        m, s = pty.openpty()
        fcntl.ioctl(s, termios.TIOCSWINSZ, struct.pack("HHHH", 30, cols, 0, 0))
        os.environ["TERM"] = "xterm-256color"
        si, so = os.dup(0), os.dup(1)
        os.dup2(s, 0); os.dup2(s, 1)
        from hmi_state import SystemState
        from hmi_tui import TUI
        scr = curses.initscr(); curses.noecho(); curses.cbreak()
        try:
            curses.resize_term(30, cols); scr.resize(30, cols)
            st = SystemState()
            t = TUI(st, lambda d: None, lambda v: None, lambda c: None)
            t.start(scr)
            st.link_up = True
            st.set_signal("pid_kp", 30.0)
            st.set_signal("pid_ki", 0.5)
            st.set_signal("pid_kd", 5.0)
            st.set_signal("pid_p", 150.0)
            st.set_signal("pid_i", 23.0)
            st.set_signal("pid_d", -8.0)
            st.set_signal("pid_integral", 46.0)
            t._pid_selected = selected
            t.render()
            rows, c = scr.getmaxyx()
            return "\n".join(
                scr.instr(y, 0).decode("utf-8", "replace") for y in range(rows))
        finally:
            curses.nocbreak(); curses.echo(); curses.endwin()
            os.dup2(si, 0); os.dup2(so, 1)

    def test_panel_shows_gains(self):
        out = self._render_pid()
        assert "PID" in out
        assert "Kp" in out and "Ki" in out and "Kd" in out
        assert "30.00" in out  # Kp value
        assert "0.50" in out   # Ki value

    def test_panel_shows_term_contributions(self):
        out = self._render_pid()
        # P/I/D term contributions in shaft counts
        assert "+150" in out   # p_term
        assert "+23" in out    # i_term

    def test_selected_gain_is_marked(self):
        out = self._render_pid(selected="Ki")
        # the selected gain gets a '>' marker
        assert ">Ki" in out

    def test_gain_command_adjusts_autopilot(self):
        """The pid_gain command path actually changes the gain."""
        from can_interface import CANinterface
        from autopilot import Autopilot
        from app_tui import HMIApp
        ci = CANinterface.__new__(CANinterface)
        HMIApp._init_can_fields(ci)
        ci.bus = None
        ap = Autopilot(ci)
        kp0 = ap.Kp
        ap.adjust_gain("Kp", ap.kp_step)
        assert ap.Kp == kp0 + ap.kp_step
        # Ki change recomputes the integral clamp
        imax0 = ap._integral_max
        ap.adjust_gain("Ki", ap.ki_step)
        assert ap._integral_max != imax0


class TestRunLoopDoesNotCrash:
    """
    Regression: the run() THREAD (not just compute_rudder_command) must not
    crash. _disengaged_goal_ref was used in run()'s disengaged branch but never
    initialized in __init__, so the thread died with AttributeError on the first
    disengaged tick. These tests exercise the real loop, which the
    compute-only tests missed.
    """

    def _autopilot(self):
        from can_interface import CANinterface
        from autopilot import Autopilot
        from app_tui import HMIApp
        from unittest import mock
        ci = CANinterface.__new__(CANinterface)
        HMIApp._init_can_fields(ci)
        # A mock bus so broadcast_status_message / send_shaft_goal don't hit a
        # real CAN interface. On the boat the bus is real; here we just need the
        # run loop to execute without a NoneType bus error.
        ci.bus = mock.MagicMock()
        ap = Autopilot(ci)
        return ap

    def test_disengaged_goal_ref_initialized(self):
        ap = self._autopilot()
        # the attribute the run loop reads must exist from construction
        assert hasattr(ap, "_disengaged_goal_ref")

    def test_run_loop_survives_disengaged_ticks(self):
        """Start the real thread, let it tick disengaged, confirm it stays alive."""
        import threading
        import time

        ap = self._autopilot()
        ap.current_heading = 179.4
        ap.autopilot_engaged_event.clear()  # disengaged path

        errors = []
        orig_run = ap.run

        def guarded_run():
            try:
                orig_run()
            except Exception as e:  # noqa: BLE001
                errors.append(e)

        ap._run_thread = threading.Thread(target=guarded_run, daemon=True)
        ap._run_thread.start()
        time.sleep(0.4)  # several 0.1s ticks through the disengaged branch
        ap.stop(timeout=2.0)

        assert not errors, f"run loop raised: {errors}"

    def test_run_loop_survives_engaged_ticks(self):
        import threading
        import time

        ap = self._autopilot()
        ap.current_heading = 179.4
        ap.heading_goal = 185.0
        ap.autopilot_engaged_event.set()  # engaged path
        ap._cog_lock = True

        errors = []
        orig_run = ap.run

        def guarded_run():
            try:
                orig_run()
            except Exception as e:  # noqa: BLE001
                errors.append(e)

        ap._run_thread = threading.Thread(target=guarded_run, daemon=True)
        ap._run_thread.start()
        time.sleep(0.4)
        ap.stop(timeout=2.0)

        assert not errors, f"run loop raised while engaged: {errors}"
