"""
Tests for the touch button bar and the wide (120-col) layout.

The operator has a touchscreen, so the TUI grows a button bar whose targets
map to the same commands as the keys. Buttons reflect state -- engage is
disabled without a COG lock, manual steps are disabled while engaged -- so the
touchscreen cannot do something the keyboard would refuse.

Button geometry is tested by driving _on_touch with coordinates taken from the
recorded button spans, which is how a real tap is dispatched. Layout is tested
by rendering into a pty at 120 and 80 columns.
"""

from __future__ import annotations

import curses
import os
import pty
import struct as _struct
import time

import pytest

from hmi_state import SystemState
from hmi_tui import MIN_COLS, MIN_ROWS, TUI


@pytest.fixture
def tui(state):
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


# ---------------------------------------------------------------------------
# Button geometry and dispatch (no screen needed)
# ---------------------------------------------------------------------------


class TestButtonDispatch:
    def _build_buttons(self, tui, cols=120):
        """Populate _buttons the way render does, without a full screen."""
        tui._buttons = []
        # _draw_buttons writes to the screen; give it a stub that ignores puts.
        tui._put = lambda *a, **k: None
        tui._draw_buttons(20, cols)
        return tui._buttons

    def test_buttons_are_registered(self, tui, state):
        buttons = self._build_buttons(tui)
        assert len(buttons) >= 6
        cmds = {b[3] for b in buttons}
        assert "manual_step_left" in cmds
        assert "autopilot_enable" in cmds
        assert "quit" in cmds

    def test_tap_snap_sets_goal(self, tui, state):
        state.set_signal("fused_heading", 143.0)
        state.set_goal(200.0, "init")
        self._build_buttons(tui)
        # find the snap button and tap its center
        y, x0, x1, cmd, en = next(b for b in tui._buttons if b[3] == "snap_goal")
        tui._on_touch(y, (x0 + x1) // 2)
        assert state.heading_goal == pytest.approx(143.0)

    def test_tap_engage_when_locked(self, tui, state):
        state.cog_lock = True
        state.autopilot_engaged = False
        self._build_buttons(tui)
        y, x0, x1, cmd, en = next(b for b in tui._buttons if b[3] == "autopilot_enable")
        assert en is True
        tui._on_touch(y, (x0 + x1) // 2)
        assert "autopilot_enable" in tui.commands

    def test_engage_button_disabled_without_lock(self, tui, state):
        state.cog_lock = False
        state.autopilot_engaged = False
        self._build_buttons(tui)
        y, x0, x1, cmd, en = next(b for b in tui._buttons if b[3] == "autopilot_enable")
        assert en is False

    def test_tapping_disabled_engage_does_not_engage(self, tui, state):
        state.cog_lock = False
        state.autopilot_engaged = False
        self._build_buttons(tui)
        y, x0, x1, cmd, en = next(b for b in tui._buttons if b[3] == "autopilot_enable")
        tui._on_touch(y, (x0 + x1) // 2)
        assert "autopilot_enable" not in tui.commands
        assert tui._flash_msg  # gave feedback rather than silently ignoring

    def test_manual_step_buttons_disabled_when_engaged(self, tui, state):
        state.autopilot_engaged = True
        self._build_buttons(tui)
        for b in tui._buttons:
            if b[3] in ("manual_step_left", "manual_step_right"):
                assert b[4] is False, f"{b[3]} should be disabled while engaged"

    def test_manual_step_buttons_enabled_when_disengaged(self, tui, state):
        state.autopilot_engaged = False
        self._build_buttons(tui)
        for b in tui._buttons:
            if b[3] in ("manual_step_left", "manual_step_right"):
                assert b[4] is True

    def test_disengage_disabled_when_not_engaged(self, tui, state):
        state.autopilot_engaged = False
        self._build_buttons(tui)
        b = next(b for b in tui._buttons if b[3] == "autopilot_disable")
        assert b[4] is False

    def test_tap_outside_any_button_does_nothing(self, tui, state):
        state.set_goal(130.0, "init")
        self._build_buttons(tui)
        tui._on_touch(0, 500)  # far off any button
        assert tui.commands == []
        assert state.heading_goal == pytest.approx(130.0)

    def test_tap_wrong_row_does_nothing(self, tui, state):
        self._build_buttons(tui)
        b = tui._buttons[0]
        tui._on_touch(b[0] + 5, (b[1] + b[2]) // 2)  # right column, wrong row
        assert tui.commands == []

    def test_servo_toggle_button(self, tui, state):
        state.servo_enabled = False
        self._build_buttons(tui)
        b = next(b for b in tui._buttons if b[3] == "servo_toggle")
        tui._on_touch(b[0], (b[1] + b[2]) // 2)
        assert "servo_enable" in tui.commands


# ---------------------------------------------------------------------------
# Layout rendering (pty)
# ---------------------------------------------------------------------------


@pytest.fixture
def render_at():
    """
    Return a function that renders the populated TUI at a given size and gives
    back the screen text. Uses a pty with stdio redirected, the pattern the
    other render tests use.
    """

    def _render(rows, cols):
        master, slave = pty.openpty()
        try:
            import fcntl
            import termios

            fcntl.ioctl(slave, termios.TIOCSWINSZ,
                        _struct.pack("HHHH", rows, cols, 0, 0))
        except Exception:
            pass

        old_term = os.environ.get("TERM")
        old_lines = os.environ.get("LINES")
        old_cols = os.environ.get("COLUMNS")
        os.environ["TERM"] = "xterm-256color"
        os.environ["LINES"] = str(rows)
        os.environ["COLUMNS"] = str(cols)

        saved_in, saved_out = os.dup(0), os.dup(1)
        os.dup2(slave, 0)
        os.dup2(slave, 1)

        stdscr = None
        text = None
        try:
            try:
                stdscr = curses.initscr()
                curses.noecho()
                curses.cbreak()
                stdscr.keypad(True)
            except Exception as e:  # noqa: BLE001
                pytest.skip(f"curses unavailable: {e}")

            try:
                curses.resize_term(rows, cols)
                stdscr.resize(rows, cols)
            except Exception:
                pass
            got_r, got_c = stdscr.getmaxyx()
            if got_r < rows or got_c < cols:
                pytest.skip(f"could not size screen to {cols}x{rows}")

            import hmi_bridge

            st = SystemState()
            t = TUI(st, lambda d: None, lambda v: None, lambda c: None)
            t.start(stdscr)
            st.link_up = True
            st.backend = "socketcan"
            st.channel = "can0"
            st.bitrate = 250000
            st.rx_total = 48213
            st.heading_source = "COG"
            st.cog_lock = True
            now = time.time()
            for k, v in (
                ("compass_heading", 264.0), ("cog", 196.4), ("sog", 22.6),
                ("fused_heading", 196.4), ("heading_sigma", 2.1),
                ("steering_angle", 1447), ("steering_goal", 1425),
                ("shaft_center", 1425), ("rudder_angle", -3.2), ("rpm", 3200),
            ):
                st.set_signal(k, v)
            st.autopilot_engaged = True
            st.servo_enabled = True
            st.cog_accepted = 1137
            st.cog_rejected = 503
            for cid in hmi_bridge.WATCHED:
                st.register_source(cid, hmi_bridge.WATCHED[cid][0],
                                   hmi_bridge.WATCHED[cid][1])
                st.mark_source(cid, now)
            t.render()
            rr, cc = stdscr.getmaxyx()
            text = "\n".join(
                stdscr.instr(y, 0).decode("utf-8", "replace").rstrip()
                for y in range(rr)
            )
        finally:
            if stdscr is not None:
                try:
                    curses.nocbreak()
                    curses.echo()
                    curses.endwin()
                except Exception:
                    pass
            os.dup2(saved_in, 0)
            os.dup2(saved_out, 1)
            os.close(saved_in)
            os.close(saved_out)
            for key, val in (("TERM", old_term), ("LINES", old_lines),
                             ("COLUMNS", old_cols)):
                if val is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = val
            os.close(master)
            os.close(slave)
        return text

    return _render


@pytest.mark.integration
class TestWideLayout:
    def test_status_panel_shows_at_120(self, render_at):
        text = render_at(24, 120)
        assert "STATUS" in text
        assert "HEADING SRC" in text

    def test_cog_lock_shown_at_120(self, render_at):
        text = render_at(24, 120)
        assert "COG LOCK" in text

    def test_buttons_render_at_120(self, render_at):
        text = render_at(24, 120)
        assert "STEP" in text
        assert "SNAP" in text
        assert "QUIT" in text

    def test_no_status_panel_at_80(self, render_at):
        """The narrow layout drops the status panel rather than overflowing."""
        text = render_at(24, 80)
        assert "STATUS" not in text or "HEADING SRC" not in text

    def test_indicators_still_shown_at_80(self, render_at):
        """Engage/servo indicators live in the rudder panel when narrow."""
        text = render_at(24, 80)
        assert "AUTOPILOT" in text

    def test_no_extra_blank_lines_before_buttons(self, render_at):
        """
        The operator reported extra lines. The events area should size to fill
        the gap so the button bar sits just above the footer, not floating
        with blank rows above it.
        """
        text = render_at(24, 120)
        lines = text.split("\n")
        # find the button bar row and the footer row
        btn_idx = next(i for i, ln in enumerate(lines) if "STEP" in ln and "QUIT" in ln)
        foot_idx = next(i for i, ln in enumerate(lines) if "[q]uit" in ln)
        assert foot_idx == btn_idx + 1, "footer should sit directly under buttons"

    def test_footer_reflects_new_keys(self, render_at):
        text = render_at(24, 120)
        # up snaps, brackets are +-10, comma/dot step
        assert "snap" in text.lower()
        assert "step" in text.lower()

    def test_quit_not_truncated(self, render_at):
        text = render_at(24, 120)
        assert "[q]uit" in text
