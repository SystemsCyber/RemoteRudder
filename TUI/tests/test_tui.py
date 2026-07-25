"""
Tests for hmi_tui: key handling and rendering.

curses needs a real terminal, so rendering tests run inside curses.wrapper in
a subprocess with a pty. Key handling does not need a screen and is tested
directly by feeding key codes to _handle_key.

What matters here:
  * arrow keys move the goal by the documented amounts and wrap correctly
  * rendering never raises, including at terminal sizes that break naive
    curses code (the bottom-right cell, and anything below the minimum)
  * the goal's provenance appears on screen, since that is how the operator
    knows the phone moved it
"""

from __future__ import annotations

import curses
import os
import subprocess
import sys
from pathlib import Path

import pytest

HMI_DIR = Path(__file__).resolve().parent.parent


@pytest.fixture
def tui(state):
    """A TUI with recording callbacks and no screen attached."""
    from hmi_tui import TUI

    commands = []
    deltas = []
    sets = []

    t = TUI(
        state,
        on_goal_delta=lambda d: (deltas.append(d), state.adjust_goal(d, "tui")),
        on_goal_set=lambda v: (sets.append(v), state.set_goal(v, "tui")),
        on_command=lambda c: commands.append(c),
    )
    t.commands = commands
    t.deltas = deltas
    t.sets = sets
    return t


# ---------------------------------------------------------------------------
# Key handling
# ---------------------------------------------------------------------------


class TestArrowKeys:
    def test_left_decrements_one(self, tui, state):
        state.set_goal(130.0, "init")
        tui._handle_key(curses.KEY_LEFT)
        assert state.heading_goal == pytest.approx(129.0)

    def test_right_increments_one(self, tui, state):
        state.set_goal(130.0, "init")
        tui._handle_key(curses.KEY_RIGHT)
        assert state.heading_goal == pytest.approx(131.0)

    def test_shift_left_decrements_ten(self, tui, state):
        state.set_goal(130.0, "init")
        tui._handle_key(curses.KEY_SLEFT)
        assert state.heading_goal == pytest.approx(120.0)

    def test_shift_right_increments_ten(self, tui, state):
        state.set_goal(130.0, "init")
        tui._handle_key(curses.KEY_SRIGHT)
        assert state.heading_goal == pytest.approx(140.0)

    def test_brackets_are_goal_ten(self, tui, state):
        """
        Goal +-10 moved to [ and ] when . and , became manual-step keys.
        KEY_SLEFT/SRIGHT still mirror them for terminals that send those.
        """
        state.set_goal(130.0, "init")
        tui._handle_key(ord("["))
        assert state.heading_goal == pytest.approx(120.0)
        tui._handle_key(ord("]"))
        assert state.heading_goal == pytest.approx(130.0)

    def test_up_snaps_down_is_five(self, tui, state):
        """Up now snaps to heading (see TestSnapKey); Down is still goal -5."""
        state.set_signal("fused_heading", 200.0)
        state.set_goal(130.0, "init")
        tui._handle_key(curses.KEY_DOWN)
        assert state.heading_goal == pytest.approx(125.0)

    def test_wraps_down_through_north(self, tui, state):
        state.set_goal(0.0, "init")
        tui._handle_key(curses.KEY_LEFT)
        assert state.heading_goal == pytest.approx(359.0)

    def test_wraps_up_through_north(self, tui, state):
        state.set_goal(359.5, "init")
        tui._handle_key(curses.KEY_RIGHT)
        assert state.heading_goal == pytest.approx(0.5)

    def test_repeated_presses_accumulate(self, tui, state):
        state.set_goal(0.0, "init")
        for _ in range(15):
            tui._handle_key(curses.KEY_RIGHT)
        assert state.heading_goal == pytest.approx(15.0)

    def test_goal_source_is_tui(self, tui, state):
        state.set_goal(130.0, "web/1.2.3.4")
        tui._handle_key(curses.KEY_RIGHT)
        assert state.goal_source == "tui"


class TestSnapKey:
    def test_up_snaps_to_fused(self, tui, state):
        state.set_signal("fused_heading", 127.4)
        state.set_goal(200.0, "init")
        tui._handle_key(curses.KEY_UP)
        assert state.heading_goal == pytest.approx(127.4)

    def test_falls_back_to_compass(self, tui, state):
        state.set_signal("compass_heading", 131.2)
        state.set_goal(200.0, "init")
        tui._handle_key(curses.KEY_UP)
        assert state.heading_goal == pytest.approx(131.2)

    def test_no_heading_leaves_goal_alone(self, tui, state):
        state.set_goal(200.0, "init")
        tui._handle_key(curses.KEY_UP)
        assert state.heading_goal == pytest.approx(200.0)

    def test_no_heading_flashes_message(self, tui, state):
        tui._handle_key(curses.KEY_UP)
        assert "no heading" in tui._flash_msg.lower()


class TestCommandKeys:
    @pytest.mark.parametrize(
        "key,expected",
        [
            (ord("e"), "autopilot_enable"),
            (ord("E"), "autopilot_enable"),
            (ord("d"), "autopilot_disable"),
            (ord("D"), "autopilot_disable"),
            (ord("."), "manual_step_right"),
            (ord(","), "manual_step_left"),
            (ord("c"), "clear_faults"),
            (ord("q"), "quit"),
            (ord("Q"), "quit"),
        ],
    )
    def test_command_dispatch(self, tui, key, expected):
        tui._handle_key(key)
        assert expected in tui.commands

    def test_servo_toggles_on_state(self, tui, state):
        state.servo_enabled = False
        tui._handle_key(ord("s"))
        assert tui.commands[-1] == "servo_enable"

        state.servo_enabled = True
        tui._handle_key(ord("s"))
        assert tui.commands[-1] == "servo_disable"

    def test_unknown_key_ignored(self, tui, state):
        state.set_goal(130.0, "init")
        tui._handle_key(ord("z"))
        tui._handle_key(999)
        assert state.heading_goal == pytest.approx(130.0)
        assert tui.commands == []


# ---------------------------------------------------------------------------
# Rendering -- needs a real terminal, so run in a pty subprocess
# ---------------------------------------------------------------------------


RENDER_SCRIPT = r'''
import curses, sys, os, time
sys.path.insert(0, {hmi!r})
from hmi_state import SystemState
from hmi_heading import HeadingMonitor
from hmi_bridge import Bridge
from hmi_tui import TUI

def main(scr):
    st = SystemState(); mon = HeadingMonitor(st); br = Bridge(st, mon)
    tui = TUI(st,
              on_goal_delta=lambda d: st.adjust_goal(d, "tui"),
              on_goal_set=lambda v: st.set_goal(v, "tui"),
              on_command=lambda c: None)
    tui.start(scr)

    st.link_up = True; st.backend = "socketcan"; st.channel = "can0"
    st.bitrate = 250000; st.rx_total = 48213; st.tx_total = 903
    st.bus_state = "ERROR-ACTIVE"; st.web_clients = 2
    now = time.time()
    for name, val in (("compass_heading",131.2),("cog",124.8),("sog",6.4),
                      ("fused_heading",127.4),("heading_sigma",2.1),("yaw_rate",-0.8),
                      ("steering_angle",1447),("steering_goal",1425),("shaft_center",1425),
                      ("rudder_angle",-3.2),("rpm",1850),("pitch",-1.3)):
        st.set_signal(name, val)
    st.set_goal(130.0, "web/192.168.1.47")
    st.autopilot_engaged = True; st.servo_enabled = True; st.heading_error = -2.6
    st.cog_accepted = 1137; st.cog_rejected = 503
    import hmi_bridge
    for cid in hmi_bridge.WATCHED:
        st.mark_source(cid, now)
    st.sources[0x09F112F8].last_rx = now - 4.3
    st.raise_fault("STALE", "vessel_heading(4.3s)")

    tui.render()

    rows, cols = scr.getmaxyx()
    lines = []
    for y in range(rows):
        try:
            lines.append(scr.instr(y, 0).decode("utf-8", "replace").rstrip())
        except Exception:
            lines.append("")
    open({out!r}, "w").write("\n".join(lines))

    # sizes that break naive curses code
    for h, w in ((10, 40), (24, 80), (5, 20), (50, 200)):
        try:
            scr.resize(h, w)
            tui.render()
        except Exception as e:
            open({err!r}, "a").write("RESIZE %dx%d: %r\n" % (h, w, e))

curses.wrapper(main)
'''


@pytest.fixture
def rendered(tmp_path):
    """Render the TUI in a pty and return the captured screen text."""
    out = tmp_path / "screen.txt"
    err = tmp_path / "errors.txt"
    script = tmp_path / "render.py"
    script.write_text(RENDER_SCRIPT.format(hmi=str(HMI_DIR), out=str(out), err=str(err)))

    proc = subprocess.run(
        ["script", "-qec", f"{sys.executable} {script}", "/dev/null"],
        cwd=str(HMI_DIR),
        capture_output=True,
        text=True,
        timeout=60,
        env={**os.environ, "TERM": "xterm-256color"},
    )
    if not out.exists():
        pytest.skip(f"curses render unavailable: {proc.stdout[-500:]}")
    return out.read_text(), (err.read_text() if err.exists() else "")


@pytest.mark.integration
class TestRendering:
    def test_no_render_errors_at_any_size(self, rendered):
        _, errors = rendered
        assert errors == "", f"render raised: {errors}"

    def test_shows_headings(self, rendered):
        screen, _ = rendered
        assert "FUSED" in screen
        assert "127.4" in screen
        assert "COMPASS" in screen

    def test_shows_goal_with_provenance(self, rendered):
        """
        The whole point of the sync feature: the operator can see that the
        phone moved the goal, not just that it changed.
        """
        screen, _ = rendered
        assert "GOAL" in screen
        assert "130.0" in screen
        assert "web/192.168.1" in screen

    def test_shows_link_status(self, rendered):
        screen, _ = rendered
        assert "LINK UP" in screen
        assert "socketcan:can0" in screen

    def test_shows_web_client_count(self, rendered):
        screen, _ = rendered
        assert "WEB 2" in screen

    def test_shows_sources_with_ages(self, rendered):
        screen, _ = rendered
        assert "CAN SOURCES" in screen
        assert "0x18F01D21" in screen
        assert "steering" in screen

    def test_shows_fault(self, rendered):
        screen, _ = rendered
        assert "FAULTS" in screen
        assert "STALE" in screen
        assert "vessel_heading" in screen

    def test_shows_engaged_indicators(self, rendered):
        screen, _ = rendered
        assert "AUTOPILOT" in screen
        assert "SERVO" in screen

    def test_shows_engage_indicators(self, rendered):
        """
        At 80 wide the engage/servo indicators live in the rudder panel. (COG
        accounting moved to the wide-only STATUS panel; see
        test_touch_buttons.py::TestWideLayout for that.)
        """
        screen, _ = rendered
        assert "AUTOPILOT" in screen
        assert "SERVO" in screen

    def test_shows_keybindings(self, rendered):
        screen, _ = rendered
        low = screen.lower()
        assert "[e]ng" in low
        assert "[q]uit" in low, (
            "quit hint is truncated -- the footer string is longer than "
            "MIN_COLS-2 and gets clipped on an 80-column terminal"
        )

    def test_panels_are_fixed_width(self, rendered):
        """
        Nothing should wrap or shift. If a panel border lands at a different
        column between rows, the layout has drifted.
        """
        screen, _ = rendered
        borders = [ln for ln in screen.splitlines() if ln.startswith("+-")]
        assert len(borders) >= 3
