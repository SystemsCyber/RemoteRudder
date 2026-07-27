"""
In-process TUI rendering tests.

test_tui.py renders in a subprocess, which is the honest end-to-end check but
invisible to coverage and slow. This module attaches curses to a pty inside
the test process instead, so the drawing code is actually measured and edge
cases are cheap to enumerate.

The failure mode being hunted: curses raises on writing the bottom-right cell,
and on any addstr past the window edge. Those errors only appear at specific
terminal sizes, which is exactly the kind of thing that works on the dev
machine and breaks on the boat.
"""

from __future__ import annotations

import curses
import os
import pty
import time

import pytest

from hmi_state import SystemState
from hmi_tui import MIN_COLS, MIN_ROWS, TUI


def _watched_items():
    """(can_id, (name, stale_after)) pairs from the bridge's watch table."""
    import hmi_bridge

    return list(hmi_bridge.WATCHED.items())


@pytest.fixture
def screen():
    """
    A real curses screen on a pty, torn down cleanly.

    curses.initscr() talks to the process's actual stdin/stdout, not to a
    handle we pass it, so the pty has to be swapped in at the file-descriptor
    level before initscr runs. Without that, cbreak() fails with ERR under
    pytest because captured stdout is not a terminal.

    Skips rather than fails where curses cannot initialize (no terminfo in
    some CI images), because a skipped render test is better than a suite
    that cannot run at all.
    """
    master, slave = pty.openpty()

    # Give the pty a known size so layout assertions are deterministic.
    try:
        import fcntl
        import struct as _struct
        import termios

        fcntl.ioctl(slave, termios.TIOCSWINSZ, _struct.pack("HHHH", 24, 80, 0, 0))
    except Exception:
        pass

    old_term = os.environ.get("TERM")
    old_lines = os.environ.get("LINES")
    old_cols = os.environ.get("COLUMNS")
    os.environ["TERM"] = "xterm-256color"
    # initscr consults LINES/COLUMNS before the tty size when they are set,
    # and under pytest the inherited values can be tiny. Pin them so the
    # screen comes up at the size the layout assertions assume.
    os.environ["LINES"] = "24"
    os.environ["COLUMNS"] = "80"

    saved_in = os.dup(0)
    saved_out = os.dup(1)
    os.dup2(slave, 0)
    os.dup2(slave, 1)

    stdscr = None
    try:
        try:
            stdscr = curses.initscr()
            curses.noecho()
            curses.cbreak()
            stdscr.keypad(True)
        except Exception as e:  # noqa: BLE001
            pytest.skip(f"curses unavailable in this environment: {e}")

        rows, cols = stdscr.getmaxyx()
        if rows < MIN_ROWS or cols < MIN_COLS:
            # Last resort: ask curses to resize its internal representation.
            try:
                curses.resize_term(24, 80)
                stdscr.resize(24, 80)
            except Exception:
                pass
            rows, cols = stdscr.getmaxyx()
            if rows < MIN_ROWS or cols < MIN_COLS:
                pytest.skip(
                    f"cannot obtain a {MIN_COLS}x{MIN_ROWS} screen "
                    f"(got {cols}x{rows}); content assertions need one"
                )
        yield stdscr
    finally:
        if stdscr is not None:
            try:
                stdscr.keypad(False)
                curses.nocbreak()
                curses.echo()
                curses.endwin()
            except Exception:
                pass
        os.dup2(saved_in, 0)
        os.dup2(saved_out, 1)
        os.close(saved_in)
        os.close(saved_out)
        for key, val in (("TERM", old_term), ("LINES", old_lines), ("COLUMNS", old_cols)):
            if val is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = val
        os.close(master)
        os.close(slave)


@pytest.fixture
def populated_state():
    """State with a realistic full set of values."""
    import hmi_bridge

    st = SystemState()
    st.link_up = True
    st.backend = "socketcan"
    st.channel = "can0"
    st.bitrate = 250000
    st.rx_total = 48213
    st.tx_total = 903
    st.bus_state = "ERROR-ACTIVE"
    st.web_clients = 2
    now = time.time()
    for name, val in (
        ("compass_heading", 131.2),
        ("cog", 124.8),
        ("sog", 6.4),
        ("fused_heading", 127.4),
        ("heading_sigma", 2.1),
        ("yaw_rate", -0.8),
        ("steering_angle", 1447),
        ("steering_goal", 1425),
        ("shaft_center", 1425),
        ("rudder_angle", -3.2),
        ("rpm", 1850),
        ("pitch", -1.3),
    ):
        st.set_signal(name, val)
    st.set_goal(130.0, "web/192.168.1.47")
    st.autopilot_engaged = True
    st.servo_enabled = True
    st.heading_error = -2.6
    st.cog_accepted = 1137
    st.cog_rejected = 503
    for cid in hmi_bridge.WATCHED:
        st.register_source(cid, hmi_bridge.WATCHED[cid][0], hmi_bridge.WATCHED[cid][1])
        st.mark_source(cid, now)
    return st


@pytest.fixture
def live_tui(screen, populated_state):
    t = TUI(
        populated_state,
        on_goal_delta=lambda d: populated_state.adjust_goal(d, "tui"),
        on_goal_set=lambda v: populated_state.set_goal(v, "tui"),
        on_command=lambda c: None,
    )
    t.start(screen)
    return t


@pytest.mark.integration
class TestRenderDoesNotRaise:
    def test_full_render(self, live_tui):
        live_tui.render()

    def test_render_with_no_data(self, screen):
        """Startup state: every signal None. Must not divide by None or crash."""
        st = SystemState()
        t = TUI(st, lambda d: None, lambda v: None, lambda c: None)
        t.start(screen)
        t.render()

    def test_render_with_faults(self, live_tui, populated_state):
        populated_state.raise_fault("BUS_OFF", "controller bus-off")
        populated_state.raise_fault("STALE", "vessel_heading(4.3s)")
        populated_state.raise_fault("HEADING_NOISE", "sigma 12.3deg")
        live_tui.render()

    def test_render_with_link_down(self, live_tui, populated_state):
        populated_state.link_up = False
        populated_state.next_reconnect_at = time.time() + 8
        live_tui.render()

    def test_render_with_long_fault_detail(self, live_tui, populated_state):
        """A long message must be clipped, not wrapped into the next panel."""
        populated_state.raise_fault("LINK_DOWN", "x" * 500)
        live_tui.render()

    def test_render_with_extreme_values(self, live_tui, populated_state):
        for name, val in (
            ("steering_angle", -99999),
            ("rpm", 999999),
            ("heading_sigma", 179.9),
            ("sog", 0.0),
        ):
            populated_state.set_signal(name, val)
        live_tui.render()

    def test_render_with_turn_indicators(self, live_tui, populated_state):
        populated_state.left_turn = True
        live_tui.render()
        populated_state.left_turn = False
        populated_state.right_turn = True
        live_tui.render()

    def test_render_many_events(self, live_tui, populated_state):
        for i in range(300):
            populated_state.log_event("TEST", f"event number {i} with some text")
        live_tui.render()

    def test_repeated_renders_stable(self, live_tui):
        for _ in range(50):
            live_tui.render()


@pytest.mark.integration
class TestRenderAtSize:
    @pytest.mark.parametrize(
        "rows,cols",
        [
            (24, 80),    # the standard, and the tightest supported
            (25, 80),
            (30, 100),
            (50, 200),
            (24, 78),    # exactly MIN_COLS
            (23, 80),    # one row short -> too-small path
            (24, 77),    # one column short -> too-small path
            (10, 40),    # far too small
            (5, 20),     # absurd
            (100, 300),  # very large
        ],
    )
    def test_no_exception(self, screen, populated_state, rows, cols):
        """
        curses raises when you write the bottom-right cell or run past the
        edge. These sizes bracket every boundary in the layout code.
        """
        t = TUI(populated_state, lambda d: None, lambda v: None, lambda c: None)
        t.start(screen)
        try:
            curses.resize_term(rows, cols)
        except Exception:
            pytest.skip("terminal cannot be resized in this environment")
        t.render()

    def test_below_minimum_shows_message(self, screen, populated_state):
        t = TUI(populated_state, lambda d: None, lambda v: None, lambda c: None)
        t.start(screen)
        try:
            curses.resize_term(10, 40)
        except Exception:
            pytest.skip("cannot resize")
        t.render()
        text = screen.instr(0, 0).decode("utf-8", "replace")
        assert "too small" in text.lower()

    def test_message_states_required_size(self, screen, populated_state):
        t = TUI(populated_state, lambda d: None, lambda v: None, lambda c: None)
        t.start(screen)
        try:
            curses.resize_term(10, 40)
        except Exception:
            pytest.skip("cannot resize")
        t.render()
        text = screen.instr(0, 0).decode("utf-8", "replace")
        assert str(MIN_COLS) in text and str(MIN_ROWS) in text


@pytest.mark.integration
class TestRenderContent:
    def _screen_text(self, scr):
        rows, cols = scr.getmaxyx()
        return "\n".join(
            scr.instr(y, 0).decode("utf-8", "replace").rstrip() for y in range(rows)
        )

    def test_goal_provenance_visible(self, live_tui, screen):
        live_tui.render()
        text = self._screen_text(screen)
        assert "web/192.168.1" in text

    def test_provenance_updates_when_tui_moves_goal(self, live_tui, screen, populated_state):
        live_tui.render()
        assert "web/192.168.1" in self._screen_text(screen)

        live_tui._handle_key(curses.KEY_RIGHT)
        live_tui.render()
        text = self._screen_text(screen)
        assert "tui" in text
        assert "131.0" in text

    def test_flash_message_shown(self, live_tui, screen):
        live_tui.flash("ENGAGE BLOCKED: sigma 12.3deg", secs=5.0)
        live_tui.render()
        assert "ENGAGE BLOCKED" in self._screen_text(screen)

    def test_flash_expires(self, live_tui, screen):
        live_tui.flash("temporary", secs=0.01)
        time.sleep(0.05)
        live_tui.render()
        assert "temporary" not in self._screen_text(screen)

    def test_stale_source_marked(self, live_tui, screen, populated_state):
        # Mark the first registered source stale. The panel is height-capped,
        # but the first source is always drawn (the cap truncates from the
        # bottom), so this is deterministic regardless of terminal height or
        # test order. Marking it well past any stale threshold.
        first_id = next(iter(populated_state.sources))
        populated_state.sources[first_id].last_rx = time.time() - 300
        populated_state.sources[first_id].ever_seen = True
        live_tui.render()
        assert "STALE" in self._screen_text(screen)

    def test_sources_panel_truncates_at_24_rows(self, live_tui, screen, populated_state):
        """
        Documents a real limitation rather than asserting desired behaviour.

        The panel is capped at 9 rows (min(3 + len(sources), 9)), leaving room
        for 6 entries after the border and header. With 8 watched PGNs, the
        last two -- currently vessel_heading and autopilot_status -- are never
        drawn on a standard 24-row terminal.

        That matters: vessel_heading is the compass source, so a compass that
        goes quiet is invisible in the sources panel. The FAULTS line still
        names it via the STALE fault, which is the saving grace, but the
        per-source age is not shown.

        If the panel is ever made scrollable or paged, this test should be
        updated to assert all eight are reachable.
        """
        live_tui.render()
        text = self._screen_text(screen)
        shown = [name for _, (name, _) in _watched_items() if name in text]
        assert len(shown) < len(populated_state.sources), (
            "panel now shows every source -- update this test if the cap changed"
        )
        assert "steering" in text, "the first sources should still be visible"

    def test_no_faults_shows_none(self, live_tui, screen):
        live_tui.render()
        assert "FAULTS: none" in self._screen_text(screen)


@pytest.mark.integration
class TestPollKeys:
    def test_poll_with_empty_buffer_returns(self, live_tui):
        """nodelay is set, so getch returns -1 and poll must not block."""
        live_tui.poll_keys()

    def test_poll_without_screen_is_safe(self, populated_state):
        t = TUI(populated_state, lambda d: None, lambda v: None, lambda c: None)
        t.poll_keys()  # scr is None; must not raise

    def test_resize_key_clears_screen(self, live_tui):
        live_tui._handle_key(curses.KEY_RESIZE)


class TestClaimColumnsInCompactTable:
    """The address-claim data (MFG, FUNCTION, IDENT) must show in the compact
    source table, not only the 136-col wide view."""

    def _render(self, cols=120):
        import os, pty, curses, struct, fcntl, termios, time
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
            st.link_up = True; st.backend = "socketcan"; st.channel = "can0"
            st.bitrate = 250000
            now = time.time()
            st.register_source(0x09F112F8, "compass_F8", 3.0)
            st.mark_source(0x09F112F8, now, data=bytes.fromhex("00A00F00FFFFFFFF"))
            st.record_claim(0xF8, {"manufacturer": "Garmin",
                                   "function": "Heading Sensor",
                                   "identity": 1039212})
            t.render()
            rows, c = scr.getmaxyx()
            return "\n".join(
                scr.instr(y, 0).decode("utf-8", "replace") for y in range(rows))
        finally:
            curses.nocbreak(); curses.echo(); curses.endwin()
            os.dup2(si, 0); os.dup2(so, 1)

    def test_compact_table_has_claim_headers(self):
        out = self._render(cols=120)
        assert "MFG" in out
        assert "FUNCTION" in out
        assert "IDENT" in out

    def test_compact_table_shows_claim_values(self):
        out = self._render(cols=120)
        assert "Garmin" in out          # manufacturer
        assert "1039212" in out         # identity
