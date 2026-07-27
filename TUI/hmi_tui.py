"""
hmi_tui.py -- Fixed-panel curses console.

The point of this module is that nothing moves. Values are redrawn in place
at fixed coordinates, so you can look at the same spot on the screen every
time to find the same number. The only region that scrolls is the event log
at the bottom, which is where transient things belong.

Layout is computed from the terminal size at startup and on SIGWINCH, with a
hard minimum below which we show a "terminal too small" message rather than
throwing curses errors.

Key handling
------------
  Left / Right      heading goal -1 / +1 deg
  Shift-Left/Right  heading goal -10 / +10 deg
  Up / Down         heading goal +5 / -5 deg
  Home              snap goal to current heading
  e / d             engage / disengage autopilot
  s                 toggle servo enable
  [ / ]             nudge rudder shaft left / right
  c                 clear latched faults
  l                 cycle log verbosity
  q                 quit

Arrow keys are read through curses keypad mode. Shift-arrows arrive as
KEY_SLEFT/KEY_SRIGHT on most terminals; on those where they do not, the
same function is available on , and . which are unshifted and always work.
"""

from __future__ import annotations

import curses
import time
from typing import Callable, Optional

from hmi_state import SystemState

MIN_COLS = 78
MIN_ROWS = 24

# Color pair ids
C_NORMAL = 1
C_OK = 2
C_WARN = 3
C_ERR = 4
C_DIM = 5
C_HEAD = 6
C_ACCENT = 7


class TUI:
    def __init__(
        self,
        state: SystemState,
        on_goal_delta: Callable[[float], None],
        on_goal_set: Callable[[float], None],
        on_command: Callable[[str], None],
    ) -> None:
        self.state = state
        self.on_goal_delta = on_goal_delta
        self.on_goal_set = on_goal_set
        self.on_command = on_command
        self.scr: Optional["curses._CursesWindow"] = None
        self.running = False
        self._flash_until = 0.0
        self._flash_msg = ""
        self._last_goal_source = ""
        self._last_goal_at = 0.0
        self._buttons = []  # [(y, x0, x1, cmd, enabled)] rebuilt each render
        self._mouse_ok = False

    # -- lifecycle ---------------------------------------------------------

    def start(self, scr) -> None:
        self.scr = scr
        self.running = True
        curses.curs_set(0)
        scr.nodelay(True)
        scr.keypad(True)
        # Enable touch/mouse. BUTTON1_CLICKED covers a tap on most touch
        # drivers (they synthesize a left click). Wrapped in try/except
        # because not every terminal supports mouse, and a headless one must
        # still run.
        try:
            curses.mousemask(curses.BUTTON1_CLICKED | curses.BUTTON1_PRESSED)
            self._mouse_ok = True
        except curses.error:
            self._mouse_ok = False
        if curses.has_colors():
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(C_NORMAL, curses.COLOR_WHITE, -1)
            curses.init_pair(C_OK, curses.COLOR_GREEN, -1)
            curses.init_pair(C_WARN, curses.COLOR_YELLOW, -1)
            curses.init_pair(C_ERR, curses.COLOR_RED, -1)
            curses.init_pair(C_DIM, curses.COLOR_CYAN, -1)
            curses.init_pair(C_HEAD, curses.COLOR_BLACK, curses.COLOR_WHITE)
            curses.init_pair(C_ACCENT, curses.COLOR_MAGENTA, -1)

    def stop(self) -> None:
        self.running = False

    def flash(self, msg: str, secs: float = 2.0) -> None:
        self._flash_msg = msg
        self._flash_until = time.time() + secs

    # -- input -------------------------------------------------------------

    def poll_keys(self) -> None:
        """Drain the key buffer. Non-blocking; safe to call on every tick."""
        if self.scr is None:
            return
        while True:
            try:
                ch = self.scr.getch()
            except curses.error:
                return
            if ch == -1:
                return
            if ch == curses.KEY_MOUSE:
                try:
                    _id, mx, my, _z, _bstate = curses.getmouse()
                    self._on_touch(my, mx)
                except curses.error:
                    pass
                continue
            self._handle_key(ch)

    def _handle_key(self, ch: int) -> None:
        if ch == curses.KEY_LEFT:
            self.on_goal_delta(-1.0)
        elif ch == curses.KEY_RIGHT:
            self.on_goal_delta(+1.0)
        elif ch == ord("[") or ch == curses.KEY_SLEFT:
            self.on_goal_delta(-10.0)
        elif ch == ord("]") or ch == curses.KEY_SRIGHT:
            self.on_goal_delta(+10.0)
        elif ch == curses.KEY_DOWN:
            self.on_goal_delta(-5.0)
        elif ch == curses.KEY_UP:
            # Up arrow snaps the goal to the current heading (was Home, which
            # needed the Fn key on the operator's keyboard). "Set goal to
            # current heading" is the most common pre-engage action, so it
            # gets the easiest-to-reach key.
            hv = self.state.signal_value("fused_heading")
            if hv is None:
                hv = self.state.signal_value("compass_heading")
            if hv is not None:
                self.on_goal_set(float(hv))
                self.flash(f"goal snapped to {float(hv):.1f}")
            else:
                self.flash("no heading available")
        elif ch == ord("."):
            # Manual motor step right. Disengaged-only; the command handler
            # enforces that and flashes if the autopilot is engaged.
            self.on_command("manual_step_right")
        elif ch == ord(","):
            self.on_command("manual_step_left")
        elif ch in (ord("e"), ord("E")):
            self.on_command("autopilot_enable")
        elif ch in (ord("d"), ord("D")):
            self.on_command("autopilot_disable")
        elif ch in (ord("s"), ord("S")):
            self.on_command(
                "servo_disable" if self.state.servo_enabled else "servo_enable"
            )
        elif ch in (ord("c"), ord("C")):
            self.on_command("clear_faults")
            self.flash("faults cleared")
        elif ch in (ord("a"), ord("A")):
            self.on_command("request_addresses")
        elif ch == ord("\t"):
            # TAB cycles which PID gain the +/- keys adjust.
            order = ["Kp", "Ki", "Kd"]
            cur = getattr(self, "_pid_selected", "Kp")
            self._pid_selected = order[(order.index(cur) + 1) % len(order)]
            self.flash(f"PID tuning: {self._pid_selected}")
        elif ch in (ord("+"), ord("=")):
            # Increase the selected gain (= is the unshifted +, easier to hit).
            sel = getattr(self, "_pid_selected", "Kp")
            self.on_command(f"pid_gain_up:{sel}")
        elif ch == ord("-"):
            sel = getattr(self, "_pid_selected", "Kp")
            self.on_command(f"pid_gain_down:{sel}")
        elif ch in (ord("q"), ord("Q")):
            self.on_command("quit")
        elif ch == curses.KEY_RESIZE:
            if self.scr:
                self.scr.clear()

    # -- drawing helpers ---------------------------------------------------

    def _put(self, y: int, x: int, text: str, attr: int = 0) -> None:
        """Bounds-checked write. curses raises if you touch the last cell."""
        if self.scr is None:
            return
        rows, cols = self.scr.getmaxyx()
        if y < 0 or y >= rows or x < 0 or x >= cols:
            return
        avail = cols - x
        if avail <= 0:
            return
        s = text[:avail]
        # Writing the bottom-right cell raises; trim one char there.
        if y == rows - 1 and x + len(s) >= cols:
            s = s[: max(0, cols - x - 1)]
        if not s:
            return
        try:
            self.scr.addstr(y, x, s, attr)
        except curses.error:
            pass

    def _box(self, y: int, x: int, h: int, w: int, title: str) -> None:
        self._put(y, x, "+" + "-" * (w - 2) + "+", curses.color_pair(C_DIM))
        if title:
            self._put(y, x + 2, f" {title} ", curses.color_pair(C_DIM) | curses.A_BOLD)
        for i in range(1, h - 1):
            self._put(y + i, x, "|", curses.color_pair(C_DIM))
            self._put(y + i, x + w - 1, "|", curses.color_pair(C_DIM))
        self._put(y + h - 1, x, "+" + "-" * (w - 2) + "+", curses.color_pair(C_DIM))

    @staticmethod
    def _fmt(v, spec: str = "{:.1f}", width: int = 7, dash: str = "---") -> str:
        if v is None:
            return dash.rjust(width)
        try:
            return spec.format(v).rjust(width)
        except (TypeError, ValueError):
            return str(v).rjust(width)

    def _age_attr(self, age: Optional[float], stale_after: float) -> int:
        if age is None:
            return curses.color_pair(C_ERR)
        if age > stale_after:
            return curses.color_pair(C_ERR) | curses.A_BOLD
        if age > stale_after * 0.5:
            return curses.color_pair(C_WARN)
        return curses.color_pair(C_OK)

    # -- render ------------------------------------------------------------

    def render(self) -> None:
        if self.scr is None:
            return
        rows, cols = self.scr.getmaxyx()
        if rows < MIN_ROWS or cols < MIN_COLS:
            self.scr.erase()
            self._put(0, 0, f"Terminal too small: need {MIN_COLS}x{MIN_ROWS}, have {cols}x{rows}")
            self.scr.refresh()
            return

        st = self.state
        now = time.time()
        self._buttons = []  # rebuilt every frame; touch maps against this
        self.scr.erase()

        # Layout. At >=118 cols we run three panels across the top so the
        # rudder panel stops sprawling into dead space and the heading-source
        # status gets a home. Narrower than that, fall back to the two-panel
        # stack the 80-col terminal needs.
        wide = cols >= 118
        self._draw_header(0, cols, now)
        if wide:
            self._draw_heading_panel(2, 0, 38)
            self._draw_rudder_panel(2, 39, 40)
            right_x = 80
            right_w = cols - right_x
            # When the terminal is wide enough for both (right region >= 72),
            # put the PID panel in the upper-right for tuning and STATUS beside
            # it. Narrower than that (e.g. 120 cols), keep STATUS in the right
            # region as before; the PID panel needs a wider terminal.
            if right_w >= 72:
                pid_w = 38
                self._draw_pid_panel(2, right_x, pid_w)
                self._draw_status_panel(2, right_x + pid_w, right_w - pid_w)
            else:
                self._draw_status_panel(2, right_x, right_w)
        else:
            self._draw_heading_panel(2, 0, 38)
            self._draw_rudder_panel(2, 39, cols - 39)

        src_top = 11
        # If the EKF is producing output, show a one-line EKF strip between the
        # top panels and the sources table. Keeps the operator's requested EKF
        # inputs/diagnostics/outputs visible without a whole extra panel.
        if self.state.signal_value("ekf_heading") is not None:
            self._draw_ekf_strip(src_top, 0, cols)
            src_top += 1
        # Panel height must leave room below for faults, events, the button
        # bar, and the footer (4 rows minimum). On a short 24-row terminal that
        # caps the table shorter than on a tall one. The panel draw uses the
        # same cap so the two never disagree (which previously let the table
        # overwrite the faults line).
        max_src_rows = max(3, (rows - 4) - src_top - 3)
        src_h = min(3 + len(st.sources), 3 + max_src_rows, 12)
        self._src_h = src_h  # the panel reads this so both agree
        self._draw_sources_panel(src_top, 0, cols, now)
        log_top = src_top + src_h
        self._draw_faults(log_top, cols)

        # Reserve two rows at the bottom: the button bar and the key footer.
        btn_row = rows - 2
        events_h = btn_row - (log_top + 1) - 1
        self._draw_events(log_top + 1, cols, max(events_h, 0))
        self._draw_buttons(btn_row, cols)
        self._draw_footer(rows - 1, cols, now)

        self.scr.refresh()

    def _draw_header(self, y: int, cols: int, now: float) -> None:
        st = self.state
        try:
            from version import VERSION
            title = f" REMOTERUDDER HMI v{VERSION} "
        except Exception:
            title = " REMOTERUDDER HMI "
        self._put(y, 0, " " * cols, curses.color_pair(C_HEAD))
        self._put(y, 1, title, curses.color_pair(C_HEAD) | curses.A_BOLD)

        if st.link_up:
            link_txt = f"LINK UP {st.backend}:{st.channel}"
            link_attr = curses.color_pair(C_HEAD)
        else:
            nxt = max(0.0, st.next_reconnect_at - now)
            link_txt = f"LINK DOWN  retry {nxt:.0f}s"
            link_attr = curses.color_pair(C_HEAD) | curses.A_BLINK | curses.A_BOLD

        self._put(y, 27, link_txt, link_attr)

        rate_txt = f"{st.bitrate // 1000}k  RX {st.rx_total}  TX {st.tx_total}"
        self._put(y, 53, rate_txt, curses.color_pair(C_HEAD))

        clk = time.strftime("%H:%M:%S", time.localtime(now))
        self._put(y, cols - 10, clk, curses.color_pair(C_HEAD))

        # Second line: web client count and bus state
        web = f"WEB {st.web_clients} client{'s' if st.web_clients != 1 else ''}"
        web_attr = curses.color_pair(C_OK) if st.web_clients else curses.color_pair(C_DIM)
        self._put(y + 1, 1, web, web_attr)
        self._put(y + 1, 16, f"BUS {st.bus_state}", curses.color_pair(C_DIM))
        if st.rx_error_count or st.tx_error_count:
            self._put(
                y + 1,
                32,
                f"ERRCNT tx={st.tx_error_count} rx={st.rx_error_count}",
                curses.color_pair(C_ERR) | curses.A_BOLD,
            )

    def _draw_heading_panel(self, y: int, x: int, w: int) -> None:
        st = self.state
        now = time.time()
        self._box(y, x, 9, w, "HEADING")

        fused = st.get_signal("fused_heading")
        comp = st.get_signal("compass_heading")
        cog = st.get_signal("cog")
        sigma = st.signal_value("heading_sigma")
        rate = st.signal_value("yaw_rate")

        row = y + 1
        # Fused, the big one
        fv = fused.value if fused else None
        self._put(row, x + 2, "FUSED", curses.A_BOLD)
        self._put(
            row, x + 10, self._fmt(fv, "{:.1f}", 8),
            curses.A_BOLD | (curses.color_pair(C_OK) if fv is not None else curses.color_pair(C_ERR)),
        )
        if sigma is not None:
            sattr = (
                curses.color_pair(C_ERR) if sigma > 6
                else curses.color_pair(C_WARN) if sigma > 4
                else curses.color_pair(C_OK)
            )
            self._put(row, x + 20, f"sig {sigma:4.1f}", sattr)
        else:
            self._put(row, x + 20, "sig  ---", curses.color_pair(C_DIM))

        row += 1
        for label, sig in (("COMPASS", comp), ("COG", cog)):
            v = sig.value if sig else None
            age = sig.age(now) if sig else None
            self._put(row, x + 2, label)
            self._put(row, x + 10, self._fmt(v, "{:.1f}", 8))
            if sig and age is not None and age != float("inf"):
                self._put(row, x + 20, f"{age:5.1f}s", self._age_attr(age, sig.stale_after))
            else:
                self._put(row, x + 20, "   ---", curses.color_pair(C_ERR))
            row += 1

        # rate
        self._put(row, x + 2, "RATE")
        self._put(row, x + 10, self._fmt(rate, "{:+.1f}", 8))
        self._put(row, x + 19, "deg/s", curses.color_pair(C_DIM))
        row += 1

        # separator
        self._put(row, x + 2, "-" * (w - 4), curses.color_pair(C_DIM))
        row += 1

        # goal, with provenance
        self._put(row, x + 2, "GOAL", curses.A_BOLD)
        self._put(
            row, x + 10, self._fmt(st.heading_goal, "{:.1f}", 8),
            curses.A_BOLD | curses.color_pair(C_ACCENT),
        )
        src = st.goal_source
        fresh = (now - st.goal_changed_at) < 3.0 if st.goal_changed_at else False
        src_attr = (curses.color_pair(C_ACCENT) | curses.A_BOLD) if fresh else curses.color_pair(C_DIM)
        self._put(row, x + 19, f"<-{src[:14]}", src_attr)
        row += 1

        # error
        self._put(row, x + 2, "ERROR")
        err_attr = (
            curses.color_pair(C_ERR) if abs(st.heading_error) > 20
            else curses.color_pair(C_WARN) if abs(st.heading_error) > 8
            else curses.color_pair(C_OK)
        )
        self._put(row, x + 10, self._fmt(st.heading_error, "{:+.1f}", 8), err_attr)

    def _draw_rudder_panel(self, y: int, x: int, w: int) -> None:
        st = self.state
        self._box(y, x, 9, w, "RUDDER / DRIVE")

        row = y + 1
        pairs = [
            ("SHAFT", st.signal_value("steering_angle"), "{:.0f}"),
            ("S.GOAL", st.signal_value("steering_goal"), "{:.0f}"),
            ("CENTER", st.signal_value("shaft_center"), "{:.0f}"),
            ("ANGLE", st.signal_value("rudder_angle"), "{:+.1f}"),
            ("RPM", st.signal_value("rpm"), "{:.0f}"),
            ("SOG", st.signal_value("sog"), "{:.1f}"),
        ]
        for label, val, spec in pairs:
            self._put(row, x + 2, label)
            self._put(row, x + 10, self._fmt(val, spec, 8), curses.A_BOLD)
            row += 1

        # In the narrow (two-panel) layout there is no status panel, so the
        # engage/servo indicators live here on the right. In the wide layout
        # the status panel owns them and this space stays clean.
        if w >= 40:
            self._draw_indicators(y, x + 24)

    def _draw_indicators(self, y: int, ix: int) -> None:
        st = self.state
        ap_attr = (
            curses.color_pair(C_OK) | curses.A_BOLD | curses.A_REVERSE
            if st.autopilot_engaged
            else curses.color_pair(C_DIM)
        )
        self._put(y + 1, ix, " AUTOPILOT " if st.autopilot_engaged else " autopilot ", ap_attr)

        sv_attr = (
            curses.color_pair(C_OK) | curses.A_BOLD | curses.A_REVERSE
            if st.servo_enabled
            else curses.color_pair(C_DIM)
        )
        self._put(y + 2, ix, "   SERVO   " if st.servo_enabled else "   servo   ", sv_attr)

        if st.left_turn:
            self._put(y + 4, ix, " << TURN L ", curses.color_pair(C_WARN) | curses.A_BOLD)
        elif st.right_turn:
            self._put(y + 4, ix, " TURN R >> ", curses.color_pair(C_WARN) | curses.A_BOLD)

    def _draw_pid_panel(self, y: int, x: int, w: int) -> None:
        """
        Wide-layout only. Live PID state: the three gains (with the selected one
        highlighted for tuning) and the P/I/D term contributions in shaft
        counts, plus the raw integral/derivative/error. Gains are adjusted with
        the tuning keys (see the footer); the selected gain is chosen with TAB.
        """
        st = self.state
        self._box(y, x, 9, w, "PID")

        kp = st.signal_value("pid_kp")
        ki = st.signal_value("pid_ki")
        kd = st.signal_value("pid_kd")
        p = st.signal_value("pid_p")
        i = st.signal_value("pid_i")
        d = st.signal_value("pid_d")
        integral = st.signal_value("pid_integral")
        deriv = st.signal_value("pid_derivative")
        err = st.signal_value("heading_error")
        if err is None:
            err = self.state.heading_error

        sel = getattr(self, "_pid_selected", "Kp")  # which gain the keys adjust

        def gain_line(row, label, gain, term, key):
            # highlight the selected gain
            selected = (label == sel)
            lab_attr = (curses.color_pair(C_ACCENT) | curses.A_BOLD if selected
                        else curses.color_pair(C_DIM))
            marker = ">" if selected else " "
            self._put(y + row, x + 2, f"{marker}{label}", lab_attr)
            self._put(y + row, x + 6, self._fmt(gain, "{:.2f}", 7),
                      curses.color_pair(C_OK) | (curses.A_BOLD if selected else 0))
            # the term contribution (shaft counts) this gain is producing now
            self._put(y + row, x + 15, "term", curses.color_pair(C_DIM))
            self._put(y + row, x + 20, self._fmt(term, "{:+.0f}", 7),
                      curses.color_pair(C_OK))

        # header row: column labels
        self._put(y + 1, x + 6, "gain", curses.color_pair(C_DIM))
        gain_line(2, "Kp", kp, p, "p")
        gain_line(3, "Ki", ki, i, "i")
        gain_line(4, "Kd", kd, d, "d")

        # raw internals, useful when tuning
        self._put(y + 5, x + 2, "err", curses.color_pair(C_DIM))
        self._put(y + 5, x + 6, self._fmt(err, "{:+.1f}", 7), curses.color_pair(C_OK))
        self._put(y + 5, x + 15, "intg", curses.color_pair(C_DIM))
        self._put(y + 5, x + 20, self._fmt(integral, "{:+.0f}", 7),
                  curses.color_pair(C_OK))
        self._put(y + 6, x + 2, "der", curses.color_pair(C_DIM))
        self._put(y + 6, x + 6, self._fmt(deriv, "{:+.2f}", 7),
                  curses.color_pair(C_OK))
        # sum of terms = commanded offset from center
        total = None
        if p is not None and i is not None and d is not None:
            total = p + i + d
        self._put(y + 6, x + 15, "sum", curses.color_pair(C_DIM))
        self._put(y + 6, x + 20, self._fmt(total, "{:+.0f}", 7),
                  curses.color_pair(C_ACCENT) | curses.A_BOLD)
        # hint
        self._put(y + 7, x + 2, "TAB sel  +/- adjust",
                  curses.color_pair(C_DIM))

    def _draw_status_panel(self, y: int, x: int, w: int) -> None:
        """
        Wide-layout only. Heading-source lock state (item 5) plus the engage
        and servo indicators, given room to breathe.
        """
        st = self.state
        self._box(y, x, 9, w, "STATUS")

        # Heading source is the headline: COG lock is what lets the autopilot
        # steer this boat, so it gets the top line in a color that reads at a
        # glance -- green locked, amber bridging on compass, red none.
        src = st.heading_source
        if src == "COG":
            src_attr = curses.color_pair(C_OK) | curses.A_BOLD
            src_txt = "COG LOCK"
        elif src == "COMPASS":
            src_attr = curses.color_pair(C_WARN) | curses.A_BOLD
            src_txt = "COMPASS (weak)"
        else:
            src_attr = curses.color_pair(C_ERR) | curses.A_BOLD
            src_txt = "NO LOCK - wait for way"
        self._put(y + 1, x + 2, "HEADING SRC", curses.color_pair(C_DIM))
        self._put(y + 2, x + 2, src_txt, src_attr)

        # Engage + servo indicators
        ap_attr = (
            curses.color_pair(C_OK) | curses.A_BOLD | curses.A_REVERSE
            if st.autopilot_engaged
            else curses.color_pair(C_DIM)
        )
        self._put(y + 4, x + 2, " AUTOPILOT " if st.autopilot_engaged else " autopilot ", ap_attr)
        sv_attr = (
            curses.color_pair(C_OK) | curses.A_BOLD | curses.A_REVERSE
            if st.servo_enabled
            else curses.color_pair(C_DIM)
        )
        self._put(y + 4, x + 15, "  SERVO  " if st.servo_enabled else "  servo  ", sv_attr)

        if st.left_turn:
            self._put(y + 6, x + 2, " << TURN L ", curses.color_pair(C_WARN) | curses.A_BOLD)
        elif st.right_turn:
            self._put(y + 6, x + 2, " TURN R >> ", curses.color_pair(C_WARN) | curses.A_BOLD)

        self._put(
            y + 7, x + 2,
            f"COG ok {st.cog_accepted} rej {st.cog_rejected}",
            curses.color_pair(C_DIM),
        )

    def _draw_ekf_strip(self, y: int, x: int, cols: int) -> None:
        """
        One-line EKF summary: inputs, outputs, and diagnostics. Shows the
        operator what the filter is doing -- its heading/yaw-rate output, its
        confidence (sigma), the learned bias (3-state), which inputs it has,
        and how many yaw-rate rejects it has made.
        """
        st = self.state
        h = st.signal_value("ekf_heading")
        rate = st.signal_value("ekf_yaw_rate")
        sigma = st.signal_value("ekf_sigma")
        bias = st.signal_value("ekf_yaw_bias")
        states = st.signal_value("ekf_states")
        rejects = st.signal_value("ekf_rejects")

        # inputs present
        has_comp = st.signal_value("compass_heading") is not None
        has_cog = st.signal_value("cog") is not None
        has_gyro = st.signal_value("node_yaw_rate") is not None

        parts = ["EKF"]
        parts.append(f"H {h:6.1f}" if h is not None else "H   ---")
        parts.append(f"rate {rate:+5.1f}/s" if rate is not None else "rate  ---")
        if sigma is not None:
            parts.append(f"sig {sigma:4.1f}")
        if bias is not None:
            parts.append(f"bias {bias:+4.1f}")
        parts.append(f"[{states or '?'}st]")
        # input indicators
        inp = ("C" if has_comp else "-") + ("G" if has_cog else "-") + ("Y" if has_gyro else "-")
        parts.append(f"in:{inp}")
        if rejects:
            parts.append(f"rej {rejects}")

        line = "  ".join(parts)
        # color sigma-driven: green if confident, warn/err as it rises
        attr = (curses.color_pair(C_OK) if (sigma is not None and sigma < 6)
                else curses.color_pair(C_WARN) if (sigma is not None and sigma < 15)
                else curses.color_pair(C_DIM))
        self._put(y, x + 1, line[: cols - 2], attr)

    def _draw_sources_panel(self, y: int, x: int, w: int, now: float) -> None:
        st = self.state
        with st._lock:
            srcs = list(st.sources.values())
            claims = dict(st.claims)
        h = getattr(self, "_src_h", min(3 + len(srcs), 12))
        self._box(y, x, h, w, "CAN SOURCES")

        # Full 8-column J1939 table when there is room (~138+ cols); otherwise
        # the compact view for an 80-col terminal.
        wide = w >= 136
        row = y + 1
        if wide:
            hdr = (f"{'ID':<10} {'DATA':<16} {'NAME':<16} {'AGE':>7} "
                   f"{'CNT':>5} {'CLASS':<12} {'FUNCTION':<14} {'MFG':<12} {'IDENT':>8}")
            self._put(row, x + 2, hdr, curses.color_pair(C_DIM))
            row += 1
            for s in srcs:
                if row >= y + h - 1:
                    break
                age = s.age(now)
                status = s.status(now)
                attr = (curses.color_pair(C_OK) if status == "OK"
                        else curses.color_pair(C_ERR) | curses.A_BOLD)
                cl = claims.get(s.can_id & 0xFF, {})
                # Show the status word, not just color: color alone is invisible
                # to a color-blind operator and does not show up in a text
                # capture. A trailing marker keeps STALE/NEVER legible.
                if status == "OK":
                    age_txt = " never" if age == float("inf") else f"{age:6.2f}s"
                else:
                    age_txt = status  # STALE or NEVER
                data_txt = (s.last_data.hex().upper()[:16]) if s.last_data else ""
                line = (
                    f"{s.can_id:08X}  "
                    f"{data_txt:<16} "
                    f"{s.name[:16]:<16} "
                    f"{age_txt:>7} "
                    f"{s.count:>5} "
                    f"{str(cl.get('vehicle_system',''))[:12]:<12} "
                    f"{str(cl.get('function',''))[:14]:<14} "
                    f"{str(cl.get('manufacturer',''))[:12]:<12} "
                    f"{str(cl.get('identity','')):>8}"
                )
                self._put(row, x + 2, line[: w - 4], attr)
                row += 1
        else:
            # Compact table, now with the address-claim columns the operator
            # wants visible without needing a 136-col terminal: MFG, FUNCTION,
            # and IDENT (the unique serial). Columns truncate to fit the width;
            # claim fields are blank until that source sends an address claim.
            hdr = (f"{'ID':<11}{'NAME':<14}{'AGE':>7} {'STATUS':<7} "
                   f"{'CNT':>5}  {'MFG':<12}{'FUNCTION':<14}{'IDENT':>9}")
            self._put(row, x + 2, hdr[: w - 4], curses.color_pair(C_DIM))
            row += 1
            for s in srcs:
                if row >= y + h - 1:
                    break
                age = s.age(now)
                status = s.status(now)
                attr = (curses.color_pair(C_OK) if status == "OK"
                        else curses.color_pair(C_ERR) | curses.A_BOLD)
                age_txt = " never" if age == float("inf") else f"{age:6.2f}s"
                cl = claims.get(s.can_id & 0xFF, {})
                mfg = str(cl.get("manufacturer", ""))[:11]
                func = str(cl.get("function", ""))[:13]
                ident = str(cl.get("identity", ""))
                # ID + NAME + AGE + STATUS + COUNT in the source color, then the
                # claim columns. Keep it one _put so it truncates cleanly.
                line = (
                    f"{('0x%08X' % s.can_id):<11}"
                    f"{s.name[:13]:<14}"
                    f"{age_txt:>7} "
                    f"{status:<7} "
                    f"{s.count:>5}  "
                    f"{mfg:<12}"
                    f"{func:<14}"
                    f"{ident:>9}"
                )
                self._put(row, x + 2, line[: w - 4], attr)
                row += 1

    def _draw_faults(self, y: int, cols: int) -> None:
        st = self.state
        faults = st.active_faults()
        if not faults:
            self._put(y, 1, "FAULTS: none", curses.color_pair(C_OK))
            return
        worst = faults[0]
        txt = f"FAULTS: {worst.code}"
        if worst.detail:
            txt += f" -- {worst.detail}"
        if len(faults) > 1:
            txt += f"   (+{len(faults)-1} more: {', '.join(f.code for f in faults[1:])})"
        self._put(y, 1, txt[: cols - 2], curses.color_pair(C_ERR) | curses.A_BOLD)

    def _draw_events(self, y: int, cols: int, height: int) -> None:
        if height < 2:
            return
        self._put(y, 1, "EVENTS", curses.color_pair(C_DIM) | curses.A_BOLD)
        events = self.state.recent_events(height - 1)
        row = y + 1
        for ts, kind, text in events:
            if row >= y + height:
                break
            stamp = time.strftime("%H:%M:%S", time.localtime(ts))
            attr = {
                "FAULT": curses.color_pair(C_ERR),
                "CLEAR": curses.color_pair(C_OK),
                "GOAL": curses.color_pair(C_ACCENT),
                "LINK": curses.color_pair(C_WARN),
                "WARN": curses.color_pair(C_WARN),
            }.get(kind, curses.color_pair(C_NORMAL))
            self._put(row, 1, stamp, curses.color_pair(C_DIM))
            self._put(row, 10, kind.ljust(6), attr)
            self._put(row, 17, text[: cols - 18], attr)
            row += 1

    def _draw_buttons(self, y: int, cols: int) -> None:
        """
        Touch button bar. Each button records its screen span in self._buttons
        so a mouse/touch event can be mapped back to a command. The same
        actions are on the keyboard; this is the touchscreen affordance.

        Buttons reflect state: engage is disabled without a COG lock (the
        thing that most often surprises the operator), and manual steps are
        disabled while engaged, matching the command handler's rules.
        """
        st = self.state
        engaged = st.autopilot_engaged
        can_engage = st.cog_lock and not engaged
        can_manual = not engaged

        # (label, command, enabled)
        specs = [
            ("< STEP", "manual_step_left", can_manual),
            ("STEP >", "manual_step_right", can_manual),
            (" SNAP ", "snap_goal", True),
            ("ENGAGE" if not engaged else "  ON  ", "autopilot_enable", can_engage),
            ("DISENG", "autopilot_disable", engaged),
            ("SERVO", "servo_toggle", True),
            ("REQ ADDR", "request_addresses", True),
            ("CLR FLT", "clear_faults", True),
            (" QUIT ", "quit", True),
        ]

        x = 1
        for label, cmd, enabled in specs:
            text = f" {label} "
            if x + len(text) >= cols - 1:
                break
            # color_pair requires an initialized screen. Guard it so button
            # geometry can be computed in tests without a live curses context;
            # the geometry (positions, enabled flags) is what matters for
            # touch dispatch, and it must not depend on drawing succeeding.
            try:
                if enabled:
                    attr = curses.color_pair(C_HEAD) | curses.A_REVERSE | curses.A_BOLD
                else:
                    attr = curses.color_pair(C_DIM) | curses.A_DIM
                self._put(y, x, text, attr)
            except curses.error:
                pass
            # Record the touch target even when disabled, so a tap gives
            # feedback ("engage blocked") rather than doing nothing silently.
            self._buttons.append((y, x, x + len(text), cmd, enabled))
            x += len(text) + 1

    def _on_touch(self, my: int, mx: int) -> None:
        """Map a mouse/touch coordinate to a button command."""
        for (by, bx0, bx1, cmd, enabled) in self._buttons:
            if my == by and bx0 <= mx < bx1:
                if not enabled:
                    self.flash("action unavailable right now", 2.0)
                    return
                self._dispatch_button(cmd)
                return

    def _dispatch_button(self, cmd: str) -> None:
        if cmd == "snap_goal":
            hv = self.state.signal_value("fused_heading")
            if hv is None:
                hv = self.state.signal_value("compass_heading")
            if hv is not None:
                self.on_goal_set(float(hv))
                self.flash(f"goal snapped to {float(hv):.1f}")
            else:
                self.flash("no heading available")
        elif cmd == "servo_toggle":
            self.on_command(
                "servo_disable" if self.state.servo_enabled else "servo_enable"
            )
        elif cmd == "clear_faults":
            self.on_command("clear_faults")
            self.flash("faults cleared")
        else:
            self.on_command(cmd)

    def _draw_footer(self, y: int, cols: int, now: float) -> None:
        if now < self._flash_until:
            self._put(y, 0, " " * (cols - 1), curses.color_pair(C_HEAD))
            self._put(y, 1, self._flash_msg[: cols - 2],
                      curses.color_pair(C_HEAD) | curses.A_BOLD)
            return
        # Reflects the current key map: arrows +-1, brackets +-10, down -5,
        # up snaps to heading, comma/dot are manual motor steps.
        keys = (
            "<-/-> 1  [/] 10  ^ snap  ,/. step  TAB/+/- pid  "
            "[e]ng [d]is [s]rv [c]lr [q]uit"
        )
        self._put(y, 0, " " * (cols - 1), curses.color_pair(C_HEAD))
        self._put(y, 1, keys[: cols - 2], curses.color_pair(C_HEAD))
