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

    # -- lifecycle ---------------------------------------------------------

    def start(self, scr) -> None:
        self.scr = scr
        self.running = True
        curses.curs_set(0)
        scr.nodelay(True)
        scr.keypad(True)
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
            self._handle_key(ch)

    def _handle_key(self, ch: int) -> None:
        if ch == curses.KEY_LEFT:
            self.on_goal_delta(-1.0)
        elif ch == curses.KEY_RIGHT:
            self.on_goal_delta(+1.0)
        elif ch == curses.KEY_SLEFT or ch == ord(","):
            self.on_goal_delta(-10.0)
        elif ch == curses.KEY_SRIGHT or ch == ord("."):
            self.on_goal_delta(+10.0)
        elif ch == curses.KEY_UP:
            self.on_goal_delta(+5.0)
        elif ch == curses.KEY_DOWN:
            self.on_goal_delta(-5.0)
        elif ch == curses.KEY_HOME or ch == ord("h"):
            hv = self.state.signal_value("fused_heading")
            if hv is None:
                hv = self.state.signal_value("compass_heading")
            if hv is not None:
                self.on_goal_set(float(hv))
                self.flash(f"goal snapped to {float(hv):.1f}")
            else:
                self.flash("no heading available")
        elif ch in (ord("e"), ord("E")):
            self.on_command("autopilot_enable")
        elif ch in (ord("d"), ord("D")):
            self.on_command("autopilot_disable")
        elif ch in (ord("s"), ord("S")):
            self.on_command(
                "servo_disable" if self.state.servo_enabled else "servo_enable"
            )
        elif ch == ord("["):
            self.on_command("rudder_left")
        elif ch == ord("]"):
            self.on_command("rudder_right")
        elif ch in (ord("c"), ord("C")):
            self.on_command("clear_faults")
            self.flash("faults cleared")
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
        self.scr.erase()

        self._draw_header(0, cols, now)
        self._draw_heading_panel(2, 0, 38)
        self._draw_rudder_panel(2, 39, cols - 39)
        self._draw_sources_panel(11, 0, cols, now)
        log_top = 11 + min(3 + len(st.sources), 9)
        self._draw_faults(log_top, cols)
        self._draw_events(log_top + 2, cols, rows - (log_top + 2) - 2)
        self._draw_footer(rows - 1, cols, now)

        self.scr.refresh()

    def _draw_header(self, y: int, cols: int, now: float) -> None:
        st = self.state
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

        self._put(y, 20, link_txt, link_attr)

        rate_txt = f"{st.bitrate // 1000}k  RX {st.rx_total}  TX {st.tx_total}"
        self._put(y, 46, rate_txt, curses.color_pair(C_HEAD))

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

        # Engage indicators on the right of this panel
        ix = x + 22
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

        # COG accept/reject from the filter
        self._put(
            y + 6, ix,
            f"COG ok {st.cog_accepted} rej {st.cog_rejected}",
            curses.color_pair(C_DIM),
        )

    def _draw_sources_panel(self, y: int, x: int, w: int, now: float) -> None:
        st = self.state
        with st._lock:
            srcs = list(st.sources.values())
        h = min(3 + len(srcs), 9)
        self._box(y, x, h, w, "CAN SOURCES")

        row = y + 1
        self._put(row, x + 2, "ID          NAME                AGE     STATUS      COUNT",
                  curses.color_pair(C_DIM))
        row += 1
        for s in srcs:
            if row >= y + h - 1:
                break
            age = s.age(now)
            status = s.status(now)
            attr = (
                curses.color_pair(C_OK) if status == "OK"
                else curses.color_pair(C_ERR) | curses.A_BOLD
            )
            age_txt = "  never" if age == float("inf") else f"{age:6.2f}s"
            self._put(row, x + 2, f"0x{s.can_id:08X}", curses.color_pair(C_DIM))
            self._put(row, x + 14, s.name[:18].ljust(18))
            self._put(row, x + 34, age_txt, attr)
            self._put(row, x + 43, status.ljust(10), attr)
            self._put(row, x + 55, str(s.count), curses.color_pair(C_DIM))
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

    def _draw_footer(self, y: int, cols: int, now: float) -> None:
        if now < self._flash_until:
            self._put(y, 0, " " * (cols - 1), curses.color_pair(C_HEAD))
            self._put(y, 1, self._flash_msg[: cols - 2],
                      curses.color_pair(C_HEAD) | curses.A_BOLD)
            return
        keys = (
            "<-/-> +-1  ,/. +-10  ^/v +-5  [Home] snap  "
            "[e]ngage [d]isen [s]ervo [c]lr [q]uit"
        )
        self._put(y, 0, " " * (cols - 1), curses.color_pair(C_HEAD))
        self._put(y, 1, keys[: cols - 2], curses.color_pair(C_HEAD))
