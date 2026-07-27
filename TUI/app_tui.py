#!/usr/bin/env python3
"""
app_tui.py -- RemoteRudder HMI: Tornado web server + curses console.

Run:
    python3 app_tui.py                    # live socketcan on can0
    python3 app_tui.py --offline          # replay a candump log on a virtual bus
    python3 app_tui.py --no-tui           # server only, plain logging
    python3 app_tui.py --channel can1 --bitrate 250000

Architecture
------------
One asyncio event loop owns everything:

    CAN reader  --(callback)-->  Bridge  -->  SystemState
                                                 |
                            +--------------------+--------------------+
                            |                                         |
                    TUI render tick (12 Hz)                  WS broadcast (5 Hz)

SystemState is the single writer-owned truth. Neither the TUI nor the browser
holds its own copy of the heading goal; both read it back after a write. That
is what makes "phone moves the goal, TUI shows it immediately" work without
any direct coupling between the two.

Last-write-wins on the goal, with the source recorded so the TUI can show
who moved it.

The curses TUI runs on the same loop rather than a thread. Rendering is a few
hundred addstr calls at 12 Hz, which is microseconds of work; keeping it on
the loop avoids needing a lock around curses, which is not thread-safe.
"""

from __future__ import annotations

import argparse
import asyncio
import functools
import json
import logging
import os
import signal
import struct
import sys
import time
from typing import Optional, Set

import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.websocket

from can_interface import CANinterface
from autopilot import Autopilot
from can_recorder import FilteredCanRecorder
from hmi_bridge import Bridge
from j1939_name import (
    REQUEST_ADDRESS_CLAIM_DATA,
    build_request_for_address_claim,
    decode_name,
    is_address_claim,
    source_address,
)
from hmi_canlink import CANLink
from hmi_heading import HeadingMonitor
from hmi_state import SystemState

# --------------------------------------------------------------------------
# Logging: when the TUI owns the terminal, log to a file instead of stderr,
# otherwise curses and the log fight over the screen.
# --------------------------------------------------------------------------

logger = logging.getLogger("hmi")

# Encoder counts per manual step (. and , keys). The steering module's shaft
# goal is in raw encoder counts, not degrees -- the "counts" label the operator
# uses for the encoder result. Center is ~1425 counts; a full sweep spans a few
# hundred counts, so 10 counts is a usable nudge. Manual steps are
# disengaged-only.
MANUAL_STEP_COUNTS = 10


def setup_logging(use_tui: bool, level: str = "INFO") -> None:
    root = logging.getLogger()
    root.setLevel(getattr(logging, level.upper(), logging.INFO))
    for h in list(root.handlers):
        root.removeHandler(h)

    fmt = logging.Formatter(
        "%(asctime)s %(levelname)-7s %(name)-10s %(message)s", "%H:%M:%S"
    )

    if use_tui:
        os.makedirs("logs", exist_ok=True)
        fh = logging.FileHandler(
            os.path.join("logs", f"hmi_{time.strftime('%Y%m%d_%H%M%S')}.log")
        )
        fh.setFormatter(fmt)
        root.addHandler(fh)
    else:
        sh = logging.StreamHandler(sys.stderr)
        sh.setFormatter(fmt)
        root.addHandler(sh)


# --------------------------------------------------------------------------
# Web layer
# --------------------------------------------------------------------------

clients: Set["WSHandler"] = set()
STATE = SystemState()

# Filtered CAN recorder: logs only the messages the HMI uses (~17% of the bus),
# so an on-water capture is small and shareable. Same recorder app.py had; kept
# here so app_tui.py is the single complete app (web + TUI + recorder).
RECORDER = FilteredCanRecorder(directory="logs", channel="can0")


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("autopilot.html")


class PlotHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("plot_data.html")


class LogDownloadHandler(tornado.web.RequestHandler):
    """Serve the current filtered CAN log for download (path shown on /plot)."""

    def get(self):
        stats = RECORDER.stats()
        path = stats.get("path")
        if not path or not os.path.exists(path):
            self.set_status(404)
            self.write("No log yet")
            return
        self.set_header("Content-Type", "application/octet-stream")
        self.set_header("Content-Disposition",
                        f'attachment; filename="{os.path.basename(path)}"')
        with open(path, "rb") as f:
            self.write(f.read())


class HealthHandler(tornado.web.RequestHandler):
    """Machine-readable status, handy for a watchdog or a second display."""

    def get(self):
        self.set_header("Content-Type", "application/json")
        self.write(json.dumps(STATE.snapshot()))


class ExitBrowserHandler(tornado.web.RequestHandler):
    def post(self):
        import subprocess

        try:
            subprocess.Popen(["pkill", "chromium"])
            self.write("Browser closed")
        except (OSError, FileNotFoundError) as e:
            logger.warning("could not close browser: %s", e)
            self.set_status(500)
            self.write(f"failed: {e}")


class WSHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        clients.add(self)
        STATE.web_clients = len(clients)
        peer = self.request.remote_ip
        self._peer = peer
        STATE.log_event("WEB", f"connect {peer} ({len(clients)} total)")
        # Send an immediate snapshot so a phone that just connected is not
        # blank until the next broadcast tick.
        try:
            self.write_message(json.dumps(STATE.snapshot(), separators=(",", ":")))
        except tornado.websocket.WebSocketClosedError:
            pass

    def on_close(self):
        clients.discard(self)
        STATE.web_clients = len(clients)
        STATE.log_event("WEB", f"disconnect {getattr(self, '_peer', '?')} ({len(clients)} left)")

    def on_message(self, message):
        STATE.web_last_activity = time.time()
        peer = getattr(self, "_peer", "web")
        try:
            data = json.loads(message)
        except (ValueError, TypeError):
            logger.warning("bad WS payload from %s: %r", peer, message[:120])
            return

        try:
            if "command" in data:
                APP.handle_command(str(data["command"]), source=f"web/{peer}")
            elif "heading_goal" in data:
                APP.set_goal(float(data["heading_goal"]), source=f"web/{peer}")
        except (TypeError, ValueError) as e:
            logger.warning("bad WS value from %s: %s", peer, e)
        except Exception:
            logger.exception("WS message handling failed")


def safe_broadcast(payload: dict) -> None:
    msg = json.dumps(payload, separators=(",", ":"))
    dead = []
    for c in list(clients):
        try:
            c.write_message(msg)
        except tornado.websocket.WebSocketClosedError:
            dead.append(c)
        except Exception:
            dead.append(c)
    for c in dead:
        clients.discard(c)
    if dead:
        STATE.web_clients = len(clients)


# --------------------------------------------------------------------------
# Application
# --------------------------------------------------------------------------


class HMIApp:
    def __init__(self, args) -> None:
        self.args = args
        self.state = STATE
        self.monitor = HeadingMonitor(self.state)
        self.bridge = Bridge(self.state, self.monitor)
        # Enable the heading EKF if requested (--ekf). Off by default so it
        # never disturbs the proven priority chain unless asked for.
        if getattr(args, "ekf", False):
            self.bridge.use_ekf = True
        self.can: Optional[CANinterface] = None
        self.autopilot: Optional[Autopilot] = None
        self.link: Optional[CANLink] = None
        self.tui = None
        self.stdscr = None
        self._shutdown = asyncio.Event()
        self._tasks = []

    # -- goal / command entry points --------------------------------------

    def set_goal(self, value: float, source: str) -> None:
        """Single funnel for goal changes. Last write wins."""
        v = self.state.set_goal(value, source)
        if self.autopilot is not None:
            self.autopilot.set_heading_goal(v)
        safe_broadcast({"heading_goal": v, "goal_source": source})

    def adjust_goal(self, delta: float, source: str) -> None:
        self.set_goal(self.state.heading_goal + delta, source)

    def handle_command(self, command: str, source: str = "?") -> None:
        st = self.state
        ap = self.autopilot
        ci = self.can

        if command == "quit":
            self.request_shutdown()
            return

        if command == "clear_faults":
            with st._lock:
                codes = list(st.faults.keys())
            for c in codes:
                st.clear_fault(c)
            return

        if command.startswith("pid_gain_"):
            # pid_gain_up:Kp / pid_gain_down:Ki etc. from the TUI tuning keys.
            if ap is None:
                st.log_event("WARN", "PID adjust ignored, autopilot not ready")
                return
            try:
                direction, which = command[len("pid_gain_"):].split(":")
            except ValueError:
                return
            step = {"Kp": ap.kp_step, "Ki": ap.ki_step,
                    "Kd": ap.kd_step}.get(which, 0.0)
            if direction == "down":
                step = -step
            ap.adjust_gain(which, step)
            st.log_event("PID", f"{which} -> {getattr(ap, which):.2f}")
            return

        if command == "snap_goal":
            # Set the goal to the current heading. Reachable from a phone
            # button; the TUI handles its own snap locally.
            hv = st.signal_value("fused_heading")
            if hv is None:
                hv = st.signal_value("compass_heading")
            if hv is not None:
                self.set_goal(float(hv), source)
            return

        if command == "request_addresses":
            # Broadcast a J1939 Request (PGN 59904) for the Address Claimed PGN
            # (60928). Every node re-announces its NAME, which populates the
            # manufacturer/function columns in the source table -- some nodes
            # (the Garmin heading source among them) do not claim on their own
            # often, so this is the way to identify them.
            if ci is None:
                st.log_event("WARN", "request_addresses ignored, CAN not ready")
                return
            try:
                import can as _can
                req = _can.Message(
                    arbitration_id=build_request_for_address_claim(),
                    data=REQUEST_ADDRESS_CLAIM_DATA,
                    is_extended_id=True,
                )
                ci.bus.send(req)
                st.log_event("CMD", f"requested address claims <- {source}")
                if self.tui:
                    self.tui.flash("requested address claims", 2.5)
            except Exception as e:  # noqa: BLE001
                st.log_event("WARN", f"address request failed: {e}")
            return

        if ap is None or ci is None:
            st.log_event("WARN", f"command '{command}' ignored, CAN not ready")
            return

        if command == "rudder_left":
            ci.adjust_shaft_goal(-5)
        elif command == "rudder_right":
            ci.adjust_shaft_goal(5)
        elif command in ("manual_step_left", "manual_step_right"):
            # Manual steering: nudge the motor a couple degrees. Only when
            # disengaged, so a manual input can never fight the autopilot --
            # the operator disengages first, then steers by hand. This is the
            # fallback for a lost COG lock or a deteriorated sensor: keep the
            # boat under control without the autopilot.
            if st.autopilot_engaged:
                st.log_event("WARN", "manual step ignored: disengage first")
                if self.tui:
                    self.tui.flash("disengage before manual steering", 2.5)
            else:
                # Ensure the servo is live so the step actually moves the motor.
                if not st.servo_enabled:
                    ci.set_servo_enabled(True)
                    st.servo_enabled = True
                step = MANUAL_STEP_COUNTS if command == "manual_step_right" else -MANUAL_STEP_COUNTS
                ci.adjust_shaft_goal(step)
                st.log_event("CMD", f"manual step {'R' if step > 0 else 'L'} {abs(step)} counts <- {source}")
        elif command == "heading_left":
            self.adjust_goal(-1, source)
        elif command == "heading_right":
            self.adjust_goal(+1, source)
        elif command == "servo_enable":
            ci.set_servo_enabled(True)
            st.servo_enabled = True
            st.log_event("CMD", f"servo enable <- {source}")
        elif command == "servo_disable":
            ci.set_servo_enabled(False)
            st.servo_enabled = False
            ap.autopilot_engaged = False
            ap.autopilot_engaged_event.clear()
            st.autopilot_engaged = False
            st.log_event("CMD", f"servo disable <- {source}")
        elif command == "autopilot_enable":
            ok, why = self.monitor.ok_to_engage()
            if not ok:
                st.log_event("BLOCK", f"engage refused: {why}")
                if self.tui:
                    self.tui.flash(f"ENGAGE BLOCKED: {why}", 4.0)
                safe_broadcast({"engage_blocked": why})
                return
            # This hull only steers with way on and a trusted COG. Refuse to
            # engage without a lock rather than centering into a heading the
            # boat cannot hold.
            if not st.cog_lock:
                msg = "no COG lock -- need forward motion to engage"
                st.log_event("BLOCK", f"engage refused: {msg}")
                if self.tui:
                    self.tui.flash(f"ENGAGE BLOCKED: {msg}", 4.0)
                safe_broadcast({"engage_blocked": msg})
                return
            # Engage must start from ZERO error: seed the goal to the current,
            # ACCURATE heading and pin the controller's current_heading to that
            # same value, so the first PID cycle sees error = 0 and the boat
            # holds its present course instead of lurching toward a stale or
            # slightly-skewed setpoint.
            #
            # Read the heading with FRESHNESS (not signal_value, which returns
            # the last value even when stale) so we never seed the goal from an
            # outdated heading. cog_lock is already required above, so a fresh
            # fused heading should be present; guard anyway.
            hsig = st.get_signal("fused_heading")
            hv = hsig.value if (hsig is not None and hsig.value is not None
                                and hsig.is_valid()) else None
            if hv is None:
                msg = "no fresh heading -- cannot engage from zero error"
                st.log_event("BLOCK", f"engage refused: {msg}")
                if self.tui:
                    self.tui.flash(f"ENGAGE BLOCKED: {msg}", 4.0)
                safe_broadcast({"engage_blocked": msg})
                return
            hv = float(hv)
            # Seed goal and current_heading to the SAME value, atomically w.r.t.
            # the PID, and clear any integral windup so the loop starts clean.
            self.set_goal(hv, f"engage/{source}")
            ap.heading_goal = hv
            ap.current_heading = hv
            ap.heading_error = 0.0
            if hasattr(ap, "reset_integral"):
                ap.reset_integral()
            ap.autopilot_engaged = True
            ap.autopilot_engaged_event.set()
            st.autopilot_engaged = True
            st.log_event("CMD", f"AUTOPILOT ENGAGED <- {source} (hold {hv:.1f}, err 0)")
        elif command == "autopilot_disable":
            ap.autopilot_engaged = False
            ap.autopilot_engaged_event.clear()
            st.autopilot_engaged = False
            st.log_event("CMD", f"autopilot disengaged <- {source}")
        elif command == "start_turn_left":
            ap.left_turn_engaged = True
            ap.right_turn_engaged = False
            st.left_turn, st.right_turn = True, False
        elif command == "start_turn_right":
            ap.right_turn_engaged = True
            ap.left_turn_engaged = False
            st.right_turn, st.left_turn = True, False
        elif command == "stop_turn_left":
            ap.left_turn_engaged = False
            st.left_turn = False
        elif command == "stop_turn_right":
            ap.right_turn_engaged = False
            st.right_turn = False
        else:
            st.log_event("WARN", f"unknown command '{command}' from {source}")

    # -- CAN supervision ---------------------------------------------------

    async def can_supervisor(self) -> None:
        """
        Keeps the bus open. On failure, retries with backoff forever rather
        than exiting the process. This is the fix for the old sys.exit() on
        a missing adapter: unplugging the PCAN now degrades the HMI to a
        display-only state that recovers when you plug it back in.
        """
        backend = "virtual" if self.args.offline else self.args.backend

        while not self._shutdown.is_set():
            if self.link is None:
                self.link = CANLink(
                    self.state,
                    channel=self.args.channel,
                    bitrate=self.args.bitrate,
                    backend=backend,
                )

            if not self.state.link_up:
                if self.link.open():
                    await self._start_can_services()
                else:
                    delay = self.link.next_backoff()
                    try:
                        await asyncio.wait_for(self._shutdown.wait(), timeout=delay)
                    except asyncio.TimeoutError:
                        pass
                    continue

            await asyncio.sleep(0.5)

    async def _start_can_services(self) -> None:
        """Construct CANinterface around the already-open bus and start it."""
        try:
            self.can = CANinterface.__new__(CANinterface)
            self.can.bus = self.link.bus
            # Re-run the parts of __init__ that do not open a bus.
            self._init_can_fields(self.can)

            self.autopilot = Autopilot(self.can)
            self.autopilot.start()

            self.can.add_listener(self.bridge.on_decoded)
            self.can.add_listener(lambda d: safe_broadcast(d))

            self._tasks.append(asyncio.create_task(self._read_loop()))

            if self.args.offline:
                path = self.args.replay or DEFAULT_REPLAY
                if os.path.exists(path):
                    self.state.log_event("LINK", f"replay {os.path.basename(path)}")
                    self._tasks.append(
                        asyncio.create_task(
                            self.can.replay_candump(path, speed=self.args.speed, loop=True)
                        )
                    )
                else:
                    self.state.log_event("WARN", f"replay file not found: {path}")
        except Exception:
            logger.exception("failed to start CAN services")
            self.state.log_event("FAULT", "CAN service start failed")
            self.link.close()

    @staticmethod
    def _init_can_fields(ci: CANinterface) -> None:
        """
        Populate CANinterface fields without touching the bus.

        This mirrors CANinterface.__init__ minus the Bus() construction, so
        the link lifecycle can be owned by CANLink. Keep in sync if you add
        fields to the interface.
        """
        import queue
        import time as _t

        ci.listeners = []
        ci.shaft_goal = 1475
        ci.servo_enabled = False
        ci.watch_ids = {
            0x18F01D21: {"name": "steering", "last_time": _t.time()},
            0x19F10D13: {"name": "rudder", "last_time": _t.time()},
            0x09F8021C: {"name": "gps_speed", "last_time": _t.time()},
            0x18FEE81C: {"name": "vehicle_direction", "last_time": _t.time()},
            0x0CF00400: {"name": "engine_control", "last_time": _t.time()},
            0x09F8011C: {"name": "gps_position_rapid", "last_time": _t.time()},
            0x09F112F8: {"name": "vessel_heading", "last_time": _t.time()},
        }
        ci.timeout_interval = 2.0
        ci.GUI_TIMEOUT = 0.35
        ci.rpm_start_time = _t.time()
        ci.compass_heading = 0.0
        ci.rudder_correction = 7.0
        ci.boat_speed = 0.0
        ci.COG = 0
        ci.heading_correction = 7.5
        ci.averaging_window = 100
        ci.heading_history = queue.Queue(maxsize=100)
        ci.COG_history = queue.Queue(maxsize=100)
        ci.max_steering_angle = 2900
        ci.min_steering_angle = 0
        ci.shaft_value = ci.max_steering_angle / 2
        ci.compass_offset = 0.0
        ci.lat = None
        ci.lon = None
        ci.rpm = None

    async def _read_loop(self) -> None:
        """
        Own read loop, replacing CANinterface.read_loop, so that a bus error
        marks the link down and lets the supervisor reopen it instead of
        raising out of a bare task.
        """
        import can

        reader = can.AsyncBufferedReader()
        notifier = can.Notifier(
            self.link.bus, [reader], loop=asyncio.get_running_loop()
        )
        self.state.log_event("LINK", "read loop started")

        try:
            while not self._shutdown.is_set():
                try:
                    msg = await asyncio.wait_for(reader.get_message(), timeout=1.0)
                except asyncio.TimeoutError:
                    continue

                if msg is None:
                    continue

                # Record used frames to the filtered log (small, shareable).
                try:
                    RECORDER.record_message(msg)
                except Exception:
                    pass

                # Freshness must measure *arrival*, not *capture*. A replayed
                # candump carries its original timestamps (months old), which
                # would make every source read as instantly stale and blank
                # the whole display in offline demo mode. Wall clock is the
                # right basis for "how long since I last heard from this node".
                self.state.mark_source(msg.arbitration_id, time.time(), msg.data)

                # J1939 address claims (PGN 60928) carry the node's NAME:
                # manufacturer, function, identity. Decode and store by source
                # address so the source table can show who each node is. This
                # is also what the "request addresses" command populates.
                if is_address_claim(msg.arbitration_id):
                    name = decode_name(msg.data)
                    if name is not None:
                        sa = source_address(msg.arbitration_id)
                        self.state.record_claim(sa, {
                            "manufacturer": name.manufacturer,
                            "mfg_code": name.mfg_code,
                            "function": name.function_name,
                            "function_code": name.function,
                            "vehicle_system": name.vehicle_system_name,
                            "identity": name.identity,
                            "industry": name.industry_name,
                        })
                        self.state.log_event(
                            "CLAIM",
                            f"SA {sa:02X}: {name.manufacturer} / {name.function_name}",
                        )
                        # Update NAME-bound heading source addresses. This is
                        # what lets a heading source keep working after an
                        # address change: the registry re-resolves its address
                        # from every claim.
                        if self.bridge is not None:
                            self.bridge.heading_registry.note_claim(
                                sa, name.mfg_code, name.function, name.identity
                            )

                if msg.arbitration_id in self.can.watch_ids:
                    self.can.watch_ids[msg.arbitration_id]["last_time"] = time.time()

                try:
                    data = self.can.process_message(msg)
                except (struct.error, IndexError, ValueError, KeyError) as e:
                    # A truncated or corrupt frame should not kill the loop.
                    # These are the shapes a bad frame actually takes:
                    # struct.error from a short payload, IndexError from
                    # msg.data[7] on a 4-byte frame, ValueError/KeyError
                    # from an out-of-range field.
                    logger.debug("decode error on 0x%08X: %s", msg.arbitration_id, e)
                    self.state.log_event("WARN", f"bad frame 0x{msg.arbitration_id:08X}")
                    continue
                except Exception:
                    logger.exception("unexpected decode failure on 0x%08X", msg.arbitration_id)
                    continue

                if data:
                    for cb in self.can.listeners:
                        try:
                            cb(data)
                        except Exception:
                            logger.exception("listener failed")
        except (OSError, can.CanError) as e:
            logger.error("read loop bus error: %s", e)
            self.state.raise_fault("LINK_DOWN", f"read loop: {e}")
            self.state.link_up = False
        finally:
            try:
                notifier.stop()
            except Exception:
                pass
            self.state.log_event("LINK", "read loop stopped")

    # -- periodic tasks ----------------------------------------------------

    async def health_tick(self) -> None:
        while not self._shutdown.is_set():
            try:
                if self.link is not None:
                    self.link.poll_health()
                self.bridge.step_ekf()
                self.bridge.derive_fused()
                self.monitor.update_state()
                if self.autopilot is not None:
                    # Push the fused heading and lock state into the autopilot.
                    # This is what actually feeds the control loop -- without
                    # it the autopilot's current_heading never updates. When
                    # fused is None (no steerage way), pass None so the loop
                    # centers and waits.
                    fused = self.state.signal_value("fused_heading")
                    if fused is not None:
                        self.autopilot.set_heading(float(fused))
                    else:
                        self.autopilot.current_heading = None
                    self.autopilot.set_cog_lock(self.state.cog_lock)
                    self.state.heading_error = self.autopilot.heading_error
                    self.state.set_signal("shaft_center", self.autopilot.shaft_center)
                    # Publish live PID state so the TUI PID panel (and the web)
                    # can show the gains and the P/I/D term contributions.
                    ap = self.autopilot
                    self.state.set_signal("pid_kp", ap.Kp)
                    self.state.set_signal("pid_ki", ap.Ki)
                    self.state.set_signal("pid_kd", ap.Kd)
                    self.state.set_signal("pid_p", ap.p_term)
                    self.state.set_signal("pid_i", ap.i_term)
                    self.state.set_signal("pid_d", ap.d_term)
                    self.state.set_signal("pid_integral", ap._integral)
                    self.state.set_signal("pid_derivative", ap.last_derivative)
            except Exception:
                logger.exception("health tick failed")
            await asyncio.sleep(0.25)

    async def broadcast_tick(self, hz: float) -> None:
        period = 1.0 / max(hz, 0.5)
        while not self._shutdown.is_set():
            try:
                if clients:
                    snap = self.state.snapshot()
                    # attach filtered-log status so /plot can show + download it
                    try:
                        rstats = RECORDER.stats()
                        snap["log_path"] = rstats.get("path")
                        snap["log_count"] = rstats.get("count")
                    except Exception:
                        pass
                    safe_broadcast(snap)
            except Exception:
                logger.exception("broadcast failed")
            await asyncio.sleep(period)

    async def tui_tick(self, hz: float) -> None:
        period = 1.0 / max(hz, 1.0)
        while not self._shutdown.is_set():
            try:
                if self.tui is not None:
                    self.tui.poll_keys()
                    self.tui.render()
            except Exception:
                logger.exception("TUI tick failed")
            await asyncio.sleep(period)

    # -- shutdown ----------------------------------------------------------

    def request_shutdown(self) -> None:
        self.state.log_event("SYS", "shutdown requested")
        self._shutdown.set()

    async def shutdown(self) -> None:
        for t in self._tasks:
            t.cancel()
        if self.autopilot is not None:
            try:
                self.autopilot.stop()
            except Exception:
                pass
        if self.can is not None:
            try:
                self.can.set_servo_enabled(False)
            except Exception:
                pass
        if self.link is not None:
            self.link.close()


DEFAULT_REPLAY = os.path.join(
    "logs", "candump-2025-08-06_16442_horsetooth_firstConstants.log"
)

APP: Optional[HMIApp] = None


def make_app() -> tornado.web.Application:
    here = os.path.dirname(os.path.abspath(__file__))
    return tornado.web.Application(
        [
            (r"/", MainHandler),
            (r"/ws", WSHandler),
            (r"/health", HealthHandler),
            (r"/exit-browser", ExitBrowserHandler),
            (r"/plot", PlotHandler),
            (r"/used-can-log", LogDownloadHandler),
            (
                # StaticFileHandler.get() takes a `path` argument that must
                # come from a capture group in the route regex. The previous
                # pattern r"/sw.js" captured nothing, and `default_filename`
                # does not fill the gap -- it only applies to directory
                # requests -- so every fetch raised
                #   TypeError: get() missing 1 required positional argument
                # and returned 500. plot_data.html registers the service
                # worker on load, so this fired on every visit to /plot and
                # silently disabled offline tile caching.
                r"/(sw\.js)",
                tornado.web.StaticFileHandler,
                {"path": os.path.join(here, "static")},
            ),
        ],
        template_path=os.path.join(here, "templates"),
        static_path=os.path.join(here, "static"),
        debug=False,  # autoreload and curses do not mix
    )


async def run(args, stdscr=None) -> None:
    global APP
    APP = HMIApp(args)
    APP.stdscr = stdscr

    if stdscr is not None:
        from hmi_tui import TUI

        APP.tui = TUI(
            APP.state,
            on_goal_delta=lambda d: APP.adjust_goal(d, "tui"),
            on_goal_set=lambda v: APP.set_goal(v, "tui"),
            on_command=lambda c: APP.handle_command(c, source="tui"),
        )
        APP.tui.start(stdscr)

    app = make_app()
    server = tornado.httpserver.HTTPServer(app)
    try:
        server.listen(args.port)
        APP.state.log_event("SYS", f"http listening on :{args.port}")
    except OSError as e:
        APP.state.log_event("FAULT", f"cannot bind port {args.port}: {e}")
        logger.error("cannot bind port %d: %s", args.port, e)
        return

    tasks = [
        asyncio.create_task(APP.can_supervisor()),
        asyncio.create_task(APP.health_tick()),
        asyncio.create_task(APP.broadcast_tick(args.ws_hz)),
    ]
    if stdscr is not None:
        tasks.append(asyncio.create_task(APP.tui_tick(args.tui_hz)))

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, APP.request_shutdown)
        except (NotImplementedError, RuntimeError):
            pass  # Windows

    await APP._shutdown.wait()
    await APP.shutdown()
    for t in tasks:
        t.cancel()
    server.stop()


def main() -> None:
    p = argparse.ArgumentParser(description="RemoteRudder HMI")
    p.add_argument("--channel", default="can0", help="CAN channel (can0, PCAN_USBBUS1)")
    p.add_argument("--bitrate", type=int, default=250000)
    p.add_argument(
        "--backend",
        default=None,
        choices=[None, "socketcan", "pcan", "virtual"],
        help="force a python-can backend; default autodetects by platform",
    )
    p.add_argument("--offline", action="store_true", help="virtual bus + log replay")
    p.add_argument("--replay", default=None, help="candump log to replay")
    p.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--no-tui", action="store_true", help="server only")
    p.add_argument("--tui-hz", type=float, default=12.0)
    p.add_argument("--ws-hz", type=float, default=5.0)
    p.add_argument("--log-level", default="INFO")
    p.add_argument("--ekf", action="store_true",
                   help="enable the heading EKF (off by default; feeds the EKF strip)")
    p.add_argument("--version", action="store_true", help="print version and exit")
    args = p.parse_args()

    if args.version:
        from version import version_banner
        print(version_banner())
        return

    use_tui = not args.no_tui
    setup_logging(use_tui, args.log_level)

    if use_tui:
        import curses

        def _wrapped(stdscr):
            asyncio.run(run(args, stdscr))

        try:
            curses.wrapper(_wrapped)
        except KeyboardInterrupt:
            pass
    else:
        try:
            asyncio.run(run(args, None))
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
