#!/usr/bin/env python3
# sudo apt install python3-tornado


# Test with vcan 
# canplayer -l i -v -I logs/test_data-21July2025.log vcan0=can0

#
import time
import asyncio
import sys
import tornado.web
import tornado.websocket
import tornado.ioloop
import tornado.httpserver
import tornado.autoreload
import os
import json
import logging
import subprocess

# ---- OFFLINE TEST SETTINGS (hardcoded) ----
OFFLINE_TEST = True
OFFLINE_BACKEND = "virtual"        # use python-can virtual bus
OFFLINE_CHANNEL = "vcan0"          # arbitrary name for virtual bus
CANDUMP_PATH = r"logs\test_data-21July2025.log"   # <-- set your file here
CANDUMP_PATH = r"logs\candump-2025-08-06_16442_horsetooth_firstConstants.log"
REPLAY_SPEED = 1.0                 # 1.0 = realtime
REPLAY_LOOP = True                 # loop file


sys.stdout.flush()

from can_interface import CANinterface
from autopilot import Autopilot
from heading_ekf import HeadingEKF2, HeadingEKF3, _angdiff
from heading_fusion import YawRateLimiter, YAW_RATE_MAX_DPS
from hmi_bridge import COG_PRIMARY_SPEED_MPH
from can_recorder import FilteredCanRecorder

# Setup logger
logger = logging.getLogger('autopilot')
logging.basicConfig(level=logging.INFO)

clients = set()
can_interface = CANinterface(channel='can0', bitrate=250000, backend=OFFLINE_BACKEND if OFFLINE_TEST else None )

# ---- Heading EKF (runs on the web server's CAN interface) ----
# Fuses the compass yaw-rate derivative with COG, per the fusion design. Uses
# the 3-state variant if a node/24xd gyro yaw rate is available, else 2-state.
# Off by default; set EKF_ENABLED to feed the live graphs and telemetry.
EKF_ENABLED = True
_ekf = None
_ekf_last_t = None
_compass_prev = None            # (value, t) for the compass derivative
_ekf_yaw_limiter = YawRateLimiter()

# ---- Filtered CAN recorder: logs only the messages the HMI uses ----
_recorder = FilteredCanRecorder(directory="logs", channel="can0")
autopilot = Autopilot(can_interface)
autopilot.start()


class LogDownloadHandler(tornado.web.RequestHandler):
    """Serve the current filtered CAN log for download."""
    def get(self):
        stats = _recorder.stats()
        path = stats["path"]
        if not path or not os.path.exists(path):
            self.set_status(404)
            self.write("No log yet")
            return
        self.set_header("Content-Type", "application/octet-stream")
        self.set_header("Content-Disposition",
                        f'attachment; filename="{os.path.basename(path)}"')
        with open(path, "rb") as f:
            self.write(f.read())


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("autopilot.html")

class PlotHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("plot_data.html")

class ExitBrowserHandler(tornado.web.RequestHandler):
    def post(self):
        # Kill Chromium process
        subprocess.Popen(["pkill", "chromium"])
        self.write("Browser closed")

class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        clients.add(self)

    def on_close(self):
        clients.discard(self)

    def check_origin(self, origin):
        return True  # allow connections from any origin
    
    def on_message(self, message):
        try:
            data = json.loads(message)
            if "command" in data:
                handle_client_command(data["command"])
            elif "heading_goal" in data:
                autopilot.set_heading_goal(data["heading_goal"])
                broadcast_can_message(data={"heading_goal": autopilot.heading_goal})
        except Exception as e:
            logger.exception("Failed to process WebSocket message")

def handle_client_command(command):
    logger.info(f"Handling command: {command}")
    if command == "rudder_left":
        can_interface.adjust_shaft_goal(-5)
    elif command == "rudder_right":
        can_interface.adjust_shaft_goal(5)
    elif command == "heading_right":
        autopilot.adjust_heading_goal(+1)
        broadcast_can_message(data={"heading_goal": autopilot.heading_goal})
    elif command == "heading_left":
        autopilot.adjust_heading_goal(-1)
        broadcast_can_message(data={"heading_goal": autopilot.heading_goal})
    elif command == "servo_enable":
        can_interface.set_servo_enabled(True)
    elif command == "servo_disable":
        can_interface.set_servo_enabled(False)
        autopilot.autopilot_engaged = False
    elif command == "autopilot_enable":
        autopilot.autopilot_engaged = True
        autopilot.autopilot_engaged_event.set()
        broadcast_can_message(data={"heading_goal": autopilot.heading_goal})
        logger.info("Received Request to Enable Autopilot.")
    elif command == "autopilot_disable":
        autopilot.autopilot_engaged = False
        autopilot.autopilot_engaged_event.clear()
        logger.info("Received Request to Disable Autopilot.")
    elif command == "start_turn_left":
        autopilot.left_turn_engaged = True
        autopilot.right_turn_engaged = False
        logger.info("Start Left Turn.")
    elif command == "start_turn_right":
        autopilot.left_turn_engaged = False
        autopilot.right_turn_engaged = True
        logger.info("Start Right Turn.")
    elif command == "stop_turn_right":
        autopilot.right_turn_engaged = False
        logger.info("Stop Right Turn.")
    elif command == "stop_turn_left":
        autopilot.left_turn_engaged = False
        logger.info("Stop Left Turn.")
        
def _safe_broadcast(payload: dict):
    dead = []
    msg = json.dumps(payload, separators=(",", ":"))
    for c in list(clients):
        try:
            c.write_message(msg)
        except Exception:
            dead.append(c)
    for c in dead:
        clients.discard(c)
            
def broadcast_can_message(data):
    if 'steering_goal' in data and "max_steering_angle" in data and "min_steering_angle" in data:
        autopilot.set_steering_shaft_limits(data["min_steering_angle"], data["max_steering_angle"])
    elif 'COG' in data:
        autopilot.set_heading(data['COG'])
    elif "rudder_value_min" in data and "rudder_value_max" in data:
        autopilot.set_rudder_counts(data["rudder_value_min"], data["rudder_value_max"])
        
    _safe_broadcast(data)

def _getattr(obj, name, default=None):
    try:
        v = getattr(obj, name)
        # Call if it's a method like get_rpm()
        return v() if callable(v) else v
    except Exception:
        return default

def _step_ekf():
    """Advance the heading EKF from the current CAN interface values."""
    global _ekf, _ekf_last_t, _compass_prev
    if not EKF_ENABLED:
        return None
    now = time.time()
    cog = _getattr(can_interface, "COG", None)
    comp = _getattr(can_interface, "compass_heading", None)
    node_yaw = _getattr(can_interface, "node_yaw_rate", None)
    sog_mps = _getattr(can_interface, "sog_mps", None)
    sog_mph = sog_mps * 2.236936 if sog_mps is not None else 0.0

    if _ekf is None:
        seed = cog if cog is not None else (comp if comp is not None else 0.0)
        _ekf = HeadingEKF3(heading0=seed) if node_yaw is not None else HeadingEKF2(heading0=seed)
        _ekf_last_t = now

    dt = now - (_ekf_last_t or now)
    _ekf_last_t = now
    if dt <= 0:
        dt = 0.1
    _ekf.predict(dt)

    # compass yaw-rate measurement (the derivative), gated for spikes
    if comp is not None:
        if _compass_prev is not None:
            pv, pt = _compass_prev
            cdt = now - pt
            if cdt > 0:
                rate = _angdiff(comp, pv) / cdt
                if abs(rate) <= YAW_RATE_MAX_DPS:
                    _ekf.update_yaw_rate(rate, R=1.0)
        _compass_prev = (comp, now)

    if node_yaw is not None and hasattr(_ekf, "update_yaw_rate_gyro"):
        _ekf.update_yaw_rate_gyro(float(node_yaw), R=0.3)

    # COG heading correction, speed-weighted (higher speed -> lower R -> more
    # trust). No hard threshold: the EKF blends rather than switches.
    if cog is not None and sog_mph > 0.3:
        R_cog = max(4.0, 400.0 / (sog_mph * sog_mph))
        if abs(_angdiff(cog, _ekf.heading)) / max(dt, 0.1) <= YAW_RATE_MAX_DPS:
            _ekf.update_heading(cog, R=R_cog)

    return {
        "ekf_heading": _ekf.heading,
        "ekf_yaw_rate": _ekf.yaw_rate,
        "ekf_sigma": _ekf.sigma,
        "ekf_yaw_bias": getattr(_ekf, "yaw_bias", None),
        "ekf_states": _ekf.n,
        "ekf_rejects": _ekf_yaw_limiter.rejected_count,
    }


def make_telemetry_snapshot():
    """
    Builds a single JSON-friendly dict of latest values.
    Tweak attribute names here to match your classes.
    """
    # Pull from CANinterface (raw) — rename as needed
    rpm         = _getattr(can_interface, "rpm", None)
    speed       = _getattr(can_interface, "boat_speed", None)  # wheel/GPS-derived speed
    sog_mps     = _getattr(can_interface, "sog_mps", None)
    cog_deg     = _getattr(can_interface, "COG", None)
    compass_deg = _getattr(can_interface, "compass_heading", None)
    lat         = _getattr(can_interface, "lat", None)
    lon         = _getattr(can_interface, "lon", None)
    shaft_goal = _getattr(autopilot, "shaft_goal", None)

    # Pull from Autopilot (fused/commanded)
    heading_deg   = _getattr(autopilot, "current_heading", None)     # fused/estimated heading
    heading_goal  = _getattr(autopilot, "heading_goal", None)
    heading_error = _getattr(autopilot, "heading_error", None)
    rudder_counts = _getattr(autopilot, "rudder_goal", None)
    rudder_angle  = _getattr(autopilot, "current_rudder", None)
    autopilot_on  = _getattr(autopilot, "autopilot_engaged", None)
    servo_on      = _getattr(can_interface, "servo_enabled", None)
    compass_offset = _getattr(can_interface, "compass_offset", None)
    
    snap = {
        "ts": time.time(),
        "rpm": rpm,
        "speed": speed,
        "heading_deg": heading_deg,
        "heading_goal": heading_goal,
        "compass_deg": compass_deg,
        "cog_deg": cog_deg,
        "sog_mps": sog_mps,
        "lat": lat,
        "lon": lon,
        "headingErr": heading_error,
        "rudder_counts": rudder_counts,
        "rudder_angle": rudder_angle,
        "autopilot_engaged": bool(autopilot_on) if autopilot_on is not None else None,
        "servo_enabled": bool(servo_on) if servo_on is not None else None,
        "shaft_goal": shaft_goal,
        "compass_offset": compass_offset,
    }

    # Which heading source the fishing logic would use right now, for the
    # "active source" indicator. Mirrors the COG-primary-at-speed rule.
    sog_mph = (sog_mps * 2.236936) if sog_mps is not None else 0.0
    if cog_deg is not None and sog_mph >= COG_PRIMARY_SPEED_MPH:
        snap["active_source"] = "COG"
    elif compass_deg is not None:
        snap["active_source"] = "COMPASS"
    else:
        snap["active_source"] = "NONE"
    snap["sog_mph"] = sog_mph

    # EKF outputs (heading, yaw rate, sigma, learned bias, diagnostics)
    ekf = _step_ekf()
    if ekf:
        snap.update(ekf)

    # Filtered-log recorder status (path + count), so the page can show/download
    rstats = _recorder.stats()
    snap["log_path"] = rstats["path"]
    snap["log_count"] = rstats["count"]

    return snap

async def telemetry_broadcaster(rate_hz: int = 10):
    """
    Periodically pushes a consolidated telemetry frame to all WebSocket clients.
    """
    period = 1.0 / max(rate_hz, 1)
    while True:
        try:
            payload = make_telemetry_snapshot()
            _safe_broadcast(payload)
        except Exception:
            logger.exception("telemetry_broadcaster failed to build/broadcast snapshot")
        await asyncio.sleep(period)

def make_app():
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/ws", WSHandler),
        (r"/exit-browser", ExitBrowserHandler),
        (r"/plot",PlotHandler),
        (r"/used-can-log", LogDownloadHandler),
        (r"/(sw\.js)", tornado.web.StaticFileHandler,
            {"path": os.path.join(os.path.dirname(__file__), "static")}),
    ],
    template_path="templates",
    static_path="static",   
    debug=True)

async def main():
    app = make_app()
    server = tornado.httpserver.HTTPServer(app)
    server.listen(5000)

    can_interface.add_listener(broadcast_can_message)
    # Record only the used CAN frames to a small, shareable log.
    can_interface.add_raw_listener(_recorder.record_message)
    
    # Start the periodic telemetry stream (10–20 Hz is plenty)
    asyncio.create_task(telemetry_broadcaster(rate_hz=5))

    # Kick off offline replay if enabled
    if OFFLINE_TEST:
        logger.info("Setting up Offline Test for replay_candump")
        asyncio.create_task(
            can_interface.replay_candump(
                CANDUMP_PATH, speed=REPLAY_SPEED, loop=REPLAY_LOOP
            )
        )

    await can_interface.read_loop()

if __name__ == "__main__":
    tornado.autoreload.watch("templates/plot_data.html")   # any file you like
    tornado.autoreload.watch("autopilot.py")   # any file you like
    tornado.autoreload.watch("can_interface.py")   # any file you like
    tornado.autoreload.watch("app.py")   # any file you like
    tornado.autoreload.watch("templates/autopilot.html")   # any file you like
    tornado.autoreload.start()            # will re-exec the process on change
    tornado.ioloop.IOLoop.current().run_sync(main)
