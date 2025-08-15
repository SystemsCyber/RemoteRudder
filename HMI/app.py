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
import os
import json
import logging
import subprocess

# ---- OFFLINE TEST SETTINGS (hardcoded) ----
OFFLINE_TEST = True
OFFLINE_BACKEND = "virtual"        # use python-can virtual bus
OFFLINE_CHANNEL = "vcan0"          # arbitrary name for virtual bus
CANDUMP_PATH = r"logs\test_data-21July2025.log"   # <-- set your file here
REPLAY_SPEED = 1.0                 # 1.0 = realtime
REPLAY_LOOP = True                 # loop file


sys.stdout.flush()

from can_interface import CANinterface
from autopilot import Autopilot

# Setup logger
logger = logging.getLogger('autopilot')
logging.basicConfig(level=logging.INFO)

clients = set()
can_interface = CANinterface(channel='can0')
autopilot = Autopilot(can_interface)
autopilot.start()


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
    rudder_counts = _getattr(autopilot, "rudder_goal", None)
    rudder_angle  = _getattr(autopilot, "current_rudder", None)
    autopilot_on  = _getattr(autopilot, "autopilot_engaged", None)
    servo_on      = _getattr(can_interface, "servo_enabled", None)
    
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
        "heading_goal": heading_goal,
        "rudder_counts": rudder_counts,
        "autopilot_engaged": bool(autopilot_on) if autopilot_on is not None else None,
        "servo_enabled": bool(servo_on) if servo_on is not None else None,
        "shaft_goal": shaft_goal
    }
    return snap

async def telemetry_broadcaster(rate_hz: int = 5):
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
        (r"/plot",PlotHandler)
    ],
    template_path="templates",
    static_path="static",   
    debug=True)

async def main():
    app = make_app()
    server = tornado.httpserver.HTTPServer(app)
    server.listen(5000)

    can_interface.add_listener(broadcast_can_message)
    
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
    tornado.ioloop.IOLoop.current().run_sync(main)
