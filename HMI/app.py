#!/usr/bin/env python3
# sudo apt install python3-tornado


# Test with vcan 
# canplayer -l i -v -I logs/test_data-21July2025.log vcan0=can0

#
import sys
sys.stdout.flush()


import tornado.web
import tornado.websocket
import tornado.ioloop
import tornado.httpserver
import os
import json
import logging

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
        logger.info("Received Request to Enable Autopilot.")
    elif command == "autopilot_disable":
        autopilot.autopilot_engaged = False
        logger.info("Received Request to Disable Autopilot.")

            
def broadcast_can_message(data):
    if 'steering_goal' in data and "min_steering_angle" in data and "min_steering_angle" in data:
        # send new goal to the can interface
        #can_interface.current_goal= data['steering_goal']
        autopilot.set_steering_shaft_limits(data["min_steering_angle"], data["max_steering_angle"])
    elif 'COG' in data:
        autopilot.set_heading(data['COG'])
    elif "rudder_value_min" in data and "rudder_value_max" in data:
        autopilot.set_rudder_counts(data["rudder_value_min"], data["rudder_value_max"])

        
    for c in clients:
        c.write_message(json.dumps(data))

def make_app():
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/ws", WSHandler),
    ],
    template_path="templates",
    static_path="static",
    debug=True)

async def main():
    app = make_app()
    server = tornado.httpserver.HTTPServer(app)
    server.listen(5000)

    can_interface.add_listener(broadcast_can_message)
    await can_interface.read_loop()

if __name__ == "__main__":
    tornado.ioloop.IOLoop.current().run_sync(main)
