#!/usr/bin/env python3
# sudo apt install python3-tornado

print(">>> app.py starting")
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

# Setup logger
logger = logging.getLogger("CANinterface")
logger.setLevel(logging.DEBUG)
logging.basicConfig(level=logging.DEBUG)

clients = set()
can_interface = CANinterface()

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
                can_interface.set_goal(data["heading_goal"])
                broadcast_can_message(data={"heading_goal": data["heading_goal"]})
        except Exception as e:
            logger.exception("Failed to process WebSocket message")

def handle_client_command(command):
    logger.info(f"Handling command: {command}")
    if command == "rudder_left":
        can_interface.adjust_goal(-5)
    elif command == "rudder_right":
        can_interface.adjust_goal(5)
    elif command == "servo_enable":
        can_interface.set_servo_enabled(True)
    elif command == "servo_disable":
        can_interface.set_servo_enabled(False)
    elif command == "goal_set":
        try:
            goal = float(command.split(":")[1])
            
            logger.info(f"Set steering goal to {goal} degrees")
        except ValueError:
            logger.error("Invalid goal value received")
            
def make_app():
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/ws", WSHandler),
    ],
    template_path="templates",
    static_path="static",
    debug=True)


def broadcast_can_message(data):
    if 'steering_goal' in data:
        can_interface.current_goal= data['steering_goal']
    for c in clients:
        c.write_message(json.dumps(data))

def handle_client_command(command):
    logger.info(f"Handling command: {command}")
    if command == "rudder_left":
        can_interface.adjust_goal(-5)
    elif command == "rudder_right":
        can_interface.adjust_goal(5)
    elif command == "servo_enable":
        can_interface.set_servo_enabled(True)
    elif command == "servo_disable":
        can_interface.set_servo_enabled(False)


async def main():
    app = make_app()
    server = tornado.httpserver.HTTPServer(app)
    server.listen(5000)

    can_interface.add_listener(broadcast_can_message)
    await can_interface.read_loop()

if __name__ == "__main__":
    tornado.ioloop.IOLoop.current().run_sync(main)
