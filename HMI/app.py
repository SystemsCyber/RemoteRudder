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

from can_reader import CANReader
from can_writer import CANWriter    

# Setup logger
logger = logging.getLogger("CANReader")
logger.setLevel(logging.DEBUG)
logging.basicConfig(level=logging.DEBUG)

clients = set()
can_writer = CANWriter(bus_channel='can1', bustype='socketcan')
   
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
        except Exception as e:
            logger.exception("Failed to process WebSocket message")

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
        can_writer.current_goal= data['steering_goal']
    for c in clients:
        c.write_message(json.dumps(data))

def handle_client_command(command):
    logger.info(f"Handling command: {command}")
    if command == "rudder_left":
        can_writer.adjust_goal(-5)
    elif command == "rudder_right":
        can_writer.adjust_goal(5)
    elif command == "servo_enable":
        can_writer.set_servo_enabled(True)
    elif command == "servo_disable":
        can_writer.set_servo_enabled(False)


async def main():
    app = make_app()
    server = tornado.httpserver.HTTPServer(app)
    server.listen(5000)

    can_reader = CANReader(bus_channel='can1', bustype='socketcan')
    can_reader.add_listener(broadcast_can_message)
    await can_reader.read_loop()

if __name__ == "__main__":
    tornado.ioloop.IOLoop.current().run_sync(main)
