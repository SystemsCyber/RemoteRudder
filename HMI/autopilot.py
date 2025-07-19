import can
import logging
import struct
import time
import sys

# Setup logger
logger = logging.getLogger("CANinterface")
logger.setLevel(logging.DEBUG)

class Autopilot():
    def __init__(self, can_interface):
        self.bus = can_interface.bus
        self.heading_goal = 0 #degrees
        self.current_heading = 0 #degrees
        self.current_rudder = 0 #degrees
        self.rudder_goal = 0 #degrees
        self.autopilot_engaged = False
        self.heading_error = 0 #degrees
        self.steering_shaft_max = 0
        self.steering_shaft_min = 0
        self.rudder_angle_max = 0
        self.rudder_angle_min = 0
        
    
    def adjust_heading_goal(self,delta):
        logger.info(f'Adjust heading goal by {delta}.')
        try:
            self.heading_goal += delta
        except:
            logger.exception("Invalid heading change adjustment.")

    def set_heading_goal(self,value):
        logger.info(f'set heading goal to {value}.')
        try:
            self.heading_goal = value % 360
        except:
            logger.exception("Invalid heading goal setting.")

    def broadcast_status_message(self):
        logger.debug("Broadcasting CAN from Autopilot")
        #counter
        #engaged
        #heading_goal
        #heading_error
        #rudder_goal
        #rudder_error

    def run(self):
        while True:
            time.sleep(0.1)
            self.heading_error = self.heading_goal - self.current_heading
            if self.heading_error < -180:
                self.heading_error += 360
            elif self.heading_error > 180:
                self.heading_error -= 360
            logger.debug(f"heading error: {self.heading_error}")

            if self.autopilot_engaged == True:
                # Generate a command and broadcast the steeing
                pass 
        
        
