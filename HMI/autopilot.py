import can
import logging
import struct
import time
import sys
import threading

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
        self.counter = 0

        self._run_thread = None
        self._stop_event = threading.Event()
    
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
        try:
            # Construct CAN data
            # Byte 0: engaged (0x01 if true, 0x00 otherwise)
            # Byte 1-2: heading goal (uint16, degrees * 100)
            # Byte 3-4: heading error (int16, degrees * 100)
            # Byte 5-6: rudder goal (int16, degrees * 100)
            # Byte 7: reserved or counter

            engaged_byte = 0x01 if self.autopilot_engaged else 0x00
            heading_goal = int(self.heading_goal * 100 + 0x8000) & 0xFFFF
            heading_error = int(self.heading_error * 100 + 0x8000) & 0xFFFF
            rudder_goal = int(self.rudder_goal * 100 + 0x8000) & 0xFFFF
            self.counter += 1 # just a rotating counter for sanity

            data = struct.pack("<BHHHB", engaged_byte, heading_goal, heading_error, rudder_goal, (self.counter & 0xFF))

            msg = can.Message(arbitration_id=0x18FF50E0,  # example PGN (can change)
                            data=data,
                            is_extended_id=True)

            self.bus.send(msg)
            logger.debug(f"Sent Autopilot status: {0x18FF50E0:08X} {data.hex()}")

        except can.CanError:
            logger.exception("CAN message failed to send")
    
    def start(self):
        if self._run_thread is None or not self._run_thread.is_alive():
            self._stop_event.clear()
            self._run_thread = threading.Thread(target=self.run, daemon=True)
            self._run_thread.start()
            logger.info("Autopilot run thread started.")

    def stop(self):
        self._stop_event.set()
        logger.info("Autopilot stop signal sent.")

    def run(self):
        while True:
            time.sleep(0.1)
            self.heading_error = self.heading_goal - self.current_heading
            if self.heading_error < -180:
                self.heading_error += 360
            elif self.heading_error > 180:
                self.heading_error -= 360
            logger.debug(f"heading error: {self.heading_error}")
            
            self.broadcast_status_message()
            if self.autopilot_engaged == True:
                # Generate a command and broadcast the steeing
                pass 
        
        
