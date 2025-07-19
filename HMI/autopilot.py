import can
import logging
import struct
import time
import sys
import threading

# Setup logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class Autopilot():
    def __init__(self, can_interface):
        self.can_interface = can_interface
        self.bus = can_interface.bus
        self.heading_goal = 0 #degrees
        self.current_heading = 0 #degrees
        self.current_rudder = 0 #degrees
        self.rudder_goal = 0 #count
        self.autopilot_engaged = False
        self.heading_error = 0 #degrees
        self.steering_shaft_max = 0
        self.steering_shaft_min = 0
        self.rudder_count_max = 0
        self.rudder_count_min = 0
        self.counter = 0

        self._run_thread = None
        self._stop_event = threading.Event()

        self.Kp = 1
        self.Ki = 0.05
        self.Kd = .1

        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = time.time()
        
    
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
    
    def set_heading(self,value):
        logger.info(f'set heading value to {value}.')
        try:
            self.current_heading = value % 360
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
    
    def set_rudder_counts(self, min_val, max_val):
        self.rudder_count_min = int(min_val)
        self.rudder_count_max = int(max_val)
        logger.debug(f"Set rudder counts: min={min_val}, max={max_val}")
        

    def set_steering_shaft_limits(self, min_val, max_val):
        self.steering_shaft_min = float(min_val)
        self.steering_shaft_max = float(max_val)
        logger.info(f"Set steering shaft limits: min={self.steering_shaft_min}, max={self.steering_shaft_max}")


    def run(self):
        while True:
            time.sleep(0.1)
            self.heading_error = self.heading_goal - self.current_heading
            if self.heading_error < -180:
                self.heading_error += 360
            elif self.heading_error > 180:
                self.heading_error -= 360
            self.rudder_goal = self.compute_rudder_command()
            self.broadcast_status_message()
            logger.debug(f"heading error: {self.heading_error:.2f}, Computed rudder goal: {self.rudder_goal:.2f}")
            if self.autopilot_engaged == True:
                # Generate a command and broadcast the steeing

                self.can_interface.set_shaft_goal(self.rudder_goal)
                self.can_interface.send_shaft_goal()
                logger.debug(f"Sent command for shaft position of {self.can_interface.shaft_goal}")

        
    def compute_rudder_command(self):
        current_time = time.time()
        dt = current_time - self._last_time
        self._last_time = current_time

        error = self.heading_error
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        rudder_command = (
            self.Kp * error +
            self.Ki * self._integral +
            self.Kd * derivative
        )
        rudder_range = self.rudder_count_max - self.rudder_count_min
        shaft_range = self.steering_shaft_max - self.steering_shaft_min
        logger.debug(f"Compute: dt: {dt:.6f}, Kp: {self.Kp}, error: {error:.3f}, Ki: {self.Ki}, integral: {self._integral:.1f}, Kd: {self.Kd}, derivative: {derivative:.3f}, rudder_command: {rudder_command:.1f}, rudder_range: {rudder_range}, shaft_range: {shaft_range}")
        # Constrain rudder_command to physical limits
        #rudder_command = max(min(rudder_command, self.rudder_count_max), self.rudder_count_min)

        return rudder_command
 
    def map_steering_to_rudder(self, shaft_angle):
        """
        Linearly map steering shaft angle to rudder angle based on calibrated limits.
        """
        # Avoid divide-by-zero error
        rudder_range = self.rudder_count_max - self.rudder_count_min
        shaft_range = self.steering_shaft_max - self.steering_shaft_min
        if shaft_range == 0:
            return 0

        # Linear interpolation
        ratio = (shaft_angle - self.steering_shaft_min) / shaft_range
        rudder_angle = self.rudder_count_min + ratio * (self.rudder_count_max - self.rudder_count_min)
        logger.debug(f"rudder_angle: {rudder_angle}")
        return rudder_angle
    
    def map_rudder_to_steering(self, rudder_angle):
        rudder_range = self.rudder_count_max - self.rudder_count_min
        shaft_range = self.steering_shaft_max - self.steering_shaft_min
        if rudder_range == 0: #degrees
            return 0

        ratio = (rudder_angle - self.rudder_count_min) / rudder_range
        shaft_angle = self.steering_shaft_min + ratio * (self.steering_shaft_max - self.steering_shaft_min)
        logger.debug(f"shaft_angle: {shaft_angle}")
        
        #Prevent overturning. 
        if shaft_angle > 1260:
            shaft_angle = 1260
        elif shaft_angle < -1260:
            shaft_angle = -1260
        return shaft_angle

