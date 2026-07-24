import can
import logging
import struct
import time
import sys
import threading
import csv
import os
import datetime
import queue
import statistics

AUTOPILOT_CAN_ID = 0x18FF50E0


# Setup logger
logger = logging.getLogger('autopilot')
logger.setLevel(logging.INFO)

QUEUE_SIZE = 1000

class Autopilot():
    def __init__(self, can_interface):
        self.can_interface = can_interface
        self.bus = can_interface.bus
        self.heading_goal = 0 #degrees
        self.current_heading = 0 #degrees
        self.current_rudder = 0 #degrees
        self.rudder_goal = 0 #count
        self.autopilot_engaged = False
        self.autopilot_engaged_event  = threading.Event()
        self.left_turn_engaged = False
        self.right_turn_engaged = False
        
        self.heading_error = 0 #degrees
        self.steering_shaft_max = 1500
        self.steering_shaft_min = 1000
        self.shaft_goal = 1500
        self.current_shaft = 1500
        self.shaft_center = 1425
        self.rudder_count_max = 0
        self.rudder_count_min = 0
        self.counter = 0
        self.heading_list = []
        self.shaft_list = []
        self.error_list = []
        self.time_list = []
        self.error_list_length = 1000

        self._run_thread = None
        self._stop_event = threading.Event()

        self.Kp = 30 # Proportional gain
        self.Ki = .5
        self.Kd = 5

        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = time.time()
        self.start_time = time.time()
        self.last_turn_time = time.time()
        
        self.init_logger()
    
    def init_logger(self, log_dir="logs"):
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(log_dir, f"autopilot_log_{timestamp}.csv")

        self.log_file = open(log_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        
        # Write CSV header
        self.csv_writer.writerow([
            "timestamp", "rel_time", "heading_goal", "current_heading", "heading_error",
            "rudder_goal", "rudder_actual", "shaft_goal", "shaft_actual", "autopilot_engaged"
        ])
        self.log_file.flush()
        logger.info(f"Autopilot CSV log initialized at {log_path}")

    def stop_logger(self):
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.close()
            logger.info("Autopilot CSV log closed.")

    def adjust_heading_goal(self,delta):
        logger.info(f'Adjust heading goal by {delta}.')
        try:
            self.heading_goal += delta
            if self.heading_goal < 0:
                self.heading_goal += 360
            elif self.heading_goal > 360:
                self.heading_goal -= 360
        except:
            logger.exception("Invalid heading change adjustment.")

    def set_heading_goal(self,value):
        logger.info(f'set heading goal to {value}.')
        try:
            self.heading_goal = value
            if self.heading_goal < 0:
                self.heading_goal += 360
            elif self.heading_goal > 360:
                self.heading_goal -= 360
        except:
            logger.exception("Invalid heading goal setting.")
    
    def set_heading(self,value):
        logger.info(f'set heading value to {value}.')
        try:
            self.current_heading = value
            if self.current_heading < 0:
                self.current_heading += 360
            elif self.current_heading > 360:
                self.current_heading -= 360
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
            engaged_byte += 0x10 if self.left_turn_engaged else 0
            engaged_byte += 0x20 if self.right_turn_engaged else 0
             
            heading_goal = int(self.heading_goal * 10 + 0x8000) & 0xFFFF
            heading_error = int(self.heading_error * 100 + 0x8000) & 0xFFFF
            rudder_goal = int(self.rudder_goal * 100 + 0x8000) & 0xFFFF 
            self.counter += 1 # just a rotating counter for sanity

            data = struct.pack("<BHHHB", engaged_byte, heading_goal, heading_error, rudder_goal, (self.counter & 0xFF))

            msg = can.Message(arbitration_id=AUTOPILOT_CAN_ID,  # example PGN (can change)
                            data=data,
                            is_extended_id=True)

            self.bus.send(msg)
            logger.debug(f"Sent Autopilot status: {AUTOPILOT_CAN_ID:08X} {data.hex()}")

        except can.CanOperationError as e:
            # Routine during shutdown (bus closed under us) and while the
            # adapter is unplugged. A full traceback at ERROR for every
            # 100 ms tick buries the log; the link fault is reported
            # separately by CANLink.
            logger.debug("status send skipped: %s", e)
        except can.CanError:
            logger.exception("CAN message failed to send")
    
    def start(self):
        if self._run_thread is None or not self._run_thread.is_alive():
            self._stop_event.clear()
            self._run_thread = threading.Thread(target=self.run, daemon=True)
            self._run_thread.start()
            logger.info("Autopilot run thread started.")

    def stop(self, timeout: float = 2.0):
        # Order matters: signal, wait for the worker to actually exit, and
        # only then close the log. Closing first is what produced
        # "ValueError: I/O operation on closed file" from the still-running
        # thread's log_data() call.
        self._stop_event.set()
        t = self._run_thread
        if t is not None and t.is_alive():
            t.join(timeout=timeout)
            if t.is_alive():
                logger.warning("Autopilot thread did not exit within %.1fs", timeout)
        self.stop_logger()
        logger.info("Autopilot stopped.")
    
    def set_rudder_counts(self, min_val, max_val):
        self.rudder_count_min = int(min_val)
        self.rudder_count_max = int(max_val)
        logger.debug(f"Set rudder counts: min={min_val}, max={max_val}")
        

    def set_steering_shaft_limits(self, min_val, max_val):
        self.steering_shaft_min = float(min_val)
        self.steering_shaft_max = float(max_val)
        logger.debug(f"Set steering shaft limits: min={self.steering_shaft_min}, max={self.steering_shaft_max}")


    def run(self):
        self.last_shaft_adjust_time = time.time()
        self.last_turn_time = time.time()
        self.last_shaft_goal = self.shaft_goal
        self.shaft_goal_list = []
        self.autopilot_engaged_event.clear()
        # _stop_event was set by stop() but never read, so this thread
        # outlived shutdown and kept writing to a closed CSV and a closed
        # bus. Wait on the event instead of sleeping so a stop is acted on
        # within one cycle rather than after a full 100 ms tick.
        while not self._stop_event.is_set():
            now = time.time()
            if self._stop_event.wait(0.1):
                break
            ## This seemed to spin up the system
            self.heading_list.append(self.current_heading)
            self.shaft_list.append(self.can_interface.shaft_value)
            if len(self.heading_list) >= QUEUE_SIZE:
                shaft_mean = statistics.mean(self.shaft_list)
                heading_mean = statistics.mean(self.heading_list)
                heading_std = statistics.stdev(self.heading_list, xbar=heading_mean)
                self.heading_list = []
                self.shaft_list = []
                if heading_std < 2: # This is an indicator the boat is going straight
                    self.shaft_center = shaft_mean
            
            if (now - self.last_turn_time > 2.5):
                self.last_turn_time = now
                if self.left_turn_engaged:
                    self.right_turn_engaged = False
                    self.heading_goal -= 1
                    if self.heading_goal < 0:
                        self.heading_goal += 360
                elif self.right_turn_engaged:
                    self.left_turn_engaged = False
                    self.heading_goal += 1
                    if self.heading_goal >= 360:
                        self.heading_goal -= 360
            
            self.heading_error = self.heading_goal - self.current_heading
            if self.heading_error < -180:
                self.heading_error += 360
            elif self.heading_error > 180:
                self.heading_error -= 360
            
            self.shaft_goal = self.compute_rudder_command()
            self.shaft_goal_list.append(self.shaft_goal)
            self.broadcast_status_message()
            logger.debug(f"heading error: {self.heading_error:.2f}, Computed shaft goal: {self.shaft_goal:.2f}")
            #if self.autopilot_engaged == True:
            if not self.autopilot_engaged_event.is_set():
                self.heading_goal = self.current_heading
                self.error_list = []
                self.time_list = []
    
            if (now - self.last_shaft_adjust_time) > 1:
                self.last_shaft_adjust_time = now
                self.shaft_goal_mean = statistics.mean(self.shaft_goal_list)
                self.shaft_goal_list = []
                if abs(self.shaft_goal_mean - self.last_shaft_goal) > 100: # deadband
                    # Generate a command and broadcast the steering
                    self.last_shaft_goal = self.shaft_goal_mean
                    if self.autopilot_engaged_event.is_set():
                        self.can_interface.set_shaft_goal(self.shaft_goal)
                        self.can_interface.send_shaft_goal()
                        logger.debug(f"Sent command for shaft position of {self.can_interface.shaft_goal}")
                    

            self.log_data()

    
    def log_data(self):
        now = datetime.datetime.now().isoformat()
        rel_time = time.time() - self.start_time
        self.csv_writer.writerow([
            now,
            rel_time,
            round(self.heading_goal, 2),
            round(self.current_heading, 2),
            round(self.heading_error, 2),
            round(self.rudder_goal, 2),
            round(self.current_rudder, 2),
            round(self.shaft_goal,1),
            int(self.autopilot_engaged)
        ])
        self.log_file.flush()    

    def compute_rudder_command(self):
        current_time = time.time()
        dt = current_time - self._last_time
        self._last_time = current_time
        
        error = self.heading_error
        
        self.time_list.append(current_time)
        self.error_list.append(error)
        if len(self.error_list) <= 1:
            return self.Kp * error

        if len(self.error_list) > self.error_list_length:
            self.time_list.pop(0)
            self.error_list.pop(0)
        
        self._integral = 0
        if len(self.time_list) > 1 and len(self.error_list) > 1:
            for i in range(1, len(self.time_list)):
                dt = self.time_list[i] - self.time_list[i-1]
                avg_val = (self.error_list[i] + self.error_list[i-1]) / 2
                self._integral += avg_val * dt

        derivative = self.compute_slope(self.time_list[-4:], self.error_list[-4:])

        # derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        shaft_command = (
            self.Kp * error +
            self.Ki * self._integral +
            self.Kd * derivative + 
            self.shaft_center
        )
        if shaft_command < 0:
            shaft_command = 0
        elif shaft_command > 2950:
            shaft_command = 2950
        logger.debug(f"Compute: dt: {dt:.6f}, Kp: {self.Kp}, error: {error:.3f}, Ki: {self.Ki}, integral: {self._integral:.1f}, Kd: {self.Kd}, derivative: {derivative:.3f}, shaft_command: {shaft_command:.1f}")

        return shaft_command

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
    
    def map_rudder_to_steering(self, rudder_count):
        '''
        Converts the rudder position into steering shaft position
        '''
        rudder_range = self.rudder_count_max - self.rudder_count_min
        shaft_range = self.steering_shaft_max - self.steering_shaft_min
        if rudder_range == 0: #degrees
            return 0

        ratio = (rudder_count - self.rudder_count_min) / rudder_range
        shaft_angle = self.steering_shaft_min + ratio * shaft_range
        logger.debug(f"shaft_angle: {shaft_angle}")
        
        return shaft_angle

    def compute_slope(self, xs, ys):
        n = len(xs)
        if n != len(ys) or n < 2:
            raise ValueError("Need at least two points")

        x_mean = sum(xs) / n
        y_mean = sum(ys) / n

        numerator = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))
        denominator = sum((x - x_mean) ** 2 for x in xs)

        if denominator == 0:
            return float('inf')  # or raise an exception

        return numerator / denominator

