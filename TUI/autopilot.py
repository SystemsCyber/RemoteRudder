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
        # COG-lock gating. When engaged without a lock, hold last rudder.
        # When there is no steerage way at all, center and wait.
        self._cog_lock = False
        self._centered = False
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
        # Reference heading for the disengaged goal-follows-heading logic. While
        # disengaged the goal tracks the heading, but only updates on a
        # meaningful change (> 1 deg) so the green goal line does not jitter on
        # compass noise. This holds the last heading we synced the goal to.
        # MUST be initialized here -- the run loop reads it every cycle.
        self._disengaged_goal_ref = None
        # Live PID term contributions (shaft counts), for the TUI PID panel.
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.last_derivative = 0.0
        self.last_error = 0.0
        # Gain adjustment steps for the TUI tuning keys.
        self.kp_step = 1.0
        self.ki_step = 0.1
        self.kd_step = 0.5

        # Shaft command output limits (encoder counts). The PID output is
        # clamped to the shaft's physical travel, so on a stationary boat the
        # integral winds up and the shaft LIMITS OUT chasing an unreachable
        # goal -- the intended behaviour.
        self.shaft_out_min = 0.0
        self.shaft_out_max = 2950.0
        # Deadband on the sent shaft command (counts). Smaller than the old 100
        # so gradual integral windup actually moves the shaft toward its limit
        # instead of being swallowed. Still large enough to avoid CAN spam and
        # servo chatter on tiny corrections.
        self.shaft_deadband = 20.0

        # Integral clamp. The integral is a TRUE accumulator (it winds up over
        # time on a persistent error), but it is bounded so that when the boat
        # finally gets steerage way and the error clears, the wound-up integral
        # does not cause a huge overshoot. The bound is expressed in shaft
        # counts of integral authority: Ki * integral is capped to +/- this.
        # Chosen so the integral alone can drive the shaft across its full
        # half-travel (~1475 counts) but not wildly beyond.
        self.integral_authority_max = 1500.0  # counts contributed by Ki*integral
        # -> integral magnitude cap = authority / Ki
        self._integral_max = self.integral_authority_max / self.Ki if self.Ki else 0.0

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
            # Normalize with modulo, exactly as SystemState.set_goal does, so
            # the two goal copies can never diverge. The old single-step
            # if/elif left 360 as 360 (state stored 0), which made the goal
            # appear to bounce between the set value and 360.
            self.heading_goal = (self.heading_goal + delta) % 360.0
        except Exception:
            logger.exception("Invalid heading change adjustment.")

    def set_heading_goal(self,value):
        logger.info(f'set heading goal to {value}.')
        try:
            self.heading_goal = float(value) % 360.0
        except Exception:
            logger.exception("Invalid heading goal setting.")

    def set_heading(self,value):
        logger.info(f'set heading value to {value}.')
        try:
            self.current_heading = float(value) % 360.0
        except Exception:
            logger.exception("Invalid heading goal setting.")

    def set_cog_lock(self, locked: bool):
        """
        Tell the autopilot whether the active heading is a trusted COG lock.

        When False while engaged, the loop holds the last rudder command
        instead of chasing a heading it cannot trust (a dropped COG at speed,
        or the weak compass). It does NOT center: on a boat still making way,
        holding a small standing correction is safer than snapping the rudder
        amidships. Centering is reserved for loss of steerage way, which shows
        up as fused_heading going None and the boat slowing below COG_MIN.
        """
        self._cog_lock = bool(locked)

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
            if self.current_heading is not None:
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
                    self.heading_goal = (self.heading_goal - 1) % 360.0
                elif self.right_turn_engaged:
                    self.left_turn_engaged = False
                    self.heading_goal = (self.heading_goal + 1) % 360.0
            
            if self.current_heading is None:
                # No heading this cycle. Leave heading_error stale and skip the
                # PID; the command-send block below will center or hold based
                # on engagement and lock state.
                self.heading_error = 0.0
                self.broadcast_status_message()
            else:
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
                # Track current heading while disengaged so engaging holds the
                # present course. But only update on a MEANINGFUL change (> 1
                # deg) so the goal -- and the green goal line on the web wheel --
                # does not jitter with the compass's natural ~0.5 deg noise.
                # Holding steady between updates keeps the display calm.
                if self.current_heading is not None:
                    if (self._disengaged_goal_ref is None or
                            abs(((self.current_heading - self._disengaged_goal_ref
                                  + 180) % 360) - 180) > 1.0):
                        self.heading_goal = self.current_heading
                        self._disengaged_goal_ref = self.current_heading
                self.error_list = []
                self.time_list = []
                # Clear the integral so a fresh engagement does not inherit
                # windup accumulated while disengaged.
                self._integral = 0.0
            else:
                # Engaged: the goal is the operator's commanded value; stop
                # tracking heading. Reset the disengaged reference so the next
                # disengage re-syncs cleanly.
                self._disengaged_goal_ref = None
    
            if (now - self.last_shaft_adjust_time) > 1:
                self.last_shaft_adjust_time = now
                if self.shaft_goal_list:
                    self.shaft_goal_mean = statistics.mean(self.shaft_goal_list)
                self.shaft_goal_list = []

                engaged = self.autopilot_engaged_event.is_set()
                have_heading = self.current_heading is not None

                if engaged and not have_heading:
                    # No steerage way / no usable heading: center the rudder
                    # once and wait for motion. Sending center repeatedly would
                    # fight the deadband, so latch it.
                    if not self._centered:
                        self.can_interface.set_shaft_goal(self.shaft_center)
                        self.can_interface.send_shaft_goal()
                        self._centered = True
                        logger.info("No heading: centering rudder, waiting for motion")
                elif engaged and not self._cog_lock:
                    # Moving but COG lock lost: hold the last rudder command.
                    # Do not send anything new; do not center. The boat keeps
                    # whatever standing correction it had until COG returns.
                    self._centered = False
                elif engaged:
                    self._centered = False
                    # Send the SMOOTHED command (the mean we just computed), not
                    # the instantaneous shaft_goal -- previously it compared the
                    # mean but sent the instantaneous value, a mismatch.
                    cmd = self.shaft_goal_mean
                    at_limit = (cmd <= self.shaft_out_min + 1 or
                                cmd >= self.shaft_out_max - 1)
                    moved = abs(cmd - self.last_shaft_goal) > self.shaft_deadband
                    # Send when the command moved past the (smaller) deadband OR
                    # when it is pinned at a travel limit but not yet sent there.
                    # The at-limit case is what makes the shaft reliably LIMIT
                    # OUT as the integral winds up on a stationary boat: gradual
                    # windup would otherwise never cross a large deadband.
                    if moved or (at_limit and self.last_shaft_goal != cmd):
                        self.last_shaft_goal = cmd
                        self.can_interface.set_shaft_goal(cmd)
                        self.can_interface.send_shaft_goal()
                        logger.debug(
                            f"Sent shaft position {self.can_interface.shaft_goal:.0f}"
                            f"{' (LIMIT)' if at_limit else ''}")
                else:
                    self._centered = False
                    

            self.log_data()

    
    def log_data(self):
        now = datetime.datetime.now().isoformat()
        rel_time = time.time() - self.start_time
        self.csv_writer.writerow([
            now,
            rel_time,
            round(self.heading_goal, 2),
            round(self.current_heading, 2) if self.current_heading is not None else "",
            round(self.heading_error, 2),
            round(self.rudder_goal, 2),
            round(self.current_rudder, 2),
            round(self.shaft_goal,1),
            int(self.autopilot_engaged)
        ])
        self.log_file.flush()    

    def compute_rudder_command(self):
        """
        PID controller. Returns a SHAFT command in encoder counts.

        Terminology (these are distinct, from different sources):
          * SHAFT -- the intermediate power shaft, geared via timing-belt
            pulleys for torque. This is what we command (shaft_goal, counts
            0..2950) and what the string potentiometer on the 'steering' node
            (0x18F01D21) reports. It is NOT the rudder position.
          * RUDDER angle -- the actual rudder, measured by a separate string
            potentiometer reported by the 'rudder' node (0x19F10D13), in
            degrees. It is feedback only; the PID does not command it directly.

        The shaft drives the rudder through the gearing, so commanding the shaft
        moves the rudder, but they are measured independently.

        Integral behaviour: a TRUE accumulator. On a stationary boat chasing a
        goal it cannot reach (no steerage way), the integral winds up every
        cycle and the output clamps to the shaft's travel -- the shaft LIMITS
        OUT. The integral is bounded (integral clamp) so overshoot stays sane
        when the boat finally gets way on and the error clears.
        """
        current_time = time.time()
        dt = current_time - self._last_time
        self._last_time = current_time
        # Guard against a bad dt (first call, clock jump, paused thread).
        if dt <= 0 or dt > 5.0:
            dt = 0.0

        error = self.heading_error

        # Keep a short error/time history for the derivative only. The integral
        # is now a persistent accumulator, not a windowed sum, so it is no
        # longer derived from this list.
        self.time_list.append(current_time)
        self.error_list.append(error)
        if len(self.error_list) > self.error_list_length:
            self.time_list.pop(0)
            self.error_list.pop(0)

        # --- TRUE ACCUMULATING INTEGRAL ---
        # Add this cycle's error*dt to the running integral. On a stationary
        # boat with a persistent error (goal it cannot reach without steerage
        # way), this grows every cycle and drives the shaft toward its limit --
        # the operator's expected behaviour. It is clamped so that when the boat
        # finally moves and the error clears, the wound-up integral does not
        # cause a huge overshoot.
        self._integral += error * dt
        if self._integral_max > 0:
            if self._integral > self._integral_max:
                self._integral = self._integral_max
            elif self._integral < -self._integral_max:
                self._integral = -self._integral_max

        # Derivative from the recent slope (smoother than a single-step diff).
        if len(self.time_list) >= 2:
            derivative = self.compute_slope(self.time_list[-4:], self.error_list[-4:])
        else:
            derivative = 0.0
        self._prev_error = error

        shaft_command = (
            self.Kp * error +
            self.Ki * self._integral +
            self.Kd * derivative +
            self.shaft_center
        )

        # Store the individual term contributions (in shaft counts) so the TUI
        # PID panel can show them live. These are the actual P/I/D outputs, not
        # just the gains -- what the operator needs to tune by.
        self.p_term = self.Kp * error
        self.i_term = self.Ki * self._integral
        self.d_term = self.Kd * derivative
        self.last_derivative = derivative
        self.last_error = error

        # Clamp the OUTPUT to the shaft's physical travel. The integral keeps
        # accumulating (up to its own clamp) even while the output is pinned, so
        # the shaft sits AT its limit while the goal is unreachable -- it "limits
        # out", exactly as expected on a stationary boat.
        if shaft_command < self.shaft_out_min:
            shaft_command = self.shaft_out_min
        elif shaft_command > self.shaft_out_max:
            shaft_command = self.shaft_out_max

        logger.debug(
            f"Compute: dt={dt:.4f} Kp={self.Kp} err={error:.2f} "
            f"Ki={self.Ki} integral={self._integral:.1f} "
            f"(Ki*I={self.Ki*self._integral:.0f}) Kd={self.Kd} "
            f"deriv={derivative:.3f} shaft_cmd={shaft_command:.1f}"
        )
        return shaft_command

    def reset_integral(self):
        """Clear the integral accumulator. Called on disengage so a fresh
        engagement does not inherit stale windup."""
        self._integral = 0.0

    def adjust_gain(self, which: str, delta: float) -> None:
        """
        Nudge a PID gain by delta (for the TUI tuning keys). Keeps gains
        non-negative and, when Ki changes, recomputes the integral clamp so the
        integral authority stays consistent.
        """
        if which == "Kp":
            self.Kp = max(0.0, self.Kp + delta)
        elif which == "Ki":
            self.Ki = max(0.0, self.Ki + delta)
            self._integral_max = (self.integral_authority_max / self.Ki
                                  if self.Ki else 0.0)
        elif which == "Kd":
            self.Kd = max(0.0, self.Kd + delta)
        logger.info(f"PID gain {which} -> {getattr(self, which):.3f}")

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

