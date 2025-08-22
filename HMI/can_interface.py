import can
import asyncio
import logging
import struct
import time
import sys
import queue
import os

from can.io.canutils import CanutilsLogReader

AUTOPILOT_CAN_ID = 0x18FF50E0

# Setup logger
logger = logging.getLogger('CAN')
logger.setLevel(logging.INFO)
logger.info("CANinterface module loaded")

class CANinterface:
    def __init__(self, channel='can0', bitrate = 250000, backend=None):
        # backend: 'virtual', 'pcan', or 'socketcan' (None keeps old auto behavior)
        if backend == "virtual":
            device = "virtual"
            channel = "vcan0"
        elif 'win' in sys.platform:
            device = 'pcan'
            channel = 'PCAN_USBBUS1'
        else:
            device = 'socketcan'
            channel = channel
         
        # Set the bitrate to 2500000 for all NMEA2000
        try:
            self.bus = can.interface.Bus(channel=channel, interface=device, bitrate=bitrate, receive_own_messages=True)
            logger.info(f"CAN bus initialized at {bitrate} on {channel} with {device}")
        except:
            logger.warning("CAN Interface Error: Please be sure hardware is plugged in.")
            sys.exit()

        self.listeners = []
        self.shaft_goal = 1475  # Start in middle degrees
        self.servo_enabled = False  # track state

        # Track last-received time by PGN
        self.watch_ids = {
            0x18F01D21: {"name": "steering", "last_time": time.time()},
            0x19F10D13: {"name": "rudder", "last_time": time.time()},
            0x09F8021C: {"name": "gps_position", "last_time": time.time()},
            0x18FEE81C: {"name": "vehicle_direction", "last_time": time.time()},
            0x0CF00400: {"name": "engine_control", "last_time": time.time()},
            0x09F8021C: {"name": "gps_speed", "last_time": time.time()},
            0x09F8011C: {"name": "gps_position_rapid", "last_time": time.time()},
            0x09F112F8: {"name": "vessel_heading", "last_time": time.time()},
        }
        self.timeout_interval = 2.0
        self.GUI_TIMEOUT = 0.35
        self.rpm_start_time = time.time()
        self.compass_heading = 0.0
        self.rudder_correction = 7.0
        self.boat_speed = 0.0
        self.COG = 0
        self.heading_correction = 7.5
        self.averaging_window = 100  # Number of samples for averaging
        self.heading_history = queue.Queue(maxsize=100)
        self.COG_history = queue.Queue(maxsize=100)
        self.max_steering_angle = 2900
        self.min_steering_angle = 0
        self.shaft_value = self.max_steering_angle/2
        
        self.disable_servo()

    async def replay_candump(self, path: str, speed: float = 1.0, loop: bool = False):
        """
        Replay a can-utils 'candump' log onto this bus.
        - path: path to candump log (e.g., C:\\logs\\demo.log)
        - speed: 1.0 = real time, 2.0 = 2x faster, 0.5 = half speed
        - loop: repeat forever
        """
        logger.info(f"Starting replay_candump with file {path}")
        while True:
            reader = CanutilsLogReader(path)
            t0_wall = time.time()
            t0_log = None

            for msg in reader:
                if t0_log is None:
                    # First timestamp in the log
                    t0_log = msg.timestamp
                    t0_wall = time.time()

                # Target wall clock time for this frame
                dt_log = (msg.timestamp - t0_log) / max(speed, 1e-6)
                target = t0_wall + dt_log
                delay = target - time.time()
                if delay > 0:
                    await asyncio.sleep(delay)

                try:
                    # Re-send onto our bus so your read_loop + decoders see it
                    if msg.arbitration_id != AUTOPILOT_CAN_ID:  # Skip autopilot status messages
                        self.bus.send(msg)
                except can.CanError as e:
                    logger.error(f"Replay send failed: {e}")
                    await asyncio.sleep(0.001)

            if not loop:
                break

    def add_listener(self, callback):
        self.listeners.append(callback)

    def adjust_shaft_goal(self, delta_degrees):
        if self.servo_enabled:
            self.shaft_goal += delta_degrees
            self.send_shaft_goal()  
    
    def set_servo_enabled(self, enable):
        self.servo_enabled = enable
        logger.info(f"Servo {'enabled' if enable else 'disabled'}")
        if self.servo_enabled:
            self.send_shaft_goal() 
        else:
            self.disable_servo() 
        
    def set_shaft_goal(self, value):
        self.shaft_goal = value
        
    def send_shaft_goal(self):
        angle_raw = int((self.shaft_goal * 1000) + 0x80000000) & 0xFFFFFFFF
        angle_bytes = angle_raw.to_bytes(4, byteorder='little')
        enable_byte = b'\x45' #if self.servo_enabled else b'\x00'

        data = angle_bytes + enable_byte + b'\xFF'*3
        msg = can.Message(arbitration_id=0x0CF34155, data=data, is_extended_id=True)
        try:
            self.bus.send(msg)
            logger.info(f"Sent CAN steering goal: {self.shaft_goal:.1f}°  {data.hex()}")
        except can.CanError as e:
            logger.error(f"CAN send failed: {e}")
    
    def disable_servo(self):
        angle_raw = int((self.shaft_goal * 1000) + 0x80000000) & 0xFFFFFFFF
        angle_bytes = angle_raw.to_bytes(4, byteorder='little')
        data = angle_bytes + b'\x00'*4
        msg = can.Message(arbitration_id=0x0CF34155, data=data, is_extended_id=True)
        try:
            self.bus.send(msg)
            logger.info(f"Sent CAN steering goal: {self.shaft_goal:.1f}°  {data.hex()}")
        except can.CanError as e:
            logger.error(f"CAN send failed: {e}")

    async def read_loop(self):
        reader = can.AsyncBufferedReader()
        notifier = can.Notifier(self.bus, [reader], loop=asyncio.get_event_loop())
        logger.info("Starting CAN read loop")

        asyncio.create_task(self._timeout_monitor())

        while True:
            msg = await reader.get_message()
            if msg.arbitration_id in self.watch_ids:
                self.watch_ids[msg.arbitration_id]["last_time"] = time.time()

            data = self.process_message(msg)
            if data:
                for cb in self.listeners:
                    cb(data)

    async def _timeout_monitor(self):
        while True:
            await asyncio.sleep(0.1)
            now = time.time()
            for msg_id, meta in self.watch_ids.items():
                if now - meta["last_time"] > self.timeout_interval:
                    logger.info(f"Timeout on {meta['name']} (ID 0x{msg_id:08X})")
                    for cb in self.listeners:
                        cb({
                            meta["name"]: None,
                            "timeout": True,
                            "source_id": msg_id
                        })
                    meta["last_time"] = now  # Avoid spamming

    def process_message(self, msg):
        now = time.time()
        if msg.arbitration_id == 0x18F01D21:  # steering
            angle = (struct.unpack('<L', msg.data[0:4])[0] - 0x80000000) / 1000
            self.shaft_value = angle
            goal_bytes = msg.data[4:8]
            if goal_bytes == b'\xFF\xFF\xFF\xFF':
                enabled = False
                goal = angle
            else:  # assume enabled if not FFs
                enabled = True
                goal = (struct.unpack('<L', goal_bytes)[0] - 0x80000000) / 1000
            if angle > self.max_steering_angle:
                self.max_steering_angle = angle
                logger.debug(f"Found new maximum steering angle: {self.max_steering_angle}")
            if angle < self.min_steering_angle:
                self.min_steering_angle = angle
                logger.debug(f"Found new minimum steering angle: {self.min_steering_angle}")
            logger.debug(f"{msg.arbitration_id:08X} {msg.data.hex()} Steering Angle: {angle:.2f}, Steering Goal: {goal:.2f}, Servo {'enabled' if enabled else 'disabled'}, max_steering_angle: {self.max_steering_angle}, min_steering_angle: {self.min_steering_angle}")
            return {"steering_angle": angle, "steering_goal": goal, "servo_enabled": enabled, "max_steering_angle": self.max_steering_angle, "min_steering_angle": self.min_steering_angle, }
           
        elif msg.arbitration_id == 0x19F10D13:  # rudder
            rudder_count = msg.data[0]
            rudder_value = struct.unpack('<H', msg.data[1:3])[0] 
            rudder_value_min = struct.unpack('<H', msg.data[3:5])[0]
            rudder_value_max = struct.unpack('<H', msg.data[5:7])[0]
            rudder_angle = (msg.data[7] - 0x80) / 2 + self.rudder_correction
            logger.debug(f"{msg.arbitration_id:08X} {msg.data.hex()} Rudder Angle: {rudder_angle:.1f}, Value: {rudder_value}, Min: {rudder_value_min}, Max: {rudder_value_max}")
            return {"rudder_angle": rudder_angle, "rudder_value": rudder_value, "rudder_value_min": rudder_value_min, "rudder_value_max": rudder_value_max}

        elif msg.arbitration_id == 0x09F8021C:  # GPS Speed, Course Over ground (COG), Speed over Ground (SOG). PGN 129026, 250ms update
            sequqence = msg.data[0]
            COG_ref = msg.data[1] & 0b00000011
            if COG_ref == 0b00:
                COG_ref = "True North"
            elif COG_ref == 0b01:
                COG_ref = "Magnetic North"
            elif COG_ref == 0b10:
                COG_ref = "Error"
            else:
                COG_ref = "Null"
            COG_rad = struct.unpack('<H', msg.data[2:4])[0] / 10000.0  # COG in radians
            self.COG = COG_rad * (180 / 3.14159)
            SOG_mps = struct.unpack('<H', msg.data[4:6])[0] / 100.0  # Speed over ground in m/s
            SOG_mph = SOG_mps * 2.236936  # Convert to mph
            self.boat_speed = SOG_mph  # Update boat speed
            if self.boat_speed > 10 and self.heading_correction is None:
                self.COG_history.put(self.COG)
            logger.debug(f"{msg.arbitration_id:08X} {msg.data.hex()} COG: {self.COG:.2f}, SOG: {SOG_mph:.2f} mph")
            if self.boat_speed > 1:
                return {"SOG": SOG_mph, "COG": self.COG}

        elif msg.arbitration_id == 0x09F8011C:  # GPS Position, Rapid Update. PGN 129025, 100ms update
            self.lat = struct.unpack('<l', msg.data[0:4])[0] / 10000000 #degrees
            self.lon = struct.unpack('<l', msg.data[4:8])[0] / 10000000 #degrees
            logger.debug(f"{msg.arbitration_id:08X} {msg.data.hex()} Lat: {self.lat:.6f}, Lon: {self.lon:.6f}")
            return {"lat": self.lat, "lon": self.lon}
        
        elif msg.arbitration_id == 0x18FEE81C:  #Vehicle Direction/Speed, PGN 65256, 1 second update
            compass = struct.unpack('<H', msg.data[0:2])[0] * 0.0078125 # degrees
            speed = struct.unpack('<H', msg.data[2:4])[0] * 0.00390625 * 1.60934  # km/h to mph
            pitch = struct.unpack('<H', msg.data[4:6])[0] * 0.0078125 #degrees
            altitude = struct.unpack('<H', msg.data[6:8])[0] * 0.125 * 3.28084  # meters to ft
            logger.debug(f"{msg.arbitration_id:08X} {msg.data.hex()} Compass: {compass:.2f}, Speed: {speed:.2f} mph, Pitch: {pitch:.2f}°, Altitude: {altitude:.2f} ft")
            return {"speed": speed, "compass": compass, "pitch": pitch, "altitude": altitude}

        elif msg.arbitration_id == 0x0CF00400:  #electronic engine control 1, PGN 61444, 10ms update
            if (now - self.rpm_start_time) > self.GUI_TIMEOUT:
                self.rpm_start_time = now
                self.rpm = struct.unpack('<H', msg.data[3:5])[0] * 0.125 # RPM
                logger.debug(f"{msg.arbitration_id:08X} {msg.data.hex()} rpm: {self.rpm:.0f}")
                return {"rpm": round(self.rpm)}
            else:
                return None  # Skip processing if too soon
        
        elif msg.arbitration_id == 0x09F112F8:  # Vessel Heading, PGN 127250 100ms update
            heading_raw = struct.unpack('<H', msg.data[1:3])[0] * 0.0001 * (180 / 3.14159)  # radians to degrees
            self.compass_offset = heading_raw - self.COG
            if self.compass_offset < -180:
                self.compass_offset += 360
            elif self.compass_offset >= 180:
                self.compass_offset -= 360
            self.compass_heading = heading_raw + self.heading_correction
            if self.boat_speed < 1:
                logger.debug(f"{msg.arbitration_id:08X} {msg.data.hex()} heading: {self.compass_heading:.2f}")
                return {"compass_heading": self.compass_heading}

        elif msg.arbitration_id == 0x18FF50E0: #Autopilot status
            engaged = bool(msg.data[0] & 0x01)
            left = bool(msg.data[0] & 0x10)
            right = bool(msg.data[0] & 0x20)
            self.heading_goal = (struct.unpack("<H", msg.data[1:3])[0] - 0x8000 )/ 10.0
            if self.heading_goal < 0:
                self.heading_goal += 360
            elif self.heading_goal >= 360:
                self.heading_goal -= 360
            heading_error = (struct.unpack_from("<H", msg.data, 3)[0] - 0x8000 )/ 100.0
            rudder_goal = (struct.unpack_from("<H", msg.data, 5)[0] - 0x8000 )/ 100.0
            counter = msg.data[7]
            logger.debug(f"{msg.arbitration_id:08X} {msg.data.hex()} Autopilot engaged: {engaged}, heading_goal: {self.heading_goal:.2f}, heading_error: {heading_error:.2f}, rudder_goal: {rudder_goal:0.2f} ")
            return {
                "right_turn": right,
                "left_turn": left,
                "autopilot_engaged": engaged,
                "heading_goal": self.heading_goal,
                "heading_error": heading_error,
                "rudder_goal": rudder_goal,
            }
        
        else:
            return None
