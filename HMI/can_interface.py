import can
import asyncio
import logging
import struct
import time
import queue

# Setup logger
logger = logging.getLogger("CANinterface")
logger.setLevel(logging.DEBUG)
logger.debug("CANinterface module loaded")

class CANinterface:
    def __init__(self, channel='can1', bitrate = 250000):
        if 'win' in sys.platform:
            device = 'pcan'           # The PCAN Drivers must be installed in Windows
            channel = 'PCAN_USBBUS1'  # Update this to your specific channel
        else:
            device = 'socketcan'
            channel = channel # Change this if there are more than 1 CAN adapter
          # Set the bitrate to 2500000 for all NMEA2000
        try:
            self.bus = can.interface.Bus(channel=channel, interface=device, bitrate=bitrate)
            logger.info(f"CAN bus initialized at {bitrate} on {channel} with {device}")
        except:
            logger.exception("CAN Interface Error: Please be sure hardware is plugged in.")
            self.bus = None

        self.listeners = []

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
        self.steering_start_time = time.time()
        self.compass_heading = 0.0
        self.boat_speed = 0.0
        self.heading_correction = None
        self.averaging_window = 100  # Number of samples for averaging
        self.heading_history = queue.Queue(maxsize=100)
        self.COG_history = queue.Queue(maxsize=100)

    def add_listener(self, callback):
        self.listeners.append(callback)

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
            self.steering_start_time = now
            
            angle = (struct.unpack('<L', msg.data[0:4])[0] - 0x80000000) / 1000
            goal_bytes = msg.data[4:8]
            if goal_bytes == b'\xFF\xFF\xFF\xFF':
                enabled = False
                goal = angle
            else:  # assume enabled if not FFs
                enabled = True
                goal = (struct.unpack('<L', goal_bytes)[0] - 0x80000000) / 1000
            logger.debug(f"{msg.arbitration_id:08X} {msg.data.hex()} Steering Angle: {angle:.2f}, Steering Goal: {goal:.2f}, Servo {'enabled' if enabled else 'disabled'}")
            return {"steering_angle": angle, "steering_goal": goal, "servo_enabled": enabled}
           
        elif msg.arbitration_id == 0x19F10D13:  # rudder
            rudder_count = msg.data[0]
            rudder_value = struct.unpack('<H', msg.data[1:3])[0]
            rudder_value_min = struct.unpack('<H', msg.data[3:5])[0]
            rudder_value_max = struct.unpack('<H', msg.data[5:7])[0]
            rudder_angle = (msg.data[7] - 0x80) / 2
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
            COG = COG_rad * (180 / 3.14159)
            SOG_mps = struct.unpack('<H', msg.data[4:6])[0] / 100.0  # Speed over ground in m/s
            SOG_mph = SOG_mps * 2.236936  # Convert to mph
            self.boat_speed = SOG_mph  # Update boat speed
            if self.boat_speed > 10 and self.heading_correction is None:
                self.COG_history.put(COG)
            logger.debug(f"{msg.arbitration_id:08X} {msg.data.hex()} COG: {COG:.2f}, SOG: {SOG_mph:.2f} mph")
            return {"SOG": SOG_mph, "COG": COG}

        elif msg.arbitration_id == 0x09F8011C:  # GPS Position, Rapid Update. PGN 129025, 100ms update
            lat = struct.unpack('<l', msg.data[0:4])[0] / 10000000 #degrees
            lon = struct.unpack('<l', msg.data[4:8])[0] / 10000000 #degrees
            logger.debug(f"{msg.arbitration_id:08X} {msg.data.hex()} Lat: {lat:.6f}, Lon: {lon:.6f}")
            return {"lat": lat, "lon": lon}
        
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
                rpm = struct.unpack('<H', msg.data[3:5])[0] * 0.125 # RPM
                logger.debug(f"{msg.arbitration_id:08X} {msg.data.hex()} rpm: {rpm:.0f}")
                return {"rpm": round(rpm)}
            else:
                return None  # Skip processing if too soon
        elif msg.arbitration_id == 0x09F112F8:  # Vessel Heading, PGN 127250 100ms update
            heading_raw = struct.unpack('<H', msg.data[1:3])[0] * 0.0001 * (180 / 3.14159)  # radians to degrees
            if self.heading_correction is None:
                if self.boat_speed > 10:
                    self.heading_history.put(heading_raw)
                if self.heading_history.full() and self.COG_history.full():
                    # Calculate average heading and COG
                    avg_heading = sum(list(self.heading_history.queue)) / self.heading_history.qsize()
                    avg_COG = sum(list(self.COG_history.queue)) / self.COG_history.qsize()
                    self.heading_correction = (avg_COG - avg_heading)
                    logger.debug(f"Average Heading: {avg_heading:.2f}, Average COG: {avg_COG:.2f}")
                    self.heading_history.queue.clear()
                    self.COG_history.queue.clear()

            if self.heading_correction is not None:
                heading = (heading_raw + self.heading_correction) % 360
            else:
                heading = heading_raw
            
            logger.debug(f"{msg.arbitration_id:08X} {msg.data.hex()} heading: {heading:.2f}")
            return {"heading": round(heading)}

        else:
            return None
