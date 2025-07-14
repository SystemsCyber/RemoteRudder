import can
import asyncio
import logging
import struct
import time

# Setup logger
logger = logging.getLogger("CANReader")
logger.setLevel(logging.DEBUG)
logger.debug("CANReader module loaded")

class CANReader:
    def __init__(self, bus_channel='can1', bustype='socketcan'):
        self.bus = can.interface.Bus(channel=bus_channel, bustype=bustype)
        logger.info(f"CAN bus initialized on {bus_channel} with bustype {bustype}")
        self.listeners = []

        # Track last-received time by PGN
        self.watch_ids = {
            0x18F01D21: {"name": "steering", "last_time": time.time()},
            0x19F10D13: {"name": "rudder", "last_time": time.time()},
            0x09F8021C: {"name": "gps_position", "last_time": time.time()},
            0x18FEE81C: {"name": "vehicle_direction", "last_time": time.time()},
            0x0CF00400: {"name": "engine_control", "last_time": time.time()},
            0x09F8011C: {"name": "gps_speed", "last_time": time.time()},
            0x09F8021C: {"name": "gps_position_rapid", "last_time": time.time()},
        }
        self.timeout_interval = 1.0

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
        if msg.arbitration_id == 0x18F01D21:  # steering
            angle = (struct.unpack('<L', msg.data[0:4])[0] - 0x80000000) / 1000
            goal = (struct.unpack('<L', msg.data[4:8])[0] - 0x80000000) / 1000
            logger.debug(f"{self.bus.channel_info}: {msg.arbitration_id:08X}  Data: {msg.data.hex()}: Steering Angle: {angle:.2f}, Steering Goal: {goal:.2f}")
            return {"steering_angle": angle, "steering_goal": goal}

        elif msg.arbitration_id == 0x19F10D13:  # rudder
            rudder_angle = struct.unpack('<h', msg.data[4:6])[0] / 1000 * (180 / 3.14159)  # radians to degrees
            logger.debug(f"Rudder Angle: {rudder_angle:.2f}°")
            return {"rudder_angle": rudder_angle}

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
            COG_rad = struct.unpack('<H', msg.data[2:4])[0] / 1000  # COG in radians
            COG = COG_rad * (180 / 3.14159)
            SOG_mps = struct.unpack('<H', msg.data[4:6])[0] / 100.0  # Speed over ground in m/s
            SOG_mph = SOG_mps * 2.236936  # Convert to mph
            logger.debug(f"GPS Data: COG: {COG:.2f}° (ref {COG_ref}), SOG: {SOG_mph:.2f} mph")
            return {"SOG": SOG_mph, "COG": COG}

        elif msg.arbitration_id == 0x09F8011C:  # GPS Position, Rapid Update. PGN 129025, 100ms update
            lat = struct.unpack('<l', msg.data[0:4])[0] / 10000000 #degrees
            lon = struct.unpack('<l', msg.data[4:8])[0] / 10000000 #degrees
            return {"lat": lat, "lon": lon}
        
        elif msg.arbitration_id == 0x18FEE81C:  #Vehicle Direction/Speed, PGN 65256, 1 second update
            compass = struct.unpack('<H', msg.data[0:2])[0] * 0.0078125 # degrees
            speed = struct.unpack('<H', msg.data[2:4])[0] * 0.00390625 * 1.60934  # km/h to mph
            logger.debug(f"Vehicle Speed: {speed:.2f} mph, Compass: {compass:.2f}°")
            pitch = struct.unpack('<H', msg.data[4:6])[0] * 0.0078125 #degrees
            altitude = struct.unpack('<H', msg.data[6:8])[0] * 0.125 * 3.28084  # meters to ft
            logger.debug(f"Vehicle Altitude: {altitude:.2f} ft, Pitch: {pitch:.2f}°")
            return {"speed": speed, "compass": compass, "pitch": pitch, "altitude": altitude}    
        
        elif msg.arbitration_id == 0x0CF00400:  #electronic engine control 1, PGN 61444, 10ms update
            rpm = struct.unpack('<H', msg.data[3:5])[0] * 0.125 # RPM
            return {"rpm": rpm}
        else:
            return None
