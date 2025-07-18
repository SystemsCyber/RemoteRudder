import struct
import json
import logging
import can

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class CANWriter:
    def __init__(self, bus_channel='can1', bustype='socketcan'):
        self.bus = can.Bus(channel=bus_channel, bustype=bustype)
        self.current_goal = 0.0  # Start at 0 degrees
        self.servo_enabled = False  # track state

    def adjust_goal(self, delta_degrees):
        self.current_goal += delta_degrees
        self.send_goal()
    
    def set_servo_enabled(self, enable):
        self.servo_enabled = enable
        logger.info(f"Servo {'enabled' if enable else 'disabled'}")
        self.send_goal()  # re-send current goal with updated enable byte

    def set_goal(self, goal):
        self.current_goal = goal
        logger.info(f"Setting steering goal to {goal} degrees")

    def send_goal(self):
        angle_raw = int((self.current_goal * 1000) + 0x80000000) & 0xFFFFFFFF
        angle_bytes = angle_raw.to_bytes(4, byteorder='little')
        enable_byte = b'\x45' if self.servo_enabled else b'\x00'

        data = angle_bytes + enable_byte
        msg = can.Message(arbitration_id=0x0CF34155, data=data, is_extended_id=True)
        try:
            self.bus.send(msg)
            logger.info(f"Sent CAN steering goal: {self.current_goal:.1f}°  {data.hex()}")
        except can.CanError as e:
            logger.error(f"CAN send failed: {e}")
