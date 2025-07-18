import can
import logging
import struct
import time
import sys

# Setup logger
logger = logging.getLogger("SensorFusion")
logger.setLevel(logging.DEBUG)
logger.debug("Sensor Fusion module loaded")

class SensorFusion:
    def __init__(self, bus):
        self.bus = bus
        