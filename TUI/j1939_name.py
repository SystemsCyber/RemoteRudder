"""
J1939 address-claim (NAME) decoding for the source table.

Every J1939 node has a 64-bit NAME it announces via PGN 60928 (0xEE00,
"Address Claimed"). The NAME encodes who the device is: manufacturer, function,
vehicle system, and a unique identity number. Decoding it turns the source
table from a list of bare hex IDs into "Garmin compass, function Navigation"
which is what you need to tell two heading sources apart.

Claims are periodic and also sent on request. A request for PGN 60928 (via the
Request PGN, 0xEA00, addressed to the global 0xFF) forces every node to
re-announce, which is how the "request addresses" command populates this table
on demand -- some nodes (the Garmin heading source among them) do not claim
often on their own.

NAME bit layout (64-bit little-endian, SAE J1939-81):
    bits  0-20   Identity Number        (21 bits)
    bits 21-31   Manufacturer Code      (11 bits)
    bits 32-34   ECU Instance           (3 bits)
    bits 35-39   Function Instance      (5 bits)
    bits 40-47   Function               (8 bits)
    bit  48      Reserved               (1 bit)
    bits 49-55   Vehicle System         (7 bits)
    bits 56-59   Vehicle System Instance(4 bits)
    bits 60-62   Industry Group         (3 bits)
    bit  63      Arbitrary Address Capable (1 bit)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


# ---------------------------------------------------------------------------
# Manufacturer codes (NMEA 2000 / J1939 registered, marine-relevant subset)
#
# The full list is hundreds long; this covers the marine electronics likely to
# appear on this boat plus a few common industrial ones. Unknown codes fall
# back to "Mfg <n>" so the number is always visible.
# ---------------------------------------------------------------------------

MANUFACTURERS = {
    78: "FW Murphy",
    85: "Maretron",
    88: "Furuno",
    114: "Garmin",
    116: "Yanmar",
    135: "Airmar",
    137: "Maretron",
    140: "Lowrance",
    144: "Mercury Marine",
    147: "Nautibus",
    165: "B&G",
    172: "Simrad",
    174: "Volvo Penta",
    193: "Yamaha Marine",
    198: "Raymarine",
    229: "Garmin",
    272: "Actisense",
    273: "Chetco",
    275: "Navico",
    283: "Yacht Devices",
    285: "Vetus",
    295: "Digital Yacht",
    304: "EmpirBus",
    305: "NovAtel",
    311: "SeaCross",
    315: "Fischer Panda",
    351: "Kvasar",
    355: "Sailmon",
    373: "Garmin",
    381: "Carling",
    396: "Victron Energy",
    404: "Garmin",
    421: "NKE",
    427: "Beede",
    437: "MilLtech",
    459: "Blue Water Data",
    481: "Garmin",
    502: "Garmin",
}


# ---------------------------------------------------------------------------
# Function codes for Marine (industry group 4). Function meaning is
# industry-group dependent; these are the marine device functions.
# ---------------------------------------------------------------------------

MARINE_FUNCTIONS = {
    0: "Reserved",
    10: "System Tools",
    20: "Safety",
    25: "Internetwork",
    30: "Gateway",
    35: "Gateway",
    40: "Display",
    50: "Sensor Comms",
    60: "Sensor Comms",
    70: "Nav Management",
    130: "Navigation",       # GPS / GNSS position
    140: "Heading Sensor",   # compass
    145: "Attitude/Compass",
    150: "Engine",
    160: "Engine Gateway",
    170: "Steering",
    190: "Rudder",
    195: "Autopilot",
    200: "Instrument",
    205: "Engine",
    210: "Battery/Charger",
}

# Vehicle System codes are also industry-dependent; for marine, a few:
MARINE_VEHICLE_SYSTEMS = {
    0: "Nonspecific",
    10: "Vessel",
    20: "Engine",
    30: "Navigation",
    40: "Instrumentation",
}


@dataclass
class J1939Name:
    """A decoded 64-bit J1939 NAME from an address claim."""

    raw: int
    identity: int
    mfg_code: int
    ecu_instance: int
    function_instance: int
    function: int
    vehicle_system: int
    vehicle_system_instance: int
    industry_group: int
    arbitrary_address: bool

    @property
    def manufacturer(self) -> str:
        return MANUFACTURERS.get(self.mfg_code, f"Mfg {self.mfg_code}")

    @property
    def function_name(self) -> str:
        # Function meaning depends on industry group; marine is group 4.
        if self.industry_group == 4:
            return MARINE_FUNCTIONS.get(self.function, f"Fn {self.function}")
        return f"Fn {self.function}"

    @property
    def vehicle_system_name(self) -> str:
        if self.industry_group == 4:
            return MARINE_VEHICLE_SYSTEMS.get(
                self.vehicle_system, f"Sys {self.vehicle_system}"
            )
        return f"Sys {self.vehicle_system}"

    @property
    def industry_name(self) -> str:
        return {
            0: "Global",
            1: "On-Highway",
            2: "Agriculture",
            3: "Construction",
            4: "Marine",
            5: "Industrial",
        }.get(self.industry_group, f"IG {self.industry_group}")


def decode_name(data: bytes) -> Optional[J1939Name]:
    """
    Decode an 8-byte J1939 address-claim payload into a NAME.

    Returns None if the payload is not 8 bytes. The NAME is little-endian:
    byte 0 is the LSB of the identity number.
    """
    if data is None or len(data) < 8:
        return None
    name = int.from_bytes(data[:8], "little")
    return J1939Name(
        raw=name,
        identity=name & 0x1FFFFF,
        mfg_code=(name >> 21) & 0x7FF,
        ecu_instance=(name >> 32) & 0x7,
        function_instance=(name >> 35) & 0x1F,
        function=(name >> 40) & 0xFF,
        vehicle_system=(name >> 49) & 0x7F,
        vehicle_system_instance=(name >> 56) & 0xF,
        industry_group=(name >> 60) & 0x7,
        arbitrary_address=bool((name >> 63) & 0x1),
    )


def is_address_claim(can_id: int) -> bool:
    """True if this extended CAN ID is a PGN 60928 (Address Claimed) message."""
    # PGN 60928 = 0xEE00. In the 29-bit ID: bits 8-25 hold the PGN. For PDU1
    # the PGN's low byte is the destination, so match the PF byte (0xEE).
    pf = (can_id >> 16) & 0xFF
    return pf == 0xEE


def source_address(can_id: int) -> int:
    """The J1939 source address is the low byte of the arbitration ID."""
    return can_id & 0xFF


def build_request_for_address_claim(source_addr: int = 0xFE) -> int:
    """
    Build the arbitration ID for a Request (PGN 59904, 0xEA00) for the
    Address Claimed PGN (60928), addressed to the global address 0xFF.

    The 3-byte data payload is the requested PGN in little-endian
    (0x00 0xEE 0x00) -- the caller sends that as the message data. Priority 6.

    Returns the 29-bit arbitration ID. Default source 0xFE is the J1939
    "null / service tool" address, appropriate for a diagnostic requester that
    has not claimed an address of its own.
    """
    priority = 6
    pf = 0xEA          # Request PGN
    ps = 0xFF          # global destination
    return (priority << 26) | (pf << 16) | (ps << 8) | source_addr


# The 3-byte data payload for a request targeting PGN 60928.
REQUEST_ADDRESS_CLAIM_DATA = bytes([0x00, 0xEE, 0x00])
