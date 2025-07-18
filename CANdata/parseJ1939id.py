import re
import json
from j1939helpers import *

with open('CANdata/BasicJ1939.json', 'r') as f:
    pgn_labels = json.load(f)
'''
Example PGN Labels{
    "PGN": {
        "0": {
            "PG Label": "Torque/Speed Control 1"
        },
        "256": {
            "PG Label": "Transmission Control 1"
        },
'''
# Load the new TRC file
file_path = "CANdata/ZigZag on Horsetooth with 1200 rpm.trc"

with open(file_path, 'r', encoding='latin1') as f:
    lines = f.readlines()

# Parse TRC lines and convert to candump format with correct timestamps and formatting
candump_lines = []
start_time = None

summary = {}
source_addresses = {}

for line in lines:
    match = re.match(
        r'^\s*\d+\s+([\d.]+)\s+DT\s+([0-9A-F]{8})\s+Rx\s+(\d)\s+((?:[0-9A-F]{2}\s*)+)',
        line.strip()
    )
    if match:
        timestamp_ms, can_id_hex, dlc, data_str = match.groups()
        timestamp = float(timestamp_ms) / 1000.0  # Convert ms to seconds
        if start_time is None:
            start_time = timestamp
        rel_timestamp = timestamp - start_time
        can_id = int(can_id_hex, 16)
        can_id_str = f"{can_id:08X}"  # Ensure 8-digit hex
        data = data_str.strip().split()
        (priority, pgn, da, sa) = parseCANid(can_id)
        if sa == 0xF3:
            print(f"{can_id_str} -> Priority={priority}, PGN={pgn}, DA={da}, SA={sa}, Data={data}")
        key = f"{pgn}_{sa}"
        if key not in summary:
            summary[key] = {
                "PGN": pgn,
                "SA": sa,
                "SA_Hex": f"{sa:02X}",
                "PGN_Hex": f"{pgn:06X}",
                "DA": da,
                "priority": priority,
                "count": 1,
                "PG Label": pgn_labels["PGN"].get(str(pgn), {}).get("PG Label", "Unknown"),
            }

        summary[key]["count"] += 1

        if sa not in source_addresses:
            source_addresses[sa] = {
                "PGNs": {},
                "SA_Hex": f"{sa:02X}",
                "count": 1,
            }

        source_addresses[sa]["PGNs"][pgn] = pgn_labels["PGN"].get(str(pgn), {}).get("PG Label", "Unknown")
        source_addresses[sa]["count"] += 1

print(json.dumps(summary, indent=4))
print(json.dumps(source_addresses, indent=4))