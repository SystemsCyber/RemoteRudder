'''
# This script converts a TRC file to a candump format log file.
# It reads the TRC file, extracts relevant CAN data, and formats it for use with canplayer.
'''
import re

# Load the new TRC file
file_path = "CANdata\ZigZag on Horsetooth with 1200 rpm.trc"

with open(file_path, 'r', encoding='latin1') as f:
    lines = f.readlines()

# Parse TRC lines and convert to candump format with correct timestamps and formatting
candump_lines = []
start_time = None

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
        candump_line = f"({rel_timestamp:.6f}) can1 {can_id_str}#{''.join(data)}"
        candump_lines.append(candump_line)

# Save converted log
output_path = file_path + ".log"
with open(output_path, 'w') as f:
    for line in candump_lines:
        f.write(line + '\n')
print(f"Converted {len(candump_lines)} lines to {output_path}")

'''

# Example command to run canplayer with the generated log file
# Note: Adjust the path to your actual log file location
canplayer -l i -v -I Documents/Figure\ 8\ on\ horsetooth\ south\ bay\ full\ turns.trc.log
'''