import os
import re
from pathlib import Path

REMOVE_IDS = ["18FF50E0", "0CF34155"]
INPUT_DIR = "./"        # Adjust this as needed
OUTPUT_FILE = "combined_filtered.log"
INTER_FILE_GAP = 0.001              # 1ms

timestamp_re = re.compile(r"\((\d+\.\d+)\)")

def extract_timestamp(line):
    match = timestamp_re.match(line)
    return float(match.group(1)) if match else None

def process_file(path):
    filtered_lines = []
    base_time = None

    with open(path, "r") as f:
        for line in f:
            parts = line.strip().split()
            id = parts[2].split("#")[0].upper()
            if len(parts) >= 3 and id not in REMOVE_IDS:
                ts = extract_timestamp(line)
                if ts is not None:
                    if base_time is None:
                        base_time = ts
                    rel_time = ts - base_time
                    filtered_lines.append((rel_time, line))
            else:
                pass
    return filtered_lines

def main():
    all_lines = []
    cumulative_offset = 0.0

    for file in sorted(Path(INPUT_DIR).glob("*.log")):
        print(f"Processing {file.name}")
        rel_lines = process_file(file)
        for rel_time, line in rel_lines:
            adjusted_time = cumulative_offset + rel_time
            new_line = re.sub(r"\(\d+\.\d+\)", f"({adjusted_time:.6f})", line)
            all_lines.append((adjusted_time, new_line))
        if rel_lines:
            last_rel_time = rel_lines[-1][0]
            cumulative_offset += last_rel_time + INTER_FILE_GAP

    with open(OUTPUT_FILE, "w") as out:
        for _, line in sorted(all_lines):
            out.write(line)

    print(f"Combined log written to {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
