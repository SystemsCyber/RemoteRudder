#!/bin/bash
# Regression harness: run the filter over every capture with a compass.
D="${1:-/home/claude/repo/RemoteRudder-main/CANdata}"
for f in "$D"/candump-*.log; do
  python3 replay.py "$f" 2>/dev/null | grep -E "^(candump|sensor|compass|cog|delta|heading sigma|   mean|validation)"
  echo
done
