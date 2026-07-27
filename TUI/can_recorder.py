"""
Filtered CAN log recorder: write ONLY the CAN messages the HMI actually uses.

On the water there is no way to upload a full candump for analysis, and a full
bus capture is mostly noise. This records a candump-format log containing just
the frames the system decodes -- the steering/rudder feedback, the heading
sources, COG/SOG/position, engine, autopilot status, the heading node, and the
address claims (needed to identify sources by NAME). That log is small, is
exactly what the fusion sees, and can be shared or replayed directly.

Format matches python-can's CanutilsLogReader / candump -l:
    (<epoch>.<us>) <channel> <ID>#<DATAHEX>

so the existing fixtures and replay path read it without changes.

The recorder is fed from the same CAN read path as the decoder, so it is always
in sync with what the HMI actually uses. It rotates by size to bound disk use.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Iterable, Optional, Set


# The CAN IDs the HMI decodes (from can_interface.process_message). Heading
# (PGN 127250) is matched by PGN from ANY source, so it is handled specially
# below rather than listed as a fixed ID.
USED_CAN_IDS: Set[int] = {
    0x18F01D21,  # steering
    0x19F10D13,  # rudder
    0x09F8021C,  # GPS COG/SOG (PGN 129026)
    0x09F8011C,  # GPS position (PGN 129025)
    0x18FEE81C,  # vehicle direction (PGN 65256)
    0x0CF00400,  # engine (PGN 61444)
    0x18FF50E0,  # autopilot status
    0x18FF80E1,  # heading node: fused heading
    0x18FF81E1,  # heading node: attitude
    0x18FF82E1,  # heading node: health
}

# PGNs matched regardless of source address.
USED_PGNS: Set[int] = {
    0x1F112,     # 127250 vessel heading (wall compass, 24xd, any source)
}


def _pgn_of(can_id: int) -> int:
    pf = (can_id >> 16) & 0xFF
    if pf < 240:
        return (can_id >> 8) & 0x3FF00
    return (can_id >> 8) & 0x3FFFF


def is_used(can_id: int) -> bool:
    """True if this CAN ID is one the HMI decodes (so worth recording)."""
    if can_id in USED_CAN_IDS:
        return True
    if _pgn_of(can_id) in USED_PGNS:
        return True
    # Address claims (PGN 60928) identify sources by NAME -- always record them
    # so a replayed log can rebuild the source registry.
    pf = (can_id >> 16) & 0xFF
    if pf == 0xEE:
        return True
    return False


class FilteredCanRecorder:
    """
    Appends used CAN frames to a candump-format log, thread-safe, size-capped.

    Call record(msg) from the CAN read loop. The current file path is available
    as .path for display / download.
    """

    def __init__(self, directory: str = "logs", channel: str = "can0",
                 max_bytes: int = 20 * 1024 * 1024, prefix: str = "used_can"):
        self.directory = directory
        self.channel = channel
        self.max_bytes = max_bytes
        self.prefix = prefix
        self._lock = threading.Lock()
        self._fh = None
        self._bytes = 0
        self.path: Optional[str] = None
        self._count = 0
        self._start_time = time.time()
        os.makedirs(directory, exist_ok=True)
        self._open_new()

    def _open_new(self) -> None:
        ts = time.strftime("%Y-%m-%d_%H%M%S", time.localtime())
        self.path = os.path.join(self.directory, f"{self.prefix}-{ts}.log")
        self._fh = open(self.path, "a", buffering=1)  # line-buffered
        self._bytes = 0

    def record(self, can_id: int, data: bytes, when: Optional[float] = None) -> None:
        if not is_used(can_id):
            return
        t = when if when is not None else time.time()
        line = f"({t:.6f}) {self.channel} {can_id:08X}#{bytes(data).hex().upper()}\n"
        with self._lock:
            if self._fh is None:
                self._open_new()
            self._fh.write(line)
            self._bytes += len(line)
            self._count += 1
            if self._bytes >= self.max_bytes:
                # rotate
                try:
                    self._fh.close()
                except Exception:
                    pass
                self._open_new()

    def record_message(self, msg) -> None:
        """Convenience for a python-can Message."""
        self.record(msg.arbitration_id, msg.data,
                    getattr(msg, "timestamp", None) or time.time())

    def stats(self) -> dict:
        with self._lock:
            return {
                "path": self.path,
                "count": self._count,
                "bytes": self._bytes,
                "seconds": time.time() - self._start_time,
            }

    def close(self) -> None:
        with self._lock:
            if self._fh is not None:
                try:
                    self._fh.close()
                finally:
                    self._fh = None
