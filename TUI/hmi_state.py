"""
hmi_state.py -- Single source of truth for HMI display state.

Design notes
------------
* Every mutation of heading_goal goes through set_goal(value, source). The
  source string is what lets the TUI show "GOAL 130.0 <- web/192.168.1.47"
  when a phone moves the goal, and "<- tui" when the arrow keys do.
* Signals carry their own freshness. A value is not just a number, it is a
  number plus the wall time it arrived. Anything that renders can then ask
  "how old is this?" rather than trusting a stale float forever.
* Faults are latched with a first-seen and last-seen time so a fault that
  flickers still leaves a trace on the screen.
* No curses, no CAN, no Tornado imports here. This module is pure data so it
  can be unit tested and so both consumers depend on it rather than on
  each other.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, List, Optional

# --------------------------------------------------------------------------
# Fault codes
# --------------------------------------------------------------------------

# Link / transport level
F_LINK_DOWN = "LINK_DOWN"            # interface will not open at all
F_NO_TRAFFIC = "NO_TRAFFIC"          # bus opened, but nothing is arriving
F_BUS_ERRORS = "BUS_ERRORS"          # kernel error counters climbing
F_BUS_OFF = "BUS_OFF"                # controller went bus-off
F_TX_FAIL = "TX_FAIL"                # send() raised

# Message level
F_STALE = "STALE"                    # a watched PGN stopped arriving

# Data quality
F_HEADING_NOISE = "HEADING_NOISE"    # fused heading sigma too large
F_HEADING_INVALID = "HEADING_INVALID"  # no usable heading at all

# Severity ordering for display. Higher sorts first.
SEVERITY = {
    F_BUS_OFF: 50,
    F_LINK_DOWN: 45,
    F_NO_TRAFFIC: 40,
    F_BUS_ERRORS: 35,
    F_TX_FAIL: 30,
    F_HEADING_INVALID: 25,
    F_HEADING_NOISE: 20,
    F_STALE: 10,
}


@dataclass
class Signal:
    """A scalar value plus when it last changed and how stale it may be."""

    name: str
    value: Any = None
    last_rx: float = 0.0
    stale_after: float = 2.0
    units: str = ""

    def update(self, value: Any, when: Optional[float] = None) -> None:
        self.value = value
        self.last_rx = when if when is not None else time.time()

    def age(self, now: Optional[float] = None) -> float:
        if self.last_rx == 0.0:
            return float("inf")
        return (now if now is not None else time.time()) - self.last_rx

    def is_stale(self, now: Optional[float] = None) -> bool:
        return self.age(now) > self.stale_after

    def is_valid(self, now: Optional[float] = None) -> bool:
        return self.value is not None and not self.is_stale(now)


@dataclass
class Fault:
    code: str
    detail: str = ""
    first_seen: float = field(default_factory=time.time)
    last_seen: float = field(default_factory=time.time)
    count: int = 1

    @property
    def severity(self) -> int:
        return SEVERITY.get(self.code, 0)


@dataclass
class SourceHealth:
    """Per-CAN-ID arrival tracking."""

    can_id: int
    name: str
    stale_after: float
    last_rx: float = 0.0
    count: int = 0
    ever_seen: bool = False

    def mark(self, when: Optional[float] = None) -> None:
        self.last_rx = when if when is not None else time.time()
        self.count += 1
        self.ever_seen = True

    def age(self, now: Optional[float] = None) -> float:
        if not self.ever_seen:
            return float("inf")
        return (now if now is not None else time.time()) - self.last_rx

    def status(self, now: Optional[float] = None) -> str:
        if not self.ever_seen:
            return "NEVER"
        return "STALE" if self.age(now) > self.stale_after else "OK"


class SystemState:
    """
    Thread-safe container for everything the HMI displays.

    The CAN reader thread writes; the TUI and the WebSocket broadcaster read.
    A single RLock guards all of it. Contention is negligible at these rates
    (a few hundred updates/sec against two readers at 5-10 Hz).
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()

        # ---- link state -------------------------------------------------
        self.link_up: bool = False
        self.link_detail: str = "not started"
        self.backend: str = ""
        self.channel: str = ""
        self.bitrate: int = 0
        self.rx_total: int = 0
        self.tx_total: int = 0
        self.rx_error_count: int = 0
        self.tx_error_count: int = 0
        self.bus_state: str = "UNKNOWN"
        self.last_rx_any: float = 0.0
        self.reconnect_attempts: int = 0
        self.next_reconnect_at: float = 0.0

        # ---- signals ----------------------------------------------------
        # stale_after values are roughly 3x the nominal publish interval so a
        # single dropped frame does not flag.
        self.signals: Dict[str, Signal] = {
            "compass_heading": Signal("compass_heading", stale_after=3.0, units="deg"),
            "cog": Signal("cog", stale_after=3.0, units="deg"),
            "sog": Signal("sog", stale_after=3.0, units="mph"),
            "fused_heading": Signal("fused_heading", stale_after=2.0, units="deg"),
            "heading_sigma": Signal("heading_sigma", stale_after=2.0, units="deg"),
            "yaw_rate": Signal("yaw_rate", stale_after=2.0, units="deg/s"),
            "pitch": Signal("pitch", stale_after=5.0, units="deg"),
            "roll": Signal("roll", stale_after=5.0, units="deg"),
            "steering_angle": Signal("steering_angle", stale_after=1.0, units="cnt"),
            "steering_goal": Signal("steering_goal", stale_after=1.0, units="cnt"),
            "rudder_angle": Signal("rudder_angle", stale_after=2.0, units="deg"),
            "rudder_value": Signal("rudder_value", stale_after=2.0, units="cnt"),
            "rpm": Signal("rpm", stale_after=2.0, units="rpm"),
            "lat": Signal("lat", stale_after=3.0, units="deg"),
            "lon": Signal("lon", stale_after=3.0, units="deg"),
            "shaft_center": Signal("shaft_center", stale_after=1e9, units="cnt"),
            "compass_offset": Signal("compass_offset", stale_after=5.0, units="deg"),
        }

        # ---- control state ----------------------------------------------
        self.heading_goal: float = 0.0
        self.goal_source: str = "init"
        self.goal_changed_at: float = 0.0
        self.autopilot_engaged: bool = False
        self.servo_enabled: bool = False
        self.left_turn: bool = False
        self.right_turn: bool = False
        self.heading_error: float = 0.0

        # ---- filter diagnostics -----------------------------------------
        self.cog_accepted: int = 0
        self.cog_rejected: int = 0
        self.filter_ready: bool = False

        # ---- source health ----------------------------------------------
        self.sources: Dict[int, SourceHealth] = {}

        # ---- faults ------------------------------------------------------
        self.faults: Dict[str, Fault] = {}

        # ---- event log ---------------------------------------------------
        self.events: Deque[tuple] = deque(maxlen=200)

        # ---- web clients --------------------------------------------------
        self.web_clients: int = 0
        self.web_last_activity: float = 0.0

        # ---- subscribers ---------------------------------------------------
        self._subscribers: List[Callable[[str, dict], None]] = []

    # -- subscription ------------------------------------------------------

    def subscribe(self, fn: Callable[[str, dict], None]) -> None:
        """
        Register a callback invoked as fn(kind, payload) on notable changes.
        Kinds: 'goal', 'fault', 'fault_clear', 'event', 'engage'.
        Callbacks must not block; the TUI and WS both just enqueue.
        """
        with self._lock:
            self._subscribers.append(fn)

    def _notify(self, kind: str, payload: dict) -> None:
        # Copy the list so a subscriber that unsubscribes mid-iteration
        # cannot mutate what we are walking.
        for fn in list(self._subscribers):
            try:
                fn(kind, payload)
            except Exception:  # a bad subscriber must not kill the producer
                pass

    # -- register sources --------------------------------------------------

    def register_source(self, can_id: int, name: str, stale_after: float) -> None:
        with self._lock:
            self.sources[can_id] = SourceHealth(can_id, name, stale_after)

    def mark_source(self, can_id: int, when: Optional[float] = None) -> None:
        with self._lock:
            src = self.sources.get(can_id)
            if src is not None:
                src.mark(when)
            self.rx_total += 1
            self.last_rx_any = when if when is not None else time.time()

    # -- signals -----------------------------------------------------------

    def set_signal(self, name: str, value: Any, when: Optional[float] = None) -> None:
        with self._lock:
            sig = self.signals.get(name)
            if sig is None:
                sig = Signal(name)
                self.signals[name] = sig
            sig.update(value, when)

    def get_signal(self, name: str) -> Optional[Signal]:
        with self._lock:
            return self.signals.get(name)

    def signal_value(self, name: str, default: Any = None) -> Any:
        with self._lock:
            sig = self.signals.get(name)
            return default if sig is None or sig.value is None else sig.value

    # -- heading goal ------------------------------------------------------

    def set_goal(self, value: float, source: str) -> float:
        """
        Set the absolute heading goal. Last write wins regardless of source.
        Returns the normalized goal actually stored.

        Normalization uses modulo so that a delta of any magnitude, or a
        garbage value from a client, still lands in [0, 360).
        """
        try:
            raw = float(value)
        except (TypeError, ValueError):
            self.log_event("WARN", f"bad goal from {source}: {value!r}")
            return self.heading_goal

        # NaN and inf both survive float() and both yield NaN from the modulo
        # below, which would silently poison the setpoint: every subsequent
        # comparison against it is False and the PID error becomes NaN. A
        # malformed WebSocket payload must not be able to do that.
        if not math.isfinite(raw):
            self.log_event("WARN", f"non-finite goal from {source}: {value!r}")
            return self.heading_goal

        v = raw % 360.0

        with self._lock:
            old = self.heading_goal
            self.heading_goal = v
            self.goal_source = source
            self.goal_changed_at = time.time()

        if abs(((v - old + 180.0) % 360.0) - 180.0) > 1e-9:
            self.log_event("GOAL", f"{v:.1f} <- {source}")
            self._notify("goal", {"heading_goal": v, "source": source})
        return v

    def adjust_goal(self, delta: float, source: str) -> float:
        with self._lock:
            base = self.heading_goal
        return self.set_goal(base + delta, source)

    # -- faults ------------------------------------------------------------

    def raise_fault(self, code: str, detail: str = "") -> None:
        now = time.time()
        with self._lock:
            existing = self.faults.get(code)
            if existing is None:
                self.faults[code] = Fault(code, detail, now, now, 1)
                new = True
            else:
                existing.last_seen = now
                existing.count += 1
                new = detail != existing.detail
                existing.detail = detail or existing.detail
        if new:
            self.log_event("FAULT", f"{code}{': ' + detail if detail else ''}")
            self._notify("fault", {"code": code, "detail": detail})

    def clear_fault(self, code: str) -> None:
        with self._lock:
            had = self.faults.pop(code, None)
        if had is not None:
            self.log_event("CLEAR", code)
            self._notify("fault_clear", {"code": code})

    def active_faults(self) -> List[Fault]:
        with self._lock:
            return sorted(
                self.faults.values(), key=lambda f: (-f.severity, f.code)
            )

    def worst_fault(self) -> Optional[Fault]:
        f = self.active_faults()
        return f[0] if f else None

    # -- event log ---------------------------------------------------------

    def log_event(self, kind: str, text: str) -> None:
        entry = (time.time(), kind, text)
        with self._lock:
            self.events.append(entry)
        self._notify("event", {"kind": kind, "text": text, "ts": entry[0]})

    def recent_events(self, n: int = 10) -> List[tuple]:
        with self._lock:
            return list(self.events)[-n:]

    # -- snapshot ----------------------------------------------------------

    def snapshot(self) -> dict:
        """
        JSON-serializable view for the WebSocket. Keys deliberately match the
        existing compass.js contract so the browser needs no changes for the
        fields it already reads; new keys are additive.
        """
        now = time.time()
        with self._lock:
            sv = self.signal_value
            snap = {
                "ts": now,
                # existing contract
                "rpm": sv("rpm"),
                "speed": sv("sog"),
                "sog_mps": (sv("sog") / 2.236936) if sv("sog") is not None else None,
                "cog_deg": sv("cog"),
                "compass_deg": sv("compass_heading"),
                "heading_deg": sv("fused_heading"),
                "heading_goal": self.heading_goal,
                "headingErr": self.heading_error,
                "lat": sv("lat"),
                "lon": sv("lon"),
                "rudder_counts": sv("rudder_value"),
                "shaft_goal": sv("steering_goal"),
                "autopilot_engaged": self.autopilot_engaged,
                "servo_enabled": self.servo_enabled,
                "compass_offset": sv("compass_offset"),
                # new: goal provenance so the phone can show who moved it
                "goal_source": self.goal_source,
                "goal_changed_at": self.goal_changed_at,
                # new: link + health
                "link_up": self.link_up,
                "link_detail": self.link_detail,
                "bus_state": self.bus_state,
                "backend": self.backend,
                "channel": self.channel,
                "bitrate": self.bitrate,
                "rx_total": self.rx_total,
                # new: filter diagnostics
                "heading_sigma": sv("heading_sigma"),
                "yaw_rate": sv("yaw_rate"),
                "filter_ready": self.filter_ready,
                "cog_accepted": self.cog_accepted,
                "cog_rejected": self.cog_rejected,
                # new: faults
                "faults": [
                    {"code": f.code, "detail": f.detail, "count": f.count}
                    for f in sorted(
                        self.faults.values(), key=lambda x: -x.severity
                    )
                ],
                "sources": [
                    {
                        "id": s.can_id,
                        "name": s.name,
                        "age": None if s.age(now) == float("inf") else round(s.age(now), 2),
                        "status": s.status(now),
                        "count": s.count,
                    }
                    for s in self.sources.values()
                ],
            }
        return snap
