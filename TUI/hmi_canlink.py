"""
hmi_canlink.py -- Bus lifecycle, health monitoring, and fault detection.

This wraps CANinterface rather than replacing it. The decode logic in
can_interface.process_message is left alone; what changes is everything
around it:

  * Opening the bus never calls sys.exit(). A missing adapter becomes a
    LINK_DOWN fault and a retry timer.
  * Bare `except:` is replaced with typed handlers so a KeyboardInterrupt
    or a genuine bug is not swallowed as "hardware not plugged in".
  * Bus health is polled from the kernel on Linux (rx/tx error counters,
    bus state) which is what actually distinguishes a wrong bitrate from
    a quiet bus.

Failure modes this is built to name correctly:

  Hardware not plugged in -> Bus() raises at open, or the netdev is absent.
                             Fault LINK_DOWN, retry with backoff.
  Wrong baud rate         -> Bus opens fine (socketcan always does), but no
                             frames arrive and the error counters climb as
                             the controller NAKs everything it half-hears.
                             Fault NO_TRAFFIC and/or BUS_ERRORS.
  Missing messages        -> Per-ID age tracking in SystemState. One PGN
                             goes quiet while others keep flowing.
                             Fault STALE with the source names.
  Noisy heading           -> Handled in hmi_filter/HeadingMonitor, which
                             raises HEADING_NOISE off the variance estimate.
"""

from __future__ import annotations

import asyncio
import logging
import os
import platform
import re
import subprocess
import sys
import time
from typing import Callable, Optional

import can

from hmi_state import (
    F_BUS_ERRORS,
    F_BUS_OFF,
    F_LINK_DOWN,
    F_NO_TRAFFIC,
    F_STALE,
    F_TX_FAIL,
    SystemState,
)

logger = logging.getLogger("canlink")

IS_LINUX = sys.platform.startswith("linux")
IS_WINDOWS = sys.platform.startswith("win")

# How long the bus can be silent after a successful open before we suspect
# the bitrate. Chosen well above the slowest PGN interval (1 s for
# vehicle_direction) with margin.
NO_TRAFFIC_TIMEOUT = 4.0

# Reconnect backoff schedule, seconds.
BACKOFF = [1.0, 2.0, 4.0, 8.0, 15.0, 30.0]


def backend_for_platform(requested: Optional[str], channel: str):
    """
    Resolve (interface, channel) for the current platform.

    Keeping this in one function is what makes the Windows port a config
    change rather than a rewrite. python-can presents the same Bus API for
    socketcan and pcan; only construction differs.
    """
    if requested == "virtual":
        return "virtual", "vcan0"
    if requested in ("socketcan", "pcan"):
        return requested, channel
    if IS_WINDOWS:
        # PEAK driver channel naming.
        return "pcan", channel if channel.startswith("PCAN_") else "PCAN_USBBUS1"
    return "socketcan", channel


def socketcan_iface_exists(channel: str) -> bool:
    """True if the netdev is present. Cheap pre-check before opening."""
    if not IS_LINUX:
        return True
    return os.path.isdir(f"/sys/class/net/{channel}")


def read_socketcan_health(channel: str) -> dict:
    """
    Pull bus state and error counters from the kernel.

    This is the single most useful diagnostic for a bitrate mismatch: the
    controller sees dominant/recessive transitions at the wrong times,
    generates form/stuff errors, and the tx/rx error counters climb even
    though zero valid frames are delivered to userspace. A quiet-but-correct
    bus shows zero errors instead.

    Returns {} on any platform or condition where this is unavailable, and
    callers degrade to the timeout heuristic.
    """
    if not IS_LINUX:
        return {}
    try:
        out = subprocess.run(
            ["ip", "-details", "-statistics", "link", "show", channel],
            capture_output=True,
            text=True,
            timeout=1.0,
        )
        if out.returncode != 0:
            return {}
        txt = out.stdout
    except (OSError, subprocess.SubprocessError):
        return {}

    health = {}

    m = re.search(r"can state (\S+)", txt)
    if m:
        health["bus_state"] = m.group(1).upper()

    m = re.search(r"bitrate (\d+)", txt)
    if m:
        health["bitrate"] = int(m.group(1))

    # "berr-counter tx 0 rx 0"
    m = re.search(r"berr-counter tx (\d+) rx (\d+)", txt)
    if m:
        health["tx_error_count"] = int(m.group(1))
        health["rx_error_count"] = int(m.group(2))

    # restart count and error counters from the stats block
    m = re.search(r"re-started bus-errors arbit-lost.*?\n\s*(\d+)\s+(\d+)\s+(\d+)", txt, re.S)
    if m:
        health["restarts"] = int(m.group(1))
        health["bus_errors"] = int(m.group(2))

    return health


class CANLink:
    """
    Owns the bus object and its health. Provides open/close/reopen and a
    health poll. Does not decode -- that stays in CANinterface.
    """

    def __init__(
        self,
        state: SystemState,
        channel: str = "can0",
        bitrate: int = 250000,
        backend: Optional[str] = None,
    ) -> None:
        self.state = state
        self.requested_backend = backend
        self.interface, self.channel = backend_for_platform(backend, channel)
        self.bitrate = bitrate
        self.bus: Optional[can.BusABC] = None
        self._backoff_idx = 0
        self._opened_at = 0.0
        self._last_health_poll = 0.0
        self._prev_rx_err = 0
        self._prev_tx_err = 0

        state.backend = self.interface
        state.channel = self.channel
        state.bitrate = bitrate

    # -- lifecycle ---------------------------------------------------------

    def open(self) -> bool:
        """
        Attempt to open the bus. Returns success. Never raises, never exits.
        """
        # Pre-check: on Linux we can tell "adapter not plugged in" from
        # "adapter present but misconfigured" before python-can obscures it.
        if self.interface == "socketcan" and not socketcan_iface_exists(self.channel):
            self._fail_open(
                f"interface {self.channel} not present -- adapter unplugged "
                f"or 'ip link set {self.channel} up type can bitrate {self.bitrate}' not run"
            )
            return False

        try:
            kwargs = dict(
                channel=self.channel,
                interface=self.interface,
                receive_own_messages=True,
            )
            # socketcan takes its bitrate from the netdev config, not from
            # python-can. Passing it is harmless but meaningless; for pcan
            # and virtual it matters.
            if self.interface != "socketcan":
                kwargs["bitrate"] = self.bitrate

            self.bus = can.interface.Bus(**kwargs)
        except can.CanInterfaceNotImplementedError as e:
            self._fail_open(f"backend '{self.interface}' unavailable: {e}")
            return False
        except can.CanInitializationError as e:
            self._fail_open(f"init failed: {e}")
            return False
        except (OSError, PermissionError) as e:
            self._fail_open(f"cannot open {self.channel}: {e}")
            return False
        except Exception as e:  # unknown driver-level failure
            self._fail_open(f"unexpected error opening {self.channel}: {e}")
            return False

        self._opened_at = time.time()
        self._backoff_idx = 0
        with self.state._lock:
            self.state.link_up = True
            self.state.link_detail = f"{self.interface}:{self.channel}"
            self.state.reconnect_attempts = 0
        self.state.clear_fault(F_LINK_DOWN)
        self.state.log_event("LINK", f"opened {self.interface}:{self.channel} @ {self.bitrate}")
        logger.info("CAN bus open: %s:%s @ %d", self.interface, self.channel, self.bitrate)
        return True

    def _fail_open(self, detail: str) -> None:
        self.bus = None
        with self.state._lock:
            self.state.link_up = False
            self.state.link_detail = detail
            self.state.reconnect_attempts += 1
            delay = BACKOFF[min(self._backoff_idx, len(BACKOFF) - 1)]
            self.state.next_reconnect_at = time.time() + delay
        self._backoff_idx += 1
        self.state.raise_fault(F_LINK_DOWN, detail)
        logger.warning("CAN open failed: %s", detail)

    def close(self) -> None:
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
        with self.state._lock:
            self.state.link_up = False

    def next_backoff(self) -> float:
        return BACKOFF[min(self._backoff_idx, len(BACKOFF) - 1)]

    # -- send --------------------------------------------------------------

    def send(self, msg: can.Message) -> bool:
        """Send with typed error handling. Returns success."""
        if self.bus is None:
            self.state.raise_fault(F_TX_FAIL, "bus not open")
            return False
        try:
            self.bus.send(msg, timeout=0.1)
            with self.state._lock:
                self.state.tx_total += 1
            self.state.clear_fault(F_TX_FAIL)
            return True
        except can.CanOperationError as e:
            # Typical when the controller is bus-off or the tx queue is full.
            self.state.raise_fault(F_TX_FAIL, str(e))
            return False
        except can.CanError as e:
            self.state.raise_fault(F_TX_FAIL, str(e))
            return False
        except OSError as e:
            # ENETDOWN when the adapter is yanked mid-run.
            self.state.raise_fault(F_TX_FAIL, f"{e}")
            self.state.raise_fault(F_LINK_DOWN, "adapter disappeared during send")
            with self.state._lock:
                self.state.link_up = False
            return False

    # -- health ------------------------------------------------------------

    def poll_health(self, now: Optional[float] = None) -> None:
        """
        Called on a timer. Evaluates traffic presence, kernel error counters,
        and per-source staleness, and raises/clears faults accordingly.
        """
        now = now or time.time()
        st = self.state

        # --- kernel counters (Linux socketcan only) ---
        if self.interface == "socketcan" and now - self._last_health_poll > 1.0:
            self._last_health_poll = now
            h = read_socketcan_health(self.channel)
            if h:
                with st._lock:
                    st.bus_state = h.get("bus_state", st.bus_state)
                    st.rx_error_count = h.get("rx_error_count", st.rx_error_count)
                    st.tx_error_count = h.get("tx_error_count", st.tx_error_count)
                    if "bitrate" in h:
                        st.bitrate = h["bitrate"]

                if h.get("bus_state") == "BUS-OFF":
                    st.raise_fault(
                        F_BUS_OFF,
                        "controller bus-off -- check termination and bitrate",
                    )
                else:
                    st.clear_fault(F_BUS_OFF)

                rx_err = h.get("rx_error_count", 0)
                tx_err = h.get("tx_error_count", 0)
                climbing = (rx_err > self._prev_rx_err) or (tx_err > self._prev_tx_err)
                self._prev_rx_err, self._prev_tx_err = rx_err, tx_err

                if climbing and (rx_err + tx_err) > 16:
                    st.raise_fault(
                        F_BUS_ERRORS,
                        f"err counters rising tx={tx_err} rx={rx_err} -- "
                        f"likely bitrate mismatch (configured {st.bitrate})",
                    )
                elif rx_err == 0 and tx_err == 0:
                    st.clear_fault(F_BUS_ERRORS)

        # --- traffic presence ---
        if st.link_up:
            since_open = now - self._opened_at
            last_any = st.last_rx_any
            silent_for = (now - last_any) if last_any else since_open

            if silent_for > NO_TRAFFIC_TIMEOUT:
                if st.rx_total == 0:
                    detail = (
                        f"no frames in {silent_for:.0f}s since open -- "
                        f"check bitrate (set {st.bitrate}) and termination"
                    )
                else:
                    detail = f"traffic stopped {silent_for:.0f}s ago"
                st.raise_fault(F_NO_TRAFFIC, detail)
            else:
                st.clear_fault(F_NO_TRAFFIC)

        # --- per-source staleness ---
        stale_names = []
        with st._lock:
            for src in st.sources.values():
                if src.ever_seen and src.age(now) > src.stale_after:
                    stale_names.append(f"{src.name}({src.age(now):.0f}s)")
        if stale_names:
            st.raise_fault(F_STALE, ", ".join(sorted(stale_names)))
        else:
            st.clear_fault(F_STALE)
