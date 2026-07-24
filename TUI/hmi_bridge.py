"""
hmi_bridge.py -- Translates decoded CAN dicts into SystemState updates.

can_interface.process_message returns loosely-shaped dicts. Rather than
rewrite that decoder, this module maps its output keys onto named signals.
Keeping the mapping in one table means that when a PGN decoder changes, or
when the Kalman filter starts publishing fused heading, there is exactly one
place to edit.

The mapping is deliberately tolerant: unknown keys are ignored rather than
raising, because a decoder that grows a new field should not crash the HMI.
"""

from __future__ import annotations

import logging
import math
from typing import Optional

from hmi_heading import HeadingMonitor
from hmi_state import SystemState

logger = logging.getLogger("bridge")

# Minimum SOG (mph) for COG to be a usable heading. Below this the boat has no
# steerage way, COG degrades to GPS noise, and -- on this hull -- there is no
# trustworthy heading at all, so the autopilot centers and waits for motion.
# This is the single knob that governs the "wait for motion" behaviour.
COG_MIN_SPEED_MPH = 1.6

# decoded-dict key -> signal name
KEY_TO_SIGNAL = {
    "steering_angle": "steering_angle",
    "steering_goal": "steering_goal",
    "rudder_angle": "rudder_angle",
    "rudder_value": "rudder_value",
    "rpm": "rpm",
    "lat": "lat",
    "lon": "lon",
    "SOG": "sog",
    "COG": "cog",
    "compass_heading": "compass_heading",
    "compass": "compass_heading",
    "pitch": "pitch",
    "roll": "roll",
    "compass_offset": "compass_offset",
}

# Watched IDs and how long each may be silent before it is called stale.
# Values are ~3x nominal publish interval.
WATCHED = {
    0x18F01D21: ("steering", 1.5),
    0x19F10D13: ("rudder", 2.0),
    0x09F8021C: ("gps_cog_sog", 1.5),
    0x09F8011C: ("gps_position", 1.0),
    0x18FEE81C: ("vehicle_dir", 3.5),
    0x0CF00400: ("engine", 1.0),
    0x09F112F8: ("vessel_heading", 1.0),
    0x18FF50E0: ("autopilot_status", 1.0),
}


class Bridge:
    def __init__(self, state: SystemState, monitor: HeadingMonitor) -> None:
        self.state = state
        self.monitor = monitor
        for can_id, (name, stale) in WATCHED.items():
            state.register_source(can_id, name, stale)

    def on_decoded(self, data: dict) -> None:
        """
        Callback registered with CANinterface.add_listener.

        Wrapped in a broad try because this runs on the CAN read path; a
        malformed frame producing an unexpected dict shape must not take
        down the reader loop. The exception is logged with its traceback so
        it is still visible rather than silently eaten.
        """
        try:
            self._on_decoded(data)
        except Exception:
            logger.exception("bridge failed on payload: %r", data)
            self.state.log_event("WARN", "decode bridge error (see log)")

    def _on_decoded(self, data: dict) -> None:
        st = self.state

        # Timeout notifications from the old interface come through here too.
        if data.get("timeout"):
            return

        for key, value in data.items():
            sig = KEY_TO_SIGNAL.get(key)
            if sig is not None and value is not None:
                st.set_signal(sig, value)

        # Heading quality inputs
        if "compass_heading" in data and data["compass_heading"] is not None:
            self.monitor.add_heading(float(data["compass_heading"]))
        elif "compass" in data and data["compass"] is not None:
            self.monitor.add_heading(float(data["compass"]))

        if data.get("pitch") is not None:
            self.monitor.add_pitch(float(data["pitch"]))
        if data.get("roll") is not None:
            self.monitor.add_roll(float(data["roll"]))

        # Fused heading: until the Kalman filter is wired in, fall back to
        # compass when it is fresh, else COG. Replace this block with the
        # filter output and the rest of the HMI needs no changes.
        if "fused_heading" in data and data["fused_heading"] is not None:
            st.set_signal("fused_heading", float(data["fused_heading"]))
        if "heading_sigma" in data and data["heading_sigma"] is not None:
            self.monitor.set_filter_sigma(float(data["heading_sigma"]))
        if "yaw_rate" in data and data["yaw_rate"] is not None:
            st.set_signal("yaw_rate", float(data["yaw_rate"]))

        # Control state echoed from the autopilot status frame
        if "autopilot_engaged" in data:
            st.autopilot_engaged = bool(data["autopilot_engaged"])
        if "servo_enabled" in data:
            st.servo_enabled = bool(data["servo_enabled"])
        if "left_turn" in data:
            st.left_turn = bool(data["left_turn"])
        if "right_turn" in data:
            st.right_turn = bool(data["right_turn"])
        if "heading_error" in data and data["heading_error"] is not None:
            st.heading_error = float(data["heading_error"])

        # COG accept/reject accounting, a proxy until the filter reports it
        if "COG" in data:
            sog = st.signal_value("sog")
            if sog is not None and sog < 1.6:
                st.cog_rejected += 1
            else:
                st.cog_accepted += 1

    def derive_fused(self) -> None:
        """
        Periodic fallback fusion. Called on the health tick.

        COG-primary, by deliberate design for this hull. It is a V-drive
        ski/wakeboard boat with a fixed prop and a small rudder: it only steers
        with forward way on, and its magnetic compass is unreliable (in the
        captured runs the compass never reads above 264 deg while COG spans the
        full circle, i.e. the magnetometer is stuck or badly calibrated). So
        the trustworthy heading is course-over-ground whenever the boat is
        moving, and the compass is only a low-confidence backup for the brief
        moments COG drops out at speed.

        Below the COG threshold there is no usable heading at all -- COG is
        GPS-noise at rest and the compass cannot be trusted -- so fused goes
        None and the autopilot centers the rudder and waits for motion. That
        "wait for motion" behaviour is the point: with no steerage way there is
        nothing useful the rudder can do anyway.

        The Kalman filter supersedes this; it should keep the same COG-primary
        weighting for this hull.
        """
        st = self.state
        comp = st.get_signal("compass_heading")
        cog = st.get_signal("cog")
        sog = st.signal_value("sog") or 0.0

        cog_ok = cog is not None and cog.is_valid() and sog >= COG_MIN_SPEED_MPH
        comp_ok = comp is not None and comp.is_valid()

        if cog_ok:
            st.set_signal("fused_heading", cog.value, cog.last_rx)
            st.heading_source = "COG"
            st.cog_lock = True
        elif comp_ok and sog >= COG_MIN_SPEED_MPH:
            # Moving, but COG dropped out. Compass is weak on this boat, so
            # this is a bridge for a few frames, not a heading to trust for
            # long. Marked distinctly so the autopilot can hold rather than
            # chase it.
            st.set_signal("fused_heading", comp.value, comp.last_rx)
            st.heading_source = "COMPASS"
            st.cog_lock = False
        else:
            # No steerage way, or no usable heading. Clear fused so the
            # autopilot centers and waits.
            st.set_signal("fused_heading", None)
            st.heading_source = "NONE"
            st.cog_lock = False
