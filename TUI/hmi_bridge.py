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


def _now() -> float:
    import time
    return time.time()

# Minimum SOG (mph) for COG to be a usable heading. Below this the boat has no
# steerage way, COG degrades to GPS noise, and -- on this hull -- there is no
# trustworthy heading at all, so the autopilot centers and waits for motion.
# This is the single knob that governs the "wait for motion" behaviour.
COG_MIN_SPEED_MPH = 1.6

# SOG (mph) at and above which COG becomes the PRIMARY heading, over magnetic
# heading sources. This is the fishing requirement: with way on, hold course
# over ground so the boat tracks straight (lines trail straight off the transom)
# even in a crosswind/current where the boat crabs and magnetic heading would
# let it drift off track. Below this, magnetic heading is primary (COG is too
# noisy at low speed -- measured: ~12% of COG samples jump >45 deg/s even at
# planing speed, so COG always passes through the yaw-rate limiter too).
# Set above COG_MIN so there is a clear band: below COG_MIN nothing, between
# COG_MIN and this magnetic heading, above this COG-primary.
#
# 4 mph, not 3: the boat at idle often exceeds 3 mph, and a threshold there
# would flip between magnetic and COG repeatedly during normal low-speed
# operation. 4 mph sits above idle so the source stays stable at trolling
# speeds (where the magnetic source is proven and fine) and only hands off to
# COG once genuinely underway. Also matches the Garmin 24xd's own 4 mph
# heading-alignment threshold, so the two agree.
COG_PRIMARY_SPEED_MPH = 4.0

# Hysteresis: once COG-primary engages at COG_PRIMARY_SPEED_MPH, it does not
# drop back to magnetic until speed falls below this lower bound. The dead-band
# between the two means a boat hovering near the threshold (idle, trolling)
# does not flip sources every time SOG wobbles across a single line -- which is
# the "I don't want a lot of transitions" requirement. 3 mph gives a 1 mph
# band; widen it by lowering this if transitions still occur.
COG_PRIMARY_DROP_MPH = 3.0

# The EKF's heading is preferred over the priority chain only when its sigma is
# below this. Above it, the EKF is not confident enough (e.g. compass-only on a
# parked boat, no COG to pin it) and derive_fused falls through to the compass/
# COG priority chain -- which is what lets the autopilot engage on a solid
# compass in the parking lot. Aligned with the engage threshold so "EKF used"
# implies "engageable".
EKF_TRUST_SIGMA = 6.0

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
    # NOTE: intentionally NO "compass" -> "compass_heading" alias. The GPS
    # vehicle-direction decode used to return {"compass": ...} (a stationary-GPS
    # course), which clobbered the real compass heading and made the display
    # bounce. Vehicle direction now has its own key below.
    "gps_vehicle_dir": "gps_vehicle_dir",
    "pitch": "pitch",
    "roll": "roll",
    "compass_offset": "compass_offset",
    # Heading node (sensor_node/PROTOCOL.md)
    "node_heading": "node_heading",
    "node_heading_sigma": "node_heading_sigma",
    "node_yaw_rate": "node_yaw_rate",
    "node_pitch": "pitch",   # the node finally gives us real pitch/roll
    "node_roll": "roll",
    "node_mag_cal": "node_mag_cal",
    "node_fix_type": "node_fix_type",
    "node_num_sats": "node_num_sats",
}

# Watched IDs and how long each may be silent before it is called stale.
# Values are ~3x nominal publish interval.
WATCHED = {
    0x18F01D21: ("steering", 1.5),
    0x19F10D13: ("rudder", 2.0),
    0x09F8021C: ("gps_cog_sog", 1.5),
    0x09F8011C: ("gps_position", 1.0),
    0x18FEE81C: ("gps_vehicle_dir", 3.5),
    0x0CF00400: ("engine", 1.0),
    0x09F112F8: ("compass_F8", 1.0),
    0x18FF50E0: ("autopilot_status", 1.0),
    # Heading node: 10 Hz heading + attitude, 1 Hz health. Allow ~5x the
    # nominal interval before calling them stale.
    0x18FF80E1: ("heading_node", 0.5),
    0x18FF81E1: ("node_attitude", 0.5),
    0x18FF82E1: ("node_health", 5.0),
}


class Bridge:
    def __init__(self, state: SystemState, monitor: HeadingMonitor,
                 registry=None) -> None:
        self.state = state
        self.monitor = monitor
        # NAME-bound heading source registry. Defaults to the profiles in
        # heading_sources.py. The calibration tool and the main TUI can toggle
        # sources on this object at runtime.
        from heading_sources import HeadingSourceRegistry
        self.heading_registry = registry or HeadingSourceRegistry()
        # Latest heading per source address, so derive_fused can pick the
        # registry's preferred source rather than whatever arrived last.
        # {sa: (value, timestamp)}
        self._heading_by_sa: dict = {}
        # Yaw-rate limiter for the fused output: discards any heading that
        # implies an impossible turn (GPS jitter, a glitching sensor), so the
        # controller does not chase erroneous data. Tuned to measured real
        # turns (~41 deg/s max) -> 45 deg/s ceiling. See heading_fusion.py.
        from heading_fusion import YawRateLimiter
        self._yaw_limiter = YawRateLimiter()
        # Optional heading EKF that fuses compass heading, the compass yaw-rate
        # derivative, COG, and an external gyro (if present) into a smooth
        # estimate. It runs ALONGSIDE the priority chain: the EKF publishes
        # "ekf_heading" as another candidate, and a registry profile can prefer
        # it, but if it is disabled or diverges the priority chain still works.
        # Off by default -- enable via use_ekf=True or at runtime. 2-state
        # unless an independent gyro is available (then 3-state pays off).
        self._ekf = None
        self._ekf_last_t = None
        self._ekf_last_meas_t = None  # when the EKF last had a real reference
        self._compass_prev = None  # (value, t) for the compass derivative
        self.use_ekf = False
        # COG-primary hysteresis state: True once engaged at
        # COG_PRIMARY_SPEED_MPH, cleared only below COG_PRIMARY_DROP_MPH.
        self._cog_primary_active = False
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

        # Heading quality inputs. Route per-source heading (tagged with its
        # source address) into per-SA storage so derive_fused can honor the
        # NAME-bound source priority. The wall compass and the 24xd both send
        # heading; without this, whichever arrived last would win.
        if data.get("heading_sa") is not None and data.get("heading_value") is not None:
            import time as _t
            sa = int(data["heading_sa"])
            self._heading_by_sa[sa] = (float(data["heading_value"]), _t.time())
            # Only feed the monitor from the registry's preferred source, so
            # the sigma/noise stats reflect the source we actually steer by.
            best = self.heading_registry.best_source()
            if best is None or best.source_address == sa:
                self.monitor.add_heading(float(data["heading_value"]))
        elif "compass_heading" in data and data["compass_heading"] is not None:
            self.monitor.add_heading(float(data["compass_heading"]))
        # (No "compass" fallback: that key was the GPS vehicle-direction course,
        # not a heading -- feeding it here polluted the heading noise stats.)

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

    def step_ekf(self) -> None:
        """
        Advance the heading EKF one tick and publish its estimate as the
        "ekf_heading" signal. Called from the health tick, alongside
        derive_fused. Safe to call whether or not use_ekf is set -- it no-ops
        when disabled.

        The EKF turns the operator's insight (a biased compass still has an
        accurate derivative) into a filtered heading: it feeds the compass YAW
        RATE (from successive compass headings) as a low-noise rate measurement
        and COG as the unbiased absolute-track reference, while largely ignoring
        the compass's biased absolute value. If the node or 24xd provides an
        independent gyro yaw rate, a 3-state filter also learns the compass
        scale error online (confirmed to help only when a gyro is present).
        """
        if not self.use_ekf:
            return
        import time as _t
        from heading_ekf import HeadingEKF2, HeadingEKF3
        from heading_fusion import YAW_RATE_MAX_DPS, _angdiff

        st = self.state
        now = _t.time()

        # Read heading references with FRESHNESS, not just last value.
        # signal_value returns the last value even when stale, so on a CAN drop
        # the compass/COG would appear frozen-but-present and keep driving the
        # filter. Use the Signal's validity so a stale reference is treated as
        # absent -- the filter then coasts (predict-only, sigma grows) and stops
        # publishing rather than locking onto a frozen heading.
        def _fresh(name):
            sig = st.get_signal(name)
            if sig is None or sig.value is None or not sig.is_valid(now):
                return None
            return sig.value

        cog = _fresh("cog")
        comp = _fresh("compass_heading")
        node_yaw = _fresh("node_yaw_rate")
        sog = st.signal_value("sog") or 0.0

        # Do not run the EKF on nothing. If there is no absolute heading
        # reference available at all (no compass, no COG) -- e.g. CAN is down --
        # the filter has nothing to seed from or correct against, so leave it
        # uninitialized and publish no ekf_heading. This prevents a blind filter
        # from feeding a fabricated heading (it used to seed to 0.0 and publish
        # north) into the control loop while CAN is down. When a real reference
        # returns, the filter builds fresh from it.
        have_reference = (comp is not None) or (cog is not None)
        if self._ekf is None and not have_reference:
            # ensure any stale published values are cleared while blind
            st.set_signal("ekf_heading", None)
            st.set_signal("ekf_sigma", None)
            self._ekf_last_t = now
            return

        # Lazily construct. 3-state only if an independent gyro is available --
        # otherwise the bias is unobservable and the 2-state is better.
        if self._ekf is None:
            seed = cog if cog is not None else comp
            self._ekf = (HeadingEKF3(heading0=seed) if node_yaw is not None
                         else HeadingEKF2(heading0=seed))
            self._ekf_last_t = now

        dt = now - (self._ekf_last_t or now)
        self._ekf_last_t = now
        if dt <= 0:
            dt = 0.1
        self._ekf.predict(dt)

        # Track when we last had a real measurement. If the reference has been
        # gone too long (CAN down mid-run), stop publishing: the coasted
        # estimate is not trustworthy and should not appear confident on the
        # display or feed the loop. derive_fused already ignores sigma > 15, but
        # clearing the signal is cleaner and matches "no signal" on the TUI.
        if have_reference:
            self._ekf_last_meas_t = now
        coasted = now - getattr(self, "_ekf_last_meas_t", now)
        if coasted > 3.0:
            st.set_signal("ekf_heading", None)
            st.set_signal("ekf_sigma", self._ekf.sigma, now)  # keep sigma for the TUI
            st.set_signal("ekf_yaw_rate", None)
            return

        # Compass yaw-rate measurement (the operator's idea): derivative of
        # successive compass headings, gated so an electrical spike does not
        # corrupt the rate.
        if comp is not None:
            if self._compass_prev is not None:
                pv, pt = self._compass_prev
                cdt = now - pt
                if cdt > 0:
                    rate = _angdiff(comp, pv) / cdt
                    if abs(rate) <= YAW_RATE_MAX_DPS:
                        self._ekf.update_yaw_rate(rate, R=1.0)
            self._compass_prev = (comp, now)

            # Compass heading as an ABSOLUTE reference. The compass is biased
            # (declination/iron), so how much we trust it for absolute heading
            # depends on whether COG -- the unbiased track reference -- is also
            # available:
            #   * COG present (moving): trust the compass absolute only weakly
            #     (R=300, ~17 deg) so COG dominates the absolute heading and the
            #     compass bias is ignored, while its DERIVATIVE still smooths
            #     yaw. This preserves the fishing behaviour (track, not heading).
            #   * COG absent (parked): the compass is the ONLY absolute
            #     reference, so trust it enough (R=25, ~5 deg) to pin the filter
            #     and let sigma converge -- otherwise a parked boat never gets a
            #     low-enough sigma to engage, despite a rock-solid compass.
            cog_present = cog is not None and sog >= COG_MIN_SPEED_MPH
            comp_R = 2000.0 if cog_present else 25.0
            self._ekf.update_heading(comp, R=comp_R)

        # Independent gyro (node/24xd), unbiased: separates the compass scale
        # bias on the 3-state.
        if node_yaw is not None and hasattr(self._ekf, "update_yaw_rate_gyro"):
            self._ekf.update_yaw_rate_gyro(float(node_yaw), R=0.3)

        # COG as the unbiased track reference at fishing speed, jitter-gated.
        if cog is not None and sog >= COG_MIN_SPEED_MPH:
            if abs(_angdiff(cog, self._ekf.heading)) / max(dt, 0.1) <= YAW_RATE_MAX_DPS:
                self._ekf.update_heading(cog, R=8.0)

        st.set_signal("ekf_heading", self._ekf.heading, now)
        st.set_signal("ekf_yaw_rate", self._ekf.yaw_rate, now)
        st.set_signal("ekf_sigma", self._ekf.sigma, now)

    def derive_fused(self) -> None:
        """
        Periodic fallback fusion. Called on the health tick.

        Priority order:
          1. heading node fused output (gyro + GPS + calibrated mag) -- valid
             at any speed, so it lifts the wait-for-motion restriction
          2. COG when moving (COG-primary, for the boat's own GPS)
          3. compass as a brief bridge if COG drops at speed (NOT trusted)
          4. nothing usable -> None, center rudder and wait

        Levels 2-4 are the COG-primary logic that applies when the heading node
        is absent or invalid. The node is preferred because it fuses a rate
        gyro and a calibrated magnetometer with GPS course, giving a heading at
        rest that COG alone cannot. If the node dies, the HMI degrades cleanly
        to the COG-primary behavior below.

        The Kalman filter, if added, would live on the node and feed level 1.
        """
        st = self.state
        node = st.get_signal("node_heading")
        comp = st.get_signal("compass_heading")
        cog = st.get_signal("cog")
        sog = st.signal_value("sog") or 0.0

        # Priority 1: the heading node's fused output. It blends gyro + GPS
        # course + a calibrated magnetometer, so when it is valid it gives a
        # trustworthy heading at ANY speed -- including at rest, which COG
        # cannot do. This is the whole reason the node exists: it lifts the
        # "wait for motion" restriction that the stuck compass forced on us.
        node_ok = node is not None and node.is_valid()
        if node_ok:
            st.set_signal("fused_heading", node.value, node.last_rx)
            st.heading_source = "NODE"
            # A valid node heading is a lock regardless of boat speed, so the
            # autopilot may steer even at rest (e.g. holding station).
            st.cog_lock = True
            # Feed the node's own sigma to the monitor so the engage gate uses
            # the filter's confidence rather than raw compass scatter.
            ns = st.signal_value("node_heading_sigma")
            if ns is not None:
                self.monitor.set_filter_sigma(ns)
            return

        # Priority 1.5: the heading EKF, but ONLY when it is genuinely
        # confident. The EKF runs alongside the priority chain, never replaces
        # it. Its sigma must be low enough to mean it has real absolute fixes --
        # not merely coasting on the compass derivative. A compass-only EKF on a
        # parked boat sits around 10-12 deg sigma (no COG to pin it), which is
        # NOT good enough to prefer over the rock-solid compass itself. So the
        # gate is EKF_TRUST_SIGMA (6 deg), well below the old 15: above that we
        # fall straight through to the priority chain, which uses the compass
        # directly and lets the autopilot engage. This is the "if sigma is too
        # high, revert to the priority sources" behaviour -- the compass is
        # good and solid, so the system should engage on it in the parking lot.
        if self.use_ekf:
            ekf_h = st.get_signal("ekf_heading")
            ekf_sig = st.signal_value("ekf_sigma")
            if (ekf_h is not None and ekf_h.is_valid()
                    and ekf_sig is not None and ekf_sig < EKF_TRUST_SIGMA):
                st.set_signal("fused_heading", ekf_h.value, ekf_h.last_rx)
                st.heading_source = "EKF"
                st.cog_lock = True
                self.monitor.set_filter_sigma(ekf_sig)
                return
            # EKF not confident enough -> fall through to the priority chain
            # below (compass/COG), which is exactly what we want on a parked
            # boat with a good compass.

        cog_ok = cog is not None and cog.is_valid() and sog >= COG_MIN_SPEED_MPH
        comp_ok = comp is not None and comp.is_valid()

        # A helper to commit a fused heading through the yaw-rate limiter. Any
        # candidate that implies an impossible turn (GPS jitter, glitch) is
        # rejected, and we HOLD the last fused heading instead of chasing the
        # bad value. This is the "don't chase erroneous data" guard, applied
        # uniformly to every source.
        def commit(value, when, source_label, lock):
            # The yaw-rate limit guards against erroneous jumps WITHIN a
            # source's stream. A deliberate source switch (e.g. COG becomes
            # available and takes over from the compass) is not a glitch, so
            # when the source changes we re-baseline the limiter instead of
            # rejecting the handoff. Within the same source, the limit applies.
            same_source = (source_label == getattr(self, "_last_fused_source", None))
            if not same_source:
                self._yaw_limiter.accept(value, when)
                st.set_signal("fused_heading", value, when)
                st.heading_source = source_label
                st.cog_lock = lock
                self._last_fused_source = source_label
                return True
            if self._yaw_limiter.check(value, when):
                self._yaw_limiter.accept(value, when)
                st.set_signal("fused_heading", value, when)
                st.heading_source = source_label
                st.cog_lock = lock
                self._last_fused_source = source_label
                return True
            # Rejected as an impossible jump. Keep the previous fused heading.
            self._yaw_limiter.reject()
            logger.debug(
                "yaw-rate reject: %s -> %.1f from %.1f",
                source_label, value,
                self._yaw_limiter.last_value if self._yaw_limiter.last_value is not None else -1,
            )
            return False

        # Gather the currently-available heading candidates so we can choose
        # by speed regime and stability.
        now = _now()
        registry_heading = None  # (value, ts, label)
        for src in self.heading_registry.enabled_sources_by_priority():
            if src.source_address is None:
                continue
            entry = self._heading_by_sa.get(src.source_address)
            if entry is None:
                continue
            value, ts = entry
            if now - ts <= 1.0:
                registry_heading = (value, ts, src.label.upper().replace(" ", "_"))
                break

        # ---- Speed-based primary selection -------------------------------
        # FISHING REQUIREMENT: at and above COG_PRIMARY_SPEED_MPH, hold COURSE
        # OVER GROUND so the boat tracks straight in wind/current. COG is
        # primary here, above magnetic heading. Below that speed (but with some
        # way on), magnetic heading is primary because low-speed COG is noise.
        #
        # Hysteresis: engage COG-primary at COG_PRIMARY_SPEED_MPH, but do not
        # drop back to magnetic until below COG_PRIMARY_DROP_MPH. The dead-band
        # stops source chatter when the boat hovers near the threshold (idle,
        # trolling).
        if self._cog_primary_active:
            if sog < COG_PRIMARY_DROP_MPH:
                self._cog_primary_active = False
        else:
            if sog >= COG_PRIMARY_SPEED_MPH:
                self._cog_primary_active = True

        if cog_ok and self._cog_primary_active:
            if commit(cog.value, cog.last_rx, "COG", True):
                return
            # COG rejected as a jump: fall back to a magnetic source near the
            # last good heading rather than to raw COG.
            if registry_heading is not None:
                v, ts, label = registry_heading
                if commit(v, ts, label, True):
                    return
            # nothing plausible this tick: hold (leave fused as-is)
            return

        # ---- Low/mid speed: magnetic heading primary ---------------------
        if registry_heading is not None:
            v, ts, label = registry_heading
            if commit(v, ts, label, True):
                return
            # magnetic source rejected as a jump; try COG if we have it
            if cog_ok and commit(cog.value, cog.last_rx, "COG", True):
                return
            return

        if cog_ok:
            if commit(cog.value, cog.last_rx, "COG", True):
                return
        elif comp_ok and sog >= COG_MIN_SPEED_MPH:
            # Moving, but COG dropped out. Compass is weak on this boat, so
            # this is a bridge for a few frames, not a heading to trust for
            # long. Marked distinctly so the autopilot can hold rather than
            # chase it.
            if commit(comp.value, comp.last_rx, "COMPASS", False):
                return
        else:
            # No steerage way, or no usable heading. Clear fused so the
            # autopilot centers and waits.
            st.set_signal("fused_heading", None)
            st.heading_source = "NONE"
            st.cog_lock = False
            return

        # Fell through without committing (e.g. the only candidate was a
        # rejected jump). Leave fused_heading holding its previous value.
