"""
Tests for the fishing-mode fusion: COG-primary at speed and yaw-rate limiting.

Requirements captured here:
  * FISHING: above COG_PRIMARY_SPEED_MPH, hold course over ground (track), not
    magnetic heading, so the boat tracks straight in a crosswind (lines trail
    straight off the transom). Below that, magnetic heading is primary.
  * Discard heading changes that exceed a realistic yaw rate (measured ~41
    deg/s max real turn -> 45 deg/s limit), so the controller does not chase
    GPS jitter or a glitching sensor.
  * A deliberate source switch is not a glitch -- re-baseline rather than
    reject the handoff.
"""

from __future__ import annotations

import time

import pytest

from heading_fusion import (YAW_RATE_MAX_DPS, YawRateLimiter, SourceStability,
                            choose_stable_near_last, _angdiff)
from hmi_bridge import COG_PRIMARY_SPEED_MPH, COG_MIN_SPEED_MPH


class TestYawRateLimiter:
    def test_accepts_first_value(self):
        lim = YawRateLimiter()
        assert lim.check(100.0, when=0.0) is True

    def test_accepts_plausible_turn(self):
        lim = YawRateLimiter()
        lim.accept(100.0, when=0.0)
        # 20 deg in 1 s = 20 deg/s, well under the limit
        assert lim.check(120.0, when=1.0) is True

    def test_rejects_impossible_jump(self):
        lim = YawRateLimiter()
        lim.accept(90.0, when=0.0)
        # 180 deg in 0.5 s = 360 deg/s, impossible
        assert lim.check(270.0, when=0.5) is False

    def test_boundary_at_limit(self):
        lim = YawRateLimiter()
        lim.accept(0.0, when=0.0)
        # exactly at the limit over 1 s
        assert lim.check(YAW_RATE_MAX_DPS, when=1.0) is True
        # just over
        lim2 = YawRateLimiter()
        lim2.accept(0.0, when=0.0)
        assert lim2.check(YAW_RATE_MAX_DPS + 5, when=1.0) is False

    def test_stale_reference_resets(self):
        lim = YawRateLimiter()
        lim.accept(90.0, when=0.0)
        # 5 s later, too stale to judge -> accept
        assert lim.check(270.0, when=5.0) is True

    def test_consecutive_rejects_rebaseline(self):
        """
        After several rejects in a row the reference is untrustworthy; accept
        the next to re-baseline rather than rejecting forever.
        """
        lim = YawRateLimiter(max_consecutive_rejects=3)
        lim.accept(90.0, when=0.0)
        for i in range(3):
            assert lim.check(270.0, when=0.01 * (i + 1)) is False
            lim.reject()
        # next one re-baselines
        assert lim.check(270.0, when=0.05) is True

    def test_wraparound(self):
        """359 -> 1 is a 2 deg turn, not 358."""
        lim = YawRateLimiter()
        lim.accept(359.0, when=0.0)
        assert lim.check(1.0, when=1.0) is True

    def test_measured_max_real_turn_accepted(self):
        """The measured max real turn (~41 deg/s) must not be clipped."""
        lim = YawRateLimiter()
        lim.accept(100.0, when=0.0)
        assert lim.check(141.0, when=1.0) is True  # 41 deg/s


class TestCogPrimaryTransition:
    def _bridge(self, bridge):
        bridge.heading_registry.note_claim(0x19, 229, 145, 1602535)
        bridge.heading_registry.set_enabled(1602535, True)
        return bridge

    def test_magnetic_primary_below_fishing_speed(self, state, bridge):
        self._bridge(bridge)
        state.set_signal("cog", 90.0)
        state.set_signal("sog", 2.0)  # below COG_PRIMARY_SPEED_MPH
        bridge.on_decoded({"compass_heading": 100.0, "heading_sa": 0x19,
                           "heading_value": 100.0})
        bridge.derive_fused()
        assert state.heading_source == "GPS_24XD"
        assert state.signal_value("fused_heading") == pytest.approx(100.0)

    def test_cog_primary_at_fishing_speed(self, state, bridge):
        """
        The core fishing behavior: at speed, hold COG (track) not heading, so
        the 10 deg crab from a crosswind is ignored and the boat tracks
        straight.
        """
        self._bridge(bridge)
        state.set_signal("cog", 90.0)
        state.set_signal("sog", 8.0)  # above COG_PRIMARY_SPEED_MPH
        bridge.on_decoded({"compass_heading": 100.0, "heading_sa": 0x19,
                           "heading_value": 100.0})
        bridge.derive_fused()
        assert state.heading_source == "COG"
        assert state.signal_value("fused_heading") == pytest.approx(90.0)

    def test_threshold_ordering(self):
        """Fishing threshold must be above the minimum-usable threshold."""
        assert COG_PRIMARY_SPEED_MPH > COG_MIN_SPEED_MPH


class TestYawLimitInFusion:
    def _bridge(self, bridge):
        bridge.heading_registry.note_claim(0x19, 229, 145, 1602535)
        bridge.heading_registry.set_enabled(1602535, True)
        return bridge

    def test_cog_glitch_held(self, state, bridge):
        """An impossible COG jump is discarded; fusion holds the last value."""
        self._bridge(bridge)
        state.set_signal("sog", 8.0)
        state.set_signal("cog", 90.0, when=time.time())
        bridge.derive_fused()
        assert state.signal_value("fused_heading") == pytest.approx(90.0)

        # glitch: 90 -> 270 near-instant
        state.set_signal("cog", 270.0, when=time.time())
        bridge.derive_fused()
        assert state.signal_value("fused_heading") == pytest.approx(90.0)  # held
        assert bridge._yaw_limiter.rejected_count >= 1

    def test_source_switch_not_rejected(self, state, bridge):
        """
        A deliberate source switch (compass -> COG) is a handoff, not a glitch,
        even if the values differ by more than the yaw limit.
        """
        self._bridge(bridge)
        # COG dropped, on compass bridge at 264
        state.set_signal("sog", 8.0)
        state.set_signal("cog", None)
        state.set_signal("compass_heading", 264.0)
        bridge.derive_fused()
        assert state.heading_source == "COMPASS"

        # COG returns at 210 -- 54 deg from the compass value, but this is a
        # source switch, so it must be accepted, not rejected.
        state.set_signal("cog", 210.0)
        bridge.derive_fused()
        assert state.heading_source == "COG"
        assert state.signal_value("fused_heading") == pytest.approx(210.0)


class TestStableFallback:
    def test_picks_steadiest_at_startup(self):
        stab = {"a": SourceStability(), "b": SourceStability()}
        now = time.time()
        # a is steady, b is jumping
        for i in range(5):
            stab["a"].add(150.0, now + i * 0.1)
            stab["b"].add(150.0 + (i % 2) * 40, now + i * 0.1)
        cands = {"a": (150.0, now), "b": (170.0, now)}
        pick = choose_stable_near_last(cands, None, stab, now=now)
        assert pick[0] == "a"

    def test_prefers_near_last(self):
        stab = {"a": SourceStability(), "b": SourceStability()}
        now = time.time()
        stab["a"].add(150.0, now)
        stab["b"].add(200.0, now)
        cands = {"a": (150.0, now), "b": (200.0, now)}
        # last heading is 148 -> a (150) is much closer than b (200)
        pick = choose_stable_near_last(cands, 148.0, stab, now=now)
        assert pick[0] == "a"

    def test_rejects_all_glitches_returns_none(self):
        """If every candidate is an impossible jump from last, return None (hold)."""
        stab = {"a": SourceStability()}
        now = time.time()
        stab["a"].add(300.0, now)
        cands = {"a": (300.0, now)}
        # last heading 100, candidate 300 = 160 deg jump, way over limit
        pick = choose_stable_near_last(cands, 100.0, stab, now=now)
        assert pick is None

    def test_scatter_measures_variability(self):
        s = SourceStability()
        now = time.time()
        for i in range(5):
            s.add(150.0, now + i * 0.1)
        assert s.scatter() < 1.0  # steady
        s2 = SourceStability()
        for i in range(5):
            s2.add(150.0 + (i % 2) * 30, now + i * 0.1)
        assert s2.scatter() > 5.0  # jumpy


class TestCogPrimaryHysteresis:
    """
    The dead-band that stops source chatter when the boat hovers near the
    threshold. Engage COG at COG_PRIMARY_SPEED_MPH, drop back only below
    COG_PRIMARY_DROP_MPH.
    """

    def _bridge(self, bridge):
        bridge.heading_registry.note_claim(0x19, 229, 145, 1602535)
        bridge.heading_registry.set_enabled(1602535, True)
        return bridge

    def _tick(self, state, bridge, sog):
        state.set_signal("cog", 100.0)
        state.set_signal("sog", sog)
        bridge.on_decoded({"compass_heading": 110.0, "heading_sa": 0x19,
                           "heading_value": 110.0})
        bridge.derive_fused()
        return state.heading_source

    def test_engages_at_upper_threshold(self, state, bridge):
        from hmi_bridge import COG_PRIMARY_SPEED_MPH
        self._bridge(bridge)
        assert self._tick(state, bridge, COG_PRIMARY_SPEED_MPH - 0.5) == "GPS_24XD"
        assert self._tick(state, bridge, COG_PRIMARY_SPEED_MPH + 0.1) == "COG"

    def test_holds_cog_in_deadband(self, state, bridge):
        """Between drop and engage thresholds, COG-primary persists once engaged."""
        from hmi_bridge import COG_PRIMARY_SPEED_MPH, COG_PRIMARY_DROP_MPH
        self._bridge(bridge)
        self._tick(state, bridge, COG_PRIMARY_SPEED_MPH + 0.5)  # engage
        mid = (COG_PRIMARY_SPEED_MPH + COG_PRIMARY_DROP_MPH) / 2
        assert self._tick(state, bridge, mid) == "COG"  # still COG in the band

    def test_drops_below_lower_threshold(self, state, bridge):
        from hmi_bridge import COG_PRIMARY_SPEED_MPH, COG_PRIMARY_DROP_MPH
        self._bridge(bridge)
        self._tick(state, bridge, COG_PRIMARY_SPEED_MPH + 0.5)  # engage
        assert self._tick(state, bridge, COG_PRIMARY_DROP_MPH - 0.1) == "GPS_24XD"

    def test_no_chatter_at_idle(self, state, bridge):
        """A boat wobbling in the dead-band produces at most one transition."""
        from hmi_bridge import COG_PRIMARY_SPEED_MPH, COG_PRIMARY_DROP_MPH
        self._bridge(bridge)
        mid = (COG_PRIMARY_SPEED_MPH + COG_PRIMARY_DROP_MPH) / 2
        # start below, one bump above, then wobble in the band
        seq = [COG_PRIMARY_DROP_MPH - 0.5, COG_PRIMARY_SPEED_MPH + 0.3,
               mid, mid + 0.2, mid - 0.2, mid]
        sources = [self._tick(state, bridge, s) for s in seq]
        transitions = sum(1 for i in range(1, len(sources))
                          if sources[i] != sources[i - 1])
        assert transitions == 1  # only the engage; no chatter after

    def test_deadband_ordering(self):
        from hmi_bridge import COG_PRIMARY_SPEED_MPH, COG_PRIMARY_DROP_MPH, COG_MIN_SPEED_MPH
        assert COG_PRIMARY_DROP_MPH < COG_PRIMARY_SPEED_MPH
        assert COG_PRIMARY_DROP_MPH >= COG_MIN_SPEED_MPH
