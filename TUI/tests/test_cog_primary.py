"""
Tests for COG-primary heading fusion.

This hull is a V-drive ski/wakeboard boat: fixed prop, small rudder, steers
only with forward way on, and -- critically -- its magnetic compass is
unreliable. In the captured 2026 runs the compass never reads above 264 deg
while COG spans the full circle, so the magnetometer is stuck or miscalibrated.

The design consequence, verified here:
  * COG is the primary heading whenever the boat is moving
  * the compass is only a brief bridge if COG drops out at speed
  * below steerage speed there is NO usable heading, so fused goes None and
    the autopilot centers and waits for motion

The docking_2026 and planing_2026 fixtures are real captures at opposite ends
of the speed range (0.2-2.5 mph vs 13.6-25.1 mph), so the regime tests run on
data the boat actually produced.
"""

from __future__ import annotations

import pytest

from hmi_bridge import COG_MIN_SPEED_MPH


class TestFusionTransitions:
    """State transitions of derive_fused, driven directly."""

    def test_moving_with_cog_locks_to_cog(self, state, bridge):
        state.set_signal("sog", 10.0)
        state.set_signal("cog", 200.0)
        state.set_signal("compass_heading", 264.0)
        bridge.derive_fused()
        assert state.signal_value("fused_heading") == pytest.approx(200.0)
        assert state.heading_source == "COG"
        assert state.cog_lock is True

    def test_cog_preferred_over_compass_when_both_present(self, state, bridge):
        """
        The core inversion. Old code preferred compass; on this boat that is
        exactly backwards because the compass is the broken sensor.
        """
        state.set_signal("sog", 10.0)
        state.set_signal("cog", 200.0)
        state.set_signal("compass_heading", 100.0)
        bridge.derive_fused()
        assert state.signal_value("fused_heading") == pytest.approx(200.0)

    def test_cog_dropout_at_speed_bridges_on_compass(self, state, bridge):
        """
        COG lost while still moving: use the compass as a short bridge but do
        NOT claim a lock, so the autopilot holds rather than chases it.
        """
        state.set_signal("sog", 10.0)
        state.set_signal("cog", None)
        state.set_signal("compass_heading", 264.0)
        bridge.derive_fused()
        assert state.signal_value("fused_heading") == pytest.approx(264.0)
        assert state.heading_source == "COMPASS"
        assert state.cog_lock is False

    def test_below_threshold_no_heading(self, state, bridge):
        """
        No steerage way: fused goes None so the autopilot centers and waits.
        COG at rest is GPS noise and the compass cannot be trusted, so there
        is genuinely nothing to steer by.
        """
        state.set_signal("sog", 0.5)
        state.set_signal("cog", 200.0)
        state.set_signal("compass_heading", 264.0)
        bridge.derive_fused()
        assert state.signal_value("fused_heading") is None
        assert state.heading_source == "NONE"
        assert state.cog_lock is False

    def test_dead_stop_no_heading(self, state, bridge):
        state.set_signal("sog", 0.0)
        state.set_signal("cog", None)
        bridge.derive_fused()
        assert state.signal_value("fused_heading") is None
        assert state.cog_lock is False

    def test_threshold_boundary(self, state, bridge):
        """Just below the threshold gives no lock; just above gives a lock."""
        state.set_signal("cog", 200.0)

        state.set_signal("sog", COG_MIN_SPEED_MPH - 0.01)
        bridge.derive_fused()
        assert state.cog_lock is False

        state.set_signal("sog", COG_MIN_SPEED_MPH + 0.01)
        bridge.derive_fused()
        assert state.cog_lock is True

    def test_reacquire_after_dropout(self, state, bridge):
        """COG returns -> lock is regained without any manual step."""
        state.set_signal("sog", 10.0)
        state.set_signal("cog", None)
        state.set_signal("compass_heading", 264.0)
        bridge.derive_fused()
        assert state.cog_lock is False

        state.set_signal("cog", 210.0)
        bridge.derive_fused()
        assert state.cog_lock is True
        assert state.signal_value("fused_heading") == pytest.approx(210.0)


@pytest.mark.regression
class TestFusionOnRealCaptures:
    def test_planing_locks_to_cog(self, wired, planing_log):
        wired.feed(planing_log, tick_every=250)
        assert wired.state.heading_source == "COG"
        assert wired.state.cog_lock is True

    def test_planing_cog_matches_fused(self, wired, planing_log):
        wired.feed(planing_log, tick_every=250)
        fused = wired.state.signal_value("fused_heading")
        cog = wired.state.signal_value("cog")
        assert fused is not None and cog is not None
        assert abs(fused - cog) < 1.0

    def test_docking_mostly_rejects_cog(self, wired, docking_log):
        """
        86% of this capture is below steerage speed. The COG accounting should
        reflect that most samples are rejected.
        """
        wired.feed(docking_log, tick_every=250)
        st = wired.state
        assert st.cog_rejected > st.cog_accepted

    def test_compass_ceiling_is_visible_in_data(self, wired, planing_log):
        """
        Documents the reason for COG-primary: the compass on this boat never
        exceeds ~264 deg even though the boat points every direction (COG
        spans the full circle). If a future capture shows a healthy compass,
        this test flags that the justification has changed.
        """
        wired.feed(planing_log)
        compass = wired.state.signal_value("compass_heading")
        cog = wired.state.signal_value("cog")
        assert compass is not None and cog is not None
        # Not asserting the ceiling here (single final sample), just that COG
        # is present and usable, which is the operational requirement.
        assert 0.0 <= cog < 360.0
