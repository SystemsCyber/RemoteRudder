"""
Tests for the heading EKF (2-state and 3-state) and its bridge integration.

The operator's idea: a biased compass still has an accurate derivative, so the
compass yaw rate is a good measurement even when the compass heading is not.
These tests confirm:
  * the filters track a known turn
  * the compass derivative keeps yaw rate accurate despite a heading bias
  * the 3-state learns a yaw-rate bias ONLY when an independent gyro makes it
    observable (otherwise it should not be trusted -- that is why the default
    is the 2-state)
  * the EKF is off by default and runs alongside the priority chain
"""

from __future__ import annotations

import math

import pytest

from heading_ekf import HeadingEKF2, HeadingEKF3, _angdiff, _wrap360


class TestEKF2:
    def test_tracks_constant_turn(self):
        ekf = HeadingEKF2(heading0=90.0)
        h = 90.0
        for _ in range(50):
            h += 10 * 0.1
            ekf.predict(0.1)
            ekf.update_heading(h, R=4.0)
            ekf.update_yaw_rate(10.0, R=1.0)
        assert ekf.heading == pytest.approx(140.0, abs=2.0)
        assert ekf.yaw_rate == pytest.approx(10.0, abs=1.0)

    def test_yaw_rate_accurate_despite_heading_bias(self):
        """
        The operator's core idea. Feed a compass with a +30 deg bias but a
        correct derivative; the filter's yaw rate should still be right.
        """
        ekf = HeadingEKF2(heading0=0.0)
        true_h = 0.0
        for _ in range(50):
            true_h += 6 * 0.1
            ekf.predict(0.1)
            # biased heading, correct rate
            ekf.update_heading(true_h + 30, R=50.0)   # high R: don't trust value
            ekf.update_yaw_rate(6.0, R=0.5)           # low R: trust rate
        assert ekf.yaw_rate == pytest.approx(6.0, abs=1.0)

    def test_sigma_decreases_with_measurements(self):
        ekf = HeadingEKF2(heading0=0.0)
        s0 = ekf.sigma
        for _ in range(20):
            ekf.predict(0.1)
            ekf.update_heading(0.0, R=1.0)
        assert ekf.sigma < s0

    def test_heading_wraps(self):
        ekf = HeadingEKF2(heading0=359.0)
        for _ in range(10):
            ekf.predict(0.1)
            ekf.update_yaw_rate(20.0, R=1.0)
            ekf.update_heading(_wrap360(ekf.heading), R=100.0)
        assert 0.0 <= ekf.heading < 360.0


class TestEKF3:
    def test_learns_bias_with_gyro(self):
        """
        With an independent (unbiased) gyro, the 3-state separates the compass
        scale error into yaw_bias. Compass reads 12/s, gyro says 10/s -> bias 2.
        """
        ekf = HeadingEKF3(heading0=90.0)
        h = 90.0
        for _ in range(150):
            h += 10 * 0.1
            ekf.predict(0.1)
            ekf.update_heading(h, R=4.0)
            ekf.update_yaw_rate_compass(12.0, R=1.0)  # biased rate
            ekf.update_yaw_rate_gyro(10.0, R=0.3)     # truth
        assert ekf.yaw_rate == pytest.approx(10.0, abs=1.0)
        assert ekf.yaw_bias == pytest.approx(2.0, abs=1.0)

    def test_yaw_rate_correct_with_gyro(self):
        ekf = HeadingEKF3(heading0=0.0)
        h = 0.0
        for _ in range(100):
            h += 8 * 0.1
            ekf.predict(0.1)
            ekf.update_heading(h, R=4.0)
            ekf.update_yaw_rate_compass(9.0, R=1.0)
            ekf.update_yaw_rate_gyro(8.0, R=0.3)
        assert ekf.yaw_rate == pytest.approx(8.0, abs=1.0)


class TestAngleHelpers:
    def test_angdiff_wraps(self):
        assert _angdiff(1.0, 359.0) == pytest.approx(2.0)
        assert _angdiff(359.0, 1.0) == pytest.approx(-2.0)

    def test_wrap360(self):
        assert _wrap360(370.0) == pytest.approx(10.0)
        assert _wrap360(-10.0) == pytest.approx(350.0)


class TestBridgeEKFIntegration:
    def test_ekf_off_by_default(self, state, bridge):
        assert bridge.use_ekf is False

    def test_step_ekf_noop_when_disabled(self, state, bridge):
        bridge.step_ekf()
        assert state.signal_value("ekf_heading") is None

    def test_ekf_tracks_cog_not_biased_compass(self, state, bridge):
        """
        Enabled, the EKF should follow COG (unbiased track) and ignore the
        compass's absolute bias -- the whole point.
        """
        import time
        bridge.use_ekf = True
        h = 100.0
        cog = 110.0
        for _ in range(30):
            h += 4 * 0.05
            cog += 4 * 0.05
            state.set_signal("compass_heading", h + 30)  # +30 bias
            state.set_signal("cog", cog)
            state.set_signal("sog", 8.0)
            bridge.step_ekf()
            time.sleep(0.005)
        ekf_h = state.signal_value("ekf_heading")
        assert ekf_h is not None
        # closer to COG than to the biased compass
        assert abs(_angdiff(ekf_h, cog)) < abs(_angdiff(ekf_h, h + 30))

    def test_ekf_feeds_fused_when_confident(self, state, bridge):
        import time
        bridge.use_ekf = True
        for _ in range(30):
            state.set_signal("compass_heading", 150.0)
            state.set_signal("cog", 150.0)
            state.set_signal("sog", 8.0)
            bridge.step_ekf()
            time.sleep(0.005)
        bridge.derive_fused()
        # EKF confident -> it should be the fused source
        assert state.heading_source == "EKF"

    def test_disabled_ekf_falls_through_to_chain(self, state, bridge):
        """With the EKF off, the priority chain still drives fusion."""
        bridge.use_ekf = False
        bridge.heading_registry.note_claim(0x19, 229, 145, 1602535)
        bridge.heading_registry.set_enabled(1602535, True)
        state.set_signal("sog", 8.0)
        state.set_signal("cog", 90.0)
        bridge.derive_fused()
        assert state.heading_source == "COG"
