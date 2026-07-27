"""
Tests for EKF behavior across CAN link transitions.

The operator asked: if the app runs a long time with no CAN (EKF sigma growing,
no signal), will enabling CAN later cause a problem? These tests pin the answer:

  1. With no CAN reference ever, the EKF must NOT publish a heading or feed the
     control loop (it used to seed to 0.0 and publish north on nothing).
  2. When CAN comes up, the EKF builds fresh from the first real reference and
     converges -- no NaN/inf from the (mathematically huge) coasted covariance.
  3. When CAN drops mid-run, the EKF stops publishing after coasting briefly,
     so a stale heading never appears confident or drives the loop.
"""

from __future__ import annotations

import math
import time

import pytest


class TestEkfNoCan:
    def test_no_publish_without_reference(self, state, bridge):
        bridge.use_ekf = True
        for _ in range(50):
            bridge.step_ekf()
        assert state.signal_value("ekf_heading") is None

    def test_does_not_feed_fused_without_reference(self, state, bridge):
        bridge.use_ekf = True
        for _ in range(50):
            bridge.step_ekf()
        bridge.derive_fused()
        assert state.heading_source != "EKF"


class TestEkfCanComesUp:
    def test_builds_and_converges_from_first_reference(self, state, bridge):
        bridge.use_ekf = True
        bridge.heading_registry.note_claim(0xF8, 229, 140, 1039212)
        for _ in range(30):
            state.set_signal("compass_heading", 90.0)
            state.set_signal("cog", 90.0)
            state.set_signal("sog", 8.0)
            bridge.step_ekf()
            time.sleep(0.003)
        h = state.signal_value("ekf_heading")
        sig = state.signal_value("ekf_sigma")
        assert h is not None
        assert abs(((h - 90.0 + 180) % 360) - 180) < 5.0
        assert sig is not None and sig < 5.0

    def test_no_nan_after_long_coast_then_measurement(self):
        """
        The raw filter: 2 hours of predict-only makes sigma huge, but the first
        measurement must recover cleanly with no NaN/inf.
        """
        from heading_ekf import HeadingEKF2
        e = HeadingEKF2(heading0=180.0)
        for _ in range(120 * 60 * 10):  # 2 h at 10 Hz
            e.predict(0.1)
        assert math.isfinite(e.sigma)
        e.predict(0.1)
        e.update_heading(90.0, R=4.0)
        assert math.isfinite(e.heading)
        assert math.isfinite(e.sigma)
        assert abs(((e.heading - 90.0 + 180) % 360) - 180) < 2.0


class TestEkfCanDrops:
    def test_stops_publishing_after_coast(self, state, bridge):
        bridge.use_ekf = True
        bridge.heading_registry.note_claim(0xF8, 229, 140, 1039212)
        # build it
        for _ in range(30):
            state.set_signal("compass_heading", 90.0)
            state.set_signal("cog", 90.0)
            state.set_signal("sog", 8.0)
            bridge.step_ekf()
            time.sleep(0.003)
        assert state.signal_value("ekf_heading") is not None

        # CAN drops: age the references past validity and advance the coast clock
        for name in ("compass_heading", "cog"):
            s = state.get_signal(name)
            s.last_rx = time.time() - 10
        bridge._ekf_last_meas_t = time.time() - 4.0
        bridge.step_ekf()
        assert state.signal_value("ekf_heading") is None

    def test_fused_falls_back_when_ekf_coasts(self, state, bridge):
        bridge.use_ekf = True
        bridge.heading_registry.note_claim(0xF8, 229, 140, 1039212)
        for _ in range(30):
            state.set_signal("compass_heading", 90.0)
            state.set_signal("cog", 90.0)
            state.set_signal("sog", 8.0)
            bridge.step_ekf()
            time.sleep(0.003)
        for name in ("compass_heading", "cog"):
            s = state.get_signal(name)
            s.last_rx = time.time() - 10
        bridge._ekf_last_meas_t = time.time() - 4.0
        bridge.step_ekf()
        bridge.derive_fused()
        assert state.heading_source != "EKF"


class TestParkedEngage:
    """
    The parking-lot scenario: boat parked, compass rock-solid, no COG. The
    system MUST be able to engage -- either the EKF converges on the compass
    (low sigma) or derive_fused falls through to the compass directly. It must
    never sit at a mid sigma that blocks engagement while a perfect compass is
    available.
    """

    def test_ekf_sigma_converges_on_steady_compass(self, state, bridge):
        import time
        from hmi_bridge import EKF_TRUST_SIGMA
        bridge.use_ekf = True
        bridge.heading_registry.note_claim(0xF8, 229, 140, 1039212)
        for _ in range(80):
            state.set_signal("compass_heading", 179.4)  # steady, no COG
            bridge.step_ekf()
            time.sleep(0.003)
        sig = state.signal_value("ekf_sigma")
        assert sig is not None and sig < EKF_TRUST_SIGMA, (
            f"EKF sigma {sig} did not converge on a steady compass -- the boat "
            f"could not engage in the parking lot"
        )

    def test_fused_uses_compass_when_ekf_not_confident(self, state, bridge):
        """
        Even if the EKF sigma were high, derive_fused must fall through to the
        compass (via the registry) so a solid compass still yields a heading.
        """
        import time
        bridge.use_ekf = True
        bridge.heading_registry.note_claim(0xF8, 229, 140, 1039212)
        # feed compass through the decode path so the registry heading-by-sa
        # is populated too
        for _ in range(10):
            bridge.on_decoded({"compass_heading": 179.4, "heading_sa": 0xF8,
                               "heading_value": 179.4})
            state.set_signal("compass_heading", 179.4)
            time.sleep(0.002)
        # force the EKF to look unconfident
        state.set_signal("ekf_sigma", 99.0)
        state.set_signal("ekf_heading", None)
        bridge.derive_fused()
        fused = state.signal_value("fused_heading")
        assert fused is not None, "no fused heading despite a solid compass"
        assert abs(((fused - 179.4 + 180) % 360) - 180) < 5.0
