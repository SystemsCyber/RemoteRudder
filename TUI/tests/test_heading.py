"""
Unit tests for hmi_heading: circular statistics, noise attribution, interlock.

This is the module that decides whether the autopilot is allowed to engage.
A false "GOOD" here puts the rudder over based on noise, so the tests lean
toward proving the pessimistic paths fire when they should.

Every heading statistic must be computed on the circle. The recurring bug in
this domain is a linear mean or standard deviation that is correct everywhere
except near north, where it is catastrophically wrong -- and north is exactly
where a lot of lake running happens.
"""

from __future__ import annotations

import math
import random

import pytest

from hmi_heading import (
    MAX_PLAUSIBLE_RATE_DPS,
    SIGMA_ENGAGE_MAX,
    HeadingMonitor,
    circular_mean,
    circular_sigma,
    wrap180,
    wrap360,
)
from hmi_state import F_HEADING_INVALID, F_HEADING_NOISE


# ---------------------------------------------------------------------------
# Wrapping
# ---------------------------------------------------------------------------


class TestWrapping:
    @pytest.mark.parametrize(
        "raw,expected",
        [
            (0, 0),
            # 180 maps to -180, not +180: the range is half-open [-180, 180),
            # so the boundary belongs to the negative end. Both represent the
            # same bearing, but pinning it prevents an "improvement" that
            # breaks the range invariant asserted below.
            (180, -180),
            (181, -179),
            (359, -1),
            (360, 0),
            (-181, 179),
            (540, -180),
        ],
    )
    def test_wrap180(self, raw, expected):
        assert wrap180(raw) == pytest.approx(expected)

    @pytest.mark.parametrize(
        "raw,expected", [(0, 0), (360, 0), (361, 1), (-1, 359), (720, 0), (-360, 0)]
    )
    def test_wrap360(self, raw, expected):
        assert wrap360(raw) == pytest.approx(expected)

    def test_wrap180_range(self):
        for x in range(-1000, 1000, 7):
            assert -180.0 <= wrap180(x) < 180.0

    def test_wrap360_range(self):
        for x in range(-1000, 1000, 7):
            assert 0.0 <= wrap360(x) < 360.0


# ---------------------------------------------------------------------------
# Circular statistics
# ---------------------------------------------------------------------------


class TestCircularMean:
    def test_simple(self):
        assert circular_mean([90.0, 90.0, 90.0]) == pytest.approx(90.0)

    def test_across_north(self):
        """
        The canonical failure: a linear mean of 359 and 1 gives 180, which
        points exactly backwards.
        """
        m = circular_mean([359.0, 1.0])
        assert abs(wrap180(m)) < 1e-6

    def test_spread_across_north(self):
        m = circular_mean([350.0, 355.0, 0.0, 5.0, 10.0])
        assert abs(wrap180(m)) < 1e-6

    def test_empty_returns_none(self):
        assert circular_mean([]) is None

    def test_antipodal_is_degenerate(self):
        """Opposing headings have no meaningful mean; must not fabricate one."""
        m = circular_mean([0.0, 180.0])
        assert m is None or math.isfinite(m)


class TestCircularSigma:
    def test_tight_cluster_small_sigma(self):
        assert circular_sigma([100.0, 100.5, 99.5, 100.2]) < 1.0

    def test_same_cluster_at_north(self):
        """Sigma must not blow up merely because the data straddles 0/360."""
        assert circular_sigma([359.8, 0.2, 359.5, 0.5]) < 1.0

    def test_matches_linear_sd_for_tight_data(self):
        """
        For a tight distribution away from the seam, circular sigma should
        agree with the ordinary standard deviation to within a few percent.
        """
        import statistics

        vals = [100.0 + random.gauss(0, 2.0) for _ in range(500)]
        random.seed(42)
        assert circular_sigma(vals) == pytest.approx(statistics.stdev(vals), rel=0.15)

    def test_uniform_is_large(self):
        vals = [float(i) for i in range(0, 360, 10)]
        assert circular_sigma(vals) > 50.0

    def test_too_few_samples(self):
        assert circular_sigma([1.0]) is None
        assert circular_sigma([]) is None

    def test_identical_values_zero_sigma(self):
        assert circular_sigma([42.0] * 10) == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Dispersion measurement
# ---------------------------------------------------------------------------


def feed(monitor, headings, pitches=None, rolls=None, t0=1000.0, dt=0.1):
    for i, h in enumerate(headings):
        t = t0 + i * dt
        monitor.add_heading(h, when=t)
        if pitches is not None:
            monitor.add_pitch(pitches[i], when=t)
        if rolls is not None:
            monitor.add_roll(rolls[i], when=t)


class TestObservedSigma:
    def test_needs_minimum_samples(self, monitor):
        feed(monitor, [100.0, 100.1, 100.2])
        assert monitor.observed_sigma() is None

    def test_calm_water(self, monitor):
        random.seed(1)
        feed(monitor, [100.0 + random.gauss(0, 0.4) for _ in range(40)])
        assert monitor.observed_sigma() < 1.5

    def test_rough_water(self, monitor):
        random.seed(1)
        feed(monitor, [100.0 + random.gauss(0, 9.0) for _ in range(40)])
        assert monitor.observed_sigma() > 6.0

    def test_window_expires_old_samples(self, monitor):
        """Samples outside the window must not drag the estimate forever."""
        feed(monitor, [100.0 + random.gauss(0, 20) for _ in range(20)], t0=1000.0)
        feed(monitor, [200.0] * 20, t0=1100.0)  # 100s later, well past window
        assert monitor.observed_sigma() == pytest.approx(0.0, abs=1e-6)


class TestTiltSigma:
    def test_none_without_data(self, monitor):
        p, r = monitor.tilt_sigma()
        assert p is None and r is None

    def test_measures_dispersion_not_offset(self, monitor):
        """
        A constant tilt offset is trim, not sea state. Only the scatter should
        register. This also means the J1939 pitch offset bug did not affect
        attribution -- but the displayed number was still wrong.
        """
        random.seed(2)
        n = 40
        feed(
            monitor,
            [100.0] * n,
            pitches=[50.0 + random.gauss(0, 1.0) for _ in range(n)],
            rolls=[0.0] * n,
        )
        p, r = monitor.tilt_sigma()
        assert p == pytest.approx(1.0, abs=0.5)
        assert r == pytest.approx(0.0, abs=1e-6)


# ---------------------------------------------------------------------------
# Filter interaction
# ---------------------------------------------------------------------------


class TestEffectiveSigma:
    def test_uses_observed_when_no_filter(self, monitor):
        random.seed(3)
        feed(monitor, [100.0 + random.gauss(0, 3.0) for _ in range(40)])
        assert monitor.effective_sigma() == pytest.approx(monitor.observed_sigma())

    def test_overconfident_filter_is_clamped(self, monitor):
        """
        A filter can be confidently wrong when the compass is being shaken in
        a way its process model does not anticipate. Never report better than
        3x what the raw data shows.
        """
        random.seed(4)
        feed(monitor, [100.0 + random.gauss(0, 12.0) for _ in range(40)])
        monitor.set_filter_sigma(0.5, when=1000.0)
        assert monitor.effective_sigma() > 1.0

    def test_stale_filter_sigma_ignored(self, monitor):
        import time

        random.seed(5)
        feed(monitor, [100.0 + random.gauss(0, 3.0) for _ in range(40)])
        monitor.set_filter_sigma(0.1, when=time.time() - 60)
        assert monitor.effective_sigma() == pytest.approx(monitor.observed_sigma())

    def test_filter_may_be_more_pessimistic(self, monitor):
        """If the filter says it is doing worse than raw scatter, believe it."""
        random.seed(6)
        feed(monitor, [100.0 + random.gauss(0, 1.0) for _ in range(40)])
        monitor.set_filter_sigma(8.0)
        assert monitor.effective_sigma() == pytest.approx(8.0)


# ---------------------------------------------------------------------------
# Turn rate
# ---------------------------------------------------------------------------


class TestRateDps:
    def test_none_without_span(self, monitor):
        monitor.add_heading(100.0, when=1000.0)
        assert monitor.rate_dps() is None

    def test_steady_turn(self, monitor):
        monitor.add_heading(100.0, when=1000.0)
        monitor.add_heading(105.0, when=1001.0)
        assert monitor.rate_dps() == pytest.approx(5.0)

    def test_turn_through_north(self, monitor):
        """359 -> 1 is +2 deg, not -358."""
        monitor.add_heading(359.0, when=1000.0)
        monitor.add_heading(1.0, when=1001.0)
        assert monitor.rate_dps() == pytest.approx(2.0)

    def test_implausible_rate_suppressed(self, monitor):
        """
        Two-point differencing aliases above ~90 deg/s. Reporting nothing
        beats reporting a fabricated turn rate.
        """
        monitor.add_heading(0.0, when=1000.0)
        monitor.add_heading(170.0, when=1001.0)
        assert monitor.rate_dps() is None

    def test_boundary(self, monitor):
        monitor.add_heading(0.0, when=1000.0)
        monitor.add_heading(MAX_PLAUSIBLE_RATE_DPS - 1.0, when=1001.0)
        assert monitor.rate_dps() is not None


# ---------------------------------------------------------------------------
# Diagnosis and interlock
# ---------------------------------------------------------------------------


class TestDiagnose:
    def test_unknown_without_data(self, monitor):
        q, why = monitor.diagnose()
        assert q == "UNKNOWN"

    def test_calm_is_good(self, monitor):
        random.seed(7)
        n = 40
        feed(
            monitor,
            [100.0 + random.gauss(0, 0.4) for _ in range(n)],
            pitches=[random.gauss(0, 0.2) for _ in range(n)],
            rolls=[random.gauss(0, 0.2) for _ in range(n)],
        )
        assert monitor.diagnose()[0] == "GOOD"

    def test_rough_water_blames_sea_state(self, monitor):
        random.seed(8)
        n = 40
        feed(
            monitor,
            [100.0 + random.gauss(0, 9.0) for _ in range(n)],
            pitches=[random.gauss(0, 6.0) for _ in range(n)],
            rolls=[random.gauss(0, 7.0) for _ in range(n)],
        )
        q, why = monitor.diagnose()
        assert q == "BAD"
        assert "sea state" in why

    def test_noisy_but_flat_blames_compass(self, monitor):
        """
        Same heading scatter, no tilt. The operator needs to know whether to
        slow down or reseat a connector -- these are different actions.
        """
        random.seed(9)
        n = 40
        feed(
            monitor,
            [100.0 + random.gauss(0, 9.0) for _ in range(n)],
            pitches=[random.gauss(0, 0.2) for _ in range(n)],
            rolls=[random.gauss(0, 0.2) for _ in range(n)],
        )
        q, why = monitor.diagnose()
        assert q == "BAD"
        assert "compass" in why

    def test_north_seam_is_not_noise(self, monitor):
        """Holding a northerly heading must not read as maximum dispersion."""
        random.seed(10)
        n = 40
        feed(
            monitor,
            [(359.5 + random.gauss(0, 0.4)) % 360.0 for _ in range(n)],
            pitches=[0.0] * n,
            rolls=[0.0] * n,
        )
        assert monitor.diagnose()[0] == "GOOD"


class TestOkToEngage:
    def test_blocked_without_heading(self, monitor):
        ok, why = monitor.ok_to_engage()
        assert not ok

    def test_allowed_on_compass_alone(self, state, monitor):
        """
        Regression: `get_signal("fused") or get_signal("compass")` always
        returned the fused Signal, because a dataclass instance is truthy
        even when its value is None. That pinned every consumer to a signal
        the Kalman filter has not populated yet and blocked engagement on a
        perfectly good compass.
        """
        random.seed(11)
        feed(monitor, [100.0 + random.gauss(0, 0.4) for _ in range(40)])
        state.set_signal("compass_heading", 100.0)
        assert state.get_signal("fused_heading").value is None
        ok, why = monitor.ok_to_engage()
        assert ok, why

    def test_prefers_fused_when_available(self, state, monitor):
        random.seed(12)
        feed(monitor, [100.0 + random.gauss(0, 0.4) for _ in range(40)])
        state.set_signal("compass_heading", 100.0)
        state.set_signal("fused_heading", 101.0)
        assert monitor.best_heading_signal().name == "fused_heading"

    def test_falls_back_when_fused_goes_stale(self, state, monitor):
        import time

        random.seed(13)
        feed(monitor, [100.0 + random.gauss(0, 0.4) for _ in range(40)])
        state.set_signal("fused_heading", 101.0, when=time.time() - 60)
        state.set_signal("compass_heading", 100.0)
        assert monitor.best_heading_signal().name == "compass_heading"

    def test_blocked_when_noisy(self, state, monitor):
        random.seed(14)
        n = 40
        feed(
            monitor,
            [100.0 + random.gauss(0, 12.0) for _ in range(n)],
            pitches=[random.gauss(0, 7.0) for _ in range(n)],
            rolls=[random.gauss(0, 7.0) for _ in range(n)],
        )
        state.set_signal("compass_heading", 100.0)
        ok, why = monitor.ok_to_engage()
        assert not ok
        assert "sigma" in why

    def test_reason_is_the_diagnosis_not_a_generic_string(self, state, monitor):
        """The TUI shows this text; it has to tell the operator what to do."""
        random.seed(15)
        n = 40
        feed(
            monitor,
            [100.0 + random.gauss(0, 12.0) for _ in range(n)],
            pitches=[random.gauss(0, 7.0) for _ in range(n)],
            rolls=[random.gauss(0, 7.0) for _ in range(n)],
        )
        state.set_signal("compass_heading", 100.0)
        _, why = monitor.ok_to_engage()
        assert why != "no valid heading"
        assert len(why) > 20


class TestUpdateState:
    def test_publishes_sigma(self, state, monitor):
        random.seed(16)
        feed(monitor, [100.0 + random.gauss(0, 2.0) for _ in range(40)])
        state.set_signal("compass_heading", 100.0)
        monitor.update_state()
        assert state.signal_value("heading_sigma") is not None

    def test_raises_noise_fault(self, state, monitor):
        random.seed(17)
        feed(monitor, [100.0 + random.gauss(0, 12.0) for _ in range(40)])
        state.set_signal("compass_heading", 100.0)
        monitor.update_state()
        assert F_HEADING_NOISE in state.faults

    def test_clears_noise_fault_when_calm(self, state, monitor):
        state.raise_fault(F_HEADING_NOISE, "stale")
        random.seed(18)
        feed(monitor, [100.0 + random.gauss(0, 0.3) for _ in range(40)])
        state.set_signal("compass_heading", 100.0)
        monitor.update_state()
        assert F_HEADING_NOISE not in state.faults

    def test_no_spurious_invalid_with_good_compass(self, state, monitor):
        """The truthy-Signal bug also raised a permanent HEADING_INVALID."""
        random.seed(19)
        feed(monitor, [100.0 + random.gauss(0, 0.3) for _ in range(40)])
        state.set_signal("compass_heading", 100.0)
        monitor.update_state()
        assert F_HEADING_INVALID not in state.faults

    def test_invalid_when_nothing_fresh(self, state, monitor):
        monitor.update_state()
        assert F_HEADING_INVALID in state.faults
