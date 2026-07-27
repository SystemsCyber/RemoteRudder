"""
Error paths and boundary conditions.

These cover branches the main suite reaches only on real hardware faults, plus
the intermediate NOISY band in the heading diagnosis, which sits between the
"everything is fine" and "refuse to engage" cases the other tests pin.

Driver-level exceptions are simulated by patching can.interface.Bus. That is
less convincing than a real unplugged adapter, but the alternative is leaving
the handlers untested until the day they fire on the water.
"""

from __future__ import annotations

import random
import time

import can
import pytest

from hmi_canlink import CANLink
from hmi_heading import SIGMA_ENGAGE_MAX, SIGMA_WARN, HeadingMonitor
from hmi_state import F_HEADING_NOISE, F_LINK_DOWN, F_TX_FAIL


# ---------------------------------------------------------------------------
# Bus open failures
# ---------------------------------------------------------------------------


class TestOpenFailureModes:
    """
    Each python-can exception type means something different to the operator,
    so each must produce its own message rather than a generic "CAN error".
    """

    def _patch_bus(self, monkeypatch, exc):
        def boom(*a, **k):
            raise exc

        monkeypatch.setattr(can.interface, "Bus", boom)

    def test_backend_not_implemented(self, state, monkeypatch):
        """Missing driver package, e.g. no PCAN DLL on Windows."""
        self._patch_bus(
            monkeypatch, can.CanInterfaceNotImplementedError("no pcan driver")
        )
        link = CANLink(state, channel="PCAN_USBBUS1", backend="pcan")
        assert link.open() is False
        assert "unavailable" in state.faults[F_LINK_DOWN].detail

    def test_initialization_error(self, state, monkeypatch):
        """Adapter present but refuses the requested configuration."""
        self._patch_bus(monkeypatch, can.CanInitializationError("bad bitrate"))
        link = CANLink(state, channel="vcan0", backend="virtual")
        assert link.open() is False
        assert "init failed" in state.faults[F_LINK_DOWN].detail

    def test_permission_error(self, state, monkeypatch):
        """
        Running without CAP_NET_RAW. Common on a fresh Pi image and very
        confusing if reported as "adapter not plugged in".
        """
        self._patch_bus(monkeypatch, PermissionError(13, "Operation not permitted"))
        link = CANLink(state, channel="vcan0", backend="virtual")
        assert link.open() is False
        assert "cannot open" in state.faults[F_LINK_DOWN].detail

    def test_unexpected_driver_error(self, state, monkeypatch):
        """An exception type we did not anticipate must still be contained."""
        self._patch_bus(monkeypatch, RuntimeError("driver exploded"))
        link = CANLink(state, channel="vcan0", backend="virtual")
        assert link.open() is False
        assert "unexpected" in state.faults[F_LINK_DOWN].detail

    @pytest.mark.parametrize(
        "exc,marker",
        [
            (can.CanInitializationError("x"), "init failed"),
            (PermissionError(13, "denied"), "cannot open"),
            (RuntimeError("x"), "unexpected"),
        ],
    )
    def test_failure_detail_identifies_the_cause(self, state, monkeypatch, exc, marker):
        """
        Each failure mode gets its own wording. A single generic message would
        leave the operator guessing between a missing driver, a permissions
        problem, and a dead adapter -- three different fixes.
        """
        self._patch_bus(monkeypatch, exc)
        link = CANLink(state, channel="can7", backend="virtual")
        assert link.open() is False
        assert marker in state.faults[F_LINK_DOWN].detail


class TestSendFailureModes:
    def test_can_operation_error(self, virtual_link, state):
        """Typical when the controller is bus-off or the TX queue is full."""

        def boom(*a, **k):
            raise can.CanOperationError("tx queue full")

        virtual_link.bus.send = boom
        assert virtual_link.send(can.Message(arbitration_id=1, data=b"\x00")) is False
        assert F_TX_FAIL in state.faults
        assert "queue" in state.faults[F_TX_FAIL].detail

    def test_generic_can_error(self, virtual_link, state):
        def boom(*a, **k):
            raise can.CanError("something else")

        virtual_link.bus.send = boom
        assert virtual_link.send(can.Message(arbitration_id=1, data=b"\x00")) is False
        assert F_TX_FAIL in state.faults

    def test_shutdown_exception_is_swallowed(self, state):
        """close() runs during teardown; it must never raise."""
        link = CANLink(state, channel="vcan_close", backend="virtual")
        link.open()
        bus = link.bus
        original = bus.shutdown

        def boom():
            raise RuntimeError("shutdown failed")

        bus.shutdown = boom
        try:
            link.close()  # must not raise
            assert state.link_up is False
        finally:
            # Put the real method back: BusABC.__del__ calls shutdown() during
            # garbage collection, and a poisoned one surfaces later as an
            # unraisable-exception warning attributed to whatever test happens
            # to trigger the collection.
            bus.shutdown = original
            try:
                original()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# The NOISY middle band
# ---------------------------------------------------------------------------


def feed(monitor, headings, pitches=None, rolls=None, t0=1000.0, dt=0.1):
    for i, h in enumerate(headings):
        t = t0 + i * dt
        monitor.add_heading(h, when=t)
        if pitches is not None:
            monitor.add_pitch(pitches[i], when=t)
        if rolls is not None:
            monitor.add_roll(rolls[i], when=t)


class TestNoisyBand:
    """
    Between SIGMA_WARN and SIGMA_ENGAGE_MAX the heading is degraded but still
    usable. Engagement is allowed with a warning. This band is what the
    operator sees most often in real conditions, so its behaviour matters as
    much as the clean and broken extremes.
    """

    def _sigma_between(self, monitor, target):
        """Feed synthetic data until observed sigma lands near `target`."""
        random.seed(99)
        for scale in [target * f for f in (0.6, 0.8, 1.0, 1.2, 1.5)]:
            monitor._heading.clear()
            feed(monitor, [100.0 + random.gauss(0, scale) for _ in range(40)])
            s = monitor.observed_sigma()
            if s is not None and SIGMA_WARN < s < SIGMA_ENGAGE_MAX:
                return s
        return monitor.observed_sigma()

    def test_noisy_quality_reported(self, state, monitor):
        s = self._sigma_between(monitor, 5.0)
        if not (SIGMA_WARN < (s or 0) < SIGMA_ENGAGE_MAX):
            pytest.skip(f"could not land sigma in the NOISY band (got {s})")
        state.set_signal("compass_heading", 100.0)
        quality, why = monitor.diagnose()
        assert quality == "NOISY"
        assert "sigma" in why

    def test_noisy_still_allows_engage(self, state, monitor):
        """
        Degraded is not the same as unusable. Blocking here would make the
        autopilot refuse to work in ordinary chop.
        """
        s = self._sigma_between(monitor, 5.0)
        if not (SIGMA_WARN < (s or 0) < SIGMA_ENGAGE_MAX):
            pytest.skip(f"could not land sigma in the NOISY band (got {s})")
        state.set_signal("compass_heading", 100.0)
        ok, why = monitor.ok_to_engage()
        assert ok is True

    def test_noisy_with_tilt_names_tilt(self, state, monitor):
        random.seed(100)
        n = 40
        found = False
        for hscale, tscale in ((5.0, 4.0), (4.5, 4.0), (5.5, 5.0)):
            monitor._heading.clear()
            monitor._pitch.clear()
            monitor._roll.clear()
            feed(
                monitor,
                [100.0 + random.gauss(0, hscale) for _ in range(n)],
                pitches=[random.gauss(0, tscale) for _ in range(n)],
                rolls=[random.gauss(0, tscale) for _ in range(n)],
            )
            s = monitor.observed_sigma()
            if s is not None and SIGMA_WARN < s < SIGMA_ENGAGE_MAX:
                found = True
                break
        if not found:
            pytest.skip("could not land in the NOISY band with active tilt")
        quality, why = monitor.diagnose()
        assert quality == "NOISY"
        assert "tilt" in why

    def test_noisy_raises_fault_but_not_invalid(self, state, monitor):
        s = self._sigma_between(monitor, 5.0)
        if not (SIGMA_WARN < (s or 0) < SIGMA_ENGAGE_MAX):
            pytest.skip("could not land in the NOISY band")
        state.set_signal("compass_heading", 100.0)
        monitor.update_state()
        assert F_HEADING_NOISE in state.faults


# ---------------------------------------------------------------------------
# Threshold boundaries
# ---------------------------------------------------------------------------


class TestThresholdBoundaries:
    def test_engage_threshold_is_the_documented_value(self):
        """
        Pinned deliberately. Observed sigma on the real Horsetooth capture runs
        10-15 degrees, well above this, so the current threshold blocks
        engagement on that data. If someone raises it to get a demo working,
        this test makes that a conscious edit rather than a silent one.
        """
        assert SIGMA_ENGAGE_MAX == 6.0
        assert SIGMA_WARN == 4.0
        assert SIGMA_WARN < SIGMA_ENGAGE_MAX

    def test_just_below_engage_max_is_allowed(self, state, monitor):
        monitor.set_filter_sigma(SIGMA_ENGAGE_MAX - 0.1)
        feed(monitor, [100.0] * 40)
        state.set_signal("compass_heading", 100.0)
        ok, _ = monitor.ok_to_engage()
        assert ok is True

    def test_just_above_engage_max_is_blocked(self, state, monitor):
        monitor.set_filter_sigma(SIGMA_ENGAGE_MAX + 0.1)
        feed(monitor, [100.0] * 40)
        state.set_signal("compass_heading", 100.0)
        ok, _ = monitor.ok_to_engage()
        assert ok is False


# ---------------------------------------------------------------------------
# Signal edge cases
# ---------------------------------------------------------------------------


class TestSignalEdges:
    def test_unknown_signal_created_on_demand(self, state):
        """
        set_signal on an unregistered name should not raise -- a decoder that
        grows a field must not need a matching edit here first.
        """
        state.set_signal("brand_new_signal", 42.0)
        assert state.signal_value("brand_new_signal") == pytest.approx(42.0)

    def test_signal_value_default(self, state):
        assert state.signal_value("nonexistent", default=-1) == -1

    def test_get_signal_missing_returns_none(self, state):
        assert state.get_signal("nonexistent") is None

    def test_events_deque_is_bounded(self, state):
        """The event log must not grow without limit on a long run."""
        for i in range(5000):
            state.log_event("TEST", f"event {i}")
        assert len(state.events) <= 200

    def test_recent_events_respects_n(self, state):
        for i in range(50):
            state.log_event("TEST", f"event {i}")
        assert len(state.recent_events(10)) == 10

    def test_recent_events_returns_newest(self, state):
        for i in range(50):
            state.log_event("TEST", f"event {i}")
        assert "event 49" in state.recent_events(1)[0][2]
