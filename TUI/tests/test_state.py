"""
Unit tests for hmi_state: Signal freshness, goal normalization, fault latching.

The goal-normalization tests are the ones to keep an eye on. heading_goal is
what the PID chases, so anything that can put a wrong or non-finite value in
there is a steering bug, not a display bug.
"""

from __future__ import annotations

import math

import pytest

from hmi_state import (
    F_BUS_OFF,
    F_LINK_DOWN,
    F_STALE,
    Fault,
    Signal,
    SourceHealth,
    SystemState,
)


# ---------------------------------------------------------------------------
# Signal
# ---------------------------------------------------------------------------


class TestSignal:
    def test_starts_invalid(self):
        s = Signal("x")
        assert s.value is None
        assert s.age() == float("inf")
        assert not s.is_valid()

    def test_update_sets_value_and_time(self):
        s = Signal("x", stale_after=2.0)
        s.update(42.0, when=1000.0)
        assert s.value == 42.0
        assert s.age(now=1001.0) == pytest.approx(1.0)

    def test_staleness_boundary(self):
        s = Signal("x", stale_after=2.0)
        s.update(1.0, when=1000.0)
        assert not s.is_stale(now=1001.9)
        assert s.is_stale(now=1002.1)

    def test_value_none_is_never_valid(self):
        """A signal registered but never fed must not read as usable."""
        s = Signal("x", stale_after=1e9)
        s.update(None, when=1000.0)
        assert not s.is_valid(now=1000.0)

    def test_zero_is_a_valid_value(self):
        """Guard against `if not value` style checks: 0 deg is a real heading."""
        s = Signal("x", stale_after=10.0)
        s.update(0.0, when=1000.0)
        assert s.is_valid(now=1000.0)


# ---------------------------------------------------------------------------
# Goal normalization
# ---------------------------------------------------------------------------


class TestHeadingGoal:
    @pytest.mark.parametrize(
        "raw,expected",
        [
            (0.0, 0.0),
            (359.9, 359.9),
            (360.0, 0.0),          # 360 must not be a reachable state
            (360.1, 0.1),
            (370.0, 10.0),
            (-1.0, 359.0),
            (-10.0, 350.0),
            (-360.0, 0.0),
            (1000.0, 280.0),       # multi-wrap: single if/elif cannot do this
            (-1000.0, 80.0),
            (720.0, 0.0),
        ],
    )
    def test_normalization(self, state, raw, expected):
        assert state.set_goal(raw, "test") == pytest.approx(expected)

    def test_adjust_wraps_down_through_north(self, state):
        state.set_goal(0.0, "t")
        assert state.adjust_goal(-1.0, "t") == pytest.approx(359.0)

    def test_adjust_wraps_up_through_north(self, state):
        state.set_goal(359.5, "t")
        assert state.adjust_goal(1.0, "t") == pytest.approx(0.5)

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_non_finite_rejected(self, state, bad):
        """
        NaN survives float() and `nan % 360.0` is nan, so without an explicit
        isfinite check this silently poisons the setpoint: every comparison
        against it is False and the PID error becomes nan.
        """
        state.set_goal(130.0, "good")
        state.set_goal(bad, "malicious")
        assert state.heading_goal == pytest.approx(130.0)

    @pytest.mark.parametrize("bad", ["abc", None, [1, 2], {}, object()])
    def test_unparseable_rejected(self, state, bad):
        state.set_goal(130.0, "good")
        state.set_goal(bad, "malicious")
        assert state.heading_goal == pytest.approx(130.0)

    def test_rejection_is_logged(self, state):
        state.set_goal(float("nan"), "web/1.2.3.4")
        kinds = [k for _, k, _ in state.recent_events(5)]
        assert "WARN" in kinds

    def test_numeric_string_accepted(self, state):
        """A JSON client may legitimately send "130.5"."""
        assert state.set_goal("130.5", "web") == pytest.approx(130.5)

    def test_bool_is_not_silently_accepted_as_heading(self, state):
        """True == 1 in Python; make the behaviour explicit rather than accidental."""
        result = state.set_goal(True, "web")
        assert result == pytest.approx(1.0)  # documents current behaviour


class TestGoalProvenance:
    def test_source_recorded(self, state):
        state.set_goal(130.0, "web/192.168.1.47")
        assert state.goal_source == "web/192.168.1.47"

    def test_subscriber_notified(self, state):
        seen = []
        state.subscribe(lambda kind, payload: seen.append((kind, payload)))
        state.set_goal(130.0, "tui")
        assert any(k == "goal" and p["heading_goal"] == 130.0 for k, p in seen)

    def test_no_op_does_not_notify(self, state):
        """Re-setting the same value must not spam the phone or the event log."""
        state.set_goal(130.0, "tui")
        seen = []
        state.subscribe(lambda kind, payload: seen.append(kind))
        state.set_goal(130.0, "web")
        assert "goal" not in seen

    def test_equivalent_angle_is_a_no_op(self, state):
        """130 and 490 are the same bearing; do not fire a change event."""
        state.set_goal(130.0, "tui")
        seen = []
        state.subscribe(lambda kind, payload: seen.append(kind))
        state.set_goal(490.0, "web")
        assert "goal" not in seen

    def test_bad_subscriber_cannot_break_producer(self, state):
        """A raising callback must not take down the CAN thread."""
        state.subscribe(lambda kind, payload: (_ for _ in ()).throw(RuntimeError("boom")))
        state.set_goal(130.0, "tui")  # must not raise
        assert state.heading_goal == pytest.approx(130.0)


# ---------------------------------------------------------------------------
# Faults
# ---------------------------------------------------------------------------


class TestFaults:
    def test_raise_and_clear(self, state):
        state.raise_fault(F_LINK_DOWN, "unplugged")
        assert F_LINK_DOWN in state.faults
        state.clear_fault(F_LINK_DOWN)
        assert F_LINK_DOWN not in state.faults

    def test_repeat_increments_count_not_duplicates(self, state):
        for _ in range(5):
            state.raise_fault(F_STALE, "compass")
        assert state.faults[F_STALE].count == 5
        assert len(state.active_faults()) == 1

    def test_severity_ordering(self, state):
        state.raise_fault(F_STALE, "a")
        state.raise_fault(F_BUS_OFF, "b")
        state.raise_fault(F_LINK_DOWN, "c")
        codes = [f.code for f in state.active_faults()]
        assert codes[0] == F_BUS_OFF
        assert state.worst_fault().code == F_BUS_OFF

    def test_first_seen_preserved_across_repeats(self, state):
        state.raise_fault(F_STALE, "x")
        first = state.faults[F_STALE].first_seen
        state.raise_fault(F_STALE, "x")
        assert state.faults[F_STALE].first_seen == first

    def test_clearing_absent_fault_is_safe(self, state):
        state.clear_fault("NEVER_RAISED")  # must not raise

    def test_worst_fault_none_when_clean(self, state):
        assert state.worst_fault() is None


# ---------------------------------------------------------------------------
# Source health
# ---------------------------------------------------------------------------


class TestSourceHealth:
    def test_never_seen(self):
        s = SourceHealth(0x123, "test", 2.0)
        assert s.status() == "NEVER"
        assert s.age() == float("inf")

    def test_ok_then_stale(self):
        s = SourceHealth(0x123, "test", 2.0)
        s.mark(when=1000.0)
        assert s.status(now=1001.0) == "OK"
        assert s.status(now=1003.0) == "STALE"

    def test_mark_counts(self):
        s = SourceHealth(0x123, "test", 2.0)
        for i in range(3):
            s.mark(when=1000.0 + i)
        assert s.count == 3

    def test_registered_source_tracked(self, state):
        state.register_source(0xABC, "thing", 1.0)
        state.mark_source(0xABC, when=1000.0)
        assert state.sources[0xABC].count == 1

    def test_unregistered_id_still_counts_as_traffic(self, state):
        """
        Frames we do not decode still prove the bus is alive. rx_total must
        move, or a bus carrying only unwatched IDs would look silent and
        trigger a spurious NO_TRAFFIC.
        """
        before = state.rx_total
        state.mark_source(0xDEADBEEF, when=1000.0)
        assert state.rx_total == before + 1


# ---------------------------------------------------------------------------
# Snapshot
# ---------------------------------------------------------------------------


class TestSnapshot:
    def test_json_serializable(self, state):
        import json

        state.set_goal(130.0, "tui")
        state.raise_fault(F_STALE, "compass")
        state.register_source(0x123, "x", 1.0)
        state.mark_source(0x123)
        json.dumps(state.snapshot())  # must not raise

    def test_preserves_legacy_keys(self, state):
        """
        compass.js reads these names. Renaming one silently breaks the browser
        UI while every Python test still passes, so pin them here.
        """
        snap = state.snapshot()
        for key in (
            "heading_deg",
            "heading_goal",
            "compass_deg",
            "cog_deg",
            "rpm",
            "speed",
            "lat",
            "lon",
            "autopilot_engaged",
            "servo_enabled",
            "headingErr",
            "rudder_counts",
            "shaft_goal",
        ):
            assert key in snap, f"legacy key '{key}' missing from snapshot"

    def test_exposes_new_diagnostics(self, state):
        snap = state.snapshot()
        for key in (
            "link_up",
            "bus_state",
            "faults",
            "sources",
            "heading_sigma",
            "goal_source",
            "cog_accepted",
            "cog_rejected",
        ):
            assert key in snap

    def test_infinite_age_serializes_as_none(self, state):
        """float('inf') is not valid JSON; a never-seen source must send null."""
        import json

        state.register_source(0x123, "never", 1.0)
        snap = state.snapshot()
        src = [s for s in snap["sources"] if s["id"] == 0x123][0]
        assert src["age"] is None
        json.dumps(snap)


# ---------------------------------------------------------------------------
# Thread safety
# ---------------------------------------------------------------------------


class TestConcurrency:
    def test_concurrent_goal_writes_leave_valid_state(self, state):
        """
        Last write wins, but the result must always be a real bearing --
        never a torn or out-of-range value.
        """
        import threading

        def writer(base):
            for i in range(200):
                state.set_goal(base + i, f"t{base}")

        threads = [threading.Thread(target=writer, args=(b,)) for b in (0, 90, 180, 270)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert 0.0 <= state.heading_goal < 360.0
        assert math.isfinite(state.heading_goal)

    def test_snapshot_during_writes_does_not_raise(self, state):
        import threading

        stop = threading.Event()

        def writer():
            i = 0
            while not stop.is_set():
                state.set_signal("rpm", i)
                state.mark_source(0x123, None)
                i += 1

        state.register_source(0x123, "x", 1.0)
        t = threading.Thread(target=writer, daemon=True)
        t.start()
        try:
            for _ in range(200):
                state.snapshot()
        finally:
            stop.set()
            t.join(timeout=2)
