"""
Stability tests: the numbers feeding the control loop and the display must not
bounce when the input is steady.

These were missing before, and their absence let two real sea-trial bugs
through:

  * the GPS vehicle-direction (~39 deg) was being written into compass_heading,
    which otherwise held the real wall compass (~180 deg south). The displayed
    heading alternated between the two -- a stdev of ~70 deg. A stability test
    on compass_heading would have caught it the first time it ran.

  * the heading goal bounced between the set value and 360, because two goal
    copies normalized differently (% 360 vs > 360). On the circle 0 and 360 are
    the same point, so a *circular* stdev hides this -- the goal test therefore
    checks the RAW stored value for consistency.

Approach: replay a real capture where the true input is known to be steady
(boat parked facing south), sample each control-loop/display signal at every
tick, and assert its standard deviation is small. Thresholds are set well above
the measured sensor noise (compass ~0.6 deg stdev parked) but far below the
magnitude of a real bounce (tens to hundreds of degrees), so the tests catch
bouncing without being flaky.
"""

from __future__ import annotations

import statistics

import pytest

from conftest import circular_stdev


# Measured on the 2026-07-25 parked-south capture after the compass/vehicle-dir
# fix: compass and fused heading both sit at ~0.6 deg stdev. 3 deg is a
# comfortable ceiling -- 5x the real noise, but a fraction of any real bounce.
STEADY_HEADING_STDEV_MAX = 3.0


class TestHeadingStabilityParked:
    """Boat parked facing south -> heading signals must be rock steady."""

    def test_compass_heading_steady(self, wired, parked_south_log):
        s = wired.feed_sampling(parked_south_log, ["compass_heading"], tick_every=50)
        vals = s["compass_heading"]
        assert len(vals) > 20, "not enough samples to judge stability"
        sd = circular_stdev(vals)
        assert sd < STEADY_HEADING_STDEV_MAX, (
            f"compass_heading stdev {sd:.1f} deg too high for a parked boat -- "
            f"something is bouncing the compass (range {min(vals):.1f}-{max(vals):.1f})"
        )

    def test_fused_heading_steady(self, wired, parked_south_log):
        """The number the control loop steers to must be steady."""
        s = wired.feed_sampling(parked_south_log, ["fused_heading"], tick_every=50)
        vals = s["fused_heading"]
        assert len(vals) > 20
        sd = circular_stdev(vals)
        assert sd < STEADY_HEADING_STDEV_MAX, (
            f"fused_heading stdev {sd:.1f} deg -- the control loop's setpoint "
            f"reference is bouncing (range {min(vals):.1f}-{max(vals):.1f})"
        )

    def test_fused_does_not_switch_sources(self, wired, parked_south_log):
        """
        A steady parked boat should hold ONE heading source, not flip between
        them (source flipping was the root of the original 179->30 'bounce').
        """
        s = wired.feed_sampling(parked_south_log, ["fused_heading"], tick_every=50)
        srcs = [x for x in s["_heading_source"] if x and x != "NONE"]
        assert srcs, "no heading source ever selected"
        from collections import Counter
        counts = Counter(srcs)
        dominant = counts.most_common(1)[0][1]
        # the chosen source should dominate overwhelmingly (allow a few frames
        # of settling at the very start)
        assert dominant / len(srcs) > 0.95, (
            f"heading source is flipping: {dict(counts)} -- the fusion is not "
            f"holding a stable source"
        )

    def test_compass_and_fused_agree(self, wired, parked_south_log):
        """
        Parked with only the wall compass, fused should equal compass. A gap
        means a second value (e.g. a GPS course) is leaking into one of them.
        """
        s = wired.feed_sampling(parked_south_log,
                                ["compass_heading", "fused_heading"])
        comp = statistics.mean(s["compass_heading"])
        fused = statistics.mean(s["fused_heading"])
        gap = abs(((comp - fused + 180) % 360) - 180)
        assert gap < 3.0, (
            f"compass ({comp:.1f}) and fused ({fused:.1f}) disagree by {gap:.1f} "
            f"deg -- a wrong source is feeding one of them"
        )


class TestControlLoopInputStability:
    """The signals the autopilot PID consumes must be stable on stable input."""

    def test_no_signal_exceeds_noise_budget(self, wired, parked_south_log):
        """
        Sweep the heading-related signals the loop/display use and assert none
        is bouncing. This is the general guard: if any future change crosses a
        wire again, one of these will spike.
        """
        signals = ["compass_heading", "fused_heading", "cog"]
        s = wired.feed_sampling(parked_south_log, signals)
        problems = []
        for name in signals:
            vals = s[name]
            if len(vals) < 10:
                continue  # not enough data (e.g. cog absent when parked)
            sd = circular_stdev(vals)
            # cog is genuinely noisy when stationary (no steerage way); only
            # the heading signals must be steady.
            budget = STEADY_HEADING_STDEV_MAX if name != "cog" else 999.0
            if sd >= budget:
                problems.append(f"{name}: stdev {sd:.1f} deg")
        assert not problems, "bouncing signals: " + ", ".join(problems)


class TestGoalSingleState:
    """
    The heading goal must be a single, consistently-normalized value. The
    360-bounce is invisible to a circular metric (0 == 360), so these check the
    RAW stored value.
    """

    def _fresh_autopilot(self):
        from autopilot import Autopilot
        from can_interface import CANinterface
        from app_tui import HMIApp
        ci = CANinterface.__new__(CANinterface)
        HMIApp._init_can_fields(ci)
        ci.bus = None
        return Autopilot(ci)

    def test_goal_never_equals_360(self):
        """360 must always normalize to 0 -- the raw value should never be 360."""
        ap = self._fresh_autopilot()
        for v in (360.0, 720.0, -360.0, 361.0, 359.9 + 0.1):
            ap.set_heading_goal(v)
            assert ap.heading_goal != 360.0, f"goal stored as 360 for input {v}"
            assert 0.0 <= ap.heading_goal < 360.0

    def test_repeated_wrap_is_stable(self):
        """
        Repeatedly nudging the goal across the 0/360 boundary must not make it
        oscillate -- each result is deterministic and in range. This is the
        raw-value stability the circular metric cannot see.
        """
        ap = self._fresh_autopilot()
        ap.set_heading_goal(1.0)
        seq = []
        # step down through zero and back up, many times
        for _ in range(5):
            for _ in range(4):
                ap.adjust_heading_goal(-1)
                seq.append(ap.heading_goal)
            for _ in range(4):
                ap.adjust_heading_goal(+1)
                seq.append(ap.heading_goal)
        # every stored value must be a clean [0,360) bearing, never 360
        assert all(0.0 <= g < 360.0 for g in seq), seq
        # returning to the start each cycle -> the set of visited values is small
        assert len(set(seq)) <= 6, f"goal visiting too many values: {sorted(set(seq))}"

    def test_state_and_autopilot_stay_in_lockstep(self):
        """
        The HMI state goal and the autopilot goal are the 'one variable' the
        operator asked for: they must never diverge, across many operations.
        """
        from hmi_state import SystemState
        st = SystemState()
        ap = self._fresh_autopilot()
        import random
        random.seed(1)
        for _ in range(200):
            g = random.uniform(-720, 720)
            v = st.set_goal(g, "test")
            ap.set_heading_goal(g)
            assert abs(v - ap.heading_goal) < 1e-9, (
                f"diverged on {g:.1f}: state {v:.3f} vs autopilot {ap.heading_goal:.3f}"
            )


class TestDisplayValueStability:
    """
    The values the TUI renders (via the state snapshot) must be as steady as
    the underlying signals -- the display must not introduce its own bounce.
    """

    def test_snapshot_heading_matches_signal(self, wired, parked_south_log):
        wired.feed_sampling(parked_south_log, ["fused_heading"], tick_every=50)
        snap = wired.state.snapshot()
        sig = wired.state.signal_value("fused_heading")
        # the snapshot the TUI/web read must agree with the live signal
        snap_val = snap.get("signals", {}).get("fused_heading")
        if snap_val is not None and sig is not None:
            gap = abs(((snap_val - sig + 180) % 360) - 180)
            assert gap < 0.5, f"snapshot heading {snap_val} != signal {sig}"
